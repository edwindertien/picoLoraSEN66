# Development context / handoff notes

This document exists so the next debugging session (human or AI) doesn't
repeat the same dead ends. Read this before changing `lora_wan.cpp`,
`main.cpp`'s core split, `sensor.cpp`'s init sequence, `lcd_display.cpp`/
`lcd_driver.h` (see "The LCD/SPI1 sharing saga" — by far the longest and
most expensive section here), or `wifi_mgr.cpp`'s reconnect behaviour (see
"Network").

## Project phases, in order

1. SEN66 over I2C → serial CSV (Phase 1) — straightforward, worked first try
   once the Sensirion library's `NO_ERROR` macro and integer-vs-float
   signature issues were sorted out.
2. Python live visualiser (`sen66_monitor.py`) — matplotlib, mirrors the
   serial CSV format so it works unchanged against later firmware phases.
3. Web dashboard, AP + STA WiFi, LittleFS config/log, tabbed graph UI
   (Phase 2) — the bulk of the working single-core baseline.
4. Dual-core LoRa integration — this took many iterations to get right; see
   below.
5. Waveshare Pico-LCD-1.14 menu (sensor/WiFi/LoRa status pages, per-channel
   graphs, Button B navigation) — took *even more* iterations, almost all
   of it about sharing SPI1 with the already-working LoRa radio; see "The
   LCD/SPI1 sharing saga" below. This is the single longest debugging arc
   in the project's history — read that section before touching
   `lcd_display.cpp`, `lcd_driver.h`, or the radio suspend/resume API in
   `lora_wan.cpp`.

## The dual-core LoRa saga — what went wrong and why

The single biggest source of wasted time was **CRC errors on SEN66 reads**
that appeared only after LoRa/dual-core code was added, despite the SEN66
sitting on a completely separate bus (I2C0, GP4/GP5) from the LoRa radio
(SPI1, GP10/11/12). Several false theories were chased before finding the
real cause:

### False theory 1: electrical noise from the LoRa shield
Suspected the stacked shield was coupling noise onto the I2C lines, or that
the SX1262 PA current spike was drooping the 3.3V rail enough to glitch I2C.
**Ruled out** — the user confirmed no hardware had changed between a working
single-core build and a failing dual-core build with identical wiring.

### False theory 2: GPIO interrupt collision
GP20 (DIO1, the SX1262 IRQ pin) is also a valid I2C0 SDA alternate function
on the RP2040. Theorised that RadioLib's `attachInterrupt()` on GP20 was
corrupting I2C0 transactions on GP4 via shared interrupt controller state.
Tried `radio.clearDio1Action()` to force polling instead of interrupts.
**Did not fix it** — the CRC errors persisted even with the interrupt
cleared, so this was at most a minor contributor, not the root cause.

### False theory 3: core 1 spinning too fast
Tried `__wfi()`, `sleep_ms()`, and various busy-wait patterns in `loop1()`,
theorising that core 1 polling too aggressively was starving core 0 of bus
cycles. **Did not fix it.**

### Actual root cause: enlarging `Config` and including `config.h` in `sensor.cpp`
The real cause was much simpler and had nothing to do with cores or
interrupts at all: **`sensor.cpp` had gained an `#include "config.h"`**
during a refactor that added LoRa fields to the `Config` struct. Enlarging
`Config` changed stack/memory layout in a way that corrupted something
adjacent — likely the ArduinoJson `JsonDocument` or the SEN66 I2C read
buffer — when both were compiled in the same translation unit as the now-
larger struct.

**The fix:** `sensor.cpp` must never include `config.h`. Anything `sensor.cpp`
needs from config (e.g. whether to run fan cleaning) is passed in as a plain
function parameter from `main.cpp`, which does have `config.h`:

```cpp
// sensor.h
void initSensor(bool doFanCleaning = true);

// main.cpp
initSensor(cfg.fan_cleaning);
```

This was confirmed by diffing against a known-good single-core project
(`picoSEN66_3`) file-by-file and restoring files verbatim, then re-adding
LoRa support field-by-field while explicitly checking `sensor.cpp` for stray
includes after every change.

**Lesson for future work:** if CRC/I2C errors reappear after adding fields
to `Config`, check `sensor.cpp`'s includes before suspecting hardware,
interrupts, or core timing. It is tempting to blame dual-core because the
symptom only appeared after dual-core was introduced, but the actual trigger
was a coincident refactor, not the cores themselves.

### SPI1 init pattern that actually works

Earle Philhower's `SPIClassRP2040` (not `arduino::MbedSPI` — that class
doesn't exist in this core, despite appearing in some other projects'
examples found online) needs pins set individually before `begin()`:

```cpp
SPI1.setRX(12);     // MISO
SPI1.setTX(11);     // MOSI
SPI1.setSCK(10);    // SCK
SPI1.begin(false);  // false = software/manual CS; RadioLib drives CS (GP3)
                     // directly as a GPIO, so setCS() is never called
```

Calling `setCS()` was tried and is unnecessary — RadioLib's `Module`
constructor takes the CS pin directly and manages it itself.

**Ownership rule that was eventually settled on:** all SPI1 setup
(`setRX/setTX/setSCK/begin`) and `radio.begin()` happen together on **core
1**, inside `setup1()`, gated by `rp2040.fifo.pop()` so core 1 doesn't touch
SPI1 until core 0 has explicitly signalled (via `rp2040.fifo.push()`) that
its own setup — including the SEN66 fan-clean delay — is fully complete.
Earlier attempts split SPI1 init across both cores or ran it before core 0
finished; both caused hangs or the CRC corruption above.

### `radio.begin()` parameter gotchas

- Calling `radio.begin()` with explicit TCXO voltage (`tcxoVoltage` param)
  caused indefinite hangs on this Waveshare module — it sends a
  `SetDIO3AsTCXOCtrl` command and waits for oscillator lock that never
  completes on this hardware. **Fix:** call `radio.begin()` with no
  arguments (defaults), then explicitly call `setFrequency`, `setBandwidth`,
  `setSpreadingFactor`, `setCodingRate`, `setSyncWord`, `setOutputPower`
  afterward.
- `radio.setDio2AsRfSwitch(true)` is required for this module's RF switch
  wiring.
- RadioLib's `LoRaWANNode::beginOTAA()` signature changed between major
  versions — v6.x takes `uint64_t joinEUI, uint64_t devEUI, uint8_t* nwkKey,
  uint8_t* appKey` and returns `void` (not `int16_t`). EUIs must be parsed
  from hex strings to `uint64_t`, not byte arrays, for this call.

## Duty cycle / interval bug

`lora_interval_s` in config and a separate hardcoded `MIN_INTERVAL_MS`
(originally 30000) were both gating LoRa stream timing, and at one point a
manual `lora send` CLI command was *also* gated by the same 30s floor,
making manual testing feel like the firmware was "hanging" for half a minute
on every test send. Resolved by keeping the 30s floor as a safety constant
for **automatic stream mode only** — manual `lora send`/`lora test` should
remain ungated for responsive testing, *however* the user's preference,
confirmed in conversation, was to keep `lora send` gated too (so the
behaviour matches stream mode exactly) once duty cycle and config interval
mismatches were untangled. Current behaviour: `lora send` respects
`dutyOk()`; only `lora test` (explicit one-shot with arbitrary frequency)
bypasses it.

## `lora test` vs `lora send` payload difference

Early in testing, `lora test <freq> <quoted-json>` worked (verified on a
spectrum analyser) while `lora send` did not appear to arrive at the
gateway, despite both reporting `TX OK` at the radio level. Investigation
showed:
- Frequency, SF, BW, and power were identical for both paths once `cfg.lora_freq`
  was set to match the test frequency permanently (rather than being
  temporarily overridden and restored by `lora test`'s save/restore logic).
- The actual unresolved variable when this was last discussed was the
  **30-second duty-cycle wait** being mistaken for a hang — `lora send`
  appeared to "block" because the CLI was reporting `[lora] Duty cycle: wait
  Xs` and the user was watching it count down, not because of a code defect.
  **Status: revisit this if it recurs** — the immediate suspects (frequency
  drift, payload encoding, quoting artefacts) were all ruled out, but a full
  root-cause was not conclusively nailed down before the session moved on to
  other features. If `lora send` silently fails to reach the gateway again,
  capture the gateway-side raw packet log (frequency, SF, payload bytes) for
  a TX that the firmware reports as successful, and compare byte-for-byte
  against a known-good `lora test` packet.

## SEN66 fan cleaning / sensor warm-up

- CO2 (SCD4x inside the SEN66) needs **roughly 30 seconds** from cold start
  to produce a non-sentinel reading — this is sensor physics, not a
  firmware bug. Don't chase "CO2 reads 0/0xFFFF" issues for the first
  30-60s after boot.
- Fan cleaning (`startFanCleaning()`, ~11s) was originally unconditional;
  made it a config-controlled boolean (`fan_cleaning`, default `true`) so it
  can be disabled during rapid iterate-flash-test debugging cycles without
  losing 11 seconds every boot. **Always re-enable for real deployment** —
  without it, PM optics accumulate debris over time.
- One debugging session found the fan was *not spinning* despite
  `startFanCleaning()` reporting no I2C error. Root cause was that
  `startContinuousMeasurement()`'s return value wasn't being checked/logged
  loudly enough to notice it had failed; once explicit
  `Serial.println("[sen66] startContinuousMeasurement OK")` /
  `"FATAL: measurement not started"` logging was added, the failure became
  visible immediately. Always check this log line first if gas-sensor
  channels read zero indefinitely past the 30-60s warm-up window.
- An I2C bus scan (addresses 1-127) was added to `initSensor()` for the same
  reason — confirms the SEN66 is physically present at `0x6B` before doing
  anything else, removing wiring/address as a variable early.

## Files restored from a known-good snapshot

At one point the project was reset to a known-working baseline
(`picoSEN66_3`, single-core, no LoRa) after too many compounding theories
made the dual-core+LoRa debugging session unproductive. All `src/*.cpp` and
`include/*.h` files were diffed and copied verbatim from that baseline
before LoRa was re-added incrementally, one verified step at a time:
1. Dual-core skeleton, core 1 idle (`sleep_ms` only) — confirmed sensor
   still worked.
2. SPI1 pin setup + `begin()` on core 1 — confirmed no hang.
3. `radio.begin()` added — confirmed `SX1262 OK`.
4. CLI-triggered single test TX (`lora test`) — confirmed over spectrum
   analyser.
5. Full plain-LoRa implementation with ACK window, stream mode, settings
   commands.

If dual-core/LoRa breaks again in a way that resists debugging, this
incremental rebuild path (idle core → SPI → radio.begin → manual TX →
full feature) is the fastest way back to a working state — faster than
trying to debug forward from a broken full-featured state.

## The LCD/SPI1 sharing saga

Adding a Waveshare Pico-LCD-1.14 (for a menu of sensor/WiFi/LoRa status
pages plus per-channel graphs) turned into by far the longest debugging
arc in this project — much longer than the dual-core CRC saga above. This
section exists specifically so nobody re-tries the things that were
already tried and ruled out.

### The core physical constraint

The LCD and the SX1262 both sit on SPI1. Sharing SCK/MOSI with separate CS
pins is completely normal and was never the issue. The actual problem is
that **the LCD's RST pin and the SX1262's MISO pin are the same physical
wire (GP12)**, hardwired on both shields with no jumper to separate them.
RST needs to be a Pico output held HIGH; MISO needs to be the SX1262's own
output, which necessarily toggles as part of any normal SPI transaction.
These two roles are fundamentally incompatible on one wire at the same
time — this was eventually understood to be an *electrical* constraint,
not something any amount of clever software timing could paper over.

### Symptom, for a long stretch: "radio works, LCD shows nothing" or vice versa

For many iterations, exactly one of the two would work at a time, and
fixing one reliably broke the other. Everything below was tried, in
roughly this order, while chasing that symptom:

1. **Mutex timing** (`mutex_init()` too late, blocking vs
   `mutex_try_enter`) — no effect on the underlying problem. A flush-
   success/skip counter was added at one point and showed **thousands of
   successful tile flushes with zero visible content** — proof that "the
   function call completed" and "the display actually updated" are not
   the same thing, and that mutex contention was never the bottleneck.
2. **SPI clock speed not being restored** between the LCD's 62.5MHz and
   RadioLib's 2MHz — tried manual `beginTransaction`/`endTransaction`
   restores, then a `radio.setSPISettings()` call that doesn't exist on
   `SX1262` (compile error, reverted), then a manual `spi1_radio_speed()`
   helper. No effect.
3. **GP12 pin-restore mechanism** — tried `SPI1.setRX()` + a second
   `begin()`, then the raw SDK `gpio_set_function()` call instead. Radio
   reliability tracked with *whether the radio ran at all*, not with which
   restore mechanism was used — this was a real clue that got
   under-weighted at the time.
4. **Double `SPI1.begin()` from both cores** — a real and worth-
   remembering finding: SPI1 is **one physical peripheral**, not
   duplicated per core, and having both core 0 (LCD) and core 1 (radio)
   independently call `begin()` does re-initialise the shared block out
   from under whichever side didn't call it last. Removing the second
   `begin()` (radio trusting the LCD's earlier init) fixed radio
   reliability — but the LCD *still* showed nothing afterward, which in
   hindsight proved this was never sufficient on its own.
5. **Full claim/release around every single operation**
   (`SPI1.end()`→reconfigure→`begin()`→work→`end()`, both sides,
   symmetric) — implemented thoroughly, including a genuine RP2040
   `reset_block(RESETS_RESET_SPI1_BITS)`/`unreset_block_wait(...)`
   hardware peripheral reset (not just the `SPI1.end()`/`begin()` Arduino
   wrapper, which does not guarantee every internal register returns to
   power-on defaults). Radio kept working; LCD still showed nothing.
6. **Defensively deasserting the other device's CS** before every claim
   (force GP3 HIGH before an LCD transfer, GP9 HIGH before a radio
   transfer) — on the theory that a stuck-low CS was letting the SX1262
   "listen in" on LCD traffic and corrupt it. No change.
7. **RadioLib's DIO1 interrupt** — `radio.clearDio1Action()` had been
   added, then lost, then re-added; a genuinely different mechanism from
   everything else (an ISR firing outside the mutex model entirely), but
   also made no difference to the LCD-blank symptom specifically.

### The reset that actually mattered: going back to a known-good baseline

After enough failed fixes, the productive move was to **stop iterating on
the broken integrated system** and rebuild forward from a state that was
provably correct:

1. Restored `lcd_driver.h`'s `lcd_init()` to the exact version that had
   been confirmed rendering correctly early on — the LCD calling its
   **own** `SPI1.begin()` (no arguments — hardware CS mode), with a real
   hardware RST pulse on GP12. This intentionally reintroduces the GP12
   conflict, on the theory that the *sharing model*, not this specific
   pin, was the real problem.
2. Rebuilt `lcd_display.cpp` from scratch as a deliberately minimal,
   single-core-only baseline: no radio code compiled in at all (`main.cpp`
   with an empty `setup1()`/`loop1()`, `lora_wan.cpp` moved out of `src/`
   entirely so it isn't even linked). Confirmed the LCD worked reliably,
   permanently on, through many pages and power cycles, with radio fully
   out of the picture.
3. Added the screen timeout and OFF-page cycling *before* touching the
   radio again, confirming the LCD's own release-to-`INPUT` and full-
   `lcd_init()`-on-wake cycle was solid in isolation — still zero radio
   involvement.
4. Only then reintroduced LoRa, with a **coarse-grained bus handoff**
   instead of any of the fine-grained per-transaction sharing tried
   before: the LCD and radio never touch SPI1 concurrently, not even
   briefly. Whoever isn't currently "in control" doesn't merely yield the
   bus for a moment — it's either fully active or fully torn down
   (`SPI1.end()` + every pin to `INPUT` on the LCD side; `radio.sleep()`
   + release on the radio side), with a simple request/acknowledge flag
   pair (`lora_request_suspend()`/`lora_is_suspended()`/
   `lora_request_resume()`) governing the handoff. This is the
   architecture described in the README and is what actually works.

**The lesson, stated plainly:** fine-grained interleaving of two devices
on a bus that share a physical pin with genuinely incompatible signal
roles cannot be made to work by better mutex discipline, faster resets, or
more careful timing — no matter how many different mechanisms you try. It
needs coarse-grained mutual exclusion: one owner at a time, full teardown
on handoff, never both "on" simultaneously even for a moment.

### Two compounding bugs that wasted significant time mid-saga

- **A file mix-up**: at one point `src/lcd_display.cpp` on the user's
  actual disk had accidentally ended up containing `main.cpp`'s content
  (duplicate `setup()`/`loop()`, global variables, linker errors like
  "multiple definition of `staMode`"). This produced confusing symptoms
  that looked like a code regression but were actually just the wrong
  file being compiled. **Lesson**: when behaviour doesn't match what the
  source clearly says it should do, check for a stale build or a
  misplaced file *before* forming a new theory about the code's logic.
- **A leftover diagnostic flag**: `LORA_FULLY_DISABLED` (added
  temporarily to isolate whether LoRa activity was blocking LCD
  rendering) was left defined for several turns after it had served its
  purpose, causing `lora_is_suspended()` to unconditionally return `true`
  and completely masking the real suspend/resume logic — producing
  exactly the "LCD always active, radio always suspended" symptom, with
  no relation to the actual bug being chased at the time. **Lesson**:
  temporary isolation flags need to be actively removed, not just left
  commented-out-by-default, and their effect should be the first thing
  checked when a symptom looks suspiciously absolute ("always", "never").

### Smaller fixes bundled into the same rebuild

- **Button B polling missed short presses.** A plain `digitalRead()` once
  per `loop()` iteration can miss a tap that starts and ends between two
  loop iterations, if `loop()` happens to be briefly busy (a web request,
  a page render's SPI transfer). Switched to `attachInterrupt()` with a
  50ms debounce inside the ISR itself — the press is latched the instant
  it happens, independent of `loop()` timing.
- **The OFF page was unreachable.** The page-cycle wrap logic checked "did
  we overflow past `PAGE_COUNT`, if so reset to `SENSOR`" *before*
  checking "did we land exactly on `OFF`" — so the overflow check always
  fired first and the OFF-check could never be reached. Fixed with a
  plain modulo wrap (`OFF` is enum value `0`, so `(current + 1) %
  PAGE_COUNT` lands there correctly on its own).
- **Default boot state changed** to screen-off/backlight-off/LoRa-active
  — the device is meant to operate as a low-power LoRa node most of the
  time, with the screen only for occasional interactive checks. The LCD
  hardware isn't even touched at boot now (no `lcd_init()` call at all
  until the first button press) specifically so the radio gets a
  completely untouched SPI1 peripheral as its first-ever user.

## Network: `_apMode` and why `net reconnect` must reboot

Adding `net ssid`/`net pass`/`net reconnect` CLI commands surfaced a
genuine arduino-pico core bug, not an application-level mistake:
[earlephilhower/arduino-pico#762](https://github.com/earlephilhower/arduino-pico/issues/762).
`WiFiClass::status()` and `WiFiClass::localIP()` both decide which
physical interface (STA vs AP) to report on via an internal `_apMode`
boolean, which is not reliably cleared by calling `WiFi.mode()` at
runtime once AP mode has been used. Symptom observed directly: after a
STA-failed→AP-fallback at boot, calling `net reconnect` (which re-ran
`startWiFi()` live) produced:
```
[wifi] Trying STA: voorhuis
[wifi] Connected! IP: 192.168.42.1
```
— note zero connection-attempt dots (a real STA handshake takes at least
a few hundred ms; here `WL_CONNECTED` was reported instantly) and an IP
identical to the AP's own address, not a real DHCP lease from the router.
Things tried that did **not** fix it: `WiFi.disconnect(true)` before
retrying (helped nothing since the flag itself wasn't touched),
`WiFi.softAPdisconnect(true)` + `WiFi.mode(WIFI_OFF)` + a settling delay
before retrying STA (still no change — `_apMode` apparently isn't cleared
by any combination of these calls). **The only fix that actually works**:
`net reconnect` now calls `rp2040.reboot()` outright. A full reboot
reinitialises the WiFi driver and its `_apMode` flag from a genuinely
clean state; nothing short of that was found to be reliable. Don't spend
time trying to find a live-reinit workaround again without checking
whether upstream has fixed the underlying issue first.

## Outstanding / not yet built

- LoRaWAN OTAA join + Cayenne LPP uplinks (Phase B) — config fields exist,
  implementation does not yet.
- ChirpStack gateway setup, MQTT integration, Node-RED/Grafana consumption —
  not started.
- Proper duty-cycle calculation from SF/BW/payload size (currently a flat
  30s constant).
- The `lora test` vs `lora send` gateway-arrival discrepancy noted above —
  inconclusively resolved, revisit if it recurs.
- Joystick/A-button support while the screen is on: now that the radio is
  fully suspended (and its pins inert) whenever the LCD is active, GP2/GP3/
  GP15/GP20 are technically free for the joystick during that window —
  this was identified as a natural follow-on but not yet implemented. Would
  need those pins reconfigured to `INPUT_PULLUP` on wake and handed back to
  the radio's expected states on sleep, mirroring what's already done for
  the LCD's own pins.
- LCD/radio genuinely simultaneous operation is not supported by design
  (see "LCD display and SPI1 sharing" in the README) — don't attempt to
  relax this without first re-reading the saga above.