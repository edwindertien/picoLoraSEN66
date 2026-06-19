# Development context / handoff notes

This document exists so the next debugging session (human or AI) doesn't
repeat the same dead ends. Read this before changing `lora_wan.cpp`,
`main.cpp`'s core split, or `sensor.cpp`'s init sequence.

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

## Outstanding / not yet built

- LoRaWAN OTAA join + Cayenne LPP uplinks (Phase B) — config fields exist,
  implementation does not yet.
- ChirpStack gateway setup, MQTT integration, Node-RED/Grafana consumption —
  not started.
- Proper duty-cycle calculation from SF/BW/payload size (currently a flat
  30s constant).
- The `lora test` vs `lora send` gateway-arrival discrepancy noted above —
  inconclusively resolved, revisit if it recurs.