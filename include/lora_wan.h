#pragma once
#include <Arduino.h>

// TEMPORARY diagnostic flag — when defined, LoRa is a complete no-op:
// initLoRa()/loopLoRa() do nothing, core 1 never touches SPI1 at all, and
// lora_is_suspended() always returns true immediately. This lets
// screen_on_page()/screen_off() run their EXACT production code path
// (claim, hardware reset, lcd_init(), use, release) with radio fully out
// of the picture — isolating whether the LCD's own release/reclaim cycle
// works at all, independent of any radio interference.
 #define LORA_FULLY_DISABLED

// ── LoRa state ────────────────────────────────────────────────────────────
enum class LoRaState {
    DISABLED,
    IDLE,
    TX_OK,
    TX_FAIL,
    ERROR
};

extern LoRaState loraState;
extern uint32_t  loraTxCount;
extern int16_t   loraLastRssi;   // from last ACK
extern float     loraLastSnr;    // from last ACK
extern String    loraLastAck;    // last received ACK string
extern bool      loraStreaming;  // stream mode on/off
extern uint32_t  loraLastTxMs;   // millis() at last TX — for countdown

// Call from core 1 setup1() after SPI1 is ready
void initLoRa();

// Call from core 1 loop1()
void loopLoRa();

// CLI-triggered actions (set flag, consumed by core 1)
void loraSendSensor();              // send one sensor JSON packet
void loraSendTest(float freq, const String& msg);  // one-shot test TX
void loraSetFreq(float mhz);
void loraSetSF(uint8_t sf);
void loraSetPower(int8_t dbm);

const char* loraStateStr();

// ── Coarse-grained SPI1 bus handoff ───────────────────────────────────────
// Instead of fine-grained per-transaction sharing (which failed under
// every mechanism tried — mutex, claim/release, CS-deassertion, interrupt
// clearing), the radio and LCD now take turns owning SPI1 entirely.
// Call lora_request_suspend() when the LCD wants exclusive access, then
// poll lora_is_suspended() until true before touching SPI1 from core 0.
// Call lora_request_resume() when done; the radio re-initialises itself
// fully on its next loopLoRa() call — no manual re-init needed.
void lora_request_suspend();
void lora_request_resume();
bool lora_is_suspended();