#pragma once
#include <Arduino.h>

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