#pragma once
#include <Arduino.h>

enum class LoRaMode {
    DISABLED,
    LORAWAN,
    RAW
};

enum class LoRaState {
    DISABLED,
    JOINING,
    JOINED,
    SEND_OK,
    SEND_FAIL,
    ERROR
};

extern LoRaMode  loraMode;
extern LoRaState loraState;
extern uint32_t  loraUplinkCount;
extern int16_t   loraLastRssi;
extern float     loraLastSnr;
extern String    loraLastRx;

void        initLoraSPI();   // call from core 0 before WiFi
void        initLoRa();       // call from core 1
void        loopLoRa();
void        loraSendNow();
const char* loraStateStr();
const char* loraModeStr();

// Returns last n log lines as a JSON array string for the web UI
String loraGetLog(uint8_t n = 10);