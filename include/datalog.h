#pragma once
#include "sensor.h"
#include <Arduino.h>

void initLog();
void appendLog(const Measurement& m);

// Returns JSON array of last `n` readings for selected channels.
// channels bitmask: bit0=temp, bit1=rh, bit2=co2, bit3=voc,
//                   bit4=nox, bit5=pm25
// Example: getHistoryJson(60, 0b111111) — all 6 channels, 60 rows
String getHistoryJson(uint16_t n, uint8_t channels);
