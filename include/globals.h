#pragma once
#include <stdint.h>
#include <stdbool.h>

// ── Debug level flags (OR-able) ───────────────────────────────────────────
// 0=off  1=web  2=sensor  4=lora  8=bus (LCD/LoRa SPI1 handoff internals)
extern uint8_t debugLevel;
#define DBG_WEB    (debugLevel & 1)
#define DBG_SENSOR (debugLevel & 2)
#define DBG_MQTT   (debugLevel & 4)
#define DBG_BUS    (debugLevel & 8)

// ── Runtime state ─────────────────────────────────────────────────────────
extern bool     streamEnabled;
extern bool     staMode;
extern uint32_t lastLogTime;
extern uint32_t logRowCount;