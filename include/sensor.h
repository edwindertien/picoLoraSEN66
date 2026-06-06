#pragma once
#include <Arduino.h>
#include <SensirionI2cSen66.h>

// ── Sentinel values (SEN66 datasheet) ────────────────────────────────────
static constexpr uint16_t UINT16_INVALID = 0xFFFF;
static constexpr int16_t  INT16_INVALID  = 0x7FFF;
static constexpr float    PM_SATURATED   = 6016.0f;  // 0xEB00/10

struct Measurement {
    bool     valid   = false;
    bool     pm_sat  = false;
    float    pm1, pm25, pm4, pm10;
    float    rh, temp;
    float    voc, nox;
    uint16_t co2;
    uint32_t ts_ms;
};

extern Measurement latest;
extern SensirionI2cSen66 sen66;

void initSensor();
bool readSensor(Measurement& m);

// Human-readable one-liner for [read] CLI command
void printReadable(const Measurement& m);

// CSV line — Phase 1 compatible, used by stream mode
// ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm
void printMeasurement(const Measurement& m);
