#include "sensor.h"
#include "globals.h"
#include <Wire.h>

#ifdef NO_ERROR
  #undef NO_ERROR
#endif
#define NO_ERROR 0

SensirionI2cSen66 sen66;
Measurement       latest;

static char errMsg[64];

static void printError(const char* label, int16_t err) {
    errorToString(err, errMsg, sizeof(errMsg));
    Serial.printf("[ERR] %s: %s\n", label, errMsg);
}

void initSensor(bool doFanCleaning) {
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    sen66.begin(Wire, SEN66_I2C_ADDR_6B);

    int16_t err = sen66.deviceReset();
    if (err != NO_ERROR) printError("deviceReset", err);
    delay(1200);

    err = sen66.startContinuousMeasurement();
    if (err != NO_ERROR) printError("startMeasurement", err);

    if (doFanCleaning) {
        err = sen66.startFanCleaning();
        if (err != NO_ERROR) printError("fanCleaning", err);
        Serial.print("[sen66] Fan cleaning ");
        for (int i = 0; i < 10; i++) {
            delay(1000);
            Serial.print(".");
        }
        Serial.println(" done");
    }
    Serial.println("[sen66] Ready");
}

bool readSensor(Measurement& m) {
    bool    dataReady = false;
    uint8_t padding   = 0;

    int16_t err = sen66.getDataReady(padding, dataReady);
    if (err != NO_ERROR) { printError("getDataReady", err); return false; }
    if (!dataReady) return false;

    uint16_t pm1Raw, pm25Raw, pm4Raw, pm10Raw;
    int16_t  rhRaw, tempRaw, vocRaw, noxRaw;
    uint16_t co2Raw;

    err = sen66.readMeasuredValuesAsIntegers(
        pm1Raw, pm25Raw, pm4Raw, pm10Raw,
        rhRaw, tempRaw, vocRaw, noxRaw, co2Raw);
    if (err != NO_ERROR) { printError("readMeasured", err); return false; }

    m.ts_ms = millis();
    m.rh    = rhRaw   / 100.0f;
    m.temp  = tempRaw / 200.0f;
    m.voc   = (vocRaw  == INT16_INVALID)  ? 0.0f : vocRaw  / 10.0f;
    m.nox   = (noxRaw  == INT16_INVALID)  ? 0.0f : noxRaw  / 10.0f;
    m.co2   = (co2Raw  == UINT16_INVALID) ? 0     : co2Raw;

    bool pmInvalid = (pm1Raw == UINT16_INVALID);
    float pm1  = pmInvalid ? 0.0f : pm1Raw  / 10.0f;
    float pm25 = pmInvalid ? 0.0f : pm25Raw / 10.0f;
    float pm4  = pmInvalid ? 0.0f : pm4Raw  / 10.0f;
    float pm10 = pmInvalid ? 0.0f : pm10Raw / 10.0f;
    m.pm_sat = (!pmInvalid) && (fabsf(pm25 - PM_SATURATED) < 1.0f);
    m.pm1 = pm1; m.pm25 = pm25; m.pm4 = pm4; m.pm10 = pm10;
    m.valid = true;

    if (DBG_SENSOR) {
        Serial.printf("[sensor] T=%.2f RH=%.1f CO2=%u VOC=%.0f NOx=%.0f PM2.5=%s\n",
            m.temp, m.rh, m.co2, m.voc, m.nox,
            m.pm_sat ? "SAT" : String(m.pm25, 1).c_str());
    }
    return true;
}

void printReadable(const Measurement& m) {
    Serial.printf(
        "[read] T=%.2f°C  RH=%.1f%%  CO2=%uppm  VOC=%.0f  NOx=%.0f"
        "  PM2.5=%s  uptime=%lus\n",
        m.temp, m.rh, m.co2, m.voc, m.nox,
        m.pm_sat ? "SAT" : String(m.pm25, 1).c_str(),
        millis() / 1000);
}

void printMeasurement(const Measurement& m) {
    char s1[10], s2[10], s3[10], s4[10];
    if (m.pm_sat) {
        strcpy(s1,"null"); strcpy(s2,"null");
        strcpy(s3,"null"); strcpy(s4,"null");
    } else {
        dtostrf(m.pm1,  1, 1, s1); dtostrf(m.pm25, 1, 1, s2);
        dtostrf(m.pm4,  1, 1, s3); dtostrf(m.pm10, 1, 1, s4);
    }
    Serial.printf("%lu,%s,%s,%s,%s,%.1f,%.2f,%.1f,%.1f,%u\n",
        m.ts_ms, s1, s2, s3, s4,
        m.rh, m.temp, m.voc, m.nox, m.co2);
}