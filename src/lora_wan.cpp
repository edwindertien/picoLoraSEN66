// ═══════════════════════════════════════════════════════════════════════════
// lora_wan.cpp — Plain LoRa with ACK/RSSI range testing
//
// Waveshare Pico-LoRa-SX1262:
//   MISO=GP12  MOSI=GP11  SCK=GP10
//   CS=GP3  DIO1=GP20  RST=GP15  BUSY=GP2
//
// All radio activity on core 1.
// CLI commands set flags; core 1 consumes them in loopLoRa().
// ═══════════════════════════════════════════════════════════════════════════

#include "lora_wan.h"
#include "config.h"
#include <pico/mutex.h>
extern mutex_t spi1_mutex;  // defined in lcd_display.cpp
#include "sensor.h"
#include "globals.h"
#include <RadioLib.h>

// ── Radio object — SPI1 already init'd by setup1() before initLoRa() ─────
extern SX1262 radio;   // defined in main.cpp setup1 scope — forward declared

// ── State ─────────────────────────────────────────────────────────────────
LoRaState loraState    = LoRaState::DISABLED;
uint32_t  loraTxCount  = 0;
int16_t   loraLastRssi = 0;
float     loraLastSnr  = 0.0f;
String    loraLastAck  = "";
bool      loraStreaming = false;

// ── Inter-core flags (volatile — written by core 0, read by core 1) ──────
static volatile bool    txSensorReq  = false;
static volatile bool    txTestReq    = false;
static volatile float   txTestFreq   = 868.1f;
static String           txTestMsg    = "";  // String is not volatile-safe but
                                            // set before flag, read after
static volatile bool    setFreqReq   = false;
static volatile float   setFreqVal   = 868.1f;
static volatile bool    setSFReq     = false;
static volatile uint8_t setSFVal     = 9;
static volatile bool    setPowReq    = false;
static volatile int8_t  setPowVal    = 14;

// ── Timing ────────────────────────────────────────────────────────────────
#define ACK_TIMEOUT_MS   2000   // RX window after TX
#define MIN_INTERVAL_MS  30000  // EU868 duty cycle floor

uint32_t loraLastTxMs = 0;
#define lastTxMs loraLastTxMs

// ── Duty cycle guard ──────────────────────────────────────────────────────
static bool dutyOk() {
    if (millis() - lastTxMs < MIN_INTERVAL_MS) {
        uint32_t wait = MIN_INTERVAL_MS - (millis() - lastTxMs);
        Serial.printf("[lora] Duty cycle: wait %lus\n", wait/1000);
        return false;
    }
    return true;
}

// ── Build sensor JSON payload ─────────────────────────────────────────────
static String buildPayload() {
    char buf[128];
    if (!latest.valid) return String("<") + cfg.node_id + ">no_data";
    if (latest.pm_sat)
        snprintf(buf, sizeof(buf),
            "<%s>{\"t\":%.2f,\"rh\":%.1f,\"co2\":%u,\"voc\":%.0f,\"nox\":%.0f,\"pm\":\"sat\"}",
            cfg.node_id, latest.temp, latest.rh, latest.co2, latest.voc, latest.nox);
    else
        snprintf(buf, sizeof(buf),
            "<%s>{\"t\":%.2f,\"rh\":%.1f,\"co2\":%u,\"voc\":%.0f,\"nox\":%.0f"
            ",\"pm25\":%.1f,\"pm10\":%.1f,\"pm1\":%.1f}",
            cfg.node_id, latest.temp, latest.rh, latest.co2, latest.voc, latest.nox,
            latest.pm25, latest.pm10, latest.pm1);
    return String(buf);
}

// ── TX + ACK window ──────────────────────────────────────────────────────
static void transmitAndListen(const String& payload) {
    mutex_enter_blocking(&spi1_mutex);
    Serial.printf("[lora] TX %d bytes: %s\n", payload.length(), payload.c_str());

    int16_t state = radio.transmit(payload.c_str());
    lastTxMs = millis();

    if (state != RADIOLIB_ERR_NONE) {
        loraState = LoRaState::TX_FAIL;
        Serial.printf("[lora] TX failed: code %d\n", state);
        mutex_exit(&spi1_mutex);
        return;
    }
    loraTxCount++;
    loraState = LoRaState::TX_OK;
    Serial.printf("[lora] TX #%lu OK\n", loraTxCount);

    // ── ACK listen window ─────────────────────────────────────────────────
    radio.startReceive();
    uint32_t rxStart = millis();
    bool gotAck = false;

    while (millis() - rxStart < ACK_TIMEOUT_MS) {
        if (radio.available()) {
            uint8_t rxBuf[128] = {0};
            size_t  rxLen = sizeof(rxBuf) - 1;
            if (radio.readData(rxBuf, rxLen) == RADIOLIB_ERR_NONE) {
                loraLastRssi = radio.getRSSI();
                loraLastSnr  = radio.getSNR();
                loraLastAck  = String((char*)rxBuf);
                Serial.printf("[lora] ACK: \"%s\"  RSSI=%ddBm  SNR=%.1fdB\n",
                    loraLastAck.c_str(), loraLastRssi, loraLastSnr);
                gotAck = true;
            }
            break;
        }
        delay(5);
    }

    if (!gotAck) {
        if (DBG_MQTT) Serial.printf("[lora] No ACK (timeout %dms)\n", ACK_TIMEOUT_MS);
    }

    radio.standby();
    mutex_exit(&spi1_mutex);
}

// ── Apply radio settings ──────────────────────────────────────────────────
static void applyFreq(float mhz) {
    radio.setFrequency(mhz);
    Serial.printf("[lora] Frequency → %.3f MHz\n", mhz);
}

static void applySF(uint8_t sf) {
    if (sf < 7 || sf > 12) { Serial.println("[lora] SF must be 7-12"); return; }
    radio.setSpreadingFactor(sf);
    Serial.printf("[lora] SF → %u\n", sf);
}

static void applyPower(int8_t dbm) {
    if (dbm < 2 || dbm > 22) { Serial.println("[lora] Power must be 2-22 dBm"); return; }
    radio.setOutputPower(dbm);
    Serial.printf("[lora] Power → %d dBm\n", dbm);
}

// ── Public API (called from core 0 CLI) ───────────────────────────────────
void loraSendSensor()                        { txSensorReq = true; }
void loraSetFreq(float mhz)                  { setFreqVal = mhz; setFreqReq = true; }
void loraSetSF(uint8_t sf)                   { setSFVal = sf;  setSFReq  = true; }
void loraSetPower(int8_t dbm)                { setPowVal = dbm; setPowReq = true; }

void loraSendTest(float freq, const String& msg) {
    txTestFreq = freq;
    txTestMsg  = msg;
    txTestReq  = true;
}

const char* loraStateStr() {
    switch (loraState) {
        case LoRaState::DISABLED: return "disabled";
        case LoRaState::IDLE:     return "idle";
        case LoRaState::TX_OK:    return "tx_ok";
        case LoRaState::TX_FAIL:  return "tx_fail";
        case LoRaState::ERROR:    return "error";
        default:                  return "unknown";
    }
}

// ── Init (called from core 1 setup1, after SPI1.begin()) ─────────────────
void initLoRa() {
    Serial.println("[lora] init...");
    int16_t state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[lora] SX1262 failed: %d\n", state);
        loraState = LoRaState::ERROR;
        return;
    }
    Serial.println("[lora] SX1262 OK");
    radio.setDio2AsRfSwitch(true);

    // Apply config defaults
    applyFreq(cfg.lora_freq);
    applySF(cfg.lora_sf);
    radio.setBandwidth(cfg.lora_bw);
    applyPower(cfg.lora_power);
    radio.setSyncWord(0x12);  // private network (not LoRaWAN 0x34)

    loraState  = LoRaState::IDLE;
    lastTxMs   = millis() - MIN_INTERVAL_MS;  // allow immediate first TX
    loraStreaming = cfg.lora_stream;
    Serial.printf("[lora] Ready  %.3fMHz  SF%u  BW%.0fkHz  %ddBm\n",
        cfg.lora_freq, cfg.lora_sf, cfg.lora_bw, cfg.lora_power);
}

// ── Loop (called from core 1 loop1) ──────────────────────────────────────
void loopLoRa() {
    if (loraState == LoRaState::ERROR || loraState == LoRaState::DISABLED) return;

    // Apply pending settings from CLI
    if (setFreqReq) { setFreqReq = false; applyFreq(setFreqVal); }
    if (setSFReq)   { setSFReq   = false; applySF(setSFVal); }
    if (setPowReq)  { setPowReq  = false; applyPower(setPowVal); }

    // One-shot test TX
    if (txTestReq) {
        txTestReq = false;
        float savedFreq = cfg.lora_freq;
        applyFreq(txTestFreq);
        char buf[128];
        snprintf(buf, sizeof(buf), "<%s>%s", cfg.node_id, txTestMsg.c_str());
        transmitAndListen(String(buf));
        applyFreq(savedFreq);  // restore
        return;
    }

    // One-shot sensor TX
    if (txSensorReq) {
        txSensorReq = false;
        if (dutyOk()) transmitAndListen(buildPayload());
        return;
    }

    // Stream mode — auto TX at interval
    if (loraStreaming) {
        uint32_t intervalMs = max((uint32_t)cfg.lora_interval_s * 1000UL,
                                  (uint32_t)MIN_INTERVAL_MS);
        if (millis() - lastTxMs >= intervalMs) {
            transmitAndListen(buildPayload());
        }
    }
}
