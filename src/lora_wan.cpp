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
#include <hardware/resets.h>
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

// ── Coarse-grained bus handoff state ───────────────────────────────────────
// See lora_wan.h for the rationale — one side owns SPI1 entirely at a time.
static volatile bool suspendRequested = false;
static volatile bool suspended        = false;

void lora_request_suspend() { suspendRequested = true; }
void lora_request_resume()  { suspendRequested = false; }
#ifdef LORA_FULLY_DISABLED
bool lora_is_suspended() { return true; }  // radio never active — always "suspended"
#else
bool lora_is_suspended() { return suspended; }
#endif

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
// ── Full SPI1 claim/release for the radio ────────────────────────────────
// Rather than assuming SPI1 is left in a usable state by the LCD (core 0),
// fully tear down and rebuild the peripheral's pin assignment before every
// radio operation, and release it again afterward. Mirrors lcd_spi_claim()/
// lcd_spi_release() in lcd_display.cpp — neither side trusts shared state.
static void radio_spi_claim() {
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);

    // True hardware reset — see lcd_spi_claim() in lcd_display.cpp for why
    // SPI1.end()/begin() alone isn't trusted to fully clear peripheral state.
    reset_block(RESETS_RESET_SPI1_BITS);
    unreset_block_wait(RESETS_RESET_SPI1_BITS);

    SPI1.setRX(12);   // MISO
    SPI1.setTX(11);   // MOSI
    SPI1.setSCK(10);  // SCK
    SPI1.begin(false);
}
static void radio_spi_release() {
    SPI1.end();
}

static void transmitAndListen(const String& payload) {
    mutex_enter_blocking(&spi1_mutex);
    radio_spi_claim();
    if (DBG_MQTT) Serial.printf("[lora] TX %d bytes: %s\n", payload.length(), payload.c_str());

    int16_t state = radio.transmit(payload.c_str());
    lastTxMs = millis();

    if (state != RADIOLIB_ERR_NONE) {
        loraState = LoRaState::TX_FAIL;
        Serial.printf("[lora] TX failed: code %d\n", state);
        radio_spi_release();
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
    radio.clearDio1Action();  // startReceive() above may have (re-)attached it
    radio_spi_release();
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
#ifdef LORA_FULLY_DISABLED
    Serial.println("[lora] LORA_FULLY_DISABLED — skipping entirely, SPI1 untouched by core 1");
    loraState = LoRaState::DISABLED;
    return;
#endif
    Serial.println("[lora] init...");
    // Full init under mutex — lcd_display_init() may run concurrently on core 0
    mutex_enter_blocking(&spi1_mutex);
    radio_spi_claim();
    int16_t state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        radio_spi_release();
        mutex_exit(&spi1_mutex);
        Serial.printf("[lora] SX1262 failed: %d\n", state);
        loraState = LoRaState::ERROR;
        return;
    }
    radio.setDio2AsRfSwitch(true);
    radio.setFrequency(cfg.lora_freq);
    radio.setSpreadingFactor(cfg.lora_sf);
    radio.setBandwidth(cfg.lora_bw);
    radio.setOutputPower(cfg.lora_power);
    radio.setSyncWord(0x12);
    // Detach any DIO1 interrupt RadioLib may attach internally (e.g. via
    // startReceive()). GP20/DIO1 is a shared pin with implications beyond
    // our own mutex — an ISR firing asynchronously wouldn't be caught by
    // any of our SPI1 claim/release or CS-deassertion logic, since it
    // doesn't go through a normal transaction at all. Never re-added
    // after lora_wan.cpp was rewritten from scratch — untested against
    // the current LCD conflict.
    radio.clearDio1Action();
    radio_spi_release();
    mutex_exit(&spi1_mutex);

    loraState     = LoRaState::IDLE;
    lastTxMs      = millis() - MIN_INTERVAL_MS;
    loraStreaming  = cfg.lora_stream;
    Serial.println("[lora] SX1262 OK");
    Serial.printf("[lora] Ready  %.3fMHz  SF%u  BW%.0fkHz  %ddBm\n",
        cfg.lora_freq, cfg.lora_sf, cfg.lora_bw, cfg.lora_power);
}

// ── Loop (called from core 1 loop1) ──────────────────────────────────────
void loopLoRa() {
#ifdef LORA_FULLY_DISABLED
    return;  // core 1 never touches SPI1 — pure isolation test
#endif
    // ── Coarse-grained bus handoff ─────────────────────────────────────────
    // If the LCD wants the bus, sleep the radio and fully release SPI1,
    // then do nothing else until asked to resume. This only happens on
    // rare events (button press / screen timeout), never per-transaction,
    // so re-initialising the radio fully on resume is cheap and reliable.
    if (suspendRequested) {
        if (!suspended) {
            mutex_enter_blocking(&spi1_mutex);
            radio_spi_claim();
            radio.sleep();
            radio.clearDio1Action();
            radio_spi_release();
            mutex_exit(&spi1_mutex);
            suspended = true;
            Serial.println("[lora] Suspended — bus released for LCD");
        }
        return;  // stay idle, touch nothing, until resume is requested
    }
    if (suspended) {
        // Resume requested — fully re-initialise the radio fresh.
        // Simpler and more robust than trying to wake from sleep() with
        // partial state; this only happens when the user closes the menu.
        Serial.println("[lora] Resuming — reclaiming bus");
        initLoRa();
        suspended = false;
        return;
    }

    if (loraState == LoRaState::ERROR || loraState == LoRaState::DISABLED) return;

    // Apply pending settings from CLI — all need mutex
    if (setFreqReq) {
        setFreqReq = false;
        mutex_enter_blocking(&spi1_mutex);
        radio_spi_claim(); applyFreq(setFreqVal); radio_spi_release();
        mutex_exit(&spi1_mutex);
    }
    if (setSFReq) {
        setSFReq = false;
        mutex_enter_blocking(&spi1_mutex);
        radio_spi_claim(); applySF(setSFVal); radio_spi_release();
        mutex_exit(&spi1_mutex);
    }
    if (setPowReq) {
        setPowReq = false;
        mutex_enter_blocking(&spi1_mutex);
        radio_spi_claim(); applyPower(setPowVal); radio_spi_release();
        mutex_exit(&spi1_mutex);
    }

    // One-shot test TX
    if (txTestReq) {
        txTestReq = false;
        float savedFreq = cfg.lora_freq;
        mutex_enter_blocking(&spi1_mutex);
        radio_spi_claim(); applyFreq(txTestFreq); radio_spi_release();
        mutex_exit(&spi1_mutex);
        char buf[128];
        snprintf(buf, sizeof(buf), "<%s>%s", cfg.node_id, txTestMsg.c_str());
        transmitAndListen(String(buf));
        mutex_enter_blocking(&spi1_mutex);
        radio_spi_claim(); applyFreq(savedFreq); radio_spi_release();  // restore
        mutex_exit(&spi1_mutex);
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