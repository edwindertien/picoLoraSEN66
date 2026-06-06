// ═══════════════════════════════════════════════════════════════════════════
// lora_wan.cpp — LoRa radio: LoRaWAN OTAA mode + Raw peer-to-peer mode
//
// Hardware: Waveshare Pico-LoRa-SX1262-868M stacked on Pico W
//   SPI1  SCK  → GP10   MOSI → GP11   MISO → GP12
//   CS → GP3   DIO1 → GP20   RST → GP15   BUSY → GP2
// ═══════════════════════════════════════════════════════════════════════════

#include "lora_wan.h"
#include "config.h"
#include "sensor.h"
#include "globals.h"

#include <SPI.h>
#include <RadioLib.h>

// ── Pin definitions ───────────────────────────────────────────────────────
#define LORA_CS    3
#define LORA_DIO1  20
#define LORA_RST   15
#define LORA_BUSY  2
#define LORA_SCK   10
#define LORA_MOSI  11
#define LORA_MISO  12

// Raw mode LoRa parameters (EU868, must match receiver)
#define RAW_FREQ_MHZ   868.1f
#define RAW_BW_KHZ     125.0f
#define RAW_SF         9
#define RAW_CR         7        // coding rate 4/7
#define RAW_SYNC       0x12     // private LoRa sync word (0x34 = LoRaWAN)
#define RAW_POWER_DBM  14
#define RAW_RX_MS      2000     // listen window after TX (ms)

// ── RadioLib objects ──────────────────────────────────────────────────────
// Earle Philhower: use global SPI1 with RADIOLIB_DEFAULT_SPI_SETTINGS
static SX1262      radio = new Module(LORA_CS, LORA_DIO1,
                                      LORA_RST, LORA_BUSY,
                                      SPI1, RADIOLIB_DEFAULT_SPI_SETTINGS);
static LoRaWANNode node(&radio, &EU868);

// ── State ─────────────────────────────────────────────────────────────────
LoRaMode loraMode         = LoRaMode::DISABLED;
LoRaState loraState       = LoRaState::DISABLED;
uint32_t  loraUplinkCount = 0;
int16_t   loraLastRssi    = 0;
float     loraLastSnr     = 0.0f;
String    loraLastRx      = "";

static bool     joinedOk      = false;
static uint32_t lastTxMs      = 0;
static bool     sendRequested = false;

// Ring buffer of recent log lines for the web /lorastatus endpoint
// Each entry: timestamp + message
static const uint8_t LOG_SIZE = 12;
static String   loraLog[LOG_SIZE];
static uint8_t  loraLogHead = 0;

// ── Logging — serial + web ring buffer ───────────────────────────────────
static void loraLogLine(const char* msg) {
    Serial.println(msg);
    String entry = String("[") + String(millis()/1000) + "s] " + String(msg);
    loraLog[loraLogHead] = entry;
    loraLogHead = (loraLogHead + 1) % LOG_SIZE;
}

static void loraLogLinef(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    loraLogLine(buf);
}

// Returns last N log lines as a JSON array string
// Called by web endpoint /lorastatus
String loraGetLog(uint8_t n) {
    if (n > LOG_SIZE) n = LOG_SIZE;
    String out = "[";
    // Walk backwards from head to get newest first
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (loraLogHead + LOG_SIZE - 1 - i) % LOG_SIZE;
        if (loraLog[idx].length() == 0) continue;
        if (out.length() > 1) out += ",";
        out += "\"";
        // Escape quotes in message
        String e = loraLog[idx];
        e.replace("\"", "'");
        out += e;
        out += "\"";
    }
    out += "]";
    return out;
}

// ── Cayenne LPP encoder ───────────────────────────────────────────────────
#define LPP_CH_TEMP  1
#define LPP_CH_RH    2
#define LPP_CH_VOC   4
#define LPP_CH_NOX   5
#define LPP_CH_CO2   6
#define LPP_CH_PM25  7
#define LPP_CH_PM10  8
#define LPP_CH_PM1   9

static uint8_t lppBuf[64];
static uint8_t lppLen = 0;

static void lppReset() { lppLen = 0; }

static void lppAddTemperature(uint8_t ch, float val) {
    int16_t v = (int16_t)(val * 10);
    lppBuf[lppLen++] = ch;   lppBuf[lppLen++] = 0x67;
    lppBuf[lppLen++] = (v >> 8) & 0xFF;
    lppBuf[lppLen++] = v & 0xFF;
}
static void lppAddHumidity(uint8_t ch, float val) {
    lppBuf[lppLen++] = ch;   lppBuf[lppLen++] = 0x68;
    lppBuf[lppLen++] = (uint8_t)(val * 2);
}
static void lppAddAnalog(uint8_t ch, float val) {
    int16_t v = (int16_t)(val * 100);
    lppBuf[lppLen++] = ch;   lppBuf[lppLen++] = 0x02;
    lppBuf[lppLen++] = (v >> 8) & 0xFF;
    lppBuf[lppLen++] = v & 0xFF;
}

// ── Key parsing ───────────────────────────────────────────────────────────
static uint8_t hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

static uint64_t hexToU64(const char* hex) {
    uint64_t v = 0;
    for (uint8_t i = 0; i < 16; i++)
        v = (v << 4) | hexNibble(hex[i]);
    return v;
}

static bool hexToBytes16(const char* hex, uint8_t* out) {
    if (strlen(hex) != 32) return false;
    for (uint8_t i = 0; i < 16; i++)
        out[i] = (hexNibble(hex[i*2]) << 4) | hexNibble(hex[i*2+1]);
    return true;
}

// ── State strings ─────────────────────────────────────────────────────────
const char* loraStateStr() {
    switch (loraState) {
        case LoRaState::DISABLED:  return "disabled";
        case LoRaState::JOINING:   return "joining";
        case LoRaState::JOINED:    return "joined";
        case LoRaState::SEND_OK:   return "send_ok";
        case LoRaState::SEND_FAIL: return "send_fail";
        case LoRaState::ERROR:     return "error";
        default:                   return "unknown";
    }
}

const char* loraModeStr() {
    switch (loraMode) {
        case LoRaMode::DISABLED: return "disabled";
        case LoRaMode::LORAWAN:  return "lorawan";
        case LoRaMode::RAW:      return "raw";
        default:                 return "unknown";
    }
}

// ── Build compact JSON payload (raw mode) ─────────────────────────────────
static String buildRawJson() {
    char buf[128];
    if (latest.pm_sat) {
        snprintf(buf, sizeof(buf),
            "{\"t\":%.2f,\"rh\":%.1f,\"co2\":%u,\"voc\":%.0f,\"nox\":%.0f,\"pm\":\"sat\"}",
            latest.temp, latest.rh, latest.co2, latest.voc, latest.nox);
    } else {
        snprintf(buf, sizeof(buf),
            "{\"t\":%.2f,\"rh\":%.1f,\"co2\":%u,\"voc\":%.0f,\"nox\":%.0f,\"pm25\":%.1f,\"pm10\":%.1f,\"pm1\":%.1f}",
            latest.temp, latest.rh, latest.co2, latest.voc, latest.nox,
            latest.pm25, latest.pm10, latest.pm1);
    }
    return String(buf);
}

// ── Raw TX + RX window ────────────────────────────────────────────────────
static void doRawSend() {
    if (!latest.valid) {
        loraLogLine("[lora/raw] Skipping — no valid sensor data");
        return;
    }
    String payload = buildRawJson();

    if (DBG_MQTT)
        loraLogLinef("[lora/raw] TX (%d bytes): %s", payload.length(), payload.c_str());

    int16_t state = radio.transmit(payload.c_str());
    if (state == RADIOLIB_ERR_NONE) {
        loraUplinkCount++;
        loraLastRssi = 0;
        loraLastSnr  = 0.0f;
        loraState    = LoRaState::SEND_OK;
        loraLogLinef("[lora/raw] TX #%lu OK  %d bytes", loraUplinkCount, payload.length());
    } else {
        loraState = LoRaState::SEND_FAIL;
        loraLogLinef("[lora/raw] TX failed: code %d", state);
    }

    // Listen for reply — startReceive + poll loop (SX126x has no timed receive)
    radio.startReceive();
    uint32_t rxStart = millis();
    state = RADIOLIB_ERR_RX_TIMEOUT;
    while (millis() - rxStart < RAW_RX_MS) {
        if (radio.available()) {
            uint8_t rxBuf[128] = {0};
            size_t  rxLen = sizeof(rxBuf) - 1;
            state = radio.readData(rxBuf, rxLen);
            if (state == RADIOLIB_ERR_NONE) {
                loraLastRssi = radio.getRSSI();
                loraLastSnr  = radio.getSNR();
                loraLastRx   = String((char*)rxBuf);
                loraLogLinef("[lora/raw] RX: %s  RSSI=%d  SNR=%.1f",
                    loraLastRx.c_str(), loraLastRssi, loraLastSnr);
            } else {
                loraLogLinef("[lora/raw] RX error: code %d", state);
            }
            break;
        }
        delay(10);
    }
    if (state == RADIOLIB_ERR_RX_TIMEOUT) {
        if (DBG_MQTT) loraLogLine("[lora/raw] RX timeout (no reply)");
    }
    radio.standby();

    lastTxMs      = millis();
    sendRequested = false;
}

// ── LoRaWAN uplink ────────────────────────────────────────────────────────
static void doLoRaWANSend() {
    if (!latest.valid) {
        loraLogLine("[lora/wan] Skipping — no valid sensor data");
        return;
    }

    lppReset();
    lppAddTemperature(LPP_CH_TEMP, latest.temp);
    lppAddHumidity(LPP_CH_RH,     latest.rh);
    lppAddAnalog(LPP_CH_VOC,      latest.voc);
    lppAddAnalog(LPP_CH_NOX,      latest.nox);
    lppAddAnalog(LPP_CH_CO2,      (float)latest.co2);
    if (!latest.pm_sat) {
        lppAddAnalog(LPP_CH_PM25, latest.pm25);
        lppAddAnalog(LPP_CH_PM10, latest.pm10);
        lppAddAnalog(LPP_CH_PM1,  latest.pm1);
    }

    if (DBG_MQTT) {
        char hex[lppLen*2+1];
        for (uint8_t i = 0; i < lppLen; i++) sprintf(hex+i*2, "%02X", lppBuf[i]);
        hex[lppLen*2] = '\0';
        loraLogLinef("[lora/wan] TX %u bytes: %s", lppLen, hex);
    }

    int16_t state = node.sendReceive(lppBuf, lppLen, 1, nullptr, nullptr, false);

    if (state == RADIOLIB_ERR_NONE || state == RADIOLIB_ERR_RX_TIMEOUT) {
        loraUplinkCount++;
        loraLastRssi = radio.getRSSI();
        loraLastSnr  = radio.getSNR();
        loraState    = LoRaState::SEND_OK;
        loraLogLinef("[lora/wan] Uplink #%lu OK  RSSI=%d  SNR=%.1f",
            loraUplinkCount, loraLastRssi, loraLastSnr);
    } else {
        loraState = LoRaState::SEND_FAIL;
        loraLogLinef("[lora/wan] Uplink failed: code %d", state);
    }

    lastTxMs      = millis();
    sendRequested = false;
}

// ── SX1262 hardware init (shared by both modes) ───────────────────────────
// Call this from core 0, BEFORE WiFi init
void initLoraSPI() {
    SPI1.setCS(LORA_CS);
    SPI1.setSCK(LORA_SCK);
    SPI1.setTX(LORA_MOSI);
    SPI1.setRX(LORA_MISO);
    SPI1.begin(false);
    Serial.println("[lora] SPI1 initialised on core 0");
}

static bool initRadioHardware() {
    // SPI1 already started by initLoraSPI() on core 0
    delay(10);
    loraLogLine("[lora] Calling radio.begin()...");
    int16_t state = radio.begin(868.1, 125.0, 9, 7, 0x12, 14, 8, 0.0, false);
    if (state != RADIOLIB_ERR_NONE) {
        loraLogLinef("[lora] SX1262 init failed: code %d", state);
        return false;
    }
    loraLogLine("[lora] SX1262 OK");
    radio.setDio2AsRfSwitch(true);
    return true;
}

// ── Init ──────────────────────────────────────────────────────────────────
void initLoRa() {
    // Parse mode from config string
    String modeStr = String(cfg.lora_mode);
    if (modeStr == "lorawan")     loraMode = LoRaMode::LORAWAN;
    else if (modeStr == "raw")    loraMode = LoRaMode::RAW;
    else { loraMode = LoRaMode::DISABLED; loraState = LoRaState::DISABLED;
           loraLogLine("[lora] Disabled"); return; }

    loraLogLinef("[lora] Mode: %s", loraModeStr());

    if (!initRadioHardware()) { loraState = LoRaState::ERROR; return; }

    if (loraMode == LoRaMode::RAW) {
        // Configure radio for raw LoRa
        // Parameters already set in radio.begin() — just confirm
        loraState = LoRaState::JOINED;
        loraLogLinef("[lora/raw] Ready  %.1fMHz  SF%d  BW%.0fkHz",
            RAW_FREQ_MHZ, RAW_SF, RAW_BW_KHZ);
        lastTxMs = millis();
        return;
    }

    // LoRaWAN mode — validate and join
    if (strlen(cfg.lora_dev_eui) != 16 ||
        strlen(cfg.lora_app_eui) != 16 ||
        strlen(cfg.lora_app_key) != 32) {
        loraLogLine("[lora/wan] ERROR: EUI/key wrong length");
        loraState = LoRaState::ERROR; return;
    }

    uint64_t joinEUI = hexToU64(cfg.lora_app_eui);
    uint64_t devEUI  = hexToU64(cfg.lora_dev_eui);
    uint8_t  appKey[16];
    if (!hexToBytes16(cfg.lora_app_key, appKey)) {
        loraLogLine("[lora/wan] ERROR: AppKey parse failed");
        loraState = LoRaState::ERROR; return;
    }

    node.beginOTAA(joinEUI, devEUI, nullptr, appKey);

    loraLogLine("[lora/wan] Joining... (may take ~30s)");
    loraState = LoRaState::JOINING;

    int16_t state = node.activateOTAA();
    if (state != RADIOLIB_LORAWAN_NEW_SESSION &&
        state != RADIOLIB_LORAWAN_SESSION_RESTORED) {
        loraLogLinef("[lora/wan] Join failed: code %d", state);
        loraState = LoRaState::ERROR; return;
    }

    joinedOk  = true;
    loraState = LoRaState::JOINED;
    loraLogLine("[lora/wan] Joined!");
    lastTxMs = millis();
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loopLoRa() {
    if (loraMode  == LoRaMode::DISABLED) return;
    if (loraState == LoRaState::ERROR)   return;
    if (loraMode  == LoRaMode::LORAWAN && !joinedOk) return;

    uint32_t intervalMs = (uint32_t)cfg.lora_interval_s * 1000UL;
    bool     due        = (millis() - lastTxMs >= intervalMs);

    if (sendRequested || due) {
        if (loraMode == LoRaMode::RAW)     doRawSend();
        else                               doLoRaWANSend();
    }
}

void loraSendNow() {
    if (loraMode == LoRaMode::DISABLED) {
        Serial.println("[lora] Disabled — cannot send");
        return;
    }
    if (loraMode == LoRaMode::LORAWAN && !joinedOk) {
        Serial.println("[lora] Not joined — cannot send");
        return;
    }
    sendRequested = true;
}