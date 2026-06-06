// ═══════════════════════════════════════════════════════════════════════════
// SEN66 Monitor — Phase 2 + LoRa
// Core 0: WiFi, web server, sensor, serial CLI
// Core 1: LoRa TX/RX (isolated from CYW43 PIO IRQs)
//
// SPI architecture:
//   CYW43 WiFi → PIO-based SPI (internal, not SPI0/SPI1)
//   SX1262 LoRa → SPI1 (GP10/11/12, hardware)
//   SEN66 sensor → I2C0 (GP4/GP5, hardware)
//   No conflicts between any of these.
// ═══════════════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>

#include "globals.h"
#include "config.h"
#include "sensor.h"
#include "datalog.h"
#include "wifi_mgr.h"
#include "http_server.h"
#include "lora_wan.h"

uint8_t  debugLevel    = 0;
bool     streamEnabled = false;
bool     staMode       = false;
uint32_t lastLogTime   = 0;

// ── Serial CLI ────────────────────────────────────────────────────────────
static void printHelp() {
    Serial.println("──────────────────────────────────────────────");
    Serial.println(" SEN66 Serial CLI");
    Serial.println("  help           — this message");
    Serial.println("  status         — WiFi, IP, uptime, flash, LoRa");
    Serial.println("  read           — single human-readable reading");
    Serial.println("  stream         — toggle CSV stream (Python compat)");
    Serial.println("  lora status    — LoRa mode, state, RSSI, last RX");
    Serial.println("  lora send      — force TX now (any mode)");
    Serial.println("  lora log       — show recent LoRa event log");
    Serial.println("  debug <n>      — flags: 0=off 1=web 2=sensor 4=lora");
    Serial.println("──────────────────────────────────────────────");
}

static void printStatus() {
    FSInfo fs;
    LittleFS.info(fs);
    Serial.println("──────────────────────────────────────────────");
    Serial.printf(" WiFi:    %s\n", staMode ? "STA" : "AP");
    Serial.printf(" IP:      %s\n", staMode
        ? WiFi.localIP().toString().c_str()
        : WiFi.softAPIP().toString().c_str());
    Serial.printf(" SSID:    %s\n", staMode ? cfg.wifi_ssid : cfg.ap_ssid);
    Serial.printf(" Uptime:  %lus\n", millis() / 1000);
    Serial.printf(" Flash:   %u / %u bytes used\n", fs.usedBytes, fs.totalBytes);
    Serial.printf(" Log:     %lu rows\n", logRowCount);
    Serial.printf(" Stream:  %s\n", streamEnabled ? "ON" : "OFF");
    Serial.printf(" Debug:   %u\n", debugLevel);
    Serial.println(" ── LoRa ─────────────────────────────────────");
    Serial.printf(" Mode:    %s\n", loraModeStr());
    Serial.printf(" State:   %s\n", loraStateStr());
    Serial.printf(" Uplinks: %lu\n", loraUplinkCount);
    Serial.printf(" RSSI:    %d dBm\n", loraLastRssi);
    Serial.printf(" SNR:     %.1f dB\n", loraLastSnr);
    Serial.printf(" Interval:%us\n", cfg.lora_interval_s);
    if (loraLastRx.length() > 0)
        Serial.printf(" Last RX: %s\n", loraLastRx.c_str());
    Serial.println("──────────────────────────────────────────────");
}

static void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toLowerCase();

    if (cmd == "help") {
        printHelp();
    } else if (cmd == "status") {
        printStatus();
    } else if (cmd == "read") {
        if (latest.valid) printReadable(latest);
        else Serial.println("[cli] No reading yet");
    } else if (cmd == "stream") {
        streamEnabled = !streamEnabled;
        Serial.printf("[cli] Stream %s%s\n",
            streamEnabled ? "ON" : "OFF",
            streamEnabled ? " — CSV, compatible with sen66_monitor.py" : "");
        if (streamEnabled)
            Serial.println("ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm");
    } else if (cmd == "lora status") {
        Serial.printf("[lora] mode=%s state=%s uplinks=%lu rssi=%d snr=%.1f\n",
            loraModeStr(), loraStateStr(),
            loraUplinkCount, loraLastRssi, loraLastSnr);
        if (loraLastRx.length() > 0)
            Serial.printf("[lora] last RX: %s\n", loraLastRx.c_str());
    } else if (cmd == "lora send") {
        loraSendNow();
        Serial.println("[cli] TX queued");
    } else if (cmd == "lora log") {
        String log = loraGetLog(10);
        log.replace("[\"", ""); log.replace("\"]", "");
        log.replace("\",\"", "\n");
        Serial.println(log);
    } else if (cmd.startsWith("debug")) {
        String arg = cmd.substring(5); arg.trim();
        if (arg.length() > 0) {
            debugLevel = (uint8_t)arg.toInt();
            Serial.printf("[cli] Debug → %u  (web=%d sensor=%d lora=%d)\n",
                debugLevel, (int)DBG_WEB, (int)DBG_SENSOR, (int)DBG_MQTT);
        } else {
            Serial.printf("[cli] Debug: %u\n", debugLevel);
        }
    } else if (cmd.length() > 0) {
        Serial.printf("[cli] Unknown: '%s'  (try 'help')\n", cmd.c_str());
    }
}

// ── Core 1: LoRa ──────────────────────────────────────────────────────────
// Runs independently of core 0. SPI1 is initialised here on core 1,
// which is the correct approach — the bus owner should init it.
// CYW43 uses PIO (not SPI0/SPI1) so there is no hardware conflict.
// The 5s delay ensures core 0 config/serial is ready before we print.
void setup1() {
    delay(5000);
    Serial.println("[core1] Starting LoRa...");
    // Init SPI1 here on core 1 — sole owner of this bus
    SPI1.setCS(9);          // dummy CS — RadioLib overrides with Module CS pin
    SPI1.setSCK(10);
    SPI1.setTX(11);
    SPI1.setRX(12);
    SPI1.begin(false);      // false = RadioLib manages CS
    delay(50);
    Serial.println("[core1] SPI1 ready");
    initLoRa();
}

void loop1() {
    loopLoRa();
    delay(50);
}

// ── Core 0: everything else ───────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n═══ SEN66 Monitor Phase 2 + LoRa ═══");

    if (!LittleFS.begin()) {
        Serial.println("[fs] Mount failed — formatting...");
        LittleFS.format();
        LittleFS.begin();
    }
    Serial.println("[fs] LittleFS mounted");

    loadConfig();
    initLog();
    startWiFi();
    setupWebServer();
    initSensor();

    printHelp();
    Serial.println("[boot] Core 0 ready — LoRa on core 1 starts in 5s");
}

void loop() {
    server.handleClient();
    handleSerial();

    static uint32_t lastReadMs = 0;
    if (millis() - lastReadMs >= 1000) {
        lastReadMs = millis();
        readSensor(latest);
        if (streamEnabled && latest.valid)
            printMeasurement(latest);
    }

    if (latest.valid &&
        millis() - lastLogTime >= (uint32_t)cfg.interval_s * 1000) {
        lastLogTime = millis();
        appendLog(latest);
    }
}