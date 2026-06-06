// ═══════════════════════════════════════════════════════════════════════════
// SEN66 Monitor — Phase 2 + LoRaWAN/Raw
// ═══════════════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>

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
        // Print each entry on its own line
        log.replace("[\"", "");
        log.replace("\"]", "");
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

// ── Setup / Loop ──────────────────────────────────────────────────────────
// ── Core 1: LoRa runs here, isolated from CYW43 WiFi interrupts ──────────
// The CYW43 WiFi driver on Pico W conflicts with GPIO interrupts on core 0.
// Running RadioLib on core 1 keeps the IRQ handlers on separate cores.

void setup1() {
    // Wait for core 0 to complete WiFi + sensor init
    // Serial is shared and safe after core 0 has started
    delay(6000);
    Serial.println("[core1] LoRa init starting...");
    Serial.flush();
    initLoRa();
}

void loop1() {
    loopLoRa();
    delay(50);
}

// ── Core 0: WiFi, web server, sensor, serial ──────────────────────────────
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
    initLoraSPI();   // SPI1 must be claimed before CYW43 WiFi starts
    startWiFi();
    setupWebServer();
    initSensor();
    // LoRa init happens on core 1 via setup1()

    printHelp();
    Serial.println("[boot] Core 0 ready — LoRa starting on core 1");
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