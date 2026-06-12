// ═══════════════════════════════════════════════════════════════════════════
// SEN66 Monitor — SPI1 init test
// Core 0: everything as normal
// Core 1: SPI1 init only — no RadioLib yet
// ═══════════════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RadioLib.h>

#include "globals.h"
#include "config.h"
#include "sensor.h"
#include "datalog.h"
#include "wifi_mgr.h"
#include "http_server.h"

uint8_t  debugLevel    = 0;
volatile bool loraTxRequested = false;  // set by CLI, consumed by core 1
bool     streamEnabled = false;
bool     staMode       = false;
uint32_t lastLogTime   = 0;

// ── Core 1: SPI1 + radio.begin() test ───────────────────────────────────
// Waveshare Pico-LoRa-SX1262 pins:
//   MISO=GP12  MOSI=GP11  SCK=GP10
//   CS=GP3  DIO1=GP20  RST=GP15  BUSY=GP2
static SX1262 radio = new Module(3, 20, 15, 2, SPI1);

void setup1() {
    rp2040.fifo.pop();  // wait for core 0
    Serial.println("[core1] Configuring SPI1 pins...");
    SPI1.setRX(12);
    SPI1.setTX(11);
    SPI1.setSCK(10);
    SPI1.begin(false);
    Serial.println("[core1] SPI1 ready — calling radio.begin()...");
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("[core1] SX1262 OK!");
        radio.setDio2AsRfSwitch(true);
    } else {
        Serial.printf("[core1] SX1262 failed: code %d\n", state);
    }
}

void loop1() {
    if (loraTxRequested) {
        loraTxRequested = false;
        Serial.println("[core1] Transmitting...");
        int state = radio.transmit("SEN66 hello from Pico W");
        if (state == RADIOLIB_ERR_NONE)
            Serial.println("[core1] TX OK");
        else
            Serial.printf("[core1] TX failed: %d", state);
    }
    sleep_ms(50);
}

// ── Serial CLI ────────────────────────────────────────────────────────────
static void printHelp() {
    Serial.println("─────────────────────────────────────────────");
    Serial.println(" SEN66 Serial CLI  [radio.begin() test]");
    Serial.println("  help  status  read  stream  debug <n>");
    Serial.println("  lora send          — transmit test packet");
    Serial.println("  lora freq <mhz>    — set frequency (e.g. 868.1)");
    Serial.println("─────────────────────────────────────────────");
}

static void printStatus() {
    FSInfo fs; LittleFS.info(fs);
    Serial.println("─────────────────────────────────────────────");
    Serial.printf(" WiFi:   %s  IP: %s\n", staMode?"STA":"AP",
        staMode?WiFi.localIP().toString().c_str():WiFi.softAPIP().toString().c_str());
    Serial.printf(" Uptime: %lus\n", millis()/1000);
    Serial.printf(" Flash:  %u/%u bytes\n", fs.usedBytes, fs.totalBytes);
    Serial.printf(" Log:    %lu rows\n", logRowCount);
    Serial.println("─────────────────────────────────────────────");
}

static void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toLowerCase();
    if      (cmd == "help")   printHelp();
    else if (cmd == "status") printStatus();
    else if (cmd == "read") {
        if (latest.valid) printReadable(latest);
        else Serial.println("[cli] No reading yet");
    } else if (cmd == "stream") {
        streamEnabled = !streamEnabled;
        Serial.printf("[cli] Stream %s\n", streamEnabled?"ON":"OFF");
        if (streamEnabled)
            Serial.println("ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm");
    } else if (cmd.startsWith("debug")) {
        String arg = cmd.substring(5); arg.trim();
        if (arg.length()>0) { debugLevel=(uint8_t)arg.toInt(); Serial.printf("[cli] Debug → %u\n",debugLevel); }
    } else if (cmd == "lora send") {
        loraTxRequested = true;
        Serial.println("[cli] TX queued");
    } else if (cmd == "lora freq") {
        // Set frequency: lora freq 868.1
        // (for now just confirms current default)
        Serial.println("[cli] Frequency: 434.0MHz (default) — use 'lora freq <mhz>' to change");
    } else if (cmd.startsWith("lora freq ")) {
        float freq = cmd.substring(10).toFloat();
        if (freq > 100.0 && freq < 1000.0) {
            radio.setFrequency(freq);
            Serial.printf("[cli] Frequency set to %.1f MHz\n", freq);
        } else {
            Serial.println("[cli] Invalid frequency");
        }
    } else if (cmd.length()>0)
        Serial.printf("[cli] Unknown: '%s'\n", cmd.c_str());
}

// ── Core 0 ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("═══ SEN66 Monitor — radio.begin() test ═══");

    if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
    Serial.println("[fs] mounted");
    loadConfig();
    initLog();
    startWiFi();
    setupWebServer();
    initSensor(cfg.fan_cleaning);

    printHelp();
    Serial.println("[boot] Complete — signalling core 1");
    rp2040.fifo.push(0xAA55AA55);
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