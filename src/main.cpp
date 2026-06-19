// ═══════════════════════════════════════════════════════════════════════════
// SEN66 Monitor — Phase 2 + Plain LoRa
// Core 0: WiFi, web, sensor, serial CLI
// Core 1: LoRa TX/RX (SPI1 owned entirely by core 1)
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
#include "lora_wan.h"

uint8_t  debugLevel    = 0;
bool     streamEnabled = false;
bool     staMode       = false;
uint32_t lastLogTime   = 0;

// ── SX1262 radio object — owned by core 1 ────────────────────────────────
// Waveshare Pico-LoRa-SX1262: CS=3, DIO1=20, RST=15, BUSY=2
SX1262 radio = new Module(3, 20, 15, 2, SPI1);

// ── Core 1: LoRa ──────────────────────────────────────────────────────────
void setup1() {
    rp2040.fifo.pop();          // wait for core 0 to complete
    SPI1.setRX(12);
    SPI1.setTX(11);
    SPI1.setSCK(10);
    SPI1.begin(false);          // RadioLib manages CS
    initLoRa();                 // init radio + apply config settings
}

void loop1() {
    loopLoRa();
    sleep_ms(50);
}

// ── Serial CLI ────────────────────────────────────────────────────────────
static void printHelp() {
    Serial.println("─────────────────────────────────────────────");
    Serial.println(" SEN66 Serial CLI");
    Serial.println("  help              — this message");
    Serial.println("  status            — system status");
    Serial.println("  read              — single sensor reading");
    Serial.println("  stream            — toggle CSV serial stream");
    Serial.println("  debug <n>         — 0=off 1=web 2=sensor 4=lora");
    Serial.println(" ── LoRa ──────────────────────────────────────");
    Serial.println("  lora status       — radio state, last ACK");
    Serial.println("  lora send         — TX one sensor packet now");
    Serial.println("  lora stream on    — auto TX at interval");
    Serial.println("  lora stream off   — stop auto TX");
    Serial.println("  lora test <freq> <msg>  — one-shot test TX");
    Serial.println("    e.g.  lora test 868.75 hello world");
    Serial.println("  lora freq <mhz>   — set frequency");
    Serial.println("  lora sf <7-12>    — set spreading factor");
    Serial.println("  lora power <dbm>  — set TX power (2-22)");
    Serial.println("─────────────────────────────────────────────");
}

static void printStatus() {
    FSInfo fs; LittleFS.info(fs);
    Serial.println("─────────────────────────────────────────────");
    Serial.printf(" WiFi:    %s  IP: %s\n", staMode ? "STA" : "AP",
        staMode ? WiFi.localIP().toString().c_str()
                : WiFi.softAPIP().toString().c_str());
    Serial.printf(" Uptime:  %lus\n", millis()/1000);
    Serial.printf(" Flash:   %u/%u bytes\n", fs.usedBytes, fs.totalBytes);
    Serial.printf(" Log:     %lu rows\n", logRowCount);
    Serial.printf(" Debug:   %u\n", debugLevel);
    Serial.println(" ── LoRa ──────────────────────────────────────");
    Serial.printf(" State:   %s\n", loraStateStr());
    Serial.printf(" Node:    %s\n", cfg.node_id);
    Serial.printf(" Freq:    %.3f MHz\n", cfg.lora_freq);
    Serial.printf(" SF/BW:   SF%u / %.0f kHz\n", cfg.lora_sf, cfg.lora_bw);
    Serial.printf(" Power:   %d dBm\n", cfg.lora_power);
    Serial.printf(" Stream:  %s  interval=%us\n",
        loraStreaming ? "ON" : "OFF", cfg.lora_interval_s);
    Serial.printf(" TX count:%lu\n", loraTxCount);
    if (loraStreaming) {
        uint32_t intervalMs = max((uint32_t)cfg.lora_interval_s * 1000UL, 30000UL);
        uint32_t elapsed = millis() - loraLastTxMs;
        if (elapsed < intervalMs)
            Serial.printf(" Next TX: %lus\n", (intervalMs - elapsed) / 1000);
        else
            Serial.println(" Next TX: due now");
    }
    if (loraLastAck.length() > 0)
        Serial.printf(" Last ACK: \"%s\"  RSSI=%d  SNR=%.1f\n",
            loraLastAck.c_str(), loraLastRssi, loraLastSnr);
    Serial.println("─────────────────────────────────────────────");
}

static void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    String cmdL = cmd; cmdL.toLowerCase();

    if      (cmdL == "help")   printHelp();
    else if (cmdL == "status") printStatus();
    else if (cmdL == "read") {
        if (latest.valid) printReadable(latest);
        else Serial.println("[cli] No reading yet");
    }
    else if (cmdL == "stream") {
        streamEnabled = !streamEnabled;
        Serial.printf("[cli] Serial stream %s\n", streamEnabled ? "ON" : "OFF");
        if (streamEnabled)
            Serial.println("ts_ms,pm1.0,pm2.5,pm4.0,pm10,rh_%,temp_C,voc_idx,nox_idx,co2_ppm");
    }
    else if (cmdL.startsWith("debug")) {
        String arg = cmdL.substring(5); arg.trim();
        if (arg.length() > 0) {
            debugLevel = (uint8_t)arg.toInt();
            Serial.printf("[cli] Debug → %u\n", debugLevel);
        }
    }
    // ── LoRa commands ─────────────────────────────────────────────────────
    else if (cmdL == "lora status") {
        printStatus();  // status includes LoRa section
    }
    else if (cmdL == "lora send") {
        loraSendSensor();
        Serial.println("[cli] Sensor TX queued");
    }
    else if (cmdL == "lora stream on") {
        loraStreaming = true;
        Serial.printf("[cli] LoRa stream ON — interval %us\n", cfg.lora_interval_s);
    }
    else if (cmdL == "lora stream off") {
        loraStreaming = false;
        Serial.println("[cli] LoRa stream OFF");
    }
    else if (cmdL.startsWith("lora test ")) {
        // Format: lora test <freq> <message...>
        String rest = cmd.substring(10);  // preserve case for message
        int sp = rest.indexOf(' ');
        if (sp < 0) { Serial.println("[cli] Usage: lora test <freq> <message>"); return; }
        float freq = rest.substring(0, sp).toFloat();
        String msg = rest.substring(sp + 1);
        if (freq < 100.0 || freq > 1000.0) {
            Serial.println("[cli] Invalid frequency"); return;
        }
        Serial.printf("[cli] Test TX @ %.3f MHz: %s\n", freq, msg.c_str());
        loraSendTest(freq, msg);
    }
    else if (cmdL == "lora freq") {
        Serial.printf("[lora] Current frequency: %.3f MHz\n", cfg.lora_freq);
    }
    else if (cmdL.startsWith("lora freq ")) {
        float freq = cmdL.substring(10).toFloat();
        if (freq < 100.0 || freq > 1000.0) { Serial.println("[cli] Invalid frequency"); return; }
        cfg.lora_freq = freq;
        loraSetFreq(freq);
    }
    else if (cmdL.startsWith("lora sf ")) {
        uint8_t sf = (uint8_t)cmdL.substring(8).toInt();
        cfg.lora_sf = sf;
        loraSetSF(sf);
    }
    else if (cmdL.startsWith("lora power ")) {
        int8_t pwr = (int8_t)cmdL.substring(11).toInt();
        cfg.lora_power = pwr;
        loraSetPower(pwr);
    }
    else if (cmd.length() > 0)
        Serial.printf("[cli] Unknown: '%s'  (try 'help')\n", cmd.c_str());
}

// ── Core 0 ────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n═══ SEN66 Monitor Phase 2 + LoRa ═══");

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

    // ── LoRa stream countdown ticker — one dot per second on a single line,
    // only while streaming. Line resets (newline) right before next TX.
    static uint32_t lastCountdownMs = 0;
    static bool     countdownLineOpen = false;
    if (loraStreaming && millis() - lastCountdownMs >= 1000) {
        lastCountdownMs = millis();
        uint32_t intervalMs = max((uint32_t)cfg.lora_interval_s * 1000UL, 30000UL);
        uint32_t elapsed = millis() - loraLastTxMs;
        if (elapsed < intervalMs) {
            if (!countdownLineOpen) {
                Serial.print("[lora] next TX ");
                countdownLineOpen = true;
            }
            Serial.print(".");
        }
    }
    if (countdownLineOpen && (!loraStreaming ||
        millis() - loraLastTxMs >= max((uint32_t)cfg.lora_interval_s * 1000UL, 30000UL))) {
        Serial.println();
        countdownLineOpen = false;
    }
}