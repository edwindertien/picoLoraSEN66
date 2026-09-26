// ═══════════════════════════════════════════════════════════════════════════
// SEN66 Monitor — Step 1 of LCD/LoRa re-integration plan
// Core 0: WiFi, web, sensor, serial CLI, LCD (always on, no bus sharing)
// Core 1: COMPLETELY IDLE — radio not touched at all in this step.
//
// Plan: (1) confirm LCD works reliably and permanently on its own — this
// step. (2) add screen timeout + bus release/reclaim, LCD-only, no radio
// yet. (3) reintroduce the radio carefully, one change at a time.
//
// LoRa CLI/status code from the working build is preserved below as
// comments marking exactly where it goes back in during Step 3.
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
#include "lcd_display.h"

uint8_t  debugLevel    = 0;
bool     streamEnabled = false;
bool     staMode       = false;
uint32_t lastLogTime   = 0;

// ── Core 1: fully idle — radio not reintroduced yet (Step 1) ─────────────
void setup1() { }
void loop1()  { sleep_ms(1000); }

// ── Serial CLI ────────────────────────────────────────────────────────────
static void printHelp() {
    Serial.println("─────────────────────────────────────────────");
    Serial.println(" SEN66 Serial CLI  [Step 1: LCD-only, no radio]");
    Serial.println("  help              — this message");
    Serial.println("  status            — system status");
    Serial.println("  read              — single sensor reading");
    Serial.println("  stream            — toggle CSV serial stream");
    Serial.println("  debug <n>         — 0=off 1=web 2=sensor 4=lora");
    // Step 3 will restore:
    //  lora status / lora send / lora stream on|off / lora test <freq> <msg>
    //  lora freq / lora sf / lora power
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
    // Step 3 will restore the LoRa status block here (state, freq, SF/BW,
    // power, TX count, stream/countdown, last ACK) and the LCD flush stats.
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
    // Step 3 will restore all "lora ..." commands here, unchanged from
    // the working build: status/send/stream on/off/test/freq/sf/power.
    else if (cmd.length() > 0)
        Serial.printf("[cli] Unknown: '%s'  (try 'help')\n", cmd.c_str());
}

// ── Core 0 ────────────────────────────────────────────────────────────────
extern mutex_t spi1_mutex;

void setup() {
    mutex_init(&spi1_mutex);

    Serial.begin(115200);
    delay(2000);
    Serial.println("\n═══ SEN66 Monitor — Step 1: LCD-only ═══");

    if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
    Serial.println("[fs] mounted");

    loadConfig();
    initLog();
    startWiFi();
    setupWebServer();
    initSensor(cfg.fan_cleaning);
    lcd_display_init();

    printHelp();
    Serial.println("[boot] Complete — core 1 idle (Step 1: no radio)");
    // Step 3 will re-add: rp2040.fifo.push(0xAA55AA55) to release core 1
    // once setup1() there is reinstated to actually init the radio.
}

void loop() {
    server.handleClient();
    handleSerial();
    lcd_display_handle_buttons();
    lcd_display_update();

    static uint32_t lastReadMs = 0;
    if (millis() - lastReadMs >= 1000) {
        lastReadMs = millis();
        readSensor(latest);
        if (latest.valid)
            lcd_push_reading(latest.temp, latest.rh, (float)latest.co2,
                             latest.voc, latest.nox,
                             latest.pm_sat ? 0.0f : latest.pm25);
        if (streamEnabled && latest.valid)
            printMeasurement(latest);
    }

    if (latest.valid &&
        millis() - lastLogTime >= (uint32_t)cfg.interval_s * 1000) {
        lastLogTime = millis();
        appendLog(latest);
    }

    // Step 3 will restore the LoRa stream countdown ticker here.
}