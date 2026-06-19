#pragma once
#include <Arduino.h>

struct Config {
    char     wifi_ssid[64]  = "";
    char     wifi_pass[64]  = "";
    char     ap_ssid[32]    = "SEN66-Monitor";
    char     ap_pass[32]    = "sen66pass";
    uint16_t interval_s     = 10;    // logging interval in seconds
    uint16_t log_max        = 1000;  // max CSV rows before ring-wrap

    // ── Sensor ────────────────────────────────────────────────────────────
    bool     fan_cleaning    = true;   // run 10s fan clean on boot

    // ── Node ──────────────────────────────────────────────────────────────
    char     node_id[16]     = "SEN66-01";

    // ── LoRa ──────────────────────────────────────────────────────────────
    float    lora_freq       = 868.1f;  // MHz
    uint8_t  lora_sf         = 9;       // spreading factor 7-12
    float    lora_bw         = 125.0f;  // kHz
    int8_t   lora_power      = 14;      // dBm (2-22)
    uint16_t lora_interval_s = 60;      // stream interval (min 30s)
    bool     lora_stream     = false;   // auto-stream on boot
    // LoRaWAN (Phase B — future)
    char     lora_dev_eui[17]= "0000000000000000";
    char     lora_app_eui[17]= "0000000000000000";
    char     lora_app_key[33]= "00000000000000000000000000000000";
};

extern Config cfg;

void loadConfig();
bool saveConfig();