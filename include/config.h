#pragma once
#include <Arduino.h>

struct Config {
    // ── WiFi ──────────────────────────────────────────────────────────────
    char     wifi_ssid[64]  = "";
    char     wifi_pass[64]  = "";
    char     ap_ssid[32]    = "SEN66-Monitor";
    char     ap_pass[32]    = "sen66pass";
    uint16_t interval_s     = 10;
    uint16_t log_max        = 1000;

    // ── LoRa ──────────────────────────────────────────────────────────────
    // lora_mode: "disabled" | "lorawan" | "raw"
    char     lora_mode[12]      = "raw";
    char     lora_dev_eui[17]   = "0000000000000000";
    char     lora_app_eui[17]   = "0000000000000000";
    char     lora_app_key[33]   = "00000000000000000000000000000000";
    uint16_t lora_interval_s    = 60;
};

extern Config cfg;

void loadConfig();
bool saveConfig();