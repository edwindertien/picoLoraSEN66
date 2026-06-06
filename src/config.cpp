#include "config.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

Config cfg;

void loadConfig() {
    if (!LittleFS.exists("/config.json")) {
        Serial.println("[cfg] No config.json — using defaults");
        return;
    }
    File f = LittleFS.open("/config.json", "r");
    if (!f) return;

    JsonDocument doc;
    if (deserializeJson(doc, f) != DeserializationError::Ok) {
        Serial.println("[cfg] Parse error — using defaults");
        f.close(); return;
    }
    f.close();

    strlcpy(cfg.wifi_ssid, doc["wifi_ssid"] | "",              sizeof(cfg.wifi_ssid));
    strlcpy(cfg.wifi_pass, doc["wifi_pass"] | "",              sizeof(cfg.wifi_pass));
    strlcpy(cfg.ap_ssid,   doc["ap_ssid"]   | "SEN66-Monitor", sizeof(cfg.ap_ssid));
    strlcpy(cfg.ap_pass,   doc["ap_pass"]   | "sen66pass",     sizeof(cfg.ap_pass));
    cfg.interval_s = doc["interval_s"] | 10;
    cfg.log_max    = doc["log_max"]    | 1000;

    strlcpy(cfg.lora_mode,    doc["lora_mode"]    | "disabled",                        sizeof(cfg.lora_mode));
    strlcpy(cfg.lora_dev_eui, doc["lora_dev_eui"] | "0000000000000000",                sizeof(cfg.lora_dev_eui));
    strlcpy(cfg.lora_app_eui, doc["lora_app_eui"] | "0000000000000000",                sizeof(cfg.lora_app_eui));
    strlcpy(cfg.lora_app_key, doc["lora_app_key"] | "00000000000000000000000000000000", sizeof(cfg.lora_app_key));
    cfg.lora_interval_s = doc["lora_interval_s"] | 60;

    Serial.printf("[cfg] Loaded: ssid='%s' interval=%ds lora=%s\n",
        cfg.wifi_ssid, cfg.interval_s, cfg.lora_mode);
}

bool saveConfig() {
    File f = LittleFS.open("/config.json", "w");
    if (!f) return false;

    JsonDocument doc;
    doc["wifi_ssid"]       = cfg.wifi_ssid;
    doc["wifi_pass"]       = cfg.wifi_pass;
    doc["ap_ssid"]         = cfg.ap_ssid;
    doc["ap_pass"]         = cfg.ap_pass;
    doc["interval_s"]      = cfg.interval_s;
    doc["log_max"]         = cfg.log_max;
    doc["lora_mode"]       = cfg.lora_mode;
    doc["lora_dev_eui"]    = cfg.lora_dev_eui;
    doc["lora_app_eui"]    = cfg.lora_app_eui;
    doc["lora_app_key"]    = cfg.lora_app_key;
    doc["lora_interval_s"] = cfg.lora_interval_s;
    serializeJson(doc, f);
    f.close();
    return true;
}