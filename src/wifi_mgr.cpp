#include "wifi_mgr.h"
#include "config.h"
#include "globals.h"
#include <WiFi.h>

void startWiFi() {
    if (strlen(cfg.wifi_ssid) > 0) {
        // Only disconnect if we're actually already associated — this is
        // for the "net reconnect" case where begin() would otherwise be
        // called on top of an existing connection. Calling disconnect()
        // unconditionally, even on the very first boot before WiFi has
        // ever been started, was found to break the initial STA attempt
        // entirely (fell straight back to AP every time).
        if (WiFi.status() == WL_CONNECTED) {
            WiFi.disconnect(true);
            delay(100);
        }

        Serial.printf("[wifi] Trying STA: %s\n", cfg.wifi_ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
        uint32_t t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
            delay(250); Serial.print(".");
        }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
            staMode = true;
            // WL_CONNECTED can report true a moment before DHCP has
            // actually finished — wait briefly for a real (non-zero)
            // address before trusting it.
            uint32_t dt = millis();
            while (WiFi.localIP() == IPAddress(0,0,0,0) && millis() - dt < 3000) {
                delay(100);
            }
            Serial.printf("[wifi] Connected! IP: %s\n",
                WiFi.localIP().toString().c_str());
            return;
        }
        Serial.println("[wifi] STA failed — falling back to AP");
    }

    // AP fallback — softAPConfig MUST come before softAP to start DHCP server
    WiFi.mode(WIFI_AP);
    IPAddress apIP(192, 168, 4, 1);
    IPAddress apGW(192, 168, 4, 1);
    IPAddress apSN(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apGW, apSN);
    delay(100);
    WiFi.softAP(cfg.ap_ssid, cfg.ap_pass);
    delay(500);
    staMode = false;
    Serial.printf("[wifi] AP mode: SSID=%s  IP=%s\n",
        cfg.ap_ssid, WiFi.softAPIP().toString().c_str());
}