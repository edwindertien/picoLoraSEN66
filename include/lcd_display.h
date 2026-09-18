#pragma once
#include <Arduino.h>

// ── Pages ─────────────────────────────────────────────────────────────────
// Button B (GP17) cycles forward through pages
enum class LcdPage : uint8_t {
    OFF = 0,        // backlight off
    SENSOR,         // all sensor values
    WIFI,           // WiFi + log status
    LORA,           // LoRa radio status
    GRAPH_TEMP,     // temperature sparkline
    GRAPH_RH,       // humidity sparkline
    GRAPH_CO2,      // CO2 sparkline
    GRAPH_VOC,      // VOC index sparkline
    GRAPH_NOX,      // NOx index sparkline
    GRAPH_PM25,     // PM2.5 sparkline
    PAGE_COUNT      // sentinel — keep last
};

// ── Ring buffer for on-device graph data ─────────────────────────────────
// Separate from the web /history endpoint — kept small for RAM
#define LCD_HISTORY  60    // 60 readings = 10 min at 10s interval

void lcd_display_init();                // call from setup() on core 0
void lcd_display_update();             // call from loop() on core 0 ~5Hz
void lcd_display_handle_buttons();     // call from loop() on core 0
void lcd_push_reading(float temp, float rh, float co2,
                      float voc, float nox, float pm25); // call each sensor read
