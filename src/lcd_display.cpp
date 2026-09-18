// ═══════════════════════════════════════════════════════════════════════════
// lcd_display.cpp — Waveshare Pico-LCD-1.14 UI for SEN66 monitor
//
// 240×135 ST7789V, SPI1 shared with SX1262 LoRa (CS=GP9 vs CS=GP3).
// spi1_mutex serialises access between core 0 (LCD) and core 1 (LoRa).
//
// Button B (GP17) only — all other pins conflict with LoRa shield.
// Single press: wake screen (if off) or advance to next page.
// Auto-off: backlight off after LCD_TIMEOUT_MS of inactivity.
// ═══════════════════════════════════════════════════════════════════════════

#include "lcd_display.h"
#include "lcd_driver.h"
#include "lcd_buttons.h"
#include "sensor.h"
#include "config.h"
#include "globals.h"
#include "lora_wan.h"
#include <WiFi.h>
#include <pico/mutex.h>
#include <math.h>

extern bool      staMode;
extern uint32_t  logRowCount;

// ── SPI1 mutex — shared with LoRa on core 1 ──────────────────────────────
mutex_t spi1_mutex;

// ── Timing ────────────────────────────────────────────────────────────────
#define LCD_TIMEOUT_MS   20000  // backlight off after 20s inactivity
#define BTN_DEBOUNCE_MS  200
#define UPDATE_INTERVAL  200    // 5Hz refresh

static LcdPage  currentPage      = LcdPage::SENSOR;
static bool     pageDirty        = true;
static bool     screenOn         = true;
static uint32_t lastActivityMs   = 0;
static uint32_t lastUpdateMs     = 0;
static uint32_t lastBtnMs        = 0;

// ── History ring buffer ───────────────────────────────────────────────────
static float hist_temp[LCD_HISTORY] = {};
static float hist_rh  [LCD_HISTORY] = {};
static float hist_co2 [LCD_HISTORY] = {};
static float hist_voc [LCD_HISTORY] = {};
static float hist_nox [LCD_HISTORY] = {};
static float hist_pm25[LCD_HISTORY] = {};
static uint8_t histHead  = 0;
static uint8_t histCount = 0;

void lcd_push_reading(float temp, float rh, float co2,
                      float voc,  float nox, float pm25) {
    hist_temp[histHead] = temp;
    hist_rh  [histHead] = rh;
    hist_co2 [histHead] = co2;
    hist_voc [histHead] = voc;
    hist_nox [histHead] = nox;
    hist_pm25[histHead] = pm25;
    histHead = (histHead + 1) % LCD_HISTORY;
    if (histCount < LCD_HISTORY) histCount++;
    if (screenOn) pageDirty = true;
}

// ── Colour scheme ─────────────────────────────────────────────────────────
#define BG       COL_BLACK
#define FG       COL_WHITE
#define ACCENT   COL_AMBER
#define DIM      COL_GRAY
#define OK_COL   COL_GREEN
#define WARN_COL RGB565(255,180,0)
#define BAD_COL  COL_RED

// ── Flush a tile row (with mutex) ─────────────────────────────────────────
static void flush_tile(uint16_t tile_y) {
    mutex_enter_blocking(&spi1_mutex);
    lcd_flush_tile(tile_y);
    mutex_exit(&spi1_mutex);
}

// ── Fill entire tile row with one colour ─────────────────────────────────
static void fill_tile(uint16_t tile_y, uint16_t colour) {
    for (uint32_t i = 0; i < LCD_W * TILE_H; i++) _tile_buf[i] = colour;
    flush_tile(tile_y);
}

// ── Draw one text line into a tile row ───────────────────────────────────
static void draw_line(uint16_t tile_y, const char* label,
                      const char* value, uint16_t val_col, uint8_t scale=2) {
    for (uint32_t i = 0; i < LCD_W * TILE_H; i++) _tile_buf[i] = BG;
    tile_str(2,  2, label, DIM,     BG, scale);
    tile_str(90, 2, value, val_col, BG, scale);
    flush_tile(tile_y);
}

// ── Header tile (y=0, height=TILE_H=20) ──────────────────────────────────
static void draw_header(const char* title) {
    uint16_t hbg = RGB565(30,20,0);
    for (uint32_t i = 0; i < LCD_W * TILE_H; i++) _tile_buf[i] = hbg;
    tile_str(4, 2, title, ACCENT, hbg, 2);
    // Page indicator dots at right edge
    uint8_t total = (uint8_t)LcdPage::PAGE_COUNT - 1;
    uint8_t cur   = (currentPage == LcdPage::OFF) ? 0
                    : (uint8_t)currentPage - 1;
    uint16_t dx = LCD_W - total * 8 - 4;
    for (uint8_t i = 0; i < total; i++)
        tile_fill_rect(dx + i*8, 7, 5, 5, (i==cur) ? ACCENT : DIM);
    flush_tile(0);
}

// ── Get ordered history (oldest → newest) ────────────────────────────────
static uint8_t get_history(const float* src, float* dst) {
    if (histCount == 0) return 0;
    uint8_t start = (histCount < LCD_HISTORY) ? 0 : histHead;
    for (uint8_t i = 0; i < histCount; i++)
        dst[i] = src[(start + i) % LCD_HISTORY];
    return histCount;
}

// ── Full-screen graph (y=20..134, 115px tall, 240px wide) ────────────────
// Renders into a static pixel buffer then flushes tile by tile.
// All data samples are mapped into the full 115px height so the line
// is continuous and not fragmented across tile boundaries.
#define GRAPH_Y0    20      // first pixel row of graph area (below header)
#define GRAPH_H    115      // 135 - 20 = 115px
#define GRAPH_W    LCD_W    // 240px

static uint16_t graph_buf[GRAPH_W * GRAPH_H];  // ~55 KB — fits in RAM

static void render_graph_full(const float* src, const char* label,
                               const char* unit, uint16_t colour) {
    static float ordered[LCD_HISTORY];
    uint8_t cnt = get_history(src, ordered);

    // Clear buffer to background
    for (uint32_t i = 0; i < GRAPH_W * GRAPH_H; i++) graph_buf[i] = BG;

    float yMin = 0, yMax = 1;  // defaults if no data

    if (cnt >= 2) {
        // Find min/max with small padding
        float mn = ordered[0], mx = ordered[0];
        for (uint8_t i = 1; i < cnt; i++) {
            if (ordered[i] < mn) mn = ordered[i];
            if (ordered[i] > mx) mx = ordered[i];
        }
        float pad = (mx - mn) * 0.08f;
        if (pad < 0.01f) pad = 0.5f;
        yMin = mn - pad; yMax = mx + pad;
        float yRange = yMax - yMin;

        // Subtle gridlines at 25/50/75% height
        for (uint8_t g = 1; g <= 3; g++) {
            uint16_t gy = (uint16_t)(GRAPH_H * g / 4);
            for (uint16_t x = 0; x < GRAPH_W; x++)
                graph_buf[gy * GRAPH_W + x] = RGB565(30,30,30);
        }

        // Pre-compute pixel coords for all samples
        static int16_t px[LCD_HISTORY], py[LCD_HISTORY];
        for (uint8_t i = 0; i < cnt; i++) {
            px[i] = (int16_t)((uint32_t)i * (GRAPH_W - 1) / (cnt - 1));
            float norm = (ordered[i] - yMin) / yRange;
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;
            py[i] = (int16_t)(GRAPH_H - 1 - norm * (GRAPH_H - 1));
        }

        // Bresenham line between each consecutive pair of points
        for (uint8_t i = 1; i < cnt; i++) {
            int16_t x0 = px[i-1], y0 = py[i-1];
            int16_t x1 = px[i],   y1 = py[i];
            int16_t dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int16_t err = dx + dy;
            while (true) {
                // Plot pixel (and one below for thickness)
                if (x0 >= 0 && x0 < GRAPH_W && y0 >= 0 && y0 < GRAPH_H) {
                    graph_buf[y0 * GRAPH_W + x0] = colour;
                    if (y0 + 1 < GRAPH_H)
                        graph_buf[(y0+1) * GRAPH_W + x0] = colour;
                }
                if (x0 == x1 && y0 == y1) break;
                int16_t e2 = 2 * err;
                if (e2 >= dy) { err += dy; x0 += sx; }
                if (e2 <= dx) { err += dx; y0 += sy; }
            }
        }
    }

    // Flush graph_buf to LCD tile by tile, overlaying Y-axis labels
    // Labels: max at top tile, mid at middle tile, min at bottom tile
    // Y-axis labels at tile-aligned rows: top=0, middle=60, bottom=100
    // (GRAPH_H=115, TILE_H=20 — valid tile starts: 0,20,40,60,80,100)
    float labelVals[3]    = { yMax, (yMax+yMin)/2.0f, yMin };
    uint16_t labelRows[3] = { 0, 60, 100 };

    // X-axis: time span label bottom-right
    // histCount readings at cfg.interval_s seconds each
    uint32_t spanS = (uint32_t)histCount * cfg.interval_s;
    char timeLabel[16];
    if (spanS < 120)       snprintf(timeLabel, sizeof(timeLabel), "%lus", spanS);
    else if (spanS < 3600) snprintf(timeLabel, sizeof(timeLabel), "%lum",  spanS/60);
    else                   snprintf(timeLabel, sizeof(timeLabel), "%.1fh", spanS/3600.0f);

    for (uint16_t ty = 0; ty < GRAPH_H; ty += TILE_H) {
        uint16_t rows = ((ty + TILE_H) <= GRAPH_H) ? TILE_H : (GRAPH_H - ty);
        memcpy(_tile_buf, &graph_buf[ty * GRAPH_W],
               rows * GRAPH_W * sizeof(uint16_t));
        if (rows < TILE_H)
            memset(_tile_buf + rows * GRAPH_W, 0,
                   (TILE_H - rows) * GRAPH_W * sizeof(uint16_t));

        // Overlay Y-axis label if this tile row matches a label row
        for (uint8_t li = 0; li < 3; li++) {
            if (ty == labelRows[li]) {
                char lbuf[12];
                float v = labelVals[li];
                // Auto-format: no decimal if large, 1 decimal if medium, 2 if small
                if (fabsf(v) >= 100.0f)     snprintf(lbuf, sizeof(lbuf), "%.0f", v);
                else if (fabsf(v) >= 10.0f) snprintf(lbuf, sizeof(lbuf), "%.1f", v);
                else                         snprintf(lbuf, sizeof(lbuf), "%.2f", v);
                // Draw semi-transparent dark bg behind label (first 38px wide)
                for (uint16_t row = 0; row < rows; row++)
                    for (uint16_t col = 0; col < 38; col++)
                        _tile_buf[row * GRAPH_W + col] = RGB565(15,15,15);
                tile_str(1, 2, lbuf, DIM, RGB565(15,15,15), 1);
            }
        }

        // Overlay time span label on bottom tile, right-aligned
        if (ty + TILE_H >= GRAPH_H) {
            // Dark bg strip at bottom-right
            uint16_t lw = strlen(timeLabel) * 6 + 2;  // approx 6px per char at scale 1
            for (uint16_t row = rows > 4 ? rows-8 : 0; row < rows; row++)
                for (uint16_t col = GRAPH_W - lw - 2; col < GRAPH_W; col++)
                    _tile_buf[row * GRAPH_W + col] = RGB565(15,15,15);
            tile_str(GRAPH_W - lw - 1, rows > 8 ? rows-8 : 0,
                     timeLabel, DIM, RGB565(15,15,15), 1);
        }

        flush_tile(GRAPH_Y0 + ty);
    }
}

// ── Screen off ────────────────────────────────────────────────────────────
static void screen_off() {
    screenOn    = false;
    currentPage = LcdPage::OFF;
    mutex_enter_blocking(&spi1_mutex);
    digitalWrite(LCD_BL, LOW);
    mutex_exit(&spi1_mutex);
}

static void screen_on_page(LcdPage p) {
    screenOn       = true;
    currentPage    = p;
    pageDirty      = true;
    lastActivityMs = millis();
    digitalWrite(LCD_BL, HIGH);
}

// ── Page renderers ────────────────────────────────────────────────────────
static void render_sensor() {
    draw_header("SEN66");
    char buf[32];

    snprintf(buf, sizeof(buf), "%.1fC  %.0f%%", latest.temp, latest.rh);
    draw_line(20, "T/RH", buf, FG);

    uint16_t co2col = latest.co2 == 0  ? DIM :
                      latest.co2 < 800  ? OK_COL :
                      latest.co2 < 1500 ? WARN_COL : BAD_COL;
    snprintf(buf, sizeof(buf), "%u ppm", latest.co2);
    draw_line(40, "CO2", buf, co2col);

    uint16_t voccol = latest.voc < 100 ? OK_COL :
                      latest.voc < 200  ? WARN_COL : BAD_COL;
    snprintf(buf, sizeof(buf), "%.0f", latest.voc);
    draw_line(60, "VOC", buf, voccol);

    snprintf(buf, sizeof(buf), "%.0f", latest.nox);
    draw_line(80, "NOx", buf, FG);

    if (latest.pm_sat) {
        draw_line(100, "PM2.5", "SAT", WARN_COL);
    } else {
        snprintf(buf, sizeof(buf), "%.1f ug", latest.pm25);
        draw_line(100, "PM2.5", buf, FG);
    }

    snprintf(buf, sizeof(buf), "up %lus", millis()/1000);
    for (uint32_t i = 0; i < LCD_W * TILE_H; i++) _tile_buf[i] = BG;
    tile_str(2, 2, buf, DIM, BG, 1);
    flush_tile(120);
}

static void render_wifi() {
    draw_header("WiFi");
    char buf[32];

    draw_line(20, "mode", staMode ? "STA" : "AP",
              staMode ? OK_COL : WARN_COL);

    String ip = staMode ? WiFi.localIP().toString() : String("192.168.4.1");
    draw_line(40, "IP", ip.c_str(), FG, 1);

    draw_line(60, "SSID", staMode ? cfg.wifi_ssid : cfg.ap_ssid, FG, 1);

    snprintf(buf, sizeof(buf), "%lu rows", logRowCount);
    draw_line(80, "log", buf, FG);

    snprintf(buf, sizeof(buf), "%lus", millis()/1000);
    draw_line(100, "up", buf, DIM);

    snprintf(buf, sizeof(buf), "%us intv", cfg.interval_s);
    draw_line(120, "sens", buf, DIM);
}

static void render_lora() {
    draw_header("LoRa");
    char buf[32];

    draw_line(20, "state", loraStateStr(),
        loraState == LoRaState::TX_OK ? OK_COL :
        loraState == LoRaState::ERROR ? BAD_COL : FG);

    snprintf(buf, sizeof(buf), "%.3f MHz", cfg.lora_freq);
    draw_line(40, "freq", buf, FG);

    snprintf(buf, sizeof(buf), "SF%u %ddBm", cfg.lora_sf, cfg.lora_power);
    draw_line(60, "radio", buf, FG);

    snprintf(buf, sizeof(buf), "%lu", loraTxCount);
    draw_line(80, "TX#", buf, FG);

    draw_line(100, "stream", loraStreaming ? "ON" : "OFF",
              loraStreaming ? OK_COL : DIM);

    draw_line(120, "ack",
              loraLastAck.length() > 0 ? loraLastAck.c_str() : "none",
              loraLastAck.length() > 0 ? OK_COL : DIM, 1);
}

// ── Main update ───────────────────────────────────────────────────────────
void lcd_display_update() {
    uint32_t now = millis();

    // Auto-off after timeout
    if (screenOn && (now - lastActivityMs >= LCD_TIMEOUT_MS)) {
        screen_off();
        return;
    }
    if (!screenOn) return;

    if (!pageDirty && (now - lastUpdateMs < UPDATE_INTERVAL)) return;
    lastUpdateMs = now;
    pageDirty    = false;

    switch (currentPage) {
        case LcdPage::SENSOR:     render_sensor(); break;
        case LcdPage::WIFI:       render_wifi();   break;
        case LcdPage::LORA:       render_lora();   break;
        case LcdPage::GRAPH_TEMP:
            draw_header("Temp C");
            render_graph_full(hist_temp, "T",    "C",  COL_ORANGE);  break;
        case LcdPage::GRAPH_RH:
            draw_header("Humidity %");
            render_graph_full(hist_rh,   "RH",   "%",  COL_BLUE);    break;
        case LcdPage::GRAPH_CO2:
            draw_header("CO2 ppm");
            render_graph_full(hist_co2,  "CO2",  "p",  OK_COL);      break;
        case LcdPage::GRAPH_VOC:
            draw_header("VOC index");
            render_graph_full(hist_voc,  "VOC",  "",   WARN_COL);    break;
        case LcdPage::GRAPH_NOX:
            draw_header("NOx index");
            render_graph_full(hist_nox,  "NOx",  "",   BAD_COL);     break;
        case LcdPage::GRAPH_PM25:
            draw_header("PM2.5 ug/m3");
            render_graph_full(hist_pm25, "PM25", "u",  COL_BLUE);    break;
        case LcdPage::OFF: break;
        default: break;
    }
}

// ── Button handler ────────────────────────────────────────────────────────
void lcd_display_handle_buttons() {
    buttons_poll();
    const ButtonState& b = buttons();

    if (b.b_edge && millis() - lastBtnMs > BTN_DEBOUNCE_MS) {
        lastBtnMs      = millis();
        lastActivityMs = millis();

        if (!screenOn) {
            // Wake screen — go to SENSOR page
            screen_on_page(LcdPage::SENSOR);
            return;
        }
        // Advance to next page — include OFF so user can blank screen manually
        uint8_t next = (uint8_t)currentPage + 1;
        if (next >= (uint8_t)LcdPage::PAGE_COUNT) next = 1; // wrap to SENSOR
        if ((LcdPage)next == LcdPage::OFF) {
            screen_off();
        } else {
            screen_on_page((LcdPage)next);
        }
    }
}

// ── Init ──────────────────────────────────────────────────────────────────
void lcd_display_init() {
    mutex_init(&spi1_mutex);

    pinMode(BTN_B, INPUT_PULLUP);

    lcd_init();

    mutex_enter_blocking(&spi1_mutex);
    lcd_fill(BG);
    mutex_exit(&spi1_mutex);

    lastActivityMs = millis();
    pageDirty      = true;
    screenOn       = true;
    Serial.println("[lcd] Initialised — B=next page, auto-off 20s");
}