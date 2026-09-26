// ═══════════════════════════════════════════════════════════════════════════
// lcd_display.cpp — Waveshare Pico-LCD-1.14 UI for SEN66 monitor
//
// STEP 3 of the re-integration plan: LoRa is back. Coarse-grained bus
// handoff — screen_on_page() suspends the radio and fully releases SPI1
// before waking the display; screen_off() releases the LCD's hold on the
// bus and resumes the radio. Only one side ever touches SPI1 at a time.
//
// 240×135 ST7789V. lcd_init() (in lcd_driver.h) owns SPI1 entirely via its
// own SPI1.begin() call, with a real hardware RST pulse on GP12.
//
// Button B (GP17) is the only pin with no radio conflict — it always
// toggles the menu on/off. The rest of the joystick (UP/DOWN/LEFT/RIGHT/
// PRESS/A on GP2/GP18/GP16/GP20/GP3/GP15) is claimed only while the menu
// is open, during the same window the radio is already fully suspended —
// see menu_buttons_claim()/menu_buttons_release() in lcd_buttons.h and
// screen_on_page()/screen_off() below. LEFT/RIGHT switch pages; on the
// SETTINGS page, UP/DOWN move the row selection and PRESS/A activates it.
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

// ── SPI1 mutex — kept in place for when core 1/radio is reintroduced.
// Harmless for now since core 1 is fully idle in this step.
mutex_t spi1_mutex;

// ── Timing ────────────────────────────────────────────────────────────────
#define BTN_DEBOUNCE_MS  200
#define UPDATE_INTERVAL  200    // 5Hz refresh
#define LCD_TIMEOUT_MS   20000  // backlight off after 20s inactivity

static LcdPage  currentPage      = LcdPage::SENSOR;
static bool     pageDirty        = true;
static bool     screenOn         = false;  // device boots as a LoRa node, screen off
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

// ── Flush a tile row — simple blocking mutex, matching the original
// confirmed-working version. No claim/release, no hardware reset, no
// per-call SPI reconfiguration — just the mutex around the raw flush.
static void flush_tile(uint16_t tile_y) {
    mutex_enter_blocking(&spi1_mutex);
    lcd_flush_tile(tile_y);
    mutex_exit(&spi1_mutex);
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
// Renders into a static pixel buffer then flushes tile by tile, so the
// line is continuous across tile boundaries (Bresenham between samples).
#define GRAPH_Y0    20
#define GRAPH_H    115
#define GRAPH_W    LCD_W

static uint16_t graph_buf[GRAPH_W * GRAPH_H];

static void render_graph_full(const float* src, const char* label,
                               const char* unit, uint16_t colour) {
    static float ordered[LCD_HISTORY];
    uint8_t cnt = get_history(src, ordered);

    for (uint32_t i = 0; i < GRAPH_W * GRAPH_H; i++) graph_buf[i] = BG;

    float yMin = 0, yMax = 1;

    if (cnt >= 2) {
        float mn = ordered[0], mx = ordered[0];
        for (uint8_t i = 1; i < cnt; i++) {
            if (ordered[i] < mn) mn = ordered[i];
            if (ordered[i] > mx) mx = ordered[i];
        }
        float pad = (mx - mn) * 0.08f;
        if (pad < 0.01f) pad = 0.5f;
        yMin = mn - pad; yMax = mx + pad;
        float yRange = yMax - yMin;

        for (uint8_t g = 1; g <= 3; g++) {
            uint16_t gy = (uint16_t)(GRAPH_H * g / 4);
            for (uint16_t x = 0; x < GRAPH_W; x++)
                graph_buf[gy * GRAPH_W + x] = RGB565(30,30,30);
        }

        static int16_t px[LCD_HISTORY], py[LCD_HISTORY];
        for (uint8_t i = 0; i < cnt; i++) {
            px[i] = (int16_t)((uint32_t)i * (GRAPH_W - 1) / (cnt - 1));
            float norm = (ordered[i] - yMin) / yRange;
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;
            py[i] = (int16_t)(GRAPH_H - 1 - norm * (GRAPH_H - 1));
        }

        for (uint8_t i = 1; i < cnt; i++) {
            int16_t x0 = px[i-1], y0 = py[i-1];
            int16_t x1 = px[i],   y1 = py[i];
            int16_t dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
            int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
            int16_t err = dx + dy;
            while (true) {
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

    float labelVals[3]    = { yMax, (yMax+yMin)/2.0f, yMin };
    uint16_t labelRows[3] = { 0, 60, 100 };

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

        for (uint8_t li = 0; li < 3; li++) {
            if (ty == labelRows[li]) {
                char lbuf[12];
                float v = labelVals[li];
                if (fabsf(v) >= 100.0f)     snprintf(lbuf, sizeof(lbuf), "%.0f", v);
                else if (fabsf(v) >= 10.0f) snprintf(lbuf, sizeof(lbuf), "%.1f", v);
                else                         snprintf(lbuf, sizeof(lbuf), "%.2f", v);
                for (uint16_t row = 0; row < rows; row++)
                    for (uint16_t col = 0; col < 38; col++)
                        _tile_buf[row * GRAPH_W + col] = RGB565(15,15,15);
                tile_str(1, 2, lbuf, DIM, RGB565(15,15,15), 1);
            }
        }

        if (ty + TILE_H >= GRAPH_H) {
            uint16_t lw = strlen(timeLabel) * 6 + 2;
            for (uint16_t row = rows > 4 ? rows-8 : 0; row < rows; row++)
                for (uint16_t col = GRAPH_W - lw - 2; col < GRAPH_W; col++)
                    _tile_buf[row * GRAPH_W + col] = RGB565(15,15,15);
            tile_str(GRAPH_W - lw - 1, rows > 8 ? rows-8 : 0,
                     timeLabel, DIM, RGB565(15,15,15), 1);
        }

        flush_tile(GRAPH_Y0 + ty);
    }
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

    draw_line(20, "state",
        lora_is_suspended() ? "suspend" : loraStateStr(),
        lora_is_suspended() ? DIM :
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

// ── Settings page — toggles/values only, no text entry ──────────────────
// UP/DOWN moves the highlighted row, LEFT/RIGHT adjusts a numeric row,
// PRESS or A activates/toggles the highlighted row. Wired from
// lcd_display_handle_buttons() only while currentPage == SETTINGS.
#define SETTINGS_ROW_COUNT 8
static uint8_t settingsSelected = 0;
static uint8_t settingsScroll   = 0;

static const char* settings_label(uint8_t idx) {
    static const char* labels[SETTINGS_ROW_COUNT] = {
        "LoRa stream", "LoRa send now", "LoRa freq", "LoRa SF",
        "LoRa power",  "Fan clean",     "WiFi mode", "Reconnect"
    };
    return labels[idx];
}

static void settings_get_value(uint8_t idx, char* buf, size_t buflen, uint16_t* col) {
    switch (idx) {
        case 0: snprintf(buf, buflen, "%s", loraStreaming ? "ON" : "OFF");
                *col = loraStreaming ? OK_COL : DIM; break;
        case 1: snprintf(buf, buflen, "[press]"); *col = FG; break;
        case 2: snprintf(buf, buflen, "%.1f MHz", cfg.lora_freq); *col = FG; break;
        case 3: snprintf(buf, buflen, "SF%u", cfg.lora_sf); *col = FG; break;
        case 4: snprintf(buf, buflen, "%d dBm", cfg.lora_power); *col = FG; break;
        case 5: snprintf(buf, buflen, "%s", cfg.fan_cleaning ? "ON" : "OFF");
                *col = cfg.fan_cleaning ? OK_COL : DIM; break;
        case 6: snprintf(buf, buflen, "%s", cfg.wifi_force_ap ? "AP" : "STA");
                *col = FG; break;
        case 7: snprintf(buf, buflen, "[press]"); *col = FG; break;
        default: buf[0] = 0; *col = FG; break;
    }
}

// PRESS/A — toggle or trigger. Rows 2/3/4 (numeric) are LEFT/RIGHT only.
static void settings_activate(uint8_t idx) {
    switch (idx) {
        case 0: loraStreaming = !loraStreaming; break;
        case 1: loraSendSensor(); break;
        case 5: cfg.fan_cleaning  = !cfg.fan_cleaning;  saveConfig(); break;
        case 6: cfg.wifi_force_ap = !cfg.wifi_force_ap; saveConfig(); break;
        case 7:
            Serial.println("[menu] Rebooting to apply network settings...");
            delay(200);
            rp2040.reboot();
            break;
        default: break;
    }
}

// LEFT/RIGHT — adjust a numeric row. dir is -1 or +1.
static void settings_adjust(uint8_t idx, int dir) {
    switch (idx) {
        case 2: {
            cfg.lora_freq += dir * 0.1f;
            loraSetFreq(cfg.lora_freq);
            saveConfig();
        } break;
        case 3: {
            int sf = (int)cfg.lora_sf + dir;
            if (sf < 7) sf = 7; if (sf > 12) sf = 12;
            cfg.lora_sf = (uint8_t)sf;
            loraSetSF(cfg.lora_sf);
            saveConfig();
        } break;
        case 4: {
            int p = (int)cfg.lora_power + dir;
            if (p < 2) p = 2; if (p > 22) p = 22;
            cfg.lora_power = (int8_t)p;
            loraSetPower(cfg.lora_power);
            saveConfig();
        } break;
        default: break;  // toggle/action rows don't respond to LEFT/RIGHT
    }
}

static void render_settings() {
    draw_header("Settings");

    // Keep the highlighted row within the 6-row visible window
    if (settingsSelected < settingsScroll) settingsScroll = settingsSelected;
    if (settingsSelected > settingsScroll + 5) settingsScroll = settingsSelected - 5;

    for (uint8_t i = 0; i < 6; i++) {
        uint8_t idx = settingsScroll + i;
        uint16_t tileY = 20 + i * TILE_H;
        bool sel = (idx == settingsSelected) && (idx < SETTINGS_ROW_COUNT);
        uint16_t rowBg = sel ? RGB565(25,20,10) : BG;

        for (uint32_t p = 0; p < LCD_W * TILE_H; p++) _tile_buf[p] = rowBg;
        if (idx < SETTINGS_ROW_COUNT) {
            char valbuf[24]; uint16_t valcol;
            settings_get_value(idx, valbuf, sizeof(valbuf), &valcol);
            tile_str(2,   2, settings_label(idx), sel ? ACCENT : DIM, rowBg, 1);
            tile_str(150, 2, valbuf, valcol, rowBg, 1);
        }
        flush_tile(tileY);
    }
}

// ── Screen off — full bus release ────────────────────────────────────────
// Releases the SPI1 peripheral and tri-states every pin the LCD drives,
// including GP12 (LCD_RST) which is shared with the SX1262's MISO once
// the radio is reintroduced in Step 3. Setting pins to INPUT rather than
// just stopping SPI1 means nothing is left actively driving a shared
// line while another user of the bus might need it.
static void screen_off() {
    if (DBG_BUS) Serial.printf("[lcd] screen_off() at t=%lu\n", millis());
    screenOn    = false;
    currentPage = LcdPage::OFF;
    digitalWrite(LCD_BL, LOW);

    SPI1.end();
    pinMode(LCD_CS,   INPUT);
    pinMode(LCD_DC,   INPUT);
    pinMode(LCD_RST,  INPUT);
    pinMode(LCD_SCK,  INPUT);
    pinMode(LCD_MOSI, INPUT);

    menu_buttons_release();  // joystick pins back to inert before radio reclaims them
    lora_request_resume();
}

// ── Screen on — full bus reclaim + reinit ────────────────────────────────
// Waking from OFF re-runs the complete lcd_init() sequence rather than
// just re-enabling SPI1: every pin was released as an INPUT, so they all
// need to be reclaimed as OUTPUTs, SPI1 needs a fresh begin(), and the
// ST7789 controller needs its full reset+config sequence again — not just
// pin reconfiguration. This is the same lcd_init() called once at boot.
static void screen_on_page(LcdPage p) {
    if (!screenOn) {
        if (DBG_BUS) Serial.printf("[lcd] screen_on_page() waking at t=%lu\n", millis());
        // Ask the radio to suspend and fully release SPI1 before we touch
        // any pins. This only happens on a button press, so a short
        // blocking wait is fine — worst case is a TX + ACK window
        // finishing up, typically well under the timeout below.
        lora_request_suspend();
        uint32_t t0 = millis();
        while (!lora_is_suspended() && millis() - t0 < 3000) {
            delay(5);
        }
        if (!lora_is_suspended()) {
            // Kept unconditional — a real problem worth always seeing.
            Serial.println("[lcd] WARNING: radio didn't suspend in time — proceeding anyway");
        }
        menu_buttons_claim();  // joystick safe now — radio confirmed off these pins
        settingsSelected = 0;
        settingsScroll   = 0;

        lcd_init();
        mutex_enter_blocking(&spi1_mutex);
        lcd_fill(BG);
        mutex_exit(&spi1_mutex);
        if (DBG_BUS) Serial.println("[lcd] Wake complete");
    }

    screenOn       = true;
    currentPage    = p;
    pageDirty      = true;
    lastActivityMs = millis();
    digitalWrite(LCD_BL, HIGH);
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
        case LcdPage::SETTINGS:   render_settings(); break;
        default: break;
    }
}

// ── Button handler ────────────────────────────────────────────────────────
// Button B: the one pin with no radio conflict — always active, always
// just toggles the menu itself (open at SENSOR / close), regardless of
// which page is currently showing.
// Joystick UP/DOWN: page navigation, only read while the menu is open
// (buttons_poll(screenOn) only claims/reads the joystick pins in that
// window — see lcd_buttons.h). LEFT/RIGHT and PRESS/A are read and
// available for future settings pages (value adjust, select/activate)
// but not yet wired to anything.
void lcd_display_handle_buttons() {
    buttons_poll(screenOn);
    const ButtonState& b = buttons();

    if (b.b_edge && millis() - lastBtnMs > BTN_DEBOUNCE_MS) {
        lastBtnMs = millis();
        if (!screenOn) screen_on_page(LcdPage::SENSOR);
        else            screen_off();
        return;
    }

    if (!screenOn) return;  // joystick only matters while the menu is open

    if (currentPage == LcdPage::SETTINGS) {
        // UP/DOWN: move the highlighted row. LEFT/RIGHT: adjust a numeric
        // row. PRESS/A: activate/toggle the highlighted row.
        if (b.up_edge || b.down_edge) {
            lastActivityMs = millis();
            if (b.down_edge && settingsSelected < SETTINGS_ROW_COUNT - 1) settingsSelected++;
            else if (b.up_edge && settingsSelected > 0)                  settingsSelected--;
            pageDirty = true;
        }
        if (b.left_edge)  { lastActivityMs = millis(); settings_adjust(settingsSelected, -1); pageDirty = true; }
        if (b.right_edge) { lastActivityMs = millis(); settings_adjust(settingsSelected, +1); pageDirty = true; }
        if (b.press_edge || b.a_edge) {
            lastActivityMs = millis();
            settings_activate(settingsSelected);
            pageDirty = true;
        }
        return;
    }

    // Every other page: LEFT/RIGHT switches between top-level pages.
    if (b.left_edge || b.right_edge) {
        lastActivityMs = millis();
        uint8_t count = (uint8_t)LcdPage::PAGE_COUNT - 1;  // exclude OFF
        uint8_t cur   = (uint8_t)currentPage - 1;          // 0-based, real pages only
        cur = b.right_edge ? (cur + 1) % count : (cur + count - 1) % count;
        screen_on_page((LcdPage)(cur + 1));
    }
}

// ── Init ──────────────────────────────────────────────────────────────────
// Screen starts OFF: the device boots as a LoRa node, backlight dark.
// We deliberately do NOT call lcd_init() here — screen_on_page() already
// runs the full init sequence on first wake, so touching the hardware now
// would just be immediately torn down again. Skipping it means the radio
// gets a completely untouched SPI1 peripheral as its first-ever user.
void lcd_display_init() {
    mutex_init(&spi1_mutex);
    buttons_init();  // interrupt-driven — see lcd_buttons.h

    pinMode(LCD_CS,   INPUT);
    pinMode(LCD_DC,   INPUT);
    pinMode(LCD_RST,  INPUT);
    pinMode(LCD_SCK,  INPUT);
    pinMode(LCD_MOSI, INPUT);
    pinMode(LCD_BL,   OUTPUT);
    digitalWrite(LCD_BL, LOW);

    screenOn       = false;
    currentPage    = LcdPage::OFF;
    pageDirty      = false;
    lastActivityMs = millis();
    Serial.println("[lcd] Initialised — screen OFF, press B to wake");
}