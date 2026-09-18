#pragma once
// ============================================================
//  lcd_driver.h  –  Waveshare Pico-LCD-1.14 low-level driver
//
//  ST7789V, 240x135, RGB565, SPI1
//  Pins (hard-wired on board):
//    GP10 = SCK    GP11 = MOSI   GP9  = CS
//    GP8  = DC     GP12 = RST    GP13 = BL
//
//  Uses a 240×TILE_H tile buffer (9.6 KB) for partial updates.
//  Full screen = 8 tiles; individual regions can be refreshed
//  cheaply by only pushing dirty tiles.
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <hardware/gpio.h>

// ---- Pins --------------------------------------------------
#define LCD_SCK   10
#define LCD_MOSI  11
#define LCD_CS     9
#define LCD_DC     8
#define LCD_RST   12
#define LCD_BL    13

// ---- Screen dimensions ------------------------------------
#define LCD_W   240
#define LCD_H   135

// ---- Tile buffer -------------------------------------------
#define TILE_H   20           // rows per tile
#define TILE_BUF_BYTES  (LCD_W * TILE_H * 2)
static uint16_t _tile_buf[LCD_W * TILE_H];

// ---- RGB565 colour helpers ---------------------------------
#define RGB565(r,g,b) ((uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)))

// Colour palette
#define COL_BLACK    RGB565(  0,   0,   0)
#define COL_WHITE    RGB565(255, 255, 255)
#define COL_AMBER    RGB565(255, 160,   0)   // drawbar active
#define COL_AMBER_DIM RGB565( 60,  38,   0)  // drawbar empty
#define COL_GREEN    RGB565(  0, 200,  60)   // MIDI activity
#define COL_ORANGE   RGB565(255,  80,   0)   // percussion
#define COL_BLUE     RGB565( 40, 120, 255)   // volume
#define COL_GRAY     RGB565( 80,  80,  80)   // labels
#define COL_DARKGRAY RGB565( 30,  30,  30)   // backgrounds
#define COL_RED      RGB565(220,  40,  40)   // click

// ---- SPI helper --------------------------------------------
static SPISettings _lcd_spi_settings(62500000, MSBFIRST, SPI_MODE0);

inline void _lcd_cmd(uint8_t cmd) {
    digitalWrite(LCD_DC, LOW);
    digitalWrite(LCD_CS, LOW);
    SPI1.transfer(cmd);
    digitalWrite(LCD_CS, HIGH);
}

inline void _lcd_data8(uint8_t d) {
    digitalWrite(LCD_DC, HIGH);
    digitalWrite(LCD_CS, LOW);
    SPI1.transfer(d);
    digitalWrite(LCD_CS, HIGH);
}

inline void _lcd_data16(uint16_t d) {
    digitalWrite(LCD_DC, HIGH);
    digitalWrite(LCD_CS, LOW);
    SPI1.transfer16(d);
    digitalWrite(LCD_CS, HIGH);
}

// ---- Set address window ------------------------------------
// ST7789 on Waveshare Pico-LCD-1.14: physical 240x135 sits at
// offset (40,53) within the 240x320 controller framebuffer (landscape)
#define LCD_X_OFFSET  40
#define LCD_Y_OFFSET  53

inline void _lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    x0 += LCD_X_OFFSET; x1 += LCD_X_OFFSET;
    y0 += LCD_Y_OFFSET; y1 += LCD_Y_OFFSET;
    _lcd_cmd(0x2A);
    digitalWrite(LCD_DC, HIGH); digitalWrite(LCD_CS, LOW);
    SPI1.transfer(x0 >> 8); SPI1.transfer(x0 & 0xFF);
    SPI1.transfer(x1 >> 8); SPI1.transfer(x1 & 0xFF);
    digitalWrite(LCD_CS, HIGH);
    _lcd_cmd(0x2B);
    digitalWrite(LCD_DC, HIGH); digitalWrite(LCD_CS, LOW);
    SPI1.transfer(y0 >> 8); SPI1.transfer(y0 & 0xFF);
    SPI1.transfer(y1 >> 8); SPI1.transfer(y1 & 0xFF);
    digitalWrite(LCD_CS, HIGH);
    _lcd_cmd(0x2C);
}

// ---- Push tile buffer to screen ----------------------------
// Pushes TILE_H rows starting at y_start
inline void lcd_push_tile(uint16_t y_start) {
    uint16_t y_end = y_start + TILE_H - 1;
    if (y_end >= LCD_H) y_end = LCD_H - 1;
    _lcd_set_window(0, y_start, LCD_W - 1, y_end);
    digitalWrite(LCD_DC, HIGH);
    digitalWrite(LCD_CS, LOW);
    // Transfer as bytes (big-endian swap needed: RGB565 high byte first)
    uint32_t n = (uint32_t)(y_end - y_start + 1) * LCD_W;
    for (uint32_t i = 0; i < n; i++) {
        uint16_t px = _tile_buf[i];
        SPI1.transfer(px >> 8);
        SPI1.transfer(px & 0xFF);
    }
    digitalWrite(LCD_CS, HIGH);
}

// ---- Tile buffer pixel write -------------------------------
inline void tile_pixel(uint16_t x, uint16_t tile_y, uint16_t colour) {
    _tile_buf[tile_y * LCD_W + x] = colour;
}

// Fill a rect in the tile buffer (tile_y is local y within tile)
inline void tile_fill_rect(uint16_t x, uint16_t tile_y, uint16_t w, uint16_t h, uint16_t colour) {
    for (uint16_t row = tile_y; row < tile_y + h; row++) {
        for (uint16_t col = x; col < x + w; col++) {
            _tile_buf[row * LCD_W + col] = colour;
        }
    }
}

// Draw a single char (5x7 font, scaled) into tile buffer
// Only supports printable ASCII 32-127
// Font stored as 5 columns of 7 bits
static const uint8_t _font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x40,0x3C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x10,0x08,0x08,0x10,0x08}, // '~'
};

// Draw char at tile-local y, scale 1 or 2
inline void tile_char(uint16_t x, uint16_t ty, char c, uint16_t fg, uint16_t bg, uint8_t scale=2) {
    if (c < 32 || c > 126) c = '?';
    const uint8_t* glyph = _font5x7[c - 32];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            uint16_t colour = (bits >> row) & 1 ? fg : bg;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++)
                    tile_pixel(x + col*scale + sx, ty + row*scale + sy, colour);
        }
    }
}

inline void tile_str(uint16_t x, uint16_t ty, const char* s, uint16_t fg, uint16_t bg, uint8_t scale=2) {
    uint16_t cx = x;
    while (*s) {
        tile_char(cx, ty, *s++, fg, bg, scale);
        cx += (5 * scale) + scale;  // char width + 1px gap
    }
}

// ---- Initialisation ----------------------------------------
inline void lcd_init() {
    // Pins
    pinMode(LCD_CS,  OUTPUT); digitalWrite(LCD_CS,  HIGH);
    pinMode(LCD_DC,  OUTPUT); digitalWrite(LCD_DC,  HIGH);
    pinMode(LCD_RST, OUTPUT); digitalWrite(LCD_RST, HIGH);
    pinMode(LCD_BL,  OUTPUT); digitalWrite(LCD_BL,  HIGH);

    // SPI1: GP10=SCK, GP11=MOSI, no MISO (display only)
    SPI1.setTX(LCD_MOSI);
    SPI1.setSCK(LCD_SCK);
    SPI1.setRX(PIN_SPI1_MISO);  // default MISO, not connected, harmless
    SPI1.begin();

    // Reset
    digitalWrite(LCD_RST, LOW);  delay(10);
    digitalWrite(LCD_RST, HIGH); delay(120);

    SPI1.beginTransaction(_lcd_spi_settings);

    // ST7789 init sequence
    _lcd_cmd(0x01); delay(150);   // Software reset
    _lcd_cmd(0x11); delay(500);   // Sleep out
    _lcd_cmd(0x3A); _lcd_data8(0x05); // 16-bit colour
    _lcd_cmd(0x36); _lcd_data8(0x70); // Memory data access: landscape, RGB
    _lcd_cmd(0xB2); // Porch setting
    _lcd_data8(0x0C); _lcd_data8(0x0C); _lcd_data8(0x00);
    _lcd_data8(0x33); _lcd_data8(0x33);
    _lcd_cmd(0xB7); _lcd_data8(0x35); // Gate control
    _lcd_cmd(0xBB); _lcd_data8(0x19); // VCOM setting
    _lcd_cmd(0xC0); _lcd_data8(0x2C); // LCM control
    _lcd_cmd(0xC2); _lcd_data8(0x01); // VDV/VRH enable
    _lcd_cmd(0xC3); _lcd_data8(0x12); // VRH set
    _lcd_cmd(0xC4); _lcd_data8(0x20); // VDV set
    _lcd_cmd(0xC6); _lcd_data8(0x0F); // Frame rate (60Hz)
    _lcd_cmd(0xD0); _lcd_data8(0xA4); _lcd_data8(0xA1); // Power control
    _lcd_cmd(0x29); delay(20);         // Display on

    SPI1.endTransaction();
}

// Fill entire screen with one colour (no tile buffer needed)
inline void lcd_fill(uint16_t colour) {
    SPI1.beginTransaction(_lcd_spi_settings);
    _lcd_set_window(0, 0, LCD_W-1, LCD_H-1);
    digitalWrite(LCD_DC, HIGH);
    digitalWrite(LCD_CS, LOW);
    uint32_t total = (uint32_t)LCD_W * LCD_H;
    uint8_t hi = colour >> 8, lo = colour & 0xFF;
    for (uint32_t i = 0; i < total; i++) {
        SPI1.transfer(hi);
        SPI1.transfer(lo);
    }
    digitalWrite(LCD_CS, HIGH);
    SPI1.endTransaction();
}

// Push one tile (y_start must be multiple of TILE_H)
inline void lcd_flush_tile(uint16_t y_start) {
    SPI1.beginTransaction(_lcd_spi_settings);
    lcd_push_tile(y_start);
    SPI1.endTransaction();
}