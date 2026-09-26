#pragma once
// ============================================================
//  lcd_buttons.h — Button B (GP17) only, interrupt-driven
//
//  All other LCD button/joystick pins conflict with the
//  Waveshare Pico-LoRa-SX1262 shield:
//    GP2  = SX1262 BUSY  (was BTN_UP)
//    GP3  = SX1262 CS    (was BTN_PRESS)
//    GP15 = SX1262 RST   (was BTN_A)
//    GP20 = SX1262 DIO1  (was BTN_RIGHT)
//  Only GP17 (Button B) is free.
//
//  Interrupt-driven rather than polled: polling once per loop()
//  can miss a short tap if loop() is momentarily busy (web server,
//  a page render's SPI transfer, etc). The ISR latches the press
//  the instant it happens, independent of loop() timing.
// ============================================================

#include <Arduino.h>

#define BTN_B  17

static volatile bool     _btn_b_flag    = false;
static volatile uint32_t _btn_b_last_ms = 0;

static void _btn_b_isr() {
    uint32_t now = millis();
    if (now - _btn_b_last_ms < 50) return;  // debounce inside the ISR itself
    _btn_b_last_ms = now;
    _btn_b_flag    = true;
}

struct ButtonState {
    bool b;
    bool b_edge;
};

static ButtonState _btn_state = {};

inline void buttons_init() {
    pinMode(BTN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_B), _btn_b_isr, FALLING);
}

inline void buttons_poll() {
    _btn_state.b_edge = false;
    if (_btn_b_flag) {
        _btn_b_flag = false;
        _btn_state.b_edge = true;
    }
    _btn_state.b = !digitalRead(BTN_B);
}

inline const ButtonState& buttons() { return _btn_state; }