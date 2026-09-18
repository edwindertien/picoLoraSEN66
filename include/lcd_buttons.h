#pragma once
// ============================================================
//  lcd_buttons.h  –  Joystick + buttons on Pico-LCD-1.14
//
//  All inputs are active-low (pulled up internally).
//
//  Joystick: UP=GP2  DOWN=GP18  LEFT=GP16  RIGHT=GP20  PRESS=GP3
//  Buttons:  A=GP15  B=GP17
// ============================================================

#include <Arduino.h>

#define BTN_UP    2
#define BTN_DOWN  18
#define BTN_LEFT  16
#define BTN_RIGHT 20
#define BTN_PRESS  3
#define BTN_A     15
#define BTN_B     17

struct ButtonState {
    bool up, down, left, right, press, a, b;
    // Edge detection: true only on the first poll after press
    bool up_edge, down_edge, left_edge, right_edge, press_edge, a_edge, b_edge;
};

static ButtonState _btn_state = {};
static ButtonState _btn_prev  = {};

inline void buttons_init() {
    int pins[] = {BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_PRESS, BTN_A, BTN_B};
    for (int p : pins) {
        pinMode(p, INPUT_PULLUP);
    }
}

// Call from loop() — updates state and edge detection
inline void buttons_poll() {
    _btn_prev = _btn_state;

    _btn_state.up    = !digitalRead(BTN_UP);
    _btn_state.down  = !digitalRead(BTN_DOWN);
    _btn_state.left  = !digitalRead(BTN_LEFT);
    _btn_state.right = !digitalRead(BTN_RIGHT);
    _btn_state.press = !digitalRead(BTN_PRESS);
    _btn_state.a     = !digitalRead(BTN_A);
    _btn_state.b     = !digitalRead(BTN_B);

    // Edge = pressed now but not last poll
    _btn_state.up_edge    = _btn_state.up    && !_btn_prev.up;
    _btn_state.down_edge  = _btn_state.down  && !_btn_prev.down;
    _btn_state.left_edge  = _btn_state.left  && !_btn_prev.left;
    _btn_state.right_edge = _btn_state.right && !_btn_prev.right;
    _btn_state.press_edge = _btn_state.press && !_btn_prev.press;
    _btn_state.a_edge     = _btn_state.a     && !_btn_prev.a;
    _btn_state.b_edge     = _btn_state.b     && !_btn_prev.b;
}

inline const ButtonState& buttons() { return _btn_state; }