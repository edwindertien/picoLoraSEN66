#pragma once
// ============================================================
//  lcd_buttons.h — Button B always on; full joystick only
//  while the LCD menu is open (radio confirmed suspended)
//
//  GP2/GP3/GP15/GP20 (UP/PRESS/A/RIGHT) are SX1262 BUSY/CS/RST/
//  DIO1. Reading them while the radio is ACTIVE would interfere
//  with it exactly as the old always-on joystick code did.
//  They are only ever configured as inputs between
//  menu_buttons_claim() and menu_buttons_release() — i.e. only
//  during the same window where the radio is already fully
//  suspended and its pins inert (see lora_wan.h suspend API).
//
//  GP17 (Button B) is the one pin with no radio conflict, so it
//  stays permanently interrupt-driven and toggles the menu itself.
// ============================================================

#include <Arduino.h>

#define BTN_UP     2
#define BTN_DOWN  18
#define BTN_LEFT  16
#define BTN_RIGHT 20
#define BTN_PRESS  3
#define BTN_A     15
#define BTN_B     17

struct ButtonState {
    bool up, down, left, right, press, a, b;
    bool up_edge, down_edge, left_edge, right_edge, press_edge, a_edge, b_edge;
};

static ButtonState _btn_state = {};
static ButtonState _btn_prev  = {};

// B is interrupt-driven, always active — same reasoning as before:
// polling once per loop() can miss a short tap if loop() is briefly busy.
static volatile bool     _btn_b_flag    = false;
static volatile uint32_t _btn_b_last_ms = 0;

static void _btn_b_isr() {
    uint32_t now = millis();
    if (now - _btn_b_last_ms < 50) return;  // debounce in the ISR itself
    _btn_b_last_ms = now;
    _btn_b_flag    = true;
}

inline void buttons_init() {
    pinMode(BTN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_B), _btn_b_isr, FALLING);
}

// Call once the radio has confirmed suspended (lora_is_suspended()==true),
// before reading the joystick for the first time this menu session.
inline void menu_buttons_claim() {
    pinMode(BTN_UP,    INPUT_PULLUP);
    pinMode(BTN_DOWN,  INPUT_PULLUP);
    pinMode(BTN_LEFT,  INPUT_PULLUP);
    pinMode(BTN_RIGHT, INPUT_PULLUP);
    pinMode(BTN_PRESS, INPUT_PULLUP);
    pinMode(BTN_A,     INPUT_PULLUP);
}

// Call when the menu closes, BEFORE lora_request_resume() — releases the
// joystick pins back to a safe, inert state before the radio reclaims them.
inline void menu_buttons_release() {
    pinMode(BTN_UP,    INPUT);
    pinMode(BTN_DOWN,  INPUT);
    pinMode(BTN_LEFT,  INPUT);
    pinMode(BTN_RIGHT, INPUT);
    pinMode(BTN_PRESS, INPUT);
    pinMode(BTN_A,     INPUT);
}

// poll_full=true also reads the joystick — only valid while the menu is
// open (pins claimed). poll_full=false (menu closed) only updates B,
// safe to call anytime regardless of radio state.
inline void buttons_poll(bool poll_full = false) {
    _btn_prev = _btn_state;

    _btn_state.b_edge = false;
    if (_btn_b_flag) { _btn_b_flag = false; _btn_state.b_edge = true; }
    _btn_state.b = !digitalRead(BTN_B);

    if (poll_full) {
        _btn_state.up    = !digitalRead(BTN_UP);
        _btn_state.down  = !digitalRead(BTN_DOWN);
        _btn_state.left  = !digitalRead(BTN_LEFT);
        _btn_state.right = !digitalRead(BTN_RIGHT);
        _btn_state.press = !digitalRead(BTN_PRESS);
        _btn_state.a     = !digitalRead(BTN_A);

        _btn_state.up_edge    = _btn_state.up    && !_btn_prev.up;
        _btn_state.down_edge  = _btn_state.down  && !_btn_prev.down;
        _btn_state.left_edge  = _btn_state.left  && !_btn_prev.left;
        _btn_state.right_edge = _btn_state.right && !_btn_prev.right;
        _btn_state.press_edge = _btn_state.press && !_btn_prev.press;
        _btn_state.a_edge     = _btn_state.a     && !_btn_prev.a;
    } else {
        _btn_state.up = _btn_state.down = _btn_state.left =
        _btn_state.right = _btn_state.press = _btn_state.a = false;
        _btn_state.up_edge = _btn_state.down_edge = _btn_state.left_edge =
        _btn_state.right_edge = _btn_state.press_edge = _btn_state.a_edge = false;
    }
}

inline const ButtonState& buttons() { return _btn_state; }