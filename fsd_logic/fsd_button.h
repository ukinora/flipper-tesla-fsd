#pragma once
/*
 * fsd_button.h — "was that a press, a hold, or a bounce?"
 *
 * The module is meant to accept a generic Bluetooth button as an input, the way
 * TSL does. Everything about the RADIO side of that is device-specific and
 * unmeasurable until a button is in hand — which service, which characteristic,
 * what the report bytes mean, whether it bonds. None of that is here.
 *
 * What IS device-independent is the timing: a contact bounces, a hold is not a
 * tap, and a button wedged under a seat must not look like a press held for an
 * hour. That is this file, and it is testable today.
 *
 * The caller says "button N is down / up, at time T". This says what that was.
 *
 * DOUBLE-PRESS IS OPT-IN, PER BUTTON
 * ----------------------------------
 * This file used to refuse double-press outright, and the reasoning was sound:
 * supporting it costs every SHORT the width of the window, because a tap cannot
 * be reported until a second one is known not to be coming. For "step the speed
 * profile" that latency is the whole feel of the control.
 *
 * The reasoning still holds — so the COST is opt-in rather than the feature
 * being absent. fsd_btn_set_double() turns it on for one button, and only that
 * button pays. A button with nothing mapped to a double press behaves exactly
 * as it did: SHORT on release, no delay, byte for byte the same path.
 *
 * Turn it on only where a second action is actually mapped. "It might be useful
 * later" is not a reason to make every tap wait.
 *
 * (T2's gesture detector implements its own double-press, on a car button we do
 * not own and cannot change. Different problem: there the two taps ARE the
 * signal, and there is no latency budget to protect.)
 *
 * WHAT A PRESS DOES TODAY
 * -----------------------
 * Nothing. The only action wired to a button is the speed-profile step, and
 * that is locked behind fsd_sp_encoding_ok() and FsdSpInputs.tx_armed, neither
 * of which can be satisfied before a capture. So presses are classified,
 * counted and published — which is what lets a button be paired and proven
 * end-to-end long before anything is armed.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* How many buttons are tracked at once.
 *
 * 5 single-button remotes, one logical button each — ble_central.cpp maps slot
 * index straight to button index, so BLE_CENTRAL_MAX_BUTTONS and this must
 * agree. The cost is one FsdBtnState per slot, a couple of dozen bytes.
 *
 * 🔴 The number comes from the RADIO, not from this file. The Arduino core's
 * prebuilt controller allows six simultaneous links and the phone holds one.
 * It was 8 for a day (2026-08-18) and that boot-looped the board — the
 * static_assert that now catches it lives in esp32/.firmware/ble_central.h,
 * because that is where the budget is visible. */
#ifndef FSD_BTN_MAX
#define FSD_BTN_MAX 5
#endif

/* Shorter than this is contact bounce or a dropped report, not a press. */
#define FSD_BTN_DEBOUNCE_MS 30u

/* Held this long becomes a LONG, and it fires WHILE STILL HELD rather than on
 * release: a hold that only reported at the end would feel like a lag, and the
 * user has no way to know it registered. */
#define FSD_BTN_LONG_MS 600u

/* Held longer than this is not a person. A button under a seat cushion, a
 * peripheral that stopped sending its release report, a stuck contact — all
 * look identical from here, and all must stop producing events. Cleared only
 * by an actual release. */
#define FSD_BTN_STUCK_MS 10000u

/* How long a tap waits for its twin, on buttons where double-press is enabled.
 * Shorter than FSD_BTN_LONG_MS on purpose: a hold must still be a hold, and the
 * wait a person feels on a single tap is exactly this number. */
#define FSD_BTN_DOUBLE_MS 300u

typedef enum {
    FSD_BTN_EV_NONE = 0,
    FSD_BTN_EV_SHORT, // pressed and released inside the long threshold
    FSD_BTN_EV_LONG,  // still held at the long threshold; fires once
    FSD_BTN_EV_STUCK, // held past all reason; fires once, then silence
    /* Two qualifying taps inside FSD_BTN_DOUBLE_MS. Only on buttons where
     * fsd_btn_set_double() turned it on — appended last so the existing values
     * do not shift. */
    FSD_BTN_EV_DOUBLE,
} FsdBtnEvent;

/* What a button is physically able to tell us.
 *
 * MEASURED, NOT CHOSEN. A TSL remote was put on the bench on 2026-08-18 and it
 * reports only that a press happened — never that it ended. That is almost
 * certainly why that product offers 1-click and 2-click but no long press: you
 * cannot time a hold you are never told the end of.
 *
 * 🔴 THE DEFAULT IS EVENT BECAUSE THE MISTAKES ARE NOT SYMMETRIC.
 *
 * | 실제 | 우리가 EVENT 로 봄 | 우리가 LEVEL 로 봄 |
 * |---|---|---|
 * | LEVEL | 길게만 못 쓴다 | 정상 |
 * | EVENT | 정상 | 🔴 0.6 s 에 유령 LONG, 10 s 에 STUCK — **그 버튼이 잠긴다** |
 *
 * STUCK is cleared only by a release, and an event button has none to give, so
 * the bottom-right cell is permanent until reboot. Start EVENT; promote to
 * LEVEL only after an actual release has been seen. */
typedef enum {
    FSD_BTN_KIND_EVENT = 0, // "눌렸다" 만 온다. 1회·2회만 가능
    FSD_BTN_KIND_LEVEL,     // 눌림과 뗌이 온다. 길게도 가능
} FsdBtnKind;

typedef struct {
    FsdBtnKind kind;     // 0 = EVENT, so a zeroed struct is the safe one
    bool down;
    uint32_t down_ms;
    bool long_fired;
    bool stuck;
    bool want_double;    // opt-in; false means the old, undelayed path
    bool tap_pending;    // a SHORT is being withheld while its twin might arrive
    uint32_t pending_ms; // when that tap was released
    uint16_t shorts;
    uint16_t longs;
    uint16_t doubles;
    uint16_t bounces; // presses too brief to count — a link-quality signal
    uint16_t last_hold_ms;
} FsdBtnState;

typedef struct {
    FsdBtnState btn[FSD_BTN_MAX];
} FsdButtons;

/** Clear every button to released. */
void fsd_btn_init(FsdButtons* b);

/** What this button is able to report. EVENT until told otherwise.
 *
 *  Changing it **resets that button** — down, hold, stuck and any withheld tap
 *  are cleared. The registration probe flips this at runtime, and a stale
 *  `down` left behind by the old mode would wedge the button the instant it is
 *  promoted. Counters and the double-press setting survive. */
void fsd_btn_set_kind(FsdButtons* b, uint8_t idx, FsdBtnKind kind);
FsdBtnKind fsd_btn_kind(const FsdButtons* b, uint8_t idx);

/** Feed one press from an EVENT button — the whole gesture, arriving at once.
 *
 *  There is no hold to measure, so this can return SHORT or DOUBLE and never
 *  LONG or STUCK. On a LEVEL button it does nothing: mixing the two entry
 *  points is a caller mistake, and guessing which one was meant is how a
 *  gesture ends up counted twice. */
FsdBtnEvent fsd_btn_pulse(FsdButtons* b, uint8_t idx, uint32_t now_ms);

/** Feed a level. **LEVEL buttons only** — ignored on an EVENT button, which is
 *  what stops a stray "down" from starting a hold that never ends.
 *
 *  Call whenever the peripheral reports, at whatever rate it
 *  reports — this takes edges from the level itself, so a repeated "still
 *  down" costs nothing.
 *
 *  Returns SHORT on a qualifying release. LONG and STUCK come from
 *  fsd_btn_tick(), because they happen while nothing is being reported. */
FsdBtnEvent fsd_btn_report(FsdButtons* b, uint8_t idx, bool pressed, uint32_t now_ms);

/** Advance a held button. Call from the main loop for every index; it is a
 *  comparison when nothing is held. Returns LONG once at the threshold and
 *  STUCK once past FSD_BTN_STUCK_MS. */
FsdBtnEvent fsd_btn_tick(FsdButtons* b, uint8_t idx, uint32_t now_ms);

/** True while this button is considered stuck — no further events until it is
 *  seen to release. */
bool fsd_btn_is_stuck(const FsdButtons* b, uint8_t idx);

/** Watch for double presses on this button. OFF after init.
 *
 *  Turning it on delays THIS button's SHORT by FSD_BTN_DOUBLE_MS — the tap can
 *  only be reported once a second one is known not to be coming. Other buttons
 *  are unaffected. Turning it off drops any tap that is currently waiting. */
void fsd_btn_set_double(FsdButtons* b, uint8_t idx, bool on);

/** Whether this button is watching for double presses. */
bool fsd_btn_double_enabled(const FsdButtons* b, uint8_t idx);

/** Counters, for the serial line and a future BLE surface. */
uint16_t fsd_btn_shorts(const FsdButtons* b, uint8_t idx);
uint16_t fsd_btn_longs(const FsdButtons* b, uint8_t idx);
uint16_t fsd_btn_doubles(const FsdButtons* b, uint8_t idx);
uint16_t fsd_btn_bounces(const FsdButtons* b, uint8_t idx);
uint16_t fsd_btn_last_hold_ms(const FsdButtons* b, uint8_t idx);

/** Human-readable event, for logs. */
const char* fsd_btn_event_str(FsdBtnEvent e);

#ifdef __cplusplus
}
#endif
