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
 * NO DOUBLE-PRESS, DELIBERATELY
 * -----------------------------
 * Supporting it would cost every SHORT press a delay of the double-press window
 * — a tap could not be reported until we knew a second one was not coming. For
 * "step the speed profile" that latency is the whole feel of the control, and
 * LONG already provides a second action per button without costing anything.
 *
 * (T2's gesture detector does implement double-press, on a car button we do not
 * own and cannot change. Different problem: there the two taps ARE the signal.)
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

/* How many buttons are tracked at once. A remote with up/down is two; the cap
 * exists so the struct has a size, not because four is a target. */
#ifndef FSD_BTN_MAX
#define FSD_BTN_MAX 4
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

typedef enum {
    FSD_BTN_EV_NONE = 0,
    FSD_BTN_EV_SHORT, // pressed and released inside the long threshold
    FSD_BTN_EV_LONG,  // still held at the long threshold; fires once
    FSD_BTN_EV_STUCK, // held past all reason; fires once, then silence
} FsdBtnEvent;

typedef struct {
    bool down;
    uint32_t down_ms;
    bool long_fired;
    bool stuck;
    uint16_t shorts;
    uint16_t longs;
    uint16_t bounces; // presses too brief to count — a link-quality signal
    uint16_t last_hold_ms;
} FsdBtnState;

typedef struct {
    FsdBtnState btn[FSD_BTN_MAX];
} FsdButtons;

/** Clear every button to released. */
void fsd_btn_init(FsdButtons* b);

/** Feed a level. Call whenever the peripheral reports, at whatever rate it
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

/** Counters, for the serial line and a future BLE surface. */
uint16_t fsd_btn_shorts(const FsdButtons* b, uint8_t idx);
uint16_t fsd_btn_longs(const FsdButtons* b, uint8_t idx);
uint16_t fsd_btn_bounces(const FsdButtons* b, uint8_t idx);
uint16_t fsd_btn_last_hold_ms(const FsdButtons* b, uint8_t idx);

/** Human-readable event, for logs. */
const char* fsd_btn_event_str(FsdBtnEvent e);

#ifdef __cplusplus
}
#endif
