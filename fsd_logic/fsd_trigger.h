#pragma once
/*
 * fsd_trigger.h — signal changes -> "the owner did something".
 *
 * WHERE THIS SITS
 * ---------------
 *      fsd_signal.c     bytes -> named values          (no state)
 *          |
 *      THIS FILE        values -> named events          <-- remembers
 *          |
 *      rule engine      event -> action
 *
 * IT DOES NOT CLASSIFY PRESSES ITSELF
 * -----------------------------------
 * fsd_button.c already does that, for the BLE remote, and it carries the
 * expensive lessons: debounce, LONG fires while still held, a wedged button
 * goes STUCK and stays quiet, double-press costs latency so it is opt-in per
 * button. Writing a second classifier for car buttons would mean two places to
 * fix the next one of those.
 *
 * So a switch signal is fed to fsd_btn_report() like any other button, in this
 * file's own FsdButtons instance. The remote keeps its own. They share the type
 * and the code, never the state.
 *
 * 🔴 CAR BUTTONS ARE LEVEL. THE REMOTE IS EVENT.
 * ----------------------------------------------
 * Measured 2026-09-01: a map-light switch bit follows the finger — 160-360 ms
 * for a tap, 1.2-3.7 s for a deliberate hold. The J6 remote is the opposite:
 * six of its seven buttons report press and release together.
 *
 * fsd_button.h explains why that asymmetry is dangerous rather than merely
 * different: calling an EVENT button LEVEL produces a phantom LONG at 0.6 s and
 * a STUCK at 10 s, and STUCK clears only on a release an event button can never
 * send. So it defaults to EVENT and must be told. This file tells it LEVEL,
 * once, at init, for every switch signal — because here we measured it.
 *
 * 🔴 A STATE TRIGGER FIRES ON OUR OWN WRITES TOO
 * ----------------------------------------------
 * "when the map light comes on" means "whoever turned it on", and we are one of
 * the whoevers. Two rules touching one state is a loop. So an action tells this
 * file which signals it disturbed, and state triggers on those stay quiet for a
 * short window.
 *
 * The suppression applies ONLY to state signals. We cannot press a physical
 * switch, so a switch trigger can only have come from a person, and silencing
 * it would silence the owner.
 *
 * 🔴 A MULTI-PRESS CANNOT BE TAKEN BACK
 * -------------------------------------
 * "window up, three times" means the window went up twice before we decided
 * anything. TSL lives with it and so will we (owner decision 2026-09-01), but
 * the app has to say so where a rule is built, because it is not visible from
 * the rule.
 */

#include "fsd_button.h"
#include "fsd_signal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Switch-kind signals get a button slot each: four map lights, eight window
 * switches. Asserted against the table in the .c so adding a switch signal
 * without a slot is a build error rather than a silent drop. */
#define FSD_TRIG_SWITCH_COUNT 12

/* 🔴 And they do NOT fit in one FsdButtons.
 *
 * FSD_BTN_MAX is capped at 10 by ble_central.cpp's 32-bit action mask
 * (FSD_BTN_MAX * FSD_BTN_EVENTS <= 32) — raising it stops the board compiling,
 * which is how that ceiling was found. So this file holds as many banks as it
 * needs and indexes across them. Same type, same code, separate state. */
#define FSD_TRIG_BANKS ((FSD_TRIG_SWITCH_COUNT + FSD_BTN_MAX - 1) / FSD_BTN_MAX)

/* How long a state signal stays deaf after we disturb it. Our own writes land
 * fast — the map light lit 65 ms after TSL's first frame and the gear moved in
 * 14 ms — so this is generous by design, and being generous costs only a
 * person's simultaneous press, which is the safe thing to lose. */
#define FSD_TRIG_SUPPRESS_MS 500u

typedef enum {
    FSD_TRIG_NONE = 0,
    FSD_TRIG_PRESS,       // a switch: tapped
    FSD_TRIG_LONG,        // a switch: held past the threshold, fires while held
    FSD_TRIG_DOUBLE,      // a switch: two taps, only where opted in
    FSD_TRIG_STUCK,       // a switch: held past all reason. Not a rule trigger.
    FSD_TRIG_STATE_ENTER, // a state became `value`
    FSD_TRIG_STATE_LEAVE, // a state stopped being `value`
    FSD_TRIG_DELTA,       // the scroll moved by `value` detents
} FsdTriggerKind;

typedef struct {
    FsdSignal signal;
    FsdTriggerKind kind;
    /* ENTER/LEAVE: the state. DELTA: the detent count, signed. Otherwise 0. */
    int32_t value;
    uint32_t at_ms;
} FsdTriggerEvent;

typedef struct {
    /* This file's own button banks. Separate instances from the remote's —
     * same code, same lessons, no shared indices. */
    FsdButtons switches[FSD_TRIG_BANKS];

    bool seen[FSD_SIG_COUNT];
    int32_t last[FSD_SIG_COUNT];

    /* State signals only; see the header note. 0 means not suppressed, which
     * is also what a zeroed struct means. */
    uint32_t quiet_until[FSD_SIG_COUNT];

    /* Set once a suppression window has been armed, so "0 means never" cannot
     * be confused with a window that legitimately expired at tick 0. */
    bool quiet_armed[FSD_SIG_COUNT];
} FsdTriggers;

/** Clear everything, and tell the button bank that car switches are LEVEL.
 *
 *  🔴 The LEVEL declaration happens here and nowhere else. Leaving it to the
 *  caller is how one of twelve gets missed, and a missed one is a switch whose
 *  hold never registers. */
void fsd_trig_init(FsdTriggers* t);

/** Watch for double presses on one switch signal. Off after init.
 *
 *  Costs THIS signal's tap the width of the window — see fsd_button.h. Turn it
 *  on only where a rule actually uses a double press. */
void fsd_trig_set_double(FsdTriggers* t, FsdSignal s, bool on);

/** An action of ours disturbed this state; ignore its triggers for a while.
 *
 *  Ignored for switch signals: we cannot press a switch, so anything that
 *  arrives there is a person. */
void fsd_trig_disturbed(FsdTriggers* t, FsdSignal s, uint32_t now_ms);

/** True while `s` is deaf because we disturbed it. */
bool fsd_trig_is_quiet(const FsdTriggers* t, FsdSignal s, uint32_t now_ms);

/** Feed one CAN frame. Returns how many events were written to `out`.
 *
 *  Signals this frame does not carry are left entirely alone — a frame of the
 *  wrong multiplex updates nothing, rather than reading zeros into every field
 *  it does not contain. */
uint8_t fsd_trig_on_frame(FsdTriggers* t, uint32_t can_id, const uint8_t* data, uint8_t dlc,
                          uint32_t now_ms, FsdTriggerEvent* out, uint8_t max_out);

/** Advance held switches. LONG and STUCK arrive from here, because they happen
 *  while nothing is being reported. Call from the loop. */
uint8_t fsd_trig_tick(FsdTriggers* t, uint32_t now_ms, FsdTriggerEvent* out, uint8_t max_out);

/** Human-readable kind, for logs and the app. */
const char* fsd_trig_kind_str(FsdTriggerKind k);

/** How many switch-kind signals the table actually holds.
 *
 *  Derived from fsd_signal.c at runtime, so it cannot be asserted against
 *  FSD_TRIG_SWITCH_COUNT at compile time. The tests compare them, which is
 *  where adding a switch signal without room for it turns red. */
int fsd_trig_switch_count(void);

#ifdef __cplusplus
}
#endif
