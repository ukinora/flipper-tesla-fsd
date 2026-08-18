/*
 * fsd_button.c — see fsd_button.h. Timing only; nothing radio-specific.
 */

#include "fsd_button.h"

#include <string.h>

void fsd_btn_init(FsdButtons* b) {
    if(!b) return;
    memset(b, 0, sizeof(*b));
}

static uint16_t clamp16(uint32_t v) {
    return (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v;
}

/* A completed tap, wherever it came from. Shared so the two entry points cannot
 * drift — the double-press rule has to be identical or one kind of button gets
 * a different feel from the other for no reason. */
static FsdBtnEvent finish_tap(FsdBtnState* s, uint32_t now_ms) {
    if(s->want_double) {
        if(s->tap_pending) {
            s->tap_pending = false;
            s->doubles++;
            return FSD_BTN_EV_DOUBLE;
        }
        s->tap_pending = true;
        s->pending_ms = now_ms;
        return FSD_BTN_EV_NONE; // withheld; fsd_btn_tick() releases it
    }
    s->shorts++;
    return FSD_BTN_EV_SHORT;
}

FsdBtnEvent fsd_btn_pulse(FsdButtons* b, uint8_t idx, uint32_t now_ms) {
    if(!b || idx >= FSD_BTN_MAX) return FSD_BTN_EV_NONE;
    FsdBtnState* s = &b->btn[idx];
    /* Not a guess about what the caller meant — the wrong entry point is a bug
     * in the caller, and acting on it would double-count a gesture. */
    if(s->kind != FSD_BTN_KIND_EVENT) return FSD_BTN_EV_NONE;
    return finish_tap(s, now_ms);
}

FsdBtnEvent fsd_btn_report(FsdButtons* b, uint8_t idx, bool pressed, uint32_t now_ms) {
    if(!b || idx >= FSD_BTN_MAX) return FSD_BTN_EV_NONE;
    FsdBtnState* s = &b->btn[idx];

    /* 🔴 THE TRAP. On an event button a "down" would start a hold that no
     * release ever ends: LONG at 600 ms, STUCK at 10 s, and STUCK only clears
     * on a release. Ignoring it here is what keeps that button alive. */
    if(s->kind != FSD_BTN_KIND_LEVEL) return FSD_BTN_EV_NONE;

    if(pressed) {
        if(!s->down) {
            s->down = true;
            s->down_ms = now_ms;
            s->long_fired = false;
        }
        return FSD_BTN_EV_NONE; // a press is only ever classified on the way out
    }

    if(!s->down) return FSD_BTN_EV_NONE; // repeated "up": nothing happened

    const uint32_t held = (uint32_t)(now_ms - s->down_ms);
    s->down = false;
    s->last_hold_ms = clamp16(held);

    /* A release is the ONLY thing that clears stuck. That is the point: the
     * evidence that a button is not wedged is seeing it come up. */
    if(s->stuck) {
        s->stuck = false;
        s->long_fired = false;
        return FSD_BTN_EV_NONE; // the release that ends a stuck press is not a press
    }
    if(held < FSD_BTN_DEBOUNCE_MS) {
        s->bounces++;
        return FSD_BTN_EV_NONE;
    }
    if(s->long_fired) {
        s->long_fired = false;
        return FSD_BTN_EV_NONE; // already reported as a LONG while held
    }

    /* This IS a short press. Whether it is reported now depends on whether a
     * twin might still be coming — see fsd_button.h. */
    return finish_tap(s, now_ms);
}

/* The withheld-tap timeout. The only part of tick() an event button has. */
/* The window this button waits. 0 means "never set" — the default. */
static uint16_t window_of(const FsdBtnState* s) {
    return s->double_ms ? s->double_ms : (uint16_t)FSD_BTN_DOUBLE_MS;
}

static FsdBtnEvent release_pending(FsdBtnState* s, uint32_t now_ms) {
    if(s->tap_pending && (uint32_t)(now_ms - s->pending_ms) >= window_of(s)) {
        s->tap_pending = false;
        s->shorts++;
        return FSD_BTN_EV_SHORT;
    }
    return FSD_BTN_EV_NONE;
}

FsdBtnEvent fsd_btn_tick(FsdButtons* b, uint8_t idx, uint32_t now_ms) {
    if(!b || idx >= FSD_BTN_MAX) return FSD_BTN_EV_NONE;
    FsdBtnState* s = &b->btn[idx];

    /* An event button has no hold, so it has no LONG and no STUCK. Waiting for
     * a twin is the only thing time does for it. */
    if(s->kind == FSD_BTN_KIND_EVENT) return release_pending(s, now_ms);

    /* A withheld tap is only released while the button is UP. If it is down
     * again the second press is still in progress, and letting the window
     * expire underneath it would produce a SHORT and then a DOUBLE from one
     * gesture — the mapped action would fire twice. */
    if(!s->down) return release_pending(s, now_ms);
    if(s->stuck) return FSD_BTN_EV_NONE;

    const uint32_t held = (uint32_t)(now_ms - s->down_ms);

    /* Checked before LONG so a button that was already wedged when we started
     * listening cannot produce a LONG on its way to being declared stuck. */
    if(held >= FSD_BTN_STUCK_MS) {
        s->stuck = true;
        s->last_hold_ms = clamp16(held);
        s->tap_pending = false; // a wedged button does not complete a gesture
        return FSD_BTN_EV_STUCK;
    }
    if(!s->long_fired && held >= FSD_BTN_LONG_MS) {
        s->long_fired = true;
        s->longs++;
        /* Tap-then-hold is a hold. Dropping the waiting tap loses it, which is
         * better than surfacing it after the LONG: one gesture, one event. */
        s->tap_pending = false;
        return FSD_BTN_EV_LONG;
    }
    return FSD_BTN_EV_NONE;
}

void fsd_btn_set_kind(FsdButtons* b, uint8_t idx, FsdBtnKind kind) {
    if(!b || idx >= FSD_BTN_MAX) return;
    FsdBtnState* s = &b->btn[idx];
    if(s->kind == kind) return;
    s->kind = kind;
    /* Everything that describes a gesture in progress is meaningless in the
     * other mode. Leaving `down` set would hand the new mode a hold that
     * started under rules it does not use — and on the LEVEL side that is a
     * button wedged the moment it is promoted. Counters and want_double are
     * settings, not state, so they stay. */
    s->down = false;
    s->down_ms = 0;
    s->long_fired = false;
    s->stuck = false;
    s->tap_pending = false;
    s->pending_ms = 0;
}

FsdBtnKind fsd_btn_kind(const FsdButtons* b, uint8_t idx) {
    return (b && idx < FSD_BTN_MAX) ? b->btn[idx].kind : FSD_BTN_KIND_EVENT;
}

void fsd_btn_set_double(FsdButtons* b, uint8_t idx, bool on) {
    if(!b || idx >= FSD_BTN_MAX) return;
    b->btn[idx].want_double = on;
    if(!on) b->btn[idx].tap_pending = false; // nothing left to wait for
}

bool fsd_btn_double_enabled(const FsdButtons* b, uint8_t idx) {
    return (b && idx < FSD_BTN_MAX) ? b->btn[idx].want_double : false;
}

void fsd_btn_set_double_window(FsdButtons* b, uint8_t idx, uint16_t ms) {
    if(!b || idx >= FSD_BTN_MAX) return;
    /* A window at or past the hold threshold eats the hold: the tap would still
     * be waiting when LONG fires, and LONG drops it — so the button would have
     * a double-press that can never complete and a long-press that always
     * wins. Clamp rather than refuse; the caller asked for "as long as
     * possible" and this is what that is. */
    if(ms >= (uint16_t)FSD_BTN_LONG_MS) ms = (uint16_t)(FSD_BTN_LONG_MS - 100u);
    b->btn[idx].double_ms = ms;
}

uint16_t fsd_btn_double_window(const FsdButtons* b, uint8_t idx) {
    if(!b || idx >= FSD_BTN_MAX) return 0;
    return window_of(&b->btn[idx]);
}

bool fsd_btn_is_stuck(const FsdButtons* b, uint8_t idx) {
    if(!b || idx >= FSD_BTN_MAX) return false;
    return b->btn[idx].stuck;
}

uint16_t fsd_btn_shorts(const FsdButtons* b, uint8_t idx) {
    return (b && idx < FSD_BTN_MAX) ? b->btn[idx].shorts : 0;
}
uint16_t fsd_btn_longs(const FsdButtons* b, uint8_t idx) {
    return (b && idx < FSD_BTN_MAX) ? b->btn[idx].longs : 0;
}
uint16_t fsd_btn_doubles(const FsdButtons* b, uint8_t idx) {
    return (b && idx < FSD_BTN_MAX) ? b->btn[idx].doubles : 0;
}
uint16_t fsd_btn_bounces(const FsdButtons* b, uint8_t idx) {
    return (b && idx < FSD_BTN_MAX) ? b->btn[idx].bounces : 0;
}
uint16_t fsd_btn_last_hold_ms(const FsdButtons* b, uint8_t idx) {
    return (b && idx < FSD_BTN_MAX) ? b->btn[idx].last_hold_ms : 0;
}

const char* fsd_btn_event_str(FsdBtnEvent e) {
    switch(e) {
    case FSD_BTN_EV_NONE: return "none";
    case FSD_BTN_EV_SHORT: return "short";
    case FSD_BTN_EV_LONG: return "long";
    case FSD_BTN_EV_STUCK: return "stuck";
    case FSD_BTN_EV_DOUBLE: return "double";
    }
    return "?";
}
