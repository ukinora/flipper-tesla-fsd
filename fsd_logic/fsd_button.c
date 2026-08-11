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

FsdBtnEvent fsd_btn_report(FsdButtons* b, uint8_t idx, bool pressed, uint32_t now_ms) {
    if(!b || idx >= FSD_BTN_MAX) return FSD_BTN_EV_NONE;
    FsdBtnState* s = &b->btn[idx];

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

    s->shorts++;
    return FSD_BTN_EV_SHORT;
}

FsdBtnEvent fsd_btn_tick(FsdButtons* b, uint8_t idx, uint32_t now_ms) {
    if(!b || idx >= FSD_BTN_MAX) return FSD_BTN_EV_NONE;
    FsdBtnState* s = &b->btn[idx];
    if(!s->down || s->stuck) return FSD_BTN_EV_NONE;

    const uint32_t held = (uint32_t)(now_ms - s->down_ms);

    /* Checked before LONG so a button that was already wedged when we started
     * listening cannot produce a LONG on its way to being declared stuck. */
    if(held >= FSD_BTN_STUCK_MS) {
        s->stuck = true;
        s->last_hold_ms = clamp16(held);
        return FSD_BTN_EV_STUCK;
    }
    if(!s->long_fired && held >= FSD_BTN_LONG_MS) {
        s->long_fired = true;
        s->longs++;
        return FSD_BTN_EV_LONG;
    }
    return FSD_BTN_EV_NONE;
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
    }
    return "?";
}
