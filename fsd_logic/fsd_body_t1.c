/*
 * fsd_body_t1.c — see fsd_body_t1.h. Constructs no CAN frame.
 */

#include "fsd_body_t1.h"

#include <string.h>

void fsd_t1_init(FsdT1* t) {
    if(!t) return;
    memset(t, 0, sizeof(*t));
    t->last_verdict = FSD_BODY_NOT_ENABLED;
}

/* Only these three say anything about the door. OPENING and CLOSING are
 * transitions: they mean the latch is moving and the previous stable answer is
 * still the best one we have. Everything else — SNA, TIMEOUT, DEFAULT, FAULT,
 * or a value the enum does not define — is not knowledge, and is treated as
 * silence so the side goes stale and every aggregate needing it fails closed. */
static bool stable_value(uint8_t v) {
    return v == FSD_LATCH_OPENED || v == FSD_LATCH_CLOSED || v == FSD_LATCH_AJAR;
}

static bool transitional(uint8_t v) {
    return v == FSD_LATCH_OPENING || v == FSD_LATCH_CLOSING;
}

void fsd_t1_observe_door(FsdT1* t, FsdBodySide side, uint32_t can_id, const uint8_t* data,
                         uint8_t dlc, uint32_t now_ms) {
    if(!t || !data) return;
    if(side >= FSD_BODY_SIDE_COUNT) return;
    const uint32_t want = (side == FSD_BODY_SIDE_LEFT) ? FSD_T1_CAN_ID_LEFT : FSD_T1_CAN_ID_RIGHT;
    if(can_id != want) return; // mis-dispatch: nothing learned, nothing stamped
    if(dlc < 8) return;

    FsdT1Side* s = &t->side[side];
    const uint8_t v = (uint8_t)((data[0] >> 4) & 0x0Fu); // rearLatchStatus 4|4@1+
    s->raw = v;

    if(transitional(v)) {
        /* The latch is moving. Keep the last stable answer and keep the side
         * alive — going stale mid-swing would make a slow door look like a
         * dead bus. */
        s->seen_ms = now_ms;
        s->seen = true;
        return;
    }
    if(!stable_value(v)) return; // not knowledge: do not stamp

    s->seen_ms = now_ms;
    s->seen = true;

    if(s->has_stable && s->stable == v) {
        s->has_candidate = false; // already there
        return;
    }
    if(!s->has_candidate || s->candidate != v) {
        s->candidate = v;
        s->cand_ms = now_ms;
        s->has_candidate = true;
        /* First ever reading adopts immediately: there is no old value to
         * protect, and waiting would leave the aggregate unknown for no gain. */
        if(!s->has_stable) {
            s->stable = v;
            s->has_stable = true;
            s->has_candidate = false;
        }
        return;
    }
    if((uint32_t)(now_ms - s->cand_ms) >= FSD_T1_DEBOUNCE_MS) {
        s->stable = v;
        s->has_stable = true;
        s->has_candidate = false;
    }
}

static bool side_fresh(const FsdT1Side* s, uint32_t now_ms) {
    return s->seen && s->has_stable && (uint32_t)(now_ms - s->seen_ms) < FSD_BODY_FRESH_MS;
}

FsdT1Action fsd_t1_tick(FsdT1* t, const FsdBodyInputs* in, uint32_t now_ms) {
    if(!t) return FSD_T1_ACT_NONE;

    /* ON needs only one door to be open. OFF needs BOTH to be fresh and CLOSED
     * — the asymmetry is deliberate: a door we cannot see must not count as
     * shut. AJAR reads as open for ON and as not-closed for OFF, which is the
     * fail-closed reading of a door that is neither. */
    bool any_open = false;
    bool all_shut = true;
    for(int i = 0; i < FSD_BODY_SIDE_COUNT; i++) {
        const FsdT1Side* s = &t->side[i];
        const bool fresh = side_fresh(s, now_ms);
        if(fresh && (s->stable == FSD_LATCH_OPENED || s->stable == FSD_LATCH_AJAR))
            any_open = true;
        if(!fresh || s->stable != FSD_LATCH_CLOSED) all_shut = false;
    }

    bool next;
    if(any_open) {
        next = true;
    } else if(all_shut) {
        next = false;
    } else {
        return FSD_T1_ACT_NONE; // we do not know; do nothing
    }

    if(!t->has_state) {
        /* Adopt the first known aggregate WITHOUT acting. Otherwise a module
         * powering up next to an open door would fire an ON for an edge that
         * happened before it was listening. */
        t->open_state = next;
        t->has_state = true;
        return FSD_T1_ACT_NONE;
    }
    if(next == t->open_state) return FSD_T1_ACT_NONE;

    const FsdT1Action act = next ? FSD_T1_ACT_ON : FSD_T1_ACT_OFF;

    /* The edge is consumed here whether or not we are allowed to act on it.
     * Holding it pending would mean a door opened while refused could fire
     * minutes later when permission arrived — acting on a world that has since
     * moved on. */
    t->open_state = next;

    if(t->has_acted && (uint32_t)(now_ms - t->last_action_ms) < FSD_T1_MIN_GAP_MS) {
        t->last_verdict = FSD_BODY_NOT_ENABLED;
        return FSD_T1_ACT_NONE;
    }

    if(!t->window_start_ms && !t->window_count) t->window_start_ms = now_ms;
    if((uint32_t)(now_ms - t->window_start_ms) >= FSD_T1_WINDOW_MS) {
        t->window_start_ms = now_ms;
        t->window_count = 0;
    }
    if(t->window_count >= FSD_T1_MAX_PER_WINDOW) {
        t->last_verdict = FSD_BODY_NOT_ENABLED;
        return FSD_T1_ACT_NONE;
    }

    const FsdBodyVerdict v = fsd_body_allows(in, FSD_ACT_MAP_LIGHT, now_ms);
    t->last_verdict = v;
    if(v != FSD_BODY_OK) return FSD_T1_ACT_NONE;

    t->last_action_ms = now_ms;
    t->has_acted = true;
    t->window_count++;
    t->total_actions++;
    return act;
}

uint8_t fsd_t1_latch_raw(const FsdT1* t, FsdBodySide side) {
    if(!t || side >= FSD_BODY_SIDE_COUNT) return 0xFFu;
    return t->side[side].seen ? t->side[side].raw : 0xFFu;
}

uint16_t fsd_t1_window_count(const FsdT1* t) {
    return t ? t->window_count : 0;
}

FsdBodyVerdict fsd_t1_last_verdict(const FsdT1* t) {
    return t ? t->last_verdict : FSD_BODY_NOT_ENABLED;
}
