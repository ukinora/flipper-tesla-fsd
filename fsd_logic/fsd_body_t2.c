/*
 * fsd_body_t2.c — see fsd_body_t2.h. Detector only; constructs no CAN frame.
 */

#include "fsd_body_t2.h"

#include <string.h>

/* Provisional, and nothing acts on it. 40 ms rejects contact bounce, 500 ms is
 * about where a tap becomes "I am moving the window", 120 ms is a placeholder
 * for the mux-0 frame period (unmeasured — see fsd_t2_mux0_min_gap_ms), 900 ms
 * is where two taps stop reading as one gesture. */
const FsdT2Timing FSD_T2_TIMING = {
    .press_min_ms = 40u,
    .press_max_ms = 500u,
    .gap_min_ms = 120u,
    .gap_max_ms = 900u,
    .verified = false,
};

void fsd_t2_init(FsdT2* t) {
    if(!t) return;
    memset(t, 0, sizeof(*t));
}

static uint16_t clamp16(uint32_t v) {
    return (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v;
}

bool fsd_t2_observe_switch(FsdT2* t, uint32_t can_id, const uint8_t* data, uint8_t dlc,
                           uint32_t now_ms) {
    if(!t || !data) return false;
    if(can_id != FSD_T2_CAN_ID) return false;
    if(dlc < 8) return false;

    /* VCLEFT_switchStatusIndex M : 0|2@1+. Mux 1 carries the steering wheel and
     * the rear-left door's own buttons; byte 4 bit 0 means a DIFFERENT button
     * there. Reading past this check would conflate them. */
    if((data[0] & 0x03u) != 0u) return false;

    /* The frame period, which nothing has ever measured on this car. */
    if(t->have_mux0) {
        const uint32_t gap = (uint32_t)(now_ms - t->last_mux0_ms);
        if(gap > 0u && (t->mux0_min_gap_ms == 0u || gap < t->mux0_min_gap_ms))
            t->mux0_min_gap_ms = gap;
    }
    t->last_mux0_ms = now_ms;
    t->have_mux0 = true;

    t->driver_present = ((data[0] >> 4) & 0x01u) != 0u; // VCLEFT_driverPresent 4|1
    t->driver_ms = now_ms;
    t->driver_seen = true;

    const uint8_t b5 = data[5];
    const bool up = (b5 & 0x01u) != 0u;      // btnWindowSwPackUpRF     40|1
    const bool auto_up = (b5 & 0x02u) != 0u; // btnWindowSwPackAutoUpRF 41|1

    /* EDGES, not levels. A level test for "twice" fires on a press-and-hold. */
    if(up && !t->pressed) {
        t->pressed = true;
        t->rise_ms = now_ms;
        t->last_byte5 = b5;
        return false;
    }
    if(!up && t->pressed) {
        t->pressed = false;
        const uint16_t press_ms = clamp16((uint32_t)(now_ms - t->rise_ms));
        t->last_press_ms = press_ms;

        /* Auto-up means the driver asked the window to close by itself. That is
         * a window action by definition, and the strongest discriminator
         * available — which is not saying much, since it only catches the case
         * where the driver held long enough to trigger it. */
        if(auto_up) {
            t->last_reject = FSD_T2_REJ_AUTOUP;
            t->have_first = false;
            return false;
        }
        if(press_ms < FSD_T2_TIMING.press_min_ms) {
            t->last_reject = FSD_T2_REJ_PRESS_SHORT;
            t->have_first = false; // a bounce breaks the sequence
            return false;
        }
        if(press_ms > FSD_T2_TIMING.press_max_ms) {
            t->last_reject = FSD_T2_REJ_PRESS_LONG;
            t->have_first = false;
            return false;
        }

        if(!t->have_first) {
            t->have_first = true;
            t->first_release_ms = now_ms;
            t->first_press_ms = press_ms;
            t->last_reject = FSD_T2_REJ_NONE;
            return false;
        }

        const uint16_t gap_ms = clamp16((uint32_t)(t->rise_ms - t->first_release_ms));
        t->last_gap_ms = gap_ms;
        if(gap_ms < FSD_T2_TIMING.gap_min_ms) {
            t->last_reject = FSD_T2_REJ_GAP_SHORT;
            /* Too close to be two intentions, but the second press was itself
             * well formed — bank it as the new first rather than throwing the
             * sequence away, so three fast taps cannot be made to vanish. */
            t->first_release_ms = now_ms;
            t->first_press_ms = press_ms;
            return false;
        }
        if(gap_ms > FSD_T2_TIMING.gap_max_ms) {
            t->last_reject = FSD_T2_REJ_GAP_LONG;
            t->first_release_ms = now_ms;
            t->first_press_ms = press_ms;
            return false;
        }

        t->have_first = false;
        t->last_reject = FSD_T2_REJ_NONE;
        t->gesture_count++;
        t->last_gesture_ms = now_ms;
        /* PUBLISHED, NOT ACTED ON. There is no emitter, and
         * FSD_BODY_CAPS[FSD_BODY_T2_DOOR].armable_at_runtime is false. */
        return true;
    }
    return false;
}

uint16_t fsd_t2_gesture_count(const FsdT2* t) { return t ? t->gesture_count : 0; }
uint32_t fsd_t2_last_gesture_ms(const FsdT2* t) { return t ? t->last_gesture_ms : 0; }
uint16_t fsd_t2_last_press_ms(const FsdT2* t) { return t ? t->last_press_ms : 0; }
uint16_t fsd_t2_last_gap_ms(const FsdT2* t) { return t ? t->last_gap_ms : 0; }
uint8_t fsd_t2_last_byte5(const FsdT2* t) { return t ? t->last_byte5 : 0; }
uint8_t fsd_t2_last_reject(const FsdT2* t) { return t ? t->last_reject : FSD_T2_REJ_NONE; }
uint32_t fsd_t2_mux0_min_gap_ms(const FsdT2* t) { return t ? t->mux0_min_gap_ms : 0; }
bool fsd_t2_driver_present(const FsdT2* t) { return t ? t->driver_present : false; }
uint32_t fsd_t2_driver_ms(const FsdT2* t) { return t ? t->driver_ms : 0; }
bool fsd_t2_driver_seen(const FsdT2* t) { return t ? t->driver_seen : false; }
