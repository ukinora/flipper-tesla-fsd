#pragma once
/*
 * fsd_body_t2.h — the double window-up gesture. DETECTOR ONLY.
 *
 * TSL opens the passenger door when the driver taps the passenger window-UP
 * button twice. This file recognises that gesture. It does not open anything,
 * it evaluates no interlock, and it constructs no CAN frame — and unlike T1,
 * where the emitter is merely deferred, T2's emitter is not planned.
 *
 * WHY NOT
 * -------
 * There is no CAN-observable way to tell the gesture apart from an ordinary
 * double tap on the passenger window.
 *
 * The natural discriminator would be "the window is already shut, so this tap
 * cannot be a window action". That signal does not exist: searching all five
 * DBCs for a window POSITION returns nothing — only the twenty btnWindow*
 * switch bits. Nor is there a vehicle-locked signal, nor a child-lock signal.
 * Three of the interlocks a door release ought to have cannot be sourced from
 * the bus at all.
 *
 * So a false positive is: the driver adjusts the passenger window twice, and the
 * passenger door unlatches. Possibly at speed, possibly with a child against it.
 * That is an unbounded consequence gated on a guess, and the passenger door
 * already has an interior release handle. Until something changes, the honest
 * position is that T2's action does not get built.
 *
 * WHAT THIS IS FOR, THEN
 * ----------------------
 * Two things, both real:
 *
 *  1. It MEASURES the gesture. Every candidate — accepted or rejected — records
 *     its press and gap durations and why it was rejected, so the one-shot TSL
 *     capture can be validated offline and the timing window sized from data
 *     instead of guessed. Without this the capture would have to be repeated.
 *  2. It measures the 0x3C2 mux-0 transmit period, which is the hard floor on
 *     how close two presses can even resolve as two. No DBC in this repo states
 *     a cycle time for anything.
 *
 * The permission side is already closed against it: FSD_BODY_CAPS[T2] has
 * armable_at_runtime = false, so fsd_body_allows() returns NOT_ARMABLE in every
 * state and no code path can change that without editing the table.
 *
 * INPUT — AND IT IS FREE
 * ----------------------
 * 0x3C2 VCLEFT_switchStatus, mux 0. The same frame the scroll wheel rides on,
 * which we already receive. The project notes said 0x102; that was wrong.
 *   VCLEFT_switchStatusIndex M  : 0|2@1+   -> byte 0 bits [1:0]
 *   VCLEFT_btnWindowSwPackUpRF  : 40|1@1+  -> byte 5 bit 0   (driver's armrest,
 *   VCLEFT_btnWindowSwPackAutoUpRF: 41|1@1+ -> byte 5 bit 1    PASSENGER window)
 *   VCLEFT_driverPresent        : 4|1@1+   -> byte 0 bit 4
 *
 * The mux check is not a formality: byte 4 bit 0 is btnWindowSwPackUpLF on mux 0
 * and btnWindowUpLR on mux 1. Skipping it conflates two different buttons.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FSD_T2_CAN_ID 0x3C2u

/* Every number here is a guess, and the flag says so. Nothing consumes the
 * timing yet, which is the only reason a guess is acceptable: the capture
 * replaces these before anything could act on them. Same discipline as
 * FsdSpEncoding.verified. */
typedef struct {
    uint16_t press_min_ms; // shorter than this is a bounce, not a press
    uint16_t press_max_ms; // longer is someone actually moving the window
    uint16_t gap_min_ms;   // closer than this cannot be two deliberate taps
    uint16_t gap_max_ms;   // further apart is two separate intentions
    bool verified;
} FsdT2Timing;

extern const FsdT2Timing FSD_T2_TIMING;

typedef enum {
    FSD_T2_REJ_NONE = 0,
    FSD_T2_REJ_PRESS_SHORT,
    FSD_T2_REJ_PRESS_LONG,
    FSD_T2_REJ_GAP_SHORT,
    FSD_T2_REJ_GAP_LONG,
    FSD_T2_REJ_AUTOUP, // the auto-up bit was set: a window action, not a gesture
} FsdT2Reject;

typedef struct {
    bool pressed;       // current debounced level of btnWindowSwPackUpRF
    uint32_t rise_ms;   // when the current press started
    bool have_first;    // one complete press is banked
    uint32_t first_release_ms;
    uint16_t first_press_ms;

    uint32_t last_mux0_ms;
    bool have_mux0;
    uint32_t mux0_min_gap_ms; // smallest observed inter-arrival, 0 = none yet

    bool driver_present;
    uint32_t driver_ms;
    bool driver_seen;

    uint16_t gesture_count;
    uint32_t last_gesture_ms;
    uint16_t last_press_ms;
    uint16_t last_gap_ms;
    uint8_t last_byte5;
    uint8_t last_reject;
} FsdT2;

void fsd_t2_init(FsdT2* t);

/** Feed a 0x3C2 frame. Returns true on the tick a complete gesture is
 *  recognised — which the caller PUBLISHES and does not act on.
 *  Ignores any other CAN ID, any dlc < 8, and any mux other than 0. */
bool fsd_t2_observe_switch(FsdT2* t, uint32_t can_id, const uint8_t* data, uint8_t dlc,
                           uint32_t now_ms);

uint16_t fsd_t2_gesture_count(const FsdT2* t);
uint32_t fsd_t2_last_gesture_ms(const FsdT2* t);

/* Measured, and published even when the candidate was rejected — the rejects
 * are the data that sizes the window. */
uint16_t fsd_t2_last_press_ms(const FsdT2* t);
uint16_t fsd_t2_last_gap_ms(const FsdT2* t);
uint8_t fsd_t2_last_byte5(const FsdT2* t);
uint8_t fsd_t2_last_reject(const FsdT2* t);

/** Smallest observed gap between two mux-0 frames. The floor on how close two
 *  presses can resolve as two, and unmeasured until now. */
uint32_t fsd_t2_mux0_min_gap_ms(const FsdT2* t);

/* driverPresent rides in the same frame, so it costs nothing to carry. */
bool fsd_t2_driver_present(const FsdT2* t);
uint32_t fsd_t2_driver_ms(const FsdT2* t);
bool fsd_t2_driver_seen(const FsdT2* t);

#ifdef __cplusplus
}
#endif
