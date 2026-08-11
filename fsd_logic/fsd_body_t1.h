#pragma once
/*
 * fsd_body_t1.h — rear door opens or closes. Emits an EVENT, never a frame.
 *
 * T1 is what TSL does that we have to replace: a rear door opening turns the
 * cabin light on, closing turns it off.
 *
 * NOTHING IN THIS FILE CONSTRUCTS A CAN FRAME. The action is returned as an
 * enum for the caller to publish. See fsd_body.h for why there is no emitter
 * anywhere in this feature.
 *
 * WHAT THE OUTPUT WOULD BE, IF IT EXISTED
 * ---------------------------------------
 * The best candidate is 0x3F5 VCFRONT_courtesyLightingRequest (bit 24). The
 * project's own notes said for weeks that no interior-light signal existed in
 * any DBC; it does, and the firmware already had CAN_ID_VCFRONT_LIGHT defined.
 * But 0x3F5 is broadcast BY VCFRONT, carries neither counter nor checksum while
 * 0x3E9 DAS_bodyControls on the same bus carries both, and every structural
 * sign points at status rather than command. Whether it actuates anything is
 * unmeasured, and the detector below is finishable without knowing.
 *
 * THE HONEST LIMIT
 * ----------------
 * T1's natural moment is a rear door opening on a PARKED car whose driver has
 * already got out — and that is exactly when this module cannot act. The
 * hardware listen-only bit is cleared only by OpMode_Active, an unattended
 * module sits in Autonomous, and Autonomous is listen-only in hardware. So T1
 * as TSL does it may be out of reach without a change to that rule, which is an
 * owner's decision and not one to make by widening a gate quietly.
 *
 * What T1 requires instead is a DRIVE SESSION: a P -> D/R with the belt latched
 * since arming. It asks whether a drive happened, not whether one is happening,
 * which covers "parked at the shops, opening the back door" and excludes "sat
 * on the drive since yesterday".
 */

#include "fsd_body.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 0x102 VCLEFT_doorStatus / 0x103 VCRIGHT_doorStatus.
 *   SG_ VC*_rearLatchStatus : 4|4@1+  -> byte 0, bits [7:4]
 * (tools/dbc/upstream/tesla_model3_vehicle.dbc, BO_ 258 and BO_ 259) */
#define FSD_T1_CAN_ID_LEFT 0x102u
#define FSD_T1_CAN_ID_RIGHT 0x103u

/* The latch enum, from VAL_ 258/259 VC*_rearLatchStatus (vehicle.dbc:311,:335).
 * Only OPENED, CLOSED and AJAR are stable states worth acting on. */
#define FSD_LATCH_SNA 0u
#define FSD_LATCH_OPENED 1u
#define FSD_LATCH_CLOSED 2u
#define FSD_LATCH_CLOSING 3u
#define FSD_LATCH_OPENING 4u
#define FSD_LATCH_AJAR 5u
#define FSD_LATCH_TIMEOUT 6u
#define FSD_LATCH_DEFAULT 7u
#define FSD_LATCH_FAULT 8u

/* A new stable value must hold this long before it replaces the old one. A
 * latch bounces through OPENING/CLOSING and can flick through AJAR. */
#define FSD_T1_DEBOUNCE_MS 120u

/* Floor between two actions, and a fixed-window budget on top. A door being
 * worked repeatedly — loading shopping, a child playing with it — must not turn
 * into an unbounded stream of requests. Fixed window rather than a rolling one
 * because it needs no history: when the window expires, both counter and window
 * reset. */
#define FSD_T1_MIN_GAP_MS 1000u
#define FSD_T1_WINDOW_MS 3600000u
#define FSD_T1_MAX_PER_WINDOW 40u

typedef enum {
    FSD_BODY_SIDE_LEFT = 0,
    FSD_BODY_SIDE_RIGHT,
    FSD_BODY_SIDE_COUNT,
} FsdBodySide;

typedef enum {
    FSD_T1_ACT_NONE = 0,
    FSD_T1_ACT_ON,  // a rear door opened
    FSD_T1_ACT_OFF, // both rear doors are shut again
} FsdT1Action;

typedef struct {
    uint8_t stable;    // last debounced latch value
    bool has_stable;   //
    uint8_t raw;       // last decoded nibble, whatever it was — published for bring-up
    uint8_t candidate; // value waiting out the debounce
    uint32_t cand_ms;
    bool has_candidate;
    uint32_t seen_ms; // last frame we UNDERSTOOD (unknown values do not stamp)
    bool seen;
} FsdT1Side;

typedef struct {
    FsdT1Side side[FSD_BODY_SIDE_COUNT];
    bool open_state; // the aggregate the edges are taken from
    bool has_state;
    uint32_t last_action_ms;
    bool has_acted;
    uint32_t window_start_ms;
    uint16_t window_count;
    uint16_t total_actions;
    FsdBodyVerdict last_verdict;
} FsdT1;

/** Reset. Leaves both sides unseen, so every aggregate fails closed. */
void fsd_t1_init(FsdT1* t);

/** Feed a 0x102 or 0x103 frame. `side` must match the id — a mismatch is
 *  ignored and NOT stamped, so a mis-dispatch makes that side go stale rather
 *  than inventing a latch state. Same rule as fsd_drive_observe_gear(), and for
 *  the same reason: that observer shipped reading the wrong frame. */
void fsd_t1_observe_door(FsdT1* t, FsdBodySide side, uint32_t can_id, const uint8_t* data,
                         uint8_t dlc, uint32_t now_ms);

/** Advance. Returns an action ONLY on an aggregate edge, only once per edge,
 *  and only when fsd_body_allows() says yes — the verdict is stored either way
 *  so the app can show which gate refused. */
FsdT1Action fsd_t1_tick(FsdT1* t, const FsdBodyInputs* in, uint32_t now_ms);

/** Last raw latch nibble for a side, whatever it was. For bring-up: a build
 *  that only ever emits three of the nine enum values is worth knowing about. */
uint8_t fsd_t1_latch_raw(const FsdT1* t, FsdBodySide side);

/** Actions taken in the current window, and the verdict at the last edge. */
uint16_t fsd_t1_window_count(const FsdT1* t);
FsdBodyVerdict fsd_t1_last_verdict(const FsdT1* t);

#ifdef __cplusplus
}
#endif
