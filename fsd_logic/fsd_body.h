#pragma once
/*
 * fsd_body.h — the authority axis for body control, and nothing else.
 *
 * WHY A THIRD AXIS
 * ----------------
 * The safety design has stood on one sentence: the only thing we ever write is
 * a 0x3C2 scroll detent. T1 (rear door -> interior light) and T2 (double
 * window-up -> passenger door) are body-control writes, so that sentence has to
 * be extended rather than quietly stretched. This header is the extension.
 *
 * WHAT IT IS NOT
 * --------------
 * It is not permission to transmit. fsd_can_transmit() still has to say yes
 * first; this then says yes again, with more conditions. Its OpMode allow-set is
 * exactly {Active, Service} — the same set fsd_can_transmit() already admits —
 * so the axis is structurally incapable of widening anything. It can only
 * subtract. No OpMode value is added, and neither copy of fsd_can_transmit()
 * changes.
 *
 * THERE IS NO EMITTER ANYWHERE IN THIS FEATURE
 * --------------------------------------------
 * Not in this file, not in fsd_body_t1.c, not in fsd_body_t2.c. No frame is
 * constructed, no encoding table exists, and fsd_body_tx_id_refused() below
 * refuses the candidate IDs unconditionally.
 *
 * That is a decision, not an omission. The output frames for T1 and T2 are not
 * in any of the five DBCs, and the two candidates we do have (0x3F5 lighting,
 * 0x102/0x103 door status) are STATUS frames broadcast by the very controllers
 * that would have to act — 0x3F5 carries neither a counter nor a checksum,
 * while 0x3E9 DAS_bodyControls on the same bus carries both. Writing an encoder
 * for a frame whose identity, semantics, byte layout and drive shape are all
 * unmeasured is five independent ways to put a wrong frame on a live bus. The
 * detectors below are complete and testable today; the emitter waits for the
 * one-shot capture, or never happens.
 *
 * WHY T1 AND T2 DO NOT SHARE A ROW
 * --------------------------------
 * T1 is idempotent and its worst case is a light left on. T2 unlatches a door:
 * no undo, no upper bound. Sharing an axis would force the union of both risk
 * profiles onto both features. So each gets its own capability row, and T2's row
 * is all-zero — every restriction on, and `armable_at_runtime` false, which
 * means NO CODE PATH CAN ENABLE IT. That is the point of the row, and it is
 * checked by a static assertion rather than by review.
 *
 * See 페일세이프-정책.md.
 */

#include "fsd_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FSD_BODY_T1_LIGHT = 0,
    FSD_BODY_T2_DOOR,
    FSD_BODY_FEATURE_COUNT,
} FsdBodyFeature;

/* Every field is PERMISSIVE WHEN TRUE, so C's zero-fill is the tightest
 * possible row. A capability someone forgets to write is a capability that is
 * not granted — the opposite of the deny-list mistake that shipped once already
 * and had to be inverted in PR #10. */
typedef struct {
    FsdBodyFeature feature; // must equal its own index; checked at runtime
    bool may_act_while_moving;
    bool may_act_out_of_park;
    bool may_act_without_driver;
    bool may_act_without_drive_session;
    bool armable_at_runtime; // false = nothing may set feature_enabled for it
} FsdBodyCaps;

/* Freshness for every supervision input here. Same window and same reasoning as
 * FSD_DRIVE_CTX_FRESH_MS in fsd_autonomy.h. */
#define FSD_BODY_FRESH_MS 1000u

/* Above this the car is moving, for any feature whose row says it may not act
 * while moving. Deliberately low: the question is "is it standing still", not
 * "is it slow". */
#define FSD_BODY_STANDSTILL_KPH 0.5f

/* Everything the predicate needs, snapshotted by the caller — the same shape as
 * FsdSpInputs, and for the same reason: it makes the gate testable on the host
 * with no firmware, and it makes every input visible at the call site instead of
 * reached for through a global. */
typedef struct {
    OpMode op_mode;

    /* The CAN controller's hardware listen-only bit is per-controller, not
     * per-frame, and CanDriver::send() refuses outright while it is set. Only
     * OpMode_Active clears it. Carried as an input rather than assumed, and
     * DEFAULTS FALSE when the caller cannot tell — the same fail-closed
     * convention as FsdSpInputs.tx_armed. No body feature may clear that bit as
     * a side effect of being enabled. */
    bool bus_tx_open;

    /* Read raw. The ESP32 copy of fsd_can_transmit() honours an `ignore_ota`
     * override that is NVS-persisted and settable from the web dashboard; a body
     * write must not inherit that escape hatch, so this gate never consults it
     * and is therefore strictly narrower in every state. */
    bool ota_in_progress;
    bool rx_stale;

    /* SESSION-SCOPED. Never persisted to NVS — that is operator intent for the
     * camera axis only, where the flag grants nothing on its own. Here it is
     * closer to authority, so it dies with the power. */
    bool feature_enabled[FSD_BODY_FEATURE_COUNT];

    /* A drive has actually happened since arming: a P -> D/R transition with the
     * belt latched. Asks whether a drive HAPPENED, not whether one is happening
     * now — which is what lets T1 act on a parked car whose driver just got out,
     * without granting anything to a car that has sat untouched all night. */
    bool drive_session;

    bool driver_seen;
    bool driver_present; // VCLEFT_driverPresent, 0x3C2 mux 0 bit 4
    uint32_t driver_ms;

    bool gear_seen;
    uint8_t gear; // FSD_GEAR_* from fsd_autonomy.h
    uint32_t gear_ms;

    bool speed_seen;
    float speed_kph; // drivetrain, never GPS — see camera_task.h
    uint32_t speed_ms;
} FsdBodyInputs;

/* One named value per refusal, so "the light did not come on" can be answered
 * over BLE instead of guessed at. Same shape and purpose as FsdSupVerdict. */
typedef enum {
    FSD_BODY_OK = 0,
    FSD_BODY_UNKNOWN_FEATURE, // out of range, or a capability row that drifted
    FSD_BODY_NOT_ARMABLE,     // the row forbids arming this at all (T2)
    FSD_BODY_NOT_ENABLED,
    FSD_BODY_NO_MODE,   // op_mode outside {Active, Service}
    FSD_BODY_BUS_SHUT,  // hardware listen-only: send() would refuse anyway
    FSD_BODY_OTA,
    FSD_BODY_RX_STALE,
    FSD_BODY_NO_DRIVER,
    FSD_BODY_DRIVER_STALE,
    FSD_BODY_NO_DRIVER_PRESENT,
    FSD_BODY_NO_DRIVE_SESSION,
    FSD_BODY_NO_GEAR,
    FSD_BODY_GEAR_STALE,
    FSD_BODY_NOT_PARK,
    FSD_BODY_NO_SPEED,
    FSD_BODY_SPEED_STALE,
    FSD_BODY_MOVING,
} FsdBodyVerdict;

/** May this feature act right now? Holds no state, latches nothing, and grants
 *  no grace period. Checked in a fixed order so a missing input can never hide
 *  the gates behind it. */
FsdBodyVerdict fsd_body_allows(const FsdBodyInputs* in, FsdBodyFeature f, uint32_t now_ms);

/** Human-readable verdict, for logs and the BLE surface. */
const char* fsd_body_verdict_str(FsdBodyVerdict v);

/** True for every CAN ID this feature could conceivably want to write.
 *
 *  UNCONDITIONAL, and deliberately not derived from the capability table: this
 *  firmware constructs no body frame at all, so the honest answer for every one
 *  of these IDs is "refused", in every state, forever. It exists so that a
 *  future call site cannot forget its gate — a backstop, never a permission. */
bool fsd_body_tx_id_refused(uint32_t can_id);

#ifdef __cplusplus
}
#endif
