#pragma once
/*
 * fsd_body.h — the authority axis for body control, and nothing else.
 *
 * WHY A THIRD AXIS
 * ----------------
 * The safety design stood on one sentence: the only thing we ever write is a
 * 0x3C2 scroll detent. Body control is a write, so that sentence has to be
 * extended rather than quietly stretched. This header is the extension.
 *
 * 🔴 THE ROWS ARE ACTIONS, NOT FEATURES (rewritten 2026-09-01)
 * ------------------------------------------------------------
 * They used to be T1 ("rear door -> interior light") and T2 ("double window-up
 * -> passenger door") -- a trigger and an action welded together. The owner
 * wants a rule engine: arbitrary trigger -> arbitrary action, overlapping. Once
 * triggers and actions are M:N, a row like T1 cannot exist.
 *
 * Risk does not live on the trigger. It lives on WHAT WE DO TO THE CAR. Index
 * the table by action and a hundred rules still check the same seven rows.
 *
 *      rule engine     trigger -> action      (owner builds these in the app)
 *          |
 *      THIS AXIS       may this action happen right now
 *          |
 *      emitter         builds the frame
 *          |
 *      TX chokepoint   the last denial
 *
 * WHAT IT IS NOT
 * --------------
 * It is not permission to transmit. fsd_can_transmit() still has to say yes
 * first; this then says yes again, with more conditions. Its OpMode allow-set is
 * exactly {Active, Service} -- the same set fsd_can_transmit() already admits --
 * so the axis is structurally incapable of widening anything. It can only
 * subtract. No OpMode value is added, and neither copy of fsd_can_transmit()
 * changes.
 *
 * THE EMITTER EXISTS FOR EXACTLY ONE ACTION (2026-09-03)
 * ------------------------------------------------------
 * 🔴 This section said "THERE IS STILL NO EMITTER ANYWHERE" until the map
 * light command was measured. One now exists, in fsd_body_emit.c, and it builds
 * a frame for MAP_LIGHT and refuses every other action.
 *
 * 🟢 NOTHING TRANSMITS. The emitter returns bytes; no caller puts them on a
 * bus. The first real write is a decision to be made in the car, with something
 * reversible, and it is not made in code.
 *
 * Only one row is armable (MAP_LIGHT, carried over unchanged from T1); every
 * other row has armable_at_runtime = false. Each row says in its comment what
 * evidence flips its bool.
 *
 * 🔴 The two statements must move together. armable_at_runtime and "has an
 * emitter" describe the same fact from two sides, and test_body_emit.c asserts
 * they agree for all seven actions — so opening a row without an encoding, or
 * writing an encoding without opening the row, turns a test red.
 *
 * See 권한축-재설계.md and 페일세이프-정책.md.
 */

#include "fsd_types.h"

#include <stdbool.h>
#include <stddef.h> // NULL — fsd_body_caps() returns it, and newlib does not
                    // hand it over for free the way glibc happens to. The host
                    // tests compiled fine without this; only the ESP32 build
                    // caught it. Fifth pattern, exactly.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One row per thing we can do TO THE CAR. Direction (seat forward/back, scroll
 * up/down) is a parameter of the emitter, not a row: it does not change the
 * risk, and a row that cannot change the answer is a row that should not exist.
 * Which PART of the car does change it -- the passenger seat needs an occupancy
 * check the driver's seat does not -- so those are separate rows. */
typedef enum {
    FSD_ACT_MAP_LIGHT = 0,  // was the old T1 row
    FSD_ACT_DOOR_OPEN,      // was the old T2 row
    FSD_ACT_CAMERA,         // 0x3C2 byte6 bit3 (toggle) -- measured 2026-09-01
    FSD_ACT_SEAT_DRIVER,    // 0x3C2 byte1 bits 2-3 / 0-1
    FSD_ACT_SEAT_PASSENGER, // 🔴 NOT the same frame: 0x3C3 byte0
                            //   (앞 bits 4-5 / 뒤 bits 2-3), measured
                            //   2026-09-03. Left/right is a different
                            //   FRAME, not different bits — VCLEFT vs
                            //   VCRIGHT. The old comment would have
                            //   sent an emitter to 0x3C2.
    FSD_ACT_SCROLL,         // 0x3C2 byte3, 6-bit signed detents
    FSD_ACT_GEAR_D,         // 0x229 -- P -> D only. See caps row.
    FSD_ACT_COUNT,
} FsdBodyAction;

/* Every field is PERMISSIVE WHEN TRUE (or, for the numbers, PERMISSIVE WHEN
 * NON-ZERO), so C's zero-fill is the tightest possible row. A capability
 * someone forgets to write is a capability that is not granted -- the opposite
 * of the deny-list mistake that shipped once already and had to be inverted in
 * PR #10. */
typedef struct {
    FsdBodyAction action; // must equal its own index; checked at runtime
    bool may_act_while_moving;
    bool may_act_out_of_park;
    bool may_act_without_driver;
    bool may_act_without_drive_session;
    bool armable_at_runtime; // false = nothing may set action_enabled for it

    /* Gear selection is the one action where the CURRENT gear is part of the
     * permission: P -> D is the only transition this firmware will ever ask
     * for, so anything else refuses. Distinct from may_act_out_of_park, which
     * asks whether the action may happen at all outside P. */
    bool requires_park;

    /* The passenger seat must not move onto someone. frontOccupancySwitch
     * (0x3C2 mux 0, 50|2) is already on the bus -- reading it costs nothing and
     * not reading it would be laziness, not a trade-off. */
    bool requires_passenger_empty;

    /* Gear selection additionally wants the belt latched. The trigger the owner
     * has in mind IS the belt, but a gate that trusts its trigger is not a
     * gate. */
    bool requires_belt;

    /* 0 = this action may never fire. Rate limiting is not politeness: two
     * rules that both fire on the same state produce a burst, and a seat motor
     * driven by a burst goes to the end of its rail. */
    uint16_t min_interval_ms;

    /* 0 = this action may not be held at all (single-shot only). An action that
     * must be re-sent to stay in effect needs an upper bound, or one stuck
     * flag transmits forever. */
    uint16_t max_hold_ms;
} FsdBodyCaps;

/** The row for an action, or NULL if the index is out of range or the table
 *  drifted. Exposed so the emitter can read max_hold_ms without a second copy
 *  of the table -- two copies is how the deny-lists diverged before. */
const FsdBodyCaps* fsd_body_caps(FsdBodyAction a);

/* Freshness for every supervision input here. Same window and same reasoning as
 * FSD_DRIVE_CTX_FRESH_MS in fsd_autonomy.h. */
#define FSD_BODY_FRESH_MS 1000u

/* Above this the car is moving, for any action whose row says it may not act
 * while moving. Deliberately low: the question is "is it standing still", not
 * "is it slow". */
#define FSD_BODY_STANDSTILL_KPH 0.5f

/* Everything the predicate needs, snapshotted by the caller -- the same shape as
 * FsdSpInputs, and for the same reason: it makes the gate testable on the host
 * with no firmware, and it makes every input visible at the call site instead of
 * reached for through a global. */
typedef struct {
    OpMode op_mode;

    /* The CAN controller's hardware listen-only bit is per-controller, not
     * per-frame, and CanDriver::send() refuses outright while it is set. Only
     * OpMode_Active clears it. Carried as an input rather than assumed, and
     * DEFAULTS FALSE when the caller cannot tell -- the same fail-closed
     * convention as FsdSpInputs.tx_armed. No body action may clear that bit as
     * a side effect of being enabled. */
    bool bus_tx_open;

    /* Read raw. The ESP32 copy of fsd_can_transmit() honours an `ignore_ota`
     * override that is NVS-persisted; a body write must not inherit that escape
     * hatch, so this gate never consults it and is therefore strictly narrower
     * in every state. */
    bool ota_in_progress;
    bool rx_stale;

    /* SESSION-SCOPED. Never persisted to NVS -- that is operator intent for the
     * camera axis only, where the flag grants nothing on its own. Here it is
     * closer to authority, so it dies with the power. */
    bool action_enabled[FSD_ACT_COUNT];

    /* When each action last fired, for min_interval_ms. The CALLER owns this
     * array: the predicate stays stateless, which is what lets the host tests
     * drive it to any instant without a fixture. 0 means "never fired", and the
     * unsigned wrap below treats that as long ago -- correct, because it is. */
    uint32_t last_act_ms[FSD_ACT_COUNT];

    /* A drive has actually happened since arming: a P -> D/R transition with the
     * belt latched. Asks whether a drive HAPPENED, not whether one is happening
     * now -- which is what lets a light act on a parked car whose driver just
     * got out, without granting anything to a car that has sat untouched all
     * night. */
    bool drive_session;

    bool driver_seen;
    bool driver_present; // VCLEFT_driverPresent, 0x3C2 mux 0 bit 4
    uint32_t driver_ms;

    bool gear_seen;
    uint8_t gear; // FSD_GEAR_* from fsd_autonomy.h
    uint32_t gear_ms;

    bool speed_seen;
    float speed_kph; // drivetrain, never GPS -- see camera_task.h
    uint32_t speed_ms;

    bool belt_seen;
    bool belt_latched; // 0x3C2 mux 0, frontBuckleSwitch (48|2) == 2
    uint32_t belt_ms;

    bool passenger_seen;
    bool passenger_present; // 0x3C2 mux 0, frontOccupancySwitch (50|2)
    uint32_t passenger_ms;
} FsdBodyInputs;

/* One named value per refusal, so "the light did not come on" can be answered
 * over BLE instead of guessed at. Same shape and purpose as FsdSupVerdict. */
typedef enum {
    FSD_BODY_OK = 0,
    FSD_BODY_UNKNOWN_ACTION, // out of range, or a capability row that drifted
    FSD_BODY_NOT_ARMABLE,    // the row forbids arming this at all
    FSD_BODY_NOT_ENABLED,
    FSD_BODY_NO_MODE,   // op_mode outside {Active, Service}
    FSD_BODY_BUS_SHUT,  // hardware listen-only: send() would refuse anyway
    FSD_BODY_OTA,
    FSD_BODY_RX_STALE,
    FSD_BODY_TOO_SOON, // min_interval_ms, or a row that may never fire
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
    FSD_BODY_NO_BELT,
    FSD_BODY_BELT_STALE,
    FSD_BODY_NO_PASSENGER_SIGNAL,
    FSD_BODY_PASSENGER_PRESENT,
} FsdBodyVerdict;

/** May this action happen right now? Holds no state, latches nothing, and
 *  grants no grace period. Checked in a fixed order so a missing input can
 *  never hide the gates behind it. */
FsdBodyVerdict fsd_body_allows(const FsdBodyInputs* in, FsdBodyAction a, uint32_t now_ms);

/** 🔴 NOT THE GATE. fsd_body_allows() is the gate; this is the half of it that
 *  reads vehicle state, split out so a caller can pass a row directly.
 *
 *  It deliberately does NOT check armable_at_runtime or action_enabled, so
 *  calling it from firmware would bypass arming. Nothing in the firmware calls
 *  it and nothing should.
 *
 *  It exists because six of the seven rows are not armable yet, which would
 *  otherwise ship the park / belt / occupancy / rate gates with no test at all
 *  — and this repository's own history is that an untested gate is a wrong
 *  gate. The tests build rows the table does not contain yet and drive every
 *  branch through here. */
FsdBodyVerdict fsd_body_caps_verdict(const FsdBodyCaps* c, const FsdBodyInputs* in,
                                     FsdBodyAction a, uint32_t now_ms);

/** Human-readable verdict, for logs and the BLE surface. */
const char* fsd_body_verdict_str(FsdBodyVerdict v);

/** Human-readable action name, for the same reason. */
const char* fsd_body_action_str(FsdBodyAction a);

/** True for every CAN ID this feature could conceivably want to write.
 *
 *  UNCONDITIONAL, and deliberately not derived from the capability table: this
 *  firmware constructs no body frame at all, so the honest answer for every one
 *  of these IDs is "refused", in every state, forever. It exists so that a
 *  future call site cannot forget its gate -- a backstop, never a permission.
 *
 *  🔴 0x3C2 is NOT here, and must not be added: the scroll path already writes
 *  it under its own double gate. That is exactly why step 2 of the redesign
 *  moves this chokepoint from frame granularity to BIT granularity -- camera,
 *  seat and scroll share one frame, so "refuse the ID" and "allow the ID" are
 *  both wrong answers. */
bool fsd_body_tx_id_refused(uint32_t can_id);

#ifdef __cplusplus
}
#endif
