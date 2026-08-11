#pragma once
/*
 * fsd_autonomy.h — "is a person driving right now?"
 *
 * WHY THIS EXISTS
 * ---------------
 * The module is meant to keep responding to speed cameras when the phone is
 * not in the car. That removes the phone as a source of judgement, but it does
 * NOT remove the reason the phone was being watched in the first place: an
 * unattended module holding TX authority has nobody to notice it misbehaving.
 *
 * Those were two different questions wearing one answer. This header owns the
 * second one, and answers it from the car rather than from the link:
 *
 *     gear in {D, R}  AND  seat belt latched  AND  both signals fresh
 *
 * A phone can be at home, in a pocket, or sitting switched-on in an empty car.
 * A latched belt in a car that is in gear is direct evidence of a person in the
 * driver's seat, and both signals are already on the bus we tap.
 *
 * WHY SPEED IS NOT ONE OF THE CONDITIONS
 * --------------------------------------
 * It is tempting — "moving means someone is there". It deadlocks. Stopping at
 * a red light would drop the speed to zero, withdraw authority, and strand the
 * profile in whatever lowered state we had put it in, with no way to restore
 * it. Gear is the session marker: a car waiting at a light is still in D, and
 * so is its driver. Speed belongs to the policy layer, not to this gate.
 *
 * FAIL-CLOSED
 * -----------
 * Every unknown is a refusal. Never seen the frame, seen it too long ago,
 * gear value we do not recognise — all report "not supervised". Being wrong in
 * this direction costs a missed camera; being wrong the other way means
 * injecting into a car we know nothing about.
 *
 * NOT LATCHED
 * -----------
 * Recomputed every call. When a condition disappears, authority goes with it
 * immediately — there is no grace period, because unlike a BLE link dropping
 * for a screen blank, a belt coming undone at speed is not a transient.
 *
 * See docs 페일세이프-정책.md §3.
 */

#include "fsd_state.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// DI_gear (0x286, party bus). opendbc tesla_model3_party.dbc:
//   VAL_ 646 DI_gear 0 "INVALID" 1 "P" 2 "R" 3 "N" 4 "D" 7 "SNA"
#define FSD_GEAR_INVALID 0u
#define FSD_GEAR_P 1u
#define FSD_GEAR_R 2u
#define FSD_GEAR_N 3u
#define FSD_GEAR_D 4u
#define FSD_GEAR_SNA 7u

/* How stale a supervision signal may be before it stops counting as evidence.
 * 0x286 and 0x311 both run well above 1 Hz, so this is loose enough to ride out
 * a dropped frame and tight enough that a gateway going quiet is caught within
 * one second. Same shape as FSD_RX_STALE_MS and NAG_CTX_FRESH_MS. */
#define FSD_DRIVE_CTX_FRESH_MS 1000u

/* Which gate refused. Returned rather than logged so the caller can put the
 * reason on the BLE Result characteristic — "autonomy is not acting" is
 * useless to a user who cannot see which condition is missing. */
typedef enum {
    FSD_SUP_OK = 0,
    FSD_SUP_NO_GEAR,        // 0x286 never parsed
    FSD_SUP_GEAR_STALE,
    FSD_SUP_GEAR_NOT_DRIVE, // P / N / INVALID / SNA
    FSD_SUP_NO_BELT,        // 0x311 never parsed
    FSD_SUP_BELT_STALE,
    FSD_SUP_BELT_UNLATCHED,
    FSD_SUP_RX_STALE,       // bus is quiet: we cannot hear what we would change
    FSD_SUP_OTA,            // Tesla is updating
} FsdSupVerdict;

/** Read DI_gear out of a 0x286 DI_state frame and stamp it.
 *
 *  Deliberately separate from the existing fsd_handle_di_state(): that one is
 *  upstream's general parser and is not compiled into the ESP32 build at all,
 *  which is why the firmware has never had a gear reading. This one is small,
 *  shared by both builds, and writes only the fields the supervision gate owns.
 *  Ignores frames too short to contain the signal, and does not stamp them —
 *  stamping at the call site would mark a rejected frame as fresh. */
void fsd_drive_observe_gear(FSDState* state, const CANFRAME* frame, uint32_t now_ms);

/** Read buckleStatus out of a 0x311 UI_warning frame and stamp it.
 *  Writes the same ui_buckle_status the Flipper parser writes; on that build
 *  both run and agree, on the ESP32 this is the only writer. */
void fsd_drive_observe_belt(FSDState* state, const CANFRAME* frame, uint32_t now_ms);

/** Why supervision is (not) satisfied. FSD_SUP_OK means a person is driving. */
FsdSupVerdict fsd_supervised_drive_why(const FSDState* state, uint32_t now_ms);

/** True when a person is demonstrably driving. Convenience over _why(). */
bool fsd_supervised_drive(const FSDState* state, uint32_t now_ms);

/** Human-readable verdict, for logs and the BLE Result characteristic. */
const char* fsd_sup_verdict_str(FsdSupVerdict v);

// ── operating mode floor ─────────────────────────────────────────────────────

/** The lowest mode this module may sit in when nothing else has raised it.
 *
 *  Used in two places that used to hardcode Listen-Only: the boot default, and
 *  the point where a dropped BLE link hands Active back. Both now land on
 *  Autonomous when the operator has enabled it, which is what "the module keeps
 *  working when the phone is left behind" actually means in code.
 *
 *  Falling to Autonomous is not a weakening of either guard. It grants no
 *  general TX — fsd_can_transmit() is false there — and the camera path still
 *  has to satisfy fsd_supervised_drive(). A car sitting in a garage at 3 a.m.
 *  reaches exactly the same place it did before: unable to transmit anything. */
OpMode fsd_autonomy_floor(const FSDState* state);

/** The camera path's only door. Everything it needs, in one predicate:
 *  the operator enabled it, the mode allows it, and a person is driving.
 *
 *  Active is accepted alongside Autonomous so a phone-connected drive behaves
 *  the same way — otherwise connecting the app would silently switch the
 *  feature off, which is the opposite of what a user would expect. */
bool fsd_autonomy_allows(const FSDState* state, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
