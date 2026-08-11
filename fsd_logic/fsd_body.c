/*
 * fsd_body.c — see fsd_body.h. One predicate, no emitters, no wire tables.
 */

#include "fsd_body.h"

#include "fsd_autonomy.h" // FSD_GEAR_*

/* The capability table. Designated initialisers so a row cannot silently land
 * at the wrong index, plus a self-check inside each row and a length assertion
 * below — three independent ways to catch the same class of mistake, because
 * this project has already shipped a permission table that defaulted open. */
static const FsdBodyCaps FSD_BODY_CAPS[] = {
    [FSD_BODY_T1_LIGHT] =
        {
            .feature = FSD_BODY_T1_LIGHT,
            /* An interior light is idempotent and its worst case is a light
             * left on, so none of the motion gates apply. What DOES apply is
             * that a drive must have happened: it stops a car that has sat
             * untouched since yesterday from doing anything at all. */
            .may_act_while_moving = true,
            .may_act_out_of_park = true,
            .may_act_without_driver = true,
            .may_act_without_drive_session = false,
            .armable_at_runtime = true,
        },
    /* T2 opens a door. Every restriction on, and NOT ARMABLE — no code path
     * may set feature_enabled for it, so the detector can run and publish while
     * the action stays structurally unreachable. An all-zero row is exactly the
     * tightest row, which is why the struct is written permissive-when-true. */
    [FSD_BODY_T2_DOOR] = {.feature = FSD_BODY_T2_DOOR},
};

_Static_assert(sizeof(FSD_BODY_CAPS) / sizeof(FSD_BODY_CAPS[0]) == FSD_BODY_FEATURE_COUNT,
               "every FsdBodyFeature needs an explicit capability row");

static bool stale(uint32_t now_ms, uint32_t stamp_ms) {
    return (uint32_t)(now_ms - stamp_ms) >= FSD_BODY_FRESH_MS;
}

FsdBodyVerdict fsd_body_allows(const FsdBodyInputs* in, FsdBodyFeature f, uint32_t now_ms) {
    if(!in) return FSD_BODY_UNKNOWN_FEATURE;
    if(f >= FSD_BODY_FEATURE_COUNT) return FSD_BODY_UNKNOWN_FEATURE;

    const FsdBodyCaps* c = &FSD_BODY_CAPS[f];
    /* Catches row drift, which the length assertion cannot: a table where two
     * rows were transposed is the right length and entirely wrong. */
    if(c->feature != f) return FSD_BODY_UNKNOWN_FEATURE;

    if(!c->armable_at_runtime) return FSD_BODY_NOT_ARMABLE;
    if(!in->feature_enabled[f]) return FSD_BODY_NOT_ENABLED;

    /* Exactly the set fsd_can_transmit() admits. Repeated rather than called so
     * this file depends on no FSDState, and narrower by construction: it can
     * never say yes where that says no. */
    if(in->op_mode != OpMode_Active && in->op_mode != OpMode_Service) return FSD_BODY_NO_MODE;

    if(!in->bus_tx_open) return FSD_BODY_BUS_SHUT;
    if(in->ota_in_progress) return FSD_BODY_OTA; // never consults ignore_ota
    if(in->rx_stale) return FSD_BODY_RX_STALE;

    if(!c->may_act_without_driver) {
        if(!in->driver_seen) return FSD_BODY_NO_DRIVER;
        if(stale(now_ms, in->driver_ms)) return FSD_BODY_DRIVER_STALE;
        if(!in->driver_present) return FSD_BODY_NO_DRIVER_PRESENT;
    }

    if(!c->may_act_without_drive_session && !in->drive_session) {
        return FSD_BODY_NO_DRIVE_SESSION;
    }

    if(!c->may_act_out_of_park) {
        if(!in->gear_seen) return FSD_BODY_NO_GEAR;
        if(stale(now_ms, in->gear_ms)) return FSD_BODY_GEAR_STALE;
        if(in->gear != FSD_GEAR_P) return FSD_BODY_NOT_PARK;
    }

    if(!c->may_act_while_moving) {
        if(!in->speed_seen) return FSD_BODY_NO_SPEED;
        if(stale(now_ms, in->speed_ms)) return FSD_BODY_SPEED_STALE;
        if(in->speed_kph > FSD_BODY_STANDSTILL_KPH) return FSD_BODY_MOVING;
    }

    return FSD_BODY_OK;
}

bool fsd_body_tx_id_refused(uint32_t can_id) {
    /* Hardcoded, not derived from the capability table, and true regardless of
     * any flag: this firmware constructs no body frame, so there is no state in
     * which putting one of these on the wire is intended. 0x3F5 is the lighting
     * frame, 0x102/0x103 the door status frames — the candidates a future
     * emitter would reach for, and the ones a mistake would reach for first. */
    return can_id == 0x3F5u || can_id == 0x102u || can_id == 0x103u;
}

const char* fsd_body_verdict_str(FsdBodyVerdict v) {
    switch(v) {
    case FSD_BODY_OK: return "ok";
    case FSD_BODY_UNKNOWN_FEATURE: return "unknown feature";
    case FSD_BODY_NOT_ARMABLE: return "not armable";
    case FSD_BODY_NOT_ENABLED: return "not enabled";
    case FSD_BODY_NO_MODE: return "mode";
    case FSD_BODY_BUS_SHUT: return "bus listen-only";
    case FSD_BODY_OTA: return "tesla updating";
    case FSD_BODY_RX_STALE: return "bus quiet";
    case FSD_BODY_NO_DRIVER: return "no driver signal";
    case FSD_BODY_DRIVER_STALE: return "driver signal stale";
    case FSD_BODY_NO_DRIVER_PRESENT: return "no driver";
    case FSD_BODY_NO_DRIVE_SESSION: return "no drive yet";
    case FSD_BODY_NO_GEAR: return "no gear";
    case FSD_BODY_GEAR_STALE: return "gear stale";
    case FSD_BODY_NOT_PARK: return "not in park";
    case FSD_BODY_NO_SPEED: return "no speed";
    case FSD_BODY_SPEED_STALE: return "speed stale";
    case FSD_BODY_MOVING: return "moving";
    }
    return "?";
}
