/*
 * fsd_body.c — see fsd_body.h. One predicate, no emitters, no wire tables.
 */

#include "fsd_body.h"

#include "fsd_autonomy.h" // FSD_GEAR_*

/* The capability table. Designated initialisers so a row cannot silently land
 * at the wrong index, plus a self-check inside each row and a length assertion
 * below — three independent ways to catch the same class of mistake, because
 * this project has already shipped a permission table that defaulted open.
 *
 * 🔴 EXACTLY ONE ROW IS ARMABLE. MAP_LIGHT carries over unchanged from the old
 * T1; every other row is armable_at_runtime = false, so rewriting this table
 * opened nothing. Each row names the evidence that flips its bool. */
static const FsdBodyCaps FSD_BODY_CAPS[] = {
    [FSD_ACT_MAP_LIGHT] =
        {
            .action = FSD_ACT_MAP_LIGHT,
            /* An interior light is idempotent and its worst case is a light
             * left on, so none of the motion gates apply. What DOES apply is
             * that a drive must have happened: it stops a car that has sat
             * untouched since yesterday from doing anything at all. */
            .may_act_while_moving = true,
            .may_act_out_of_park = true,
            .may_act_without_driver = true,
            .may_act_without_drive_session = false,
            .armable_at_runtime = true,
            .min_interval_ms = 500u,
            /* TSL holds the light by re-sending; whether we must too is not
             * settled (see 권한축-재설계.md 8-D), so the bound exists before the
             * emitter does rather than after it misbehaves. */
            .max_hold_ms = 30000u,
        },

    /* Opening a door has no undo and no upper bound on consequences. Every
     * restriction on, and NOT ARMABLE — no code path may enable it, so a
     * detector can run and publish while the action stays structurally
     * unreachable. An all-zero row is exactly the tightest row, which is why
     * the struct is written permissive-when-true.
     * FLIPS WHEN: the unfiltered capture identifies the actual command frame. */
    [FSD_ACT_DOOR_OPEN] = {.action = FSD_ACT_DOOR_OPEN},

    [FSD_ACT_CAMERA] =
        {
            .action = FSD_ACT_CAMERA,
            /* Owner decision 2026-09-01: both on and off. The risk of "off" is
             * not that it is hard to undo — it is that nobody notices for
             * hours. That is answered by logging and by showing the state on
             * the dashboard, not by a gate. */
            .may_act_while_moving = true,
            .may_act_out_of_park = true,
            .may_act_without_driver = true,
            .armable_at_runtime = false, // FLIPS WHEN: step 3, first write test
            .min_interval_ms = 500u,
        },

    [FSD_ACT_SEAT_DRIVER] =
        {
            .action = FSD_ACT_SEAT_DRIVER,
            /* Owner decision 2026-09-01: moving as well. A Tesla driver can
             * already adjust their own seat under way, so this is not a new
             * capability for the car — but a stuck rule driving a motor is, and
             * that is what max_hold_ms bounds. No occupancy check: the person
             * in this seat is the person asking. */
            .may_act_while_moving = true,
            .may_act_out_of_park = true,
            .armable_at_runtime = false, // FLIPS WHEN: step 3
            .min_interval_ms = 500u,
            .max_hold_ms = 1000u,
        },

    [FSD_ACT_SEAT_PASSENGER] =
        {
            .action = FSD_ACT_SEAT_PASSENGER,
            .may_act_while_moving = true,
            .may_act_out_of_park = true,
            .armable_at_runtime = false, // FLIPS WHEN: step 3
            /* The difference from the driver's seat, and the whole reason these
             * are two rows: nobody asked on behalf of the passenger. */
            .requires_passenger_empty = true,
            .min_interval_ms = 500u,
            .max_hold_ms = 1000u,
        },

    [FSD_ACT_SCROLL] =
        {
            .action = FSD_ACT_SCROLL,
            /* The detent the whole safety story was originally written around.
             * Its own path (fsd_speed_profile.c) keeps a separate double gate;
             * this row does not replace it. */
            .may_act_while_moving = true,
            .may_act_out_of_park = true,
            .armable_at_runtime = false, // FLIPS WHEN: encoding measured + armed
            .min_interval_ms = 150u,
        },

    [FSD_ACT_GEAR_D] =
        {
            .action = FSD_ACT_GEAR_D,
            /* Owner decision 2026-09-01: adopted, replicating what TSL already
             * does in this car (belt latched -> D). Every gate this axis has,
             * plus two of its own.
             *
             * 🟢 The load-bearing interlock is not ours: Tesla will not leave P
             * without the brake pressed, and our frame is a stalk REQUEST that
             * the drive controller still judges. We cannot bypass it.
             * 🔴 We have also never verified that. All three captures had the
             * brake down because that is what the procedure said. Until the
             * no-brake case is measured, this row stays unarmable — borrowed
             * safety you have not seen is safety you are only assuming.
             *
             * FLIPS WHEN: the no-brake refusal is observed on the car. */
            .armable_at_runtime = false,
            .requires_park = true, // P -> D and nothing else
            .requires_belt = true,
            .min_interval_ms = 1000u,
        },
};

_Static_assert(sizeof(FSD_BODY_CAPS) / sizeof(FSD_BODY_CAPS[0]) == FSD_ACT_COUNT,
               "every FsdBodyAction needs an explicit capability row");

static bool stale(uint32_t now_ms, uint32_t stamp_ms) {
    return (uint32_t)(now_ms - stamp_ms) >= FSD_BODY_FRESH_MS;
}

const FsdBodyCaps* fsd_body_caps(FsdBodyAction a) {
    if(a >= FSD_ACT_COUNT) return NULL;
    const FsdBodyCaps* c = &FSD_BODY_CAPS[a];
    if(c->action != a) return NULL;
    return c;
}

FsdBodyVerdict fsd_body_allows(const FsdBodyInputs* in, FsdBodyAction a, uint32_t now_ms) {
    if(!in) return FSD_BODY_UNKNOWN_ACTION;

    /* Catches row drift, which the length assertion cannot: a table where two
     * rows were transposed is the right length and entirely wrong. */
    const FsdBodyCaps* c = fsd_body_caps(a);
    if(!c) return FSD_BODY_UNKNOWN_ACTION;

    /* The two arming gates live HERE and not in fsd_body_caps_verdict(), so
     * that helper can never be the thing that grants an action. */
    if(!c->armable_at_runtime) return FSD_BODY_NOT_ARMABLE;
    if(!in->action_enabled[a]) return FSD_BODY_NOT_ENABLED;

    return fsd_body_caps_verdict(c, in, a, now_ms);
}

FsdBodyVerdict fsd_body_caps_verdict(const FsdBodyCaps* c, const FsdBodyInputs* in,
                                     FsdBodyAction a, uint32_t now_ms) {
    if(!c || !in || a >= FSD_ACT_COUNT) return FSD_BODY_UNKNOWN_ACTION;

    /* Exactly the set fsd_can_transmit() admits. Repeated rather than called so
     * this file depends on no FSDState, and narrower by construction: it can
     * never say yes where that says no. */
    if(in->op_mode != OpMode_Active && in->op_mode != OpMode_Service) return FSD_BODY_NO_MODE;

    if(!in->bus_tx_open) return FSD_BODY_BUS_SHUT;
    if(in->ota_in_progress) return FSD_BODY_OTA; // never consults ignore_ota
    if(in->rx_stale) return FSD_BODY_RX_STALE;

    /* Before any vehicle-state gate, because it needs no input that could be
     * missing: a row with min_interval_ms == 0 may never fire at all, and that
     * has to be answerable even on a car that is telling us nothing. */
    if(c->min_interval_ms == 0u) return FSD_BODY_TOO_SOON;
    if((uint32_t)(now_ms - in->last_act_ms[a]) < c->min_interval_ms) return FSD_BODY_TOO_SOON;

    if(!c->may_act_without_driver) {
        if(!in->driver_seen) return FSD_BODY_NO_DRIVER;
        if(stale(now_ms, in->driver_ms)) return FSD_BODY_DRIVER_STALE;
        if(!in->driver_present) return FSD_BODY_NO_DRIVER_PRESENT;
    }

    if(!c->may_act_without_drive_session && !in->drive_session) {
        return FSD_BODY_NO_DRIVE_SESSION;
    }

    /* Two different questions about the gear, and they are not the same gate.
     * may_act_out_of_park asks "may this happen anywhere but P"; requires_park
     * asks "is P the starting point this action transitions FROM". Gear
     * selection needs the second and would pass the first. */
    if(!c->may_act_out_of_park || c->requires_park) {
        if(!in->gear_seen) return FSD_BODY_NO_GEAR;
        if(stale(now_ms, in->gear_ms)) return FSD_BODY_GEAR_STALE;
        if(in->gear != FSD_GEAR_P) return FSD_BODY_NOT_PARK;
    }

    if(c->requires_belt) {
        if(!in->belt_seen) return FSD_BODY_NO_BELT;
        if(stale(now_ms, in->belt_ms)) return FSD_BODY_BELT_STALE;
        if(!in->belt_latched) return FSD_BODY_NO_BELT;
    }

    if(c->requires_passenger_empty) {
        /* Fail-closed on silence: not hearing the seat is not the same as an
         * empty seat, and this is the gate whose failure puts a motor against
         * a person. */
        if(!in->passenger_seen) return FSD_BODY_NO_PASSENGER_SIGNAL;
        if(stale(now_ms, in->passenger_ms)) return FSD_BODY_NO_PASSENGER_SIGNAL;
        if(in->passenger_present) return FSD_BODY_PASSENGER_PRESENT;
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
     * any flag. 0x3F5 is the lighting frame, 0x102/0x103 the door status frames.
     *
     * 🔴 0x3F5 WAS OUR MAP-LIGHT CANDIDATE AND IT WAS WRONG (2026-09-03).
     * TSL turns the map lights on with 0x273, not 0x3F5 — six captures agree.
     * The refusal stays anyway: nothing constructs 0x3F5, so refusing it costs
     * nothing and it remains exactly what a mistake would reach for first.
     *
     * 🔴 0x273 IS DELIBERATELY NOT ON THIS LIST. It is the frame the map
     * light emitter builds, so refusing it here would refuse the one thing this
     * axis exists to allow. Its protection is the other three layers — the rule
     * has to match, the axis has to allow, and today nothing calls the emitter
     * at all. If that ever feels thin, the answer is a narrower gate on 0x273
     * (bit 59 only, as fsd_body_wire.c already does for 0x3C2), not a blanket
     * refusal that would make the feature impossible. */
    return can_id == 0x3F5u || can_id == 0x102u || can_id == 0x103u;
}

const char* fsd_body_action_str(FsdBodyAction a) {
    switch(a) {
    case FSD_ACT_MAP_LIGHT: return "map-light";
    case FSD_ACT_DOOR_OPEN: return "door-open";
    case FSD_ACT_CAMERA: return "camera";
    case FSD_ACT_SEAT_DRIVER: return "seat-driver";
    case FSD_ACT_SEAT_PASSENGER: return "seat-passenger";
    case FSD_ACT_SCROLL: return "scroll";
    case FSD_ACT_GEAR_D: return "gear-D";
    case FSD_ACT_COUNT: break;
    }
    return "?";
}

const char* fsd_body_verdict_str(FsdBodyVerdict v) {
    switch(v) {
    case FSD_BODY_OK: return "ok";
    case FSD_BODY_UNKNOWN_ACTION: return "unknown action";
    case FSD_BODY_NOT_ARMABLE: return "not armable";
    case FSD_BODY_NOT_ENABLED: return "not enabled";
    case FSD_BODY_NO_MODE: return "mode";
    case FSD_BODY_BUS_SHUT: return "bus listen-only";
    case FSD_BODY_OTA: return "tesla updating";
    case FSD_BODY_RX_STALE: return "bus quiet";
    case FSD_BODY_TOO_SOON: return "too soon";
    case FSD_BODY_NO_DRIVER: return "no driver signal";
    case FSD_BODY_DRIVER_STALE: return "driver signal stale";
    case FSD_BODY_NO_DRIVER_PRESENT: return "no driver";
    case FSD_BODY_NO_DRIVE_SESSION: return "no drive since arming";
    case FSD_BODY_NO_GEAR: return "no gear signal";
    case FSD_BODY_GEAR_STALE: return "gear signal stale";
    case FSD_BODY_NOT_PARK: return "not in park";
    case FSD_BODY_NO_SPEED: return "no speed signal";
    case FSD_BODY_SPEED_STALE: return "speed signal stale";
    case FSD_BODY_MOVING: return "moving";
    case FSD_BODY_NO_BELT: return "belt not latched";
    case FSD_BODY_BELT_STALE: return "belt signal stale";
    case FSD_BODY_NO_PASSENGER_SIGNAL: return "no passenger signal";
    case FSD_BODY_PASSENGER_PRESENT: return "passenger seated";
    }
    return "?";
}
