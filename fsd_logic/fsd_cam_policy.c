/*
 * fsd_cam_policy.c — see fsd_cam_policy.h.
 */

#include "fsd_cam_policy.h"

#include <string.h>

/* Posted limit -> target profile.
 *
 * The reference implementation this project benchmarks against simply pinned
 * Standard everywhere and reported that as sufficient. We know from the
 * nationwide database that 30 km/h zones are 53% of all cameras, so the slowest
 * band gets one step lower; nothing goes above Standard, because a camera is
 * never a reason to go faster.
 *
 * All of it is assumption until the profiles are measured on the road. */
static const struct {
    uint8_t max_kph;
    uint8_t profile;
} PROFILE_FOR_LIMIT[] = {
    {30u, FSD_POL_PROFILE_CHILL},
    {60u, FSD_POL_PROFILE_STANDARD},
};

uint8_t fsd_pol_profile_for_limit(uint8_t limit_kph) {
    for(unsigned i = 0; i < sizeof(PROFILE_FOR_LIMIT) / sizeof(PROFILE_FOR_LIMIT[0]);
        i++) {
        if(limit_kph <= PROFILE_FOR_LIMIT[i].max_kph) return PROFILE_FOR_LIMIT[i].profile;
    }
    return FSD_POL_PROFILE_STANDARD;
}

float fsd_pol_lead_distance_m(float speed_kph) {
    float v = speed_kph > 0.0f ? speed_kph : 0.0f;
    float by_time = v / 3.6f * FSD_POL_LEAD_TIME_S;
    if(by_time < FSD_POL_MIN_LEAD_M) by_time = FSD_POL_MIN_LEAD_M;
    if(by_time > FSD_POL_MAX_LEAD_M) by_time = FSD_POL_MAX_LEAD_M;
    return by_time;
}

void fsd_pol_init(FsdPolicy* p) {
    if(!p) return;
    memset(p, 0, sizeof(*p));
    p->phase = FSD_POL_IDLE;
}

const char* fsd_pol_phase_str(FsdPolPhase ph) {
    switch(ph) {
    case FSD_POL_IDLE: return "idle";
    case FSD_POL_ARMED: return "armed";
    case FSD_POL_ACTIVE: return "active";
    case FSD_POL_HOLDING: return "holding";
    case FSD_POL_RESTORING: return "restoring";
    default: return "unknown";
    }
}

static void clear_target(FsdPolicy* p) {
    p->has_target = false;
    p->key = 0;
    p->limit_kph = 0;
    p->distance_m = 0.0f;
    p->passed_m = 0.0f;
    p->held_s = 0.0f;
}

void fsd_pol_abandon(FsdPolicy* p) {
    if(!p) return;
    /* Forget the entry profile deliberately. Putting back a value the driver
     * has since moved away from would be us overruling them, and an override is
     * exactly when this gets called. */
    clear_target(p);
    p->entry_valid = false;
    p->entry_profile = 0;
    p->restore_s = 0.0f;
    p->requested_valid = false;
    p->phase = FSD_POL_IDLE;
}

void fsd_pol_on_pass(FsdPolicy* p, uint64_t key) {
    if(!p || !p->has_target || p->key != key) return;

    if(p->phase == FSD_POL_HOLDING) return; // already counting it out
    if(p->phase == FSD_POL_ACTIVE) {
        p->phase = FSD_POL_HOLDING;
        p->passed_m = 0.0f;
        p->held_s = 0.0f;
        return;
    }
    /* Never lowered for it, so there is nothing to hold. The entry profile is
     * left alone: if one is on file it belongs to an earlier camera whose
     * restore is still owed. */
    clear_target(p);
    if(p->phase != FSD_POL_RESTORING) p->phase = FSD_POL_IDLE;
}

void fsd_pol_new_drive(FsdPolicy* p) {
    if(!p) return;
    p->overrides = 0;
    p->failures = 0;
    p->suspended = false;
    p->observed_valid = false;
    p->requested_valid = false;
}

bool fsd_pol_suspended(const FsdPolicy* p) {
    return p && p->suspended;
}

void fsd_pol_on_convergence_failed(FsdPolicy* p) {
    if(!p) return;
    if(p->failures < 0xFFu) p->failures++;
    if(p->failures >= FSD_POL_MAX_FAILURES) {
        p->suspended = true;
        fsd_pol_abandon(p);
    }
}

void fsd_pol_on_convergence_ok(FsdPolicy* p) {
    if(!p) return;
    p->failures = 0;
}

/* Distance from a profile value to the one we asked for. Plain magnitude — the
 * profiles are an ordered scale, so "how many steps away" is all that matters. */
static uint8_t steps_from(uint8_t v, uint8_t target) {
    return (uint8_t)(v > target ? v - target : target - v);
}

bool fsd_pol_observe_profile(FsdPolicy* p, uint8_t observed) {
    if(!p) return false;

    uint8_t prev = p->last_observed;
    bool had_prev = p->observed_valid;
    p->last_observed = observed;
    p->observed_valid = true;

    if(!had_prev || prev == observed) return false; // nothing moved
    if(!p->requested_valid) return false;           // we are not asking for anything

    /* Closer to what we asked for: that is our own convergence stepping, and a
     * multi-step request legitimately passes through values that are not the
     * target. Only movement AWAY from the request is a second hand on the
     * wheel. Overshoot counts as away too — if the car sailed past the value we
     * asked for, something is wrong and letting go is the safe response. */
    if(steps_from(observed, p->requested) < steps_from(prev, p->requested)) return false;

    if(p->overrides < 0xFFu) p->overrides++;
    fsd_pol_abandon(p); // forgets the entry profile: the driver's value is theirs now
    if(p->overrides >= FSD_POL_MAX_OVERRIDES) p->suspended = true;
    return true;
}

static FsdPolDecision decide(FsdPolicy* p, FsdPolAction act, uint8_t profile,
                             float trigger_m) {
    /* Remember what we asked for. fsd_pol_observe_profile() needs it to tell
     * our own convergence from the driver's hand, and recording it here means
     * every exit from fsd_pol_tick() is covered — a return added later cannot
     * forget to do it. */
    p->requested_valid = (act != FSD_POL_ACT_NONE);
    p->requested = profile;

    FsdPolDecision d;
    memset(&d, 0, sizeof(d));
    d.phase = p->phase;
    d.action = act;
    d.target_profile = profile;
    d.camera_key = p->has_target ? p->key : 0;
    d.limit_kph = p->has_target ? p->limit_kph : 0;
    d.distance_m = p->has_target ? p->distance_m : 0.0f;
    d.trigger_m = trigger_m;
    return d;
}

/* Never raise. A camera response that makes the car faster is a defect, not a
 * trade-off, so the request is clamped against what the car is actually on. */
static uint8_t lower_only(uint8_t want, uint8_t observed) {
    return want > observed ? observed : want;
}

FsdPolDecision fsd_pol_tick(FsdPolicy* p, const FsdPolTarget* ahead,
                            uint8_t observed_profile, float speed_kph,
                            float moved_m, float dt_s) {
    FsdPolDecision none;
    if(!p) {
        memset(&none, 0, sizeof(none));
        return none;
    }
    if(moved_m < 0.0f) moved_m = 0.0f;
    if(dt_s < 0.0f) dt_s = 0.0f;

    /* Suspended: the driver has corrected us repeatedly, or the car has stopped
     * responding to the scroll. Either way, keep quiet until the next drive
     * rather than spending the whole one arguing. */
    if(p->suspended) {
        p->phase = FSD_POL_IDLE;
        return decide(p, FSD_POL_ACT_NONE, 0, 0.0f);
    }

    // ── holding past a camera ────────────────────────────────────────────────
    // Runs before anything else and cannot be interrupted by a new camera.
    // Cutting the hold short is the one thing the hold exists to prevent.
    if(p->phase == FSD_POL_HOLDING) {
        p->passed_m += moved_m;
        p->held_s += dt_s;

        bool far_enough = p->passed_m >= FSD_POL_RELEASE_DIST_M;
        bool long_enough = p->held_s >= FSD_POL_RELEASE_MIN_S;
        if(!(far_enough && long_enough)) {
            uint8_t want = lower_only(fsd_pol_profile_for_limit(p->limit_kph),
                                      observed_profile);
            return decide(p, FSD_POL_ACT_LOWER, want, 0.0f);
        }

        clear_target(p);
        if(p->entry_valid) {
            p->phase = FSD_POL_RESTORING;
            p->restore_s = 0.0f;
        } else {
            p->phase = FSD_POL_IDLE;
        }
        // Falls through: the same tick can adopt a camera that is already
        // waiting, so one is never lost to the hold.
    }

    // ── putting the profile back ─────────────────────────────────────────────
    if(p->phase == FSD_POL_RESTORING) {
        p->restore_s += dt_s;

        if(observed_profile == p->entry_profile) { // arrived
            p->entry_valid = false;
            p->phase = FSD_POL_IDLE;
        } else if(p->restore_s >= FSD_POL_RESTORE_TIMEOUT_S) {
            /* The car is not taking it. Stop asking — a request repeated
             * forever is indistinguishable from a fault, and the driver can see
             * the profile on the screen. */
            p->entry_valid = false;
            p->phase = FSD_POL_IDLE;
        } else if(ahead && ahead->valid) {
            /* A camera ahead outranks a restore — we are about to lower again
             * anyway. The entry profile SURVIVES: it is the value the driver
             * chose, and the next camera must not overwrite it with the value
             * we ourselves lowered to, or a chain of cameras would leave the
             * car permanently slowed with nothing to restore towards. */
            p->phase = FSD_POL_IDLE;
        } else {
            return decide(p, FSD_POL_ACT_RESTORE, p->entry_profile, 0.0f);
        }
    }

    // ── adopt / follow the nearest camera ────────────────────────────────────
    if(ahead && ahead->valid) {
        if(!p->has_target || ahead->key != p->key || ahead->distance_m < p->distance_m) {
            if(!p->has_target || ahead->key != p->key) {
                p->key = ahead->key;
                p->limit_kph = ahead->limit_kph;
                p->has_target = true;
                if(p->phase == FSD_POL_IDLE) p->phase = FSD_POL_ARMED;
            }
            p->distance_m = ahead->distance_m;
        } else {
            p->distance_m = ahead->distance_m;
            p->limit_kph = ahead->limit_kph;
        }
    } else if(p->phase != FSD_POL_ACTIVE) {
        /* Nothing ahead and we are not holding anything down: idle. When we ARE
         * active the camera stays adopted — losing it from the scan is not the
         * same as having passed it, and only fsd_pol_on_pass() may end that. */
        clear_target(p);
        p->phase = FSD_POL_IDLE;
        return decide(p, FSD_POL_ACT_NONE, 0, 0.0f);
    }

    if(!p->has_target) {
        p->phase = FSD_POL_IDLE;
        return decide(p, FSD_POL_ACT_NONE, 0, 0.0f);
    }

    float trigger = fsd_pol_lead_distance_m(speed_kph);

    // Standing still is not an approach. Anything already lowered stays that
    // way — a queue at a school-zone camera is the last place to speed up.
    if(speed_kph < FSD_POL_MIN_SPEED_KPH) {
        if(p->phase == FSD_POL_ACTIVE) {
            uint8_t want = lower_only(fsd_pol_profile_for_limit(p->limit_kph),
                                      observed_profile);
            return decide(p, FSD_POL_ACT_LOWER, want, trigger);
        }
        return decide(p, FSD_POL_ACT_NONE, 0, trigger);
    }

    if(p->distance_m <= trigger) {
        if(p->phase != FSD_POL_ACTIVE) {
            /* Capture what the driver had, but only if we do not already owe a
             * restore. The moment of first engagement is the only time the
             * observed profile is still theirs; one tick later it is whatever
             * we set, and on a road with cameras in sequence that value would
             * ratchet the "original" downward on every one of them. */
            if(!p->entry_valid) {
                p->entry_profile = observed_profile;
                p->entry_valid = true;
            }
            p->phase = FSD_POL_ACTIVE;
        }
        uint8_t want =
            lower_only(fsd_pol_profile_for_limit(p->limit_kph), observed_profile);
        return decide(p, FSD_POL_ACT_LOWER, want, trigger);
    }

    p->phase = FSD_POL_ARMED;
    return decide(p, FSD_POL_ACT_NONE, 0, trigger);
}
