/*
 * fsd_speed_profile.c — see fsd_speed_profile.h for the rationale.
 *
 * The machine is deliberately boring: every exit is bounded, every
 * precondition is re-checked on each poll (they can vanish mid-flight — the
 * car can start an OTA, the link can drop to Listen-Only), and nothing is
 * emitted unless the operator armed it AND the encoding table has been
 * confirmed against a real capture.
 */

#include "fsd_speed_profile.h"

#include <string.h>

// Provisional. `verified = false` is what actually gates transmission — see
// fsd_sp_request(). Fix these three fields from a capture, flip verified, and
// the state machine needs no changes.
const FsdSpEncoding FSD_SP_ENCODING_DEFAULT = {
    .tick_toward_higher = 1, // assumption: +1 detent raises the profile
    .ticks_per_step = 1,
    .wrap = false, // assumption: the ends stop rather than wrap
    .verified = false,
};

// swcRightScrollTicks: 0x3C2 mux=1, byte 3 bits 0-5, 6-bit signed.
#define SCROLL_TICKS_BYTE 3u
#define SCROLL_TICKS_MASK 0x3Fu
#define SCROLL_MUX_BYTE 0u
#define SCROLL_MUX_MASK 0x03u
#define SCROLL_MUX_SWITCHES 1u

void fsd_sp_init(FsdSpeedProfile* sp) {
    if(!sp) return;
    memset(sp, 0, sizeof(*sp));
    sp->enc = FSD_SP_ENCODING_DEFAULT;
    sp->phase = FSD_SP_IDLE;
    sp->last_error = FSD_SP_OK;
}

bool fsd_sp_busy(const FsdSpeedProfile* sp) {
    return sp && (sp->phase == FSD_SP_STEP || sp->phase == FSD_SP_SETTLE);
}

void fsd_sp_abort(FsdSpeedProfile* sp, FsdSpError why, uint32_t now_ms) {
    if(!sp) return;
    sp->phase = (why == FSD_SP_OK) ? FSD_SP_DONE : FSD_SP_FAILED;
    sp->last_error = why;
    sp->pending_ticks = 0;
    sp->finished_ms = now_ms;
}

// The encoding table is filled in by a tool from a capture, so treat it as
// input, not as a constant. A value that cannot be expressed in the 6-bit
// signed field would be masked rather than clamped — +40 reads back as -24,
// i.e. the module would scroll the opposite way from what the table says.
bool fsd_sp_encoding_ok(const FsdSpEncoding* e) {
    if(!e) return false;
    if(!e->verified) return false;
    if(e->tick_toward_higher == 0) return false;
    if(e->tick_toward_higher > FSD_SP_DETENT_MAX) return false;
    if(e->tick_toward_higher < FSD_SP_DETENT_MIN) return false;
    // A step that cannot fit in the budget could never complete, and the budget
    // is now spent per whole step.
    if(e->ticks_per_step == 0 || e->ticks_per_step > FSD_SP_MAX_TICKS) return false;
    return true;
}

// Preconditions shared by request() and every poll(). Order matters only for
// which reason the caller sees first; all of them are hard gates.
static FsdSpError check_inputs(const FsdSpeedProfile* sp, const FsdSpInputs* in) {
    if(!in) return FSD_SP_ERR_ABORTED;
    if(in->listen_only) return FSD_SP_ERR_LISTEN_ONLY;
    if(!in->tx_armed) return FSD_SP_ERR_NOT_ARMED;
    if(!fsd_sp_encoding_ok(&sp->enc)) return FSD_SP_ERR_UNVERIFIED;
    if(in->ota_in_progress) return FSD_SP_ERR_OTA;
    if(!in->scroll_bus_present) return FSD_SP_ERR_NO_SCROLL_BUS;
    if(!in->status_fresh) return FSD_SP_ERR_NO_STATE;
    return FSD_SP_OK;
}

FsdSpError fsd_sp_request(FsdSpeedProfile* sp, const FsdSpInputs* in,
                          uint8_t target, uint32_t now_ms) {
    if(!sp) return FSD_SP_ERR_ABORTED;

    if(target > FSD_SP_PROFILE_MAX) {
        sp->last_error = FSD_SP_ERR_RANGE;
        return sp->last_error;
    }
    if(fsd_sp_busy(sp)) {
        sp->last_error = FSD_SP_ERR_BUSY;
        return sp->last_error;
    }
    // Cooldown runs from the end of the previous request, not its start.
    if((sp->phase == FSD_SP_DONE || sp->phase == FSD_SP_FAILED) &&
       (uint32_t)(now_ms - sp->finished_ms) < FSD_SP_COOLDOWN_MS) {
        sp->last_error = FSD_SP_ERR_COOLDOWN;
        return sp->last_error;
    }

    FsdSpError e = check_inputs(sp, in);
    if(e != FSD_SP_OK) {
        sp->last_error = e;
        return e;
    }
    if(in->observed_profile > FSD_SP_PROFILE_MAX) {
        sp->last_error = FSD_SP_ERR_NO_STATE;
        return sp->last_error;
    }

    sp->target = target;
    sp->observed = in->observed_profile;
    sp->start_profile = in->observed_profile;
    sp->ticks_used = 0;
    sp->pending_ticks = 0;
    sp->stalls = 0;
    sp->started_ms = now_ms;
    sp->phase_ms = now_ms;
    sp->last_error = FSD_SP_OK;

    // Already there: finish immediately rather than emitting a no-op tick.
    if(sp->observed == target) {
        sp->phase = FSD_SP_DONE;
        sp->finished_ms = now_ms;
    } else {
        sp->phase = FSD_SP_STEP;
    }
    return FSD_SP_OK;
}

void fsd_sp_observe(FsdSpeedProfile* sp, uint8_t profile, uint32_t now_ms) {
    if(!sp || profile > FSD_SP_PROFILE_MAX) return;
    if(sp->observed == profile) return;

    sp->observed = profile;
    // The car moved. Stop waiting and re-decide on the next poll.
    if(sp->phase == FSD_SP_SETTLE && sp->pending_ticks == 0) {
        sp->phase = FSD_SP_STEP;
        sp->phase_ms = now_ms;
        sp->stalls = 0;
    }
}

// Which way to turn. Without wrap this is just the sign of the difference;
// with wrap, take the shorter way around the ring.
static FsdSpAction direction(const FsdSpeedProfile* sp) {
    int diff = (int)sp->target - (int)sp->observed;
    if(!sp->enc.wrap) {
        return (diff > 0) ? FSD_SP_ACT_TICK_UP : FSD_SP_ACT_TICK_DOWN;
    }
    int n = (int)FSD_SP_PROFILE_COUNT;
    int up = ((diff % n) + n) % n; // steps going up to reach target
    return (up <= n - up) ? FSD_SP_ACT_TICK_UP : FSD_SP_ACT_TICK_DOWN;
}

// Emit one detent, charging it against the budget.
static FsdSpAction emit(FsdSpeedProfile* sp, FsdSpAction act, uint32_t now_ms) {
    sp->ticks_used++;
    if(sp->pending_ticks > 0) sp->pending_ticks--;
    // Only wait for the car once the whole step has been sent.
    sp->phase = (sp->pending_ticks > 0) ? FSD_SP_STEP : FSD_SP_SETTLE;
    sp->phase_ms = now_ms;
    return act;
}

FsdSpAction fsd_sp_poll(FsdSpeedProfile* sp, const FsdSpInputs* in,
                        uint32_t now_ms) {
    if(!fsd_sp_busy(sp)) return FSD_SP_ACT_NONE;

    // Converged — settle this FIRST. The car is already where it was asked to
    // be, and nothing discovered later in this poll undoes that. Checking it
    // after the timeout or the precondition re-check would report a request
    // that succeeded as FAILED whenever an OTA (say) starts in the same tick.
    if(sp->observed == sp->target && sp->pending_ticks == 0) {
        sp->phase = FSD_SP_DONE;
        sp->finished_ms = now_ms;
        sp->last_error = FSD_SP_OK;
        return FSD_SP_ACT_NONE;
    }

    if((uint32_t)(now_ms - sp->started_ms) >= FSD_SP_TIMEOUT_MS) {
        fsd_sp_abort(sp, FSD_SP_ERR_TIMEOUT, now_ms);
        return FSD_SP_ACT_NONE;
    }
    // Preconditions can disappear mid-flight; treat that as an abort, not a
    // pause, so a half-finished sequence never resumes against a changed car.
    if(check_inputs(sp, in) != FSD_SP_OK) {
        fsd_sp_abort(sp, FSD_SP_ERR_ABORTED, now_ms);
        return FSD_SP_ACT_NONE;
    }

    if(sp->phase == FSD_SP_SETTLE) {
        if((uint32_t)(now_ms - sp->phase_ms) < FSD_SP_SETTLE_MS) {
            return FSD_SP_ACT_NONE; // still giving the car time
        }
        sp->stalls++;
        if(sp->stalls >= FSD_SP_STALL_LIMIT) {
            fsd_sp_abort(sp, FSD_SP_ERR_STALLED, now_ms);
            return FSD_SP_ACT_NONE;
        }
        sp->phase = FSD_SP_STEP;
        sp->phase_ms = now_ms;
    }

    // Mid-step: the budget for these detents was reserved when the step began,
    // so finish it. Re-checking the ceiling here is what used to cut a step in
    // half and hand the car a partial turn.
    if(sp->pending_ticks > 0) {
        return emit(sp, direction(sp), now_ms);
    }

    // A step is atomic: reserve its whole cost up front or do not start it.
    uint8_t step = (sp->enc.ticks_per_step > 0) ? sp->enc.ticks_per_step : 1u;
    if((uint32_t)sp->ticks_used + step > FSD_SP_MAX_TICKS) {
        fsd_sp_abort(sp, FSD_SP_ERR_EXHAUSTED, now_ms);
        return FSD_SP_ACT_NONE;
    }

    // No end-of-range guard is needed: target is validated to 0..3 and we stop
    // as soon as observed == target, so a tick is never aimed past an end. If
    // the encoding's direction turns out to be inverted (it is unconfirmed),
    // the car simply refuses to move and the stall counter ends the request.
    sp->pending_ticks = step;
    return emit(sp, direction(sp), now_ms);
}

bool fsd_sp_apply_scroll(const FsdSpeedProfile* sp, FsdSpAction act,
                         uint8_t* buf, uint8_t len) {
    if(!sp || !buf || act == FSD_SP_ACT_NONE) return false;
    if(len <= SCROLL_TICKS_BYTE) return false;
    if((buf[SCROLL_MUX_BYTE] & SCROLL_MUX_MASK) != SCROLL_MUX_SWITCHES) return false;
    // Last line of defence. check_inputs() already rejects a bad table, but this
    // is the function that actually writes to the wire, so it re-checks rather
    // than trusting a caller to have gone through the state machine.
    if(sp->enc.tick_toward_higher == 0) return false;
    if(sp->enc.tick_toward_higher > FSD_SP_DETENT_MAX) return false;
    if(sp->enc.tick_toward_higher < FSD_SP_DETENT_MIN) return false;

    int8_t tick = sp->enc.tick_toward_higher;
    if(act == FSD_SP_ACT_TICK_DOWN) tick = (int8_t)(-tick);

    buf[SCROLL_TICKS_BYTE] =
        (uint8_t)((buf[SCROLL_TICKS_BYTE] & (uint8_t)~SCROLL_TICKS_MASK) |
                  ((uint8_t)tick & SCROLL_TICKS_MASK));
    return true;
}

const char* fsd_sp_error_str(FsdSpError e) {
    switch(e) {
    case FSD_SP_OK: return "ok";
    case FSD_SP_ERR_RANGE: return "target out of range";
    case FSD_SP_ERR_LISTEN_ONLY: return "listen-only";
    case FSD_SP_ERR_NOT_ARMED: return "not armed";
    case FSD_SP_ERR_NO_STATE: return "no fresh 0x3FD";
    case FSD_SP_ERR_NO_SCROLL_BUS: return "no 0x3C2 (not Vehicle CAN)";
    case FSD_SP_ERR_OTA: return "OTA in progress";
    case FSD_SP_ERR_BUSY: return "busy";
    case FSD_SP_ERR_COOLDOWN: return "cooldown";
    case FSD_SP_ERR_UNVERIFIED: return "encoding unverified";
    case FSD_SP_ERR_STALLED: return "car did not respond";
    case FSD_SP_ERR_EXHAUSTED: return "tick budget spent";
    case FSD_SP_ERR_TIMEOUT: return "timeout";
    case FSD_SP_ERR_ABORTED: return "aborted";
    default: return "unknown";
    }
}
