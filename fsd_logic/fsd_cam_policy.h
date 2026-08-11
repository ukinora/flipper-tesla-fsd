#pragma once
/*
 * fsd_cam_policy.h — when to lower the speed profile, and when to put it back.
 *
 * fsd_cam_track.h decides which camera is ours. This decides what to do about
 * it. Three questions, and the third is the hard one.
 *
 * WHEN — time, not distance
 * -------------------------
 * The same 200 m is 24 seconds at 30 km/h and 6.5 seconds at 110. Changing the
 * profile does not slow the car instantly, so the lead has to be measured in
 * time for the margin to mean the same thing at every speed. Bounded at both
 * ends: a time-only rule fires on top of the camera in a school zone and half a
 * kilometre early on a motorway.
 *
 * HOW MUCH — a table, deliberately
 * --------------------------------
 * How Sloth/Chill/Standard/Hurry actually translate into road speed has not
 * been measured on this car. Everything uncertain is therefore in one table,
 * so the measurement changes PROFILE_FOR_LIMIT and nothing else.
 *
 * WHEN TO PUT IT BACK — the careful one
 * -------------------------------------
 * Lowering is the safe direction; restoring is not. So:
 *
 *   - not at the camera, but a set distance beyond it. Public data does not
 *     mark which cameras shoot the rear plate after you pass, so every one of
 *     them is assumed to.
 *   - restoring is a REQUEST that keeps being made until the car actually gets
 *     there or a deadline passes, not a single edge that can be missed.
 *   - fsd_pol_abandon() drops it entirely, with no restore. That is what the
 *     caller calls when the driver overrides us or authority is withdrawn —
 *     see 페일세이프-정책.md §6, §7. Restoring a value the driver has since
 *     moved away from would be us overruling them.
 *
 * TWO DEFECTS OF THE PROTOTYPE ARE FIXED HERE
 * -------------------------------------------
 * 1. camera-db/policy.py returns profile_for_limit() unconditionally, so a car
 *    sitting in Sloth that meets a 60 km/h camera gets RAISED to Standard —
 *    made faster next to a speed camera. A camera response must only ever
 *    lower, so the request is clamped against the observed profile.
 * 2. The prototype is edge-driven (on_approach / on_pass). A camera that
 *    appears while we are still holding past the previous one is adopted
 *    immediately, cutting the hold short — exactly the case the hold exists
 *    for. Here the steady state is a level: the caller passes whatever is
 *    nearest right now, every tick, and the hold cannot be interrupted.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FSD v14 Lite: four profiles, higher is faster. Which value carries which
 * name is NOT established for this build (CLAUDE.md) — only the ordering is
 * relied on here. */
#define FSD_POL_PROFILE_SLOTH 0u
#define FSD_POL_PROFILE_CHILL 1u
#define FSD_POL_PROFILE_STANDARD 2u
#define FSD_POL_PROFILE_HURRY 3u

/* Lead time, and the bounds that keep it sane at both ends. */
#define FSD_POL_LEAD_TIME_S 12.0f
#define FSD_POL_MIN_LEAD_M 80.0f
#define FSD_POL_MAX_LEAD_M 400.0f

/* How far past the camera before restoring. Assumes rear-facing enforcement. */
#define FSD_POL_RELEASE_DIST_M 120.0f
/* And a floor in time, so crawling past in traffic does not restore early. */
#define FSD_POL_RELEASE_MIN_S 4.0f

/* Below this we stop deciding. Car parks and standstills are not approaches. */
#define FSD_POL_MIN_SPEED_KPH 5.0f

/* Give up asking for the original profile back after this long. The car may
 * simply refuse; asking forever would fight it. */
#define FSD_POL_RESTORE_TIMEOUT_S 20.0f

/* Overrides in one drive before we stop offering. Once is a disagreement about
 * a camera; three times is the driver telling us the settings are wrong, and
 * continuing to argue is worse than being switched off. */
#define FSD_POL_MAX_OVERRIDES 3u

/* Consecutive convergence failures before the same thing happens.
 *
 * This matters far more without a phone in the car. A firmware update that
 * moves what 0x3C2 means would leave a supervised drive with a driver who sees
 * it; an unattended module would just keep scrolling at a car that is not
 * listening, for the whole drive. */
#define FSD_POL_MAX_FAILURES 3u

typedef enum {
    FSD_POL_IDLE = 0,  // nothing of ours nearby
    FSD_POL_ARMED,     // a camera is ours, but still too far to act
    FSD_POL_ACTIVE,    // holding the profile down
    FSD_POL_HOLDING,   // past it, still holding (rear-facing assumption)
    FSD_POL_RESTORING, // asking for the entry profile back
} FsdPolPhase;

typedef enum {
    FSD_POL_ACT_NONE = 0, // we are asking for nothing; leave the car alone
    FSD_POL_ACT_LOWER,    // hold target_profile
    FSD_POL_ACT_RESTORE,  // put target_profile back (the value we found)
} FsdPolAction;

/** The nearest camera on our path right now, from fsd_trk_nearest(). */
typedef struct {
    bool valid;
    uint64_t key;
    uint8_t limit_kph;
    float distance_m;
} FsdPolTarget;

typedef struct {
    FsdPolPhase phase;
    FsdPolAction action;
    uint8_t target_profile; // meaningful unless action is NONE
    uint64_t camera_key;
    uint8_t limit_kph;
    float distance_m;
    float trigger_m; // the lead distance in force at this speed
} FsdPolDecision;

typedef struct {
    FsdPolPhase phase;
    bool has_target;
    uint64_t key;
    uint8_t limit_kph;
    float distance_m;
    uint8_t entry_profile; // what the car was on when we first lowered
    bool entry_valid;
    float passed_m; // travelled since the pass
    float held_s;
    float restore_s;

    // Session bookkeeping. Reset by fsd_pol_new_drive(), not by init, because
    // "this drive" is the unit these budgets are spent over.
    uint8_t requested;      // profile of the last LOWER/RESTORE we handed out
    bool requested_valid;
    uint8_t last_observed;
    bool observed_valid;
    uint8_t overrides;      // driver corrections seen this drive
    uint8_t failures;       // consecutive convergence failures
    bool suspended;         // stop offering for the rest of this drive
} FsdPolicy;

/** Reset to idle, forgetting any entry profile. */
void fsd_pol_init(FsdPolicy* p);

/** Target profile for a posted limit. The one table that a road measurement
 *  changes. */
uint8_t fsd_pol_profile_for_limit(uint8_t limit_kph);

/** Lead distance at this speed, bounded. */
float fsd_pol_lead_distance_m(float speed_kph);

/** The camera was passed (or dropped). Starts the hold when we had lowered,
 *  otherwise clears. Key must match the current target or it is ignored. */
void fsd_pol_on_pass(FsdPolicy* p, uint64_t key);

/** Stop, and do not restore. For a driver override or a withdrawal of
 *  authority: the entry profile is forgotten rather than forced back. */
void fsd_pol_abandon(FsdPolicy* p);

/** The tracker is no longer following our target and cannot report on it again.
 *
 *  NOT the same as passing it — we do not know where the camera went, only that
 *  nobody is measuring it any more. The caller reaches this after wiping the
 *  tracker's in-flight measurements: a gap in fixes, a refused fix, a reboot.
 *
 *  It has to be said out loud, because fsd_pol_tick() deliberately keeps an
 *  ACTIVE target adopted when `ahead` goes invalid — losing a camera from one
 *  scan is not the same as having passed it, and the comment there says only
 *  fsd_pol_on_pass() may end it. That is right while the tracker is still
 *  following the camera. Once the tracker has forgotten it THE EVENT CAN NEVER
 *  COME, and the frozen distance goes on satisfying the trigger — so LOWER
 *  latches for the rest of the drive.
 *
 *  Handled as a pass: hold, then restore. Both alternatives are worse — keeping
 *  it adopted latches, and abandoning leaves the car on the profile we lowered
 *  it to with nothing owed back.
 *
 *  The cost is a shorter margin: the hold is measured from where we lost it
 *  rather than from the camera, so one lost 60 m short is released 60 m past it
 *  instead of 120 m. That is the price of not latching, and it is only paid
 *  when the position was already unreliable. */
void fsd_pol_target_lost(FsdPolicy* p);

/** Advance one fix.
 *
 *  `ahead` may be NULL or invalid when nothing is on our path.
 *  `observed_profile` is the profile read back from the car — it both clamps
 *  the request (never raise) and tells a restore when it is finished.
 *  `moved_m` / `dt_s` are since the previous call. */
FsdPolDecision fsd_pol_tick(FsdPolicy* p, const FsdPolTarget* ahead,
                            uint8_t observed_profile, float speed_kph,
                            float moved_m, float dt_s);

/** Start of a drive: clears the override and failure budgets and lifts any
 *  suspension. Call when the gear leaves P (see fsd_autonomy.h) — a budget that
 *  never resets would eventually latch the feature off forever. */
void fsd_pol_new_drive(FsdPolicy* p);

/** Feed the profile read back from 0x3FD, every time it is decoded.
 *
 *  Detects the driver turning the wheel while we are mid-request. The naive
 *  test — "the value is not what we asked for" — false-positives on our own
 *  convergence, because stepping 3 -> 1 passes through 2. What actually
 *  separates them is DIRECTION: moving closer to the request is us, moving away
 *  from it is a person.
 *
 *  Returns true when an override was detected, in which case the current target
 *  has already been abandoned (no restore — the driver's value is now theirs).
 *  After FSD_POL_MAX_OVERRIDES in one drive it also suspends. */
bool fsd_pol_observe_profile(FsdPolicy* p, uint8_t observed);

/** The convergence machine gave up. FSD_POL_MAX_FAILURES in a row suspends for
 *  the rest of the drive. */
void fsd_pol_on_convergence_failed(FsdPolicy* p);

/** The convergence machine arrived. Clears the consecutive-failure count. */
void fsd_pol_on_convergence_ok(FsdPolicy* p);

/** True while the policy has stopped offering for this drive. */
bool fsd_pol_suspended(const FsdPolicy* p);

/** Human-readable phase, for logs and the app. */
const char* fsd_pol_phase_str(FsdPolPhase ph);

#ifdef __cplusplus
}
#endif
