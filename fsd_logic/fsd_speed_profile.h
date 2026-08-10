#pragma once
/*
 * fsd_speed_profile.h — closed-loop speed-profile convergence via scroll emulation.
 *
 * WHY THIS EXISTS
 * ---------------
 * The existing path writes the profile field of 0x3FD directly. This module
 * takes the other route: replay the user input the car already understands —
 * right-scrollwheel detents on 0x3C2 — and let the car move its own state.
 * That is what a shipping commander product does on this exact car, so it is
 * the better-evidenced path for a 2021 HW3 / MCU2 vehicle.
 *
 * Scroll is a RELATIVE command ("one notch"), so a dropped or mis-timed frame
 * silently desynchronizes open-loop stepping. This module therefore closes the
 * loop: it re-reads the profile from 0x3FD after every tick and keeps stepping
 * until the observed value equals the target, with bounded retries.
 *
 * DESIGN NOTES
 * ------------
 *  * No dependency on FSDState. Callers pass a small FsdSpInputs snapshot, so
 *    the state machine is testable on the host without the firmware.
 *  * The wire encoding (which tick value moves which way, how many ticks per
 *    step, whether the ends wrap) is isolated in FsdSpEncoding. None of it is
 *    confirmed on-car yet — see `verified`. After a capture, only that table
 *    changes; the state machine does not.
 *
 * SAFETY
 * ------
 * An Intel HW3 / 2026.14.6 emergency-braking incident is on record for scroll
 * injection, and this project's car is that exact hardware combination.
 * Therefore FsdSpInputs.tx_armed defaults to false at every call site: the
 * machine will plan and converge in simulation but refuse to emit a tick until
 * a capture of the reference implementation has been taken and the operator
 * explicitly arms it. Arming is deliberately not persisted.
 */

/*
 * INTEGRATION (not wired yet — deliberately)
 * ------------------------------------------
 * Three things are still missing, and each depends on measurements we do not
 * have. Wiring them on guesses would mean tearing them out later.
 *
 *   1. 0x3FD RX decode. Nothing currently READS the profile off the bus; the
 *      existing path only writes it. Feed the 2-bit field (byte 6, mask 0x06,
 *      shift 1) to fsd_sp_observe() once a capture confirms it.
 *   2. Freshness. FsdSpInputs.status_fresh needs a timestamp of the last
 *      0x3FD. FSDState has no such field yet.
 *   3. Tick emission. Whether a detent is injected by writing our own 0x3C2 or
 *      by another mechanism is an open hardware question. fsd_sp_apply_scroll()
 *      produces the frame body either way.
 *
 * The BLE SET_PROFILE command can call fsd_sp_request() before any of that and
 * get a precise refusal, which is exactly what the phone app needs to develop
 * against.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// FSD v14 Lite exposes four profiles. 0x3FD carries them in 2 bits, which
// matches — but the value-to-name mapping is NOT established for this build.
#define FSD_SP_PROFILE_MIN 0u
#define FSD_SP_PROFILE_MAX 3u
#define FSD_SP_PROFILE_COUNT (FSD_SP_PROFILE_MAX - FSD_SP_PROFILE_MIN + 1u)

// Bounds. Generous enough for a slow car, tight enough that a broken loop
// gives up instead of scrolling forever.
#define FSD_SP_MAX_TICKS 6u        // 3 steps would do; 2x headroom for a miss
#define FSD_SP_SETTLE_MS 400u      // wait for the car to act on one tick
#define FSD_SP_STALL_LIMIT 3u      // settles with no observed change -> retry
#define FSD_SP_TIMEOUT_MS 4000u    // whole-request ceiling
#define FSD_SP_COOLDOWN_MS 1000u   // after finishing, before accepting another
#define FSD_SP_STATE_FRESH_MS 1000u // 0x3FD older than this = we are blind

typedef enum {
    FSD_SP_IDLE = 0,
    FSD_SP_STEP,    // a tick is due
    FSD_SP_SETTLE,  // tick emitted; watching for the car to move
    FSD_SP_DONE,
    FSD_SP_FAILED,
} FsdSpPhase;

typedef enum {
    FSD_SP_OK = 0,
    FSD_SP_ERR_RANGE,        // target outside 0..3
    FSD_SP_ERR_LISTEN_ONLY,  // read-only mode
    FSD_SP_ERR_NOT_ARMED,    // tx_armed is false (pre-capture safety lock)
    FSD_SP_ERR_NO_STATE,     // no fresh 0x3FD: current profile unknown
    FSD_SP_ERR_NO_SCROLL_BUS,// no 0x3C2: not a direct Vehicle CAN tap
    FSD_SP_ERR_OTA,          // Tesla update running
    FSD_SP_ERR_BUSY,         // a request is already converging
    FSD_SP_ERR_COOLDOWN,
    FSD_SP_ERR_UNVERIFIED,   // encoding table not confirmed on-car
    FSD_SP_ERR_STALLED,      // gave up: car never moved
    FSD_SP_ERR_EXHAUSTED,    // gave up: tick budget spent
    FSD_SP_ERR_TIMEOUT,
    FSD_SP_ERR_ABORTED,      // preconditions vanished mid-flight
} FsdSpError;

typedef enum {
    FSD_SP_ACT_NONE = 0,
    FSD_SP_ACT_TICK_UP,   // toward a HIGHER profile value
    FSD_SP_ACT_TICK_DOWN, // toward a LOWER profile value
} FsdSpAction;

// Everything the state machine needs to know about the car right now. The
// caller fills this from FSDState (or a test fixture) on each poll.
typedef struct {
    bool tx_armed;           // operator armed scroll injection (see SAFETY)
    bool listen_only;
    bool ota_in_progress;
    bool scroll_bus_present; // 0x3C2 has been seen -> direct Vehicle CAN
    bool status_fresh;       // 0x3FD seen within FSD_SP_STATE_FRESH_MS
    uint8_t observed_profile;// last value decoded from 0x3FD (0..3)
} FsdSpInputs;

// Wire encoding. NOTHING here is confirmed on this car yet.
typedef struct {
    int8_t tick_toward_higher; // 6-bit signed detent that raises the profile
    uint8_t ticks_per_step;    // detents needed for one profile step
    bool wrap;                 // do the ends wrap around?
    bool verified;             // confirmed against a real capture?
} FsdSpEncoding;

// Provisional table: +1 detent assumed to raise the profile, one detent per
// step, no wrap. Direction and wrap are both open questions in the project
// notes. `verified` stays false until a capture settles them.
extern const FsdSpEncoding FSD_SP_ENCODING_DEFAULT;

typedef struct {
    FsdSpPhase phase;
    uint8_t target;
    uint8_t observed;      // profile seen at the last observe()
    uint8_t start_profile; // profile when the request was accepted
    uint8_t ticks_used;
    uint8_t pending_ticks; // detents still owed for the current step
    uint8_t stalls;
    uint32_t phase_ms;
    uint32_t started_ms;
    uint32_t finished_ms;
    FsdSpError last_error;
    FsdSpEncoding enc;
} FsdSpeedProfile;

/** Reset to IDLE with the default encoding. */
void fsd_sp_init(FsdSpeedProfile* sp);

/** Ask for `target`. Runs every precondition; on refusal nothing changes and
 *  the reason is returned (also stored in last_error). */
FsdSpError fsd_sp_request(FsdSpeedProfile* sp, const FsdSpInputs* in,
                          uint8_t target, uint32_t now_ms);

/** Feed a profile value decoded from 0x3FD. Cheap; call on every frame. */
void fsd_sp_observe(FsdSpeedProfile* sp, uint8_t profile, uint32_t now_ms);

/** Advance the machine. Returns the tick to emit right now, or ACT_NONE.
 *  Emitting is the caller's job — see fsd_sp_apply_scroll(). */
FsdSpAction fsd_sp_poll(FsdSpeedProfile* sp, const FsdSpInputs* in,
                        uint32_t now_ms);

/** Write `act` into a 0x3C2 mux=1 frame body (swcRightScrollTicks, byte3
 *  bits 0-5, 6-bit signed). Returns false if the frame is not mux=1, too
 *  short, or act is ACT_NONE — in which case buf is untouched. */
bool fsd_sp_apply_scroll(const FsdSpeedProfile* sp, FsdSpAction act,
                         uint8_t* buf, uint8_t len);

/** End the current request. `why == FSD_SP_OK` finishes as DONE, anything else
 *  as FAILED. Stamps finished_ms so the cooldown applies to failures too —
 *  otherwise a car that keeps refusing could be retried in a tight loop.
 *  Safe to call in any phase. */
void fsd_sp_abort(FsdSpeedProfile* sp, FsdSpError why, uint32_t now_ms);

/** True while a request is converging (STEP or SETTLE). */
bool fsd_sp_busy(const FsdSpeedProfile* sp);

/** Human-readable error, for logs and the BLE Result characteristic. */
const char* fsd_sp_error_str(FsdSpError e);

#ifdef __cplusplus
}
#endif
