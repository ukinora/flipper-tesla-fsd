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

/* ---- THE SCALE THIS MACHINE SPEAKS ----------------------------------------
 *
 * Everything below (target, observed, FsdSpInputs.observed_profile) is a SPEED
 * RANK, not the number the car puts on the wire:
 *
 *     rank 0 = slowest ... rank 3 = fastest
 *
 * That has always been the machine's scale — direction() takes the sign of a
 * difference and `wrap` treats it as a ring, and neither means anything unless
 * the numbers rise with speed. What was missing was WHICH raw value is which
 * rank. The capture supplies it; see FSD_SP_RAW_BY_RANK below.
 *
 * It is the same scale fsd_cam_policy.h uses (FSD_POL_PROFILE_SLOTH..HURRY),
 * deliberately: the two modules must not disagree about which way is faster.
 */
#define FSD_SP_PROFILE_MIN 0u
#define FSD_SP_PROFILE_MAX 3u
#define FSD_SP_PROFILE_COUNT (FSD_SP_PROFILE_MAX - FSD_SP_PROFILE_MIN + 1u)

/* ---- THE CAR'S OWN VALUES, IN ORDER OF SPEED ------------------------------
 *
 * MEASURED, not assumed. 2nd visit capture (2026-09-03, gear D, 872 frames):
 * one right-scroll detent walked the car up one profile at a time and
 * 0x3FD mux 2 byte7 bits[6:4] read back
 *
 *     Sloth 4  ->  Chill 0  ->  Standard 1  ->  Hurry 2
 *
 * 🔴 The raw scale is NOT monotonic in speed. Sloth is 4 and Hurry is 2, so
 * the sign of (target - observed) computed on RAW values points the wrong way
 * whenever Sloth is one of the two — it makes Sloth look like the fastest
 * profile there is. That is the finding the 5th red team pinned down on
 * 2026-09-04, and test_camera.c's test_policy_scale_is_not_the_raw_can_value
 * guards the same fact from the policy side.
 *
 * So a raw value has to become a rank at the BOUNDARY — fsd_sp_rank_from_raw()
 * — and must never be fed straight into fsd_sp_observe().
 * fsd_sp_observe_raw() does both in one call and is the safer entry point.
 *
 * ⚠️ fsd_sp_decode_profile() still reads the WRONG BITS for this car (see its
 * own note). Fixing that read site is a separate decision, and this table is
 * its prerequisite: fixing the read alone would start feeding raw 4 into a
 * numeric clamp, which is worse than reading nothing at all.
 */
#define FSD_SP_RAW_SLOTH 4u
#define FSD_SP_RAW_CHILL 0u
#define FSD_SP_RAW_STANDARD 1u
#define FSD_SP_RAW_HURRY 2u

/** Raw 0x3FD value for each speed rank, slowest first. */
extern const uint8_t FSD_SP_RAW_BY_RANK[FSD_SP_PROFILE_COUNT];

/** Raw 0x3FD value -> speed rank. False (and *rank_out untouched) for a value
 *  this car has never been seen to send — a mis-decode has to look like
 *  silence, not like a profile. */
bool fsd_sp_rank_from_raw(uint8_t raw, uint8_t* rank_out);

/** Speed rank -> raw 0x3FD value. False for a rank outside 0..3. */
bool fsd_sp_raw_from_rank(uint8_t rank, uint8_t* raw_out);

// Bounds. Generous enough for a slow car, tight enough that a broken loop
// gives up instead of scrolling forever.
#define FSD_SP_MAX_TICKS 6u        // 3 steps would do; 2x headroom for a miss
#define FSD_SP_SETTLE_MS 400u      // wait for the car to act on one tick
#define FSD_SP_STALL_LIMIT 3u      // settles with no observed change -> retry
#define FSD_SP_TIMEOUT_MS 4000u    // whole-request ceiling
#define FSD_SP_COOLDOWN_MS 1000u   // after finishing, before accepting another
#define FSD_SP_STATE_FRESH_MS 1000u // 0x3FD older than this = we are blind

// swcRightScrollTicks is 6 bits signed, so the field holds -32..31. The usable
// range is narrower and symmetric: a detent must survive negation, because UP
// and DOWN are the same magnitude with opposite sign. -32 negates to +32, which
// does not fit and wraps back to -32 — UP and DOWN would encode identically.
// Zero is excluded for the obvious reason that it moves nothing.
#define FSD_SP_DETENT_MAX 31
#define FSD_SP_DETENT_MIN (-31)

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
    // "UP" is a higher RANK, i.e. a FASTER profile. It is NOT a higher raw CAN
    // value: on this car the fastest profile (Hurry) is raw 2 and the slowest
    // (Sloth) is raw 4. See FSD_SP_RAW_BY_RANK.
    FSD_SP_ACT_TICK_UP,   // toward a FASTER profile
    FSD_SP_ACT_TICK_DOWN, // toward a SLOWER profile
} FsdSpAction;

// Everything the state machine needs to know about the car right now. The
// caller fills this from FSDState (or a test fixture) on each poll.
typedef struct {
    bool tx_armed;           // operator armed scroll injection (see SAFETY)
    bool listen_only;
    bool ota_in_progress;
    bool scroll_bus_present; // 0x3C2 has been seen -> direct Vehicle CAN
    bool status_fresh;       // 0x3FD seen within FSD_SP_STATE_FRESH_MS
    uint8_t observed_profile;// last profile as a SPEED RANK (0..3), not the raw
                             // 0x3FD value — convert with fsd_sp_rank_from_raw()
} FsdSpInputs;

// Wire encoding. Every field except `verified` is now MEASURED — see the
// default table's comment for which capture supplied which number.
typedef struct {
    int8_t tick_toward_faster; // 6-bit signed detent count for ONE step toward
                               // a faster profile. Negated for the slow way.
    uint8_t ticks_per_step;    // detents needed for one profile step
    bool wrap;                 // do the ends wrap around?
    bool verified;             // every field confirmed, including the ends?
} FsdSpEncoding;

// MEASURED from captures of the commercial device driving this exact path on
// this exact car. Field by field, with the evidence:
//
//   tick_toward_faster = +1
//       2nd visit (2026-09-03, gear D). Three consecutive +1 detents walked
//       0x3FD from Sloth to Chill to Standard to Hurry, 221-298 ms behind each
//       tick. Positive is the FAST way. (Raw values 4 -> 0 -> 1 -> 2, which is
//       why this field is named after speed and not after the number.)
//
//   ticks_per_step = 1
//       Same capture: one detent, one step. Three for three, no doubles.
//
//   wrap = false
//       Bottom end measured (2nd visit): at Sloth a -5 detent moved nothing.
//       Top end NOT measured — see `verified`. false is the conservative value
//       either way: with wrap off the machine only ever ticks toward the
//       target and stops on equality, so it cannot be steered past an end.
//
//   verified = false
//       🔴 Deliberate, and the only thing still holding the gate shut. Two
//       reasons, and neither is a coding task:
//         1. The top end is unobserved. The 4th visit sent +1 and +5 while the
//            car was already at Hurry and it stayed at Hurry — but the car was
//            PARKED (no 0x118, no 0x257: the drive inverter was silent), so
//            "saturated at the top" and "parked, so ignored" are the same
//            picture. Nothing in that capture can tell them apart.
//         2. Flipping it opens scroll injection on a car with a recorded Intel
//            HW3 emergency-braking incident. That is the owner's call, not a
//            side effect of filling in a table.
//
// The frame shape is measured too, and lives in the .c next to the constants:
// 0x3C2 mux 1, byte 3, 6-bit two's complement, one frame carries the WHOLE
// count (the device sent 0x05 once for +5, not 0x01 five times), inserted
// 0-1 ms after the car's own frame. No counter, no checksum.
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

/** True when the table is confirmed AND every field is expressible on the wire.
 *  The table is written by a tool from a capture, so it is input, not a
 *  constant: an out-of-range detent would be masked into the wrong direction
 *  rather than rejected. Requests are refused unless this passes. */
bool fsd_sp_encoding_ok(const FsdSpEncoding* e);

/** Ask for `target`. Runs every precondition; on refusal nothing changes and
 *  the reason is returned (also stored in last_error). */
FsdSpError fsd_sp_request(FsdSpeedProfile* sp, const FsdSpInputs* in,
                          uint8_t target, uint32_t now_ms);

/** Pull the speed profile out of a 0x3FD DAS_autopilotControl frame.
 *
 *  This is not guesswork and never needed a capture: the WRITE path in
 *  fsd_handler.c has been putting the value in these exact bits for as long as
 *  the project has existed, so reading the same bits is symmetric with code
 *  that is already known to work on this car.
 *
 *    HW3   mux 0, byte 6 bits [2:1]  (2 bits, 0..3)
 *    HW4   mux 2, byte 7 bits [7:5]  (3 bits, 0..7)
 *
 *  What DOES need measuring is which value carries which name (is 0 Sloth?)
 *  and which scroll direction raises it. Neither is needed here: the
 *  convergence loop only compares values, and the direction lives in
 *  FsdSpEncoding behind its own `verified` flag.
 *
 *  Returns false — leaving *out untouched — when the frame is the wrong mux or
 *  too short to hold the field. */
bool fsd_sp_decode_profile(const uint8_t* data, uint8_t dlc, bool hw4, uint8_t* out);

/** Feed a profile SPEED RANK (0..3). Cheap; call on every frame.
 *  Values outside 0..3 are ignored.
 *
 *  🔴 Do not hand this the value fsd_sp_decode_profile() returns — that is a
 *  raw CAN value, and raw Sloth is 4, which this function drops. Use
 *  fsd_sp_observe_raw(). */
void fsd_sp_observe(FsdSpeedProfile* sp, uint8_t profile, uint32_t now_ms);

/** Same, but takes the RAW 0x3FD value and converts it. Returns false when the
 *  car sent a value that is not one of the four this car is known to use — in
 *  which case nothing is observed, which is the honest outcome for a frame we
 *  cannot interpret. This is the entry point a caller should wire. */
bool fsd_sp_observe_raw(FsdSpeedProfile* sp, uint8_t raw, uint32_t now_ms);

/** Advance the machine. Returns the tick to emit right now, or ACT_NONE.
 *  Emitting is the caller's job — see fsd_sp_apply_scroll(). */
FsdSpAction fsd_sp_poll(FsdSpeedProfile* sp, const FsdSpInputs* in,
                        uint32_t now_ms);

/** Write `act` into a 0x3C2 mux=1 frame body (swcRightScrollTicks, byte3
 *  bits 0-5, 6-bit signed). Returns false if the frame is not mux=1, too
 *  short, or act is ACT_NONE — in which case buf is untouched.
 *
 *  One step's worth of detents, taken from the encoding table. For an
 *  arbitrary count in a single frame, see fsd_sp_apply_detents(). */
bool fsd_sp_apply_scroll(const FsdSpeedProfile* sp, FsdSpAction act,
                         uint8_t* buf, uint8_t len);

/** Write an arbitrary detent count into a 0x3C2 mux=1 frame body.
 *
 *  The commercial device puts the whole count in ONE frame: for "+5" it sent
 *  byte3 = 0x05 exactly once, not 0x01 five times (4th visit, 2026-09-05 —
 *  80 mux-1 frames in the file and precisely one with byte3 != 0). So an
 *  emitter never needs to repeat itself, and this is the function that says so.
 *
 *  Gated identically to fsd_sp_apply_scroll(): the encoding table must pass
 *  fsd_sp_encoding_ok(), which today it does not. That is on purpose — this
 *  builds the bytes that scroll a car with a recorded emergency-braking
 *  incident, so it must not become the easy way around the flag.
 *
 *  `detents` must be non-zero and within FSD_SP_DETENT_MIN..MAX. Returns false
 *  and leaves buf untouched otherwise. */
bool fsd_sp_apply_detents(const FsdSpeedProfile* sp, int8_t detents,
                          uint8_t* buf, uint8_t len);

/** Read a detent count back out of a 0x3C2 mux=1 frame body, sign-extending
 *  the 6-bit field. Ungated: reading a frame the car (or the commercial
 *  device) sent is how we check our own work, and it puts nothing on the wire.
 *  False — *out untouched — if the frame is not mux=1 or too short. */
bool fsd_sp_read_detents(const uint8_t* buf, uint8_t len, int8_t* out);

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
