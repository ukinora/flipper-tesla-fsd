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
 *    step, whether the ends wrap) is isolated in FsdSpEncoding. All of it is
 *    now confirmed on-car — see `verified`. Only that table changed; the state
 *    machine did not.
 *
 * SAFETY
 * ------
 * An Intel HW3 / 2026.14.6 emergency-braking incident is on record for scroll
 * injection, and this project's car is that exact hardware combination.
 * Therefore FsdSpInputs.tx_armed defaults to false at every call site: the
 * machine will plan and converge in simulation but refuse to emit a tick until
 * the operator explicitly arms it. Arming is deliberately not persisted.
 *
 * 🔴 THERE WERE TWO LOCKS AND ONE OPENED (2026-09-06). `verified` is now true
 * — see FSD_SP_ENCODING_DEFAULT for what closed the last open question and who
 * decided it. tx_armed did not move, and it is the one that matters at
 * runtime: nothing persists it, and NOTHING IN THE FIRMWARE SETS IT. Together
 * with the fact that fsd_sp_request / fsd_sp_poll / fsd_sp_apply_scroll /
 * fsd_sp_apply_detents still have no caller outside the tests, this module
 * remains structurally unable to put a frame on a bus.
 *
 * What verifying DID open is narrow and worth naming: the two apply_ functions
 * are gated on fsd_sp_encoding_ok() alone, so they now build bytes into a
 * caller's buffer where before they refused. That is the same posture as the
 * body emitters — bytes exist, no caller, nothing transmits.
 */

/*
 * INTEGRATION (not wired yet — deliberately)
 * ------------------------------------------
 * 🔴 Item 1 was CORRECTED on 2026-09-06 and it had been wrong, not merely
 * incomplete: it told the next person to feed "the 2-bit field (byte 6, mask
 * 0x06, shift 1)" to fsd_sp_observe(), and both halves of that would have hurt.
 * The field is at mux 2 byte 7 bits [6:4] on this car, and fsd_sp_observe()
 * takes a RANK, not the raw number.
 *
 *   1. 0x3FD RX decode. ✅ DONE. camera_task_observe_profile() reads the frame
 *      through fsd_sp_decode_profile() and converts with fsd_sp_rank_from_raw()
 *      before anything numeric sees it. A new caller should use
 *      fsd_sp_observe_raw(), which does both in one step.
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
 * 🟢 fsd_sp_decode_profile() READS THE RIGHT BITS NOW (fixed 2026-09-06). This
 * paragraph used to warn that it did not, and that fixing the read alone would
 * be worse than leaving it broken — raw 4 arriving at a numeric clamp. Both
 * halves landed in the same change, which is the only way either was safe:
 * the decoder moved to mux 2 byte 7 bits [6:4], and every consumer converts
 * through fsd_sp_rank_from_raw() at the boundary.
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
/* How long to give the car to act on one detent before counting a stall.
 *
 * MEASURED, 2nd visit (2026-09-03): three +1 detents, and 0x3FD followed at
 * 298 / 221 / 297 ms. Raised 400 -> 500 on 2026-09-06.
 *
 * 🔴 400 left 102 ms over the slowest of THREE samples, and three samples do
 * not bound a distribution. The two ways of being wrong are not symmetric:
 * too short and the machine calls a car that IS responding a stall and ticks
 * again — an extra detent nobody asked for, on a car with a recorded
 * emergency-braking incident. Too long only means a slower give-up, and the
 * whole-request ceiling (FSD_SP_TIMEOUT_MS) still bounds that. */
#define FSD_SP_SETTLE_MS 500u
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
//       BOTH ENDS MEASURED NOW.
//         bottom — 2nd visit (2026-09-03): at Sloth a -5 detent moved nothing.
//         top    — the OWNER, 2026-09-06: "신속에서 한 번 더 올려도 신속 그대로
//                  유지되는 것을 관측했다" (at Hurry, one more up stays at
//                  Hurry). Observed from the driver's seat, while driving.
//
//       🔴 That report is not the same evidence as the 4th visit's capture,
//       and that is exactly why it settles the question. The capture sent +1
//       and +5 at Hurry and nothing moved — but the car was PARKED (no 0x118,
//       no 0x257: the drive inverter was silent), so "saturated at the top" and
//       "parked, so ignored" were the same picture and nothing in the file
//       could separate them. A person watching the screen in a moving car can.
//
//       false stays the conservative value either way: with wrap off the
//       machine only ever ticks toward the target and stops on equality, so it
//       cannot be steered past an end.
//
//   verified = true
//       🔴 Flipped 2026-09-06 by the OWNER, and only the owner could: both
//       reasons this field stayed false were closed, and only one of them was
//       a measurement.
//         1. The top end — see `wrap` above. Closed by the owner's own
//            observation, not by a capture.
//         2. Arming scroll injection on a car with a recorded Intel HW3
//            emergency-braking incident was never a coding decision. Asked and
//            answered: "허용한다".
//
//       ⚠️ This opens the ENCODING gate, not the transmit gate. See SAFETY at
//       the top of this file for what that does and does not change.
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

/** Pull the RAW speed profile out of a 0x3FD DAS_autopilotControl frame.
 *
 *    !hw4   mux 2, byte 7 bits [6:4]  (3 bits) — THIS CAR, measured 2026-09-03
 *    hw4    mux 2, byte 7 bits [7:5]  (3 bits) — the documented HW4 layout
 *
 *  🔴 THE FIRST LINE USED TO SAY "HW3: mux 0, byte 6 bits [2:1]" AND THAT WAS
 *  WRONG FOR THIS CAR (corrected 2026-09-06). The old position was not a
 *  guess — it is the DBC's, and it is where fsd_handler.c's WRITE path has
 *  always put the value — but symmetry with our own writes is not evidence
 *  about what the car SENDS. The 2nd visit measured it: mux 0 byte 6 sat at 0
 *  for an entire drive while mux 2 byte 7 bits [6:4] followed the scroll wheel
 *  tick for tick, 221-298 ms behind each one. One bit below the HW4 field,
 *  which is the difference between four distinct values and two (the HW4
 *  layout collides Chill and Standard onto the same number here).
 *
 *  The non-hw4 branch delegates to fsd_decode_profile_obs() in fsd_types.h so
 *  there is ONE definition of where this car keeps the profile. The dashboard
 *  reads it through that function too; what the two paths still do differently
 *  is what they produce, which is the part that has to stay separate.
 *
 *  🔴 THE RETURNED VALUE IS RAW AND IS NOT A RANK. Raw Sloth is 4, so feeding
 *  this straight to anything that compares numbers — fsd_sp_observe(), the
 *  camera policy's never-raise clamp — makes Sloth the fastest profile there
 *  is. Convert at the boundary with fsd_sp_rank_from_raw(), or use
 *  fsd_sp_observe_raw() which does both.
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
