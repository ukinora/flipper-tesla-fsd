#pragma once
/*
 * fsd_gps.h — the car's own GPS, off the bus, turned into an FsdCamFix.
 *
 * WHY THIS EXISTS
 * ---------------
 * The whole "camera handling works with the phone left at home" design rests on
 * one claim: the car broadcasts its position on CAN, so the module needs no GPS
 * hardware and no phone. fsd_camera.h says so in its opening comment and every
 * layer above it consumes an FsdCamFix — but nothing has ever PRODUCED one.
 * The tracker, the learning store and the policy are all reachable only through
 * a fix, so until this file existed none of them could run on a car.
 *
 * This is the missing producer, and nothing else.
 *
 * WHAT IS AND IS NOT MEASURED
 * ---------------------------
 * The bit layouts below come from opendbc's tesla_can.dbc and are as solid as
 * the 0x3FD layout the write path has used for years — they are arithmetic, not
 * guesses. What is NOT established is whether these frames flow on OUR car
 * (2021 M3, MCU2, HW3, KR build) and on the pair we tap. That question is not
 * answerable from a desk, and it does not have to be: every consumer already
 * goes through fsd_gps_fix_why(), which returns FSD_GPS_NO_POSITION until a
 * frame actually arrives. If the frames are absent the feature refuses by name
 * instead of misbehaving.
 *
 * See 벤치-브링업-체크리스트.md 8-B for the measurement that settles it.
 *
 * FAIL-CLOSED, LIKE fsd_autonomy.h
 * --------------------------------
 * Same discipline, same reason. Never seen, too old, out of range, receiver
 * reporting no data, position frozen — every one of them is a refusal with a
 * name attached. Being wrong in this direction costs a missed camera. Being
 * wrong the other way means steering the car's speed profile from a position
 * that is not where the car is.
 *
 * THE FROZEN FIX IS THE INTERESTING FAILURE
 * -----------------------------------------
 * Staleness catches a bus that goes quiet. It does NOT catch the failure this
 * layer actually has: in a tunnel or an underground car park the MCU keeps
 * broadcasting 0x3D8 at its last known position. Every frame is fresh, every
 * value is plausible, and the module believes it is parked on top of whichever
 * camera it last drove past — the "유령 카메라" hazard in 페일세이프-정책.md.
 *
 * Freshness cannot see it and neither can the GPS speed, because that freezes
 * with everything else. It takes an INDEPENDENT witness that the car is moving,
 * which is why this module requires 0x257 DI_vehicleSpeed and refuses with
 * FSD_GPS_NO_MOTION_REF without it. "Driving at speed, position has not moved"
 * is a contradiction the drivetrain can prove and GPS cannot.
 */

#include "fsd_camera.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* opendbc tesla_can.dbc:
 *   BO_ 984 MCU_locationStatus: 8 MCU        (0x3D8)
 *     MCU_latitude    :  0|28@1- (1e-06, 0) deg
 *     MCU_longitude   : 28|29@1- (1e-06, 0) deg
 *     MCU_gpsAccuracy : 57|7@1+  (0.2,   0) m
 *   BO_ 760 UI_gpsVehicleSpeed: 8 GTW        (0x2F8)
 *     UI_gpsHDOP           :  0|8@1+  (0.1,        0)
 *     UI_gpsVehicleHeading :  8|16@1+ (0.0078125,  0) deg
 *     UI_gpsVehicleSpeed   : 24|16@1+ (0.00390625, 0) km/h
 *     UI_gpsNmeaMIA        : 53|1@1+
 * All signals are little-endian (@1), so each is a shift and a mask out of the
 * 8 bytes read as one 64-bit little-endian word. */
#define FSD_CAN_ID_MCU_LOCATION 0x3D8u
#define FSD_CAN_ID_UI_GPS 0x2F8u

/* Position and heading arrive on different frames from different senders, so
 * they age independently and are checked independently.
 *
 * Three seconds assumes both frames run at or above ~1 Hz, which is the one
 * number here that is a guess rather than arithmetic. If they turn out slower,
 * every fix reports FSD_GPS_POSITION_STALE — a named refusal that is fixed by
 * changing this line once the rate is measured, not a silent misbehaviour. A
 * rate that slow would sink the feature anyway: at 100 km/h a 5 s gap is 139 m
 * of unseen road. Measure it at 벤치-브링업-체크리스트.md 8-B.
 *
 * The motion reference gets the tight window fsd_autonomy.h uses, because
 * 0x257 is a checksummed drivetrain frame and runs far faster than either. */
#define FSD_GPS_POS_FRESH_MS 3000u
#define FSD_GPS_VEL_FRESH_MS 3000u
#define FSD_GPS_REF_FRESH_MS 1000u

/* Freeze detection. All three conditions must hold together:
 *
 *   - the drivetrain says we have been above FREEZE_MIN_KPH for the whole
 *     window (so a red light is not a frozen fix),
 *   - the reported position has stayed within FREEZE_MOVED_M of where the
 *     window started (a metre, against 8+ m of real travel at the threshold
 *     speed — the gap is an order of magnitude, not a judgement call),
 *   - and at least FREEZE_MIN_SAMPLES position frames have arrived saying so.
 *
 * The sample count is what makes this safe without knowing the frame rate. We
 * have never measured how often this car sends 0x3D8; if it turns out to be
 * slower than one frame per FREEZE_MS, a time-only test would call every normal
 * gap a freeze. Requiring frames means the detector cannot fire on silence —
 * silence is already staleness, and it has its own verdict. */
#define FSD_GPS_FREEZE_MS 3000u
#define FSD_GPS_FREEZE_MIN_SAMPLES 3
#define FSD_GPS_FREEZE_MIN_KPH 10.0f
#define FSD_GPS_FREEZE_MOVED_M 1.0f

/* Which gate refused, so the reason can go on the BLE Result characteristic.
 * "No camera warnings" is useless to a user who cannot see whether the car is
 * in a tunnel, the frame never existed on this build, or the receiver has no
 * satellites. Same shape and same purpose as FsdSupVerdict. */
typedef enum {
    FSD_GPS_OK = 0,
    FSD_GPS_NO_POSITION,   // 0x3D8 has never been decoded on this bus
    FSD_GPS_POSITION_STALE,
    FSD_GPS_NO_HEADING,    // 0x2F8 has never been decoded
    FSD_GPS_HEADING_STALE,
    FSD_GPS_NO_FIX,        // UI_gpsNmeaMIA: the receiver itself reports no data
    FSD_GPS_NO_MOTION_REF, // 0x257 missing or stale: a freeze would be invisible
    FSD_GPS_FROZEN,        // moving for seconds, position did not move with us
} FsdGpsVerdict;

typedef struct {
    // ── 0x3D8 ────────────────────────────────────────────────────────────────
    int32_t lat_e7; // note the scale change: the frame carries 1e-6 degrees,
    int32_t lon_e7; // FsdCamFix and camera.bin are 1e-7. See fsd_gps_decode_position().
    float accuracy_m;
    bool pos_seen;
    uint32_t pos_ms;

    // ── 0x2F8 ────────────────────────────────────────────────────────────────
    float bearing_deg; // 0 = north, clockwise. Meaningless at a standstill —
                       // GPS heading is derived from movement — but harmless,
                       // because nothing is being approached at a standstill.
    float gps_speed_kph;
    float hdop;
    bool nmea_mia;
    bool vel_seen;
    uint32_t vel_ms;

    // ── 0x257, the independent witness ───────────────────────────────────────
    float ref_speed_kph;
    bool ref_seen;
    uint32_t ref_ms;

    // ── freeze detector ──────────────────────────────────────────────────────
    int32_t still_lat_e7; // where the current no-movement window started
    int32_t still_lon_e7;
    uint32_t still_since_ms;
    uint16_t still_samples;
    uint32_t fast_since_ms; // when the car last became, and stayed, fast
    bool fast;

    // ── diagnostics ──────────────────────────────────────────────────────────
    // Counted rather than logged: on a bring-up drive the useful question is
    // "did anything arrive and was any of it thrown away", and a counter pair
    // answers it over BLE without a serial cable.
    uint32_t pos_frames;
    uint32_t vel_frames;
    uint32_t rejects;
} FsdGps;

/** Clear everything. Leaves the module in FSD_GPS_NO_POSITION. */
void fsd_gps_init(FsdGps* g);

/** Decode a 0x3D8 body into 1e-7 degrees and metres, without touching any
 *  state. Split out from the observer so the arithmetic can be tested directly
 *  and so a caller can decode a captured frame offline.
 *
 *  Returns false — leaving the outputs untouched — when the frame is too short
 *  or the result is not a place on Earth. */
bool fsd_gps_decode_position(const uint8_t* data, uint8_t dlc, int32_t* lat_e7,
                             int32_t* lon_e7, float* accuracy_m);

/** Decode a 0x2F8 body. Returns false on a short frame or a heading outside
 *  0..360 — the field holds up to 511.99, so an out-of-range value means we are
 *  reading something that is not this signal, and folding it into range would
 *  turn that into a plausible-looking wrong heading. */
bool fsd_gps_decode_velocity(const uint8_t* data, uint8_t dlc, float* bearing_deg,
                             float* speed_kph, float* hdop, bool* nmea_mia);

/** Feed a 0x3D8 frame. Returns true when it was accepted; a rejected frame is
 *  counted and does NOT stamp the clock, so a receiver emitting rubbish goes
 *  stale rather than looking alive. */
bool fsd_gps_observe_position(FsdGps* g, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

/** Feed a 0x2F8 frame. Same rejection rule. A frame with UI_gpsNmeaMIA set IS
 *  accepted and stamped — it is a valid report that the receiver has nothing,
 *  which is different information from the frame being absent, and the two get
 *  different verdicts. */
bool fsd_gps_observe_velocity(FsdGps* g, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

/** Feed the independent motion reference: DI_vehicleSpeed from 0x257, already
 *  decoded (fsd_handle_di_speed() does it, or the shared observer below).
 *  Without this the freeze detector cannot run and every fix is refused. */
void fsd_gps_observe_motion_ref(FsdGps* g, float kph, uint32_t now_ms);

/** Read DI_vehicleSpeed out of a 0x257 DI_speed frame and feed it in one step.
 *
 *  Exists for the same reason fsd_drive_observe_gear() does: the ESP32 build
 *  does not compile fsd_logic/fsd_handler.c, so its 0x257 parser has never run
 *  on this hardware and FSDState.vehicle_speed_kph is structurally zero there.
 *  This one is small, shared by both builds, and depends on nothing.
 *
 *  DI_vehicleSpeed : 12|12@1+ (0.08, -40) km/h. Returns false on a short frame,
 *  in which case nothing is fed and nothing is stamped. */
bool fsd_gps_observe_di_speed(FsdGps* g, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

/** Why there is (not) a usable fix, filling `out` when the answer is
 *  FSD_GPS_OK. `out` may be NULL if only the verdict is wanted; it is left
 *  untouched on any refusal.
 *
 *  FsdCamFix.speed_kph is taken from the motion reference, not from
 *  UI_gpsVehicleSpeed. Both are available and they usually agree, but only one
 *  of them is still telling the truth in the tunnel this module exists to
 *  survive — and the fix's speed feeds the ETA the policy acts on. */
FsdGpsVerdict fsd_gps_fix_why(const FsdGps* g, uint32_t now_ms, FsdCamFix* out);

/** True when there is a usable fix. Convenience over _why(). */
bool fsd_gps_fix(const FsdGps* g, uint32_t now_ms, FsdCamFix* out);

/** Human-readable verdict, for logs and the BLE Result characteristic. */
const char* fsd_gps_verdict_str(FsdGpsVerdict v);

#ifdef __cplusplus
}
#endif
