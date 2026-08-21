#pragma once
/*
 * fsd_wire.h — the BLE payloads, as pure arithmetic.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * The State and CamStat packers lived inside ble_server.cpp, which is Arduino +
 * NimBLE and cannot be compiled on a host. So the bytes the phone app parses —
 * every clamp, every scale factor, every little-endian split — had **no tests at
 * all**, on either side of the link.
 *
 * That is the same split this project already makes three times over
 * (FsdSpInputs, FsdBodyInputs, FsdGps): gather on the platform, decide in pure
 * C. Here the platform gathers FSDState under a mutex and fills the plain
 * structs below; the packing is a function of those structs and nothing else.
 *
 * WHAT IT BUYS BEYOND TESTS
 * -------------------------
 * A canonical byte vector. test_wire.c writes test/fixtures/wire_vectors.json
 * from these packers, and the Android app's JVM tests parse the SAME file and
 * assert they recover the same fields.
 *
 * That makes the two implementations check each other rather than both checking
 * the documentation — which matters here, because the wire has already moved
 * twice: State byte 9 went from cruise state to gear (ver 1 -> 2), and CamStat
 * went from 12 bytes to 20. A doc-copying app would have gone quietly wrong on
 * both. The same trick test_camera.c already plays with a real camera.bin.
 *
 * THE SOURCE OF TRUTH IS STILL THE FIRMWARE. The fixture is generated, never
 * hand-written; if it disagrees with the app, the app is wrong.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Both payloads are 20 bytes because that is the most a default 23-byte ATT MTU
 * carries. Not a coincidence and not adjustable without a version bump. */
#define FSD_WIRE_STATE_LEN 22u
#define FSD_WIRE_CAMSTAT_LEN 20u
#define FSD_WIRE_RESULT_LEN 4u

/* Wire versions. Each payload carries its own — sharing one meant that bumping
 * State also announced a CamStat change that had not happened. */
#define FSD_WIRE_STATE_VERSION 4u
#define FSD_WIRE_CAMSTAT_VERSION 2u

/* FSD v14 Lite exposes four speed profiles. */
#define FSD_WIRE_PROFILE_MAX 3

/* Everything State is built from. Filled by the caller from FSDState.
 *
 * Note what is NOT here: flags bits 5 and 7. They are structurally zero --
 * blind-spot is not extracted on the ESP32 path and the closed loop that would
 * set bit 7 emits nothing -- so they are not inputs to anything. Giving them a
 * field would invite someone to fill it. */
typedef struct {
    bool rx_seen; // any CAN frame ever received
    bool ota_in_progress;
    bool blinker_left;  // *BlinkerOn  — closer to the stalk than the lamp
    bool blinker_right;

    /* Byte 20 bits[3:0]. 2 bits each: 0 off, 1 blinking-dark, 2 blinking-lit.
     * The LAMP. See FSDState for why both pairs are carried and why nothing
     * may depend on these being non-zero. */
    uint8_t blinker_left_blinking;
    uint8_t blinker_right_blinking;

    /* Byte 20 bits[5:4]. Where the speed limit came from -- SpeedLimitSource:
     * 0 none, 1 map, 2 vision, 3 ACC.
     *
     * The number alone could not be read. Three CAN frames write the same
     * field and the last one wins, so "60" might be a sign the camera just
     * read, what the navigation map says about this road, or what adaptive
     * cruise is holding to. The firmware always knew which; it just never
     * said. */
    uint8_t speed_limit_source;

    /* Byte 21 bits[3:0]. 2 bits each: 0 none, 1 vehicle present, 2 do not
     * change lanes, 3 SNA. Byte 20 had only two spare bits and four were
     * needed, so this cost a byte and a version.
     *
     * On the phone these OVERRIDE the turn signal on the same side bar --
     * the warning outranks the intention. */
    uint8_t blind_spot_left;
    uint8_t blind_spot_right;
    bool brake_applied;

    /* bit 4. Whether the capture recorder is actually recording -- the RING is
     * allocated, not merely the operator's wish.
     *
     * Added because the phone had no way to know. The recorder defaults to off,
     * and until BB_ENABLE existed there was no way to turn it on at all; with
     * the button but without this bit, the app knows the state right after
     * pressing it and forgets on the next reconnect. The one moment that
     * matters is the moment before a capture that cannot be retaken.
     *
     * A spare bit rather than a new field, so the length and the version are
     * unchanged. */
    bool blackbox_recording;

    uint8_t op_mode;   // 0=ListenOnly 1=Active 2=Service 3=Autonomous
    uint8_t hw_version; // 0=Unknown 1=Legacy 2=HW3 3=HW4
    int32_t speed_profile; // clamped into 0..3 by the packer
    uint8_t ap_state;      // DAS_autopilotState, raw

    float speed_kph;   // negatives clamped to 0, sent x10, saturating
    float soc_percent; // clamped 0..100

    uint8_t gear; // 0=INVALID 1=P 2=R 3=N 4=D 7=SNA

    bool speed_limit_seen;
    float speed_limit_kph; // sent only when seen; 0 otherwise

    uint16_t rx_fps;
    uint32_t crc_err_count; // saturates at 0xFFFF
    uint32_t uptime_s;
} FsdWireState;

/* Everything CamStat is built from. */
typedef struct {
    bool autonomy_enabled; // operator intent, NVS-persisted
    bool supervised_ok;    // FsdSupVerdict == OK
    bool db_loaded;
    bool autonomy_allows; // the camera path's only door
    bool pol_suspended;   // by DRIVER OVERRIDE only — see ble_server.h
    bool learning_dirty;
    bool profile_fresh; // 0x3FD read-back fresh AND in range
    bool save_failing;

    uint8_t sup_verdict; // FsdSupVerdict
    uint8_t op_mode;
    uint32_t camera_count;
    uint32_t built_at; // database build time, Unix seconds, 0 = unknown

    uint8_t gps_verdict; // FsdGpsVerdict
    uint8_t pol_phase;   // FsdPolPhase, 3 bits
    uint8_t pol_action;  // FsdPolAction, 2 bits
    uint8_t pol_target;  // target profile, 2 bits

    uint16_t nearest_m;        // 0xFFFF = nothing tracked
    uint8_t gps_accuracy_raw;  // 0.2 m units, 0xFF = unknown
    uint8_t raw_profile;       // last 0x3FD decode, RAW, 0xFF = never
    uint16_t learned_count;    // saturates into a byte
    uint16_t scan_full_count;  // saturates into a byte
} FsdWireCamStat;

/** How long a speed limit stays believable after the car last reported it.
 *
 * ⚠️ **PROVISIONAL.** The right number depends on how often 0x238 and 0x399
 * actually arrive on this car, which has never been measured -- see B-2 in
 * 차량-방문-체크리스트.md. 10 s was chosen because it is long compared with any
 * plausible rate (1-25 Hz => 10 to 250 missed frames), so it fires on a real
 * loss of signal rather than on jitter.
 *
 * The criterion is deliberately NOT "the sign changed" -- it is "the car has
 * stopped telling us", which does not depend on how fast you are driving. */
#define FSD_SPEED_LIMIT_MAX_AGE_MS 10000u

/** Is a seen speed limit still supported by something the car said recently?
 *
 * 🔴 Written because the field never expired. `speed_limit_seen` only ever
 * went true, and `speed_limit_last_ms` was stamped and **never read** -- so a
 * limit picked up half an hour ago sat on the dashboard as though it were the
 * road you are on now. The whole-State freshness warning does not cover it:
 * that only says the module is talking.
 *
 * Unsigned subtraction handles the millis() wrap: a wrapped delta comes out
 * enormous, which reads as stale. That is the safe direction. */
bool fsd_speed_limit_fresh(bool seen, uint32_t last_ms, uint32_t now_ms);

/** Pack State. `out` must hold FSD_WIRE_STATE_LEN bytes and is fully written.
 *  Does every clamp and saturation itself, so a caller cannot skip one. */
void fsd_wire_pack_state(const FsdWireState* in, uint8_t* out);

/** Pack CamStat. `out` must hold FSD_WIRE_CAMSTAT_LEN bytes. */
void fsd_wire_pack_camstat(const FsdWireCamStat* in, uint8_t* out);

/** Pack a command Result. `out` must hold FSD_WIRE_RESULT_LEN bytes. */
void fsd_wire_pack_result(uint8_t cmd, uint8_t res, uint16_t extra, uint8_t* out);

#ifdef __cplusplus
}
#endif
