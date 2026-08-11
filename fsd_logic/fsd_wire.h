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
#define FSD_WIRE_STATE_LEN 20u
#define FSD_WIRE_CAMSTAT_LEN 20u
#define FSD_WIRE_RESULT_LEN 4u

/* Wire versions. Each payload carries its own — sharing one meant that bumping
 * State also announced a CamStat change that had not happened. */
#define FSD_WIRE_STATE_VERSION 2u
#define FSD_WIRE_CAMSTAT_VERSION 2u

/* FSD v14 Lite exposes four speed profiles. */
#define FSD_WIRE_PROFILE_MAX 3

/* Everything State is built from. Filled by the caller from FSDState.
 *
 * Note what is NOT here: flags bits 4, 5 and 7. They are structurally zero —
 * blind-spot is not extracted on the ESP32 path and the closed loop that would
 * set bit 7 emits nothing — so they are not inputs to anything. Giving them a
 * field would invite someone to fill it. */
typedef struct {
    bool rx_seen; // any CAN frame ever received
    bool ota_in_progress;
    bool blinker_left;
    bool blinker_right;
    bool brake_applied;

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
