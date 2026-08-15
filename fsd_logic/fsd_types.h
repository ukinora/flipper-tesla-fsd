#pragma once
/*
 * fsd_types.h — hardware-free CAN frame type shared by the protocol logic.
 *
 * Split out of libraries/mcp_can_2515.h so the FSD protocol core
 * (fsd_handler.c) compiles without pulling in the MCP2515 SPI driver or any
 * furi/HAL dependency. This lets the logic be unit-tested on the host (see
 * test/) and is the first step toward a single protocol core shared by the
 * Flipper and ESP32 builds.
 *
 * The CANFRAME layout is unchanged — same fields, same order, same MAX_LEN —
 * so this is behavior-preserving for the existing Flipper build.
 */

#include <stdbool.h>   // fsd_mode_opens_tx() below
#include <stdint.h>

#define MAX_LEN 8

// CAN frame shared by the MCP2515 driver, the FSD protocol logic, and the ESP32
// firmware. The anonymous unions expose two field-name conventions over the
// same storage so both platforms keep their existing accessors while sharing
// one struct definition:
//   Flipper / mcp driver : canId / data_lenght / buffer
//   ESP32 firmware        : id    / dlc         / data   (CanFrame == CANFRAME)
// Same size and layout as before, so this is behavior-preserving for the
// existing Flipper build.
typedef struct {
    union { uint32_t canId; uint32_t id; };
    uint8_t ext;
    uint8_t req;
    union { uint8_t data_lenght; uint8_t dlc; };
    union { uint8_t buffer[MAX_LEN]; uint8_t data[MAX_LEN]; };
} CANFRAME;

// ── Hardware version (shared by both platforms) ───────────────────────────────
typedef enum {
    TeslaHW_Unknown = 0,
    TeslaHW_Legacy,   // HW1 / HW2 / EAP retrofit — uses 0x3EE / 0x045
    TeslaHW_HW3,
    TeslaHW_HW4,
} TeslaHWVersion;

// ── Operation mode (shared by both platforms) ─────────────────────────────────
// Numbering matches the ESP32's persisted NVS values (ListenOnly=0, Active=1),
// so unifying the enum needs no migration; the ESP32 web UI's op_mode===1==Active
// check also stays valid. Service is Flipper-only (already 2 there). ListenOnly
// is the safe boot default — no TX.
typedef enum {
    OpMode_ListenOnly = 0,  // pure passive sniff, no TX at all
    OpMode_Active,          // RX + TX, normal operation
    OpMode_Service,         // unrestricted, gates aggressive features (Flipper)
    // Single-purpose mode for unattended speed-camera response, added for the
    // car-resident build. NOT a superset of Active and not "more permissive
    // because the number is bigger": fsd_can_transmit() returns FALSE here, so
    // every general TX path is as closed as it is in Listen-Only. Only the
    // camera path has its own door (fsd_autonomy.h), and that door additionally
    // requires evidence that a person is driving.
    //
    // Active is a blanket permission — nag killer, GTW overrides, body control,
    // hazards. Handing that to a module with nobody in the car, to get one
    // scroll detent, is far more than the job needs.
    OpMode_Autonomous,
} OpMode;

/**
 * Does this mode open the CAN controller's transmitter?
 *
 * The CAN driver has a hardware listen-only register, and Listen-Only's whole
 * safety claim is that it is "physically incapable of TX even on bus error
 * frames". That claim only holds if the register tracks op_mode -- and it did
 * not: the BLE SET_MODE handler set op_mode and acknowledged success without
 * touching the driver, so the app read Active while the hardware could not
 * transmit, and the revoke path lowered op_mode while leaving a driver that
 * somebody else had opened.
 *
 * Inline in the header on purpose. fsd_can_transmit() exists twice -- once in
 * fsd_logic/fsd_handler.c for the Flipper and the host tests, once in
 * esp32/.firmware/fsd_handler.cpp because that build does not compile the
 * fsd_logic twin -- and the shim that drives the register is a third place.
 * Three copies of an allow-list drift; one predicate cannot.
 *
 * Allow-list, for the same reason fsd_can_transmit() is one: a mode added later
 * must default to a closed transmitter rather than inherit an open one.
 */
static inline bool fsd_mode_opens_tx(OpMode m) {
    return m == OpMode_Active || m == OpMode_Service;
}

/* ── 0x257 DI_speed carries TWO speeds, and they are not the same thing ──────
 *
 * Inline here, next to fsd_mode_opens_tx() and for the same reason: two places
 * need these and they must not drift. fsd_gps.c uses the first for its freeze
 * detector, and the drive observer publishes both into FSDState -- but
 * fsd_gps.c is only linked on the camera variant, so a shared .c would break
 * the link on every other board.
 */

/* DI_vehicleSpeed : 12|12@1+ (0.08,-40). ALWAYS km/h, whatever the car shows. */
static inline bool fsd_decode_di_speed_kph(const uint8_t* d, uint8_t dlc, float* out) {
    if(!d || !out || dlc < 3) return false;
    const uint16_t raw = (uint16_t)(((uint16_t)d[2] << 4) | (uint16_t)(d[1] >> 4));

    /* 4095 is SNA (signal not available), not a speed. The field is 12 bits, so
     * all-ones is the conventional "no value" marker -- and 0.08*4095-40 works
     * out to 287.6 km/h, a number that LOOKS like a speed and is not one.
     *
     * This reaches further than the display. vehicle_speed_kph is an input to
     * two safety decisions: fsd_profile.c refuses to act above 0.5 km/h, and
     * fsd_gps.c counts "kept moving fast" as one of the three conditions for
     * FROZEN. A missing signal decoded as 287.6 makes both fail CLOSED -- safe,
     * but under the wrong name, and a bench session gets spent chasing a
     * phantom. Returning false instead leaves speed_seen false, which every
     * consumer already treats as "no evidence the car is moving".
     *
     * Found by comparing against ev-open-can-tools (GPL-3.0): its injection
     * policy rejects raw > 4062 for exactly this reason. We reject only the SNA
     * value itself -- 4062 is their constant, not a measurement, and this
     * project has already been bitten once by adopting someone else's numbers
     * without evidence (the 0x286 gear read, PR #18). If a capture ever shows
     * this field pinned high below 4095, revisit with that evidence. */
    if(raw >= 4095u) return false;

    float kph = (float)raw * 0.08f - 40.0f;
    if(kph < 0.0f) kph = 0.0f;
    *out = kph;
    return true;
}

/* DI_uiSpeed : 24|8@1+ raw -- the number on the car's own display.
 *
 * 🔴 IN THE CAR'S DISPLAY UNIT, not always km/h. A car set to mph reports mph
 * here. That is exactly why it is worth having alongside the one above: the
 * ratio between them tells you which unit the car is set to, and no other frame
 * we parse says that. It is also the right source for a speedometer, because it
 * is by definition the number the driver is looking at.
 *
 * Source: canhackers/Saturn (MIT), cross-checked against opendbc. NOT yet
 * confirmed on our car -- see 차량-방문-체크리스트.md.
 *
 * ⚠️ NO SNA check here, deliberately -- unlike the decoder above. This field is
 * 8 bits, and 255 is physically reachable on this platform when the car is set
 * to km/h, so treating all-ones as "no value" could discard a real reading. The
 * asymmetry is about consequence, not consistency: the decoder above feeds
 * safety gates, this one feeds a display where a wrong number is visible to the
 * person who can act on it. If a capture ever shows what SNA looks like here,
 * add the check with that evidence.
 */
static inline bool fsd_decode_ui_speed(const uint8_t* d, uint8_t dlc, uint8_t* out) {
    if(!d || !out || dlc < 4) return false;
    *out = d[3];
    return true;
}

// Abort Guard (#108): DAS_autopilotState values that mean the car is aborting an
// engage — the moment linked to the steer-jerk in dunckencn's logs.
//
// These live here rather than in fsd_handler.h because fsd_events.h needs them
// and nothing else from that header. Including fsd_handler.h for two constants
// dragged its 38 CAN ID macros into every file that touches events -- which on
// the ESP32 is most of them, and where config.h has already defined 24 of the
// same names. That produced a wall of "macro redefined" warnings on every
// build, and a build that is never warning-clean is one where a NEW warning is
// invisible. test/check_can_ids.py now guards the two lists instead.
#define DAS_APSTATE_ABORTING 8u
#define DAS_APSTATE_ABORTED  9u
