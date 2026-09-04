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

/**
 * Does this mode open the CAN controller's transmit REGISTER?
 *
 * Identical to fsd_mode_opens_tx() today, and deliberately so — this is a
 * seam, not a behaviour change.
 *
 * 🔴 Why it exists. The comment on OpMode_Autonomous above says the camera path
 * "has its own door" separate from general TX. It does not, yet: mode_apply()
 * (which drives the register) and fsd_can_transmit() (which permits general TX)
 * both asked fsd_mode_opens_tx(). One predicate, two jobs. So the sentence
 * "fsd_can_transmit() returns FALSE here" is not a separate design — it is a
 * side effect of the register being shut in Autonomous too.
 *
 * That matters the day the scroll emitter is wired. Autonomous has to open the
 * register for a detent to reach the wire, and with one predicate the only way
 * to do that is to add Autonomous to it — which hands the SAME key to
 * fsd_can_transmit(), opening nag killer, 0x229 gear-lever, precondition and
 * 0x3FD injection on a car with nobody in it. A one-line edit made to enable one
 * frame would enable eight.
 *
 * With the seam in place that edit becomes: add Autonomous HERE (register may
 * open) and leave fsd_mode_opens_tx() alone (general TX stays shut). Splitting
 * it now costs nothing; splitting it under pressure, later, is how the wrong
 * eight get opened.
 */
static inline bool fsd_mode_opens_hw_tx(OpMode m) {
    return fsd_mode_opens_tx(m);
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

/* 0x3FD DAS_autopilotControl -- the speed profile the car is ACTUALLY on.
 *
 * 🔴 MEASURED ON THIS CAR, and it is not where either documented layout says
 * (2026-09-03, captures/2026-09-03/속도프로파일4단계).
 *
 * fsd_sp_decode_profile() knows two layouts:
 *     HW3   mux 0, byte 6 bits[2:1]   -- fixed at 0 through the whole drive here
 *     HW4   mux 2, byte 7 bits[7:5]   -- collides: 0x80 and 0x90 both read 4
 *
 * This car uses mux 2, byte 7 bits[6:4]. One bit lower than the HW4 layout,
 * which is the difference between four distinct values and two.
 *
 * How it was pinned: the right scroll wheel (0x3C2 mux 0x29 byte3, 6-bit
 * signed) ticked +1 three times, and this field changed 220-300 ms after each
 * tick, in step, in the order the owner wrote down.
 *
 *     wheel +1 @ 6.401  ->  6.699  byte7 = 0x80
 *     wheel +1 @ 7.501  ->  7.722  byte7 = 0x90
 *     wheel +1 @ 8.901  ->  9.198  byte7 = 0xA0
 *
 * With the earlier down-ticks landing on 0xC0, the ladder the owner drove is:
 *
 *     나무늘보 0xC0 -> 4     컴포트 0x80 -> 0
 *     스탠더드 0x90 -> 1     신속   0xA0 -> 2
 *
 * 🟢 Chill/Standard/Hurry come out consecutive (0,1,2) and Sloth sits apart
 * at 4 -- which is what a level added later to an existing enum looks like.
 * That is an OBSERVATION about the numbers, not an assumption feeding code.
 *
 * 🔴 This is the OBSERVED profile. Do not merge it with
 * FSDState.speed_profile, which is what we intend to WRITE -- PR #17 already
 * paid for that confusion once, when the policy clamp read an intent as a
 * measurement.
 */
#define FSD_PROFILE_MUX_MASK   0x03u
#define FSD_PROFILE_MUX        2u
#define FSD_PROFILE_BYTE       7u
#define FSD_PROFILE_SHIFT      4u
#define FSD_PROFILE_MASK       0x07u

static inline bool fsd_decode_profile_obs(const uint8_t* d, uint8_t dlc, uint8_t* out) {
    if(!d || !out || dlc < 8u) return false;
    if((d[0] & FSD_PROFILE_MUX_MASK) != FSD_PROFILE_MUX) return false;
    *out = (uint8_t)((d[FSD_PROFILE_BYTE] >> FSD_PROFILE_SHIFT) & FSD_PROFILE_MASK);
    return true;
}

/* 0x39B DAS_status byte0 low nibble -- DAS_autopilotState, HW3 positions.
 *
 * 🔴 THIS CAR PUBLISHES AP STATE ON THE "HW4" FRAME (2026-09-03).
 *
 * 2021 Model 3, HW3 hardware, Korean firmware 2026.20.7.5:
 *
 *     0x399  THREE bytes, 000000, never moves -- even with FSD engaged
 *     0x39B  EIGHT bytes, and byte0 tracks the car:
 *
 *         parked, FSD off   byte0 = 0x01     (4 captures)
 *         driving, FSD on   byte0 = 0x02 then 0x06
 *
 * fsd_handle_das_status_hw3() requires dlc == 8, so on 0x399 it returns on
 * every single frame: das_ap_state stayed 0 and the dashboard read
 * "쓸 수 없음" for an entire drive with FSD engaged.
 *
 * 🔴 The standard HW4 parser does NOT help here. It reads byte1 bits[7:4],
 * and on this car byte1 is 0x02 in every capture -- that nibble is 0. It
 * would report "unavailable" just as wrongly, from a different byte.
 *
 * 🟢 The values land exactly on the HW3 table the app already names:
 * 1 = unavailable, 2 = ready, 6 = engaged. Nothing had to be invented.
 *
 * Evidence: captures/2026-09-03/{유휴,맵등 켜기,우측앞문열기,조수석시트앞뒤}
 * versus captures/2026-09-03/속도프로파일4단계 (the one drive).
 */
/* Turn indicators and hazards, from 0x3F5 VCFRONT_lighting byte 0.
 *
 * Measured 2026-09-05. This car's 0x311 UI_warning is TWO BYTES, so the
 * dashboard's blinker fields -- documented in fsd_state.h as "UNVERIFIED on
 * this car... the visit settles it" -- were never written at all. The visit
 * settled it: not by confirming 0x311, but by finding the lamps elsewhere.
 *
 *      bit 0  left,  dark phase        bit 2  right, dark phase
 *      bit 1  left,  lit  phase        bit 3  right, lit  phase
 *      bit 4  hazards
 *
 * The 2-bit-per-side layout is NOT new -- can_signals.h already carried these
 * shifts. What was new is that they are real on this car, and which of the two
 * phases means "lit":
 *
 *      0x3F5 b0 = 0x02  <->  0x3E2 VCLEFT_lightStatus = 12 00 84 ...
 *      0x3F5 b0 = 0x01  <->  0x3E2 VCLEFT_lightStatus = 02 00 80 ...  (baseline)
 *      0x3F5 b0 = 0x08  <->  0x3E3 = 10000000
 *      0x3F5 b0 = 0x04  <->  0x3E3 = 00000000
 *
 * Two frames from two controllers agree: the HIGHER bit of each pair is the
 * lamp actually alight. That matches the wire encoding (0 off / 1 dark / 2 lit)
 * exactly, which is why this returns the raw 2-bit field untouched.
 *
 * Cross-checked against the hazards: a person pressing the button gives
 * 0x1A <-> 0x15, and TSL turning them on for reverse gives 0x7A <-> 0x75 --
 * identical in the low five bits, and 0x1A = 0x08|0x02|0x10 is precisely both
 * sides lit at once.
 *
 * 🔴 Which pair is LEFT is measured, not assumed. Two independent lines
 * agree: VCLEFT changes its own byte during the 0x01/0x02 segment, and the
 * owner confirmed the group blinking immediately before the hazards was the
 * RIGHT one -- which in the capture is 0x08/0x04. */
#define FSD_BLINK_LEFT_SHIFT  0u
#define FSD_BLINK_RIGHT_SHIFT 2u
#define FSD_BLINK_MASK        0x03u
#define FSD_BLINK_HAZARD      0x10u

static inline bool fsd_decode_blinkers(const uint8_t* d, uint8_t dlc,
                                       uint8_t* left, uint8_t* right,
                                       bool* hazard) {
    if(!d || dlc < 1u) return false;
    if(left) *left = (uint8_t)((d[0] >> FSD_BLINK_LEFT_SHIFT) & FSD_BLINK_MASK);
    if(right) *right = (uint8_t)((d[0] >> FSD_BLINK_RIGHT_SHIFT) & FSD_BLINK_MASK);
    if(hazard) *hazard = (d[0] & FSD_BLINK_HAZARD) != 0u;
    return true;
}

static inline bool fsd_decode_das_state_b0(const uint8_t* d, uint8_t dlc, uint8_t* out) {
    if(!d || !out || dlc < 1u) return false;
    *out = d[0] & 0x0Fu;
    return true;
}

/* 0x219 VCSEC_TPMSData -- the four live tyre pressures.
 *
 * 🔴 THE OLD READ WAS WRONG AND THE CAR SAID SO (2026-09-03).
 *
 * It took `data[0] & 3` as "which wheel" and `data[1]` as that wheel's
 * pressure. But byte 0 is a MUX, not a wheel index, and this car sends at
 * least six of them:
 *
 *     00 80 FF FF FF F8 0F     mux 0..3 -- byte1 = 0x80 on all four, never
 *     01 80 FF FF FF F9 0F                 moves. NOT a pressure.
 *     02 80 FF FF FF FB 0F
 *     03 80 FF FF FF FA 0F
 *     04 74 74 00 00 00 00     mux 4  -- 0x74 twice, identical on two days
 *                                        two days apart. A placard value.
 *     05 00 75 76 74 75 00     mux 5  -- FOUR values, and they MOVED between
 *                                        2026-09-01 (7A x4 = 44.3 psi) and
 *                                        2026-09-03 (42.4/42.8/42.1/42.4).
 *
 * The owner measured 42 psi with a gauge on 2026-09-03. Only mux 5 agrees.
 *
 * Masking the mux with 3 made mux 4 overwrite wheel 0 and mux 5 overwrite
 * wheel 1 with byte1 = 0x00, which is exactly what the dashboard showed:
 * front-left flickering between 42 and 46, front-right blinking empty, the
 * two rear wheels stuck at 46. The symptom named the bug.
 *
 * 🟢 Every other mux is REJECTED rather than stored. A pressure that is not
 * a pressure is worse than no reading -- the app already draws "no value"
 * for 0, and a plausible wrong number is the one nobody questions.
 *
 * 🔴 WHICH BYTE IS WHICH WHEEL IS NOT KNOWN. On the day this was decoded the
 * four readings were 42.1..42.8 psi, too close to tell apart. Set one tyre
 * to a clearly different pressure and take one capture; that settles it.
 * Until then the app lays them out by index, as it already did.
 */
#define FSD_TPMS_MUX_BYTE      0u
#define FSD_TPMS_PRESSURE_MUX  5u   /* the only mux that carries pressures */
#define FSD_TPMS_FIRST_BYTE    2u   /* d[2]..d[5] = four wheels */
#define FSD_TPMS_MIN_DLC       6u

static inline bool fsd_decode_tpms(const uint8_t* d, uint8_t dlc, uint8_t* out4) {
    if(!d || !out4 || dlc < FSD_TPMS_MIN_DLC) return false;
    if(d[FSD_TPMS_MUX_BYTE] != FSD_TPMS_PRESSURE_MUX) return false;
    for(unsigned i = 0; i < 4u; i++) out4[i] = d[FSD_TPMS_FIRST_BYTE + i];
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
