#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ── CAN frame ─────────────────────────────────────────────────────────────────
// Unified with the Flipper build: CanFrame is the shared CANFRAME
// (fsd_logic/fsd_types.h). Its anonymous unions expose id/dlc/data here and
// canId/data_lenght/buffer on the Flipper side over the same storage, so this
// firmware's existing frame.id / frame.dlc / frame.data accessors are unchanged.
#include "../../fsd_logic/fsd_types.h"
typedef CANFRAME CanFrame;

// ── Hardware version ──────────────────────────────────────────────────────────
// TeslaHWVersion and OpMode are defined in the shared fsd_types.h (included
// above). OpMode is numbered ListenOnly=0, Active=1, Service=2 — the
// ListenOnly/Active values match what this firmware already persists in NVS.

// ── Operation mode ────────────────────────────────────────────────────────────
// (OpMode: see fsd_types.h)

// ── Full FSD state (shared with the Flipper build) ────────────────────────────
#include "../../fsd_logic/fsd_state.h"

// ── API ───────────────────────────────────────────────────────────────────────

/** Initialise state with safe defaults for a given HW version. */
void fsd_state_init(FSDState *state, TeslaHWVersion hw);

/** Update state for a newly detected HW version (preserves all settings). */
void fsd_apply_hw_version(FSDState *state, TeslaHWVersion hw);

/** How long the bus may stay quiet before TX is held off. Long enough to ride
 *  out a normal gap, short enough that a pulled connector is caught fast. */
#define FSD_RX_STALE_MS 2000u

/** True when no frame has arrived for FSD_RX_STALE_MS — or when none has ever
 *  arrived. Fail-closed: before the first frame we are blind, which is exactly
 *  when we must not transmit. Not latched; the bus coming back clears it, since
 *  a car going to sleep and waking is normal. Call once per loop and store the
 *  result in state->rx_stale. */
bool fsd_rx_is_stale(const FSDState *state, uint32_t now_ms);

/** Returns true if current state allows transmitting CAN frames. */
bool fsd_can_transmit(const FSDState *state);

/** AP-First stability window (ms): AP must hold engaged this long before AP/FSD/nag
 *  injection is allowed when ap_first is on. */
#define AP_FIRST_STABLE_MS 1000u

// Minimal Inject (#108): when ap_first_minimal is on, only this many AP-enable
// frames are injected at the start of each engagement, then injection stops until
// the car disengages — moving injection to engage onset and off the later abort
// edge. A few frames reliably trigger FSD; tunable. (Value matches fsd_logic.)
#define AP_MINIMAL_INJECT_FRAMES 5u

/** AP-First gate: true if injection is allowed now. When ap_first is off, always
 *  true. When on, requires das_ap_state >= 2 stable for AP_FIRST_STABLE_MS.
 *  now_ms = millis(); ap_unstable_tick_ms is stamped whenever das_ap_state < 2. */
bool fsd_ap_first_allows(const FSDState *state, uint32_t now_ms);

/** Soft Engage: |steering angle| within this of centre before activation begins. */
#define SOFT_ENGAGE_ANGLE_DEG 5.0f

// Nag burst/pause (#122): echo NAG_BURST_MS, rest NAG_PAUSE_MS — the rest period
// is the believed reason a TSL6P-style device evades stricter 14.x detection.
#define NAG_BURST_MS 1000u
#define NAG_PAUSE_MS 1500u
// Nag torque hard cap ±1.8 Nm (raw 1870..2230) — over ±1.8 Nm has been reported
// to trigger FSD disengagements during turns (#122).
#define NAG_TORQUE_RAW_MAX 2230
#define NAG_TORQUE_RAW_MIN 1870
// Configurable signal-mapping context freshness window (#122).
#define NAG_CTX_FRESH_MS 1000u

/** Apply configurable signal mapping (#122): extract DAS/steering from the
 *  user-configured positions when cfg_*_id is set, and stamp the freshness clock. */
void fsd_apply_signal_config(FSDState *state, const CanFrame *frame, uint32_t now_ms);
/** True if the DAS context is fresh (auto mode always true; configured requires
 *  a cfg-DAS frame within NAG_CTX_FRESH_MS). */
bool fsd_das_ctx_fresh(const FSDState *state, uint32_t now_ms);

/** Soft-Engage gate (steer-jerk mitigation, #108). Returns true if injection may
 *  proceed: soft_engage off, already latched, or wheel within SOFT_ENGAGE_ANGLE_DEG
 *  of centre (latches it on). Mutates soft_engage_latched; reset it when AP drops. */
bool fsd_soft_engage_allows(FSDState *state);

// DAS_autopilotState: 2 = AVAILABLE (offered, not engaged), 3 = first genuinely
// engaged state. AP-First gates injection at >= ENGAGED so it never fires while
// AP is merely available (#108).
#define DAS_APSTATE_ENGAGED  3u
// Abort Guard (#108): DAS_autopilotState values meaning the car is aborting.
#define DAS_APSTATE_ABORTING 8u
#define DAS_APSTATE_ABORTED  9u

/** Abort-Guard latch maintenance — call once per RX frame after das_ap_state is
 *  updated. Sets abort_guard_latched on an abort state (8/9), clears it on a clean
 *  disengage (das_ap_state < 2). No-op when abort_guard is off. */
void fsd_abort_guard_update(FSDState *state);

/** Abort-Guard gate. Returns false (suppress injection) only when abort_guard is
 *  on AND an abort was latched this engagement. */
bool fsd_abort_guard_allows(const FSDState *state);

/** Parse SCCM_steeringAngleSensor (0x129) -> steering_angle_deg. */
void fsd_handle_steering_angle(FSDState *state, const CanFrame *frame);

/** Read GTW_carConfig (0x398) to detect HW version.
 *  Returns TeslaHW_Unknown if frame is not 0x398 or version unrecognised. */
TeslaHWVersion fsd_detect_hw_version(const CanFrame *frame);

/** Parse GTW_carState (0x318) — updates tesla_ota_in_progress. */
void fsd_handle_gtw_car_state(FSDState *state, const CanFrame *frame);

/** Parse DAS_followDistance (0x3F8) — updates speed_profile from stalk. */
void fsd_handle_follow_distance(FSDState *state, const CanFrame *frame);

/** Modify UI_driverAssistControl (0x3F8) — opt-in RHD driving-side override (#66).
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_driver_assist_override(FSDState *state, CanFrame *frame);

/** Modify DAS_autopilotControl (0x3FD) for HW3/HW4.
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_autopilot_frame(FSDState *state, CanFrame *frame);

/** Parse STW_ACTN_RQ (0x045) for Legacy stalk position → speed_profile. */
void fsd_handle_legacy_stalk(FSDState *state, const CanFrame *frame);

/** Modify DAS_autopilot (0x3EE) for Legacy/HW1/HW2.
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_legacy_autopilot(FSDState *state, CanFrame *frame);

/** Modify ISA speed limit frame (0x399, HW4 only) to suppress speed chime.
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_isa_speed_chime(CanFrame *frame);

/* Is the nag killer compiled into this board's firmware at all?
 *
 * 🔴 Default 1 keeps upstream's behaviour for the seven boards that exist to run
 * this feature. tesla-can-mod's T-2CAN sets it to 0 in platformio.ini, because
 * that board is wired permanently into a car whose owner has full FSD and does
 * not use nag suppression (CLAUDE.md lists it under "제외" — excluded).
 *
 * Why a compile-time switch and not just a default of false: fsd_can_transmit()
 * asks only about op_mode, OTA and bus silence. It never consults
 * fsd_supervised_drive(). So on that board Active is not "transmission is now
 * permitted" — it is "transmission has now started", and with nag_killer set the
 * thing that starts is a FORGED EPAS steering-torque frame on 0x370. One press of
 * the BOOT button (dispatch_clicks(1) -> mode_apply(Active)) was enough, with the
 * car parked and nobody in it. A runtime default would have been re-enabled by
 * any stale NVS "nag" key; this cannot be.
 *
 * Removing the feature from that board is the narrow fix. The wide one — making
 * general TX require supervision — is a structural change and is deliberately
 * NOT bundled here. */
#ifndef FSD_NAG_KILLER_ENABLED
#define FSD_NAG_KILLER_ENABLED 1
#endif

/* May the ignore_ota flag override the "no TX during a Tesla OTA" rule?
 *
 * Default 1 preserves upstream's escape hatch. tesla-can-mod's T-2CAN sets 0,
 * which makes this build's fsd_can_transmit() byte-for-byte equivalent to the
 * fsd_logic twin that the host tests actually compile — the two had drifted, so
 * a test asserting "we stay quiet during an OTA" passed while the firmware did
 * not. A flag that only exists in the untested copy is the worst place for it. */
#ifndef FSD_ALLOW_IGNORE_OTA
#define FSD_ALLOW_IGNORE_OTA 1
#endif

/** Build an echo of EPAS3P_sysStatus (0x370) with counter+1 and handsOnLevel=1.
 *  Writes result into *out.  Returns true if echo should be sent.
 *  Always returns false when FSD_NAG_KILLER_ENABLED is 0. */
bool fsd_handle_nag_killer(FSDState *state, const CanFrame *frame, CanFrame *out,
                           uint32_t now_ms);

/** Parse EPAS3P_sysStatus (0x370) steering torque. */
void fsd_handle_epas_status(FSDState *state, const CanFrame *frame);

/** Parse ESP_status (0x145) brake pedal state. */
void fsd_handle_esp_status(FSDState *state, const CanFrame *frame);

/** Parse BMS_hvBusStatus (0x132) — updates pack_voltage_v / pack_current_a. */
void fsd_handle_bms_hv(FSDState *state, const CanFrame *frame);

/** Parse BMS_socStatus (0x292) — updates soc_percent. */
void fsd_handle_bms_soc(FSDState *state, const CanFrame *frame);

/** Parse BMS_thermalStatus (0x312) — updates batt_temp_min/max_c. */
void fsd_handle_bms_thermal(FSDState *state, const CanFrame *frame);

/** Build a UI_tripPlanning (0x082) frame to trigger active battery heating. */
void fsd_build_precondition_frame(CanFrame *frame);

/** Handle CAN ID 0x331 — TLSSC Restore via DAS config spoof.
 *  Overwrites byte[0] lower 6 bits to 0x1B (SELF_DRIVING).
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_tlssc_restore(FSDState *state, CanFrame *frame);

/** Parse DAS_status from Legacy/HW3 0x399 — updates AP/speed/hands-on state. */
void fsd_handle_das_status_hw3(FSDState *state, const CanFrame *frame);

/** Parse DAS_status from HW4 0x39B — updates AP/speed/hands-on state. */
void fsd_handle_das_status_hw4(FSDState *state, const CanFrame *frame);

/** HW4 hands-on fallback: read only DAS_handsOnState (byte5[5:2]) from 0x399,
 *  for HW4 trims that never broadcast 0x39B. Call only when
 *  das_hw4_status_seen is false. Read-only, leaves das_ap_state untouched. */
void fsd_handle_das_handsonly_399(FSDState *state, const CanFrame *frame);

/** Parse GearLever / right stalk 0x229 for right-stalk detents. */
void fsd_handle_gear_lever(FSDState *state, const CanFrame *frame, uint32_t now_ms);

/** Parse UI_driverAssistMapData 0x238 map/location speed limit. */
void fsd_handle_ui_map_data(FSDState *state, const CanFrame *frame, uint32_t now_ms);

/** Parse DAS_status2 0x389 ACC speed-limit readback. */
void fsd_handle_das_status2(FSDState *state, const CanFrame *frame, uint32_t now_ms);

/** Parse DAS_control 0x2B9 cruise/AP set speed. */
void fsd_handle_das_control(FSDState *state, const CanFrame *frame);

/** Parse VCFRONT_lighting 0x3F5 turn signal request state. */
void fsd_handle_vcfront_lighting(FSDState *state, const CanFrame *frame);

/** Build GearLever / right stalk 0x229 with rolling counter and CRC byte. */
bool fsd_build_gear_lever_frame(CanFrame *frame, uint8_t gear_pos, uint8_t counter);
