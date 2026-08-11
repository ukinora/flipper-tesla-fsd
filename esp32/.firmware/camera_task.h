#pragma once
/*
 * camera_task.h — the camera judgement pipeline, on the module.
 *
 * WHAT THIS FINALLY CONNECTS
 * -------------------------
 * fsd_gps.c makes an FsdCamFix, fsd_cam_track.c follows cameras and learns which
 * ones are ours, fsd_cam_policy.c decides what profile to ask for. All three are
 * host-tested and, until this file, none of them ran on a car: nothing
 * instantiated them, so camera_store_load_learning() / save_learning() had zero
 * callers and FsdTracker.dirty could never become true outside a test.
 *
 * IT DOES NOT TRANSMIT. AT ALL.
 * -----------------------------
 * The decision below is published and never acted on. There is no send path in
 * this file, no FsdSpeedProfile is declared anywhere in this firmware, and
 * fsd_sp_request / fsd_sp_poll / fsd_sp_apply_scroll have no callers. The scroll
 * injection stays behind both of its locks (fsd_speed_profile.h SAFETY), because
 * the wire encoding has not been captured on this car and an Intel HW3 / MCU2
 * emergency-braking incident is on record for exactly this injection.
 *
 * What this buys before any of that: the whole judgement chain runs on real GPS
 * on a real drive, learning accumulates and survives a power cycle, and the app
 * can watch every refusal by name. That is what makes the eventual first tick a
 * measurement rather than a leap.
 *
 * THREADING
 * ---------
 * Every entry point here runs on the Arduino loop task. process_frame() has
 * exactly one caller, inside loop(); camera_task_tick() is called from loop();
 * the accessors are read from ble_server_tick(), which loop() also calls. This
 * firmware creates no tasks of its own.
 *
 * So nothing guards the instances in this file, and NOTHING HERE MAY EVER BE
 * CALLED FROM A NimBLE CALLBACK. The one genuinely cross-task object is the
 * camera database, which is why it is borrowed through camera_store's lock
 * rather than held.
 */

#include "../../fsd_logic/fsd_state.h"

#include <freertos/FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef BLE_SERVER_ENABLED

/** Wire up the instances and restore learning from flash. Call once from
 *  setup(), after camera_store_init() (it needs the filesystem) and before
 *  ble_server_init() (the BLE packer reads the accessors below). */
void camera_task_init(FSDState* state, portMUX_TYPE* mux);

/** Offer a frame to the GPS observers. Returns true when the id was one of
 *  ours (0x3D8 / 0x2F8 / 0x257) and the caller should stop dispatching it.
 *  Cheap: three integer comparisons for everything else. */
bool camera_task_observe(uint32_t id, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

/** Offer a 0x3FD frame for the speed-profile read-back. Must be given the
 *  ORIGINAL frame — the injection path works on a copy, and reading that copy
 *  would feed our own writes back as if the car had moved. Non-returning: the
 *  AP-control handler still needs this frame. */
void camera_task_observe_profile(bool hw4, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

/** Run the judgement at its own cadence. Call every loop(); it rate-limits
 *  itself and does nothing at all until a fix, authority and a read-back all
 *  line up. */
void camera_task_tick(uint32_t now_ms);

// ── read-only accessors, for the BLE CamStat packer ──────────────────────────
// Values, not pointers: the packer must not be able to hold a reference into
// state that the next tick rewrites.

uint8_t  camera_task_gps_verdict(void);      // FsdGpsVerdict; never structurally 0
uint8_t  camera_task_pol_phase(void);        // FsdPolPhase
uint8_t  camera_task_pol_action(void);       // FsdPolAction
uint8_t  camera_task_pol_target(void);       // profile the policy is asking for
bool     camera_task_pol_suspended(void);
uint16_t camera_task_nearest_m(void);        // 0xFFFF = none, 0xFFFE = >= 65534 m
uint8_t  camera_task_gps_accuracy_raw(void); // 0.2 m units; 0xFF = unknown
uint8_t  camera_task_raw_profile(void);      // last 0x3FD decode, 0xFF = never
bool     camera_task_profile_fresh(void);    // fresh AND inside 0..3
bool     camera_task_learning_dirty(void);   // passes recorded but not written
bool     camera_task_save_failing(void);     // writes are failing, not silent
uint16_t camera_task_learned_count(void);
uint16_t camera_task_scan_full_count(void);

/** The DRIVETRAIN speed, off 0x257 — the one named source in this firmware.
 *
 *  Named and exported so that nothing wires an interlock to GPS velocity by
 *  mistake: UI_gpsVehicleSpeed freezes in a tunnel along with the position,
 *  which is the whole reason fsd_gps.c requires this frame in the first place.
 *  Anything asking "is the car moving" for a safety decision asks here. */
bool camera_task_ref_speed_seen(void);
float camera_task_ref_speed_kph(void);
uint32_t camera_task_ref_speed_ms(void);

#else  // no camera core on this variant — no-op shims so main.cpp needs no #ifdef

static inline void camera_task_init(FSDState*, portMUX_TYPE*) {}
// MUST be false: it is what lets the main.cpp call site fold away entirely.
static inline bool camera_task_observe(uint32_t, const uint8_t*, uint8_t, uint32_t) { return false; }
static inline void camera_task_observe_profile(bool, const uint8_t*, uint8_t, uint32_t) {}
static inline void camera_task_tick(uint32_t) {}
static inline uint8_t  camera_task_gps_verdict(void) { return 0; }
static inline uint8_t  camera_task_pol_phase(void) { return 0; }
static inline uint8_t  camera_task_pol_action(void) { return 0; }
static inline uint8_t  camera_task_pol_target(void) { return 0; }
static inline bool     camera_task_pol_suspended(void) { return false; }
static inline uint16_t camera_task_nearest_m(void) { return 0xFFFFu; }
static inline uint8_t  camera_task_gps_accuracy_raw(void) { return 0xFFu; }
static inline uint8_t  camera_task_raw_profile(void) { return 0xFFu; }
static inline bool     camera_task_profile_fresh(void) { return false; }
static inline bool     camera_task_learning_dirty(void) { return false; }
static inline bool     camera_task_save_failing(void) { return false; }
static inline uint16_t camera_task_learned_count(void) { return 0; }
static inline uint16_t camera_task_scan_full_count(void) { return 0; }
static inline bool     camera_task_ref_speed_seen(void) { return false; }
static inline float    camera_task_ref_speed_kph(void) { return 0.0f; }
static inline uint32_t camera_task_ref_speed_ms(void) { return 0; }

#endif
