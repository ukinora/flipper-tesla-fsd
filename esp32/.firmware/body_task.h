#pragma once
/*
 * body_task.h — the wiring that lets the body detectors actually run. READ ONLY.
 *
 * WHAT A SHIM IS FOR
 * ------------------
 * fsd_body.c, fsd_body_t1.c and fsd_body_t2.c are pure C: they know nothing
 * about Arduino, the CAN drivers, millis() or FreeRTOS. That is what makes them
 * host-testable, and it is also why, until this file, the linker discarded all
 * three — nothing held an instance, nothing handed them a frame, nothing asked
 * them anything.
 *
 * This file does those four things and no more. It is the same job
 * camera_task.cpp does for the camera stack, and it is deliberately the same
 * shape.
 *
 * IT SENDS NOTHING, AND THERE IS NOTHING TO SEND
 * ----------------------------------------------
 * Not "it is configured not to send". There is no code anywhere in this feature
 * that builds a body frame: no encoding table, no apply(), nothing. T1's action
 * is an enum that gets logged. T2's gesture is a counter and two durations.
 *
 * ⚠️ THAT IS A STATEMENT ABOUT THESE DETECTORS, NOT ABOUT THE BOARD. Body
 * frames very much exist elsewhere -- rule_task.cpp puts them on the bus. What
 * is true here is that T1 and T2 have no emitter of their own and never had
 * one: T1's action is an enum that gets logged, T2's gesture is a counter and
 * two durations.
 *
 * 🔴 The old wording listed three things that would each have to change first,
 * one of them being FSD_BODY_CAPS[...].armable_at_runtime. That column is gone
 * (owner's instruction, 2026-09-10), so the honest list is two: someone would
 * have to write an emitter for these actions, and send_on_bus() would still
 * refuse the ids it refuses.
 *
 * WHY IT IS WORTH RUNNING NOW
 * ---------------------------
 * Because the detectors' present job is to MEASURE, and measurements only
 * happen if the code runs while the car is doing the thing.
 *
 * While TSL's functions are being reproduced on the car, this quietly records
 * the numbers nobody has: how long a window-button tap actually lasts, how long
 * the gap between two of them is (including the ones that were rejected and
 * why), how often 0x3C2 mux 0 is transmitted — which is the floor on how close
 * two taps can even resolve as two — and which of the nine latch enum values
 * this build actually emits.
 *
 * Without this the timing window has to be guessed, or the car visited again.
 */

#include "../../fsd_logic/fsd_body.h"   // FsdBodyAction
#include "../../fsd_logic/fsd_state.h"

#include <freertos/FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef BLE_SERVER_ENABLED

/** Hold the instances. Call once from setup(), after the state mutex exists. */
void body_task_init(FSDState* state, portMUX_TYPE* mux);

/** Offer a frame to the detectors. Returns true when the id was one of theirs
 *  (0x102, 0x103, 0x3C2) — but the CALLER MUST NOT STOP DISPATCHING ON IT:
 *  0x3C2 is shared with the scroll path, and door frames may gain other
 *  readers. Returning a bool at all is for the serial counters, not for flow
 *  control. Non-returning at the call site, on purpose. */
bool body_task_observe(uint32_t id, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

/** Advance the detectors and emit the periodic measurement line. Call from
 *  loop(); it rate-limits itself. */
void body_task_tick(uint32_t now_ms);

/** Report whether any CAN controller is currently able to transmit at all.
 *  Supplied by main.cpp, which owns the drivers. Defaults to false — a shim
 *  that cannot tell must report the bus shut. */
void body_task_set_bus_tx_open(bool open);

/** True for an ID this firmware must never put on the wire. Forwards to
 *  fsd_body_tx_id_refused(); it exists so main.cpp can call it on every variant
 *  without linking fsd_body.c, which only this one carries.
 *
 *  NOTE for anyone porting the Flipper's hazard/wiper writes over: those DO
 *  write 0x3F5 (fsd_logic/fsd_handler.c, Service mode only) and this would
 *  refuse them. That is intended for now — the ESP32 has never sent 0x3F5 and
 *  the refusal is logged, not silent — but it is the one place the two builds
 *  deliberately disagree. */
bool body_task_tx_refused(uint32_t can_id);

// ── read-only accessors, for logging and a future BLE surface ────────────────
uint16_t body_task_t1_actions(void);   // T1 actions that would have been taken
uint8_t  body_task_latch_raw(uint8_t side); // last raw nibble, 0xFF = unseen
uint16_t body_task_t2_gestures(void);
uint16_t body_task_t2_last_press_ms(void);
uint16_t body_task_t2_last_gap_ms(void);
uint8_t  body_task_t2_last_reject(void);
uint32_t body_task_mux0_period_ms(void); // smallest observed 0x3C2 mux-0 gap

#else  // no body core on this variant — no-op shims

static inline void body_task_init(FSDState*, portMUX_TYPE*) {}
static inline bool body_task_observe(uint32_t, const uint8_t*, uint8_t, uint32_t) { return false; }
static inline void body_task_tick(uint32_t) {}
static inline void body_task_set_bus_tx_open(bool) {}
/* No body feature on this variant, so nothing here would build a body frame
 * either — and the Flipper's hazard write is not part of this build. */
static inline bool body_task_tx_refused(uint32_t) { return false; }
static inline uint16_t body_task_t1_actions(void) { return 0; }
static inline uint8_t  body_task_latch_raw(uint8_t) { return 0xFFu; }
static inline uint16_t body_task_t2_gestures(void) { return 0; }
static inline uint16_t body_task_t2_last_press_ms(void) { return 0; }
static inline uint16_t body_task_t2_last_gap_ms(void) { return 0; }
static inline uint8_t  body_task_t2_last_reject(void) { return 0; }
static inline uint32_t body_task_mux0_period_ms(void) { return 0; }

#endif
