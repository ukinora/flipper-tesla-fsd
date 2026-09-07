#pragma once

/*
 * rule_task.h — the thin layer between fsd_pipeline.c and the bus.
 *
 * Everything that DECIDES lives in fsd_logic/fsd_pipeline.c, where the host
 * tests run it on frames copied out of captures. What is left here is what
 * cannot be tested on a desk: owning the trigger state, reading the car's
 * frames as they arrive, and the one line that hands bytes to a CAN driver.
 *
 * 🔴 THE ARM SWITCH IS SESSION-ONLY AND THAT IS DELIBERATE.
 * rule_task_set_armed() is not stored anywhere. It dies with the power, which
 * on this car means every time it sleeps. A rule engine that can operate the
 * body of a car should not come back armed after a reboot nobody watched —
 * "the operator asked for this, in this session, while present" is the whole
 * claim it makes, and NVS cannot make that claim.
 *
 * It is also not enough on its own. Arming sets the per-action enables the
 * permission axis reads; the axis still asks about mode, bus, OTA, RX
 * freshness, driver, gear, speed and the drive session, and the emitter and
 * the chokepoint still have to agree afterwards.
 */

#include "../../fsd_logic/fsd_state.h"

#include <freertos/FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef BLE_SERVER_ENABLED

/** How a built frame reaches the bus.
 *
 *  main.cpp supplies this because send_on_bus() is static there, next to the
 *  mode gate and the ID refusals that still apply after everything here has
 *  said yes. Returns whether the frame actually went out. */
typedef bool (*RuleTaskSend)(uint32_t can_id, const uint8_t* data, uint8_t dlc);

/** Wire up the state pointer, its lock and the send path, and clear the
 *  trigger and template state. Call once from setup(), after
 *  body_task_init(). */
void rule_task_init(FSDState* state, portMUX_TYPE* mux, RuleTaskSend send);

/** Arm or disarm the rule engine for THIS SESSION. Never persisted.
 *
 *  Disarming is immediate and total: the permission axis refuses every action
 *  the moment this is false, so a command already decided but not yet sent
 *  cannot be sent. */
void rule_task_set_armed(bool armed);

/** Is it armed right now? For the serial line and the app. */
bool rule_task_armed(void);

/** Offer one received frame. NON-RETURNING, like body_task_observe(): several
 *  readers want the same frames.
 *
 *  Does two things — feeds the trigger layer, and stores the frame as a
 *  template for any action whose wire row names it. Both are cheap; the
 *  overwhelming majority of frames match nothing. */
void rule_task_observe(uint32_t can_id, const uint8_t* data, uint8_t dlc, uint32_t now_ms);

/** Advance held switches. LONG and STUCK arrive from here. Call from loop(). */
void rule_task_tick(uint32_t now_ms);

/* ── counters, for the serial line and the app ─────────────────────────── */

/** Frames actually put on the bus by the rule engine. */
uint32_t rule_task_sent(void);

/** Decisions that were refused, by whichever of the four layers refused. */
uint32_t rule_task_refused(void);

/** The last refusal, as "stage: reason" — or "none" if there has not been one.
 *  Never NULL. */
const char* rule_task_last_refusal(void);

/** Print a summary. The serial `ruleq` command. */
void rule_task_print(void);

#else // !BLE_SERVER_ENABLED

/* 🔴 STUBS, BECAUSE main.cpp CALLS THESE UNCONDITIONALLY.
 *
 * Only one of the eight boards defines BLE_SERVER_ENABLED, so a header that
 * declared nothing here would build on that one and fail to compile on the
 * other seven — which is exactly what happened the first time, and exactly the
 * shape body_task.h already guards against a few lines further down its own
 * file. Building one variant proves nothing about the other seven. */
typedef bool (*RuleTaskSend)(uint32_t can_id, const uint8_t* data, uint8_t dlc);

static inline void rule_task_init(FSDState*, portMUX_TYPE*, RuleTaskSend) {}
static inline void rule_task_set_armed(bool) {}
static inline bool rule_task_armed(void) { return false; }
static inline void rule_task_observe(uint32_t, const uint8_t*, uint8_t, uint32_t) {}
static inline void rule_task_tick(uint32_t) {}
static inline uint32_t rule_task_sent(void) { return 0; }
static inline uint32_t rule_task_refused(void) { return 0; }
static inline const char* rule_task_last_refusal(void) { return "none"; }
static inline void rule_task_print(void) {}

#endif // BLE_SERVER_ENABLED
