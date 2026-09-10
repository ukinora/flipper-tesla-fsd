/*
 * body_task.cpp — see body_task.h. Receives, measures, logs. Sends nothing.
 *
 * Threading: every entry point runs on the Arduino loop task, exactly as
 * camera_task.cpp does — process_frame() has one caller inside loop(), and the
 * tick is called from loop(). Nothing here is guarded, and nothing here may be
 * called from a NimBLE callback.
 */

#include "body_task.h"

#ifdef BLE_SERVER_ENABLED

#include "../../fsd_logic/fsd_autonomy.h"
#include "../../fsd_logic/fsd_body.h"
#include "../../fsd_logic/fsd_body_t1.h"
#include "../../fsd_logic/fsd_body_t2.h"
#include "camera_task.h" // the one named drivetrain-speed source

#include <Arduino.h>
#include <string.h>

/* The detectors are cheap, but the door aggregate only changes when a latch
 * does. 10 Hz is far more than enough and keeps loop() honest. */
#define BODY_TICK_MS 100u

/* How often the measurement line goes out, and only when something moved. A
 * bring-up session watches this; a quiet car should not fill the log. */
#define BODY_LOG_MS 10000u

static FsdT1 g_t1;
static FsdT2 g_t2;
static FSDState* g_state = nullptr;
static portMUX_TYPE* g_mux = nullptr;

static uint32_t g_last_tick_ms = 0;
static uint32_t g_last_log_ms = 0;
static bool g_bus_tx_open = false; // fail-closed until main.cpp says otherwise

/* Set once a drive has actually happened — gear in D/R with the belt latched.
 * Never cleared, and it does not need to be: the accessory feed is switched, so
 * this dies with the power every time the car sleeps. It therefore means "a
 * drive has happened since the car woke", which is exactly the question. */

/* Only counted. Nothing acts on a T1 action because nothing can. */
static uint16_t g_t1_actions = 0;

/* Watched so the log only speaks when something changed. */
static uint16_t g_logged_actions = 0;
static uint16_t g_logged_gestures = 0;

void body_task_init(FSDState* state, portMUX_TYPE* mux) {
    g_state = state;
    g_mux = mux;
    fsd_t1_init(&g_t1);
    fsd_t2_init(&g_t2);
}

void body_task_set_bus_tx_open(bool open) {
    g_bus_tx_open = open;
}

bool body_task_tx_refused(uint32_t can_id) {
    return fsd_body_tx_id_refused(can_id);
}

bool body_task_observe(uint32_t id, const uint8_t* data, uint8_t dlc, uint32_t now_ms) {
    if(!data) return false;
    switch(id) {
    case FSD_T1_CAN_ID_LEFT:
        fsd_t1_observe_door(&g_t1, FSD_BODY_SIDE_LEFT, id, data, dlc, now_ms);
        return true;
    case FSD_T1_CAN_ID_RIGHT:
        fsd_t1_observe_door(&g_t1, FSD_BODY_SIDE_RIGHT, id, data, dlc, now_ms);
        return true;
    case FSD_T2_CAN_ID:
        /* Shared with the scroll path, which reads mux 1. The detector reads
         * mux 0 and ignores everything else, so both can have it. */
        fsd_t2_observe_switch(&g_t2, id, data, dlc, now_ms);
        return true;
    default:
        return false;
    }
}

/* 🔴🔴 body_task_permission_inputs() IS GONE (owner's instruction,
 * 2026-09-10): "차의 모든 안전게이트관련사항을 삭제해라."
 *
 * It snapshotted everything the permission predicate needed -- mode, bus, OTA,
 * RX freshness, gear, speed, belt, occupancy, per-action enable. There is no
 * predicate any more, so there is nothing to snapshot.
 *
 * 🟢 THE LESSON IT CARRIED IS STILL TRUE AND IS WORTH KEEPING HERE: this
 * function used to copy the FSDState fields by hand and MISSED THE BELT -- the
 * gate learned to read it, 4,437 host tests went green, six mutations bit,
 * eight boards built, and the bench still refused, because the two new fields
 * sat at the memset's false with no producer. A field with no producer is this
 * repository's oldest failure shape. */

void body_task_tick(uint32_t now_ms) {
    if(!g_state || !g_mux) return;
    if((uint32_t)(now_ms - g_last_tick_ms) < BODY_TICK_MS) return;
    g_last_tick_ms = now_ms;

    /* The action is COUNTED, not performed. */
    if(fsd_t1_tick(&g_t1, now_ms) != FSD_T1_ACT_NONE) g_t1_actions++;

    if((uint32_t)(now_ms - g_last_log_ms) < BODY_LOG_MS) return;

    const uint16_t gestures = fsd_t2_gesture_count(&g_t2);
    const bool moved = (g_t1_actions != g_logged_actions) || (gestures != g_logged_gestures);
    /* Speak the first time regardless — a session that starts silent should
     * still show the latch values and the frame period once they are known. */
    const bool first = (g_last_log_ms == 0);
    if(!moved && !first) return;

    g_last_log_ms = now_ms;
    g_logged_actions = g_t1_actions;
    g_logged_gestures = gestures;

    Serial.printf(
        /* 🔴 win / b5 / last ARE THE MEASUREMENT. These detectors exist to
         * measure, not to act — the whole point is to characterise T1's window
         * and T2's gesture offline against the one-shot capture. Their
         * accessors had no caller anywhere, tests included (red team,
         * 2026-08-19), so the numbers were computed and thrown away.
         *
         *   win  how many T1 windows opened
         *   b5   the raw 0x3C2 byte the T2 detector last saw — the byte a
         *        person compares against the capture by eye
         *   last when the last T2 gesture landed, so it can be found in a dump */
        "[BODY] t1:%u win:%u latch L:%X R:%X | t2:%u press:%ums gap:%ums "
        "rej:%u b5:%02X last:%ums | mux0:%ums drv:%u/%u\n",
        (unsigned)g_t1_actions,
        (unsigned)fsd_t1_window_count(&g_t1),
        (unsigned)fsd_t1_latch_raw(&g_t1, FSD_BODY_SIDE_LEFT),
        (unsigned)fsd_t1_latch_raw(&g_t1, FSD_BODY_SIDE_RIGHT), (unsigned)gestures,
        (unsigned)fsd_t2_last_press_ms(&g_t2), (unsigned)fsd_t2_last_gap_ms(&g_t2),
        (unsigned)fsd_t2_last_reject(&g_t2),
        (unsigned)fsd_t2_last_byte5(&g_t2), (unsigned)fsd_t2_last_gesture_ms(&g_t2),
        (unsigned)fsd_t2_mux0_min_gap_ms(&g_t2),
        /* 🔴 NOT A GATE ANY MORE -- a measurement. The occupancy gate that read
         * this was removed on 2026-09-08, which left fsd_t2_driver_*() with no
         * production caller at all: this repo's oldest failure mode. It is
         * printed instead of deleted because the question it answers is still
         * open and now reachable from the app -- driverPresent is offered as a
         * RULE TRIGGER (signal 17), and on this car the bit reads 100/100 while
         * parked with the gear being worked and 0/100 across an entire drive.
         * Anyone about to build a rule on it should be able to watch it. */
        (unsigned)fsd_t2_driver_seen(&g_t2),
        (unsigned)fsd_t2_driver_present(&g_t2));
}

uint16_t body_task_t1_actions(void) { return g_t1_actions; }
uint16_t body_task_t2_gestures(void) { return fsd_t2_gesture_count(&g_t2); }
uint16_t body_task_t2_last_press_ms(void) { return fsd_t2_last_press_ms(&g_t2); }
uint16_t body_task_t2_last_gap_ms(void) { return fsd_t2_last_gap_ms(&g_t2); }
uint8_t body_task_t2_last_reject(void) { return fsd_t2_last_reject(&g_t2); }
uint32_t body_task_mux0_period_ms(void) { return fsd_t2_mux0_min_gap_ms(&g_t2); }

uint8_t body_task_latch_raw(uint8_t side) {
    if(side >= FSD_BODY_SIDE_COUNT) return 0xFFu;
    return fsd_t1_latch_raw(&g_t1, (FsdBodySide)side);
}

#endif // BLE_SERVER_ENABLED
