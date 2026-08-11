/*
 * test_body.c — host tests for the body-control authority axis and the two
 * detectors (fsd_body.c, fsd_body_t1.c, fsd_body_t2.c).
 *
 * The thing worth testing here is not that the features work. It is that they
 * REFUSE, in every way they are supposed to, and that nothing anywhere can put
 * a body frame on the wire. So most of what follows asserts an absence.
 *
 * The assertions that matter most:
 *   - T2 can never be armed, by any input, in any mode.
 *   - the capability table is tightest when zero-filled.
 *   - an observer handed the wrong CAN ID learns nothing and stamps nothing.
 *   - fsd_body_tx_id_refused() is unconditional.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_autonomy.h"
#include "fsd_body.h"
#include "fsd_body_t1.h"
#include "fsd_body_t2.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (cond) {                                                             \
            g_pass++;                                                           \
        } else {                                                                \
            g_fail++;                                                           \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                       \
            printf(__VA_ARGS__);                                                \
            printf("\n");                                                       \
        }                                                                       \
    } while (0)

// Everything a T1 action needs, all satisfied. Tests then take one away.
static FsdBodyInputs good_inputs(uint32_t now_ms) {
    FsdBodyInputs in;
    memset(&in, 0, sizeof(in));
    in.op_mode = OpMode_Active;
    in.bus_tx_open = true;
    in.ota_in_progress = false;
    in.rx_stale = false;
    in.feature_enabled[FSD_BODY_T1_LIGHT] = true;
    in.feature_enabled[FSD_BODY_T2_DOOR] = true; // must not help; see below
    in.drive_session = true;
    in.driver_seen = true;
    in.driver_present = true;
    in.driver_ms = now_ms;
    in.gear_seen = true;
    in.gear = FSD_GEAR_P;
    in.gear_ms = now_ms;
    in.speed_seen = true;
    in.speed_kph = 0.0f;
    in.speed_ms = now_ms;
    return in;
}

// ── the axis ─────────────────────────────────────────────────────────────────

static void test_t2_can_never_be_armed(void) {
    printf("\n-- T2 is unreachable, by construction --\n");

    // Every input satisfied, every flag set, both permissive modes tried. The
    // capability row is the only thing refusing, and nothing at runtime can
    // reach it.
    const uint32_t now = 10000;
    FsdBodyInputs in = good_inputs(now);
    CHECK(fsd_body_allows(&in, FSD_BODY_T2_DOOR, now) == FSD_BODY_NOT_ARMABLE,
          "T2 refused with everything satisfied");

    in.op_mode = OpMode_Service;
    CHECK(fsd_body_allows(&in, FSD_BODY_T2_DOOR, now) == FSD_BODY_NOT_ARMABLE,
          "Service does not help");

    // NOT_ARMABLE is checked FIRST, so it is the answer even when other gates
    // would also refuse — the app must never be told "just start the car".
    memset(&in, 0, sizeof(in));
    CHECK(fsd_body_allows(&in, FSD_BODY_T2_DOOR, now) == FSD_BODY_NOT_ARMABLE,
          "and it is the FIRST refusal, not a later one");
}

static void test_zero_is_the_tightest_row(void) {
    printf("\n-- an all-zero capability row grants nothing --\n");

    // The struct is written permissive-when-true precisely so that a row
    // somebody forgets to fill in is a row that grants nothing. If a field is
    // ever flipped to restrictive-when-true this test fails, which is the point.
    FsdBodyCaps zero;
    memset(&zero, 0, sizeof(zero));
    CHECK(!zero.may_act_while_moving && !zero.may_act_out_of_park &&
              !zero.may_act_without_driver && !zero.may_act_without_drive_session &&
              !zero.armable_at_runtime,
          "every field in a zeroed row is restrictive");
}

static void test_axis_refuses_in_order(void) {
    printf("\n-- every gate, and the order they answer in --\n");

    const uint32_t now = 10000;
    FsdBodyInputs in = good_inputs(now);
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_OK, "baseline is OK");

    in = good_inputs(now); in.feature_enabled[FSD_BODY_T1_LIGHT] = false;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_NOT_ENABLED, "not enabled");

    // The allow-set is exactly the one fsd_can_transmit() admits: this axis can
    // only ever subtract from it.
    in = good_inputs(now); in.op_mode = OpMode_ListenOnly;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_NO_MODE, "listen-only");
    in.op_mode = OpMode_Autonomous;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_NO_MODE,
          "Autonomous grants nothing here — it is the camera path's mode only");
    in.op_mode = OpMode_Service;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_OK, "Service is allowed");

    in = good_inputs(now); in.bus_tx_open = false;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_BUS_SHUT,
          "hardware listen-only");

    in = good_inputs(now); in.ota_in_progress = true;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_OTA, "Tesla updating");

    in = good_inputs(now); in.rx_stale = true;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_RX_STALE, "bus quiet");

    // T1's row waives the driver, gear and speed gates but NOT the drive
    // session: a car that has sat untouched since yesterday does nothing.
    in = good_inputs(now); in.drive_session = false;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_NO_DRIVE_SESSION,
          "no drive has happened");

    in = good_inputs(now); in.driver_present = false; in.driver_seen = false;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_OK,
          "T1 does not need a driver in the seat — that is its whole point");
    in = good_inputs(now); in.gear = FSD_GEAR_D; in.speed_kph = 90.0f;
    CHECK(fsd_body_allows(&in, FSD_BODY_T1_LIGHT, now) == FSD_BODY_OK,
          "nor a stationary car");

    CHECK(fsd_body_allows(NULL, FSD_BODY_T1_LIGHT, now) == FSD_BODY_UNKNOWN_FEATURE,
          "NULL inputs");
    in = good_inputs(now);
    CHECK(fsd_body_allows(&in, FSD_BODY_FEATURE_COUNT, now) == FSD_BODY_UNKNOWN_FEATURE,
          "out-of-range feature");

    for (int v = 0; v <= FSD_BODY_MOVING; v++) {
        const char* s = fsd_body_verdict_str((FsdBodyVerdict)v);
        CHECK(s && s[0] && strcmp(s, "?") != 0, "verdict %d has a name", v);
    }
}

static void test_tx_ids_are_refused_unconditionally(void) {
    printf("\n-- no body frame may ever leave --\n");

    CHECK(fsd_body_tx_id_refused(0x3F5u), "0x3F5 lighting refused");
    CHECK(fsd_body_tx_id_refused(0x102u), "0x102 door status refused");
    CHECK(fsd_body_tx_id_refused(0x103u), "0x103 door status refused");
    // Not a blanket ban: the camera path's own frame must still pass.
    CHECK(!fsd_body_tx_id_refused(0x3C2u), "0x3C2 is not a body ID");
    CHECK(!fsd_body_tx_id_refused(0x3FDu), "0x3FD is not a body ID");
}

// ── T1 ───────────────────────────────────────────────────────────────────────

static void door(FsdT1* t, FsdBodySide side, uint8_t latch, uint32_t now_ms) {
    uint8_t d[8];
    memset(d, 0, sizeof(d));
    d[0] = (uint8_t)((latch & 0x0Fu) << 4); // rearLatchStatus 4|4@1+
    const uint32_t id = (side == FSD_BODY_SIDE_LEFT) ? FSD_T1_CAN_ID_LEFT : FSD_T1_CAN_ID_RIGHT;
    fsd_t1_observe_door(t, side, id, d, 8, now_ms);
}

/* Hold both doors at the given latch values for `span_ms`, feeding a frame
 * every 50 ms the way the car does, and ticking as we go.
 *
 * Feeding one frame per state change would be the unrealistic thing here: a
 * latch bounces through OPENING/CLOSING and can flick through AJAR, which is
 * exactly what FSD_T1_DEBOUNCE_MS exists for, so a new value only becomes
 * stable once it has been repeated. The bus repeats it; a test that does not
 * is testing a car that does not exist.
 *
 * Counts the actions rather than returning one, so "exactly once per edge" is
 * assertable. */
static uint32_t doors_for(FsdT1* t, uint8_t left, uint8_t right, uint32_t now,
                          uint32_t span_ms, bool drive_session, int* on_count,
                          int* off_count) {
    for (uint32_t e = 0; e <= span_ms; e += 50) {
        const uint32_t at = now + e;
        FsdBodyInputs in = good_inputs(at);
        in.drive_session = drive_session;
        door(t, FSD_BODY_SIDE_LEFT, left, at);
        door(t, FSD_BODY_SIDE_RIGHT, right, at);
        const FsdT1Action a = fsd_t1_tick(t, &in, at);
        if (a == FSD_T1_ACT_ON && on_count) (*on_count)++;
        if (a == FSD_T1_ACT_OFF && off_count) (*off_count)++;
    }
    return now + span_ms;
}

/* Settle both sides CLOSED and consume the initial adoption. */
static uint32_t t1_settle(FsdT1* t, uint32_t now) {
    int on = 0, off = 0;
    now = doors_for(t, FSD_LATCH_CLOSED, FSD_LATCH_CLOSED, now, 400, true, &on, &off);
    return now;
}

static void test_t1_edges(void) {
    printf("\n-- T1: one action per edge, and none on adoption --\n");

    FsdT1 t;
    fsd_t1_init(&t);
    uint32_t now = 10000;
    int on = 0, off = 0;

    // Booting next to a closed car must not fire anything.
    now = doors_for(&t, FSD_LATCH_CLOSED, FSD_LATCH_CLOSED, now, 400, true, &on, &off);
    CHECK(on == 0 && off == 0, "adopting the first state is silent");

    // Open the left rear and hold it: exactly one ON, no matter how many frames.
    now += 1000;
    on = off = 0;
    now = doors_for(&t, FSD_LATCH_OPENED, FSD_LATCH_CLOSED, now, 1000, true, &on, &off);
    CHECK(on == 1, "door opened -> exactly one ON, got %d", on);
    CHECK(off == 0, "and no OFF");

    // Close it. Needs BOTH sides fresh and CLOSED.
    now += 1000;
    on = off = 0;
    now = doors_for(&t, FSD_LATCH_CLOSED, FSD_LATCH_CLOSED, now, 1000, true, &on, &off);
    CHECK(off == 1, "both shut -> exactly one OFF, got %d", off);

    // AJAR counts as open.
    now += 1000;
    on = off = 0;
    now = doors_for(&t, FSD_LATCH_CLOSED, FSD_LATCH_AJAR, now, 1000, true, &on, &off);
    CHECK(on == 1, "ajar is open, got %d ON", on);
}

static void test_t1_fails_closed(void) {
    printf("\n-- T1: what it does when it does not know --\n");

    FsdT1 t;
    fsd_t1_init(&t);
    uint32_t now = 10000;
    int on = 0, off = 0;
    now = t1_settle(&t, now);

    // Open the left rear.
    now += 1000;
    on = off = 0;
    now = doors_for(&t, FSD_LATCH_OPENED, FSD_LATCH_CLOSED, now, 1000, true, &on, &off);
    CHECK(on == 1, "opened");

    // Now close the left while the RIGHT side goes quiet. OFF must not fire:
    // a door we cannot see does not count as shut.
    now += 1000;
    on = off = 0;
    for (uint32_t e = 0; e <= 3000; e += 50) {
        const uint32_t at = now + e;
        FsdBodyInputs in = good_inputs(at);
        door(&t, FSD_BODY_SIDE_LEFT, FSD_LATCH_CLOSED, at); // right: nothing
        const FsdT1Action a = fsd_t1_tick(&t, &in, at);
        if (a == FSD_T1_ACT_OFF) off++;
    }
    CHECK(off == 0, "a door we cannot see does not count as shut, got %d OFF", off);

    // Unknown latch values are silence, not knowledge: they must not stamp.
    FsdT1 u;
    fsd_t1_init(&u);
    for (uint32_t e = 0; e <= 400; e += 50) {
        door(&u, FSD_BODY_SIDE_LEFT, FSD_LATCH_SNA, 1000 + e);
        door(&u, FSD_BODY_SIDE_LEFT, FSD_LATCH_FAULT, 1000 + e);
        door(&u, FSD_BODY_SIDE_RIGHT, FSD_LATCH_CLOSED, 1000 + e);
    }
    FsdBodyInputs in2 = good_inputs(1400);
    CHECK(fsd_t1_tick(&u, &in2, 1400) == FSD_T1_ACT_NONE, "unknown latches decide nothing");

    // Wrong CAN ID for the side: learned nothing, stamped nothing.
    FsdT1 w;
    fsd_t1_init(&w);
    uint8_t d[8];
    memset(d, 0, sizeof(d));
    d[0] = (uint8_t)(FSD_LATCH_OPENED << 4);
    fsd_t1_observe_door(&w, FSD_BODY_SIDE_LEFT, FSD_T1_CAN_ID_RIGHT, d, 8, 1000);
    CHECK(fsd_t1_latch_raw(&w, FSD_BODY_SIDE_LEFT) == 0xFFu, "mis-dispatch teaches nothing");
    fsd_t1_observe_door(&w, FSD_BODY_SIDE_LEFT, FSD_T1_CAN_ID_LEFT, d, 7, 1000);
    CHECK(fsd_t1_latch_raw(&w, FSD_BODY_SIDE_LEFT) == 0xFFu, "short frame teaches nothing");
}

static void test_t1_respects_the_axis(void) {
    printf("\n-- T1: the edge happens, the action does not --\n");

    FsdT1 t;
    fsd_t1_init(&t);
    uint32_t now = 10000;
    int on = 0, off = 0;
    now = t1_settle(&t, now);

    // Refused: no drive has happened yet.
    now += 1000;
    on = off = 0;
    now = doors_for(&t, FSD_LATCH_OPENED, FSD_LATCH_CLOSED, now, 1000, false, &on, &off);
    CHECK(on == 0, "refused, got %d ON", on);
    CHECK(fsd_t1_last_verdict(&t) == FSD_BODY_NO_DRIVE_SESSION, "and says why");

    // The edge was CONSUMED. Granting permission afterwards must not replay a
    // door event that happened while we were not allowed to act — the world has
    // moved on. The door is still open; only a NEW edge counts.
    now += 1000;
    on = off = 0;
    now = doors_for(&t, FSD_LATCH_OPENED, FSD_LATCH_CLOSED, now, 1000, true, &on, &off);
    CHECK(on == 0, "a refused edge is not replayed, got %d ON", on);

    // Closing is a new edge, and it acts.
    now += 1000;
    on = off = 0;
    now = doors_for(&t, FSD_LATCH_CLOSED, FSD_LATCH_CLOSED, now, 1000, true, &on, &off);
    CHECK(off == 1, "closing acts, got %d OFF", off);
}

static void test_t1_budget(void) {
    printf("\n-- T1: a door being played with is bounded --\n");

    FsdT1 t;
    fsd_t1_init(&t);
    uint32_t now = 10000;
    now = t1_settle(&t, now);

    // 200 open/close cycles, each held long enough to debounce and spaced past
    // FSD_T1_MIN_GAP_MS. 200 * 1100 ms is ~3.7 minutes, well inside the window.
    int on = 0, off = 0;
    for (int i = 0; i < 200; i++) {
        const uint8_t l = (i & 1) ? FSD_LATCH_CLOSED : FSD_LATCH_OPENED;
        now = doors_for(&t, l, FSD_LATCH_CLOSED, now, 1050, true, &on, &off);
        now += 50;
    }
    const int acted = on + off;
    CHECK(acted <= (int)FSD_T1_MAX_PER_WINDOW, "bounded at %u per window, got %d",
          FSD_T1_MAX_PER_WINDOW, acted);
    CHECK(acted > 0, "but not zero — it does work");
}

// ── T2 ───────────────────────────────────────────────────────────────────────

static void t2_frame(FsdT2* t, bool up, bool auto_up, uint32_t now_ms) {
    uint8_t d[8];
    memset(d, 0, sizeof(d));
    d[0] = 0x00u;                            // mux 0
    d[5] = (uint8_t)((up ? 1u : 0u) | (auto_up ? 2u : 0u));
    fsd_t2_observe_switch(t, FSD_T2_CAN_ID, d, 8, now_ms);
}

/* One press of `press_ms`, released at now+press_ms. Returns the release time
 * and reports whether the release completed a gesture. */
static uint32_t t2_press(FsdT2* t, uint32_t at, uint16_t press_ms, bool* fired) {
    uint8_t d[8];
    memset(d, 0, sizeof(d));
    d[5] = 0x01u;
    if (fired) *fired = false;
    fsd_t2_observe_switch(t, FSD_T2_CAN_ID, d, 8, at);
    d[5] = 0x00u;
    const bool f = fsd_t2_observe_switch(t, FSD_T2_CAN_ID, d, 8, at + press_ms);
    if (fired) *fired = f;
    return at + press_ms;
}

static void test_t2_gesture(void) {
    printf("\n-- T2: the gesture, and what it refuses --\n");

    FsdT2 t;
    bool fired = false;

    // Two well-formed taps with a legal gap.
    fsd_t2_init(&t);
    uint32_t rel = t2_press(&t, 1000, 100, &fired);
    CHECK(!fired, "one press is not a gesture");
    t2_press(&t, rel + 300, 100, &fired);
    CHECK(fired, "two presses are");
    CHECK(fsd_t2_gesture_count(&t) == 1, "counted once");
    CHECK(fsd_t2_last_gap_ms(&t) == 300, "gap measured: %u", fsd_t2_last_gap_ms(&t));
    CHECK(fsd_t2_last_press_ms(&t) == 100, "press measured");

    // A press-and-hold is one press, not two — which a level test would miss.
    fsd_t2_init(&t);
    {
        uint8_t d[8];
        memset(d, 0, sizeof(d));
        d[5] = 0x01u;
        for (uint32_t k = 0; k < 10; k++)
            CHECK(!fsd_t2_observe_switch(&t, FSD_T2_CAN_ID, d, 8, 1000 + k * 50),
                  "holding is never a gesture");
    }

    // Too long a press is someone moving the window.
    fsd_t2_init(&t);
    rel = t2_press(&t, 1000, 100, &fired);
    t2_press(&t, rel + 300, (uint16_t)(FSD_T2_TIMING.press_max_ms + 50u), &fired);
    CHECK(!fired, "a long press is a window action");
    CHECK(fsd_t2_last_reject(&t) == FSD_T2_REJ_PRESS_LONG, "and says so");

    // Gap too long: two separate intentions.
    fsd_t2_init(&t);
    rel = t2_press(&t, 1000, 100, &fired);
    t2_press(&t, rel + FSD_T2_TIMING.gap_max_ms + 100u, 100, &fired);
    CHECK(!fired, "too far apart");
    CHECK(fsd_t2_last_reject(&t) == FSD_T2_REJ_GAP_LONG, "and says so");

    // Auto-up is a window action by definition.
    fsd_t2_init(&t);
    rel = t2_press(&t, 1000, 100, &fired);
    t2_frame(&t, true, true, rel + 300);
    t2_frame(&t, false, true, rel + 400);
    CHECK(fsd_t2_last_reject(&t) == FSD_T2_REJ_AUTOUP, "auto-up rejected");

    // Rejections still publish their measurements — that is what sizes the
    // window when the capture comes back.
    CHECK(fsd_t2_last_press_ms(&t) == 100, "a rejected candidate still reports its press");
}

static void test_t2_frame_discipline(void) {
    printf("\n-- T2: wrong frame, wrong mux, short frame --\n");

    FsdT2 t;
    uint8_t d[8];
    memset(d, 0, sizeof(d));
    d[5] = 0x01u;

    fsd_t2_init(&t);
    CHECK(!fsd_t2_observe_switch(&t, 0x102u, d, 8, 1000), "wrong CAN ID ignored");
    CHECK(!fsd_t2_observe_switch(&t, FSD_T2_CAN_ID, d, 7, 1000), "short frame ignored");

    // Mux 1: byte 5 means something else there. Reading it would conflate two
    // different buttons.
    d[0] = 0x01u;
    CHECK(!fsd_t2_observe_switch(&t, FSD_T2_CAN_ID, d, 8, 1000), "mux 1 ignored");
    d[0] = 0x00u;
    fsd_t2_observe_switch(&t, FSD_T2_CAN_ID, d, 8, 1000);
    CHECK(t.pressed, "mux 0 is read");

    // The frame period, which no DBC in this repo states.
    fsd_t2_init(&t);
    memset(d, 0, sizeof(d));
    fsd_t2_observe_switch(&t, FSD_T2_CAN_ID, d, 8, 1000);
    fsd_t2_observe_switch(&t, FSD_T2_CAN_ID, d, 8, 1100);
    fsd_t2_observe_switch(&t, FSD_T2_CAN_ID, d, 8, 1140);
    CHECK(fsd_t2_mux0_min_gap_ms(&t) == 40, "smallest mux-0 gap measured: %u",
          fsd_t2_mux0_min_gap_ms(&t));

    // driverPresent rides in the same frame.
    memset(d, 0, sizeof(d));
    d[0] = (uint8_t)(1u << 4);
    fsd_t2_observe_switch(&t, FSD_T2_CAN_ID, d, 8, 1200);
    CHECK(fsd_t2_driver_present(&t) && fsd_t2_driver_seen(&t), "driverPresent carried");

    CHECK(!FSD_T2_TIMING.verified, "the timing table is NOT verified, and says so");
}

int main(void) {
    printf("test_body\n");
    test_t2_can_never_be_armed();
    test_zero_is_the_tightest_row();
    test_axis_refuses_in_order();
    test_tx_ids_are_refused_unconditionally();
    test_t1_edges();
    test_t1_fails_closed();
    test_t1_respects_the_axis();
    test_t1_budget();
    test_t2_gesture();
    test_t2_frame_discipline();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
