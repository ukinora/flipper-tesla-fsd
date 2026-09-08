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
    in.action_enabled[FSD_ACT_MAP_LIGHT] = true;
    in.action_enabled[FSD_ACT_DOOR_OPEN] = true; // must not help; see below
    in.gear_seen = true;
    in.gear = FSD_GEAR_P;
    in.gear_ms = now_ms;
    in.speed_seen = true;
    in.speed_kph = 0.0f;
    in.speed_ms = now_ms;
    // Added with the action-indexed rewrite. Fresh and permissive, so a test
    // that wants to see one of these refuse has to take it away on purpose.
    in.belt_seen = true;
    in.belt_latched = true;
    in.belt_ms = now_ms;
    in.passenger_seen = true;
    in.passenger_present = false;
    in.passenger_ms = now_ms;
    return in;
}

// ── the axis ─────────────────────────────────────────────────────────────────

/* This test used to assert the opposite: "T2 is unreachable, by construction".
 * The row said it would open when the command frame was measured, the third
 * visit measured it (0x1F9 byte 1 = 0x03, 2026-09-05), and it opened.
 *
 * 🔴 So the test has to get STRONGER, not go away. What protected the door
 * before was one bool. What protects it now is five gates, and the job of this
 * test is to prove that taking away any single one of them refuses -- because
 * the failure mode to fear is no longer "someone flips the bool", it is
 * "someone widens a row while adding something unrelated". */
/* One-line breakers, so the table above reads as a list of gates rather than a
 * list of struct assignments. */
static void break_speed(FsdBodyInputs* in) { in->speed_kph = 5.0f; }
static void break_gear(FsdBodyInputs* in) { in->gear = FSD_GEAR_D; }
static void break_enable(FsdBodyInputs* in) { in->action_enabled[FSD_ACT_DOOR_OPEN] = false; }
static void break_mode(FsdBodyInputs* in) { in->op_mode = OpMode_ListenOnly; }
static void break_bus(FsdBodyInputs* in) { in->bus_tx_open = false; }

static void test_door_is_armed_but_every_gate_still_holds(void) {
    printf("\n-- the door row opened; each gate still refuses alone --\n");

    const uint32_t now = 10000;
    FsdBodyInputs in = good_inputs(now);

    CHECK(fsd_body_allows(&in, FSD_ACT_DOOR_OPEN, now) == FSD_BODY_OK,
          "with every input satisfied, the door is allowed");

    /* One at a time, from the full set. Each must refuse, and refuse with the
     * reason that names the missing thing -- a door that refuses for the wrong
     * reason sends someone to fix the wrong input. */
    struct {
        const char* name;
        FsdBodyVerdict want;
        void (*break_it)(FsdBodyInputs*);
    } cases[] = {
        {"moving", FSD_BODY_MOVING, break_speed},
        {"not in park", FSD_BODY_NOT_PARK, break_gear},
        {"not enabled", FSD_BODY_NOT_ENABLED, break_enable},
        {"listen-only", FSD_BODY_NO_MODE, break_mode},
        {"bus shut", FSD_BODY_BUS_SHUT, break_bus},
    };
    for(unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        FsdBodyInputs bad = good_inputs(now);
        cases[i].break_it(&bad);
        FsdBodyVerdict got = fsd_body_allows(&bad, FSD_ACT_DOOR_OPEN, now);
        CHECK(got == cases[i].want, "%s -> %s, got %s", cases[i].name,
              fsd_body_verdict_str(cases[i].want), fsd_body_verdict_str(got));
    }

    /* 🔴 And the rate limit is real. A stuck rule asking twice in a second
     * is the shape this bound exists for. */
    FsdBodyInputs again = good_inputs(now);
    again.last_act_ms[FSD_ACT_DOOR_OPEN] = now - 2999u;
    CHECK(fsd_body_allows(&again, FSD_ACT_DOOR_OPEN, now) == FSD_BODY_TOO_SOON,
          "2999 ms after the last one -> too soon");
    again.last_act_ms[FSD_ACT_DOOR_OPEN] = now - 3000u;
    CHECK(fsd_body_allows(&again, FSD_ACT_DOOR_OPEN, now) == FSD_BODY_OK,
          "3000 ms after -> allowed");
}

static void test_zero_is_the_tightest_row(void) {
    printf("\n-- an all-zero capability row grants nothing --\n");

    // The struct is written permissive-when-true precisely so that a row
    // somebody forgets to fill in is a row that grants nothing. If a field is
    // ever flipped to restrictive-when-true this test fails, which is the point.
    FsdBodyCaps zero;
    memset(&zero, 0, sizeof(zero));
    CHECK(!zero.may_act_while_moving && !zero.may_act_out_of_park &&
              !zero.armable_at_runtime,
          "every field in a zeroed row is restrictive");
}

static void test_axis_refuses_in_order(void) {
    printf("\n-- every gate, and the order they answer in --\n");

    const uint32_t now = 10000;
    FsdBodyInputs in = good_inputs(now);
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_OK, "baseline is OK");

    in = good_inputs(now); in.action_enabled[FSD_ACT_MAP_LIGHT] = false;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_NOT_ENABLED, "not enabled");

    // The allow-set is exactly the one fsd_can_transmit() admits: this axis can
    // only ever subtract from it.
    in = good_inputs(now); in.op_mode = OpMode_ListenOnly;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_NO_MODE, "listen-only");
    in.op_mode = OpMode_Autonomous;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_NO_MODE,
          "Autonomous grants nothing here — it is the camera path's mode only");
    in.op_mode = OpMode_Service;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_OK, "Service is allowed");

    in = good_inputs(now); in.bus_tx_open = false;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_BUS_SHUT,
          "hardware listen-only");

    in = good_inputs(now); in.ota_in_progress = true;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_OTA, "Tesla updating");

    in = good_inputs(now); in.rx_stale = true;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_RX_STALE, "bus quiet");

    /* 🔴 THE DRIVE-SESSION GATE IS GONE (owner's instruction, 2026-09-08).
     *
     * It asked "has a human driven this car since the module powered on",
     * proved by a P->D/R transition with the belt latched. It was the single
     * biggest field trap in this project: the module comes up, the owner
     * presses the switch, nothing happens, and the reason is a sequence
     * nobody performs on purpose. Two visits lost time to it.
     *
     * What it uniquely blocked is narrow. For every other enabled action the
     * driver gate below already refuses an empty car; this one only added
     * "and they also drove". The map light is the exception -- it waives the
     * driver -- so for that one action the remaining proof that a person is
     * involved is that a RULE FIRED, and rules fire on physical switches.
     *
     * This test used to assert the refusal. It asserts the opposite now, so
     * that reintroducing the gate turns it red rather than quietly making the
     * car unresponsive again. */
    in = good_inputs(now);
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_OK,
          "a car that has not been driven since power-on still acts");

    /* T1 used to be the ONLY row that waived the driver. The question is not
     * asked of any row now (owner's instruction, 2026-09-08), so what this
     * asserts is no longer a waiver -- it is the absence of the gate. */
    in = good_inputs(now);
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_OK,
          "an empty car still acts — the occupancy gate is gone");
    in = good_inputs(now); in.gear = FSD_GEAR_D; in.speed_kph = 90.0f;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_OK,
          "nor a stationary car");

    CHECK(fsd_body_allows(NULL, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_UNKNOWN_ACTION,
          "NULL inputs");
    in = good_inputs(now);
    CHECK(fsd_body_allows(&in, FSD_ACT_COUNT, now) == FSD_BODY_UNKNOWN_ACTION,
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
                          uint32_t span_ms, bool enabled, int* on_count,
                          int* off_count) {
    for (uint32_t e = 0; e <= span_ms; e += 50) {
        const uint32_t at = now + e;
        FsdBodyInputs in = good_inputs(at);
        /* 🔴 This used to spoil the DRIVE SESSION to make the axis refuse.
         * That gate is gone (owner's instruction, 2026-09-08), so the refusal
         * these tests need comes from the per-action enable instead. The
         * subject is unchanged: a refused edge must not be replayed later. */
        in.action_enabled[FSD_ACT_MAP_LIGHT] = enabled;
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

    /* Refused: T1 is not enabled. This used to be refused for a DIFFERENT
     * reason -- no drive had happened yet -- and that gate is gone (owner's
     * instruction, 2026-09-08). The edge-consumption behaviour below is what
     * this test is really about, so it keeps a refusal to consume the edge
     * against; only the reason changed. */
    now += 1000;
    on = off = 0;
    now = doors_for(&t, FSD_LATCH_OPENED, FSD_LATCH_CLOSED, now, 1000, false, &on, &off);
    CHECK(on == 0, "refused, got %d ON", on);
    CHECK(fsd_t1_last_verdict(&t) == FSD_BODY_NOT_ENABLED, "and says why");

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

/* ── the action-indexed rewrite (2026-09-01) ──────────────────────────────
 *
 * The rewrite turned "features" into "actions" and added five capability
 * fields. The single most important thing to assert about it is that it
 * OPENED NOTHING: exactly one row was armable before and exactly one is
 * armable after.
 */

// Every input maximally permissive, every action enabled. Only the four rows
// whose command frame has been measured may get past arming — the other four
// must refuse on the row, not on the inputs.
static void test_rewrite_opened_nothing(void) {
    const uint32_t now = 100000;
    FsdBodyInputs in = good_inputs(now);
    for (int a = 0; a < FSD_ACT_COUNT; a++) in.action_enabled[a] = true;

    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_OK,
          "MAP_LIGHT carries over from T1");
    CHECK(fsd_body_allows(&in, FSD_ACT_DOOR_OPEN, now) == FSD_BODY_OK,
          "DOOR_OPEN joined it once its command frame was measured (2026-09-05)");
    CHECK(fsd_body_allows(&in, FSD_ACT_HAZARDS, now) == FSD_BODY_OK,
          "HAZARDS joined the same day, with the motion gates open on purpose");
    CHECK(fsd_body_allows(&in, FSD_ACT_TURN_SIGNAL, now) == FSD_BODY_OK,
          "TURN_SIGNAL joined 2026-09-06, on a measured frame plus owner consent");
    CHECK(fsd_body_allows(&in, FSD_ACT_MIRROR, now) == FSD_BODY_OK,
          "MIRROR joined 2026-09-07, on the 5th visit's two injected frames");
    CHECK(fsd_body_allows(&in, FSD_ACT_LIGHT_HORN, now) == FSD_BODY_OK,
          "LIGHT_HORN joined the same day, on the same visit's press/release");

    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        if (a == FSD_ACT_MAP_LIGHT || a == FSD_ACT_DOOR_OPEN ||
            a == FSD_ACT_HAZARDS || a == FSD_ACT_TURN_SIGNAL ||
            a == FSD_ACT_MIRROR || a == FSD_ACT_LIGHT_HORN) continue;
        CHECK(fsd_body_allows(&in, (FsdBodyAction)a, now) == FSD_BODY_NOT_ARMABLE,
              "%s must refuse on its row even with every input satisfied",
              fsd_body_action_str((FsdBodyAction)a));
    }

    // The count itself, so adding a row without deciding its arming is a
    // failing test rather than a silent grant. It moved 1 -> 2 -> 3 on
    // 2026-09-05, 3 -> 4 on 2026-09-06 and 4 -> 5 on 2026-09-07; each move cost
    // red tests, which is the price it should cost.
    int armable = 0;
    for (int a = 0; a < FSD_ACT_COUNT; a++)
        if (fsd_body_caps((FsdBodyAction)a)->armable_at_runtime) armable++;
    CHECK(armable == 6, "exactly six armable rows, found %d", armable);
}

/* 🔴 THE TURN SIGNAL IS THE SECOND ROW TO OPEN THE MOTION GATES, and unlike
 * the hazards it has a DIRECTION, so its row deserves to be pinned rather than
 * counted. Every one of these came from an owner decision on 2026-09-06 or
 * from a measurement; none is a default. */
static void test_turn_signal_row_is_open_where_it_has_to_be(void) {
    printf("\n-- 깜빡이 행: 주행 중 허용, 나머지는 그대로 --\n");

    const FsdBodyCaps *c = fsd_body_caps(FSD_ACT_TURN_SIGNAL);
    CHECK(c != NULL, "the row exists");
    if (!c) return;

    // Owner decision 2: an indicator that may only act in park cannot indicate.
    CHECK(c->may_act_while_moving, "may act while moving (owner, 2026-09-06)");
    CHECK(c->may_act_out_of_park, "may act out of park (same decision)");

    // And nothing else relaxed with it.
    CHECK(!c->requires_park && !c->requires_belt && !c->requires_passenger_empty,
          "no gate borrowed from another row");

    // 50 ms is the car's own period for 0x249 and the rate TSL re-sends at.
    // Written as a literal so that changing the constant turns this red rather
    // than moving the assertion with it.
    CHECK(c->min_interval_ms == 50u, "50 ms interval, got %u", c->min_interval_ms);
    CHECK(c->max_hold_ms == 1000u, "1000 ms hold ceiling, got %u", c->max_hold_ms);

    /* 🔴 The loosest interval in the table, so say so out loud: nothing else
     * may fire this often, and the thing that keeps it from being a burst
     * generator is max_hold_ms. If some other row is ever set below this, the
     * reasoning in fsd_body.c stops being true. */
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const FsdBodyCaps *o = fsd_body_caps((FsdBodyAction)a);
        if (!o || a == FSD_ACT_TURN_SIGNAL || o->min_interval_ms == 0u) continue;
        CHECK(o->min_interval_ms >= c->min_interval_ms,
              "%s fires faster than the turn signal (%u < %u)",
              fsd_body_action_str((FsdBodyAction)a), o->min_interval_ms,
              c->min_interval_ms);
    }

    // A moving car with no driver still refuses — the direction of the gate
    // that stayed shut, checked rather than assumed from the bool above.
    const uint32_t now = 100000;
    FsdBodyInputs in = good_inputs(now);
    in.action_enabled[FSD_ACT_TURN_SIGNAL] = true;
    in.speed_kph = 88.0f;
    CHECK(fsd_body_allows(&in, FSD_ACT_TURN_SIGNAL, now) == FSD_BODY_OK,
          "88 km/h is fine — that is the point of the row");
    /* 🔴 THE OCCUPANCY GATE IS GONE (owner's instruction, 2026-09-08).
     *
     * It asked "is this car being used by someone", from driverPresent OR the
     * belt, and three refusals lived here: NO_DRIVER (0x3C2 mux 0 never
     * arrived), DRIVER_STALE (it went quiet), NO_DRIVER_PRESENT (heard, and
     * nobody is there).
     *
     * 🔴 WHAT THAT MEANS, WRITTEN DOWN SO IT IS NOT DISCOVERED LATER: nothing
     * in this axis now ties a body write to a person being in the car. The
     * remaining link is that A RULE HAS TO FIRE. Today's rules fire on
     * physical switches, which a person has to press -- but a rule may also be
     * built on a STATE, and a state can change on a parked car with nobody in
     * it.
     *
     * These assertions are the opposite of the ones they replaced, so putting
     * the gate back turns them red instead of quietly making the car
     * unresponsive again. */
    in.belt_latched = false;
    in.belt_seen = false;
    CHECK(fsd_body_allows(&in, FSD_ACT_TURN_SIGNAL, now) == FSD_BODY_OK,
          "no belt, no signal, nobody there -- and it still acts");

    in = good_inputs(now);
    in.action_enabled[FSD_ACT_TURN_SIGNAL] = true;
    in.belt_seen = true;
    in.belt_latched = true;
    in.belt_ms = now - FSD_BODY_FRESH_MS;
    CHECK(fsd_body_allows(&in, FSD_ACT_TURN_SIGNAL, now) == FSD_BODY_OK,
          "a stale occupancy signal is not a refusal either");
}

static void test_caps_table_is_well_formed(void) {
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const FsdBodyCaps *c = fsd_body_caps((FsdBodyAction)a);
        CHECK(c != NULL, "row %d exists", a);
        CHECK(c && c->action == (FsdBodyAction)a, "row %d knows its own index", a);
        const char *n = fsd_body_action_str((FsdBodyAction)a);
        CHECK(n[0] != '?', "action %d has a name", a);
    }
    CHECK(fsd_body_caps(FSD_ACT_COUNT) == NULL, "out of range is NULL, not row 0");

    // Every verdict is nameable. A verdict that prints "?" is a refusal the
    // owner cannot act on, which is the whole reason this enum exists.
    for (int v = 0; v <= FSD_BODY_PASSENGER_PRESENT; v++)
        CHECK(fsd_body_verdict_str((FsdBodyVerdict)v)[0] != '?',
              "verdict %d has a name", v);
}

static void test_rate_limit(void) {
    const uint32_t now = 100000;
    const FsdBodyCaps *c = fsd_body_caps(FSD_ACT_MAP_LIGHT);
    const uint32_t iv = c->min_interval_ms;
    CHECK(iv > 0, "MAP_LIGHT has an interval at all");

    FsdBodyInputs in = good_inputs(now);
    in.last_act_ms[FSD_ACT_MAP_LIGHT] = now;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_TOO_SOON,
          "firing twice in the same millisecond is refused");

    in.last_act_ms[FSD_ACT_MAP_LIGHT] = now - (iv - 1);
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_TOO_SOON,
          "one millisecond short still refuses");

    in.last_act_ms[FSD_ACT_MAP_LIGHT] = now - iv;
    CHECK(fsd_body_allows(&in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_OK,
          "exactly the interval is enough");

    /* 🔴 0 MEANS NEVER, AND THE SUBTRACTION DOES NOT SAY THAT NEAR BOOT.
     *
     * The field's own comment promises "0 means never fired, and the unsigned
     * wrap treats that as long ago -- correct, because it is". That is true
     * for every millis() except the first few thousand: at now = 300 the
     * arithmetic gives 300, which is LESS than the map light's 500, so the
     * very first command of a run would be refused for as long as the
     * interval lasts.
     *
     * Nobody would have noticed while the limiter was decorative. Turning it
     * on made the promise load-bearing, so the code has to keep it. */
    FsdBodyInputs boot = good_inputs(300u);
    boot.last_act_ms[FSD_ACT_MAP_LIGHT] = 0u;
    CHECK(fsd_body_allows(&boot, FSD_ACT_MAP_LIGHT, 300u) == FSD_BODY_OK,
          "300 ms after boot, an action that never fired is allowed");
    CHECK(iv > 300u, "...and this only means anything while iv (%u) exceeds it",
          (unsigned)iv);

    /* ...but 0 is not a licence: an action that DID fire at millis() 0 is
     * indistinguishable from one that never did, and the safe reading of an
     * ambiguity here is the permissive one only because the alternative
     * refuses every first command. One millisecond of licence, once a boot. */
    FsdBodyInputs later = good_inputs(300u + iv);
    later.last_act_ms[FSD_ACT_MAP_LIGHT] = 300u;
    CHECK(fsd_body_allows(&later, FSD_ACT_MAP_LIGHT, 300u + 1u) == FSD_BODY_TOO_SOON,
          "a real stamp still counts, even a tiny one");

    // A row that never set min_interval_ms may never fire. This is the
    // permissive-when-non-zero rule applied to a number, and it is why a
    // zero-filled row is still the tightest row.
    FsdBodyCaps z;
    memset(&z, 0, sizeof(z));
    z.action = FSD_ACT_MAP_LIGHT;
    in = good_inputs(now);
    CHECK(fsd_body_caps_verdict(&z, &in, FSD_ACT_MAP_LIGHT, now) == FSD_BODY_TOO_SOON,
          "min_interval_ms == 0 means never, not always");
}

// The gear row is the one the owner overruled me on, so its shape is asserted
// rather than reviewed. Nothing here can fire it — armable is false — but the
// row must already be right when that bool flips.
static void test_gear_row_shape(void) {
    const FsdBodyCaps *c = fsd_body_caps(FSD_ACT_GEAR_D);
    CHECK(!c->armable_at_runtime,
          "gear stays unarmable until the no-brake refusal is measured");
    CHECK(c->requires_park, "P is the only gear we transition from");
    CHECK(c->requires_belt, "a gate that trusts its trigger is not a gate");
    CHECK(!c->may_act_while_moving, "never while moving");
    CHECK(c->min_interval_ms >= 1000u, "at most once a second");
    CHECK(c->max_hold_ms == 0u, "single shot — one frame moved the gear in the car");

    // There is exactly one gear action in the enum. R, N and P are not
    // expressible, which is a stronger statement than any runtime check.
    CHECK(fsd_body_action_str(FSD_ACT_GEAR_D)[0] != '?', "gear-D is named");
}

// Driving the gear row's gates through the caps helper, since the row itself
// can never reach them yet.
static void test_gear_gates_when_armed(void) {
    const uint32_t now = 100000;
    FsdBodyCaps c = *fsd_body_caps(FSD_ACT_GEAR_D);
    FsdBodyInputs in = good_inputs(now);

    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_OK,
          "baseline (P, belted, seated, stopped) would pass");

    in = good_inputs(now); in.gear = FSD_GEAR_D;
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_NOT_PARK,
          "already in D: no second request");

    in = good_inputs(now); in.belt_latched = false;
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_NO_BELT,
          "belt unlatched refuses");

    in = good_inputs(now); in.belt_seen = false;
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_NO_BELT,
          "never having heard the belt refuses — silence is not a latched belt");

    in = good_inputs(now); in.belt_ms = now - FSD_BODY_FRESH_MS;
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_BELT_STALE,
          "a stale belt refuses");

    in = good_inputs(now); in.speed_kph = 1.0f;
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_MOVING,
          "moving refuses");

    /* 🔴 An "empty seat refuses" case stood here. The occupancy gate it asked
     * about is gone (owner's instruction, 2026-09-08); requires_belt, asserted
     * three cases above, is the only thing left in the axis that asks about a
     * person at all -- and this is the only row that sets it. */
}

// The passenger seat is the only row with an occupancy gate, and it is the one
// whose failure puts a motor against a person. It fails closed on silence.
static void test_passenger_seat_fails_closed(void) {
    const uint32_t now = 100000;
    FsdBodyCaps c = *fsd_body_caps(FSD_ACT_SEAT_PASSENGER);
    CHECK(c.requires_passenger_empty, "the passenger row asks");
    CHECK(!fsd_body_caps(FSD_ACT_SEAT_DRIVER)->requires_passenger_empty,
          "the driver row does not — that person is the one asking");

    FsdBodyInputs in = good_inputs(now);
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_SEAT_PASSENGER, now) == FSD_BODY_OK,
          "empty and fresh passes");

    in = good_inputs(now); in.passenger_present = true;
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_SEAT_PASSENGER, now)
              == FSD_BODY_PASSENGER_PRESENT,
          "someone in the seat refuses");

    in = good_inputs(now); in.passenger_seen = false;
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_SEAT_PASSENGER, now)
              == FSD_BODY_NO_PASSENGER_SIGNAL,
          "never heard: refuses, because silence is not an empty seat");

    in = good_inputs(now); in.passenger_ms = now - FSD_BODY_FRESH_MS;
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_SEAT_PASSENGER, now)
              == FSD_BODY_NO_PASSENGER_SIGNAL,
          "stale: refuses");

    // The driver's seat is unaffected by an occupied passenger seat.
    FsdBodyCaps d = *fsd_body_caps(FSD_ACT_SEAT_DRIVER);
    in = good_inputs(now); in.passenger_present = true;
    CHECK(fsd_body_caps_verdict(&d, &in, FSD_ACT_SEAT_DRIVER, now) == FSD_BODY_OK,
          "a seated passenger does not block the driver's own seat");
}

// The owner chose "seats may move while driving". That is a decision, not a
// bug, and the row must say so out loud — with the hold bound that makes it
// survivable.
static void test_owner_decisions_are_in_the_table(void) {
    CHECK(fsd_body_caps(FSD_ACT_SEAT_DRIVER)->may_act_while_moving,
          "owner decision: seats move under way");
    CHECK(fsd_body_caps(FSD_ACT_SEAT_DRIVER)->max_hold_ms > 0 &&
              fsd_body_caps(FSD_ACT_SEAT_DRIVER)->max_hold_ms <= 1000u,
          "and a stuck rule cannot drive the motor to the end of its rail");
    CHECK(fsd_body_caps(FSD_ACT_CAMERA)->may_act_while_moving,
          "owner decision: camera on AND off, including under way");
    /* 🔴 This line used to read "the door row is still all-zero: its command
     * frame is unknown". The frame is known now, so what has to be asserted is
     * no longer that the row is shut -- it is that opening it relaxed NOTHING
     * else. Every restriction the all-zero row gave for free is now written
     * out, and stated here so removing one is a red test. */
    const FsdBodyCaps* d = fsd_body_caps(FSD_ACT_DOOR_OPEN);
    CHECK(d->armable_at_runtime, "the door row opened: its frame was measured");
    CHECK(!d->may_act_while_moving && !d->may_act_out_of_park,
          "and every gate the all-zero row gave for free is still shut");
    CHECK(d->min_interval_ms >= 3000u,
          "a door is rate-limited an order above a light, got %u",
          (unsigned)d->min_interval_ms);

    /* 🔴 Hazards are the row where "every restriction on" would be the
     * UNSAFE choice. TSL's own rule flashes them for reverse -- moving, out of
     * park -- and a hazard light that may only act in P cannot do what hazard
     * lights are for. Stated here so a later tidy-up that "hardens" the table
     * by closing every gate has to argue with a red test. */
    const FsdBodyCaps* h = fsd_body_caps(FSD_ACT_HAZARDS);
    CHECK(h->armable_at_runtime, "hazards armable: frame measured 2026-09-05");
    CHECK(h->may_act_while_moving, "owner-visible decision: hazards while moving");
    CHECK(h->may_act_out_of_park, "and out of park -- reverse is the whole case");
    CHECK(h->max_hold_ms > 0u,
          "the failure to survive is a stuck rule HOLDING them, so bound it");

    /* 🔴 THE MIRROR IS THE ROW THAT LOOKS LIKE THE HAZARDS AND IS ITS
     * OPPOSITE, which is exactly why it gets pinned rather than counted.
     *
     * Both arrived on a measured frame, both are small body commands, and the
     * hazard row two lines up opened its motion gates on the argument that a
     * light which may only act in park cannot do its job. That argument does
     * not transfer: FOLDING A MIRROR AT SPEED REMOVES REARWARD VISION, and
     * every rule an owner would actually write for it -- fold on walking away,
     * unfold on getting in -- happens in park.
     *
     * So the next person who reads the hazard row and "makes the mirror
     * consistent with it" has to argue with a red test. That is the entire
     * purpose of this block: a mutation that opened these two gates passed
     * every other assertion in this repo on 2026-09-07. */
    const FsdBodyCaps* m = fsd_body_caps(FSD_ACT_MIRROR);
    CHECK(m->armable_at_runtime, "mirror armable: frame measured 2026-09-06");
    CHECK(!m->may_act_while_moving,
          "🔴 mirrors must not fold at speed -- that is rearward vision");
    CHECK(!m->may_act_out_of_park, "and not out of park, for the same reason");
    CHECK(m->min_interval_ms >= 3000u,
          "the mirror drives a motor that takes seconds to travel, got %u",
          (unsigned)m->min_interval_ms);
    CHECK(m->max_hold_ms > 0u,
          "TSL sends one frame per direction, so a re-sender is a bug to bound");

    /* 🔴 THE LIGHT HORN SITS ON THE HAZARDS' SIDE OF THAT LINE, and the two
     * rows above are why it gets said out loud. A horn is for warning somebody
     * who is about to hit you, which happens at speed; a horn that may only
     * sound in park cannot do that, exactly as with the hazards.
     *
     * What bounds it instead is the interval. The failure this row has to
     * survive is not one beep in the wrong place, it is a stuck rule beeping
     * over and over -- so it is rate-limited an order above the map light. */
    const FsdBodyCaps* lh = fsd_body_caps(FSD_ACT_LIGHT_HORN);
    CHECK(lh->armable_at_runtime, "light horn armable: frame measured 2026-09-06");
    CHECK(lh->may_act_while_moving, "a horn that only sounds in park is not a horn");
    CHECK(lh->may_act_out_of_park, "and out of park, for the same reason");
    CHECK(lh->min_interval_ms >= 1000u,
          "a stuck rule must not be able to hold on the horn, got %u",
          (unsigned)lh->min_interval_ms);
}

/* 🔴 THE BENCH REFUSED WHILE ALL OF THE ABOVE WAS GREEN.
 *
 * On 2026-09-07 the driver gate learned to accept a latched belt. 4,437 host
 * tests passed, six mutations bit, eight boards built -- and replaying the very
 * capture the fix was written for still produced "axis: no driver". The gate
 * was right; nothing filled belt_seen / belt_latched on the way in. Every test
 * above builds FsdBodyInputs by hand, so every test above was blind to it.
 *
 * These two stand where that blindness was. The first watches the copy itself;
 * the second replays the bench: driverPresent false for the whole drive, belt
 * latched, and the answer has to be OK. Delete either belt line in
 * fsd_body_inputs_from_state() and the second one fails. */
static void test_state_fields_reach_the_inputs(void) {
    const uint32_t now = 900000;

    FSDState st;
    memset(&st, 0, sizeof(st));
    st.op_mode = OpMode_Active;
    st.tesla_ota_in_progress = true;
    st.rx_stale = true;
    st.di_gear = FSD_GEAR_D;
    st.di_gear_seen = true;
    st.di_gear_ms = now - 11;
    st.ui_buckle_status = true;
    st.belt_seen = true;
    st.belt_seen_ms = now - 22;

    FsdBodyInputs in;
    memset(&in, 0, sizeof(in));
    in.bus_tx_open = true; // caller's own field: must survive the copy
    fsd_body_inputs_from_state(&in, &st);

    CHECK(in.op_mode == OpMode_Active, "op_mode did not arrive");
    CHECK(in.ota_in_progress, "ota_in_progress did not arrive");
    CHECK(in.rx_stale, "rx_stale did not arrive");
    CHECK(in.gear == FSD_GEAR_D, "gear did not arrive");
    CHECK(in.gear_seen, "gear_seen did not arrive");
    CHECK(in.gear_ms == now - 11, "gear_ms did not arrive");
    CHECK(in.belt_seen, "belt_seen did not arrive -- the 2026-09-07 bug");
    CHECK(in.belt_latched, "belt_latched did not arrive -- the 2026-09-07 bug");
    CHECK(in.belt_ms == now - 22, "belt_ms did not arrive");
    CHECK(in.bus_tx_open, "the copy clobbered a field it does not own");

    /* The unlatched belt has to travel too, or the gate reads a stale true. */
    st.ui_buckle_status = false;
    fsd_body_inputs_from_state(&in, &st);
    CHECK(!in.belt_latched, "an unlatched belt did not arrive");

    /* NULL either side and nothing is touched. */
    FsdBodyInputs keep = in;
    fsd_body_inputs_from_state(&in, NULL);
    fsd_body_inputs_from_state(NULL, &st);
    CHECK(memcmp(&keep, &in, sizeof(in)) == 0, "NULL wrote something");
}

static void test_belt_from_state_rescues_the_gate(void) {
    const uint32_t now = 900000;

    /* The bench vector, byte for byte in the fields that matter: 0x3C2 mux 0
     * with driverPresent clear on all 100 frames and frontBuckleSwitch == 2. */
    FSDState st;
    memset(&st, 0, sizeof(st));
    st.op_mode = OpMode_Active;
    st.di_gear = FSD_GEAR_P;
    st.di_gear_seen = true;
    st.di_gear_ms = now;
    st.ui_buckle_status = true;
    st.belt_seen = true;
    st.belt_seen_ms = now;

    /* 🔴 ASKED OF GEAR_D, NOT THE TURN SIGNAL. The occupancy gate this test
     * was written against is gone (2026-09-08) and the turn signal no longer
     * reads the belt at all -- but the SUBJECT here is not the turn signal, it
     * is that fsd_body_inputs_from_state() carries the belt from FSDState into
     * the axis. requires_belt still reads it, and GEAR_D is the row that sets
     * it. Retargeting keeps the subject; deleting the test would have lost it
     * along with the gate. */
    FsdBodyCaps c = *fsd_body_caps(FSD_ACT_GEAR_D);
    CHECK(c.requires_belt, "the row this test leans on still asks for a belt");

    FsdBodyInputs in = good_inputs(now);
    in.action_enabled[FSD_ACT_GEAR_D] = true;
    /* Take away what the helper handed us, so ONLY the state copy can put the
     * belt back. Without this the test passes with the copy deleted. */
    in.belt_seen = false;
    in.belt_latched = false;
    in.belt_ms = 0;

    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_NO_BELT,
          "without the belt this is exactly what the bench said");

    fsd_body_inputs_from_state(&in, &st);
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_OK,
          "the belt reached the axis on the bench but not here: got %s",
          fsd_body_verdict_str(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now)));

    /* And it is the belt doing the work, not the copy loosening something
     * else: unlatch it in FSDState and the refusal comes straight back. */
    st.ui_buckle_status = false;
    fsd_body_inputs_from_state(&in, &st);
    CHECK(fsd_body_caps_verdict(&c, &in, FSD_ACT_GEAR_D, now) == FSD_BODY_NO_BELT,
          "an unlatched belt still let the action through");
}

/* ── The field vector that produced 19 refusals ────────────────────────────
 *
 * 🔴 THIS IS NOT A HAND-BUILT STRUCT. The eight bytes below are lifted from
 * captures/2026-09-07-첫쓰기/운전석맵등6회 -- the run where the owner pressed
 * the driver map-light switch six times with the belt UNFASTENED and not one
 * frame left the board. `ruleq` said `axis: no driver`, nineteen times.
 *
 * Both vectors appear here because the pair is what proves the bits are the
 * ones that mattered: the two captures from that visit differ in exactly two
 * bits of byte 6, and the belted one DID transmit.
 *
 *   벨트 풀림  0x3C2#00 55 55 55 00 00 69 85   byte6 & 3 == 1  (unlatched)
 *   벨트 착용  0x3C2#00 55 55 55 00 00 6A 85   byte6 & 3 == 2  (latched)
 *
 * driverPresent is bit 4 of byte 0 and is 0 in BOTH -- that is the dead-gate
 * finding of 2026-09-07, still visible right here in the data.
 *
 * The frames go through the REAL observers, not through fields set by hand.
 * A host test that assembles FsdBodyInputs itself is exactly what let the
 * missing belt producer reach the bench on 2026-09-07, so this one starts
 * from bytes and ends at a verdict. */
static void test_the_capture_that_refused_now_passes(void) {
    printf("\n-- 2026-09-07 벨트 풀림 캡처: 그때 19회 거부, 지금은 --\n");

    const uint32_t now = 500000;
    const uint8_t unlatched[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};
    const uint8_t latched[8]   = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x6A, 0x85};

    /* The bytes say what the capture said. If either of these is ever wrong the
     * rest of the test is measuring something else. */
    CHECK((unlatched[6] & 0x03u) == 1u, "the unfastened vector really is 1");
    CHECK((latched[6] & 0x03u) == 2u, "and the fastened one really is 2");
    CHECK(((unlatched[0] >> 4) & 0x01u) == 0u,
          "driverPresent is 0 in the field data -- the dead gate, in the data");

    FSDState st;
    memset(&st, 0, sizeof(st));
    st.op_mode = OpMode_Active;

    CANFRAME f;
    memset(&f, 0, sizeof(f));
    f.id = 0x3C2u;
    f.data_lenght = 8;
    memcpy(f.buffer, unlatched, 8);
    fsd_drive_observe_belt_switch(&st, &f, now);

    CHECK(st.belt_seen, "the observer heard the frame");
    CHECK(!st.ui_buckle_status, "and read the belt as unfastened");

    FsdBodyInputs in;
    memset(&in, 0, sizeof(in));
    in.op_mode = OpMode_Active;
    in.bus_tx_open = true;
    in.action_enabled[FSD_ACT_TURN_SIGNAL] = true;
    in.gear_seen = true;
    in.gear = FSD_GEAR_P;
    in.gear_ms = now;
    in.speed_seen = true;
    in.speed_ms = now;
    fsd_body_inputs_from_state(&in, &st);

    /* 🟢 THE WHOLE POINT. Same bytes, same action, and the answer flipped. */
    const FsdBodyVerdict v = fsd_body_allows(&in, FSD_ACT_TURN_SIGNAL, now);
    CHECK(v == FSD_BODY_OK,
          "the vector that refused 19 times is allowed now, got %s",
          fsd_body_verdict_str(v));

    /* And the belted one is not treated any differently -- if it were, some
     * gate would still be reading the belt for this action. */
    memcpy(f.buffer, latched, 8);
    fsd_drive_observe_belt_switch(&st, &f, now);
    CHECK(st.ui_buckle_status, "the observer read the belt as fastened");
    fsd_body_inputs_from_state(&in, &st);
    CHECK(fsd_body_allows(&in, FSD_ACT_TURN_SIGNAL, now) == FSD_BODY_OK,
          "and fastening it changes nothing -- nobody is reading it");

    /* Never having heard 0x3C2 at all is the third case, and it was its own
     * refusal (NO_DRIVER). It must not be a refusal any more either. */
    FsdBodyInputs blind = in;
    blind.belt_seen = false;
    blind.belt_latched = false;
    blind.belt_ms = 0;
    CHECK(fsd_body_allows(&blind, FSD_ACT_TURN_SIGNAL, now) == FSD_BODY_OK,
          "never hearing the frame is not a refusal either");

    /* 🔴 AND THE ONE ROW THAT STILL READS THE BELT IS STILL SHUT -- but for a
     * different reason, and it must be that reason. If GEAR_D ever refused
     * with a belt verdict here it would mean the row had become reachable. */
    const FsdBodyVerdict g = fsd_body_allows(&in, FSD_ACT_GEAR_D, now);
    CHECK(g == FSD_BODY_NOT_ARMABLE,
          "GEAR_D is refused for being unarmable, not for a belt: got %s",
          fsd_body_verdict_str(g));
}

int main(void) {
    printf("test_body\n");
    test_state_fields_reach_the_inputs();
    test_belt_from_state_rescues_the_gate();
    test_the_capture_that_refused_now_passes();
    test_door_is_armed_but_every_gate_still_holds();
    test_zero_is_the_tightest_row();
    test_axis_refuses_in_order();
    test_tx_ids_are_refused_unconditionally();
    test_t1_edges();
    test_t1_fails_closed();
    test_t1_respects_the_axis();
    test_t1_budget();
    test_t2_gesture();
    test_t2_frame_discipline();
    test_rewrite_opened_nothing();
    test_turn_signal_row_is_open_where_it_has_to_be();
    test_caps_table_is_well_formed();
    test_rate_limit();
    test_gear_row_shape();
    test_gear_gates_when_armed();
    test_passenger_seat_fails_closed();
    test_owner_decisions_are_in_the_table();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
