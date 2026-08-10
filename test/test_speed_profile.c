/*
 * test_speed_profile.c — host unit tests for the closed-loop speed-profile
 * convergence machine (fsd_logic/fsd_speed_profile.c).
 *
 * Every refusal path is asserted, because the refusals ARE the feature: this
 * runs on a car where scroll injection has a recorded emergency-braking
 * incident, so "does nothing unless explicitly armed and verified" has to be
 * provable, not assumed.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_speed_profile.h"

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

// A car that satisfies every precondition, with the encoding marked verified.
// Tests that care about a specific refusal knock out exactly one field.
static FsdSpInputs good_inputs(uint8_t profile) {
    FsdSpInputs in;
    memset(&in, 0, sizeof(in));
    in.tx_armed = true;
    in.listen_only = false;
    in.ota_in_progress = false;
    in.scroll_bus_present = true;
    in.status_fresh = true;
    in.observed_profile = profile;
    return in;
}

static void ready(FsdSpeedProfile* sp) {
    fsd_sp_init(sp);
    sp->enc.verified = true; // pretend a capture confirmed the table
}

// ── defaults ────────────────────────────────────────────────────────────────
static void test_init(void) {
    FsdSpeedProfile sp;
    fsd_sp_init(&sp);
    CHECK(sp.phase == FSD_SP_IDLE, "init phase=%d exp IDLE", sp.phase);
    CHECK(!fsd_sp_busy(&sp), "init must not be busy");
    CHECK(sp.last_error == FSD_SP_OK, "init error=%d exp OK", sp.last_error);
    CHECK(sp.enc.verified == false,
          "shipped encoding must be UNVERIFIED — this is the safety default");
    CHECK(sp.enc.ticks_per_step == 1, "default ticks_per_step=%u exp 1",
          sp.enc.ticks_per_step);
    CHECK(FSD_SP_ENCODING_DEFAULT.verified == false,
          "the exported default table must also be unverified");
}

// ── refusals ────────────────────────────────────────────────────────────────
static void test_refuses_unverified(void) {
    FsdSpeedProfile sp;
    fsd_sp_init(&sp); // NOT marked verified
    FsdSpInputs in = good_inputs(0);
    FsdSpError e = fsd_sp_request(&sp, &in, 3, 1000);
    CHECK(e == FSD_SP_ERR_UNVERIFIED, "unverified encoding -> %s",
          fsd_sp_error_str(e));
    CHECK(!fsd_sp_busy(&sp), "refused request must not start");
}

static void test_refuses_unarmed(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);
    in.tx_armed = false;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_NOT_ARMED,
          "unarmed must refuse");
    CHECK(!fsd_sp_busy(&sp), "refused request must not start");
}

static void test_refuses_environment(void) {
    FsdSpeedProfile sp;
    FsdSpInputs in;

    ready(&sp);
    in = good_inputs(0);
    in.listen_only = true;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_LISTEN_ONLY,
          "listen-only must refuse");

    ready(&sp);
    in = good_inputs(0);
    in.ota_in_progress = true;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_OTA,
          "OTA must refuse");

    ready(&sp);
    in = good_inputs(0);
    in.scroll_bus_present = false;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_NO_SCROLL_BUS,
          "no 0x3C2 must refuse (Bus 6 tap cannot do this)");

    ready(&sp);
    in = good_inputs(0);
    in.status_fresh = false;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_NO_STATE,
          "stale 0x3FD must refuse — relative stepping needs a known start");
}

static void test_refuses_bad_target(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);
    CHECK(fsd_sp_request(&sp, &in, 4, 1000) == FSD_SP_ERR_RANGE,
          "target 4 must refuse (0..3 only)");
    CHECK(fsd_sp_request(&sp, &in, 255, 1000) == FSD_SP_ERR_RANGE,
          "target 255 must refuse");
}

static void test_refuses_busy_and_cooldown(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);

    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_OK, "first request ok");
    CHECK(fsd_sp_busy(&sp), "should be converging");
    CHECK(fsd_sp_request(&sp, &in, 2, 1010) == FSD_SP_ERR_BUSY,
          "second request while busy must refuse");

    // Finish it, then a request inside the cooldown window must be refused.
    fsd_sp_abort(&sp, FSD_SP_OK, 2000);
    in.observed_profile = 3;
    CHECK(fsd_sp_request(&sp, &in, 1, 2000 + FSD_SP_COOLDOWN_MS - 1) ==
              FSD_SP_ERR_COOLDOWN,
          "request inside cooldown must refuse");
    CHECK(fsd_sp_request(&sp, &in, 1, 2000 + FSD_SP_COOLDOWN_MS) == FSD_SP_OK,
          "request after cooldown must be accepted");

    // A FAILED request must cool down too, or a car that keeps refusing would
    // be retried in a tight loop.
    ready(&sp);
    in = good_inputs(0);
    CHECK(fsd_sp_request(&sp, &in, 3, 5000) == FSD_SP_OK, "request ok");
    fsd_sp_abort(&sp, FSD_SP_ERR_STALLED, 6000);
    CHECK(fsd_sp_request(&sp, &in, 3, 6000 + FSD_SP_COOLDOWN_MS - 1) ==
              FSD_SP_ERR_COOLDOWN,
          "failure must also start a cooldown");
}

// ── happy paths ─────────────────────────────────────────────────────────────
static void test_already_at_target(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(2);
    CHECK(fsd_sp_request(&sp, &in, 2, 1000) == FSD_SP_OK, "same-value request ok");
    CHECK(sp.phase == FSD_SP_DONE, "same value -> DONE immediately");
    CHECK(sp.ticks_used == 0, "same value must emit no ticks, got %u",
          sp.ticks_used);
    CHECK(fsd_sp_poll(&sp, &in, 1010) == FSD_SP_ACT_NONE, "DONE polls idle");
}

// Drive a full convergence with a simulated car that obeys every tick.
static void test_converges_up(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);
    uint32_t t = 1000;
    CHECK(fsd_sp_request(&sp, &in, 3, t) == FSD_SP_OK, "request 0->3 ok");

    int ticks = 0;
    uint8_t car = 0; // the car's actual profile
    for (int guard = 0; guard < 50 && fsd_sp_busy(&sp); guard++) {
        FsdSpAction act = fsd_sp_poll(&sp, &in, t);
        if (act != FSD_SP_ACT_NONE) {
            CHECK(act == FSD_SP_ACT_TICK_UP, "0->3 must tick UP");
            ticks++;
            if (act == FSD_SP_ACT_TICK_UP && car < FSD_SP_PROFILE_MAX) car++;
            t += 100; // car reacts before the settle window closes
            in.observed_profile = car;
            fsd_sp_observe(&sp, car, t);
        } else {
            t += 50;
        }
    }
    CHECK(sp.phase == FSD_SP_DONE, "0->3 must converge, phase=%d err=%s",
          sp.phase, fsd_sp_error_str(sp.last_error));
    CHECK(ticks == 3, "0->3 should take 3 ticks, took %d", ticks);
    CHECK(sp.observed == 3, "final observed=%u exp 3", sp.observed);
    // start_profile survives the whole request — callers report "0 -> 3".
    CHECK(sp.start_profile == 0, "start_profile=%u exp 0", sp.start_profile);
    CHECK(sp.last_error == FSD_SP_OK, "converged request must report OK, got %s",
          fsd_sp_error_str(sp.last_error));
}

static void test_converges_down(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(3);
    uint32_t t = 1000;
    CHECK(fsd_sp_request(&sp, &in, 1, t) == FSD_SP_OK, "request 3->1 ok");

    int ticks = 0;
    uint8_t car = 3;
    for (int guard = 0; guard < 50 && fsd_sp_busy(&sp); guard++) {
        FsdSpAction act = fsd_sp_poll(&sp, &in, t);
        if (act != FSD_SP_ACT_NONE) {
            CHECK(act == FSD_SP_ACT_TICK_DOWN, "3->1 must tick DOWN");
            ticks++;
            if (car > FSD_SP_PROFILE_MIN) car--;
            t += 100;
            in.observed_profile = car;
            fsd_sp_observe(&sp, car, t);
        } else {
            t += 50;
        }
    }
    CHECK(sp.phase == FSD_SP_DONE, "3->1 must converge, err=%s",
          fsd_sp_error_str(sp.last_error));
    CHECK(ticks == 2, "3->1 should take 2 ticks, took %d", ticks);
}

// With wrap enabled, 0->3 is one tick DOWN, not three UP.
static void test_wrap_takes_short_way(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    sp.enc.wrap = true;
    FsdSpInputs in = good_inputs(0);
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_OK, "wrap request ok");
    CHECK(fsd_sp_poll(&sp, &in, 1000) == FSD_SP_ACT_TICK_DOWN,
          "wrap 0->3 should go DOWN (1 step) not UP (3 steps)");
}

// ── failure paths ───────────────────────────────────────────────────────────
static void test_stalls_when_car_ignores(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);
    uint32_t t = 1000;
    CHECK(fsd_sp_request(&sp, &in, 3, t) == FSD_SP_OK, "request ok");

    // The car never moves. Advance well past each settle window.
    for (int guard = 0; guard < 60 && fsd_sp_busy(&sp); guard++) {
        fsd_sp_poll(&sp, &in, t);
        t += FSD_SP_SETTLE_MS;
    }
    CHECK(sp.phase == FSD_SP_FAILED, "unresponsive car must fail, phase=%d",
          sp.phase);
    CHECK(sp.last_error == FSD_SP_ERR_STALLED ||
              sp.last_error == FSD_SP_ERR_TIMEOUT ||
              sp.last_error == FSD_SP_ERR_EXHAUSTED,
          "expected a bounded give-up, got %s", fsd_sp_error_str(sp.last_error));
    CHECK(sp.ticks_used <= FSD_SP_MAX_TICKS, "ticks %u must stay within budget",
          sp.ticks_used);
}

static void test_timeout(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_OK, "request ok");
    fsd_sp_poll(&sp, &in, 1000 + FSD_SP_TIMEOUT_MS);
    CHECK(sp.phase == FSD_SP_FAILED, "timeout must fail");
    CHECK(sp.last_error == FSD_SP_ERR_TIMEOUT, "expected timeout, got %s",
          fsd_sp_error_str(sp.last_error));
}

// A precondition that disappears mid-flight aborts rather than pausing.
static void test_aborts_when_conditions_vanish(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_OK, "request ok");
    CHECK(fsd_sp_poll(&sp, &in, 1000) == FSD_SP_ACT_TICK_UP, "first tick");

    in.ota_in_progress = true; // car starts updating mid-sequence
    CHECK(fsd_sp_poll(&sp, &in, 1600) == FSD_SP_ACT_NONE,
          "must not tick once a precondition is gone");
    CHECK(sp.phase == FSD_SP_FAILED, "vanished precondition must abort");
    CHECK(sp.last_error == FSD_SP_ERR_ABORTED, "expected abort, got %s",
          fsd_sp_error_str(sp.last_error));
}

static void test_tick_budget(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);
    uint32_t t = 1000;
    fsd_sp_request(&sp, &in, 3, t);
    // Car oscillates without ever reaching the target: 0 -> 1 -> 0 -> 1 ...
    uint8_t car = 0;
    for (int guard = 0; guard < 80 && fsd_sp_busy(&sp); guard++) {
        if (fsd_sp_poll(&sp, &in, t) != FSD_SP_ACT_NONE) {
            car = (uint8_t)(car ? 0 : 1);
            t += 100;
            in.observed_profile = car;
            fsd_sp_observe(&sp, car, t);
        } else {
            t += 100;
        }
    }
    CHECK(!fsd_sp_busy(&sp), "oscillating car must terminate the request");
    CHECK(sp.ticks_used <= FSD_SP_MAX_TICKS,
          "must never exceed the tick budget: used %u, max %u", sp.ticks_used,
          FSD_SP_MAX_TICKS);
}

// ── wire encoding ───────────────────────────────────────────────────────────
static void test_apply_scroll(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    uint8_t buf[8];

    // mux=1 frame: tick lands in byte3 bits 0-5.
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "mux=1 must accept");
    CHECK(buf[3] == 0x01, "UP -> byte3=0x%02X exp 0x01", buf[3]);

    // DOWN is -1 in 6-bit two's complement = 0x3F.
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_DOWN, buf, 8),
          "mux=1 must accept DOWN");
    CHECK(buf[3] == 0x3F, "DOWN -> byte3=0x%02X exp 0x3F (6-bit -1)", buf[3]);

    // Upper two bits of byte3 belong to other signals and must survive.
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    buf[3] = 0xC0;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8), "accept");
    CHECK(buf[3] == 0xC1, "must preserve byte3 bits 6-7: 0x%02X exp 0xC1",
          buf[3]);

    // mux=0 is the seat/window view — writing there would move a seat.
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x00;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "mux=0 must be refused");
    CHECK(buf[3] == 0x00, "refused call must not touch the frame");

    // Short frame and ACT_NONE are both no-ops.
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 3),
          "DLC 3 has no byte3 — must refuse");
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_NONE, buf, 8),
          "ACT_NONE must refuse");
}

// The direction constant is the one thing a capture will change. Prove the
// machine follows the table rather than a hardcoded sign.
static void test_encoding_is_table_driven(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    sp.enc.tick_toward_higher = -2; // pretend the capture says the opposite
    uint8_t buf[8];
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8), "accept");
    CHECK(buf[3] == 0x3E, "UP with tick=-2 -> byte3=0x%02X exp 0x3E", buf[3]);

    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_DOWN, buf, 8), "accept");
    CHECK(buf[3] == 0x02, "DOWN with tick=-2 -> byte3=0x%02X exp 0x02", buf[3]);
}

static void test_ticks_per_step(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    sp.enc.ticks_per_step = 2; // two detents per profile step
    FsdSpInputs in = good_inputs(0);
    CHECK(fsd_sp_request(&sp, &in, 1, 1000) == FSD_SP_OK, "request ok");
    CHECK(fsd_sp_poll(&sp, &in, 1000) == FSD_SP_ACT_TICK_UP, "first detent");
    CHECK(sp.phase == FSD_SP_STEP,
          "mid-step must not settle yet, phase=%d", sp.phase);
    CHECK(fsd_sp_poll(&sp, &in, 1010) == FSD_SP_ACT_TICK_UP,
          "second detent must follow immediately");
    CHECK(sp.phase == FSD_SP_SETTLE, "step complete -> SETTLE, phase=%d",
          sp.phase);
    CHECK(sp.ticks_used == 2, "ticks_used=%u exp 2", sp.ticks_used);
}

// ── null safety ─────────────────────────────────────────────────────────────
static void test_null_safe(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    fsd_sp_init(NULL);
    fsd_sp_observe(NULL, 1, 0);
    fsd_sp_abort(NULL, FSD_SP_OK, 0);
    CHECK(!fsd_sp_busy(NULL), "busy(NULL) must be false");
    CHECK(fsd_sp_poll(NULL, NULL, 0) == FSD_SP_ACT_NONE, "poll(NULL) is a no-op");
    CHECK(fsd_sp_request(NULL, NULL, 1, 0) == FSD_SP_ERR_ABORTED,
          "request(NULL) must refuse");
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, NULL, 8),
          "apply_scroll(NULL buf) must refuse");
    // Out-of-range observations are ignored rather than corrupting state.
    sp.observed = 2;
    fsd_sp_observe(&sp, 9, 0);
    CHECK(sp.observed == 2, "out-of-range observe must be ignored, got %u",
          sp.observed);
}

// ── adversarial: cases the first round of tests did not cover ───────────────

// A step is a unit. The tick budget is counted per tick, so a step whose ticks
// straddle the budget ceiling gets cut in half: the car receives some of the
// detents and never the rest. What a half-turned wheel does is unknown, and
// "unknown input to the steering column" is exactly what this module must not
// produce. Budget must be spent in whole steps.
static void test_never_abandons_a_half_sent_step(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    sp.enc.ticks_per_step = 4;  // FSD_SP_MAX_TICKS (6) is not a multiple of 4
    FsdSpInputs in = good_inputs(0);
    uint32_t t = 1000;
    CHECK(fsd_sp_request(&sp, &in, 3, t) == FSD_SP_OK, "request ok");

    int ticks = 0;
    // The car never responds, so this ends in a bounded give-up. HOW it gives
    // up is the point.
    for (int guard = 0; guard < 100 && fsd_sp_busy(&sp); guard++) {
        if (fsd_sp_poll(&sp, &in, t) != FSD_SP_ACT_NONE) {
            ticks++;
            t += 10;
        } else {
            t += 100;
        }
    }
    CHECK(!fsd_sp_busy(&sp), "must terminate");
    CHECK(sp.pending_ticks == 0,
          "gave up mid-step with %u detents still owed — the car got a partial "
          "turn", sp.pending_ticks);
    CHECK(ticks % (int)sp.enc.ticks_per_step == 0,
          "emitted %d ticks, which is not a whole number of %u-tick steps",
          ticks, sp.enc.ticks_per_step);
}

// The convergence check sits AFTER the precondition re-check, so a request that
// already reached its target is reported as a failure if anything changes in
// the same instant. The phone then sees FAILED for work that succeeded.
static void test_converged_result_survives_a_late_precondition_loss(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(0);
    uint32_t t = 1000;
    CHECK(fsd_sp_request(&sp, &in, 1, t) == FSD_SP_OK, "request 0->1 ok");
    CHECK(fsd_sp_poll(&sp, &in, t) == FSD_SP_ACT_TICK_UP, "one tick");

    t += 100;
    in.observed_profile = 1;
    fsd_sp_observe(&sp, 1, t);  // the car arrived

    in.ota_in_progress = true;  // ...and an OTA starts in the same moment
    fsd_sp_poll(&sp, &in, t + 10);

    CHECK(sp.observed == sp.target, "precondition of this test: target reached");
    CHECK(sp.phase == FSD_SP_DONE,
          "converged request reported as phase=%d (%s) — success must not be "
          "overwritten by a late abort", sp.phase, fsd_sp_error_str(sp.last_error));
}

// swcRightScrollTicks is 6 bits signed: -32..31. A larger value is not clamped
// anywhere, it is masked — and 40 & 0x3F reads back as -24, i.e. the module
// scrolls the OPPOSITE way from what the table says. Since the table is filled
// in from a capture by a tool, a bad value must be refused, not reinterpreted.
static void test_out_of_range_detent_is_refused(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    uint8_t buf[8];

    sp.enc.tick_toward_higher = 40;  // outside 6-bit signed
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "detent +40 does not fit 6 bits — must be refused, got byte3=0x%02X",
          buf[3]);

    sp.enc.tick_toward_higher = -40;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "detent -40 does not fit 6 bits — must be refused");

    // A zero detent is not a movement; emitting it burns budget for nothing.
    sp.enc.tick_toward_higher = 0;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "a zero detent moves nothing — must be refused");

    // -32 fits the field but its negation (+32) does not: it wraps back to -32,
    // so UP and DOWN would put the identical value on the wire. The usable
    // range has to be symmetric, which makes -32 invalid despite fitting.
    sp.enc.tick_toward_higher = -32;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "-32 must be refused: negating it does not fit, so UP == DOWN");

    // The real edges stay valid and stay distinguishable.
    sp.enc.tick_toward_higher = 31;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8), "+31 is valid");
    CHECK(buf[3] == 0x1F, "byte3=0x%02X exp 0x1F", buf[3]);
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_DOWN, buf, 8), "-31 is valid");
    CHECK(buf[3] == 0x21, "byte3=0x%02X exp 0x21 (6-bit -31)", buf[3]);

    sp.enc.tick_toward_higher = -31;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8), "-31 as UP is valid");
    CHECK(buf[3] == 0x21, "byte3=0x%02X exp 0x21", buf[3]);
}

// The table now gates requests, not just the wire write.
static void test_bad_encoding_refuses_requests(void) {
    FsdSpeedProfile sp;
    FsdSpInputs in = good_inputs(0);

    ready(&sp);
    sp.enc.tick_toward_higher = 40;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_UNVERIFIED,
          "out-of-range detent must refuse the request, not just the write");

    ready(&sp);
    sp.enc.tick_toward_higher = 0;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_UNVERIFIED,
          "zero detent must refuse the request");

    // A step larger than the whole budget could never complete now that the
    // budget is reserved per step — catch it up front instead of emitting
    // nothing and reporting EXHAUSTED.
    ready(&sp);
    sp.enc.ticks_per_step = FSD_SP_MAX_TICKS + 1u;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_UNVERIFIED,
          "a step bigger than the budget must refuse the request");

    ready(&sp);
    sp.enc.ticks_per_step = 0;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_UNVERIFIED,
          "zero ticks per step must refuse the request");

    // And the shipped default is still refused for the original reason.
    fsd_sp_init(&sp);
    CHECK(!fsd_sp_encoding_ok(&sp.enc), "shipped table is unverified");
    ready(&sp);
    CHECK(fsd_sp_encoding_ok(&sp.enc), "a verified, in-range table passes");
}

int main(void) {
    printf("test_speed_profile\n");
    test_init();
    test_refuses_unverified();
    test_refuses_unarmed();
    test_refuses_environment();
    test_refuses_bad_target();
    test_refuses_busy_and_cooldown();
    test_already_at_target();
    test_converges_up();
    test_converges_down();
    test_wrap_takes_short_way();
    test_stalls_when_car_ignores();
    test_timeout();
    test_aborts_when_conditions_vanish();
    test_tick_budget();
    test_apply_scroll();
    test_encoding_is_table_driven();
    test_ticks_per_step();
    test_null_safe();
    test_never_abandons_a_half_sent_step();
    test_converged_result_survives_a_late_precondition_loss();
    test_out_of_range_detent_is_refused();
    test_bad_encoding_refuses_requests();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
