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
    sp.enc.tick_toward_faster = -2; // pretend the capture says the opposite
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

    sp.enc.tick_toward_faster = 40;  // outside 6-bit signed
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "detent +40 does not fit 6 bits — must be refused, got byte3=0x%02X",
          buf[3]);

    sp.enc.tick_toward_faster = -40;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "detent -40 does not fit 6 bits — must be refused");

    // A zero detent is not a movement; emitting it burns budget for nothing.
    sp.enc.tick_toward_faster = 0;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "a zero detent moves nothing — must be refused");

    // -32 fits the field but its negation (+32) does not: it wraps back to -32,
    // so UP and DOWN would put the identical value on the wire. The usable
    // range has to be symmetric, which makes -32 invalid despite fitting.
    sp.enc.tick_toward_faster = -32;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(!fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8),
          "-32 must be refused: negating it does not fit, so UP == DOWN");

    // The real edges stay valid and stay distinguishable.
    sp.enc.tick_toward_faster = 31;
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_UP, buf, 8), "+31 is valid");
    CHECK(buf[3] == 0x1F, "byte3=0x%02X exp 0x1F", buf[3]);
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x01;
    CHECK(fsd_sp_apply_scroll(&sp, FSD_SP_ACT_TICK_DOWN, buf, 8), "-31 is valid");
    CHECK(buf[3] == 0x21, "byte3=0x%02X exp 0x21 (6-bit -31)", buf[3]);

    sp.enc.tick_toward_faster = -31;
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
    sp.enc.tick_toward_faster = 40;
    CHECK(fsd_sp_request(&sp, &in, 3, 1000) == FSD_SP_ERR_UNVERIFIED,
          "out-of-range detent must refuse the request, not just the write");

    ready(&sp);
    sp.enc.tick_toward_faster = 0;
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


// ── the car's own scale (4th visit, 2026-09-05 + 2nd visit, 2026-09-03) ──────
//
// Frames below are COPIED OUT OF THE CAPTURES, byte for byte. A test built from
// the table could only prove the table agrees with itself.

// The car's idle 0x3C2 mux-1 frame, and the two the commercial device inserted
// 0-1 ms behind it. Every byte but byte3 is identical, which is the finding.
static const uint8_t CAR_IDLE[8]   = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};
static const uint8_t TSL_PLUS1[8]  = {0x29, 0x55, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80};
static const uint8_t TSL_PLUS5[8]  = {0x29, 0x55, 0x00, 0x05, 0x00, 0x00, 0x00, 0x80};

static void test_raw_scale_is_measured_and_not_monotonic(void) {
    // Measured on the car, 2nd visit (2026-09-03, gear D): one +1 detent walked
    // 0x3FD from Sloth to Chill to Standard to Hurry, and the values it read
    // back were 4, 0, 1, 2. Slowest first.
    CHECK(FSD_SP_RAW_BY_RANK[0] == 4u, "rank 0 (Sloth) raw=%u exp 4",
          FSD_SP_RAW_BY_RANK[0]);
    CHECK(FSD_SP_RAW_BY_RANK[1] == 0u, "rank 1 (Chill) raw=%u exp 0",
          FSD_SP_RAW_BY_RANK[1]);
    CHECK(FSD_SP_RAW_BY_RANK[2] == 1u, "rank 2 (Standard) raw=%u exp 1",
          FSD_SP_RAW_BY_RANK[2]);
    CHECK(FSD_SP_RAW_BY_RANK[3] == 2u, "rank 3 (Hurry) raw=%u exp 2",
          FSD_SP_RAW_BY_RANK[3]);

    // 🔴 The whole reason this table exists, stated as an assertion: the raw
    // numbers do NOT rise with speed. Sloth (4) sorts above Hurry (2), so a
    // direction taken from the sign of a raw difference points the wrong way.
    int raw_monotonic = 1;
    for(unsigned i = 1; i < FSD_SP_PROFILE_COUNT; i++) {
        if(FSD_SP_RAW_BY_RANK[i] > FSD_SP_RAW_BY_RANK[i - 1]) continue;
        raw_monotonic = 0;
    }
    CHECK(!raw_monotonic, "raw CAN values must NOT be ordered by speed");
    CHECK(FSD_SP_RAW_BY_RANK[0] > FSD_SP_RAW_BY_RANK[FSD_SP_PROFILE_MAX],
          "raw Sloth (%u) must sort above raw Hurry (%u) — a numeric clamp inverts",
          FSD_SP_RAW_BY_RANK[0], FSD_SP_RAW_BY_RANK[FSD_SP_PROFILE_MAX]);

    // The named constants and the ordered table are the same statement.
    CHECK(FSD_SP_RAW_SLOTH == 4u && FSD_SP_RAW_CHILL == 0u &&
          FSD_SP_RAW_STANDARD == 1u && FSD_SP_RAW_HURRY == 2u,
          "named raw constants must match the capture");
}

static void test_rank_conversion_round_trips(void) {
    for(uint8_t rank = 0; rank <= FSD_SP_PROFILE_MAX; rank++) {
        uint8_t raw = 0xFFu, back = 0xFFu;
        CHECK(fsd_sp_raw_from_rank(rank, &raw), "rank %u must convert", rank);
        CHECK(fsd_sp_rank_from_raw(raw, &back), "raw %u must convert back", raw);
        CHECK(back == rank, "round trip rank %u -> raw %u -> rank %u", rank, raw,
              back);
    }

    // Values the car has never sent are not profiles. Saying "unknown" is the
    // point: a mis-decoded frame that looks like a profile is worse than one
    // that looks like nothing, because the clamp would trust it.
    uint8_t r = 0xEEu;
    CHECK(!fsd_sp_rank_from_raw(3u, &r), "raw 3 is not a value this car sends");
    CHECK(r == 0xEEu, "a refused conversion must not touch the output");
    CHECK(!fsd_sp_rank_from_raw(7u, &r), "raw 7 (HW4 range) must be refused");
    CHECK(!fsd_sp_rank_from_raw(255u, &r), "raw 255 must be refused");
    uint8_t raw = 0xEEu;
    CHECK(!fsd_sp_raw_from_rank(4u, &raw), "rank 4 does not exist");
    CHECK(raw == 0xEEu, "a refused conversion must not touch the output");
    CHECK(!fsd_sp_rank_from_raw(0u, NULL), "NULL out must be refused");
    CHECK(!fsd_sp_raw_from_rank(0u, NULL), "NULL out must be refused");
}

// The concrete harm, and the boundary that prevents it.
static void test_observe_raw_is_the_boundary(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    FsdSpInputs in = good_inputs(FSD_SP_PROFILE_MAX); // Hurry, as a rank
    CHECK(fsd_sp_request(&sp, &in, 0, 1000) == FSD_SP_OK, "Hurry -> Sloth ok");

    // 🔴 Feeding the RAW value straight in is the mistake. Raw Sloth is 4,
    // which is outside the rank scale, so fsd_sp_observe() drops it: the car
    // reaches the target and the machine never notices.
    fsd_sp_observe(&sp, FSD_SP_RAW_SLOTH, 1100);
    CHECK(sp.observed == FSD_SP_PROFILE_MAX,
          "raw 4 fed to observe() must be ignored, observed=%u", sp.observed);

    // Through the boundary it lands as rank 0 and the request converges.
    CHECK(fsd_sp_observe_raw(&sp, FSD_SP_RAW_SLOTH, 1100),
          "observe_raw must accept raw Sloth");
    CHECK(sp.observed == 0u, "raw 4 must land as rank 0, got %u", sp.observed);

    // And a value the car never sends is reported as unknown, not silently
    // taken as some profile.
    CHECK(!fsd_sp_observe_raw(&sp, 3u, 1200), "raw 3 must be refused");
    CHECK(sp.observed == 0u, "a refused raw value must not move observed");
    CHECK(!fsd_sp_observe_raw(NULL, FSD_SP_RAW_CHILL, 1200),
          "observe_raw(NULL) is a no-op");
}

// ── the wire, against the bytes the commercial device actually sent ─────────
static void test_detents_match_the_captured_frames(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    uint8_t buf[8];

    // +1: our bytes must equal the device's bytes, all eight of them.
    memcpy(buf, CAR_IDLE, sizeof(buf));
    CHECK(fsd_sp_apply_detents(&sp, 1, buf, 8), "+1 must be accepted");
    CHECK(memcmp(buf, TSL_PLUS1, sizeof(buf)) == 0,
          "+1 frame must equal the captured one: got %02X%02X%02X%02X%02X%02X%02X%02X",
          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

    // +5: ONE frame carrying five, not five frames carrying one. This is the
    // 4th visit's finding and the reason this function exists at all.
    memcpy(buf, CAR_IDLE, sizeof(buf));
    CHECK(fsd_sp_apply_detents(&sp, 5, buf, 8), "+5 must be accepted");
    CHECK(memcmp(buf, TSL_PLUS5, sizeof(buf)) == 0,
          "+5 frame must equal the captured one: got %02X%02X%02X%02X%02X%02X%02X%02X",
          buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);

    // Turned around: the decoder must read the device's own frames back as the
    // counts a human read off the capture.
    int8_t n = 0;
    CHECK(fsd_sp_read_detents(TSL_PLUS1, 8, &n) && n == 1,
          "captured +1 frame must read back as 1, got %d", n);
    CHECK(fsd_sp_read_detents(TSL_PLUS5, 8, &n) && n == 5,
          "captured +5 frame must read back as 5, got %d", n);
    CHECK(fsd_sp_read_detents(CAR_IDLE, 8, &n) && n == 0,
          "the car's own idle frame must read back as 0, got %d", n);
}

// The negatives were measured too — 2nd visit, the owner turning the wheel by
// hand. Two's complement, so they are the interesting half.
static void test_negative_detents_are_twos_complement(void) {
    FsdSpeedProfile sp;
    ready(&sp);
    uint8_t buf[8];

    // Captured byte3 values and the detent counts the capture attributes to
    // them: 0x3E = -2, 0x3D = -3, 0x3C = -4, 0x3B = -5.
    static const struct { uint8_t byte3; int8_t detents; } CAPTURED[] = {
        {0x3Eu, -2}, {0x3Du, -3}, {0x3Cu, -4}, {0x3Bu, -5},
    };
    for(unsigned i = 0; i < sizeof(CAPTURED) / sizeof(CAPTURED[0]); i++) {
        memcpy(buf, CAR_IDLE, sizeof(buf));
        CHECK(fsd_sp_apply_detents(&sp, CAPTURED[i].detents, buf, 8),
              "%d must be accepted", CAPTURED[i].detents);
        CHECK(buf[3] == CAPTURED[i].byte3,
              "%d -> byte3=0x%02X exp 0x%02X", CAPTURED[i].detents, buf[3],
              CAPTURED[i].byte3);

        uint8_t frame[8];
        memcpy(frame, CAR_IDLE, sizeof(frame));
        frame[3] = CAPTURED[i].byte3;
        int8_t n = 0;
        CHECK(fsd_sp_read_detents(frame, 8, &n) && n == CAPTURED[i].detents,
              "0x%02X must read back as %d, got %d", CAPTURED[i].byte3,
              CAPTURED[i].detents, n);
    }

    // Symmetry: +n and -n are the same magnitude, and the field never confuses
    // them. 0x3F is -1, not +63 — without sign extension a scroll DOWN would
    // be reported as a large scroll UP.
    for(int8_t n = 1; n <= FSD_SP_DETENT_MAX; n++) {
        uint8_t up[8], down[8];
        memcpy(up, CAR_IDLE, sizeof(up));
        memcpy(down, CAR_IDLE, sizeof(down));
        CHECK(fsd_sp_apply_detents(&sp, n, up, 8) &&
              fsd_sp_apply_detents(&sp, (int8_t)-n, down, 8),
              "+/-%d must both be accepted", n);
        CHECK(up[3] != down[3], "+%d and -%d must differ on the wire", n, n);
        int8_t a = 0, b = 0;
        CHECK(fsd_sp_read_detents(up, 8, &a) && a == n,
              "+%d round trip got %d", n, a);
        CHECK(fsd_sp_read_detents(down, 8, &b) && b == (int8_t)-n,
              "-%d round trip got %d", n, b);
    }
}

static void test_detent_writer_is_gated_and_bounded(void) {
    // 🔴 The shipped encoding is UNVERIFIED, so the writer must refuse even a
    // perfectly legal count. This is the flag that stands between us and a car
    // with a recorded emergency-braking incident; a new function must not
    // become the way around it.
    FsdSpeedProfile shipped;
    fsd_sp_init(&shipped);
    uint8_t buf[8];
    memcpy(buf, CAR_IDLE, sizeof(buf));
    CHECK(!fsd_sp_apply_detents(&shipped, 1, buf, 8),
          "the shipped (unverified) encoding must refuse to build a frame");
    CHECK(memcmp(buf, CAR_IDLE, sizeof(buf)) == 0,
          "a refused write must not touch the frame");

    FsdSpeedProfile sp;
    ready(&sp);
    // 0 moves nothing; out-of-range values would be MASKED into the opposite
    // direction rather than clamped (+40 lands on the wire as -24).
    static const int16_t BAD[] = {0, 32, -32, 40, -40, 127, -128};
    for(unsigned i = 0; i < sizeof(BAD) / sizeof(BAD[0]); i++) {
        memcpy(buf, CAR_IDLE, sizeof(buf));
        CHECK(!fsd_sp_apply_detents(&sp, (int8_t)BAD[i], buf, 8),
              "detent %d must be refused", BAD[i]);
        CHECK(buf[3] == 0x00u, "detent %d must not touch the frame", BAD[i]);
    }

    // mux 0 is the seat/window/button view — writing byte3 there moves a seat.
    //
    // These four byte0 values are every one this car was seen to send on 0x3C2
    // across three visits: idle mux 0, and the three button presses (horn,
    // hazard button, driver-present). All of them are mux 0, and none of them
    // may be mistaken for a scroll frame.
    //
    // ⚠️ What these frames CANNOT settle is the width of the selector itself.
    // 0x00/0x04/0x08/0x10 are refused whether the mask is 2 or 3 bits, and the
    // scroll frame's 0x29 is mux 1 under either. Only a mux-1 frame WITH one of
    // those switch bits set would tell them apart, and no capture has one. The
    // mask stays at 2 bits because the 4th visit showed 0x04 is "mux 0 with the
    // horn pressed" rather than a mux 4 that does not exist — a 3-bit mask
    // would read it as the latter.
    static const uint8_t MUX0_BYTE0[] = {0x00u, 0x04u, 0x08u, 0x10u};
    for(unsigned i = 0; i < sizeof(MUX0_BYTE0) / sizeof(MUX0_BYTE0[0]); i++) {
        uint8_t m0[8] = {MUX0_BYTE0[i], 0x55u, 0x55u, 0x55u,
                         0x00u,         0x00u, 0x69u, 0x85u};
        int8_t junk = 0;
        CHECK(!fsd_sp_apply_detents(&sp, 1, m0, 8),
              "byte0=0x%02X is mux 0 — must be refused", MUX0_BYTE0[i]);
        CHECK(m0[3] == 0x55u, "a refused write must not touch byte3");
        CHECK(!fsd_sp_read_detents(m0, 8, &junk),
              "byte0=0x%02X is mux 0 — byte3 there is not a detent",
              MUX0_BYTE0[i]);
    }

    memcpy(buf, CAR_IDLE, sizeof(buf));
    buf[0] = 0x00u;
    CHECK(!fsd_sp_apply_detents(&sp, 1, buf, 8), "mux 0 must be refused");
    CHECK(!fsd_sp_read_detents(buf, 8, NULL) , "read with NULL out is refused");
    int8_t n = 0;
    CHECK(!fsd_sp_read_detents(buf, 8, &n), "mux 0 must not be read as a detent");
    CHECK(!fsd_sp_apply_detents(&sp, 1, buf, 3), "DLC 3 has no byte3");
    CHECK(!fsd_sp_apply_detents(&sp, 1, NULL, 8), "NULL buf is refused");
    CHECK(!fsd_sp_apply_detents(NULL, 1, buf, 8), "NULL sp is refused");
    CHECK(!fsd_sp_read_detents(NULL, 8, &n), "NULL buf is refused");

    // Bits 6-7 of byte3 belong to other signals and must survive.
    memcpy(buf, CAR_IDLE, sizeof(buf));
    buf[3] = 0xC0u;
    CHECK(fsd_sp_apply_detents(&sp, 5, buf, 8), "accept");
    CHECK(buf[3] == 0xC5u, "must preserve byte3 bits 6-7: 0x%02X exp 0xC5", buf[3]);
}

// 🔴 Filling the table in is NOT the same as arming it. Every field above is
// measured, and the gate must still be shut.
static void test_measured_table_is_still_not_verified(void) {
    FsdSpeedProfile sp;
    fsd_sp_init(&sp);

    CHECK(FSD_SP_ENCODING_DEFAULT.tick_toward_faster == 1,
          "measured: +1 detent = one step toward FASTER, got %d",
          FSD_SP_ENCODING_DEFAULT.tick_toward_faster);
    CHECK(FSD_SP_ENCODING_DEFAULT.ticks_per_step == 1,
          "measured: one detent, one step, got %u",
          FSD_SP_ENCODING_DEFAULT.ticks_per_step);
    CHECK(FSD_SP_ENCODING_DEFAULT.wrap == false,
          "the ends saturate; and with the top end unobserved, false is also "
          "the conservative value");

    CHECK(FSD_SP_ENCODING_DEFAULT.verified == false,
          "the shipped table must stay UNVERIFIED — the top end was never "
          "observed (the car was parked) and arming is the owner's decision");
    CHECK(!fsd_sp_encoding_ok(&FSD_SP_ENCODING_DEFAULT),
          "an unverified table must not pass the gate, however complete it is");
    CHECK(!fsd_sp_encoding_ok(&sp.enc),
          "a freshly initialised machine must not pass the gate");

    FsdSpInputs in = good_inputs(0);
    CHECK(fsd_sp_request(&sp, &in, FSD_SP_PROFILE_MAX, 1000) == FSD_SP_ERR_UNVERIFIED,
          "a request against the shipped table must be refused as UNVERIFIED");

    // ...and flipping just that one field is what opens it. Stated here so the
    // day someone flips it, this test says out loud what changed.
    FsdSpEncoding armed = FSD_SP_ENCODING_DEFAULT;
    armed.verified = true;
    CHECK(fsd_sp_encoding_ok(&armed),
          "the measured table is complete: `verified` is the only thing left");
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
    test_raw_scale_is_measured_and_not_monotonic();
    test_rank_conversion_round_trips();
    test_observe_raw_is_the_boundary();
    test_detents_match_the_captured_frames();
    test_negative_detents_are_twos_complement();
    test_detent_writer_is_gated_and_bounded();
    test_measured_table_is_still_not_verified();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
