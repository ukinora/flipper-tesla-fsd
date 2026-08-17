/*
 * test_button.c — host tests for fsd_logic/fsd_button.c.
 *
 * Timing only. Everything radio-specific lives in ble_central.cpp and cannot be
 * tested without a button, which is exactly why the split exists.
 *
 * The cases that matter are the ones a person cannot produce on purpose: a
 * contact bouncing, a report lost so a release never arrives, a button wedged
 * under a seat. Those are what turn a control into a fault.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_button.h"

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

/* Hold button 0 from `at` for `hold_ms`, ticking every 20 ms the way loop()
 * would, and report what came out. */
static void press(FsdButtons* b, uint32_t at, uint32_t hold_ms, FsdBtnEvent* on_hold,
                  FsdBtnEvent* on_release) {
    if (on_hold) *on_hold = FSD_BTN_EV_NONE;
    fsd_btn_report(b, 0, true, at);
    for (uint32_t e = 0; e <= hold_ms; e += 20) {
        const FsdBtnEvent t = fsd_btn_tick(b, 0, at + e);
        if (t != FSD_BTN_EV_NONE && on_hold && *on_hold == FSD_BTN_EV_NONE) *on_hold = t;
    }
    const FsdBtnEvent r = fsd_btn_report(b, 0, false, at + hold_ms);
    if (on_release) *on_release = r;
}

static void test_short_and_long(void) {
    printf("\n-- a tap, and a hold --\n");

    FsdButtons b;
    FsdBtnEvent held = FSD_BTN_EV_NONE, rel = FSD_BTN_EV_NONE;

    fsd_btn_init(&b);
    press(&b, 1000, 100, &held, &rel);
    CHECK(held == FSD_BTN_EV_NONE, "a tap produces nothing while held");
    CHECK(rel == FSD_BTN_EV_SHORT, "and SHORT on release");
    CHECK(fsd_btn_shorts(&b, 0) == 1, "counted");
    CHECK(fsd_btn_last_hold_ms(&b, 0) == 100, "hold measured: %u",
          fsd_btn_last_hold_ms(&b, 0));

    // A hold fires WHILE held. Reporting it only on release would feel like lag
    // and the user would have no way to know it registered.
    fsd_btn_init(&b);
    press(&b, 1000, FSD_BTN_LONG_MS + 200u, &held, &rel);
    CHECK(held == FSD_BTN_EV_LONG, "LONG arrives while the button is still down");
    CHECK(rel == FSD_BTN_EV_NONE, "and the release adds nothing");
    CHECK(fsd_btn_longs(&b, 0) == 1 && fsd_btn_shorts(&b, 0) == 0,
          "counted as a long, not also a short");

    // Exactly at the threshold is a long.
    fsd_btn_init(&b);
    press(&b, 1000, FSD_BTN_LONG_MS, &held, &rel);
    CHECK(held == FSD_BTN_EV_LONG, "the threshold itself is a LONG");

    // Only once, no matter how long.
    fsd_btn_init(&b);
    fsd_btn_report(&b, 0, true, 1000);
    int longs = 0;
    for (uint32_t e = 0; e < 3000; e += 20)
        if (fsd_btn_tick(&b, 0, 1000 + e) == FSD_BTN_EV_LONG) longs++;
    CHECK(longs == 1, "LONG fires exactly once, got %d", longs);
}

static void test_bounce(void) {
    printf("\n-- a bounce is not a press --\n");

    FsdButtons b;
    FsdBtnEvent rel = FSD_BTN_EV_NONE;
    fsd_btn_init(&b);

    press(&b, 1000, FSD_BTN_DEBOUNCE_MS - 1u, NULL, &rel);
    CHECK(rel == FSD_BTN_EV_NONE, "too brief to be a press");
    CHECK(fsd_btn_bounces(&b, 0) == 1, "counted as a bounce");
    CHECK(fsd_btn_shorts(&b, 0) == 0, "and not as a short");

    // The bounce counter is the point: a link dropping reports looks exactly
    // like a bouncing contact from here, and both are worth seeing.
    for (int i = 0; i < 5; i++) press(&b, 2000 + i * 100u, 5, NULL, &rel);
    CHECK(fsd_btn_bounces(&b, 0) == 6, "bounces accumulate: %u",
          fsd_btn_bounces(&b, 0));
}

static void test_stuck(void) {
    printf("\n-- a button under a seat cushion --\n");

    FsdButtons b;
    fsd_btn_init(&b);
    fsd_btn_report(&b, 0, true, 1000);

    FsdBtnEvent stuck = FSD_BTN_EV_NONE;
    int events = 0;
    for (uint32_t e = 0; e <= FSD_BTN_STUCK_MS + 5000u; e += 20) {
        const FsdBtnEvent t = fsd_btn_tick(&b, 0, 1000 + e);
        if (t == FSD_BTN_EV_STUCK) stuck = t;
        if (t != FSD_BTN_EV_NONE) events++;
    }
    CHECK(stuck == FSD_BTN_EV_STUCK, "declared stuck");
    CHECK(fsd_btn_is_stuck(&b, 0), "and stays stuck");
    // LONG first, then STUCK, and then silence — not a stream.
    CHECK(events == 2, "exactly LONG then STUCK, got %d events", events);

    // A lost release report is indistinguishable from a wedged button, so the
    // only thing that clears it is seeing the button come up.
    const FsdBtnEvent r = fsd_btn_report(&b, 0, false, 1000 + FSD_BTN_STUCK_MS + 6000u);
    CHECK(r == FSD_BTN_EV_NONE, "the release that ends a stuck press is not a press");
    CHECK(!fsd_btn_is_stuck(&b, 0), "and it clears");

    // Working again afterwards.
    FsdBtnEvent rel = FSD_BTN_EV_NONE;
    press(&b, 100000, 100, NULL, &rel);
    CHECK(rel == FSD_BTN_EV_SHORT, "and the button works again");
}

static void test_levels_not_edges(void) {
    printf("\n-- repeated reports cost nothing --\n");

    // A peripheral that notifies its state at a fixed rate sends "down, down,
    // down..." — that must be one press, not one per report.
    FsdButtons b;
    fsd_btn_init(&b);
    for (uint32_t e = 0; e < 200; e += 20)
        CHECK(fsd_btn_report(&b, 0, true, 1000 + e) == FSD_BTN_EV_NONE,
              "a repeated down is not a new press");
    CHECK(fsd_btn_report(&b, 0, false, 1200) == FSD_BTN_EV_SHORT, "one press out");

    // And repeated "up" is nothing at all.
    for (int i = 0; i < 5; i++)
        CHECK(fsd_btn_report(&b, 0, false, 1300 + i * 20u) == FSD_BTN_EV_NONE,
              "a repeated up is nothing");
    CHECK(fsd_btn_shorts(&b, 0) == 1, "still exactly one press");
}

/*
 * Double-press. OFF by default, and that default is the point.
 *
 * fsd_button.h used to say "no double-press, deliberately": supporting it costs
 * every SHORT the width of the window, because a tap cannot be reported until a
 * second one is known not to be coming. That reasoning still holds — so the
 * cost is now OPT-IN, per button. A button with nothing mapped to a double
 * press behaves exactly as before, with no added latency.
 */
static void test_double_off_by_default(void) {
    printf("\n-- double-press is opt-in --\n");

    FsdButtons b;
    FsdBtnEvent rel = FSD_BTN_EV_NONE;
    fsd_btn_init(&b);

    CHECK(!fsd_btn_double_enabled(&b, 0), "off after init");

    // The whole reason it is off: this release must still be immediate.
    press(&b, 1000, 100, NULL, &rel);
    CHECK(rel == FSD_BTN_EV_SHORT, "SHORT still arrives on release, undelayed");
    CHECK(fsd_btn_doubles(&b, 0) == 0, "and nothing is counted as a double");
}

static void test_double_delays_the_short(void) {
    printf("\n-- with it on, a tap waits for its twin --\n");

    FsdButtons b;
    FsdBtnEvent rel = FSD_BTN_EV_NONE;
    fsd_btn_init(&b);
    fsd_btn_set_double(&b, 0, true);
    CHECK(fsd_btn_double_enabled(&b, 0), "on");

    press(&b, 1000, 100, NULL, &rel);
    CHECK(rel == FSD_BTN_EV_NONE, "the release is held back");
    CHECK(fsd_btn_shorts(&b, 0) == 0, "and not counted yet");

    // The window closes on a tick, because nothing is being reported by then.
    FsdBtnEvent late = FSD_BTN_EV_NONE;
    int events = 0;
    for (uint32_t e = 0; e <= FSD_BTN_DOUBLE_MS + 100u; e += 20) {
        const FsdBtnEvent t = fsd_btn_tick(&b, 0, 1100 + e);
        if (t != FSD_BTN_EV_NONE) { late = t; events++; }
    }
    CHECK(late == FSD_BTN_EV_SHORT, "SHORT arrives when the window closes");
    CHECK(events == 1, "exactly once, got %d", events);
    CHECK(fsd_btn_shorts(&b, 0) == 1, "counted then, not before");
}

static void test_double_fires(void) {
    printf("\n-- two taps inside the window --\n");

    FsdButtons b;
    FsdBtnEvent rel = FSD_BTN_EV_NONE;
    fsd_btn_init(&b);
    fsd_btn_set_double(&b, 0, true);

    press(&b, 1000, 60, NULL, &rel);                 // released at 1060
    CHECK(rel == FSD_BTN_EV_NONE, "first tap held back");
    press(&b, 1120, 60, NULL, &rel);                 // well inside the window
    CHECK(rel == FSD_BTN_EV_DOUBLE, "the second tap makes it a DOUBLE");
    CHECK(fsd_btn_doubles(&b, 0) == 1, "counted as a double");
    CHECK(fsd_btn_shorts(&b, 0) == 0, "and not as two shorts");

    // A third tap starts a fresh cycle rather than extending the last one.
    press(&b, 3000, 60, NULL, &rel);
    CHECK(rel == FSD_BTN_EV_NONE, "third tap opens a new window");
    for (uint32_t e = 0; e <= FSD_BTN_DOUBLE_MS + 100u; e += 20)
        fsd_btn_tick(&b, 0, 3060 + e);
    CHECK(fsd_btn_shorts(&b, 0) == 1, "and lands as one SHORT");
    CHECK(fsd_btn_doubles(&b, 0) == 1, "still one double");
}

static void test_double_window_expires(void) {
    printf("\n-- two taps too far apart are two taps --\n");

    FsdButtons b;
    FsdBtnEvent rel = FSD_BTN_EV_NONE;
    fsd_btn_init(&b);
    fsd_btn_set_double(&b, 0, true);

    press(&b, 1000, 60, NULL, &rel);
    for (uint32_t e = 0; e <= FSD_BTN_DOUBLE_MS + 100u; e += 20)
        fsd_btn_tick(&b, 0, 1060 + e);               // first SHORT lands here
    press(&b, 2000, 60, NULL, &rel);
    for (uint32_t e = 0; e <= FSD_BTN_DOUBLE_MS + 100u; e += 20)
        fsd_btn_tick(&b, 0, 2060 + e);

    CHECK(fsd_btn_shorts(&b, 0) == 2, "two shorts, got %u", fsd_btn_shorts(&b, 0));
    CHECK(fsd_btn_doubles(&b, 0) == 0, "no double");
}

static void test_double_then_hold(void) {
    printf("\n-- tap then hold is a hold --\n");

    // A held second press is a LONG, and the waiting first tap is dropped rather
    // than surfacing after it. Two events from one gesture would be worse than
    // losing the tap: the action would fire twice.
    FsdButtons b;
    FsdBtnEvent held = FSD_BTN_EV_NONE, rel = FSD_BTN_EV_NONE;
    fsd_btn_init(&b);
    fsd_btn_set_double(&b, 0, true);

    press(&b, 1000, 60, NULL, &rel);
    CHECK(rel == FSD_BTN_EV_NONE, "tap waits");
    press(&b, 1120, FSD_BTN_LONG_MS + 100u, &held, &rel);
    CHECK(held == FSD_BTN_EV_LONG, "the hold reports LONG while down");

    int late = 0;
    for (uint32_t e = 0; e <= FSD_BTN_DOUBLE_MS + 200u; e += 20)
        if (fsd_btn_tick(&b, 0, 1900 + e) != FSD_BTN_EV_NONE) late++;
    CHECK(late == 0, "and no SHORT turns up afterwards, got %d", late);
    CHECK(fsd_btn_shorts(&b, 0) == 0 && fsd_btn_doubles(&b, 0) == 0, "neither counted");
}

static void test_double_bounce_keeps_waiting(void) {
    printf("\n-- a bounce does not answer for the second tap --\n");

    FsdButtons b;
    FsdBtnEvent rel = FSD_BTN_EV_NONE;
    fsd_btn_init(&b);
    fsd_btn_set_double(&b, 0, true);

    press(&b, 1000, 60, NULL, &rel);
    press(&b, 1100, FSD_BTN_DEBOUNCE_MS - 1u, NULL, &rel);   // too brief to count
    CHECK(rel == FSD_BTN_EV_NONE, "the bounce is not a DOUBLE");
    CHECK(fsd_btn_bounces(&b, 0) == 1, "counted as a bounce");

    // The real second tap still has its chance.
    press(&b, 1200, 60, NULL, &rel);
    CHECK(rel == FSD_BTN_EV_DOUBLE, "and the tap after it completes the double");
}

static void test_independent_and_bounds(void) {
    printf("\n-- buttons do not interfere; indices are checked --\n");

    FsdButtons b;
    fsd_btn_init(&b);
    fsd_btn_report(&b, 0, true, 1000);
    fsd_btn_report(&b, 1, true, 1000);
    fsd_btn_report(&b, 0, false, 1100);
    CHECK(fsd_btn_shorts(&b, 0) == 1, "button 0 released");
    CHECK(fsd_btn_shorts(&b, 1) == 0, "button 1 still held");
    fsd_btn_report(&b, 1, false, 1200);
    CHECK(fsd_btn_shorts(&b, 1) == 1, "and then released");

    CHECK(fsd_btn_report(&b, FSD_BTN_MAX, true, 1000) == FSD_BTN_EV_NONE,
          "out-of-range index is refused");
    CHECK(fsd_btn_tick(&b, FSD_BTN_MAX, 1000) == FSD_BTN_EV_NONE, "and on tick");
    CHECK(fsd_btn_report(NULL, 0, true, 1000) == FSD_BTN_EV_NONE, "NULL is refused");
    CHECK(!fsd_btn_is_stuck(NULL, 0) && fsd_btn_shorts(NULL, 0) == 0, "NULL accessors");

    for (int e = 0; e <= FSD_BTN_EV_DOUBLE; e++) {
        const char* s = fsd_btn_event_str((FsdBtnEvent)e);
        CHECK(s && s[0] && strcmp(s, "?") != 0, "event %d has a name", e);
    }
}

int main(void) {
    printf("test_button\n");
    test_short_and_long();
    test_double_off_by_default();
    test_double_delays_the_short();
    test_double_fires();
    test_double_window_expires();
    test_double_then_hold();
    test_double_bounce_keeps_waiting();
    test_bounce();
    test_stuck();
    test_levels_not_edges();
    test_independent_and_bounds();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
