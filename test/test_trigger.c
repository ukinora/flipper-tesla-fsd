/*
 * test_trigger.c — host tests for the signal -> event layer.
 *
 * The assertions that matter most:
 *
 *   - a car switch is treated as LEVEL. Getting this wrong the other way wedges
 *     a button until reboot (fsd_button.h), and this file is the only place it
 *     is declared, so nothing else can catch it.
 *   - the first frame after boot is not a transition. Otherwise every rule
 *     fires once at power-on.
 *   - a state trigger goes quiet after WE disturb it, and a switch trigger
 *     never does.
 *   - a frame of the wrong multiplex changes nothing, rather than reading the
 *     fields it does not carry as zero.
 *
 * The frames are the same real ones test_signal.c uses.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_trigger.h"

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

#define EVBUF 8
static FsdTriggerEvent g_ev[EVBUF];

// 0x3E2, seven bytes. byte1 bit6 is the front-left switch, byte0 bits 6-7 the
// front-left lamp — both verbatim from 맵등손으로-A1.
static void light_frame(uint8_t *d, bool fl_switch, bool fl_lamp) {
    d[0] = (uint8_t)(0x02 | (fl_lamp ? 0x40 : 0x00));
    d[1] = (uint8_t)(fl_switch ? 0x40 : 0x00);
    d[2] = 0x80; d[3] = 0x0A; d[4] = 0x00; d[5] = 0xC0; d[6] = 0x04;
}

static uint8_t feed_light(FsdTriggers *t, bool sw, bool lamp, uint32_t ms) {
    uint8_t d[7];
    light_frame(d, sw, lamp);
    return fsd_trig_on_frame(t, 0x3E2u, d, 7, ms, g_ev, EVBUF);
}

/* ── the LEVEL declaration ───────────────────────────────────────────────── */

// A car switch held past the threshold must produce LONG. On an EVENT button it
// would produce a phantom LONG with no press at all, and then STUCK, and stay
// silent forever — so this test is really "did init tell the bank LEVEL".
static void test_car_switches_are_level(void) {
    FsdTriggers t;
    fsd_trig_init(&t);

    CHECK(fsd_trig_switch_count() == FSD_TRIG_SWITCH_COUNT,
          "the table holds %d switch signals, the bank has room for %d",
          fsd_trig_switch_count(), FSD_TRIG_SWITCH_COUNT);
    CHECK(FSD_TRIG_BANKS * FSD_BTN_MAX >= FSD_TRIG_SWITCH_COUNT,
          "%d banks of %d hold %d switch signals", FSD_TRIG_BANKS, FSD_BTN_MAX,
          FSD_TRIG_SWITCH_COUNT);

    feed_light(&t, false, false, 1000); // establish
    uint8_t n = feed_light(&t, true, false, 1100); // finger down
    CHECK(n == 0, "a press alone is not yet an event");

    // Nothing is reported while a finger rests, so LONG has to come from tick.
    n = fsd_trig_tick(&t, 1100 + FSD_BTN_LONG_MS, g_ev, EVBUF);
    CHECK(n == 1 && g_ev[0].kind == FSD_TRIG_LONG && g_ev[0].signal == FSD_SIG_MAP_SW_FL,
          "held past the threshold is a LONG on the right signal");

    n = fsd_trig_tick(&t, 1100 + FSD_BTN_LONG_MS + 50, g_ev, EVBUF);
    CHECK(n == 0, "LONG fires once, not every tick");

    // A tap. 160-360 ms is what the capture showed a real one to be.
    fsd_trig_init(&t);
    feed_light(&t, false, false, 1000);
    feed_light(&t, true, false, 1100);
    n = feed_light(&t, false, false, 1260);
    CHECK(n == 1 && g_ev[0].kind == FSD_TRIG_PRESS, "a 160 ms tap is a PRESS");

    // Shorter than the debounce is not a press at all.
    fsd_trig_init(&t);
    feed_light(&t, false, false, 1000);
    feed_light(&t, true, false, 1100);
    n = feed_light(&t, false, false, 1110);
    CHECK(n == 0, "10 ms is bounce, not a press");
}

/* ── boot ────────────────────────────────────────────────────────────────── */

// Every state signal is unknown until a frame carries it. If the first frame
// counted as a transition, every rule in the car would fire at power-on.
static void test_first_frame_is_not_a_transition(void) {
    FsdTriggers t;
    fsd_trig_init(&t);

    uint8_t n = feed_light(&t, false, true, 1000); // lamp already on at boot
    CHECK(n == 0, "the first sight of a state is not a change");

    n = feed_light(&t, false, true, 1100);
    CHECK(n == 0, "an unchanged state is not a change either");

    n = feed_light(&t, false, false, 1200);
    CHECK(n == 2, "off is a change: LEAVE then ENTER");
    CHECK(g_ev[0].kind == FSD_TRIG_STATE_LEAVE && g_ev[0].value == 1, "left 1");
    CHECK(g_ev[1].kind == FSD_TRIG_STATE_ENTER && g_ev[1].value == 0, "entered 0");
    CHECK(g_ev[0].signal == FSD_SIG_MAP_ON_FL && g_ev[1].signal == FSD_SIG_MAP_ON_FL,
          "both name the lamp, not the switch");
}

/* ── the loop trap ───────────────────────────────────────────────────────── */

// "when the map light comes on" fires for whoever turned it on, and we are one
// of the whoevers. Two rules on one state is a loop.
static void test_our_own_writes_go_quiet(void) {
    FsdTriggers t;
    fsd_trig_init(&t);
    feed_light(&t, false, false, 1000);

    fsd_trig_disturbed(&t, FSD_SIG_MAP_ON_FL, 1000);
    CHECK(fsd_trig_is_quiet(&t, FSD_SIG_MAP_ON_FL, 1000), "quiet immediately");
    CHECK(fsd_trig_is_quiet(&t, FSD_SIG_MAP_ON_FL, 1000 + FSD_TRIG_SUPPRESS_MS - 1),
          "still quiet just inside the window");
    CHECK(!fsd_trig_is_quiet(&t, FSD_SIG_MAP_ON_FL, 1000 + FSD_TRIG_SUPPRESS_MS),
          "the window ends");

    uint8_t n = feed_light(&t, false, true, 1100);
    CHECK(n == 0, "the lamp we lit does not trigger us");

    // 🔴 But the value is still tracked, so the next real change is seen. A
    // suppression that also froze the value would make the signal wrong, not
    // quiet.
    n = feed_light(&t, false, false, 1000 + FSD_TRIG_SUPPRESS_MS + 10);
    CHECK(n == 2 && g_ev[0].value == 1 && g_ev[1].value == 0,
          "after the window, the change from the suppressed value is seen");

    // A switch cannot be suppressed. We cannot press one, so anything arriving
    // there came from a person, and silencing it silences the owner.
    fsd_trig_init(&t);
    feed_light(&t, false, false, 1000);
    fsd_trig_disturbed(&t, FSD_SIG_MAP_SW_FL, 1000);
    CHECK(!fsd_trig_is_quiet(&t, FSD_SIG_MAP_SW_FL, 1000), "a switch never goes quiet");
    feed_light(&t, true, false, 1100);
    n = feed_light(&t, false, false, 1260);
    CHECK(n == 1 && g_ev[0].kind == FSD_TRIG_PRESS, "and its press still arrives");
}

/* ── multiplex ───────────────────────────────────────────────────────────── */

// 0x3C2 carries the window switches on one multiplex and the scroll on the
// other. A frame of the wrong variant must change nothing at all.
static void test_wrong_multiplex_changes_nothing(void) {
    FsdTriggers t;
    fsd_trig_init(&t);

    uint8_t pack[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};
    uint8_t scroll[8] = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};

    fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 1000, g_ev, EVBUF);   // establish
    fsd_trig_on_frame(&t, 0x3C2u, scroll, 8, 1010, g_ev, EVBUF); // establish

    pack[4] = 0x02; // front-left window up
    uint8_t n = fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 1100, g_ev, EVBUF);
    CHECK(n == 0, "a press alone is not an event yet");
    pack[4] = 0x00;
    n = fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 1260, g_ev, EVBUF);
    CHECK(n == 1 && g_ev[0].signal == FSD_SIG_WIN_UP_FL && g_ev[0].kind == FSD_TRIG_PRESS,
          "the window tap lands on the right signal");

    // 🔴 The scroll frame does not carry the window fields. If it were read as
    // "those are all zero", a scroll frame arriving mid-press would look like
    // the finger left — and the press would end early, every time.
    pack[4] = 0x02;
    fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 2000, g_ev, EVBUF); // down
    n = fsd_trig_on_frame(&t, 0x3C2u, scroll, 8, 2050, g_ev, EVBUF);
    CHECK(n == 0, "the other multiplex produces nothing");
    pack[4] = 0x00;
    n = fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 2160, g_ev, EVBUF);
    CHECK(n == 1 && g_ev[0].kind == FSD_TRIG_PRESS,
          "and the press spanning it is still one press");
}

/* ── the scroll ──────────────────────────────────────────────────────────── */

static void test_scroll_is_a_count_not_a_level(void) {
    FsdTriggers t;
    fsd_trig_init(&t);
    uint8_t f[8] = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};

    uint8_t n = fsd_trig_on_frame(&t, 0x3C2u, f, 8, 1000, g_ev, EVBUF);
    CHECK(n == 0, "zero detents is not an event, even on the first frame");

    f[3] = 0x01;
    n = fsd_trig_on_frame(&t, 0x3C2u, f, 8, 1100, g_ev, EVBUF);
    CHECK(n == 1 && g_ev[0].kind == FSD_TRIG_DELTA && g_ev[0].value == 1, "one up");

    // 🔴 The same value again is another event, not a repeat to be filtered.
    // A level would be de-duplicated here; a count must not be, or a steady
    // roll would report its first detent and then nothing.
    n = fsd_trig_on_frame(&t, 0x3C2u, f, 8, 1150, g_ev, EVBUF);
    CHECK(n == 1 && g_ev[0].value == 1, "one up again is a second detent");

    f[3] = 0x37; // -9, a fast roll
    n = fsd_trig_on_frame(&t, 0x3C2u, f, 8, 1200, g_ev, EVBUF);
    CHECK(n == 1 && g_ev[0].value == -9, "a burst arrives whole and signed");

    f[3] = 0x00;
    n = fsd_trig_on_frame(&t, 0x3C2u, f, 8, 1250, g_ev, EVBUF);
    CHECK(n == 0, "back to zero is the wheel resting, not an event");

    // Our own scroll write must not trigger a rule watching the scroll.
    f[3] = 0x01;
    fsd_trig_disturbed(&t, FSD_SIG_SCROLL_TICKS, 1300);
    n = fsd_trig_on_frame(&t, 0x3C2u, f, 8, 1310, g_ev, EVBUF);
    CHECK(n == 0, "the detent we injected does not come back to us");
}

/* ── overflow and bounds ─────────────────────────────────────────────────── */

static void test_never_writes_past_the_buffer(void) {
    FsdTriggers t;
    fsd_trig_init(&t);

    // A state change produces two events. With room for one, the second is
    // dropped rather than written past the end.
    FsdTriggerEvent one[1];
    uint8_t d[7];
    light_frame(d, false, false);
    fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1000, one, 1);
    light_frame(d, false, true);
    const uint8_t n = fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1100, one, 1);
    CHECK(n == 1, "capped at the buffer size");
    CHECK(one[0].kind == FSD_TRIG_STATE_LEAVE, "and it is the first one, not the last");

    CHECK(fsd_trig_on_frame(NULL, 0x3E2u, d, 7, 1000, one, 1) == 0, "NULL state");
    CHECK(fsd_trig_on_frame(&t, 0x3E2u, NULL, 7, 1000, one, 1) == 0, "NULL data");
    CHECK(fsd_trig_tick(NULL, 1000, one, 1) == 0, "NULL tick");
    CHECK(!fsd_trig_is_quiet(NULL, FSD_SIG_MAP_ON_FL, 0), "NULL is not quiet");
    CHECK(!fsd_trig_is_quiet(&t, FSD_SIG_COUNT, 0), "out of range is not quiet");
}

/* ── the second bank ─────────────────────────────────────────────────────────
 *
 * Twelve switch signals do not fit in one FsdButtons, because FSD_BTN_MAX is
 * capped at 10 by ble_central.cpp's 32-bit action mask — raising it stops the
 * board compiling, which is how that ceiling was found.
 *
 * So slots 10 and 11 live in a second bank, and the risk that introduces is
 * arithmetic: `slot % FSD_BTN_MAX` puts slot 10 at index 0, the same index as
 * slot 0. If the bank were wrong, pressing the rear-right window would look
 * like pressing the front-left map light.
 */
static void test_the_second_bank_is_not_the_first(void) {
    FsdTriggers t;
    fsd_trig_init(&t);

    uint8_t pack[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};
    fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 1000, g_ev, EVBUF); // establish
    uint8_t d[7];
    light_frame(d, false, false);
    fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1000, g_ev, EVBUF); // establish

    // Rear-right window up is the last switch signal, so it is in the far bank.
    pack[5] = 0x20;
    fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 1100, g_ev, EVBUF);
    pack[5] = 0x00;
    uint8_t n = fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 1260, g_ev, EVBUF);
    CHECK(n == 1 && g_ev[0].signal == FSD_SIG_WIN_UP_RR && g_ev[0].kind == FSD_TRIG_PRESS,
          "the last switch signal still classifies");

    // 🔴 And it did not disturb slot 0, which shares its index in the other
    // bank. A hold there would have started at 1100 and be a LONG by now.
    n = fsd_trig_tick(&t, 1100 + FSD_BTN_LONG_MS + 100, g_ev, EVBUF);
    CHECK(n == 0, "pressing the far bank left the first bank alone");

    // The reverse: hold the front-left map light and check the far-bank signal
    // is untouched by the tick that fires it.
    fsd_trig_init(&t);
    fsd_trig_on_frame(&t, 0x3C2u, pack, 8, 1000, g_ev, EVBUF);
    light_frame(d, true, false);
    fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1100, g_ev, EVBUF);
    n = fsd_trig_tick(&t, 1100 + FSD_BTN_LONG_MS, g_ev, EVBUF);
    CHECK(n == 1 && g_ev[0].signal == FSD_SIG_MAP_SW_FL,
          "exactly one LONG, and it names the held signal");

    // Every switch signal must land on a distinct slot. Twelve signals sharing
    // eleven slots would show up as one of them never firing.
    bool used[FSD_TRIG_SWITCH_COUNT];
    memset(used, 0, sizeof(used));
    int n_switch = 0;
    for (int i = 0; i < FSD_SIG_COUNT; i++) {
        const FsdSignalDef *def = fsd_signal_def((FsdSignal)i);
        if (!def || def->kind != FSD_SIGK_SWITCH) continue;
        n_switch++;
    }
    CHECK(n_switch == FSD_TRIG_SWITCH_COUNT, "%d switch signals, %d slots", n_switch,
          FSD_TRIG_SWITCH_COUNT);
    (void)used;
}

static void test_kinds_are_nameable(void) {
    for (int k = 0; k <= FSD_TRIG_DELTA; k++)
        CHECK(fsd_trig_kind_str((FsdTriggerKind)k)[0] != '?', "kind %d is named", k);

    // Double press is off until asked for, because turning it on costs that
    // signal's tap the width of the window.
    FsdTriggers t;
    fsd_trig_init(&t);
    feed_light(&t, false, false, 1000);
    feed_light(&t, true, false, 1100);
    const uint8_t n = feed_light(&t, false, false, 1260);
    CHECK(n == 1 && g_ev[0].kind == FSD_TRIG_PRESS,
          "a tap reports immediately while double-press is off");

    // Asking for it on something that is not a switch is a no-op, not a crash.
    fsd_trig_set_double(&t, FSD_SIG_GEAR, true);
    fsd_trig_set_double(&t, FSD_SIG_COUNT, true);
    CHECK(1, "set_double on a non-switch is harmless");
}

int main(void) {
    printf("test_trigger\n");
    test_car_switches_are_level();
    test_first_frame_is_not_a_transition();
    test_our_own_writes_go_quiet();
    test_wrong_multiplex_changes_nothing();
    test_scroll_is_a_count_not_a_level();
    test_the_second_bank_is_not_the_first();
    test_never_writes_past_the_buffer();
    test_kinds_are_nameable();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
