/*
 * test_signal.c — host tests for the named-signal table.
 *
 * 🔴 EVERY FRAME IN THIS FILE IS A REAL ONE.
 *
 * Not constructed to match the table — copied out of the 2026-09-01 capture,
 * with the timestamp it carried. That direction matters: a test built from the
 * table can only prove the table agrees with itself, which is the trap this
 * repository has walked into twice (a threshold test computed from the
 * threshold; a mask sweep that read the mask it was checking).
 *
 * The map-light and window walks are the strongest evidence here. The owner
 * pressed four switches one second apart in a fixed order, so the capture shows
 * one field moving at a time — if any two of these bit positions were swapped,
 * the sequence below would not line up.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_signal.h"

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

// Reads `s` out of `d` and compares. Any verdict but OK is a failure with the
// verdict named, so "wrong multiplex" never hides as "wrong value".
static void expect(FsdSignal s, uint32_t id, const uint8_t *d, uint8_t dlc, int32_t want,
                   const char *what) {
    int32_t got = 0x7FFFFFFF;
    const FsdSignalVerdict v = fsd_signal_extract(s, id, d, dlc, &got);
    if (v != FSD_SIGV_OK) {
        g_fail++;
        printf("  FAIL %s: %s\n", what, fsd_signal_verdict_str(v));
        return;
    }
    CHECK(got == want, "%s: got %ld, want %ld", what, (long)got, (long)want);
}

/* ── 0x3E2: the map-light walk ───────────────────────────────────────────────
 *
 * 맵등손으로-A1, front-left then front-right then rear-left then rear-right,
 * one second apart. Each press is a frame and each release is the next.
 */
static void test_map_light_walk(void) {
    struct {
        uint8_t d[7];
        const char *when;
        int sw[4]; // FL FR RL RR pressed
        int on[4]; // FL FR RL RR lit
    } steps[] = {
        {{0x02, 0x00, 0x80, 0x0A, 0x00, 0xC0, 0x04}, "0.050 baseline", {0,0,0,0}, {0,0,0,0}},
        {{0x42, 0x40, 0x80, 0x0A, 0x00, 0xC0, 0x04}, "6.281 FL down",  {1,0,0,0}, {1,0,0,0}},
        {{0x42, 0x00, 0x80, 0x0A, 0x00, 0xC0, 0x04}, "6.441 FL up",    {0,0,0,0}, {1,0,0,0}},
        {{0x42, 0x81, 0x80, 0x0A, 0x00, 0xC0, 0x04}, "7.492 FR down",  {0,1,0,0}, {1,1,0,0}},
        {{0x42, 0x01, 0x80, 0x0A, 0x00, 0xC0, 0x04}, "7.641 FR up",    {0,0,0,0}, {1,1,0,0}},
        {{0x42, 0x05, 0x81, 0x0A, 0x00, 0xC0, 0x04}, "8.481 RL down",  {0,0,1,0}, {1,1,1,0}},
        {{0x42, 0x05, 0x80, 0x0A, 0x00, 0xC0, 0x04}, "8.682 RL up",    {0,0,0,0}, {1,1,1,0}},
        {{0x42, 0x15, 0x82, 0x0A, 0x00, 0xC0, 0x04}, "9.571 RR down",  {0,0,0,1}, {1,1,1,1}},
        {{0x42, 0x15, 0x80, 0x0A, 0x00, 0xC0, 0x04}, "9.722 RR up",    {0,0,0,0}, {1,1,1,1}},
    };
    const FsdSignal sw[4] = {FSD_SIG_MAP_SW_FL, FSD_SIG_MAP_SW_FR, FSD_SIG_MAP_SW_RL,
                             FSD_SIG_MAP_SW_RR};
    const FsdSignal on[4] = {FSD_SIG_MAP_ON_FL, FSD_SIG_MAP_ON_FR, FSD_SIG_MAP_ON_RL,
                             FSD_SIG_MAP_ON_RR};

    for (size_t k = 0; k < sizeof(steps) / sizeof(steps[0]); k++) {
        for (int i = 0; i < 4; i++) {
            char buf[64];
            snprintf(buf, sizeof(buf), "%s sw%d", steps[k].when, i);
            expect(sw[i], 0x3E2u, steps[k].d, 7, steps[k].sw[i], buf);
            snprintf(buf, sizeof(buf), "%s on%d", steps[k].when, i);
            expect(on[i], 0x3E2u, steps[k].d, 7, steps[k].on[i], buf);
        }
    }

    // 🔴 The lamp stays lit after the finger leaves, and the switch does not.
    // That is the whole reason they are two signals: a rule on "pressed" fires
    // once, a rule on "lit" is true for as long as the light is on.
    const uint8_t held[7] = {0x42, 0x40, 0x80, 0x0A, 0x00, 0xC0, 0x04};
    const uint8_t after[7] = {0x42, 0x00, 0x80, 0x0A, 0x00, 0xC0, 0x04};
    expect(FSD_SIG_MAP_SW_FL, 0x3E2u, held, 7, 1, "held: switch");
    expect(FSD_SIG_MAP_ON_FL, 0x3E2u, held, 7, 1, "held: lamp");
    expect(FSD_SIG_MAP_SW_FL, 0x3E2u, after, 7, 0, "released: switch clears");
    expect(FSD_SIG_MAP_ON_FL, 0x3E2u, after, 7, 1, "released: lamp stays");
}

/* ── 0x3C2 mux 0: the window walk ────────────────────────────────────────── */
static void test_window_walk(void) {
    // 창문올림순서-A1, same order, one press each.
    struct {
        uint8_t b4, b5;
        FsdSignal s;
        int32_t want;
        const char *when;
    } up[] = {
        {0x02, 0x00, FSD_SIG_WIN_UP_FL, 2, "6.135 FL up"},
        {0x00, 0x02, FSD_SIG_WIN_UP_FR, 2, "6.875 FR up"},
        {0x20, 0x00, FSD_SIG_WIN_UP_RL, 2, "7.865 RL up"},
        {0x00, 0x10, FSD_SIG_WIN_UP_RR, 1, "8.725 RR up, first detent"},
        {0x00, 0x20, FSD_SIG_WIN_UP_RR, 2, "8.805 RR up, second detent"},
    };
    // 창문내림순서-A1.
    struct {
        uint8_t b4, b5;
        FsdSignal s;
        int32_t want;
        const char *when;
    } dn[] = {
        {0x04, 0x00, FSD_SIG_WIN_DN_FL, 1, "6.094 FL down, first"},
        {0x08, 0x00, FSD_SIG_WIN_DN_FL, 2, "6.223 FL down, second"},
        {0x00, 0x04, FSD_SIG_WIN_DN_FR, 1, "6.733 FR down, first"},
        {0x00, 0x08, FSD_SIG_WIN_DN_FR, 2, "6.803 FR down, second"},
        {0x40, 0x00, FSD_SIG_WIN_DN_RL, 1, "8.555 RL down, first"},
        {0x80, 0x00, FSD_SIG_WIN_DN_RL, 2, "8.634 RL down, second"},
        {0x00, 0x40, FSD_SIG_WIN_DN_RR, 1, "9.154 RR down, first"},
    };

    uint8_t f[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};

    for (size_t k = 0; k < sizeof(up) / sizeof(up[0]); k++) {
        f[4] = up[k].b4;
        f[5] = up[k].b5;
        expect(up[k].s, 0x3C2u, f, 8, up[k].want, up[k].when);
    }
    for (size_t k = 0; k < sizeof(dn) / sizeof(dn[0]); k++) {
        f[4] = dn[k].b4;
        f[5] = dn[k].b5;
        expect(dn[k].s, 0x3C2u, f, 8, dn[k].want, dn[k].when);
    }

    // Pressing one window must not read as any other. Eight fields in two
    // bytes is exactly where a transposed bit hides.
    const FsdSignal all[8] = {FSD_SIG_WIN_UP_FL, FSD_SIG_WIN_DN_FL, FSD_SIG_WIN_UP_RL,
                              FSD_SIG_WIN_DN_RL, FSD_SIG_WIN_UP_FR, FSD_SIG_WIN_DN_FR,
                              FSD_SIG_WIN_UP_RR, FSD_SIG_WIN_DN_RR};
    f[4] = 0x02;
    f[5] = 0x00;
    for (int i = 0; i < 8; i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "FL-up pressed, signal %d", i);
        expect(all[i], 0x3C2u, f, 8, (all[i] == FSD_SIG_WIN_UP_FL) ? 2 : 0, buf);
    }
}

/* ── 0x3C2 mux 0: belt and driver, from the gear capture ─────────────────── */
static void test_belt_and_driver(void) {
    // 기어D-A1: byte6 goes 0x69 -> 0x6A at t=6.362, and TSL asks for D 78 ms
    // later. byte0 is 0x10 throughout that capture and 0x00 in the others.
    uint8_t f[8] = {0x10, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x95};
    expect(FSD_SIG_BELT_FRONT, 0x3C2u, f, 8, 1, "6.061 belt unlatched");
    expect(FSD_SIG_DRIVER_PRESENT, 0x3C2u, f, 8, 1, "driver present");

    f[6] = 0x6A;
    expect(FSD_SIG_BELT_FRONT, 0x3C2u, f, 8, 2, "6.362 belt latched");

    // 🔴 driverPresent lives IN the multiplex byte. The mux comparison has to
    // mask it out or half the frames stop matching — which is why MUX_MASK is
    // not 0xFF.
    f[0] = 0x00;
    expect(FSD_SIG_DRIVER_PRESENT, 0x3C2u, f, 8, 0, "driver absent");
    expect(FSD_SIG_BELT_FRONT, 0x3C2u, f, 8, 2, "belt still readable with bit 4 clear");
}

/* ── 0x3C2 mux 0x29: scroll detents ──────────────────────────────────────── */
static void test_scroll_accumulates(void) {
    uint8_t f[8] = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};
    struct { uint8_t b3; int32_t want; const char *when; } t[] = {
        {0x00, 0, "idle"},
        {0x01, 1, "6.516 one up"},
        {0x3F, -1, "9.217 one down"},
        {0x08, 8, "7.331 a fast roll up"},
        {0x37, -9, "9.331 a fast roll down"},
        // The edges of a 6-bit signed field, which is where sign extension
        // written the lazy way goes wrong.
        {0x1F, 31, "largest positive"},
        {0x20, -32, "most negative"},
    };
    for (size_t k = 0; k < sizeof(t) / sizeof(t[0]); k++) {
        f[3] = t[k].b3;
        expect(FSD_SIG_SCROLL_TICKS, 0x3C2u, f, 8, t[k].want, t[k].when);
    }

    // Bits 6-7 of that byte were zero in every capture and are not part of the
    // field. A signal that swallowed them would read 0x40 as 64, or as -64.
    f[3] = 0x41;
    expect(FSD_SIG_SCROLL_TICKS, 0x3C2u, f, 8, 1, "bits 6-7 are not ours");
}

/* ── the multiplex ───────────────────────────────────────────────────────── */
static void test_multiplex_is_not_guessed(void) {
    const uint8_t pack[8] = {0x00, 0x55, 0x55, 0x55, 0x02, 0x00, 0x69, 0x85};
    const uint8_t scroll[8] = {0x29, 0x55, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80};
    int32_t v = 0;

    CHECK(fsd_signal_extract(FSD_SIG_SCROLL_TICKS, 0x3C2u, pack, 8, &v) == FSD_SIGV_WRONG_MUX,
          "the scroll detent is not in the switch-pack frame");
    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x3C2u, scroll, 8, &v) == FSD_SIGV_WRONG_MUX,
          "a window switch is not in the scroll frame");

    // 🔴 And the value is left alone on a refusal. A caller that ignores the
    // verdict gets its own stale reading, not a zero that looks like "not
    // pressed" — the difference between "I did not hear" and "it is off".
    v = 0x5A5A;
    CHECK(fsd_signal_extract(FSD_SIG_SCROLL_TICKS, 0x3C2u, pack, 8, &v) != FSD_SIGV_OK &&
              v == 0x5A5A,
          "a refused read does not write out");
}

/* ── doors and gear ──────────────────────────────────────────────────────── */
static void test_doors_and_gear(void) {
    // 운전석문열림-A1 / 운전석문닫힘-A1. 2 = shut, 1 = open, both directions seen.
    const uint8_t shut[8] = {0x22, 0xB3, 0x88, 0x02, 0x80, 0x0A, 0x21, 0x08};
    const uint8_t open[8] = {0x21, 0x72, 0x88, 0x02, 0x80, 0x0C, 0x21, 0x08};
    expect(FSD_SIG_DOOR_FL_LATCH, 0x102u, shut, 8, 2, "driver door shut");
    expect(FSD_SIG_DOOR_FL_LATCH, 0x102u, open, 8, 1, "driver door open");

    // 우뒷문열기-A1: TSL opened this one. Different byte, different bits.
    const uint8_t r_shut[8] = {0x22, 0xB3, 0x48, 0x02, 0xCE, 0xCA, 0x20, 0x02};
    const uint8_t r_open[8] = {0x12, 0x91, 0x50, 0x06, 0xCE, 0xCA, 0x20, 0x02};
    expect(FSD_SIG_DOOR_RR_LATCH, 0x103u, r_shut, 8, 2, "right rear shut");
    expect(FSD_SIG_DOOR_RR_LATCH, 0x103u, r_open, 8, 1, "right rear open");

    // 기어D-A1: byte2 0x32 -> 0x92 fourteen milliseconds after TSL's stalk
    // frame. 1 is P and 4 is D, which is the same numbering TSL put in the
    // command nibble.
    uint8_t drv[8] = {0xF6, 0x63, 0x32, 0x00, 0x00, 0x48, 0x00, 0x00};
    expect(FSD_SIG_GEAR, 0x118u, drv, 8, 1, "6.003 park");
    drv[2] = 0x92;
    expect(FSD_SIG_GEAR, 0x118u, drv, 8, 4, "6.454 drive");
    drv[2] = 0x95;
    expect(FSD_SIG_GEAR, 0x118u, drv, 8, 4, "6.463 still drive, other bits settled");
}

/* ── the table itself ────────────────────────────────────────────────────── */
static void test_table_is_well_formed(void) {
    for (int s = 0; s < FSD_SIG_COUNT; s++) {
        const FsdSignalDef *d = fsd_signal_def((FsdSignal)s);
        CHECK(d != NULL, "signal %d has a row", s);
        if (!d) continue;
        CHECK(d->signal == (FsdSignal)s, "row %d knows its index", s);
        CHECK(d->name && d->name[0], "signal %d has a name", s);
        CHECK(d->bit_len >= 1 && d->bit_len <= 32, "signal %d has a width", s);
        CHECK(d->can_id != 0, "signal %d names a frame", s);
        // Only the accumulating one is signed. Reading a switch as signed would
        // turn a two-bit field of value 2 into a value of -2.
        CHECK((d->kind == FSD_SIGK_DELTA) == (s == FSD_SIG_SCROLL_TICKS),
              "signal %d: exactly one delta signal", s);
    }
    CHECK(fsd_signal_def(FSD_SIG_COUNT) == NULL, "out of range is NULL");

    // Names are what the app shows the owner, so two signals sharing one is a
    // rule pointing at something the owner cannot tell apart.
    for (int a = 0; a < FSD_SIG_COUNT; a++)
        for (int b = a + 1; b < FSD_SIG_COUNT; b++)
            CHECK(strcmp(fsd_signal_def((FsdSignal)a)->name, fsd_signal_def((FsdSignal)b)->name),
                  "signals %d and %d share a name", a, b);

    for (int v = 0; v <= FSD_SIGV_BAD_ARGS; v++)
        CHECK(fsd_signal_verdict_str((FsdSignalVerdict)v)[0] != '?', "verdict %d is named", v);
}

static void test_refuses_rather_than_guesses(void) {
    const uint8_t f[8] = {0x00, 0x55, 0x55, 0x55, 0x02, 0x00, 0x69, 0x85};
    int32_t v = 0;

    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x102u, f, 8, &v) == FSD_SIGV_WRONG_ID,
          "wrong frame");
    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x3C2u, f, 4, &v) == FSD_SIGV_SHORT,
          "a frame too short to hold the field");
    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x3C2u, NULL, 8, &v) == FSD_SIGV_BAD_ARGS,
          "NULL data");
    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x3C2u, f, 8, NULL) == FSD_SIGV_BAD_ARGS,
          "NULL out");
    CHECK(fsd_signal_extract(FSD_SIG_COUNT, 0x3C2u, f, 8, &v) == FSD_SIGV_UNKNOWN,
          "out of range");

    // A zero-length frame is short for everything, including the mux read that
    // happens before the length check.
    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x3C2u, f, 0, &v) == FSD_SIGV_SHORT,
          "an empty frame carries nothing");
}


/* ── 0x3C2 byte 0: the buttons that share a byte with the selector ──────────
 *
 * Every frame below is copied out of a capture. These four byte-0 values are
 * the only ones this car was seen to send on 0x3C2 mux 0 across three visits.
 */
static void test_byte0_buttons(void) {
    /* 4차 경적두번 (2026-09-05): three presses of 60-130 ms. */
    const uint8_t horn[8]  = {0x04, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};
    /* 3차 좌우비상등 (2026-09-05). */
    const uint8_t hazbtn[8]= {0x08, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};
    /* 3차 후진: the owner was in the seat and driving. */
    const uint8_t seated[8]= {0x10, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};
    /* Nothing pressed. */
    const uint8_t idle[8]  = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};

    expect(FSD_SIG_HORN_SW, 0x3C2u, horn, 8, 1, "horn pressed");
    expect(FSD_SIG_HORN_SW, 0x3C2u, idle, 8, 0, "horn released");
    expect(FSD_SIG_HAZARD_BTN, 0x3C2u, hazbtn, 8, 1, "hazard button pressed");
    expect(FSD_SIG_HAZARD_BTN, 0x3C2u, idle, 8, 0, "hazard button released");
    expect(FSD_SIG_DRIVER_PRESENT, 0x3C2u, seated, 8, 1, "driver present");
    expect(FSD_SIG_DRIVER_PRESENT, 0x3C2u, idle, 8, 0, "seat empty");

    /* One button at a time: each frame must set its own signal and no other. */
    expect(FSD_SIG_HAZARD_BTN, 0x3C2u, horn, 8, 0, "horn frame: hazard button clear");
    expect(FSD_SIG_DRIVER_PRESENT, 0x3C2u, horn, 8, 0, "horn frame: seat clear");
    expect(FSD_SIG_HORN_SW, 0x3C2u, hazbtn, 8, 0, "hazard frame: horn clear");
    expect(FSD_SIG_HORN_SW, 0x3C2u, seated, 8, 0, "seated frame: horn clear");
}

/* 🔴 THE ONE THAT WAS BROKEN, AND THE REASON THE MASK CHANGED.
 *
 * The selector lives in byte 0 and so do those three buttons. A six-bit mask
 * over that byte swallowed two of them: while the horn or the hazard button was
 * held, every mux-0 signal came back WRONG_MUX -- all eight windows, the belt,
 * driver-present. Nothing logged it and no test looked.
 */
static void test_a_pressed_button_does_not_hide_the_rest_of_the_frame(void) {
    /* Same frame twice: the second one has the horn down. Byte 4 = 0x02 is the
     * driver's window at its second detent, from 4차 운전석창문올림. */
    const uint8_t quiet[8] = {0x00, 0x55, 0x55, 0x55, 0x02, 0x00, 0x69, 0x85};
    const uint8_t honking[8] = {0x04, 0x55, 0x55, 0x55, 0x02, 0x00, 0x69, 0x85};
    const uint8_t hazarding[8] = {0x08, 0x55, 0x55, 0x55, 0x02, 0x00, 0x69, 0x85};
    const uint8_t seated[8] = {0x10, 0x55, 0x55, 0x55, 0x02, 0x00, 0x69, 0x85};

    expect(FSD_SIG_WIN_UP_FL, 0x3C2u, quiet, 8, 2, "window up, nothing else pressed");
    expect(FSD_SIG_WIN_UP_FL, 0x3C2u, honking, 8, 2, "window up while the horn is down");
    expect(FSD_SIG_WIN_UP_FL, 0x3C2u, hazarding, 8, 2,
           "window up while the hazard button is down");
    expect(FSD_SIG_WIN_UP_FL, 0x3C2u, seated, 8, 2, "window up with a driver in the seat");

    /* 🔴 And the signal a pressed button carries must be readable in the very
     * frame that carries it -- otherwise setting the bit is what stops it being
     * read, which is a signal that can never be true. */
    expect(FSD_SIG_HORN_SW, 0x3C2u, honking, 8, 1, "the horn frame reports the horn");
    expect(FSD_SIG_HAZARD_BTN, 0x3C2u, hazarding, 8, 1,
           "the hazard frame reports the hazard button");

    /* The scroll frame still is not the switch pack, and vice versa. Narrowing
     * the mask must not make the two multiplexes interchangeable. */
    const uint8_t scroll[8] = {0x29, 0x55, 0x00, 0x05, 0x00, 0x00, 0x00, 0x80};
    int32_t v = 0x5A5A;
    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x3C2u, scroll, 8, &v) == FSD_SIGV_WRONG_MUX,
          "a window switch is still not in the scroll frame");
    CHECK(v == 0x5A5A, "and the refusal still writes nothing");
    CHECK(fsd_signal_extract(FSD_SIG_HORN_SW, 0x3C2u, scroll, 8, &v) == FSD_SIGV_WRONG_MUX,
          "the horn is not in the scroll frame either");
    expect(FSD_SIG_SCROLL_TICKS, 0x3C2u, scroll, 8, 5, "and the scroll frame still reads");

    /* ⚠️ TWO BITS, NOT ONE. Our captures cannot tell those apart on their own
     * — only indexes 0 and 1 were ever seen, and a 1-bit mask partitions those
     * two identically. The width comes from opendbc, which declares
     *
     *      VCLEFT_switchStatusIndex M: 0|2   with  2 = INVALID
     *
     * so index 2 exists and means "do not read this frame". At one bit it
     * would fold into index 0 and we would read the switch pack out of a frame
     * the car has marked unusable.
     *
     * That is not a capture, so it is written as the behaviour it implies
     * rather than as a value copied out of one. */
    const uint8_t invalid[8] = {0x02, 0x55, 0x55, 0x55, 0x02, 0x00, 0x69, 0x85};
    v = 0x5A5A;
    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x3C2u, invalid, 8, &v) == FSD_SIGV_WRONG_MUX,
          "index 2 is INVALID and must not be read as the switch pack");
    CHECK(v == 0x5A5A, "and it writes nothing");
    CHECK(fsd_signal_extract(FSD_SIG_HORN_SW, 0x3C2u, invalid, 8, &v) == FSD_SIGV_WRONG_MUX,
          "nor as a button");
    CHECK(fsd_signal_extract(FSD_SIG_SCROLL_TICKS, 0x3C2u, invalid, 8, &v) == FSD_SIGV_WRONG_MUX,
          "nor as the scroll wheel");
    const uint8_t index3[8] = {0x03, 0x55, 0x55, 0x55, 0x02, 0x00, 0x69, 0x85};
    CHECK(fsd_signal_extract(FSD_SIG_WIN_UP_FL, 0x3C2u, index3, 8, &v) == FSD_SIGV_WRONG_MUX,
          "index 3 has no meaning at all and must not be read");
    CHECK(fsd_signal_extract(FSD_SIG_SCROLL_TICKS, 0x3C2u, index3, 8, &v) == FSD_SIGV_WRONG_MUX,
          "index 3 is not the scroll frame");
}

/* ── 0x249: the turn stalk ───────────────────────────────────────────────────
 *
 * 🔴 One bit left of where opendbc puts it. These are the byte-2 values the
 * capture holds, and the value table they have to line up with:
 *
 *      1 UP_1   2 UP_2   3 DOWN_1   4 DOWN_2
 */
static void test_turn_stalk_is_one_bit_left_of_the_dbc(void) {
    /* 4차 우깜빡이 (by hand): 0x02 twice, then 0x04, then back to 0. */
    const uint8_t up1[4]  = {0x9B, 0x00, 0x02, 0x00};
    const uint8_t up2[4]  = {0xA3, 0x00, 0x04, 0x00};
    /* 4차 좌깜빡이 (by hand): 0x06 twice, then 0x08. */
    const uint8_t dn1[4]  = {0x5E, 0x09, 0x06, 0x00};
    const uint8_t dn2[4]  = {0x92, 0x0A, 0x08, 0x00};
    const uint8_t idle[4] = {0x5E, 0x09, 0x00, 0x00};

    expect(FSD_SIG_TURN_STALK, 0x249u, idle, 4, 0, "stalk idle");
    expect(FSD_SIG_TURN_STALK, 0x249u, up1, 4, 1, "0x02 -> UP_1");
    expect(FSD_SIG_TURN_STALK, 0x249u, up2, 4, 2, "0x04 -> UP_2 (right indicator)");
    expect(FSD_SIG_TURN_STALK, 0x249u, dn1, 4, 3, "0x06 -> DOWN_1");
    expect(FSD_SIG_TURN_STALK, 0x249u, dn2, 4, 4, "0x08 -> DOWN_2 (left indicator)");

    /* 🔴 The whole point, said as an assertion: at the DBC's 16|3 the left
     * indicator's own frame would read as IDLE -- 0x08 & 0x07 == 0 -- at the
     * exact moment 0x3F5 says the left lamp came on. */
    CHECK((dn2[2] & 0x07u) == 0u,
          "at 16|3 the left indicator reads as idle, which is why 17|3");
    CHECK(((dn2[2] >> 1) & 0x07u) == 4u, "and at 17|3 it reads as DOWN_2");

    /* Four distinct values, so no two stalk positions collide. */
    int32_t seen[4] = {0, 0, 0, 0};
    const uint8_t* frames[4] = {up1, up2, dn1, dn2};
    for (int i = 0; i < 4; i++)
        fsd_signal_extract(FSD_SIG_TURN_STALK, 0x249u, frames[i], 4, &seen[i]);
    for (int a = 0; a < 4; a++)
        for (int b = a + 1; b < 4; b++)
            CHECK(seen[a] != seen[b], "stalk values %d and %d collide at %ld", a, b,
                  (long)seen[a]);

    /* The frame is four bytes on this car, and byte 2 is the last one we need. */
    int32_t v = 0;
    CHECK(fsd_signal_extract(FSD_SIG_TURN_STALK, 0x249u, dn2, 2, &v) == FSD_SIGV_SHORT,
          "two bytes cannot hold byte 2");
}

/* ── 0x3F5 byte 0 bit 4: the hazards, and the bit that does not blink ───────*/
static void test_hazards_do_not_blink_on_bit_4(void) {
    /* By hand, 3차 좌우비상등: both phases. */
    const uint8_t hand_a[8] = {0x1A, 0xC8, 0x44, 0x39, 0x80, 0x0C, 0x28, 0x00};
    const uint8_t hand_b[8] = {0x15, 0xC8, 0x44, 0x39, 0x80, 0x0C, 0x28, 0x00};
    /* By the commercial device, 3차 후진: both phases, different byte 0. */
    const uint8_t auto_a[8] = {0x7A, 0xC8, 0x44, 0x39, 0x80, 0x0C, 0x28, 0x00};
    const uint8_t auto_b[8] = {0x75, 0xC8, 0x44, 0x39, 0x80, 0x0C, 0x28, 0x00};
    /* 4차 좌깜빡이 / 우깜빡이: an indicator on, hazards off. */
    const uint8_t left_a[8]  = {0x02, 0x00, 0x0B, 0x38, 0x80, 0x0D, 0x08, 0x00};
    const uint8_t left_b[8]  = {0x01, 0x00, 0x0B, 0x38, 0x80, 0x0D, 0x04, 0x00};
    const uint8_t right_a[8] = {0x08, 0x00, 0x0B, 0x38, 0x80, 0x0E, 0x20, 0x00};
    const uint8_t off[8]     = {0x00, 0x00, 0x0B, 0x38, 0x80, 0x0E, 0x00, 0x00};

    expect(FSD_SIG_HAZARD_ON, 0x3F5u, hand_a, 8, 1, "hazards, lit phase");
    expect(FSD_SIG_HAZARD_ON, 0x3F5u, hand_b, 8, 1, "hazards, dark phase");
    expect(FSD_SIG_HAZARD_ON, 0x3F5u, auto_a, 8, 1, "hazards by the device, lit");
    expect(FSD_SIG_HAZARD_ON, 0x3F5u, auto_b, 8, 1, "hazards by the device, dark");

    /* 🟢 That is the finding: the bit held through both phases, in both
     * sources. A rule watching it fires once, not three times a second. */

    expect(FSD_SIG_HAZARD_ON, 0x3F5u, left_a, 8, 0, "left indicator is not hazards");
    expect(FSD_SIG_HAZARD_ON, 0x3F5u, left_b, 8, 0, "left indicator, dark phase");
    expect(FSD_SIG_HAZARD_ON, 0x3F5u, right_a, 8, 0, "right indicator is not hazards");
    expect(FSD_SIG_HAZARD_ON, 0x3F5u, off, 8, 0, "nothing on");
}

int main(void) {
    printf("test_signal\n");
    test_map_light_walk();
    test_window_walk();
    test_belt_and_driver();
    test_scroll_accumulates();
    test_multiplex_is_not_guessed();
    test_byte0_buttons();
    test_a_pressed_button_does_not_hide_the_rest_of_the_frame();
    test_turn_stalk_is_one_bit_left_of_the_dbc();
    test_hazards_do_not_blink_on_bit_4();
    test_doors_and_gear();
    test_table_is_well_formed();
    test_refuses_rather_than_guesses();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
