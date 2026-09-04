/*
 * test_wire.c — host tests for fsd_logic/fsd_wire.c, and the generator for the
 * byte vectors the Android app is tested against.
 *
 * TWO JOBS, AND THE SECOND IS THE POINT
 * -------------------------------------
 * 1. Assert the packers. Until now nothing did: they lived inside
 *    ble_server.cpp, which cannot be compiled on a host, so every clamp and
 *    scale factor the phone depends on was unverified on both sides.
 *
 * 2. Write test/fixtures/wire_vectors.json. The app's JVM tests parse the SAME
 *    file and assert they recover the same field values. That makes the two
 *    implementations check EACH OTHER rather than both checking the prose in
 *    BLE-GATT-프로토콜.md.
 *
 * Which matters because the wire has already moved twice — State byte 9 went
 * from cruise state to gear, and CamStat went from 12 bytes to 20. An app
 * written from the document would have gone quietly wrong both times, and
 * "quietly" is the part that costs a day of debugging in a car park.
 *
 * The fixture is REGENERATED on every run, so a change to a packer shows up as a
 * diff in review rather than as a surprise in the app. The firmware is the
 * source of truth; if the app disagrees with this file, the app is wrong.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>
#include "../fsd_logic/fsd_rules.h"

#include "fsd_wire.h"

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

static uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

// ── State ────────────────────────────────────────────────────────────────────

static void test_profile_sentinel(void) {
    /* 🔴 0 is a real speed profile on this car (컴포트), so "never decoded"
     * cannot be carried by a zero -- the phone would draw a confident wrong
     * name for a car it never heard from. The sentinel must sit outside the
     * 3-bit field the decoder produces. */
    CHECK(FSD_WIRE_PROFILE_NONE > FSD_PROFILE_MASK,
          "sentinel 0x%02X must not be a real profile (mask 0x%02X)",
          FSD_WIRE_PROFILE_NONE, FSD_PROFILE_MASK);
    CHECK(FSD_WIRE_STATE_LEN == 28u, "State is 28 bytes in v7");
    CHECK(FSD_WIRE_STATE_VERSION == 7u, "version bumped with the length");
}

/* 🔴 The reverse-speed defect, pinned as an equation rather than a story.
 *
 * The owner reported "no speed shown in reverse" after the first drive
 * (2026-09-03). The cause, measured 2026-09-05: DI_vehicleSpeed goes NEGATIVE
 * below a standstill -- raw 0x1F4 is 0 km/h, and reversing ran it down to 455
 * (= -3.6 km/h) -- and the packer clamps negatives to zero, correctly, because
 * the field it feeds is unsigned.
 *
 * DI_uiSpeed, the number on the car's own display, read 0,1,2,3 through the
 * same seconds. So the fix is not to unclamp anything; it is to carry the other
 * signal, which is what a speedometer wanted all along. */
static void test_reverse_speed_needs_the_other_signal(void) {
    printf("\n-- reverse: the clamped one and the one that works --\n");

    FsdWireState w;
    memset(&w, 0, sizeof(w));
    uint8_t b[FSD_WIRE_STATE_LEN];

    /* Reversing at 3.6 km/h, as the car reported it. */
    w.speed_kph = -3.6f;
    w.ui_speed_seen = true;
    w.ui_speed = 3;
    fsd_wire_pack_state(&w, b);

    const uint16_t x10 = (uint16_t)(b[6] | ((uint16_t)b[7] << 8));
    CHECK(x10 == 0, "the km/h field still clamps -- it is unsigned, got %u", x10);
    CHECK(b[27] == 3, "and the display value survives, got %u", b[27]);

    /* Forward, both agree in magnitude. The point is that they are two fields,
     * not that they always differ. */
    w.speed_kph = 42.0f;
    w.ui_speed = 42;
    fsd_wire_pack_state(&w, b);
    CHECK((uint16_t)(b[6] | ((uint16_t)b[7] << 8)) == 420, "42.0 km/h");
    CHECK(b[27] == 42, "and 42 on the display");

    /* 🔴 They are NOT the same number in general. An mph car reports mph in
     * byte 27, and the ratio between the two is the only thing on this bus that
     * says which unit the car is set to. Merging them would destroy that. */
    w.speed_kph = 100.0f; /* km/h */
    w.ui_speed = 62;      /* the same speed shown in mph */
    fsd_wire_pack_state(&w, b);
    CHECK((uint16_t)(b[6] | ((uint16_t)b[7] << 8)) == 1000, "100.0 km/h");
    CHECK(b[27] == 62, "62 mph on the display -- a legal disagreement");
}

static void test_state_layout(void) {
    printf("\n-- State: every byte, from the spec not the code --\n");

    FsdWireState w;
    memset(&w, 0, sizeof(w));
    w.rx_seen = true;
    w.blinker_right = true;
    /* Lamp on the lit half of its cycle. Set here rather than in its own test
     * so the round-trip vector carries a non-zero byte 20 -- a field that is
     * always zero in every vector is a field nobody notices losing. */
    w.blinker_right_blinking = 2u;
    /* A car on the right, close enough that changing lanes is the wrong move.
     * Set here so the round-trip vector carries a non-zero byte 21 -- a field
     * that is zero in every vector is a field nobody notices losing. */
    w.blind_spot_right = 2u;
    /* 42, 42, 41, 43 psi in counts of 0.025 bar. Real-ish and all different,
     * so a packer that wrote one wheel four times would fail here. */
    w.tyre_pressure[0] = 116u;
    w.tyre_pressure[1] = 116u;
    w.tyre_pressure[2] = 113u;
    w.tyre_pressure[3] = 119u;
    w.brake_applied = true;
    w.blackbox_recording = true;
    w.op_mode = 1;      // Active
    w.hw_version = 2;   // HW3
    w.speed_profile = 2;
    w.ap_state = 6;
    w.speed_kph = 63.4f;
    w.soc_percent = 78.0f;
    w.gear = 4; // D
    w.speed_limit_seen = true;
    w.speed_limit_kph = 60.0f;
    w.rx_fps = 1200;
    w.crc_err_count = 3;
    w.uptime_s = 86400;

    uint8_t b[FSD_WIRE_STATE_LEN];
    fsd_wire_pack_state(&w, b);

    /* 🔴 숫자를 손으로 적지 않는다. v5 -> v6 때 이 줄만 5 로 남아
     * CI 에서 터졌다 — 로컬은 헤더가 바뀌어도 test_wire 를 다시 안 굽는다. */
    CHECK(b[0] == FSD_WIRE_STATE_VERSION,
          "ver %u, got %u", FSD_WIRE_STATE_VERSION, b[0]);
    // bit0 rx, bit3 right blinker, bit4 recording, bit6 brake = 0x59
    CHECK(b[1] == 0x59u, "flags 0x59, got 0x%02X", b[1]);
    CHECK(b[2] == 1, "op_mode");
    CHECK(b[3] == 2, "hw");
    CHECK(b[4] == 2, "profile");
    CHECK(b[5] == 6, "ap_state");
    CHECK(le16(&b[6]) == 634u, "speed x10 = 634, got %u", le16(&b[6]));
    CHECK(b[8] == 78, "soc");
    CHECK(b[9] == 4, "gear D");
    CHECK(le16(&b[10]) == 60u, "speed limit");
    CHECK(le16(&b[12]) == 1200u, "rx_fps");
    CHECK(le16(&b[14]) == 3u, "crc errors");
    CHECK(le32(&b[16]) == 86400u, "uptime");
    /* Lamp phase, added in v3. Right blinking-lit (2), left off (0). */
    CHECK(b[20] == 0x08u, "blink byte 0x08, got 0x%02X", b[20]);
    /* Blind spot, added in v4. Right at level 2, left clear. */
    CHECK(b[21] == 0x08u, "blind spot byte 0x08, got 0x%02X", b[21]);
    /* Tyres, added in v5. Order matters -- a display puts these in a square. */
    CHECK(b[22] == 116u && b[23] == 116u && b[24] == 113u && b[25] == 119u,
          "tyres %u %u %u %u", b[22], b[23], b[24], b[25]);
}

/* Each wheel lands in its own byte, in order.
 *
 * Worth its own test because the failure is silent and specific: a loop that
 * writes the wrong index puts a real pressure under the wrong wheel, and four
 * plausible numbers in a square look right whichever way they are shuffled.
 * Nobody spots that by looking. */
static void test_state_tyre_pressure(void) {
    printf("''' + NL + '''-- State: four tyres, four bytes, in order --''' + NL + '''");

    for(size_t w = 0; w < 4; w++) {
        FsdWireState in = {0};
        in.tyre_pressure[w] = (uint8_t)(100u + w);
        uint8_t b[FSD_WIRE_STATE_LEN];
        fsd_wire_pack_state(&in, b);
        for(size_t i = 0; i < 4; i++) {
            const uint8_t want = (i == w) ? (uint8_t)(100u + w) : 0u;
            CHECK(b[22 + i] == want, "wheel %u -> byte %u = %u, got %u",
                  (unsigned)w, (unsigned)(22 + i), want, b[22 + i]);
        }
        /* And it must not disturb the bytes beside it. */
        CHECK(b[21] == 0u, "wheel %u leaked into byte 21", (unsigned)w);
    }
}

/* A pressure expires, on a much longer clock than the speed limit.
 *
 * Written because "the sensor is asleep" and "the sensor has stopped" look
 * identical from here, and the two want opposite answers. Sixty seconds is
 * long enough for the first and short enough for the second. */
static void test_tyre_freshness(void) {
    printf("''' + NL + '''-- State: a tyre reading stops being believable --''' + NL + '''");

    CHECK(!fsd_tyre_fresh(false, 1000u, 1000u), "never seen is never fresh");
    CHECK(fsd_tyre_fresh(true, 1000u, 1000u), "same instant is fresh");
    CHECK(fsd_tyre_fresh(true, 1000u, 1000u + FSD_TYRE_MAX_AGE_MS - 1u),
          "just inside the window is fresh");
    CHECK(!fsd_tyre_fresh(true, 1000u, 1000u + FSD_TYRE_MAX_AGE_MS),
          "exactly at the window is stale");
    /* A sensor that reports every few seconds must never blink out. */
    CHECK(fsd_tyre_fresh(true, 1000u, 1000u + 10000u), "ten seconds is still fresh");
    /* Same wrap and backwards-clock rules as the speed limit. */
    CHECK(fsd_tyre_fresh(true, 0xFFFFF000u, 0xFFFFF000u + 500u), "across the wrap");
    CHECK(!fsd_tyre_fresh(true, 5000u, 1000u), "backwards clock is stale");
}

/* The two 2-bit blind spot fields tile byte 21 without bleeding into each
 * other, and out-of-range values are masked rather than trusted.
 *
 * Worth its own test because this is the one field on the dashboard that
 * OVERRIDES another: it takes the whole side bar away from the turn signal.
 * A left value leaking into the right half would put a warning on the wrong
 * side of the car, which is worse than no warning at all. */
static void test_state_blind_spot(void) {
    printf("''' + NL + '''-- State: blind spot, two bits a side --''' + NL + '''");

    struct {
        uint8_t l, r, want;
    } cases[] = {
        {0u, 0u, 0x00u},   {1u, 0u, 0x01u},   {2u, 0u, 0x02u},   {3u, 0u, 0x03u},
        {0u, 1u, 0x04u},   {0u, 2u, 0x08u},   {0u, 3u, 0x0Cu},
        {1u, 2u, 0x09u},   {2u, 1u, 0x06u},   {3u, 3u, 0x0Fu},
        /* Out of range on either side must not spill past its own two bits. */
        {0xFFu, 0u, 0x03u}, {0u, 0xFFu, 0x0Cu}, {0xFFu, 0xFFu, 0x0Fu},
    };
    for(size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        FsdWireState w = {0};
        w.blind_spot_left = cases[i].l;
        w.blind_spot_right = cases[i].r;
        uint8_t b[FSD_WIRE_STATE_LEN];
        fsd_wire_pack_state(&w, b);
        CHECK(b[21] == cases[i].want, "l=%u r=%u -> 0x%02X, got 0x%02X",
              cases[i].l, cases[i].r, cases[i].want, b[21]);
        /* And it must not disturb the byte beside it. */
        CHECK(b[20] == 0x00u, "case %u leaked into byte 20: 0x%02X",
              (unsigned)i, b[20]);
    }
}

/* The two 2-bit lamp fields must tile byte 20 without bleeding into each
 * other. Written because the two halves came from DBC entries that disagree
 * with themselves -- one big-endian with range [0|3], the other little-endian
 * with range [0|15] -- so "left leaks into right" is a live failure mode, not
 * a theoretical one. Values above 3 are masked rather than trusted. */
/* Where the speed limit came from rides in the same byte as the lamp phase.
 * Two things must hold and they pull in opposite directions: the source must
 * survive next to the lamp bits, and it must NOT be sent when there is no
 * limit to label. A source on an empty box reads as "we know where this
 * nothing came from". */
static void test_state_speed_limit_source(void) {
    printf("''' + NL + '''-- State: speed limit source --''' + NL + '''");

    struct {
        bool seen;
        uint8_t src, lampL, lampR, want20;
        uint16_t want_limit;
    } cases[] = {
        /* seen, src, lampL, lampR -> byte20, limit */
        {true, 1u, 0u, 0u, 0x10u, 60u},   /* map    */
        {true, 2u, 0u, 0u, 0x20u, 60u},   /* vision */
        {true, 3u, 0u, 0u, 0x30u, 60u},   /* acc    */
        {true, 0u, 0u, 0u, 0x00u, 60u},   /* none   */
        /* Not seen: the limit goes out as 0 AND the source goes with it. */
        {false, 2u, 0u, 0u, 0x00u, 0u},
        /* Lamp bits and source share the byte without touching each other. */
        {true, 2u, 2u, 1u, 0x26u, 60u},
        {true, 3u, 3u, 3u, 0x3Fu, 60u},
        /* Out of range must not spill into the lamp bits below it. */
        {true, 0xFFu, 0u, 0u, 0x30u, 60u},
    };
    for(size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        FsdWireState w = {0};
        w.speed_limit_seen = cases[i].seen;
        w.speed_limit_kph = 60.0f;
        w.speed_limit_source = cases[i].src;
        w.blinker_left_blinking = cases[i].lampL;
        w.blinker_right_blinking = cases[i].lampR;
        uint8_t b[FSD_WIRE_STATE_LEN];
        fsd_wire_pack_state(&w, b);
        CHECK(b[20] == cases[i].want20, "case %u: byte20 0x%02X, got 0x%02X",
              (unsigned)i, cases[i].want20, b[20]);
        CHECK(le16(&b[10]) == cases[i].want_limit, "case %u: limit %u, got %u",
              (unsigned)i, cases[i].want_limit, le16(&b[10]));
    }
}

/* The limit expires. Written because it never did: speed_limit_seen only ever
 * went true and speed_limit_last_ms was stamped and never read, so a value
 * picked up half an hour ago sat on the dashboard as the road you are on. */
static void test_speed_limit_freshness(void) {
    printf("''' + NL + '''-- State: a speed limit stops being believable --''' + NL + '''");

    CHECK(!fsd_speed_limit_fresh(false, 1000u, 1000u), "never seen is never fresh");
    CHECK(fsd_speed_limit_fresh(true, 1000u, 1000u), "same instant is fresh");
    CHECK(fsd_speed_limit_fresh(true, 1000u, 1000u + FSD_SPEED_LIMIT_MAX_AGE_MS - 1u),
          "just inside the window is fresh");
    CHECK(!fsd_speed_limit_fresh(true, 1000u, 1000u + FSD_SPEED_LIMIT_MAX_AGE_MS),
          "exactly at the window is stale");
    CHECK(!fsd_speed_limit_fresh(true, 1000u, 1000u + 1800000u), "half an hour is stale");

    /* millis() wraps every ~49 days. Unsigned subtraction crosses the wrap
     * correctly, so a limit seen just before it stays fresh just after. */
    CHECK(fsd_speed_limit_fresh(true, 0xFFFFF000u, 0xFFFFF000u + 500u),
          "across the millis wrap, a recent limit stays fresh");
    /* And a clock that appears to run backwards reads as stale, not as
     * infinitely fresh. Stale is the safe direction. */
    CHECK(!fsd_speed_limit_fresh(true, 5000u, 1000u), "backwards clock is stale");
}

static void test_state_blink_nibble(void) {
    printf("\n-- State: the two lamp fields do not bleed into each other --\n");

    struct {
        uint8_t l, r, want;
    } cases[] = {
        {0u, 0u, 0x00u},   {1u, 0u, 0x01u},   {2u, 0u, 0x02u},
        {0u, 1u, 0x04u},   {0u, 2u, 0x08u},   {2u, 2u, 0x0Au},
        {1u, 2u, 0x09u},   {3u, 3u, 0x0Fu},
        /* Out of range on either side must not spill past its own two bits. */
        {0xFFu, 0u, 0x03u}, {0u, 0xFFu, 0x0Cu}, {0xFFu, 0xFFu, 0x0Fu},
    };
    for(size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        FsdWireState w = {0};
        w.blinker_left_blinking = cases[i].l;
        w.blinker_right_blinking = cases[i].r;
        uint8_t b[FSD_WIRE_STATE_LEN];
        fsd_wire_pack_state(&w, b);
        CHECK(b[20] == cases[i].want, "l=%u r=%u -> 0x%02X, got 0x%02X",
              cases[i].l, cases[i].r, cases[i].want, b[20]);
    }
}

static void test_state_structural_zeros(void) {
    printf("\n-- State: the bits that are always zero stay zero --\n");

    // Bits 5 (blind spot R) and 7 (profile change) have no input field at all,
    // deliberately: giving them one would invite someone to fill it, and
    // neither is extracted or emitted on this build. The app renders them as
    // unavailable, and this pins that they cannot be set by accident.
    //
    // Bit 4 used to be in this list. It now carries blackbox_recording, which is
    // why that line below is an assertion about the FIELD rather than about the
    // bit: this test failing is exactly what should happen when a spare bit
    // stops being spare, and it did.
    FsdWireState w;
    memset(&w, 0xFF, sizeof(w)); // every bool true, every number huge
    w.speed_kph = 0.0f;
    w.soc_percent = 0.0f;
    w.speed_limit_kph = 0.0f;
    w.speed_profile = 0;

    uint8_t b[FSD_WIRE_STATE_LEN];
    fsd_wire_pack_state(&w, b);
    /* 🔴 Bit 5 asserted "never set" until 2026-09-05. It carried nothing
     * because blind-spot is not extracted on this path; now it carries
     * ui_speed_seen, and the memset above made that true. The test does not go
     * away -- it becomes the assertion that the bit TRACKS ITS FIELD, which is
     * strictly more than "it is zero". */
    CHECK((b[1] & (1u << 5)) != 0, "ui_speed_seen set when the field is true");
    CHECK((b[1] & (1u << 7)) == 0, "profile-change never set");

    w.ui_speed_seen = false;
    fsd_wire_pack_state(&w, b);
    CHECK((b[1] & (1u << 5)) == 0, "and clear when it is false");
    CHECK(b[27] == 0, "a speed nobody saw is not sent");
    w.ui_speed_seen = true;

    // ...and bit 4 tracks its field in BOTH directions, so it can neither be
    // stuck on nor silently dropped.
    w.blackbox_recording = false;
    fsd_wire_pack_state(&w, b);
    CHECK((b[1] & (1u << 4)) == 0, "recording clear when not recording");
    w.blackbox_recording = true;
    fsd_wire_pack_state(&w, b);
    CHECK((b[1] & (1u << 4)) != 0, "recording set when recording");
}

static void test_state_clamps(void) {
    printf("\n-- State: clamps and saturation --\n");

    FsdWireState w;
    uint8_t b[FSD_WIRE_STATE_LEN];

    // A negative speed is a parser artefact, not a reversing car.
    memset(&w, 0, sizeof(w));
    w.speed_kph = -12.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[6]) == 0, "negative speed clamps to 0");

    // x10 overflows the field above 6553.5 km/h — impossible, but a garbage
    // parse must saturate rather than wrap into a plausible small number.
    memset(&w, 0, sizeof(w));
    w.speed_kph = 99999.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[6]) == 0xFFFFu, "absurd speed saturates, got %u", le16(&b[6]));

    memset(&w, 0, sizeof(w));
    w.soc_percent = 140.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(b[8] == 100, "soc clamps to 100, got %u", b[8]);
    w.soc_percent = -5.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(b[8] == 0, "soc clamps to 0");

    memset(&w, 0, sizeof(w));
    w.speed_profile = 9;
    fsd_wire_pack_state(&w, b);
    CHECK(b[4] == 3, "profile clamps to 3, got %u", b[4]);
    w.speed_profile = -4;
    fsd_wire_pack_state(&w, b);
    CHECK(b[4] == 0, "profile clamps to 0");

    memset(&w, 0, sizeof(w));
    w.crc_err_count = 70000u;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[14]) == 0xFFFFu, "crc count saturates");

    // An unseen limit is zero, never a stale value — the app must show blank
    // rather than a wrong number.
    memset(&w, 0, sizeof(w));
    w.speed_limit_seen = false;
    w.speed_limit_kph = 110.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[10]) == 0, "unseen limit is 0, got %u", le16(&b[10]));

    // Rounding, not truncation: 63.45 -> 635, not 634.
    memset(&w, 0, sizeof(w));
    w.speed_kph = 63.45f;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[6]) == 635u, "speed rounds, got %u", le16(&b[6]));
}

// ── CamStat ──────────────────────────────────────────────────────────────────

static void test_camstat_layout(void) {
    printf("\n-- CamStat: v2, and v1 still parses the first 12 --\n");

    FsdWireCamStat w;
    memset(&w, 0, sizeof(w));
    w.autonomy_enabled = true;
    w.db_loaded = true;
    w.learning_dirty = true;
    w.sup_verdict = 6; // BELT_UNLATCHED
    w.op_mode = 3;     // Autonomous
    w.camera_count = 8123;
    w.built_at = 1754870400u;
    w.gps_verdict = 7; // FROZEN
    w.pol_phase = 2;   // ACTIVE
    w.pol_action = 1;  // LOWER
    w.pol_target = 1;
    w.nearest_m = 143;
    w.gps_accuracy_raw = 15;
    w.raw_profile = 3;
    w.learned_count = 42;
    w.scan_full_count = 2;

    uint8_t b[FSD_WIRE_CAMSTAT_LEN];
    fsd_wire_pack_camstat(&w, b);

    CHECK(b[0] == 2, "ver 2");
    CHECK(b[1] == (0x01u | 0x04u | 0x20u), "flags 0x25, got 0x%02X", b[1]);
    CHECK(b[2] == 6, "sup_verdict");
    CHECK(b[3] == 3, "op_mode");
    CHECK(le32(&b[4]) == 8123u, "camera count");
    CHECK(le32(&b[8]) == 1754870400u, "built_at");
    CHECK(b[12] == 7, "gps_verdict");
    // phase b0-2 | action b3-4 | target b5-6  ->  2 | (1<<3) | (1<<5) = 0x2A
    CHECK(b[13] == 0x2Au, "packed phase/action/target 0x2A, got 0x%02X", b[13]);
    CHECK(le16(&b[14]) == 143u, "nearest metres");
    CHECK(b[16] == 15, "accuracy raw");
    CHECK(b[17] == 3, "raw profile");
    CHECK(b[18] == 42, "learned count");
    CHECK(b[19] == 2, "scan full count");
}

static void test_camstat_packing_cannot_bleed(void) {
    printf("\n-- CamStat: byte 13 fields cannot bleed into each other --\n");

    // Three fields share one byte. If any mask is wrong the neighbours corrupt,
    // and the app would show a plausible wrong phase. Drive each to its maximum
    // with the others also at maximum.
    FsdWireCamStat w;
    memset(&w, 0, sizeof(w));
    w.pol_phase = 7;
    w.pol_action = 3;
    w.pol_target = 3;

    uint8_t b[FSD_WIRE_CAMSTAT_LEN];
    fsd_wire_pack_camstat(&w, b);
    CHECK((b[13] & 0x07u) == 7, "phase survives");
    CHECK(((b[13] >> 3) & 0x03u) == 3, "action survives");
    CHECK(((b[13] >> 5) & 0x03u) == 3, "target survives");

    // Out-of-range inputs are masked, not allowed to overflow into the next
    // field: a policy that grew a sixth phase must not corrupt the action.
    memset(&w, 0, sizeof(w));
    w.pol_phase = 0xFFu;
    fsd_wire_pack_camstat(&w, b);
    CHECK(((b[13] >> 3) & 0x03u) == 0, "an over-range phase does not reach action");
    CHECK(((b[13] >> 5) & 0x03u) == 0, "nor target");
}

static void test_camstat_saturates(void) {
    printf("\n-- CamStat: counts that do not fit a byte --\n");

    FsdWireCamStat w;
    memset(&w, 0, sizeof(w));
    w.learned_count = 500;   // FSD_TRK_CAM_MAX is 128 today, but the field is 16-bit
    w.scan_full_count = 999;

    uint8_t b[FSD_WIRE_CAMSTAT_LEN];
    fsd_wire_pack_camstat(&w, b);
    CHECK(b[18] == 255, "learned count saturates, got %u", b[18]);
    CHECK(b[19] == 255, "scan full count saturates, got %u", b[19]);
}

static void test_result(void) {
    printf("\n-- Result --\n");

    uint8_t b[FSD_WIRE_RESULT_LEN];
    fsd_wire_pack_result(0x41u, 3u, 8123u, b); // SET_AUTONOMY, UNSUPPORTED
    CHECK(b[0] == 0x41u, "command echoed");
    CHECK(b[1] == 3u, "result code");
    CHECK(le16(&b[2]) == 8123u, "extra LE16");
}

static void test_null_safety(void) {
    printf("\n-- NULL --\n");

    uint8_t b[FSD_WIRE_STATE_LEN];
    memset(b, 0xAA, sizeof(b));
    fsd_wire_pack_state(NULL, b);
    CHECK(b[0] == 0xAAu, "NULL input leaves the buffer alone");
    FsdWireState w;
    memset(&w, 0, sizeof(w));
    fsd_wire_pack_state(&w, NULL); // must not crash
    fsd_wire_pack_camstat(NULL, b);
    fsd_wire_pack_result(1, 2, 3, NULL);
    CHECK(1, "NULL outputs do not crash");
}

// ── the fixture the app is tested against ────────────────────────────────────

static void emit_hex(FILE* f, const uint8_t* b, size_t n) {
    fputc('"', f);
    for(size_t i = 0; i < n; i++) fprintf(f, "%02x", b[i]);
    fputc('"', f);
}

/* One case per line-ish, with BOTH the inputs and the resulting bytes. The app
 * parses `hex` and asserts it recovers `fields` — so the assertion is about
 * agreement between two implementations, not about either one's idea of the
 * spec. */
static void emit_fixture(FILE* f) {
    fprintf(f, "{\n");
    fprintf(f, "  \"_generated_by\": \"test/test_wire.c — do not hand-edit\",\n");
    fprintf(f, "  \"state_version\": %u,\n", (unsigned)FSD_WIRE_STATE_VERSION);
    fprintf(f, "  \"camstat_version\": %u,\n", (unsigned)FSD_WIRE_CAMSTAT_VERSION);
    fprintf(f, "  \"state_len\": %u,\n", (unsigned)FSD_WIRE_STATE_LEN);
    fprintf(f, "  \"camstat_len\": %u,\n", (unsigned)FSD_WIRE_CAMSTAT_LEN);
    fprintf(f, "  \"state\": [\n");

    struct {
        const char* name;
        FsdWireState w;
    } states[] = {
        {"zeroed", {0}},
        {"driving_hw3",
         {.rx_seen = true, .blinker_right = true, .brake_applied = true, .op_mode = 1,
          .hw_version = 2, .speed_profile = 2, .ap_state = 6, .speed_kph = 63.4f,
          .soc_percent = 78.0f, .gear = 4, .speed_limit_seen = true,
          .speed_limit_kph = 60.0f, .rx_fps = 1200, .crc_err_count = 3,
          .uptime_s = 86400}},
        {"parked_listen_only",
         {.rx_seen = true, .op_mode = 0, .hw_version = 2, .gear = 1,
          .soc_percent = 55.5f, .rx_fps = 980, .uptime_s = 42}},
        {"autonomous_no_phone",
         {.rx_seen = true, .op_mode = 3, .hw_version = 2, .speed_profile = 1,
          .speed_kph = 88.8f, .soc_percent = 31.0f, .gear = 4,
          .speed_limit_seen = true, .speed_limit_kph = 100.0f, .rx_fps = 1150,
          .uptime_s = 3600}},
        {"clamped_extremes",
         {.speed_kph = 99999.0f, .soc_percent = 140.0f, .speed_profile = 9,
          .crc_err_count = 70000u, .uptime_s = 0xFFFFFFFFu}},
        {"ota_running", {.rx_seen = true, .ota_in_progress = true, .op_mode = 0,
                         .hw_version = 2, .gear = 1, .rx_fps = 1000}},
        /* The bit the phone reads before taking a capture that cannot be
         * retaken. Its own vector so a packer change that drops it fails on
         * both sides of the link rather than on neither. */
        {"recording", {.rx_seen = true, .blackbox_recording = true, .op_mode = 0,
                       .hw_version = 2, .gear = 1, .rx_fps = 1000, .uptime_s = 60}},
        /* Both lamps lit at once. What hazards would look like IF they set
         * these bits -- which is exactly what has not been measured yet. Its
         * own vector so the app's rendering of that case is checked against
         * bytes the packer really produced, whatever the car turns out to do. */
        {"both_lamps_lit",
         {.rx_seen = true, .blinker_left = true, .blinker_right = true,
          .blinker_left_blinking = 2u, .blinker_right_blinking = 2u,
          .op_mode = 0, .hw_version = 2, .gear = 1, .rx_fps = 1000,
          .uptime_s = 120}},
        /* The same signal on the dark half of the cycle. The pair is what
         * proves the app can tell "blinking but dark" from "off" -- one bit
         * could not have. */
        /* A limit with its provenance attached. Its own vector because the
         * number alone was never readable -- three frames write it. */
        {"limit_from_vision",
         {.rx_seen = true, .op_mode = 0, .hw_version = 2, .gear = 4,
          .speed_kph = 55.0f, .soc_percent = 70.0f, .speed_limit_seen = true,
          .speed_limit_kph = 80.0f, .speed_limit_source = 2u,
          .rx_fps = 1000, .uptime_s = 300}},
        /* A car on the left while the left blinker is on -- the moment the
         * override exists for. Its own vector so the app's handling of "both
         * at once" is checked against bytes the packer really produced. */
        {"blind_spot_left_while_signalling",
         {.rx_seen = true, .op_mode = 0, .hw_version = 2, .gear = 4,
          .speed_kph = 80.0f, .soc_percent = 65.0f,
          .blinker_left = true, .blinker_left_blinking = 2u,
          .blind_spot_left = 2u,
          .rx_fps = 1000, .uptime_s = 400}},
        /* Four tyres, one of them low. Its own vector because the app lays
         * these out in a square, and a square hides a shuffle. */
        {"tyres_one_low",
         {.rx_seen = true, .op_mode = 0, .hw_version = 2, .gear = 1,
          .soc_percent = 72.0f, .rx_fps = 1000, .uptime_s = 500,
          .tyre_pressure = {116u, 116u, 96u, 117u}}},
        {"both_lamps_dark",
         {.rx_seen = true, .blinker_left = true, .blinker_right = true,
          .blinker_left_blinking = 1u, .blinker_right_blinking = 1u,
          .op_mode = 0, .hw_version = 2, .gear = 1, .rx_fps = 1000,
          .uptime_s = 121}},
    };

    const size_t ns = sizeof(states) / sizeof(states[0]);
    for(size_t i = 0; i < ns; i++) {
        uint8_t b[FSD_WIRE_STATE_LEN];
        fsd_wire_pack_state(&states[i].w, b);
        const FsdWireState* w = &states[i].w;
        fprintf(f, "    { \"name\": \"%s\", \"hex\": ", states[i].name);
        emit_hex(f, b, sizeof(b));
        fprintf(f,
                ", \"fields\": { \"ver\": %u, \"flags\": %u, \"op_mode\": %u, "
                "\"hw\": %u, \"speed_profile\": %u, \"ap_state\": %u, "
                "\"speed_kph_x10\": %u, \"soc\": %u, \"gear\": %u, "
                "\"speed_limit\": %u, \"rx_fps\": %u, \"crc_err\": %u, "
                "\"uptime_s\": %u, \"blink_l\": %u, \"blink_r\": %u, \"limit_src\": %u, \"bs_l\": %u, \"bs_r\": %u, \"tyre0\": %u, \"tyre1\": %u, \"tyre2\": %u, \"tyre3\": %u } }%s\n",
                (unsigned)b[0], (unsigned)b[1], (unsigned)w->op_mode,
                (unsigned)w->hw_version, (unsigned)b[4], (unsigned)w->ap_state,
                (unsigned)le16(&b[6]), (unsigned)b[8], (unsigned)w->gear,
                (unsigned)le16(&b[10]), (unsigned)w->rx_fps,
                (unsigned)le16(&b[14]), (unsigned)le32(&b[16]),
                (unsigned)(b[20] & 0x03u), (unsigned)((b[20] >> 2) & 0x03u),
                (unsigned)((b[20] >> 4) & 0x03u),
                (unsigned)(b[21] & 0x03u), (unsigned)((b[21] >> 2) & 0x03u),
                (unsigned)b[22], (unsigned)b[23], (unsigned)b[24], (unsigned)b[25],
                (i + 1 < ns) ? "," : "");
    }
    fprintf(f, "  ],\n  \"camstat\": [\n");

    struct {
        const char* name;
        FsdWireCamStat w;
    } cams[] = {
        {"zeroed", {0}},
        {"no_gps_yet",
         {.autonomy_enabled = true, .db_loaded = true, .sup_verdict = 1,
          .op_mode = 3, .camera_count = 8123, .built_at = 1754870400u,
          .gps_verdict = 1, .nearest_m = 0xFFFFu, .gps_accuracy_raw = 0xFFu,
          .raw_profile = 0xFFu}},
        {"lowering_for_camera",
         {.autonomy_enabled = true, .supervised_ok = true, .db_loaded = true,
          .autonomy_allows = true, .learning_dirty = true, .profile_fresh = true,
          .sup_verdict = 0, .op_mode = 3, .camera_count = 8123,
          .built_at = 1754870400u, .gps_verdict = 0, .pol_phase = 2,
          .pol_action = 1, .pol_target = 1, .nearest_m = 143,
          .gps_accuracy_raw = 15, .raw_profile = 3, .learned_count = 42}},
        {"gps_frozen_in_tunnel",
         {.autonomy_enabled = true, .supervised_ok = true, .db_loaded = true,
          .autonomy_allows = true, .profile_fresh = true, .op_mode = 3,
          .camera_count = 8123, .gps_verdict = 7, .nearest_m = 0xFFFFu,
          .gps_accuracy_raw = 10, .raw_profile = 2}},
        {"save_failing",
         {.autonomy_enabled = true, .db_loaded = true, .learning_dirty = true,
          .save_failing = true, .sup_verdict = 6, .op_mode = 3,
          .gps_verdict = 6, .nearest_m = 0xFFFFu, .gps_accuracy_raw = 0xFFu,
          .raw_profile = 0xFFu, .learned_count = 500, .scan_full_count = 999}},
        {"field_maxima",
         {.pol_phase = 7, .pol_action = 3, .pol_target = 3, .camera_count = 0xFFFFFFFFu,
          .built_at = 0xFFFFFFFFu, .nearest_m = 0xFFFFu, .gps_accuracy_raw = 0xFFu,
          .raw_profile = 0xFFu}},
    };

    const size_t nc = sizeof(cams) / sizeof(cams[0]);
    for(size_t i = 0; i < nc; i++) {
        uint8_t b[FSD_WIRE_CAMSTAT_LEN];
        fsd_wire_pack_camstat(&cams[i].w, b);
        fprintf(f, "    { \"name\": \"%s\", \"hex\": ", cams[i].name);
        emit_hex(f, b, sizeof(b));
        fprintf(f,
                ", \"fields\": { \"ver\": %u, \"flags\": %u, \"sup_verdict\": %u, "
                "\"op_mode\": %u, \"camera_count\": %u, \"built_at\": %u, "
                "\"gps_verdict\": %u, \"pol_phase\": %u, \"pol_action\": %u, "
                "\"pol_target\": %u, \"nearest_m\": %u, \"gps_accuracy_raw\": %u, "
                "\"raw_profile\": %u, \"learned_count\": %u, "
                "\"scan_full_count\": %u } }%s\n",
                (unsigned)b[0], (unsigned)b[1], (unsigned)b[2], (unsigned)b[3],
                (unsigned)le32(&b[4]), (unsigned)le32(&b[8]), (unsigned)b[12],
                (unsigned)(b[13] & 0x07u), (unsigned)((b[13] >> 3) & 0x03u),
                (unsigned)((b[13] >> 5) & 0x03u), (unsigned)le16(&b[14]),
                (unsigned)b[16], (unsigned)b[17], (unsigned)b[18], (unsigned)b[19],
                (i + 1 < nc) ? "," : "");
    }
    fprintf(f, "  ],\n  \"result\": [\n");

    struct { const char* name; uint8_t cmd, res; uint16_t extra; } res[] = {
        {"set_mode_ok", 0x01u, 0u, 0u},
        {"set_profile_unsupported", 0x10u, 3u, 0u},
        {"dump_not_found", 0x30u, 5u, 0u},
        {"ping_ok_extra", 0x50u, 0u, 65535u},
    };
    const size_t nr = sizeof(res) / sizeof(res[0]);
    for(size_t i = 0; i < nr; i++) {
        uint8_t b[FSD_WIRE_RESULT_LEN];
        fsd_wire_pack_result(res[i].cmd, res[i].res, res[i].extra, b);
        fprintf(f, "    { \"name\": \"%s\", \"hex\": ", res[i].name);
        emit_hex(f, b, sizeof(b));
        fprintf(f, ", \"fields\": { \"cmd\": %u, \"result\": %u, \"extra\": %u } }%s\n",
                (unsigned)res[i].cmd, (unsigned)res[i].res, (unsigned)res[i].extra,
                (i + 1 < nr) ? "," : "");
    }
    /* -- rules ---------------------------------------------------------------
     *
     * 🔴 These did not exist, and three comments said they *could* not -- the
     * stated reason was that the firmware had no rule packer. It has one
     * (`fsd_rule_pack`), and without vectors the 12-byte layout is written
     * twice from the same prose: once in `fsd_rules.c`, once in `Rules.kt`.
     * That is exactly the pair this fixture exists to hold together.
     *
     * The cases are picked for what a hand-written codec gets wrong: a
     * negative `value` (scroll down) and a large negative `arg` prove the
     * int32 is little-endian two-s-complement in both languages; `enabled`
     * false with every other field set proves byte 0 is a flag and not a
     * length; a filled disabled row proves nothing is zeroed on the way out. */
    fprintf(f, "  ],\n  \"rule_len\": %u,\n  \"rule_max\": %u,\n  \"rules\": [\n",
            (unsigned)FSD_RULE_WIRE_LEN, (unsigned)FSD_RULE_MAX);
    static const struct {
        const char* name;
        FsdRule r;
    } rules[] = {
        /* What `fsd_rules_init` writes. Not all-zero: signal 0 and action 0 are
         * real values, so "untouched" is a shape, not a blank. The app has to
         * agree on it byte for byte or half-built rules vanish from its list. */
        {"empty", {false, FSD_SIG_MAP_SW_FL, FSD_TRIG_NONE, 0, FSD_ACT_MAP_LIGHT, 0}},
        {"map_light_press",
         {true, FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, FSD_ACT_MAP_LIGHT, 0}},
        /* Negative value AND negative arg: the two int32s are packed by the
         * same helper, so a sign bug that only shows on one of them would mean
         * the helper is being called differently in the two places. */
        {"scroll_down",
         {true, FSD_SIG_SCROLL_TICKS, FSD_TRIG_DELTA, -1, FSD_ACT_SCROLL, -3}},
        {"door_state_enter",
         {true, FSD_SIG_DOOR_FL_LATCH, FSD_TRIG_STATE_ENTER, 1, FSD_ACT_MAP_LIGHT, 0}},
        /* Every field set while `enabled` is false. Byte 0 is a flag field;
         * anything that treats it as a length or that clears a disabled row on
         * the way out fails here rather than in the car. */
        {"disabled_but_filled",
         {false, FSD_SIG_GEAR, FSD_TRIG_STATE_ENTER, 4, FSD_ACT_CAMERA, 1}},
        {"big_negative_arg",
         {true, FSD_SIG_MAP_SW_FR, FSD_TRIG_LONG, 0, FSD_ACT_SEAT_DRIVER, -2000000000}},
        /* The largest values each enum can carry today. A byte that silently
         * truncates would still look right for the small cases above. */
        {"max_enums",
         {true, (FsdSignal)(FSD_SIG_COUNT - 1), FSD_TRIG_DELTA, 2147483647,
          (FsdBodyAction)(FSD_ACT_COUNT - 1), 2147483647}},
    };
    const size_t nu = sizeof(rules) / sizeof(rules[0]);
    for(size_t i = 0; i < nu; i++) {
        uint8_t b[FSD_RULE_WIRE_LEN];
        fsd_rule_pack(&rules[i].r, b);
        fprintf(f, "    { \"name\": \"%s\", \"hex\": ", rules[i].name);
        emit_hex(f, b, sizeof(b));
        fprintf(f,
                /* 0/1, not true/false: every other vector's `fields` is a map
                 * of integers and the app's loader refuses anything else. A
                 * bool here would make the loader throw on a good fixture. */
                ", \"fields\": { \"enabled\": %u, \"signal\": %u, \"kind\": %u, "
                "\"value\": %ld, \"action\": %u, \"arg\": %ld } }%s\n",
                rules[i].r.enabled ? 1u : 0u, (unsigned)rules[i].r.signal,
                (unsigned)rules[i].r.kind, (long)rules[i].r.value,
                (unsigned)rules[i].r.action, (long)rules[i].r.arg,
                (i + 1 < nu) ? "," : "");
    }

    fprintf(f, "  ]\n}\n");
    (void)ns;
    (void)nc;
    (void)nr;
    (void)nu;
}

static void write_fixture(void) {
    const char* path = "fixtures/wire_vectors.json";
    FILE* f = fopen(path, "w");
    if(f) {
        emit_fixture(f);
        fclose(f);
        printf("  wrote %s\n", path);
    } else {
        printf("  NOTE could not write %s — run from test/\n", path);
    }

    /* Also to stdout, between markers. The committed copy has to come FROM the
     * packers rather than from anyone's idea of them, and this is how it gets
     * out of a CI runner and into the repository. */
    printf("----8<---- wire_vectors.json ----8<----\n");
    emit_fixture(stdout);
    printf("---->8---- wire_vectors.json ---->8----\n");
}

int main(void) {
    printf("test_wire\n");
    test_reverse_speed_needs_the_other_signal();
    test_state_layout();
    test_state_structural_zeros();
    test_state_blink_nibble();
    test_state_blind_spot();
    test_state_tyre_pressure();
    test_profile_sentinel();
    test_tyre_freshness();
    test_state_speed_limit_source();
    test_speed_limit_freshness();
    test_state_clamps();
    test_camstat_layout();
    test_camstat_packing_cannot_bleed();
    test_camstat_saturates();
    test_result();
    test_null_safety();

    printf("\n-- fixture --\n");
    write_fixture();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
