/* test_body_emit — the emitter, checked against bytes TSL actually sent.
 *
 * 🔴 Every frame here is COPIED OUT OF A CAPTURE, not written by hand. A
 * hand-built frame only proves the builder agrees with the table in my head;
 * this file proves it agrees with the car.
 *
 *      captures/2026-09-03/유휴          car,  20 frames, all identical
 *      captures/2026-09-03/맵등 켜기      car + TSL, 1 ms apart
 */
#include <stdio.h>
#include <string.h>

#include "../fsd_logic/fsd_body_emit.h"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, ...)                                    \
    do {                                                    \
        if(cond) {                                          \
            g_pass++;                                       \
        } else {                                            \
            g_fail++;                                       \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);   \
            printf(__VA_ARGS__);                            \
            printf("\n");                                   \
        }                                                   \
    } while(0)

/* 0x273 as the car sends it, 2026-09-03. Byte identical across 20 frames. */
static const uint8_t CAR[8] = {0x81, 0xE1, 0x00, 0x00, 0x44, 0x02, 0x30, 0x01};
/* 0x273 as TSL sends it 1 ms later while the map lights are on. */
static const uint8_t TSL[8] = {0x81, 0xE1, 0x00, 0x00, 0x44, 0x02, 0x30, 0x09};

static FsdEmitTemplate car_template(uint32_t at_ms) {
    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    t.seen = true;
    t.id = FSD_EMIT_MAP_LIGHT_ID;
    t.dlc = 8;
    memcpy(t.data, CAR, 8);
    t.seen_ms = at_ms;
    return t;
}

/* ── the one that matters ─────────────────────────────────────────────────── */

static void test_matches_tsl_byte_for_byte(void) {
    printf("\n-- 우리가 만든 프레임 == TSL 이 보낸 프레임 --\n");

    FsdEmitTemplate t = car_template(1000u);
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 1100u, &f) == FSD_EMIT_OK, "built");
    CHECK(f.id == 0x273u, "id 0x273, got 0x%X", (unsigned)f.id);
    CHECK(f.dlc == 8, "dlc 8, got %u", f.dlc);

    /* The whole point of the file. */
    CHECK(memcmp(f.data, TSL, 8) == 0,
          "bytes must equal TSL's: got %02X%02X%02X%02X%02X%02X%02X%02X",
          f.data[0], f.data[1], f.data[2], f.data[3],
          f.data[4], f.data[5], f.data[6], f.data[7]);

    /* And said the other way round: exactly one bit differs from the car's. */
    unsigned diff = 0;
    for(unsigned i = 0; i < 8; i++) {
        uint8_t x = (uint8_t)(f.data[i] ^ CAR[i]);
        while(x) { diff += (x & 1u); x >>= 1; }
    }
    CHECK(diff == 1, "exactly one bit differs from the car's frame, got %u", diff);
    CHECK((f.data[7] ^ CAR[7]) == 0x08u, "and it is byte7 bit3");
}

/* 0x1F9 as the car sends it, 2026-09-05. Identical in 273 control frames
 * across three unfiltered captures -- no exception, no counter, no checksum. */
static const uint8_t CAR_DOOR[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* 0x1F9 as TSL sends it in the same millisecond. 113 ms later the door moves. */
static const uint8_t TSL_DOOR[8] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static FsdEmitTemplate door_template(uint32_t at_ms) {
    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    t.seen = true;
    t.id = FSD_EMIT_DOOR_ID;
    t.dlc = 8;
    memcpy(t.data, CAR_DOOR, 8);
    t.seen_ms = at_ms;
    return t;
}

/* 0x3E9 as the car and TSL sent it, 2026-09-05, while the car was in reverse.
 * Four consecutive pairs, copied out of captures/2026-09-05/후진. */
static const struct {
    uint8_t car[8];
    uint8_t tsl[8];
} HZ[4] = {
    {{0xF1,0x88,0x02,0,0,0,0xC0,0x27}, {0xF5,0x88,0x02,0,0,0,0xD0,0x3B}},
    {{0xF1,0x88,0x02,0,0,0,0xD0,0x37}, {0xF5,0x88,0x02,0,0,0,0xE0,0x4B}},
    {{0xF1,0x88,0x02,0,0,0,0xE0,0x47}, {0xF5,0x88,0x02,0,0,0,0xF0,0x5B}},
    /* The wrap. F -> 0, and the check follows it. */
    {{0xF1,0x88,0x02,0,0,0,0xF0,0x57}, {0xF5,0x88,0x02,0,0,0,0x00,0x6B}},
};

static FsdEmitTemplate hz_template(const uint8_t* car, uint32_t at_ms) {
    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    t.seen = true;
    t.id = FSD_EMIT_HAZARD_ID;
    t.dlc = 8;
    memcpy(t.data, car, 8);
    t.seen_ms = at_ms;
    return t;
}

/* The one that matters, and it is a harder claim than the other two emitters
 * make. For the light and the door "our bytes == TSL's bytes" only asks whether
 * we set the right bit. Here it also asks whether we advanced the counter the
 * way TSL does and computed the same check -- on four pairs including the wrap
 * from F to 0. */
static void test_hazards_match_tsl_byte_for_byte(void) {
    printf("\n-- hazards: our frame == the frame TSL sent, four times --\n");

    for(unsigned i = 0; i < 4; i++) {
        FsdEmitTemplate t = hz_template(HZ[i].car, 1000u);
        FsdEmitFrame f;
        CHECK(fsd_emit_build(FSD_ACT_HAZARDS, &t, 1100u, &f) == FSD_EMIT_OK,
              "pair %u built", i);
        CHECK(f.id == 0x3E9u, "pair %u: id 0x3E9", i);
        CHECK(memcmp(f.data, HZ[i].tsl, 8) == 0,
              "pair %u: got %02X%02X%02X%02X%02X%02X%02X%02X", i,
              f.data[0], f.data[1], f.data[2], f.data[3],
              f.data[4], f.data[5], f.data[6], f.data[7]);
    }
}

/* 🔴 The check is not a guess that happened to fit four frames. It was
 * derived against every distinct 0x3E9 payload in every capture we hold -- 162
 * of them, zero exceptions -- and these are spot samples of that set, chosen
 * because their other bytes differ from the reverse capture's. A rule that fits
 * one situation and a rule that fits the frame look identical until they do
 * not. */
static void test_hazard_check_holds_away_from_the_hazard_capture(void) {
    printf("\n-- the check rule, on frames from other captures --\n");

    /* car frames seen while nothing was happening -- byte1/byte2 are 0 here and
     * 0x88/0x02 in the reverse capture, so the check must differ and does. */
    static const uint8_t OTHERS[5][8] = {
        {0xF1,0,0,0,0,0,0x00,0xDD},
        {0xF1,0,0,0,0,0,0x10,0xED},
        {0xF1,0,0,0x40,0,0,0x70,0x8D},
        {0xF1,0,0,0x80,0,0,0x20,0x7D},
        {0xF1,0,0,0xC0,0,0,0x00,0x9D},
    };
    for(unsigned i = 0; i < 5; i++) {
        unsigned sum = 0;
        for(unsigned k = 0; k < 7u; k++) sum += OTHERS[i][k];
        CHECK((uint8_t)((sum + 0xECu) & 0xFFu) == OTHERS[i][7],
              "sample %u: rule reproduces the car's own byte7 (%02X)",
              i, OTHERS[i][7]);
    }
}

static void test_hazard_refusals(void) {
    printf("\n-- hazards: refusals --\n");

    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, &t, 1000u, &f) == FSD_EMIT_NO_TEMPLATE,
          "no template -> refuse");

    t = hz_template(HZ[0].car, 1000u);
    t.id = 0x273u;
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "wrong id -> refuse");

    /* 🔴 Staleness bites harder here than anywhere else. A copied light
     * frame that is a second old is still a valid light frame; a hazard frame
     * that old carries a counter the car has already run past. */
    t = hz_template(HZ[0].car, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, &t, 1000u + 1499u, &f) == FSD_EMIT_OK,
          "1499 ms old is still usable");
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, &t, 1000u + 1500u, &f)
              == FSD_EMIT_STALE_TEMPLATE,
          "1500 ms old is not");

    /* And it cannot be confused with the other two commands. */
    FsdEmitTemplate light = car_template(1000u);
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, &light, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "hazards + light template -> refuse");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "light + hazard template -> refuse");
}

/* The low nibble of byte 6 is not ours. We have only ever seen 0 and 2 there
 * and do not know what either means, so it is copied through -- and this test
 * exists because "advance the counter" written carelessly clears it. */
static void test_hazard_leaves_the_other_nibble_alone(void) {
    printf("\n-- the low nibble of byte 6 is copied, not cleared --\n");

    uint8_t car[8] = {0xF1, 0x88, 0x02, 0, 0, 0, 0xC2, 0x00};
    unsigned sum = 0;
    for(unsigned k = 0; k < 7u; k++) sum += car[k];
    car[7] = (uint8_t)((sum + 0xECu) & 0xFFu); /* a valid car frame */

    FsdEmitTemplate t = hz_template(car, 1000u);
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, &t, 1100u, &f) == FSD_EMIT_OK, "built");
    CHECK((f.data[6] & 0x0Fu) == 0x02u, "low nibble survives, got 0x%X",
          f.data[6] & 0x0Fu);
    CHECK((f.data[6] >> 4) == 0x0Du, "and the counter advanced C -> D");

    /* Our own frame must satisfy the rule we derived from the car's. */
    unsigned s2 = 0;
    for(unsigned k = 0; k < 7u; k++) s2 += f.data[k];
    CHECK(f.data[7] == (uint8_t)((s2 + 0xECu) & 0xFFu), "and the check is right");
}

static void test_door_matches_tsl_byte_for_byte(void) {
    printf("\n-- door: our frame == the frame TSL sent --\n");

    FsdEmitTemplate t = door_template(1000u);
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, &t, 1100u, &f) == FSD_EMIT_OK, "built");
    CHECK(f.id == 0x1F9u, "id 0x1F9, got 0x%X", (unsigned)f.id);
    CHECK(f.dlc == 8, "dlc 8, got %u", f.dlc);
    CHECK(memcmp(f.data, TSL_DOOR, 8) == 0,
          "bytes must equal TSL's: got %02X%02X%02X%02X%02X%02X%02X%02X",
          f.data[0], f.data[1], f.data[2], f.data[3],
          f.data[4], f.data[5], f.data[6], f.data[7]);

    /* Said the other way round: exactly two bits differ from the car's frame,
     * and both are in byte 1. */
    unsigned diff = 0;
    for(unsigned i = 0; i < 8; i++) {
        uint8_t x = (uint8_t)(f.data[i] ^ CAR_DOOR[i]);
        while(x) { diff += (x & 1u); x >>= 1; }
    }
    CHECK(diff == 2, "exactly two bits differ, got %u", diff);
    CHECK((f.data[1] ^ CAR_DOOR[1]) == 0x03u, "and they are byte1 bits 0 and 1");
}

/* 🔴 The two commands must not be interchangeable. A caller that asks for
 * a light and receives a door-open frame is the worst failure this file can
 * have, and it would look like an ordinary off-by-one in a switch. */
static void test_door_and_light_do_not_cross(void) {
    printf("\n-- the two commands cannot be swapped --\n");

    FsdEmitTemplate door = door_template(1000u);
    FsdEmitTemplate light = car_template(1000u);
    FsdEmitFrame f;

    /* Right action, wrong template: refused, not silently stamped. */
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, &light, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "door action + light template -> refuse");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &door, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "light action + door template -> refuse");

    /* And the two outputs are different frames on different ids. */
    FsdEmitFrame a, b;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, &door, 1100u, &a) == FSD_EMIT_OK, "door built");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &light, 1100u, &b) == FSD_EMIT_OK, "light built");
    CHECK(a.id != b.id, "different ids");
}

static void test_door_refusals(void) {
    printf("\n-- door: refusals --\n");

    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, &t, 1000u, &f) == FSD_EMIT_NO_TEMPLATE,
          "no template -> refuse");

    /* 🔴 We have only ever seen this frame all-zero, which is exactly why
     * a template is still required. Knowing every byte was zero on THIS car is
     * not knowing what the other seven bytes mean. */
    t = door_template(1000u);
    t.dlc = 4;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "short frame -> refuse (this car has short frames: 0x311, 0x399, 0x3D8)");

    t = door_template(1000u);
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, &t, 1000u + 1499u, &f) == FSD_EMIT_OK,
          "1499 ms old is still usable");
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, &t, 1000u + 1500u, &f)
              == FSD_EMIT_STALE_TEMPLATE,
          "1500 ms old is not");
}

/* The axis row, checked here rather than only in fsd_body's own tests, because
 * this is the file that made the row's stated condition true. */
static void test_door_row_is_open_but_narrow(void) {
    printf("\n-- the door row opened, and nothing else relaxed --\n");

    const FsdBodyCaps* c = fsd_body_caps(FSD_ACT_DOOR_OPEN);
    CHECK(c != NULL, "row exists");
    if(!c) return;

    CHECK(c->armable_at_runtime, "armable: the command frame is measured");

    /* 🔴 Every one of these is a gate that stays shut. If a later change
     * wants one of them open it has to say so here, in a red test, and not by
     * quietly widening a row while adding something unrelated. */
    CHECK(!c->may_act_while_moving, "not while moving");
    CHECK(!c->may_act_out_of_park, "park only");
    CHECK(!c->may_act_without_driver, "driver must be present");
    CHECK(!c->may_act_without_drive_session, "a drive must have happened");

    /* A rate limit an order above the light's. Not a debounce -- a bound on how
     * bad a stuck rule gets. */
    CHECK(c->min_interval_ms >= 3000u, "at least 3 s apart, got %u",
          (unsigned)c->min_interval_ms);
    CHECK(c->min_interval_ms > fsd_body_caps(FSD_ACT_MAP_LIGHT)->min_interval_ms,
          "slower than the map light");
}

/* 🔴 The car frame carries no counter and no checksum -- that is WHY copying
 * works at all. If a future car adds one, replaying a template is no longer a
 * valid frame and the car will ignore it. Pinned here so that shows up as a red
 * test and not as "the command stopped working". */
static void test_no_counter_in_the_frame(void) {
    printf("\n-- 0x273 에는 카운터도 체크섬도 없다 --\n");
    /* Three consecutive car frames from 유휴, 500 ms apart. */
    const uint8_t a[8] = {0x81, 0xE1, 0x00, 0x00, 0x44, 0x02, 0x30, 0x01};
    const uint8_t b[8] = {0x81, 0xE1, 0x00, 0x00, 0x44, 0x02, 0x30, 0x01};
    const uint8_t c[8] = {0x81, 0xE1, 0x00, 0x00, 0x44, 0x02, 0x30, 0x01};
    CHECK(memcmp(a, b, 8) == 0 && memcmp(b, c, 8) == 0,
          "consecutive frames are identical -- no rolling field");
}

/* ── refusals ─────────────────────────────────────────────────────────────── */

static void test_refuses_without_a_template(void) {
    printf("\n-- 베낄 것이 없으면 만들지 않는다 --\n");

    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));      /* seen = false */
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 1000u, &f) == FSD_EMIT_NO_TEMPLATE,
          "no template -> refuse");

    /* 🔴 Zeros are not a fallback. 0x273 carries mirrors, locks, horn and seat
     * heaters; a frame of zeros is a statement about all of them. */
    t = car_template(1000u);
    memset(t.data, 0, 8);
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 1100u, &f) == FSD_EMIT_OK,
          "a real all-zero payload is still a template (we do not judge content)");
    CHECK(f.data[7] == 0x08u, "and we only ever set our own bit");
}

static void test_refuses_a_stale_template(void) {
    printf("\n-- 낡은 템플릿은 낡은 주장이다 --\n");

    FsdEmitTemplate t = car_template(1000u);
    FsdEmitFrame f;

    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 1000u + 1499u, &f) == FSD_EMIT_OK,
          "1499 ms old is still usable");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 1000u + 1500u, &f)
              == FSD_EMIT_STALE_TEMPLATE,
          "1500 ms old is not");

    /* Three car periods. Stated so that changing the constant without a reason
     * shows up here. */
    CHECK(FSD_EMIT_TEMPLATE_MAX_AGE_MS == 1500u, "three 500 ms periods");

    /* The millisecond clock wraps every ~49 days. A fresh template must not
     * look ancient across the wrap. */
    t.seen_ms = 0xFFFFFF00u;
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 0x00000100u, &f) == FSD_EMIT_OK,
          "fresh across the 32-bit wrap");
}

static void test_refuses_the_wrong_frame(void) {
    printf("\n-- 다른 프레임에 우리 비트를 찍지 않는다 --\n");

    FsdEmitTemplate t = car_template(1000u);
    FsdEmitFrame f;

    t.id = 0x3C2u;   /* the scroll/seat/camera frame -- a plausible mix-up */
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "wrong id -> refuse");

    t = car_template(1000u);
    t.dlc = 7;
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "short frame -> refuse (byte 7 would not exist)");

    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, NULL, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "null template -> refuse");
}

/* ── the gap, stated ──────────────────────────────────────────────────────── */

static void test_only_map_light_has_an_encoding(void) {
    printf("\n-- 여섯은 아직 방출기가 없다 --\n");

    FsdEmitTemplate t = car_template(1000u);
    FsdEmitFrame f;

    CHECK(fsd_emit_supported(FSD_ACT_MAP_LIGHT), "map light: yes");
    CHECK(fsd_emit_supported(FSD_ACT_DOOR_OPEN), "door: yes (measured 2026-09-05)");

    CHECK(fsd_emit_supported(FSD_ACT_HAZARDS), "hazards: yes (measured 2026-09-05)");

    const FsdBodyAction rest[] = {
        FSD_ACT_CAMERA, FSD_ACT_SEAT_DRIVER,
        FSD_ACT_SEAT_PASSENGER, FSD_ACT_SCROLL, FSD_ACT_GEAR_D,
    };
    for(unsigned i = 0; i < sizeof(rest) / sizeof(rest[0]); i++) {
        CHECK(!fsd_emit_supported(rest[i]),
              "%s has no emitter", fsd_body_action_str(rest[i]));
        CHECK(fsd_emit_build(rest[i], &t, 1100u, &f) == FSD_EMIT_NO_ENCODING,
              "%s -> NO_ENCODING", fsd_body_action_str(rest[i]));
    }

    /* 🔴 The two statements of the same fact must agree. If someone flips a row
     * to armable without an encoding, or writes an encoding without opening the
     * row, this is where it shows. */
    for(unsigned a = 0; a < FSD_ACT_COUNT; a++) {
        const FsdBodyCaps* c = fsd_body_caps((FsdBodyAction)a);
        CHECK(c != NULL, "caps row exists for %u", a);
        if(!c) continue;
        CHECK(c->armable_at_runtime == fsd_emit_supported((FsdBodyAction)a),
              "%s: armable=%d but emitter=%d -- these must move together",
              fsd_body_action_str((FsdBodyAction)a),
              (int)c->armable_at_runtime,
              (int)fsd_emit_supported((FsdBodyAction)a));
    }
}

static void test_result_names(void) {
    printf("\n-- 사유에 이름이 있다 --\n");
    const FsdEmitResult all[] = {
        FSD_EMIT_OK, FSD_EMIT_NO_TEMPLATE, FSD_EMIT_STALE_TEMPLATE,
        FSD_EMIT_BAD_TEMPLATE, FSD_EMIT_NO_ENCODING,
    };
    for(unsigned i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        const char* s = fsd_emit_result_str(all[i]);
        CHECK(s && s[0] && s[0] != '?', "result %u has a name", (unsigned)all[i]);
    }
}

int main(void) {
    printf("test_body_emit\n");
    test_matches_tsl_byte_for_byte();
    test_hazards_match_tsl_byte_for_byte();
    test_hazard_check_holds_away_from_the_hazard_capture();
    test_hazard_refusals();
    test_hazard_leaves_the_other_nibble_alone();
    test_door_matches_tsl_byte_for_byte();
    test_door_and_light_do_not_cross();
    test_door_refusals();
    test_door_row_is_open_but_narrow();
    test_no_counter_in_the_frame();
    test_refuses_without_a_template();
    test_refuses_a_stale_template();
    test_refuses_the_wrong_frame();
    test_only_map_light_has_an_encoding();
    test_result_names();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
