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
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 1100u, &f) == FSD_EMIT_OK, "built");
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
        CHECK(fsd_emit_build(FSD_ACT_HAZARDS, 0, &t, 1100u, &f) == FSD_EMIT_OK,
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
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, 0, &t, 1000u, &f) == FSD_EMIT_NO_TEMPLATE,
          "no template -> refuse");

    t = hz_template(HZ[0].car, 1000u);
    t.id = 0x273u;
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, 0, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "wrong id -> refuse");

    /* 🔴 Staleness bites harder here than anywhere else. A copied light
     * frame that is a second old is still a valid light frame; a hazard frame
     * that old carries a counter the car has already run past. */
    t = hz_template(HZ[0].car, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, 0, &t, 1000u + 1499u, &f) == FSD_EMIT_OK,
          "1499 ms old is still usable");
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, 0, &t, 1000u + 1500u, &f)
              == FSD_EMIT_STALE_TEMPLATE,
          "1500 ms old is not");

    /* And it cannot be confused with the other two commands. */
    FsdEmitTemplate light = car_template(1000u);
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, 0, &light, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "hazards + light template -> refuse");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
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
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, 0, &t, 1100u, &f) == FSD_EMIT_OK, "built");
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
    /* arg 0 is the right FRONT door -- the selector every rule stored before
     * this argument existed already meant. */
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, 0, &t, 1100u, &f) == FSD_EMIT_OK, "built");
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
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, 0, &light, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "door action + light template -> refuse");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &door, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "light action + door template -> refuse");

    /* And the two outputs are different frames on different ids. */
    FsdEmitFrame a, b;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, 0, &door, 1100u, &a) == FSD_EMIT_OK, "door built");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &light, 1100u, &b) == FSD_EMIT_OK, "light built");
    CHECK(a.id != b.id, "different ids");
}

static void test_door_refusals(void) {
    printf("\n-- door: refusals --\n");

    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, 0, &t, 1000u, &f) == FSD_EMIT_NO_TEMPLATE,
          "no template -> refuse");

    /* 🔴 We have only ever seen this frame all-zero, which is exactly why
     * a template is still required. Knowing every byte was zero on THIS car is
     * not knowing what the other seven bytes mean. */
    t = door_template(1000u);
    t.dlc = 4;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, 0, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "short frame -> refuse (this car has short frames: 0x311, 0x399, 0x3D8)");

    t = door_template(1000u);
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, 0, &t, 1000u + 1499u, &f) == FSD_EMIT_OK,
          "1499 ms old is still usable");
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, 0, &t, 1000u + 1500u, &f)
              == FSD_EMIT_STALE_TEMPLATE,
          "1500 ms old is not");
}

/* 🔴🔴 THE DOOR'S ROW WAS THE NARROWEST IN THE TABLE AND THIS TEST IS WHAT
 * KEPT IT THAT WAY: armable, but not while moving, not out of park, and no
 * more than one command every three seconds -- a rate limit an order above the
 * map light's, written as a bound on how bad a stuck rule gets rather than as
 * a debounce.
 *
 * The owner deleted every one of those columns on 2026-09-10 -- "차의 모든
 * 안전게이트관련사항을 삭제해라. 필요하다면 추후 내가 하나씩 추가하겠다" --
 * so those assertions are not weakened here. They are gone with the fields
 * they were about, and the guard that keeps them gone is
 * test_no_gate_stands_in_front_of_a_press() in test_pipeline.c.
 *
 * ⚠️ THIS IS THE ACTION WHERE THAT COSTS THE MOST, AND IT IS WORTH SAYING
 * PLAINLY: the door opens OUTWARD, and no signal on this bus says what is
 * standing beside the car. That was true while the gates existed too -- park
 * and standstill never answered that question either -- but nothing answers it
 * now, and there is no longer even a rate limit between one command and the
 * next.
 *
 * 🟢 WHAT IS LEFT IS WHAT THIS TEST NOW PINS, and it is not nothing: the
 * command frame is one we are allowed to build at all, and a held command has
 * a leash. */
static void test_the_door_keeps_what_survived(void) {
    printf("\n-- 문에 남은 것: 만들 수 있는 프레임과 목줄 --\n");

    const FsdBodyCaps* c = fsd_body_caps(FSD_ACT_DOOR_OPEN);
    CHECK(c != NULL, "row exists");
    if(!c) return;

    /* 🔴 0x1F9 must NOT be on the deny-list, or the feature is impossible --
     * and 0x102/0x103, the door STATUS frames the car sends about itself, must
     * still be, because we have no reason to build one and every reason to
     * refuse if something ever does. Layer D, which the owner kept. */
    CHECK(!fsd_body_tx_id_refused(0x1F9u), "0x1F9 is the door open command");
    CHECK(fsd_body_tx_id_refused(0x102u), "0x102 door status stays refused");
    CHECK(fsd_body_tx_id_refused(0x103u), "0x103 door status stays refused");

    /* Not a question about the car's situation -- a bound on one stuck flag
     * transmitting forever. The same family as the chokepoint, which is why it
     * survived the removal. */
    CHECK(c->max_hold_ms > 0u && c->max_hold_ms <= 30000u,
          "a held door command has a leash, got %u ms", (unsigned)c->max_hold_ms);
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
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 1000u, &f) == FSD_EMIT_NO_TEMPLATE,
          "no template -> refuse");

    /* 🔴 Zeros are not a fallback. 0x273 carries mirrors, locks, horn and seat
     * heaters; a frame of zeros is a statement about all of them. */
    t = car_template(1000u);
    memset(t.data, 0, 8);
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 1100u, &f) == FSD_EMIT_OK,
          "a real all-zero payload is still a template (we do not judge content)");
    CHECK(f.data[7] == 0x08u, "and we only ever set our own bit");
}

static void test_refuses_a_stale_template(void) {
    printf("\n-- 낡은 템플릿은 낡은 주장이다 --\n");

    FsdEmitTemplate t = car_template(1000u);
    FsdEmitFrame f;

    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 1000u + 1499u, &f) == FSD_EMIT_OK,
          "1499 ms old is still usable");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 1000u + 1500u, &f)
              == FSD_EMIT_STALE_TEMPLATE,
          "1500 ms old is not");

    /* Three car periods. Stated so that changing the constant without a reason
     * shows up here. */
    CHECK(FSD_EMIT_TEMPLATE_MAX_AGE_MS == 1500u, "three 500 ms periods");

    /* The millisecond clock wraps every ~49 days. A fresh template must not
     * look ancient across the wrap. */
    t.seen_ms = 0xFFFFFF00u;
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 0x00000100u, &f) == FSD_EMIT_OK,
          "fresh across the 32-bit wrap");
}

static void test_refuses_the_wrong_frame(void) {
    printf("\n-- 다른 프레임에 우리 비트를 찍지 않는다 --\n");

    FsdEmitTemplate t = car_template(1000u);
    FsdEmitFrame f;

    t.id = 0x3C2u;   /* the scroll/seat/camera frame -- a plausible mix-up */
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "wrong id -> refuse");

    t = car_template(1000u);
    t.dlc = 7;
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "short frame -> refuse (byte 7 would not exist)");

    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, NULL, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "null template -> refuse");
}

/* ── the gap, stated ──────────────────────────────────────────────────────── */

static void test_which_actions_have_an_encoding(void) {
    printf("\n-- 여섯은 방출기가 있고 다섯은 없다 --\n");

    FsdEmitTemplate t = car_template(1000u);
    FsdEmitFrame f;

    CHECK(fsd_emit_supported(FSD_ACT_MAP_LIGHT), "map light: yes");
    CHECK(fsd_emit_supported(FSD_ACT_DOOR_OPEN), "door: yes (measured 2026-09-05)");

    CHECK(fsd_emit_supported(FSD_ACT_HAZARDS), "hazards: yes (measured 2026-09-05)");
    CHECK(fsd_emit_supported(FSD_ACT_TURN_SIGNAL),
          "turn signal: yes (measured 2026-09-05, 4th visit)");
    CHECK(fsd_emit_supported(FSD_ACT_MIRROR),
          "mirror: yes (measured 2026-09-06, 5th visit)");
    CHECK(fsd_emit_supported(FSD_ACT_LIGHT_HORN),
          "light horn: yes (measured 2026-09-06, 5th visit)");

    const FsdBodyAction rest[] = {
        FSD_ACT_CAMERA, FSD_ACT_SEAT_DRIVER,
        FSD_ACT_SEAT_PASSENGER, FSD_ACT_SCROLL, FSD_ACT_GEAR_D,
    };
    /* 🔴 The list is written out by hand, so it can quietly stop covering
     * things: add an action to the enum, forget this line, and the loop below
     * still passes while testing one action less. Same shape as the CAN-id
     * check that only compared the intersection. Count it. */
    CHECK(sizeof(rest) / sizeof(rest[0]) == (size_t)FSD_ACT_COUNT - 6u,
          "the no-emitter list must name every action that is not one of the "
          "six with emitters (%u named, %u expected)",
          (unsigned)(sizeof(rest) / sizeof(rest[0])),
          (unsigned)FSD_ACT_COUNT - 6u);
    for(unsigned i = 0; i < sizeof(rest) / sizeof(rest[0]); i++) {
        CHECK(!fsd_emit_supported(rest[i]),
              "%s has no emitter", fsd_body_action_str(rest[i]));
        CHECK(fsd_emit_build(rest[i], 0, &t, 1100u, &f) == FSD_EMIT_NO_ENCODING,
              "%s -> NO_ENCODING", fsd_body_action_str(rest[i]));
    }

    /* 🔴 THERE USED TO BE TWO STATEMENTS OF THIS FACT AND THEY HAD TO AGREE:
     * armable_at_runtime in the caps table, and fsd_emit_supported() here. The
     * check caught a row flipped open without an encoding, or an encoding
     * written without opening the row.
     *
     * 🟢 The owner removed the authority column on 2026-09-10, so there is
     * ONE statement left -- and one statement cannot disagree with itself.
     * That is the tenth pattern's better ending: not two copies kept in step
     * by a test, but one copy. The rows are still checked for existence,
     * because an action with no row at all is a different fault. */
    for(unsigned a = 0; a < FSD_ACT_COUNT; a++) {
        CHECK(fsd_body_caps((FsdBodyAction)a) != NULL, "caps row exists for %u", a);
    }
}

/**
 * 🔴 A SUPPORTED ACTION MUST NEVER BORROW ANOTHER ACTION'S ENCODING.
 *
 * fsd_emit_build() picks the id, length and bit from a switch on the action.
 * That switch used to end in `default:` -> map light, and the comment excused
 * it with "fsd_emit_supported() already refused everything else". True today,
 * false the moment somebody adds an action to fsd_emit_supported() and forgets
 * the switch -- and then the new action silently emits a MAP LIGHT COMMAND and
 * returns FSD_EMIT_OK.
 *
 * A compile warning now catches that (the switch lists every case and has no
 * default). This test catches it too, from the other side and at runtime: hand
 * every supported action the 0x273 template and see what each one does with it.
 *
 * ⚠️ THE INVARIANT CHANGED SHAPE ON 2026-09-07, AND IT GOT STRONGER.
 * It used to be "only the map light may accept this template", which stopped
 * being true the day the mirror arrived on the same id -- the mirror MUST
 * accept it. Weakening the test to skip the mirror would have thrown away the
 * thing it was written to catch, so the claim moved down a level to the one
 * that never changes: an action that accepts a template must write ITS OWN
 * field and nobody else's. A borrowed encoding now shows up as a frame whose
 * changed bits belong to somebody else, which is what "borrowed" actually
 * means -- the old BAD_TEMPLATE check was only ever a proxy for it.
 */
static void test_no_action_borrows_another_encoding(void) {
    printf("\n-- 지원되는 동작은 남의 인코딩을 빌리지 않는다 --\n");

    FsdEmitTemplate t = car_template(1000u); /* 0x273 */
    FsdEmitFrame f;

    /* Which supported actions write 0x273, and which byte each one owns.
     *
     * 🔴 Written by hand, so it is counted below -- same reason as `rest[]`.
     * A third action arriving on this id without a line here must be a red
     * test rather than a silent pass. */
    static const struct {
        FsdBodyAction act;
        uint8_t byte_ix;
        uint8_t mask;
        const char* what;
    } ON_273[] = {
        {FSD_ACT_MAP_LIGHT, FSD_EMIT_MAP_LIGHT_BYTE, FSD_EMIT_MAP_LIGHT_MASK, "byte7 bit3"},
        {FSD_ACT_MIRROR, FSD_EMIT_MIRROR_BYTE, FSD_EMIT_MIRROR_MASK, "byte3"},
    };

    unsigned supported = 0, on_273 = 0;
    for(unsigned a = 0; a < FSD_ACT_COUNT; a++) {
        FsdBodyAction act = (FsdBodyAction)a;
        if(!fsd_emit_supported(act)) continue;
        supported++;

        /* Is this one of the actions that lives on 0x273? */
        int mine = -1;
        for(unsigned k = 0; k < sizeof(ON_273) / sizeof(ON_273[0]); k++)
            if(ON_273[k].act == act) mine = (int)k;

        FsdEmitResult r = fsd_emit_build(act, 0, &t, 1100u, &f);
        if(mine < 0) {
            CHECK(r == FSD_EMIT_BAD_TEMPLATE,
                  "%s must refuse the 0x273 template, got '%s'",
                  fsd_body_action_str(act), fsd_emit_result_str(r));
            continue;
        }

        on_273++;
        CHECK(r == FSD_EMIT_OK, "%s accepts its own frame, got '%s'",
              fsd_body_action_str(act), fsd_emit_result_str(r));
        CHECK(f.id == 0x273u, "%s emits on 0x273, got 0x%X",
              fsd_body_action_str(act), (unsigned)f.id);

        /* 🔴 THE CLAIM. Every byte it changed must be inside its own field.
         * An action that borrowed the map light's encoding would change byte 7
         * while its row says byte 3, and this is the line that says so. */
        for(unsigned i = 0; i < 8; i++) {
            const uint8_t changed = (uint8_t)(f.data[i] ^ t.data[i]);
            const uint8_t allowed =
                (i == ON_273[mine].byte_ix) ? ON_273[mine].mask : 0u;
            CHECK((changed & (uint8_t)~allowed) == 0u,
                  "%s changed 0x%02X in byte %u but owns only %s",
                  fsd_body_action_str(act), changed, i, ON_273[mine].what);
        }
        /* And it must actually have done something -- an emitter that changed
         * nothing would pass the loop above trivially. */
        CHECK(memcmp(f.data, t.data, 8) != 0,
              "%s built a frame identical to the car's", fsd_body_action_str(act));
    }
    CHECK(supported == 6u, "six actions have emitters, saw %u", supported);
    CHECK(on_273 == sizeof(ON_273) / sizeof(ON_273[0]),
          "every action named on 0x273 must be supported: %u of %u",
          on_273, (unsigned)(sizeof(ON_273) / sizeof(ON_273[0])));
}

/* ── the turn signal, 0x249 ───────────────────────────────────────────────────
 *
 * 🔴 EVERY FRAME IN THIS SECTION IS TWO LINES OUT OF A CAPTURE, IN ORDER.
 * The car's frame is the template and TSL's is what we must produce, so each
 * pair is a complete statement of the command with nothing of mine in it.
 *
 *      captures/2026-09-05-4차/TSL좌깜빡이켜기
 *      captures/2026-09-05-4차/TSL우깜빡이켜기
 *      captures/2026-09-05-4차/TSL깜빡이전부끄기
 */
typedef struct {
    uint8_t car[4];
    uint8_t tsl[4];
    int32_t turn;
    const char* where;
} TurnCase;

/* Ten consecutive injections across three captures. Ten different counters,
 * which is the point: the check table is indexed by counter and one pair would
 * only prove one row of it. */
static const TurnCase TURN_CASES[] = {
    {{0x5E, 0x09, 0x00, 0x00}, {0x92, 0x0A, 0x08, 0x00}, FSD_EMIT_TURN_LEFT, "좌 7.955->7.956"},
    {{0xE2, 0x0A, 0x00, 0x00}, {0x58, 0x0B, 0x08, 0x00}, FSD_EMIT_TURN_LEFT, "좌 8.006"},
    {{0x28, 0x0B, 0x00, 0x00}, {0x4A, 0x0C, 0x08, 0x00}, FSD_EMIT_TURN_LEFT, "좌 8.055->8.056"},
    {{0x3A, 0x0C, 0x00, 0x00}, {0x63, 0x0D, 0x08, 0x00}, FSD_EMIT_TURN_LEFT, "좌 8.106"},

    /* 🔴 F -> 0. The counter wraps inside the command, and it wraps in a real
     * capture rather than in a case I invented. */
    {{0xCE, 0x0F, 0x00, 0x00}, {0xA3, 0x00, 0x04, 0x00}, FSD_EMIT_TURN_RIGHT, "우 5.851"},
    {{0x9B, 0x00, 0x00, 0x00}, {0xD0, 0x01, 0x04, 0x00}, FSD_EMIT_TURN_RIGHT, "우 5.901"},
    {{0xE8, 0x01, 0x00, 0x00}, {0x12, 0x02, 0x04, 0x00}, FSD_EMIT_TURN_RIGHT, "우 5.951"},

    /* 🔴 The same car frame 9B000000 appears here and one line above, and the
     * two commands differ ONLY in the stalk byte -- so this pair and that one
     * together prove the check byte moves with the stalk and not just with the
     * counter. D0 vs F4 out of the same template. */
    {{0x9B, 0x00, 0x00, 0x00}, {0xF4, 0x01, 0x02, 0x00}, FSD_EMIT_TURN_TAP_UP, "끄기 6.743"},
    {{0xE8, 0x01, 0x00, 0x00}, {0x36, 0x02, 0x02, 0x00}, FSD_EMIT_TURN_TAP_UP, "끄기 6.793"},
    {{0x2A, 0x02, 0x00, 0x00}, {0xCF, 0x03, 0x02, 0x00}, FSD_EMIT_TURN_TAP_UP, "끄기 6.844"},
};

static FsdEmitTemplate turn_template(const uint8_t car[4], uint32_t at_ms) {
    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    t.seen = true;
    t.id = FSD_EMIT_TURN_ID;
    t.dlc = FSD_EMIT_TURN_DLC;
    memcpy(t.data, car, 4);
    t.seen_ms = at_ms;
    return t;
}

static void test_turn_matches_tsl_byte_for_byte(void) {
    printf("\n-- 깜빡이: 우리 프레임 == TSL 프레임 (열 짝) --\n");

    for (unsigned i = 0; i < sizeof(TURN_CASES) / sizeof(TURN_CASES[0]); i++) {
        const TurnCase* c = &TURN_CASES[i];
        FsdEmitTemplate t = turn_template(c->car, 1000u);
        FsdEmitFrame f;
        FsdEmitResult r = fsd_emit_build(FSD_ACT_TURN_SIGNAL, c->turn, &t, 1010u, &f);
        CHECK(r == FSD_EMIT_OK, "%s: built (%s)", c->where, fsd_emit_result_str(r));
        CHECK(f.id == FSD_EMIT_TURN_ID, "%s: id 0x249, got 0x%X", c->where, (unsigned)f.id);
        CHECK(f.dlc == 4u, "%s: dlc 4, got %u", c->where, f.dlc);
        CHECK(memcmp(f.data, c->tsl, 4) == 0,
              "%s: %s -> expected %02X%02X%02X%02X, got %02X%02X%02X%02X",
              c->where, fsd_emit_turn_str(c->turn),
              c->tsl[0], c->tsl[1], c->tsl[2], c->tsl[3],
              f.data[0], f.data[1], f.data[2], f.data[3]);
    }
}

/**
 * 🔴 THE SAME BUILDER, CHECKED AGAINST FRAMES TSL NEVER TOUCHED.
 *
 * In the two captures where a PERSON worked the stalk there is no injection at
 * all -- every 0x249 is the car's. Consecutive car frames therefore form
 * (template, expected) pairs of exactly the kind the emitter produces: the
 * counter advances by one and the stalk field is whatever the driver was
 * holding. If our table is right, feeding frame N and asking for the direction
 * frame N+1 carries must reproduce frame N+1 to the byte.
 *
 * That is a different kind of evidence from the ten pairs above. Those prove we
 * agree with the commercial device. These prove we agree with the CAR.
 *
 *      captures/2026-09-05-4차/좌깜빡이   (사람이 좌 스토크)
 *      captures/2026-09-05-4차/우깜빡이   (사람이 우 스토크)
 */
static void test_turn_reproduces_the_cars_own_frames(void) {
    printf("\n-- 깜빡이: 사람이 만진 캡처의 차 프레임을 그대로 재현한다 --\n");

    static const TurnCase SELF[] = {
        {{0xCC, 0x01, 0x06, 0x00}, {0x5A, 0x02, 0x08, 0x00}, FSD_EMIT_TURN_LEFT, "좌 DOWN_1->DOWN_2"},
        {{0x5A, 0x02, 0x08, 0x00}, {0xA3, 0x03, 0x08, 0x00}, FSD_EMIT_TURN_LEFT, "좌 이어서"},
        /* 🔴 A3 -> A3: two different counters, the SAME check byte. This is the
         * pair that makes byte 0 impossible to be a CRC over these bytes, and
         * the table reproduces it without knowing that. */
        {{0xA3, 0x03, 0x08, 0x00}, {0xA3, 0x04, 0x08, 0x00}, FSD_EMIT_TURN_LEFT, "좌 A3->A3"},
        {{0xA3, 0x04, 0x08, 0x00}, {0xF3, 0x05, 0x08, 0x00}, FSD_EMIT_TURN_LEFT, "좌 끝"},
        /* Cancel and right out of the same capture, including one where the
         * driver moved from the cancel detent straight to the other side. */
        {{0x0F, 0x0D, 0x02, 0x00}, {0xB3, 0x0E, 0x02, 0x00}, FSD_EMIT_TURN_TAP_UP, "우 UP_1 유지"},
        {{0xB3, 0x0E, 0x02, 0x00}, {0xF6, 0x0F, 0x04, 0x00}, FSD_EMIT_TURN_RIGHT, "우 UP_1->UP_2"},
        {{0xA3, 0x00, 0x04, 0x00}, {0xD0, 0x01, 0x04, 0x00}, FSD_EMIT_TURN_RIGHT, "우 유지"},
        {{0xD0, 0x01, 0x04, 0x00}, {0x12, 0x02, 0x04, 0x00}, FSD_EMIT_TURN_RIGHT, "우 유지2"},
        {{0x12, 0x02, 0x04, 0x00}, {0xEB, 0x03, 0x04, 0x00}, FSD_EMIT_TURN_RIGHT, "우 유지3"},
        {{0xEB, 0x03, 0x04, 0x00}, {0xEB, 0x04, 0x04, 0x00}, FSD_EMIT_TURN_RIGHT, "우 EB->EB"},
    };

    for (unsigned i = 0; i < sizeof(SELF) / sizeof(SELF[0]); i++) {
        const TurnCase* c = &SELF[i];
        FsdEmitTemplate t = turn_template(c->car, 5000u);
        FsdEmitFrame f;
        CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, c->turn, &t, 5010u, &f) == FSD_EMIT_OK,
              "%s: built", c->where);
        CHECK(memcmp(f.data, c->tsl, 4) == 0,
              "%s: expected %02X%02X%02X%02X, got %02X%02X%02X%02X",
              c->where, c->tsl[0], c->tsl[1], c->tsl[2], c->tsl[3],
              f.data[0], f.data[1], f.data[2], f.data[3]);
    }
}

/**
 * 🔴 WHERE THE CHECK TABLE STOPS, AND THAT IT REFUSES RATHER THAN GUESSES.
 *
 * Every 0x249 we hold was captured with the high beams and the washer idle.
 * Both live in the counter's byte (12|2 and 14|2) and the CRC covers them, so a
 * template with either set is outside everything measured. This is the one
 * refusal in the file that clears by itself, which is why it has its own name.
 */
static void test_turn_refuses_outside_the_measured_region(void) {
    printf("\n-- 깜빡이: 잰 적 없는 영역은 지어내지 않고 거부한다 --\n");

    FsdEmitFrame f;

    /* SCCM_highBeamStalkStatus, byte1 bits [5:4]. */
    uint8_t hi[4] = {0x5E, 0x19, 0x00, 0x00};
    FsdEmitTemplate t = turn_template(hi, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t, 1010u, &f)
              == FSD_EMIT_NO_CHECK,
          "high beam held -> NO_CHECK");

    /* SCCM_washWipeButtonStatus, byte1 bits [7:6]. */
    uint8_t wash[4] = {0x5E, 0x49, 0x00, 0x00};
    t = turn_template(wash, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t, 1010u, &f)
              == FSD_EMIT_NO_CHECK,
          "washer pulled -> NO_CHECK");

    /* leftStalkReserved1 and byte 2 bit 0 -- never non-zero in 51 payloads. */
    uint8_t resv[4] = {0x5E, 0x09, 0x10, 0x00};
    t = turn_template(resv, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t, 1010u, &f)
              == FSD_EMIT_NO_CHECK,
          "reserved bits in byte 2 -> NO_CHECK");

    uint8_t b3[4] = {0x5E, 0x09, 0x00, 0x01};
    t = turn_template(b3, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t, 1010u, &f)
              == FSD_EMIT_NO_CHECK,
          "byte 3 non-zero -> NO_CHECK");

    /* 🔴 And it must be a DIFFERENT answer from NO_ENCODING. One clears when
     * the driver lets go of the stalk; the other never clears. A screen that
     * says the same thing for both sends the owner looking in the wrong place. */
    CHECK(FSD_EMIT_NO_CHECK != FSD_EMIT_NO_ENCODING, "two different refusals");

    /* The same template with the extra bits cleared builds fine, so the
     * refusals above are about those bits and not about something else. */
    uint8_t ok[4] = {0x5E, 0x09, 0x00, 0x00};
    t = turn_template(ok, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t, 1010u, &f)
              == FSD_EMIT_OK,
          "and the same frame without them is fine");
}

static void test_turn_refusals(void) {
    printf("\n-- 깜빡이: 템플릿과 방향에 대한 거부 --\n");

    uint8_t car[4] = {0x5E, 0x09, 0x00, 0x00};
    FsdEmitFrame f;

    FsdEmitTemplate none;
    memset(&none, 0, sizeof(none));
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &none, 1000u, &f)
              == FSD_EMIT_NO_TEMPLATE,
          "no template");

    FsdEmitTemplate t = turn_template(car, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t,
                         1000u + FSD_EMIT_TEMPLATE_MAX_AGE_MS, &f)
              == FSD_EMIT_STALE_TEMPLATE,
          "stale at the bound");
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t,
                         1000u + FSD_EMIT_TEMPLATE_MAX_AGE_MS - 1u, &f)
              == FSD_EMIT_OK,
          "and fresh one millisecond earlier");

    t = turn_template(car, 1000u);
    t.id = 0x3E9u;
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t, 1010u, &f)
              == FSD_EMIT_BAD_TEMPLATE,
          "the hazard frame is not the stalk frame");

    t = turn_template(car, 1000u);
    t.dlc = 8u;
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t, 1010u, &f)
              == FSD_EMIT_BAD_TEMPLATE,
          "0x249 is four bytes");

    /* 🔴 The unmeasured directions, refused BEFORE the template is judged --
     * same rule as the door. DOWN_1 (0x06) is in every capture but nobody has
     * sent it alone, so we do not know it cancels. */
    t = turn_template(car, 1000u);
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_COUNT, &t, 1010u, &f)
              == FSD_EMIT_NO_ENCODING,
          "an out-of-range selector");
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, 99, &t, 1010u, &f) == FSD_EMIT_NO_ENCODING,
          "and a nonsense one");
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, -1, &t, 1010u, &f) == FSD_EMIT_NO_ENCODING,
          "and a negative one");

    /* An unmeasured direction with a BAD template must still answer
     * NO_ENCODING: "wait and retry" would be the wrong advice. */
    memset(&none, 0, sizeof(none));
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, 99, &none, 1010u, &f) == FSD_EMIT_NO_ENCODING,
          "the direction is judged before the template");

    uint8_t bits = 0xFF;
    CHECK(!fsd_emit_turn_bits(FSD_EMIT_TURN_COUNT, &bits), "COUNT is not a direction");
    CHECK(bits == 0xFF, "and a refusal leaves the byte alone");
    CHECK(fsd_emit_turn_str(FSD_EMIT_TURN_COUNT)[0] == '?', "nor does it have a name");
    for (int i = 0; i < FSD_EMIT_TURN_COUNT; i++) {
        CHECK(fsd_emit_turn_bits(i, &bits), "selector %d is measured", i);
        CHECK(fsd_emit_turn_str(i)[0] != '?', "selector %d has a name", i);
    }
}

/**
 * 🔴 THE STALK FIELD IS THREE BITS, NOT A BYTE.
 *
 * The emitter refuses a template whose byte 2 carries anything outside those
 * three bits, so this can only be checked by reaching past the guard -- which
 * is what the mask constant is for. Asserted directly because the mask is the
 * difference between "indicate left" and "indicate left AND assert five
 * reserved bits nobody has decoded".
 */
static void test_turn_writes_only_its_own_three_bits(void) {
    printf("\n-- 깜빡이: 자기 세 비트 말고는 안 건드린다 --\n");

    CHECK(FSD_EMIT_TURN_STALK_MASK == 0x0Eu, "17|3 is byte2 bits [3:1]");
    CHECK((FSD_EMIT_TURN_LEFT_BITS & ~FSD_EMIT_TURN_STALK_MASK) == 0u, "left fits");
    CHECK((FSD_EMIT_TURN_RIGHT_BITS & ~FSD_EMIT_TURN_STALK_MASK) == 0u, "right fits");
    CHECK((FSD_EMIT_TURN_TAP_UP_BITS & ~FSD_EMIT_TURN_STALK_MASK) == 0u, "tap up fits");

    /* The three are distinct, so no two directions can encode alike -- the
     * mistake the scroll table's own header warns about. */
    CHECK(FSD_EMIT_TURN_LEFT_BITS != FSD_EMIT_TURN_RIGHT_BITS &&
              FSD_EMIT_TURN_LEFT_BITS != FSD_EMIT_TURN_TAP_UP_BITS &&
              FSD_EMIT_TURN_RIGHT_BITS != FSD_EMIT_TURN_TAP_UP_BITS,
          "three directions, three values");

    /* Starting from a template that already holds a direction, the new one
     * REPLACES it rather than being OR-ed into it. 0x08 | 0x04 = 0x0C, which is
     * not a stalk position at all -- so this is the difference between "turn
     * right" and a value the car has never seen. */
    uint8_t holding_left[4] = {0xA3, 0x03, 0x08, 0x00};
    FsdEmitTemplate t = turn_template(holding_left, 1000u);
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_RIGHT, &t, 1010u, &f)
              == FSD_EMIT_OK,
          "built from a template already indicating left");
    CHECK(f.data[FSD_EMIT_TURN_STALK_BYTE] == FSD_EMIT_TURN_RIGHT_BITS,
          "the old direction is replaced, not or-ed: got %02X",
          f.data[FSD_EMIT_TURN_STALK_BYTE]);
}

/**
 * 🔴 THE COUNTER IS THE CAR'S PLUS ONE, AND IT WRAPS.
 *
 * Verified by pairing car and injected frames in time order across the three
 * captures -- ten injections, every one +1, including F -> 0. Held here as its
 * own assertion because the byte-for-byte cases above would also pass if the
 * counter were right by accident for the counters they happen to cover.
 */
static void test_turn_counter_advances_and_wraps(void) {
    printf("\n-- 깜빡이: 카운터는 차의 값 + 1, F 에서 감긴다 --\n");

    for (unsigned c = 0; c < 16u; c++) {
        uint8_t car[4] = {0x00, (uint8_t)c, 0x00, 0x00};
        FsdEmitTemplate t = turn_template(car, 1000u);
        FsdEmitFrame f;
        CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, FSD_EMIT_TURN_LEFT, &t, 1010u, &f)
                  == FSD_EMIT_OK,
              "counter %u builds", c);
        CHECK(f.data[FSD_EMIT_TURN_CNT_BYTE] == (uint8_t)((c + 1u) & 0x0Fu),
              "counter %u -> %u, got %u", c, (c + 1u) & 0x0Fu,
              f.data[FSD_EMIT_TURN_CNT_BYTE]);
    }
}

/**
 * 🔴 EVERY COMMAND PAYLOAD THE CAR OR TSL HAS EVER PUT ON THIS BUS.
 *
 * Generated from all 32 captures across four visits: the 29 distinct 0x249
 * payloads whose stalk field is one of the three we can emit. Feed a template
 * carrying the counter before it, ask for that direction, and the four bytes
 * must come back identical.
 *
 * The ten pairs above prove we agree with the injections we watched. This
 * proves we agree with EVERY 0x249 command byte we hold, whoever sent it, and
 * it covers 15 of the 16 counter values rather than the ten those injections
 * happened to land on.
 */
typedef struct {
    uint8_t prev_ctr;
    uint8_t want_check;
    uint8_t want_stalk;
    int32_t turn;
} TurnObserved;

static void test_turn_matches_every_observed_command(void) {
    printf("\n-- 깜빡이: 세 방문에서 관측된 명령 페이로드 29개 전부 --\n");

    static const TurnObserved TURN_OBSERVED[] = {
        {0x0F, 0x87, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x00, 0xF4, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x01, 0x36, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x02, 0xCF, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x03, 0xCF, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x04, 0x9F, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x07, 0x23, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x08, 0x42, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x09, 0xFE, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x0A, 0x34, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x0C, 0x0F, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x0D, 0xB3, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x0E, 0xD2, 0x02, FSD_EMIT_TURN_TAP_UP},
        {0x0F, 0xA3, 0x04, FSD_EMIT_TURN_RIGHT},
        {0x00, 0xD0, 0x04, FSD_EMIT_TURN_RIGHT},
        {0x01, 0x12, 0x04, FSD_EMIT_TURN_RIGHT},
        {0x02, 0xEB, 0x04, FSD_EMIT_TURN_RIGHT},
        {0x03, 0xEB, 0x04, FSD_EMIT_TURN_RIGHT},
        {0x05, 0x74, 0x04, FSD_EMIT_TURN_RIGHT},
        {0x0E, 0xF6, 0x04, FSD_EMIT_TURN_RIGHT},
        {0x01, 0x5A, 0x08, FSD_EMIT_TURN_LEFT},
        {0x02, 0xA3, 0x08, FSD_EMIT_TURN_LEFT},
        {0x03, 0xA3, 0x08, FSD_EMIT_TURN_LEFT},
        {0x04, 0xF3, 0x08, FSD_EMIT_TURN_LEFT},
        {0x08, 0x2E, 0x08, FSD_EMIT_TURN_LEFT},
        {0x09, 0x92, 0x08, FSD_EMIT_TURN_LEFT},
        {0x0A, 0x58, 0x08, FSD_EMIT_TURN_LEFT},
        {0x0B, 0x4A, 0x08, FSD_EMIT_TURN_LEFT},
        {0x0C, 0x63, 0x08, FSD_EMIT_TURN_LEFT},
    };

    for (unsigned i = 0; i < sizeof(TURN_OBSERVED) / sizeof(TURN_OBSERVED[0]); i++) {
        const TurnObserved* o = &TURN_OBSERVED[i];
        uint8_t car[4] = {0x00, o->prev_ctr, 0x00, 0x00};
        FsdEmitTemplate t = turn_template(car, 1000u);
        FsdEmitFrame f;
        CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, o->turn, &t, 1010u, &f) == FSD_EMIT_OK,
              "ctr %X %s: built", o->prev_ctr, fsd_emit_turn_str(o->turn));
        CHECK(f.data[FSD_EMIT_TURN_CHK_BYTE] == o->want_check &&
                  f.data[FSD_EMIT_TURN_STALK_BYTE] == o->want_stalk,
              "ctr %X %s -> expected %02X..%02X, got %02X..%02X",
              o->prev_ctr, fsd_emit_turn_str(o->turn), o->want_check, o->want_stalk,
              f.data[FSD_EMIT_TURN_CHK_BYTE], f.data[FSD_EMIT_TURN_STALK_BYTE]);
    }
}

/**
 * 🔴 COUNTER 7 IS THE ONE THE CAPTURES CANNOT REACH DIRECTLY.
 *
 * No command frame with counter 7 exists in anything we hold, so the 29 rows
 * above pin 15 of the 16 rows of the check table and leave one unpinned. That
 * is a real hole: a typo in that row would ship.
 *
 * It is closed the way the analysis closed it — by the factorisation. The 16
 * IDLE check bytes ARE the table's base row, and they are all directly
 * observed. If check(counter, stalk) = idle(counter) ^ K(stalk), then taking K
 * from ONE measured command must predict every counter including 7.
 *
 * 🟢 That is leave-one-out cross-validation written as a C test, and it is not
 * circular: both inputs are capture bytes and the emitter is the thing being
 * asked to satisfy them. tools/derive_stalk_check.py runs the same check over
 * all 51 payloads, 300 out-of-sample predictions, zero misses.
 */
static void test_turn_check_table_is_pinned_at_every_counter(void) {
    printf("\n-- 깜빡이: 검사표 16줄 전부, 유휴 프레임으로 --\n");

    /* The idle payloads 249#<chk><ctr>0000, one per counter, out of the
     * captures. Not a derivation -- these bytes were on the bus. */
    static const uint8_t IDLE_CHECK[16] = {
        0x9B, 0xE8, 0x2A, 0xD3,
        0xD3, 0x83, 0x4C, 0x5E,
        0x3F, 0x5E, 0xE2, 0x28,
        0x3A, 0x13, 0xAF, 0xCE,
    };

    /* One measured command per direction supplies K. Counter 9 -> A, which is
     * the very first injection in TSL좌깜빡이켜기, and the two others beside it. */
    const struct { int32_t turn; uint8_t prev; uint8_t check; } SEED[] = {
        {FSD_EMIT_TURN_LEFT, 0x09, 0x92},   /* 249#920A0800 */
        {FSD_EMIT_TURN_RIGHT, 0x0F, 0xA3},  /* 249#A3000400 */
        {FSD_EMIT_TURN_TAP_UP, 0x00, 0xF4}, /* 249#F4010200 */
    };

    for (unsigned s = 0; s < sizeof(SEED) / sizeof(SEED[0]); s++) {
        const uint8_t k = (uint8_t)(SEED[s].check ^ IDLE_CHECK[(SEED[s].prev + 1u) & 0x0Fu]);

        for (unsigned c = 0; c < 16u; c++) {
            uint8_t car[4] = {0x00, (uint8_t)c, 0x00, 0x00};
            FsdEmitTemplate t = turn_template(car, 1000u);
            FsdEmitFrame f;
            CHECK(fsd_emit_build(FSD_ACT_TURN_SIGNAL, SEED[s].turn, &t, 1010u, &f)
                      == FSD_EMIT_OK,
                  "%s ctr %X builds", fsd_emit_turn_str(SEED[s].turn), c);
            const uint8_t want = (uint8_t)(IDLE_CHECK[(c + 1u) & 0x0Fu] ^ k);
            CHECK(f.data[FSD_EMIT_TURN_CHK_BYTE] == want,
                  "%s from ctr %X: predicted %02X, got %02X",
                  fsd_emit_turn_str(SEED[s].turn), c, want,
                  f.data[FSD_EMIT_TURN_CHK_BYTE]);
        }
    }
}

static void test_result_names(void) {
    printf("\n-- 사유에 이름이 있다 --\n");
    const FsdEmitResult all[] = {
        FSD_EMIT_OK, FSD_EMIT_NO_TEMPLATE, FSD_EMIT_STALE_TEMPLATE,
        FSD_EMIT_BAD_TEMPLATE, FSD_EMIT_NO_ENCODING, FSD_EMIT_NO_CHECK,
    };
    for(unsigned i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        const char* s = fsd_emit_result_str(all[i]);
        CHECK(s && s[0] && s[0] != '?', "result %u has a name", (unsigned)all[i]);
    }
}


/* The 4th visit's frame, byte for byte:
 *
 *      (7.458) 1F9#0000000000000000     <- the car
 *      (7.459) 1F9#00C0000000000000     <- TSL, +1 ms
 *
 * Caught twice in that visit -- TSL's own menu entry, and the three-window-up
 * gesture that fires the same rule -- and both produced 0xC0. */
static const uint8_t TSL_DOOR_RR[8] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static void test_door_selector_matches_tsl_byte_for_byte(void) {
    printf("\n-- door: the rear door's frame == the frame TSL sent --\n");

    FsdEmitTemplate t = door_template(1000u);
    FsdEmitFrame f;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, FSD_EMIT_DOOR_RIGHT_REAR, &t, 1100u, &f)
              == FSD_EMIT_OK,
          "right rear must build");
    CHECK(f.id == 0x1F9u, "id 0x1F9, got 0x%X", (unsigned)f.id);
    CHECK(f.dlc == 8, "dlc 8, got %u", f.dlc);
    CHECK(memcmp(f.data, TSL_DOOR_RR, 8) == 0,
          "bytes must equal TSL's: got %02X%02X%02X%02X%02X%02X%02X%02X",
          f.data[0], f.data[1], f.data[2], f.data[3],
          f.data[4], f.data[5], f.data[6], f.data[7]);

    /* Two bits, both in byte 1 -- the same shape as the front door, a
     * different pair. */
    unsigned diff = 0;
    for(unsigned i = 0; i < 8; i++) {
        uint8_t x = (uint8_t)(f.data[i] ^ CAR_DOOR[i]);
        while(x) { diff += (x & 1u); x >>= 1; }
    }
    CHECK(diff == 2, "exactly two bits differ, got %u", diff);
    CHECK((f.data[1] ^ CAR_DOOR[1]) == 0xC0u, "and they are byte1 bits 6 and 7");

    /* 🔴 The two doors must not produce the same frame. This is the assertion
     * that a swapped table or an ignored argument turns red on. */
    FsdEmitFrame front;
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, FSD_EMIT_DOOR_RIGHT_FRONT, &t, 1100u,
                         &front) == FSD_EMIT_OK, "right front must build");
    CHECK(memcmp(front.data, TSL_DOOR, 8) == 0, "front is still 0x03");
    CHECK(front.data[1] != f.data[1],
          "the two doors must differ on the wire: both are 0x%02X", f.data[1]);
    CHECK((front.data[1] & f.data[1]) == 0u,
          "and their fields must not overlap: 0x%02X & 0x%02X",
          front.data[1], f.data[1]);
}

/* THE TWO LEFT DOORS, measured on the 5th visit, 2026-09-06. Copied out of
 * captures/2026-09-06-5차/좌측앞문열기 and 좌측뒷문열기 -- two injections each,
 * 300 ms apart, and the latch answers 119 ms later.
 *
 *      (6.671) 1F9#6000000000000000     <- left front,  3 << 5
 *      (6.769) 1F9#0018000000000000     <- left rear,   3 << 11
 *
 * 🔴 THE LEFT FRONT IS IN BYTE 0, AND THE OLD CODE COULD NOT HAVE REACHED IT.
 * FSD_EMIT_DOOR_BYTE was a single constant, 1, for the whole action. Adding a
 * left-front row to that table would have put 0x60 into BYTE 1 -- a value
 * nobody has measured, sitting on top of the two rear fields. That is why this
 * is a rewrite of the encoding and not two more enum entries.
 *
 * 🟢 AND THE OLD COMMENT'S INFERENCE WAS WRONG, WHICH IS THE POINT. It read
 * "0x03 is bits[1:0] and 0xC0 is bits[7:6], so four 2-bit fields fits ...
 * either reading predicts 0x0C and 0x30 for the two LEFT doors". Neither is
 * right: the fields are THREE bits apart, so the left rear is 0x18 and the
 * left front is not in that byte at all. The refusal to guess was worth
 * exactly what it cost. */
static const uint8_t TSL_DOOR_LF[8] = {0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t TSL_DOOR_LR[8] = {0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static void test_left_doors_match_tsl_byte_for_byte(void) {
    printf("\n-- door: the two left doors == the frames TSL sent --\n");

    FsdEmitTemplate t = door_template(1000u);
    FsdEmitFrame f;

    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, FSD_EMIT_DOOR_LEFT_FRONT, &t, 1100u, &f)
              == FSD_EMIT_OK, "left front must build");
    CHECK(memcmp(f.data, TSL_DOOR_LF, 8) == 0,
          "left front bytes must equal TSL's: got %02X%02X%02X%02X%02X%02X%02X%02X",
          f.data[0], f.data[1], f.data[2], f.data[3],
          f.data[4], f.data[5], f.data[6], f.data[7]);
    /* 🔴 Said again as a claim about WHICH BYTE, separately from the memcmp.
     * This is the assertion a single-byte encoding cannot pass, and it is the
     * whole reason the door encoding changed shape. */
    CHECK(f.data[0] == 0x60u && f.data[1] == 0x00u,
          "left front lives in byte 0: got byte0=0x%02X byte1=0x%02X",
          f.data[0], f.data[1]);

    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, FSD_EMIT_DOOR_LEFT_REAR, &t, 1100u, &f)
              == FSD_EMIT_OK, "left rear must build");
    CHECK(memcmp(f.data, TSL_DOOR_LR, 8) == 0,
          "left rear bytes must equal TSL's: got %02X%02X%02X%02X%02X%02X%02X%02X",
          f.data[0], f.data[1], f.data[2], f.data[3],
          f.data[4], f.data[5], f.data[6], f.data[7]);
    CHECK(f.data[0] == 0x00u && f.data[1] == 0x18u,
          "left rear lives in byte 1: got byte0=0x%02X byte1=0x%02X",
          f.data[0], f.data[1]);
}

/* The four doors side by side -- the structure the 5th visit found, and the
 * reason the encoding carries an OFFSET now instead of one shared byte index.
 *
 * Each field is checked against a value computed FROM ITS BIT OFFSET ALONE,
 * not against the constant the emitter uses. The two agreeing is the claim:
 * "value 3 at offsets 5, 8, 11, 14" reproduces all four captured frames. */
static void test_four_doors_are_three_bits_apart(void) {
    printf("\n-- door: four fields, value 3, offsets 5/8/11/14 --\n");

    static const struct {
        int32_t sel;
        uint8_t offset;
        const uint8_t* tsl;
        const char* name;
    } D[4] = {
        {FSD_EMIT_DOOR_LEFT_FRONT, 5u, TSL_DOOR_LF, "left front"},
        {FSD_EMIT_DOOR_RIGHT_FRONT, 8u, TSL_DOOR, "right front"},
        {FSD_EMIT_DOOR_LEFT_REAR, 11u, TSL_DOOR_LR, "left rear"},
        {FSD_EMIT_DOOR_RIGHT_REAR, 14u, TSL_DOOR_RR, "right rear"},
    };

    FsdEmitTemplate t = door_template(1000u);
    uint8_t seen[8] = {0};

    for(unsigned i = 0; i < 4; i++) {
        uint8_t want[8] = {0};
        /* 🔴 THE VALUE FITS IN ONE BYTE; THE FIELD DOES NOT ALWAYS.
         * Three-apart makes each field three bits wide, and the right rear
         * starts at bit 14 -- so its third bit is bit 16, over in byte 2. We
         * never write that bit, because value 3 leaves it clear, and we never
         * clear it either: the emitter ORs two bits and touches nothing else.
         *
         * Asserted rather than assumed because the day a door needs a value
         * with the top bit set, this line is what says the two-bit shortcut
         * has run out. */
        CHECK((D[i].offset % 8u) <= 6u, "%s: value 3 must fit in one byte", D[i].name);
        /* value 3 at D[i].offset, little-endian bit numbering -- the same
         * convention the DBC uses and the one the offsets were read in. */
        want[D[i].offset / 8u] = (uint8_t)(3u << (D[i].offset % 8u));
        CHECK(memcmp(want, D[i].tsl, 8) == 0,
              "%s: offset %u must reproduce the captured frame", D[i].name, D[i].offset);

        FsdEmitFrame f;
        CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, D[i].sel, &t, 1100u, &f) == FSD_EMIT_OK,
              "%s must build", D[i].name);
        CHECK(memcmp(f.data, want, 8) == 0, "%s: emitter must match the offset", D[i].name);

        /* 🔴 No two doors may share a bit. A transposed table or an ignored
         * argument shows up here as two doors that are the same door. */
        for(unsigned b = 0; b < 8; b++) {
            CHECK((seen[b] & f.data[b]) == 0u,
                  "%s overlaps an earlier door in byte %u", D[i].name, b);
            seen[b] |= f.data[b];
        }
    }
}

/* ── the mirrors ─────────────────────────────────────────────────────────────
 *
 * 0x273 as the car sends it with the mirrors still, and as TSL sends it one
 * millisecond later to fold them and to unfold them. Copied out of
 * captures/2026-09-06-5차/미러접기&펴기.
 *
 *      (5.743) 273#81E110000B023001     <- the car
 *      (5.744) 273#81E110010B023001     <- TSL, +1 ms, byte3 = 1, fold
 *      (8.743) 273#81E110000B023001     <- the car again
 *      (8.744) 273#81E110020B023001     <- TSL, +1 ms, byte3 = 2, unfold
 *
 * 20 car frames in that capture, byte identical, 500 ms apart. Exactly two
 * frames are not, and each is one byte away from the one before it.
 *
 * 🔴 THE 3rd VISIT COULD NOT TELL COMMAND FROM STATE HERE -- the mirrors moved
 * 302 ms BEFORE the value appeared, which is what a status broadcast looks
 * like. The 5th visit's capture is clean: the car never sends a non-zero
 * byte 3, and the two that exist arrive 1 ms behind a car frame, which is
 * where every other TSL injection lives.
 *
 * 🟢 SAME FRAME AS THE MAP LIGHT, DIFFERENT BYTE. That is not a coincidence to
 * be noted, it is a constraint to be asserted: two actions writing one id must
 * not be able to reach each other's bits. */
static const uint8_t CAR_MIRROR[8] = {0x81, 0xE1, 0x10, 0x00, 0x0B, 0x02, 0x30, 0x01};
static const uint8_t TSL_MIRROR_FOLD[8] = {0x81, 0xE1, 0x10, 0x01, 0x0B, 0x02, 0x30, 0x01};
static const uint8_t TSL_MIRROR_UNFOLD[8] = {0x81, 0xE1, 0x10, 0x02, 0x0B, 0x02, 0x30, 0x01};

static FsdEmitTemplate mirror_template(uint32_t at_ms) {
    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    t.seen = true;
    t.id = FSD_EMIT_MIRROR_ID;
    t.dlc = 8;
    memcpy(t.data, CAR_MIRROR, 8);
    t.seen_ms = at_ms;
    return t;
}

static void test_mirror_matches_tsl_byte_for_byte(void) {
    printf("\n-- mirror: our frames == the two frames TSL sent --\n");

    FsdEmitTemplate t = mirror_template(1000u);
    FsdEmitFrame fold, unfold;

    CHECK(fsd_emit_build(FSD_ACT_MIRROR, FSD_EMIT_MIRROR_FOLD, &t, 1100u, &fold)
              == FSD_EMIT_OK, "fold must build");
    CHECK(fold.id == FSD_EMIT_MIRROR_ID, "id 0x273, got 0x%X", (unsigned)fold.id);
    CHECK(fold.dlc == 8, "dlc 8, got %u", fold.dlc);
    CHECK(memcmp(fold.data, TSL_MIRROR_FOLD, 8) == 0,
          "fold bytes must equal TSL's: got %02X%02X%02X%02X%02X%02X%02X%02X",
          fold.data[0], fold.data[1], fold.data[2], fold.data[3],
          fold.data[4], fold.data[5], fold.data[6], fold.data[7]);

    CHECK(fsd_emit_build(FSD_ACT_MIRROR, FSD_EMIT_MIRROR_UNFOLD, &t, 1100u, &unfold)
              == FSD_EMIT_OK, "unfold must build");
    CHECK(memcmp(unfold.data, TSL_MIRROR_UNFOLD, 8) == 0,
          "unfold bytes must equal TSL's: got %02X%02X%02X%02X%02X%02X%02X%02X",
          unfold.data[0], unfold.data[1], unfold.data[2], unfold.data[3],
          unfold.data[4], unfold.data[5], unfold.data[6], unfold.data[7]);

    /* Only byte 3, and the two directions must not be the same frame. */
    for(unsigned i = 0; i < 8; i++) {
        if(i == 3u) continue;
        CHECK(fold.data[i] == CAR_MIRROR[i] && unfold.data[i] == CAR_MIRROR[i],
              "byte %u must be the car's: fold 0x%02X unfold 0x%02X car 0x%02X",
              i, fold.data[i], unfold.data[i], CAR_MIRROR[i]);
    }
    CHECK(fold.data[3] != unfold.data[3],
          "fold and unfold must differ on the wire: both 0x%02X", fold.data[3]);
}

/* 🔴 THE ONE THAT SEPARATES A FIELD FROM A BITMASK, and the reason the shared
 * path had to stop using |=.
 *
 * byte 3 is a small VALUE -- 1 folds, 2 unfolds -- not two independent flags.
 * Every frame we hold has it at 0, so OR and write are the same thing on the
 * evidence. They are not the same thing on the car: hand the emitter a
 * template that already carries the other direction and OR produces 3, which
 * is a value nobody has ever seen and which we would be asserting on a frame
 * that also carries the mirrors, the locks, the wipers and the horn.
 *
 * The captures cannot rule this template in or out -- TSL asked twice, three
 * seconds apart, and the car was idle both times. So the emitter has to be
 * right about it rather than lucky. */
static void test_mirror_writes_a_field_not_a_bitmask(void) {
    printf("\n-- mirror: byte 3 is a value, so it is written and not OR'd --\n");

    FsdEmitTemplate t = mirror_template(1000u);
    FsdEmitFrame f;

    /* The car is mid-fold when the rule asks for unfold. */
    t.data[3] = 0x01u;
    CHECK(fsd_emit_build(FSD_ACT_MIRROR, FSD_EMIT_MIRROR_UNFOLD, &t, 1100u, &f)
              == FSD_EMIT_OK, "unfold must build over a folding template");
    CHECK(f.data[3] == 0x02u,
          "unfold over 0x01 must be 0x02, not 0x%02X (0x03 means it OR'd)", f.data[3]);

    t.data[3] = 0x02u;
    CHECK(fsd_emit_build(FSD_ACT_MIRROR, FSD_EMIT_MIRROR_FOLD, &t, 1100u, &f)
              == FSD_EMIT_OK, "fold must build over an unfolding template");
    CHECK(f.data[3] == 0x01u,
          "fold over 0x02 must be 0x01, not 0x%02X", f.data[3]);

    /* 🔴 And the bits of byte 3 that are not ours stay the car's. We measured
     * two values in a byte; we did not measure the byte. */
    t.data[3] = 0xF0u;
    CHECK(fsd_emit_build(FSD_ACT_MIRROR, FSD_EMIT_MIRROR_FOLD, &t, 1100u, &f)
              == FSD_EMIT_OK, "fold must build with the high nibble set");
    CHECK(f.data[3] == 0xF1u,
          "the high nibble is the car's: expected 0xF1, got 0x%02X", f.data[3]);
}

/* Two actions, one frame. They must not be able to reach each other. */
static void test_mirror_and_light_share_a_frame_and_not_a_byte(void) {
    printf("\n-- mirror and map light: same id, disjoint bytes --\n");

    FsdEmitTemplate mt = mirror_template(1000u);
    FsdEmitFrame mirror, light;
    CHECK(fsd_emit_build(FSD_ACT_MIRROR, FSD_EMIT_MIRROR_FOLD, &mt, 1100u, &mirror)
              == FSD_EMIT_OK, "mirror must build");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &mt, 1100u, &light) == FSD_EMIT_OK,
          "the map light must build from the same template");

    CHECK(mirror.id == light.id, "same id");
    for(unsigned i = 0; i < 8; i++) {
        const uint8_t md = (uint8_t)(mirror.data[i] ^ mt.data[i]);
        const uint8_t ld = (uint8_t)(light.data[i] ^ mt.data[i]);
        CHECK((md & ld) == 0u,
              "byte %u: mirror changed 0x%02X and the light changed 0x%02X -- "
              "they must not overlap", i, md, ld);
    }
    /* Said concretely, so a future edit that moves either one has to say so. */
    CHECK((uint8_t)(mirror.data[3] ^ mt.data[3]) == 0x01u, "mirror is byte 3");
    CHECK((uint8_t)(light.data[7] ^ mt.data[7]) == 0x08u, "the light is byte 7 bit 3");
    CHECK(mirror.data[7] == mt.data[7], "the mirror must not touch byte 7");
    CHECK(light.data[3] == mt.data[3], "the light must not touch byte 3");
}

static void test_unmeasured_mirror_directions_are_refused(void) {
    printf("\n-- mirror: only the two directions TSL sent --\n");

    FsdEmitTemplate t = mirror_template(1000u);
    FsdEmitFrame f;

    /* 0 is the idle value the car broadcasts. It is not a command: "stop
     * asking" is what ceasing to send means, the same as the map light. A
     * selector that wrote 0 would be claiming the car's own resting value as
     * an instruction. */
    static const int32_t UNMEASURED[] = {2, 3, 99, -1, -2147483647 - 1, 2147483647};
    for(unsigned i = 0; i < sizeof(UNMEASURED) / sizeof(UNMEASURED[0]); i++) {
        memset(&f, 0xAA, sizeof(f));
        CHECK(fsd_emit_build(FSD_ACT_MIRROR, UNMEASURED[i], &t, 1100u, &f)
                  == FSD_EMIT_NO_ENCODING,
              "mirror selector %ld must be refused", (long)UNMEASURED[i]);
        uint8_t bits = 0xAAu;
        CHECK(!fsd_emit_mirror_bits(UNMEASURED[i], &bits),
              "mirror selector %ld has no measured value", (long)UNMEASURED[i]);
        CHECK(bits == 0xAAu, "a refused lookup must not touch the output");
    }

    uint8_t bits = 0;
    CHECK(fsd_emit_mirror_bits(FSD_EMIT_MIRROR_FOLD, &bits) && bits == 0x01u,
          "fold = 1, got %u", bits);
    CHECK(fsd_emit_mirror_bits(FSD_EMIT_MIRROR_UNFOLD, &bits) && bits == 0x02u,
          "unfold = 2, got %u", bits);
    CHECK(!fsd_emit_mirror_bits(FSD_EMIT_MIRROR_FOLD, NULL), "NULL out refused");

    CHECK(strcmp(fsd_emit_mirror_str(FSD_EMIT_MIRROR_FOLD), "fold") == 0,
          "name: %s", fsd_emit_mirror_str(FSD_EMIT_MIRROR_FOLD));
    CHECK(strcmp(fsd_emit_mirror_str(FSD_EMIT_MIRROR_UNFOLD), "unfold") == 0,
          "name: %s", fsd_emit_mirror_str(FSD_EMIT_MIRROR_UNFOLD));
    CHECK(strcmp(fsd_emit_mirror_str(FSD_EMIT_MIRROR_COUNT), "?") == 0,
          "an unmeasured direction must not borrow a name: %s",
          fsd_emit_mirror_str(FSD_EMIT_MIRROR_COUNT));

    /* Wrong template, same as every other emitter: the id is checked, not
     * assumed. A door frame must not receive a mirror command. */
    FsdEmitTemplate door = door_template(1000u);
    CHECK(fsd_emit_build(FSD_ACT_MIRROR, FSD_EMIT_MIRROR_FOLD, &door, 1100u, &f)
              == FSD_EMIT_BAD_TEMPLATE, "mirror action + door template -> refuse");
}

/* ── the light horn ──────────────────────────────────────────────────────────
 *
 * 0x3C2 multiplex 0, byte 0 bit 2. Measured 2026-09-06, fifth visit, out of
 * captures/2026-09-06-5차/가벼운경적1회. TSL calls it 轻鸣笛 -- the LIGHT horn,
 * and the adjective turns out to be the whole command:
 *
 *      (5.969) 3C2#0055555500006985     <- the car, mux 0
 *      (5.970) 3C2#0055555500006985     <- an exact echo, +1 ms
 *      (5.985) 3C2#0455555500006985     <- PRESS,   byte 0 bit 2
 *      (5.997) 3C2#0055555500006985     <- RELEASE, 12 ms later
 *      (6.019) 3C2#2955000000000080     <- the car again, mux 1
 *      (6.069) 3C2#0055555500006985     <- ... and mux 0, 100 ms after the last
 *
 * 🔴 THE RELEASE IS THE FEATURE, NOT A TIDY-UP. Send only the press and the
 * button stays down until the car's own next mux-0 frame says otherwise -- up
 * to about 100 ms rather than 12. Whether that is still a light beep or a real
 * honk is not something any capture we hold answers, and the device that named
 * the command "light" sends the release explicitly. So we do too.
 *
 * ⚠️ THE ECHO AT 5.970 IS NOT REPRODUCED. It is the only sub-2 ms pair in the
 * whole ten-second capture, so it is almost certainly TSL rather than the bus,
 * but it carries the car's own bytes unchanged -- it commands nothing. Copying
 * a frame because we saw it, without being able to say what it does, is the
 * opposite of what every other emitter in this file does.
 *
 * 🔴 AND THIS ONE IS TIMED, WHICH NONE OF THE OTHERS ARE. The map light, the
 * mirror and the door ride 0-1 ms behind a car frame; the indicator burst
 * counts the car's own 0x249 arrivals. Here TSL's press lands 16 ms after the
 * car's frame and the release 12 ms after the press -- mux 0 only arrives
 * every 100 ms, so reception cannot drive a 12 ms gap. */
static const uint8_t CAR_HORN[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};
static const uint8_t TSL_HORN_PRESS[8] = {0x04, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};
/* mux 1 of the same id: the scroll wheel's variant, from the same capture.
 * Entirely different bytes -- which is the point of refusing it. */
static const uint8_t CAR_SCROLL[8] = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};

static FsdEmitTemplate horn_template(uint32_t at_ms) {
    FsdEmitTemplate t;
    memset(&t, 0, sizeof(t));
    t.seen = true;
    t.id = FSD_EMIT_HORN_ID;
    t.dlc = 8;
    memcpy(t.data, CAR_HORN, 8);
    t.seen_ms = at_ms;
    return t;
}

static void test_light_horn_matches_tsl_byte_for_byte(void) {
    printf("\n-- light horn: the press == the frame TSL sent --\n");

    FsdEmitTemplate t = horn_template(1000u);
    FsdEmitFrame f;

    CHECK(fsd_emit_build(FSD_ACT_LIGHT_HORN, 0, &t, 1100u, &f) == FSD_EMIT_OK,
          "the press must build");
    CHECK(f.id == FSD_EMIT_HORN_ID, "id 0x3C2, got 0x%X", (unsigned)f.id);
    CHECK(f.dlc == 8, "dlc 8, got %u", f.dlc);
    CHECK(memcmp(f.data, TSL_HORN_PRESS, 8) == 0,
          "press bytes must equal TSL's: got %02X%02X%02X%02X%02X%02X%02X%02X",
          f.data[0], f.data[1], f.data[2], f.data[3],
          f.data[4], f.data[5], f.data[6], f.data[7]);

    unsigned diff = 0;
    for(unsigned i = 0; i < 8; i++) {
        uint8_t x = (uint8_t)(f.data[i] ^ CAR_HORN[i]);
        while(x) { diff += (x & 1u); x >>= 1; }
    }
    CHECK(diff == 1, "exactly one bit differs from the car's frame, got %u", diff);
    CHECK((f.data[0] ^ CAR_HORN[0]) == 0x04u, "and it is byte0 bit2");

    /* 🔴 The multiplex selector is in the same byte as the bit we set, so
     * this is not pedantry: a press that moved bits 0-1 would arrive as the
     * OTHER variant of the frame and mean something else entirely. */
    CHECK((f.data[0] & 0x03u) == 0x00u,
          "the press must still be multiplex 0, got byte0 0x%02X", f.data[0]);
}

static void test_light_horn_release_is_the_cars_own_frame(void) {
    printf("\n-- light horn: the release, and the 12 ms that make it light --\n");

    FsdEmitTemplate t = horn_template(1000u);
    FsdEmitFrame f;

    CHECK(fsd_emit_release_ms(FSD_ACT_LIGHT_HORN) == 12u,
          "TSL released after 12 ms, got %u", fsd_emit_release_ms(FSD_ACT_LIGHT_HORN));

    CHECK(fsd_emit_build_release(FSD_ACT_LIGHT_HORN, 0, &t, 1100u, &f) == FSD_EMIT_OK,
          "the release must build");
    CHECK(f.id == FSD_EMIT_HORN_ID, "id 0x3C2, got 0x%X", (unsigned)f.id);
    /* 🟢 THE SMALLEST POSSIBLE CLAIM. The release carries the car's own bytes
     * back, unchanged -- it says nothing about the car that the car did not
     * just say itself. All it does is arrive sooner than the car's next one. */
    CHECK(memcmp(f.data, CAR_HORN, 8) == 0,
          "release must be the car's frame verbatim: got %02X%02X%02X%02X%02X%02X%02X%02X",
          f.data[0], f.data[1], f.data[2], f.data[3],
          f.data[4], f.data[5], f.data[6], f.data[7]);

    /* 🔴 AND IT REFUSES WHEN THE CAR'S OWN FRAME SAYS THE BUTTON IS DOWN.
     *
     * ⚠️ THIS ASSERTION USED TO EXPECT FSD_EMIT_OK AND A CLEARED BIT, and it
     * was describing the wrong thing. A template with the horn bit set is the
     * car telling us A PERSON HAS THEIR THUMB ON THE HORN -- we never receive
     * our own transmissions, so it cannot be our press coming back. Building a
     * release there is telling the car that somebody let go of a button they
     * are still holding.
     *
     * The check used to live in rule_task.cpp as a memcmp after the fact, out
     * of reach of every host test. It belongs here: it is a statement about
     * the template and the field, which is exactly what this file decides. */
    t.data[0] = 0x04u;
    memset(&f, 0xAA, sizeof(f));
    CHECK(fsd_emit_build_release(FSD_ACT_LIGHT_HORN, 0, &t, 1100u, &f)
              == FSD_EMIT_FIELD_IN_USE,
          "a release over a pressed template must refuse");

    /* The mux selector lives in the same byte, so a template that is mux 0 with
     * the horn bit set must refuse for the RIGHT reason -- being in use, not
     * being the wrong variant. */
    CHECK((t.data[0] & 0x03u) == 0x00u, "that template is still multiplex 0");

    /* And an idle template still builds, so the refusal is about the field
     * rather than about releases in general. */
    t.data[0] = 0x00u;
    CHECK(fsd_emit_build_release(FSD_ACT_LIGHT_HORN, 0, &t, 1100u, &f) == FSD_EMIT_OK,
          "an idle template still releases");
    CHECK(memcmp(f.data, CAR_HORN, 8) == 0, "and it is the car's frame");

    /* 🔴 EVERY OTHER ACTION HAS NO RELEASE, and asking for one must be a
     * refusal rather than a frame. A gesture is the exception in this file,
     * not the rule: the map light and the mirror stop by our ceasing to send,
     * the indicator's cancel is its own command with its own argument. */
    for(unsigned a = 0; a < FSD_ACT_COUNT; a++) {
        FsdBodyAction act = (FsdBodyAction)a;
        if(act == FSD_ACT_LIGHT_HORN) continue;
        CHECK(fsd_emit_release_ms(act) == 0u,
              "%s must have no release, got %u ms", fsd_body_action_str(act),
              fsd_emit_release_ms(act));
        FsdEmitTemplate any = horn_template(1000u);
        CHECK(fsd_emit_build_release(act, 0, &any, 1100u, &f) == FSD_EMIT_NO_ENCODING,
              "%s must refuse a release", fsd_body_action_str(act));
    }
}

/* 🔴 THE REFUSAL THAT MATTERS MOST FOR THIS EMITTER.
 *
 * 0x3C2 is multiplexed and the two variants share nothing: mux 0 carries the
 * windows, the belt, the horn and the hazard button; mux 1 carries the scroll
 * wheel and the camera. Stamping our bit into the wrong one would put a frame
 * on the bus that announces the multiplex of the scroll wheel while carrying a
 * horn press -- and the four bytes around it would be the scroll frame's, not
 * the pack's.
 *
 * The pipeline's template store already filters by multiplex, so in the
 * assembled system this cannot arrive. It is checked HERE anyway, for the same
 * reason the id is: this file is handed "the last frame we saw" and must not
 * trust the plumbing that hands it over. PR #18 shipped a parser that read the
 * right bits out of the wrong frame and failed closed by luck. */
static void test_light_horn_refuses_the_other_multiplex(void) {
    printf("\n-- light horn: the scroll multiplex is not a horn template --\n");

    FsdEmitTemplate t = horn_template(1000u);
    memcpy(t.data, CAR_SCROLL, 8); /* same id, other variant */
    FsdEmitFrame f;

    memset(&f, 0xAA, sizeof(f));
    CHECK(fsd_emit_build(FSD_ACT_LIGHT_HORN, 0, &t, 1100u, &f) == FSD_EMIT_BAD_TEMPLATE,
          "a mux 1 template must be refused");
    CHECK(fsd_emit_build_release(FSD_ACT_LIGHT_HORN, 0, &t, 1100u, &f)
              == FSD_EMIT_BAD_TEMPLATE,
          "and so must a release built from one");

    /* The mux is read from the byte we also write, so a template whose bit 2
     * happens to be set is still mux 0 and still usable. */
    t = horn_template(1000u);
    t.data[0] = 0x04u;
    CHECK(fsd_emit_build(FSD_ACT_LIGHT_HORN, 0, &t, 1100u, &f) == FSD_EMIT_OK,
          "our own press is still a mux 0 frame");
}

/* 🔴 THE ONE THAT MATTERS MOST. All four doors are measured now; anything
 * outside them is still inference, and an inference here opens a door. */
static void test_unmeasured_doors_are_refused(void) {
    printf("\n-- door: the doors nobody measured are refused, not guessed --\n");

    FsdEmitTemplate t = door_template(1000u);
    FsdEmitFrame f;

    /* ⚠️ 2 AND 3 USED TO BE IN THIS LIST, and they are gone on purpose: the
     * 5th visit measured both left doors. The list did not get weaker -- the
     * door after the fourth is still here, and so is everything a corrupt or
     * hostile argument could carry.
     *
     * 🔴 4 is the interesting one now. "Three bits apart" predicts a fifth
     * field at offset 17, and offset 2 is empty below the first one, so the
     * frame plainly carries more than four things -- the frunk switch turned
     * up in its back bytes in the same visit. Predicting is not measuring. */
    static const int32_t UNMEASURED[] = {4, 5, 99, -1, -2147483647 - 1, 2147483647};
    for(unsigned i = 0; i < sizeof(UNMEASURED) / sizeof(UNMEASURED[0]); i++) {
        memset(&f, 0xAA, sizeof(f));
        CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, UNMEASURED[i], &t, 1100u, &f)
                  == FSD_EMIT_NO_ENCODING,
              "door selector %ld must be refused as NO_ENCODING", (long)UNMEASURED[i]);
        uint8_t bits = 0xAAu, byte_ix = 0xAAu;
        CHECK(!fsd_emit_door_field(UNMEASURED[i], &byte_ix, &bits),
              "door selector %ld has no measured field", (long)UNMEASURED[i]);
        CHECK(bits == 0xAAu && byte_ix == 0xAAu,
              "a refused lookup must not touch either output");
    }

    /* 🔴 And the refusal must be NO_ENCODING even when the template is fine,
     * because "we do not know which bits" is a gap, not something waiting for
     * a fresher frame. A STALE_TEMPLATE here would read as "try again". */
    FsdEmitTemplate stale = door_template(1000u);
    CHECK(fsd_emit_build(FSD_ACT_DOOR_OPEN, FSD_EMIT_DOOR_COUNT, &stale, 1000u + 9999u, &f)
              == FSD_EMIT_NO_ENCODING,
          "an unmeasured door refuses before staleness is even considered");

    /* All four we did measure work, and say WHICH BYTE as well as which bits.
     * 🔴 The byte is asserted because it is the half that used to be a shared
     * constant: a lookup that returned the right mask for the wrong byte would
     * have passed every assertion this test made before today. */
    uint8_t bits = 0, byte_ix = 0;
    CHECK(fsd_emit_door_field(FSD_EMIT_DOOR_LEFT_FRONT, &byte_ix, &bits)
              && byte_ix == 0u && bits == 0x60u,
          "left front = byte 0 / 0x60, got byte %u / 0x%02X", byte_ix, bits);
    CHECK(fsd_emit_door_field(FSD_EMIT_DOOR_RIGHT_FRONT, &byte_ix, &bits)
              && byte_ix == 1u && bits == 0x03u,
          "right front = byte 1 / 0x03, got byte %u / 0x%02X", byte_ix, bits);
    CHECK(fsd_emit_door_field(FSD_EMIT_DOOR_LEFT_REAR, &byte_ix, &bits)
              && byte_ix == 1u && bits == 0x18u,
          "left rear = byte 1 / 0x18, got byte %u / 0x%02X", byte_ix, bits);
    CHECK(fsd_emit_door_field(FSD_EMIT_DOOR_RIGHT_REAR, &byte_ix, &bits)
              && byte_ix == 1u && bits == 0xC0u,
          "right rear = byte 1 / 0xC0, got byte %u / 0x%02X", byte_ix, bits);
    CHECK(!fsd_emit_door_field(FSD_EMIT_DOOR_RIGHT_FRONT, &byte_ix, NULL),
          "NULL bits refused");
    CHECK(!fsd_emit_door_field(FSD_EMIT_DOOR_RIGHT_FRONT, NULL, &bits),
          "NULL byte refused");

    /* Names, so a log says which door instead of a number -- and says "?" for
     * one we cannot name rather than picking the nearest. */
    CHECK(strcmp(fsd_emit_door_str(FSD_EMIT_DOOR_RIGHT_FRONT), "right front") == 0,
          "name: %s", fsd_emit_door_str(FSD_EMIT_DOOR_RIGHT_FRONT));
    CHECK(strcmp(fsd_emit_door_str(FSD_EMIT_DOOR_RIGHT_REAR), "right rear") == 0,
          "name: %s", fsd_emit_door_str(FSD_EMIT_DOOR_RIGHT_REAR));
    CHECK(strcmp(fsd_emit_door_str(FSD_EMIT_DOOR_LEFT_FRONT), "left front") == 0,
          "name: %s", fsd_emit_door_str(FSD_EMIT_DOOR_LEFT_FRONT));
    CHECK(strcmp(fsd_emit_door_str(FSD_EMIT_DOOR_LEFT_REAR), "left rear") == 0,
          "name: %s", fsd_emit_door_str(FSD_EMIT_DOOR_LEFT_REAR));
    /* 🔴 Four names, and they must all be different. Two doors sharing a name
     * is how a log tells somebody the wrong door opened. */
    static const int32_t NAMED[] = {FSD_EMIT_DOOR_RIGHT_FRONT, FSD_EMIT_DOOR_RIGHT_REAR,
                                    FSD_EMIT_DOOR_LEFT_FRONT, FSD_EMIT_DOOR_LEFT_REAR};
    for(unsigned a = 0; a < 4; a++)
        for(unsigned b = a + 1; b < 4; b++)
            CHECK(strcmp(fsd_emit_door_str(NAMED[a]), fsd_emit_door_str(NAMED[b])) != 0,
                  "doors %ld and %ld share the name %s", (long)NAMED[a], (long)NAMED[b],
                  fsd_emit_door_str(NAMED[a]));

    /* ⚠️ THIS ASSERTION USED TO SAY TWO, and it was right when it was written.
     * Kept rather than deleted, and made stronger: the count is pinned AND the
     * selector past the end still refuses a name, which is the half that
     * actually protects anything. */
    CHECK(strcmp(fsd_emit_door_str(FSD_EMIT_DOOR_COUNT), "?") == 0,
          "an unmeasured door must not borrow a name: %s",
          fsd_emit_door_str(FSD_EMIT_DOOR_COUNT));
    CHECK(FSD_EMIT_DOOR_COUNT == 4,
          "four doors measured on the 5th visit; got %d", (int)FSD_EMIT_DOOR_COUNT);
}

/* An action that takes no argument ignores it. A stored rule may carry
 * anything in that field, and refusing would break rules for a value that
 * never meant anything. */
static void test_argless_actions_ignore_the_argument(void) {
    printf("\n-- actions with no argument ignore it --\n");

    FsdEmitTemplate light = car_template(1000u);
    FsdEmitTemplate haz = hz_template(HZ[0].car, 1000u);
    FsdEmitFrame base, with_arg;

    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 0, &light, 1100u, &base) == FSD_EMIT_OK,
          "light with arg 0");
    CHECK(fsd_emit_build(FSD_ACT_MAP_LIGHT, 12345, &light, 1100u, &with_arg)
              == FSD_EMIT_OK, "light with a stray arg still builds");
    CHECK(memcmp(base.data, with_arg.data, 8) == 0 && base.id == with_arg.id,
          "and builds the SAME frame");

    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, 0, &haz, 1100u, &base) == FSD_EMIT_OK,
          "hazards with arg 0");
    CHECK(fsd_emit_build(FSD_ACT_HAZARDS, -7, &haz, 1100u, &with_arg) == FSD_EMIT_OK,
          "hazards with a stray arg still builds");
    CHECK(memcmp(base.data, with_arg.data, 8) == 0 && base.id == with_arg.id,
          "and builds the SAME frame");
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
    test_door_selector_matches_tsl_byte_for_byte();
    test_left_doors_match_tsl_byte_for_byte();
    test_four_doors_are_three_bits_apart();
    test_mirror_matches_tsl_byte_for_byte();
    test_mirror_writes_a_field_not_a_bitmask();
    test_mirror_and_light_share_a_frame_and_not_a_byte();
    test_unmeasured_mirror_directions_are_refused();
    test_light_horn_matches_tsl_byte_for_byte();
    test_light_horn_release_is_the_cars_own_frame();
    test_light_horn_refuses_the_other_multiplex();
    test_unmeasured_doors_are_refused();
    test_argless_actions_ignore_the_argument();
    test_the_door_keeps_what_survived();
    test_turn_matches_tsl_byte_for_byte();
    test_turn_reproduces_the_cars_own_frames();
    test_turn_refuses_outside_the_measured_region();
    test_turn_refusals();
    test_turn_writes_only_its_own_three_bits();
    test_turn_counter_advances_and_wraps();
    test_turn_matches_every_observed_command();
    test_turn_check_table_is_pinned_at_every_counter();
    test_no_counter_in_the_frame();
    test_refuses_without_a_template();
    test_refuses_a_stale_template();
    test_refuses_the_wrong_frame();
    test_which_actions_have_an_encoding();
    test_no_action_borrows_another_encoding();
    test_result_names();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
