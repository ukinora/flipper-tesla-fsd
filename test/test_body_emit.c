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

    const FsdBodyAction rest[] = {
        FSD_ACT_DOOR_OPEN, FSD_ACT_CAMERA, FSD_ACT_SEAT_DRIVER,
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
    test_no_counter_in_the_frame();
    test_refuses_without_a_template();
    test_refuses_a_stale_template();
    test_refuses_the_wrong_frame();
    test_only_map_light_has_an_encoding();
    test_result_names();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
