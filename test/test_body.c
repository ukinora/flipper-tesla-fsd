/*
 * test_body.c — 몸통 동작 표와 ID 거부 목록.
 *
 * 🔴🔴 이 파일은 2026-09-10 에 1,221 줄에서 이만큼으로 줄었다. 차주 지시로
 * **차의 안전게이트를 전부 지웠기** 때문이다:
 *
 *     "차의 모든 안전게이트관련사항을 삭제해라.
 *      필요하다면 추후 내가 하나씩 추가하겠다."
 *
 * 없어진 것: 권한 축(`fsd_body_allows`) · 그 입력(`FsdBodyInputs`) · 거부 이름
 * (`FsdBodyVerdict`) · 능력 행의 게이트 필드 일곱 · 모드와 송신 허용.
 * 그것들을 시험하던 600여 건이 함께 없어졌다 — **시험할 것이 없어졌기
 * 때문이지 시험을 포기한 것이 아니다.**
 *
 * 🟢 남은 것은 게이트가 아니라 **표의 무결성과 그물**이다:
 *   - 능력 행이 자기 자리를 가리키는가 (`action == index`)
 *   - 동작마다 사람이 읽을 이름이 있는가 (네 번째 패턴)
 *   - 절대 나가면 안 되는 ID 를 여전히 거부하는가 (D)
 *
 * ⚠️ **게이트가 없다는 것 자체는 여기서 시험하지 않는다.** 그것은
 * `test_pipeline.c` 의 `test_no_gate_stands_in_front_of_a_press()` 가 한다 —
 * 가장 험한 상황을 한꺼번에 만들어 놓고 그래도 나가는지 묻고, 게이트가 하나라도
 * 되살아나면 빨개진다. 2026-09-08 에 사람을 묻는 게이트를 없앨 때 세운 규율
 * 그대로다: **없앤 것은 시험이 반대를 단언하게 해서 못 돌아오게 한다.**
 */

#include <stdio.h>
#include <string.h>

#include "fsd_body.h"
#include "fsd_body_t1.h"
#include "fsd_body_t2.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (cond) {                                       \
            g_pass++;                                     \
        } else {                                          \
            g_fail++;                                     \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
        }                                                 \
    } while (0)

/* ── 표의 무결성 ─────────────────────────────────────────────────────── */

/* 🔴 A row that does not sit at its own index is the failure this check
 * exists for: FSD_BODY_CAPS is indexed by the action, and the action number is
 * written into NVS by fsd_rule_pack(). One misplaced row turns every stored
 * rule about that action into a rule about another one, with nothing on screen
 * to say so. */
static void test_every_row_names_itself(void) {
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const FsdBodyCaps *c = fsd_body_caps((FsdBodyAction)a);
        CHECK(c != NULL, "action %d has no row", a);
        if (!c) continue;
        CHECK(c->action == (FsdBodyAction)a,
              "row %d claims to be action %d", a, (int)c->action);
    }
}

static void test_out_of_range_has_no_row(void) {
    CHECK(fsd_body_caps((FsdBodyAction)FSD_ACT_COUNT) == NULL,
          "one past the end must be NULL, not the next thing in memory");
    CHECK(fsd_body_caps((FsdBodyAction)200) == NULL, "well past the end too");
}

/* 🔴 The fourth pattern: an enum name must never reach the screen. The owner
 * is not a developer and there is no PC in the car. */
static void test_every_action_has_a_readable_name(void) {
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const char *n = fsd_body_action_str((FsdBodyAction)a);
        CHECK(n != NULL && n[0] != '\0', "action %d has no name", a);
        CHECK(n == NULL || n[0] != '?', "action %d falls through to '?'", a);
    }
    CHECK(fsd_body_action_str((FsdBodyAction)FSD_ACT_COUNT)[0] == '?',
          "an unknown action says so rather than naming a neighbour");
}

/* 🟢 max_hold_ms survived the removal on purpose. It is not a question about
 * the car's situation -- it is what stops one stuck flag from transmitting
 * forever, the same family as the chokepoint. */
static void test_hold_bounds_are_sane(void) {
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const FsdBodyCaps *c = fsd_body_caps((FsdBodyAction)a);
        if (!c) continue;
        /* ⚠️ 30 s IS THE REAL CEILING AND IT IS DELIBERATE -- the map light
         * and the hazards are things that are SUPPOSED to stay on, and a
         * hazard light with a five-second leash is not a hazard light. The
         * point of the bound is that one exists at all, not that it is small.
         *
         * 🔴 This assertion was written as <= 10000 and the table said 30000.
         * That is this repository's SECOND PATTERN in miniature: a test that
         * pins a number the author guessed rather than the one that was
         * decided. Caught on the first run; corrected to the measured ceiling
         * rather than the wished-for one. */
        CHECK(c->max_hold_ms <= 30000u,
              "action %d may be held %u ms — a held body frame needs a leash",
              a, (unsigned)c->max_hold_ms);
    }
}

/* ── 그물: 절대 나가면 안 되는 ID ────────────────────────────────────── */

/* 🔴 THIS IS NOT A GATE AND THAT IS WHY IT SURVIVED. It does not ask what the
 * car is doing; it says three ids may never leave this module whatever built
 * them. 0x3F5 is the lighting frame, 0x102/0x103 the door status frames -- all
 * three are things the CAR says about itself, so we have no reason to make one
 * and every reason to refuse if something ever does. */
static void test_the_deny_list_still_refuses(void) {
    CHECK(fsd_body_tx_id_refused(0x3F5u), "0x3F5 lighting stays refused");
    CHECK(fsd_body_tx_id_refused(0x102u), "0x102 door status stays refused");
    CHECK(fsd_body_tx_id_refused(0x103u), "0x103 door status stays refused");
}

/* 🔴 The frames we DO build must not be on it, or the feature is impossible.
 * 0x273 in particular: it is what the map light and the mirror write, and its
 * protection is the bit chokepoint, not a blanket refusal. */
static void test_the_deny_list_does_not_eat_our_own_frames(void) {
    CHECK(!fsd_body_tx_id_refused(0x273u), "0x273 is the map light and the mirror");
    CHECK(!fsd_body_tx_id_refused(0x249u), "0x249 is the turn signal and the wiper");
    CHECK(!fsd_body_tx_id_refused(0x1F9u), "0x1F9 is the door open command");
    CHECK(!fsd_body_tx_id_refused(0x3E9u), "0x3E9 is the hazards");
    CHECK(!fsd_body_tx_id_refused(0x3C2u), "0x3C2 is the light horn and the scroll");
}

/* ── T1 감지기 — 재는 것이지 쏘는 것이 아니다 ────────────────────────── */

/* 🔴 T1 used to ask the axis and store the refusal. The axis is gone, so it
 * reports the edge and nothing stands in front of that -- which is right,
 * because T1 has never transmitted anything. It counts. */
static void test_t1_starts_closed(void) {
    FsdT1 t;
    fsd_t1_init(&t);
    CHECK(fsd_t1_tick(&t, 1000u) == FSD_T1_ACT_NONE,
          "a detector that has seen no door reports no edge");
    CHECK(fsd_t1_window_count(&t) == 0u, "and has taken nothing");
    CHECK(fsd_t1_latch_raw(&t, FSD_BODY_SIDE_LEFT) == 0xFFu, "unseen reads 0xFF");
}

int main(void) {
    test_every_row_names_itself();
    test_out_of_range_has_no_row();
    test_every_action_has_a_readable_name();
    test_hold_bounds_are_sane();
    test_the_deny_list_still_refuses();
    test_the_deny_list_does_not_eat_our_own_frames();
    test_t1_starts_closed();

    printf("test_body: 표의 무결성 · ID 거부 목록 · T1 감지기\n\n");
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
