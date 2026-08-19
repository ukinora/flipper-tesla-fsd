/*
 * test_btn_j6.c — host tests for fsd_logic/fsd_btn_j6.c.
 *
 * EVERY BYTE SEQUENCE BELOW WAS CAPTURED FROM THE REMOTE on 2026-08-18, not
 * invented. That is the point: a hand-written vector proves the decoder agrees
 * with me, and a captured one proves it agrees with the device.
 *
 * Full measurement and how it was taken: 블루투스-버튼-조사.md
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_btn_j6.h"

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

/* ── captured bursts ──────────────────────────────────────────────────────────
 *
 * Each row is one notification. A swipe arrives as up to nine of them; the
 * report path used to keep only the last one per slot, which is exactly why
 * these were invisible until fork PR #59. */

#define ROW 5

/* 1번 — 아래로 쓸기. X 는 290 언저리에 머물고 Y 만 200 → 999 */
static const uint8_t B1_DOWN[][ROW] = {
    {0x03, 0x2C, 0x01, 0xC8, 0x00}, {0x03, 0x18, 0x01, 0x2C, 0x01},
    {0x03, 0x2C, 0x01, 0x90, 0x01}, {0x03, 0x18, 0x01, 0xF4, 0x01},
    {0x03, 0x2C, 0x01, 0x58, 0x02}, {0x03, 0x18, 0x01, 0xBC, 0x02},
    {0x03, 0x22, 0x01, 0x84, 0x03}, {0x00, 0x22, 0x01, 0xE7, 0x03},
    {0x00, 0x22, 0x01, 0xE7, 0x03},
};

/* 2번 — 위로 쓸기, 짧은 판 (Y 300 → 0). 같은 버튼이 회차마다 다른 높이에서
 * 시작한다 — 그래서 판정 근거가 시작 좌표가 아니라 **변위**여야 한다. */
static const uint8_t B2_UP_SHORT[][ROW] = {
    {0x03, 0x18, 0x01, 0x2C, 0x01}, {0x03, 0x22, 0x01, 0x64, 0x00},
    {0x00, 0x22, 0x01, 0x00, 0x00}, {0x00, 0x22, 0x01, 0x00, 0x00},
};

/* 2번 — 위로 쓸기, 긴 판 (Y 800 → 0) */
static const uint8_t B2_UP_LONG[][ROW] = {
    {0x03, 0x2C, 0x01, 0x20, 0x03}, {0x03, 0x18, 0x01, 0xBC, 0x02},
    {0x03, 0x2C, 0x01, 0x58, 0x02}, {0x03, 0x18, 0x01, 0xF4, 0x01},
    {0x03, 0x2C, 0x01, 0x90, 0x01}, {0x03, 0x18, 0x01, 0x2C, 0x01},
    {0x03, 0x22, 0x01, 0x64, 0x00}, {0x00, 0x22, 0x01, 0x00, 0x00},
    {0x00, 0x22, 0x01, 0x00, 0x00},
};

/* 3번 — 오른쪽으로 쓸기. Y 는 390/400 을 오가고 X 만 150 → 450 */
static const uint8_t B3_RIGHT[][ROW] = {
    {0x03, 0x96, 0x00, 0x86, 0x01}, {0x03, 0xC8, 0x00, 0x90, 0x01},
    {0x03, 0xFA, 0x00, 0x86, 0x01}, {0x03, 0x2C, 0x01, 0x90, 0x01},
    {0x03, 0x5E, 0x01, 0x86, 0x01}, {0x03, 0x90, 0x01, 0x90, 0x01},
    {0x03, 0xC2, 0x01, 0x86, 0x01}, {0x00, 0xC2, 0x01, 0x90, 0x01},
};

/* 4번 — 왼쪽으로 쓸기 (X 400 → 50) */
static const uint8_t B4_LEFT[][ROW] = {
    {0x03, 0x90, 0x01, 0x86, 0x01}, {0x03, 0x5E, 0x01, 0x90, 0x01},
    {0x03, 0x2C, 0x01, 0x86, 0x01}, {0x03, 0xC8, 0x00, 0x90, 0x01},
    {0x03, 0x96, 0x00, 0x86, 0x01}, {0x03, 0x64, 0x00, 0x90, 0x01},
    {0x03, 0x32, 0x00, 0x86, 0x01}, {0x00, 0x32, 0x00, 0x90, 0x01},
};

/* 6번 — 탭 (420, 850). 누름과 뗌이 **따로** 온다 */
static const uint8_t B6_PRESS[ROW] = {0x03, 0xA4, 0x01, 0x52, 0x03};
static const uint8_t B6_RELEASE[ROW] = {0x00, 0xA4, 0x01, 0x52, 0x03};

/* 7번 — 탭 (300, 500) */
static const uint8_t B7_PRESS[ROW] = {0x03, 0x2C, 0x01, 0xF4, 0x01};
static const uint8_t B7_RELEASE[ROW] = {0x00, 0x2C, 0x01, 0xF4, 0x01};

/* 컨슈머 컨트롤 — 3바이트 비트맵 */
static const uint8_t C_B3_LONG[3] = {0x00, 0x08, 0x00};
static const uint8_t C_B4_LONG[3] = {0x00, 0x10, 0x00};
static const uint8_t C_B7_LONG[3] = {0x00, 0x40, 0x00};
static const uint8_t C_AMBIG_A[3] = {0x00, 0x80, 0x00}; // 1번 길게 = 5번 톡
static const uint8_t C_AMBIG_B[3] = {0x00, 0x00, 0x01}; // 2번 길게 = 5번 톡
static const uint8_t C_RELEASE[3] = {0x00, 0x00, 0x00};

/* Feed a whole captured burst and return the ONE thing it produced. Anything
 * beyond one event is a defect in itself — a swipe is a single gesture, and a
 * decoder that fires per report would step the profile eight times. */
static FsdJ6Out feed_burst(FsdJ6* s, const uint8_t (*rows)[ROW], size_t n,
                           uint32_t at, int* fired) {
    FsdJ6Out last = {FSD_J6_NONE, FSD_J6_EDGE_NONE};
    *fired = 0;
    for (size_t i = 0; i < n; i++) {
        const FsdJ6Out o = fsd_j6_feed(s, rows[i], ROW, at + (uint32_t)(i * 10));
        if (o.edge != FSD_J6_EDGE_NONE) {
            (*fired)++;
            last = o;
        }
    }
    return last;
}

#define BURST(s, arr, at, fired) \
    feed_burst((s), (arr), sizeof(arr) / sizeof((arr)[0]), (at), (fired))

// ── the four swipes ──────────────────────────────────────────────────────────

static void test_swipes(void) {
    printf("쓸기 4방향\n");
    FsdJ6 s;
    int fired;
    FsdJ6Out o;

    fsd_j6_init(&s);
    o = BURST(&s, B1_DOWN, 1000, &fired);
    CHECK(o.btn == FSD_J6_B1, "1번: %d 이 나왔다", (int)o.btn);
    CHECK(o.edge == FSD_J6_EDGE_PULSE, "1번은 사건형이라 PULSE 여야 한다");
    CHECK(fired == 1, "쓸기 하나에 사건은 하나여야 하는데 %d 개", fired);

    fsd_j6_init(&s);
    o = BURST(&s, B2_UP_SHORT, 1000, &fired);
    CHECK(o.btn == FSD_J6_B2, "2번(짧은 판): %d", (int)o.btn);
    CHECK(fired == 1, "2번 짧은 판 사건 %d 개", fired);

    /* 같은 버튼, 다른 높이에서 시작. 시작 좌표로 판정했다면 여기서 갈라진다. */
    fsd_j6_init(&s);
    o = BURST(&s, B2_UP_LONG, 1000, &fired);
    CHECK(o.btn == FSD_J6_B2, "2번(긴 판)도 같은 버튼이어야 한다: %d", (int)o.btn);

    fsd_j6_init(&s);
    o = BURST(&s, B3_RIGHT, 1000, &fired);
    CHECK(o.btn == FSD_J6_B3, "3번: %d", (int)o.btn);
    CHECK(fired == 1, "3번 사건 %d 개", fired);

    fsd_j6_init(&s);
    o = BURST(&s, B4_LEFT, 1000, &fired);
    CHECK(o.btn == FSD_J6_B4, "4번: %d", (int)o.btn);
    CHECK(fired == 1, "4번 사건 %d 개", fired);
}

/* 반대 방향이 서로 다른 값이어야 한다. 부호를 잃으면 위아래가 같아진다. */
static void test_opposites_differ(void) {
    printf("반대 방향은 서로 다르다\n");
    CHECK(FSD_J6_B1 != FSD_J6_B2, "위/아래가 같은 값이다");
    CHECK(FSD_J6_B3 != FSD_J6_B4, "좌/우가 같은 값이다");
}

// ── the two taps ─────────────────────────────────────────────────────────────

/* 🔴 6번만 진짜 레벨이다. 유지 시간이 손을 따라온다 (0.12 / 1.97 / 5.43초 실측)
 * 므로, 누름을 **뗌까지 기다렸다가** 알리면 길게 누르기를 우리가 잴 수 없다. */
static void test_b6_is_level(void) {
    printf("6번 — 레벨\n");
    FsdJ6 s;
    fsd_j6_init(&s);

    FsdJ6Out o = fsd_j6_feed(&s, B6_PRESS, ROW, 1000);
    CHECK(o.btn == FSD_J6_B6, "6번 누름: %d", (int)o.btn);
    CHECK(o.edge == FSD_J6_EDGE_DOWN, "누르는 **순간** 알려야 한다");

    /* 5.4초를 쥐고 있어도 그 사이에는 아무 사건도 없다. */
    o = fsd_j6_feed(&s, B6_RELEASE, ROW, 6400);
    CHECK(o.btn == FSD_J6_B6, "6번 뗌: %d", (int)o.btn);
    CHECK(o.edge == FSD_J6_EDGE_UP, "뗌은 UP 이어야 한다");

    CHECK(fsd_j6_is_level(FSD_J6_B6), "6번은 레벨로 등록돼야 한다");
    CHECK(!fsd_j6_is_level(FSD_J6_B7), "7번은 레벨이 아니다");
    CHECK(!fsd_j6_is_level(FSD_J6_B1), "쓸기는 레벨이 아니다");
}

/* 7번은 잡고 있어도 누름·뗌이 한꺼번에 온다 — 사건형이다. */
static void test_b7_is_a_pulse(void) {
    printf("7번 — 사건형 탭\n");
    FsdJ6 s;
    fsd_j6_init(&s);

    FsdJ6Out o = fsd_j6_feed(&s, B7_PRESS, ROW, 1000);
    CHECK(o.edge == FSD_J6_EDGE_NONE, "탭은 뗄 때 판정된다 — 누름에 사건이 없어야");

    o = fsd_j6_feed(&s, B7_RELEASE, ROW, 1100);
    CHECK(o.btn == FSD_J6_B7, "7번: %d", (int)o.btn);
    CHECK(o.edge == FSD_J6_EDGE_PULSE, "7번은 PULSE 여야 한다");
}

/* 두 탭은 좌표로만 갈린다. 좌표를 안 보면 6번과 7번이 같은 버튼이 된다. */
static void test_taps_differ_by_coordinate(void) {
    printf("탭 둘은 좌표로 갈린다\n");
    FsdJ6 s;
    fsd_j6_init(&s);
    fsd_j6_feed(&s, B6_PRESS, ROW, 1000);
    const FsdJ6Out a = fsd_j6_feed(&s, B6_RELEASE, ROW, 1100);

    fsd_j6_init(&s);
    fsd_j6_feed(&s, B7_PRESS, ROW, 2000);
    const FsdJ6Out b = fsd_j6_feed(&s, B7_RELEASE, ROW, 2100);

    CHECK(a.btn != b.btn, "6번과 7번이 같은 값으로 나온다");
}

/* 모르는 자리의 탭은 버튼이 아니다 — 지어내지 않는다. */
static void test_unknown_tap_is_nothing(void) {
    printf("모르는 자리의 탭\n");
    FsdJ6 s;
    fsd_j6_init(&s);
    const uint8_t press[ROW] = {0x03, 0x10, 0x00, 0x10, 0x00};   // (16,16)
    const uint8_t release[ROW] = {0x00, 0x10, 0x00, 0x10, 0x00};
    fsd_j6_feed(&s, press, ROW, 1000);
    const FsdJ6Out o = fsd_j6_feed(&s, release, ROW, 1100);
    CHECK(o.btn == FSD_J6_NONE, "모르는 탭에 이름을 붙였다: %d", (int)o.btn);
}

// ── the consumer codes ───────────────────────────────────────────────────────

static void test_consumer_codes(void) {
    printf("컨슈머 코드\n");
    FsdJ6 s;
    fsd_j6_init(&s);
    FsdJ6Out o;

    o = fsd_j6_feed(&s, C_B3_LONG, 3, 1000);
    CHECK(o.btn == FSD_J6_B3_LONG && o.edge == FSD_J6_EDGE_PULSE, "00 08 00: %d", (int)o.btn);
    o = fsd_j6_feed(&s, C_RELEASE, 3, 1100);
    CHECK(o.edge == FSD_J6_EDGE_NONE, "놓임은 사건이 아니다");

    o = fsd_j6_feed(&s, C_B4_LONG, 3, 2000);
    CHECK(o.btn == FSD_J6_B4_LONG, "00 10 00: %d", (int)o.btn);
    fsd_j6_feed(&s, C_RELEASE, 3, 2100);

    o = fsd_j6_feed(&s, C_B7_LONG, 3, 3000);
    CHECK(o.btn == FSD_J6_B7_LONG, "00 40 00: %d", (int)o.btn);
}

/* 🔴 겹치는 둘은 버리는 것이 유일하게 안전한 답이다.
 *
 * 00 80 00 은 1번 길게이면서 5번 톡이고, 00 00 01 은 2번 길게이면서 5번 톡이다.
 * 5번은 전원 버튼을 겸하므로 실수로 눌릴 여지가 크고, 바이트가 같아 원리적으로
 * 구별할 수 없다. 매핑하면 전원 버튼을 스치는 것이 동작을 일으킨다. */
static void test_ambiguous_codes_are_refused(void) {
    printf("겹치는 코드는 거부\n");
    FsdJ6 s;
    fsd_j6_init(&s);

    FsdJ6Out o = fsd_j6_feed(&s, C_AMBIG_A, 3, 1000);
    CHECK(o.btn == FSD_J6_NONE, "00 80 00 을 버튼으로 읽었다: %d", (int)o.btn);
    CHECK(o.edge == FSD_J6_EDGE_NONE, "00 80 00 이 사건을 냈다");

    o = fsd_j6_feed(&s, C_AMBIG_B, 3, 2000);
    CHECK(o.btn == FSD_J6_NONE, "00 00 01 을 버튼으로 읽었다: %d", (int)o.btn);
}

// ── what must NOT be decoded ─────────────────────────────────────────────────

/* 되풀이되는 뗌 리포트가 두 번째 사건이 되면 안 된다. 실제로 1·2번의 캡처에
 * 마지막 줄이 두 번 들어 있다 — 지어낸 경우가 아니다. */
static void test_trailing_release_is_not_a_second_press(void) {
    printf("되풀이되는 뗌\n");
    FsdJ6 s;
    int fired;
    fsd_j6_init(&s);
    BURST(&s, B1_DOWN, 1000, &fired);
    CHECK(fired == 1, "뗌이 두 줄인데 사건이 %d 개 나왔다", fired);
}

static void test_wrong_lengths_are_ignored(void) {
    printf("길이가 다른 리포트\n");
    FsdJ6 s;
    fsd_j6_init(&s);
    const uint8_t batt[1] = {0x62};                    // 배터리 98%
    const uint8_t svc[4] = {0x01, 0x00, 0xFF, 0xFF};   // 서비스 변경
    CHECK(fsd_j6_feed(&s, batt, 1, 1000).edge == FSD_J6_EDGE_NONE, "1바이트가 통과했다");
    CHECK(fsd_j6_feed(&s, svc, 4, 1000).edge == FSD_J6_EDGE_NONE, "4바이트가 통과했다");
    CHECK(fsd_j6_feed(&s, NULL, 5, 1000).edge == FSD_J6_EDGE_NONE, "NULL 이 통과했다");
}

/* 5바이트라도 상태 바이트가 03/00 이 아니면 이 기기의 것이 아니다. */
static void test_bad_state_byte_is_ignored(void) {
    printf("상태 바이트가 이상한 5바이트\n");
    FsdJ6 s;
    fsd_j6_init(&s);
    const uint8_t junk[ROW] = {0x7F, 0x2C, 0x01, 0xF4, 0x01};
    CHECK(fsd_j6_feed(&s, junk, ROW, 1000).edge == FSD_J6_EDGE_NONE, "0x7F 가 통과했다");
}

/* 좌표는 1000×1000 정규화다. 넘는 값은 이 기기의 리포트가 아니다. */
static void test_out_of_range_coordinates_are_ignored(void) {
    printf("범위를 넘는 좌표\n");
    FsdJ6 s;
    fsd_j6_init(&s);
    const uint8_t wild[ROW] = {0x03, 0xFF, 0x7F, 0xFF, 0x7F}; // (32767, 32767)
    CHECK(fsd_j6_feed(&s, wild, ROW, 1000).edge == FSD_J6_EDGE_NONE, "32767 이 통과했다");
}

/* 🔴 뗌 리포트를 잃으면 다음 제스처가 **옛 시작점**에서 재어진다.
 *
 * ⚠️ 이 테스트의 첫 판은 헛것이었다 (2026-08-18). 위로 쓸다 만 뒤 아래로 쓰는
 * 조합을 골랐는데, 그 둘은 **옛 시작점으로 재도 우연히 같은 답**이 나온다 —
 * 가드를 통째로 무력화해도 통과했다. 돌연변이 검사가 아니었으면 "이 위험은
 * 막혀 있다" 고 적힌 채 남았을 것이다.
 *
 * 실제로 무는 조합은 이것이다. 위로 쓸기는 (300,800)에서 시작하고 7번 탭은
 * (300,500)에 찍히므로, 옛 시작점이 남아 있으면 변위가 -300 이 되어 **탭이
 * 위로 쓸기로 읽힌다.** 입력을 잃는 것이 아니라 **다른 버튼이 눌린 것**이 된다. */
static void test_stale_gesture_makes_a_tap_look_like_a_swipe(void) {
    printf("뗌을 잃은 제스처 — 탭이 쓸기로 둔갑\n");
    FsdJ6 s;
    fsd_j6_init(&s);

    /* 위로 쓸기의 앞부분만. 뗌이 오지 않았다 — 시작점 (300,800)이 남는다. */
    for (size_t i = 0; i < 4; i++)
        fsd_j6_feed(&s, B2_UP_LONG[i], ROW, 1000 + (uint32_t)(i * 10));

    /* 한참 뒤 7번을 탭한다. */
    fsd_j6_feed(&s, B7_PRESS, ROW, 60000);
    const FsdJ6Out o = fsd_j6_feed(&s, B7_RELEASE, ROW, 60100);
    CHECK(o.btn == FSD_J6_B7, "7번 탭이 %d 로 읽혔다", (int)o.btn);
}

/* 같은 결함의 더 나쁜 판: 레벨 버튼이 레벨이 아니게 된다.
 *
 * 6번은 **누르는 순간의 좌표**로 알아본다. 낡은 접촉이 남아 있으면 그 누름이
 * 통째로 무시되어 level 이 서지 않고, 뗌은 옛 시작점 기준의 쓸기가 된다 —
 * 즉 6번이 사라지고 엉뚱한 버튼이 하나 눌린다. */
static void test_stale_gesture_swallows_the_level_button(void) {
    printf("뗌을 잃은 제스처 — 레벨 버튼이 사라진다\n");
    FsdJ6 s;
    fsd_j6_init(&s);

    for (size_t i = 0; i < 4; i++)
        fsd_j6_feed(&s, B1_DOWN[i], ROW, 1000 + (uint32_t)(i * 10));

    const FsdJ6Out down = fsd_j6_feed(&s, B6_PRESS, ROW, 60000);
    CHECK(down.btn == FSD_J6_B6 && down.edge == FSD_J6_EDGE_DOWN,
          "6번 누름을 잃었다: btn=%d edge=%d", (int)down.btn, (int)down.edge);

    const FsdJ6Out up = fsd_j6_feed(&s, B6_RELEASE, ROW, 63000);
    CHECK(up.btn == FSD_J6_B6 && up.edge == FSD_J6_EDGE_UP,
          "6번 뗌이 %d/%d 로 나왔다", (int)up.btn, (int)up.edge);
}

/* 낡지 않은 제스처는 그대로 이어져야 한다 — 쓸기 중간의 리포트가 새 제스처로
 * 잘리면 방향을 잃는다. 위 둘과 반대 방향의 단언이라 시한을 0 으로 만들어도
 * 통과하는 일이 없다. */
static void test_gesture_in_progress_is_not_dropped(void) {
    printf("진행 중인 쓸기는 안 자른다\n");
    FsdJ6 s;
    int fired;
    fsd_j6_init(&s);
    const FsdJ6Out o = BURST(&s, B1_DOWN, 1000, &fired);
    CHECK(o.btn == FSD_J6_B1 && fired == 1,
          "진행 중인 쓸기가 잘렸다: btn=%d 사건=%d", (int)o.btn, fired);
}

/* 6번을 오래 쥐는 것은 낡은 제스처가 아니다 — 그건 정상 조작이다. */
static void test_level_hold_is_not_stale(void) {
    printf("6번은 오래 쥐어도 살아 있다\n");
    FsdJ6 s;
    fsd_j6_init(&s);
    fsd_j6_feed(&s, B6_PRESS, ROW, 1000);
    const FsdJ6Out o = fsd_j6_feed(&s, B6_RELEASE, ROW, 1000 + 120000u); // 2분
    CHECK(o.btn == FSD_J6_B6 && o.edge == FSD_J6_EDGE_UP,
          "2분 쥔 뒤의 뗌을 잃었다: btn=%d edge=%d", (int)o.btn, (int)o.edge);
}

// ── the contract with the rest of the firmware ───────────────────────────────

/* 논리 버튼 번호를 그대로 fsd_button.h 의 index 로 쓴다. 넘치면 그 버튼의
 * 사건이 조용히 사라진다 — 컴파일 타임에 막는다. */
static void test_fits_the_button_table(void) {
    printf("버튼 표에 들어간다\n");
    CHECK(FSD_J6_COUNT <= FSD_BTN_MAX, "논리 버튼 %d 개 > FSD_BTN_MAX %d",
          (int)FSD_J6_COUNT, (int)FSD_BTN_MAX);
    CHECK(FSD_J6_NONE == 0, "NONE 이 0 이 아니면 0 초기화가 안전하지 않다");
}

/* 이름은 로그로 나간다. 값이 늘었는데 이름이 없으면 숫자가 새어 나온다.
 *
 * 🔴 그리고 **공백이 없어야 한다.** 로그가 이름과 사건을 나란히 찍으므로,
 * 이름에 공백이 있으면 읽는 사람이 둘을 못 가른다 — 벤치에서 실제로
 * `[BTN] 0 3 long short (0ms)` 가 나왔다. 세 단어 중 어디까지가 버튼인지
 * 알 방법이 없다. 설명을 붙이고 싶어지는 자리라 테스트로 막는다. */
static void test_every_button_has_a_one_word_name(void) {
    printf("이름표\n");
    for (int b = 0; b < FSD_J6_COUNT; b++) {
        const char* n = fsd_j6_name((FsdJ6Btn)b);
        CHECK(n != NULL && n[0] != '\0', "%d 번에 이름이 없다", b);
        if (n) CHECK(strchr(n, ' ') == NULL, "'%s' 에 공백이 있다 — 로그가 안 갈린다", n);
    }
    CHECK(fsd_j6_name((FsdJ6Btn)FSD_J6_COUNT) != NULL, "범위 밖이 NULL 을 준다");
}

/* 이름이 서로 달라야 한다. 같으면 로그에서 두 버튼이 한 버튼으로 보인다. */
static void test_names_are_distinct(void) {
    printf("이름이 겹치지 않는다\n");
    for (int a = 0; a < FSD_J6_COUNT; a++)
        for (int b = a + 1; b < FSD_J6_COUNT; b++)
            CHECK(strcmp(fsd_j6_name((FsdJ6Btn)a), fsd_j6_name((FsdJ6Btn)b)) != 0,
                  "%d 와 %d 의 이름이 같다: '%s'", a, b, fsd_j6_name((FsdJ6Btn)a));
}


/* 🔴 창이 하나면 쓸기 버튼의 2회는 여섯 번에 한 번 실패하고, 실패는 1회 동작이
 * 두 번 실행되는 것으로 나타난다. 실측 간격 0.21~0.33초. */
static void test_swipes_get_a_wider_double_window(void) {
    printf("쓸기는 창이 더 넓다\n");
    const FsdJ6Btn swipes[] = {FSD_J6_B1, FSD_J6_B2, FSD_J6_B3, FSD_J6_B4};
    for (size_t i = 0; i < sizeof(swipes) / sizeof(swipes[0]); i++) {
        const uint16_t w = fsd_j6_double_window_ms(swipes[i]);
        CHECK(w >= 400, "%s 의 창이 %u — 실측 0.33초를 못 덮는다",
              fsd_j6_name(swipes[i]), (unsigned)w);
    }
    /* 탭은 넓힐 이유가 없다 — 넓히면 지연만 는다. */
    CHECK(fsd_j6_double_window_ms(FSD_J6_B6) < fsd_j6_double_window_ms(FSD_J6_B1),
          "탭이 쓸기만큼 기다린다");
    CHECK(fsd_j6_double_window_ms(FSD_J6_B7) < fsd_j6_double_window_ms(FSD_J6_B3),
          "탭이 쓸기만큼 기다린다");

    /* 어떤 버튼도 길게 임계값을 잡아먹으면 안 된다 — 그러면 그 버튼의 길게
     * 누르기가 영영 안 나온다. */
    for (int b = 0; b < FSD_J6_COUNT; b++)
        CHECK(fsd_j6_double_window_ms((FsdJ6Btn)b) < FSD_BTN_LONG_MS,
              "%s 의 창이 길게를 삼킨다", fsd_j6_name((FsdJ6Btn)b));
}


/* ── 뗌을 잃었을 때 ──────────────────────────────────────────────────────────
 *
 * 🔴 이것이 가정이 아니다. `ble_central.cpp` 는 24칸 링이 차면 리포트를 버리고
 * 그 수를 센다 — 알림이 유실된다는 것을 코드가 이미 알고 있다. 끊김은 슬롯
 * 정리가 다루지만, **살아 있는 링크에서 한 줄이 사라지는 것**은 아무도 안 봤다.
 *
 * 6번은 유일한 레벨 버튼이라 뗌이 올 때까지 눌린 채로 남는다. 그 뗌을 잃으면
 * 상태가 걸리고, 걸린 상태에서 다음 버튼을 누르면 **그 입력이 삼켜지고 그
 * 뗌이 6번의 뗌으로 발화한다** — 사장님이 누르지 않은 버튼이 동작한다.
 */

static void test_lost_level_release_does_not_swallow_the_next_button(void) {
    FsdJ6 s;
    fsd_j6_init(&s);

    const FsdJ6Out down = fsd_j6_feed(&s, B6_PRESS, ROW, 1000);
    CHECK(down.btn == FSD_J6_B6 && down.edge == FSD_J6_EDGE_DOWN, "6번 누름이 안 났다");

    /* 뗌 리포트가 유실됐다 — B6_RELEASE 를 주지 않는다. */

    /* 그 다음 7번 탭. 눌림이 삼켜지면 안 되고, 뗌이 7번으로 나와야 한다. */
    const FsdJ6Out p = fsd_j6_feed(&s, B7_PRESS, ROW, 30000);
    const FsdJ6Out r = fsd_j6_feed(&s, B7_RELEASE, ROW, 30100);

    /* 누름 순간에 6번이 놓인 것으로 정리된다 — 다른 자리를 눌렀다는 것은 앞의
     * 접촉이 끝났다는 뜻이기 때문이다(단일 접촉 기기다). */
    CHECK(p.btn == FSD_J6_B6 && p.edge == FSD_J6_EDGE_UP,
          "6번이 정리되지 않았다: btn=%d edge=%d", (int)p.btn, (int)p.edge);
    CHECK(r.btn == FSD_J6_B7 && r.edge == FSD_J6_EDGE_PULSE,
          "7번이 7번으로 안 났다: btn=%d edge=%d", (int)r.btn, (int)r.edge);
}

/* 같은 자리가 다시 눌린 것은 **같은 접촉**이다. 뗌을 잃었든 기기가 되풀이했든
 * 물리적 사실은 "6번이 눌려 있다" 로 같으므로, 유지를 끊지 않는다. */
static void test_the_same_point_again_is_the_same_hold(void) {
    FsdJ6 s;
    fsd_j6_init(&s);

    fsd_j6_feed(&s, B6_PRESS, ROW, 1000);
    const FsdJ6Out again = fsd_j6_feed(&s, B6_PRESS, ROW, 4000);
    CHECK(again.edge == FSD_J6_EDGE_NONE,
          "같은 자리 재누름이 사건을 냈다: btn=%d edge=%d", (int)again.btn, (int)again.edge);

    const FsdJ6Out up = fsd_j6_feed(&s, B6_RELEASE, ROW, 6000);
    CHECK(up.btn == FSD_J6_B6 && up.edge == FSD_J6_EDGE_UP, "유지가 끊겼다");
}

/* 🔴 뗌을 잃은 뒤의 **쓸기**도 자기 버튼으로 나야 한다. 탭만 고치면 쓸기가
 * 여전히 6번의 뗌으로 둔갑한다 — 실제로 측정된 모양이 그것이었다. */
static void test_lost_level_release_then_a_swipe(void) {
    FsdJ6 s;
    fsd_j6_init(&s);
    fsd_j6_feed(&s, B6_PRESS, ROW, 1000);

    int fired = 0;
    const FsdJ6Out o = feed_burst(&s, B3_RIGHT, sizeof(B3_RIGHT) / ROW, 30000, &fired);
    CHECK(o.btn == FSD_J6_B3 && o.edge == FSD_J6_EDGE_PULSE,
          "쓸기가 3번으로 안 났다: btn=%d edge=%d", (int)o.btn, (int)o.edge);
}

int main(void) {
    printf("=== J6 리모컨 디코더 ===\n\n");
    test_swipes();
    test_opposites_differ();
    test_b6_is_level();
    test_b7_is_a_pulse();
    test_taps_differ_by_coordinate();
    test_unknown_tap_is_nothing();
    test_consumer_codes();
    test_ambiguous_codes_are_refused();
    test_trailing_release_is_not_a_second_press();
    test_wrong_lengths_are_ignored();
    test_bad_state_byte_is_ignored();
    test_out_of_range_coordinates_are_ignored();
    test_stale_gesture_makes_a_tap_look_like_a_swipe();
    test_stale_gesture_swallows_the_level_button();
    test_gesture_in_progress_is_not_dropped();
    test_level_hold_is_not_stale();
    test_lost_level_release_does_not_swallow_the_next_button();
    test_the_same_point_again_is_the_same_hold();
    test_lost_level_release_then_a_swipe();
    test_fits_the_button_table();
    test_every_button_has_a_one_word_name();
    test_names_are_distinct();
    test_swipes_get_a_wider_double_window();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
