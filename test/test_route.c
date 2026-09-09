/* test_route.c — 앱이 준 경로로 "이 카메라가 우리 길 위인가" 를 가른다.
 *
 * 🔴 이 파일이 지키는 것은 **거부의 방향**이다. 경로 필터가 하는 일은 카메라를
 * 버리는 것이고, 잘못 버리면 우리 길 위의 단속카메라를 못 보게 된다. 반대로
 * 잘못 남기면 남의 길 카메라에 한 번 느려질 뿐이다. 두 실패의 무게가 다르므로
 * **확신할 때만 버린다** — 그것이 아래 단언들의 전부다.
 */
#include <stdio.h>
#include <string.h>

#include "fsd_route.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if(cond) {                                                             \
            g_pass++;                                                          \
        } else {                                                               \
            g_fail++;                                                          \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                      \
            printf(__VA_ARGS__);                                               \
            printf("\n");                                                      \
        }                                                                      \
    } while(0)

/* 서울 시청 앞에서 동쪽으로 곧게 뻗은 길. 1e-7 도 단위이고, 경도 0.001 도는
 * 이 위도에서 약 88 m 다. */
#define LAT0 375665000
#define LON0 1269780000

static void put(uint8_t* b, int i, int32_t lat, int32_t lon) {
    const uint32_t a = (uint32_t)lat, o = (uint32_t)lon;
    b[i * 8 + 0] = (uint8_t)a;         b[i * 8 + 1] = (uint8_t)(a >> 8);
    b[i * 8 + 2] = (uint8_t)(a >> 16); b[i * 8 + 3] = (uint8_t)(a >> 24);
    b[i * 8 + 4] = (uint8_t)o;         b[i * 8 + 5] = (uint8_t)(o >> 8);
    b[i * 8 + 6] = (uint8_t)(o >> 16); b[i * 8 + 7] = (uint8_t)(o >> 24);
}

/** 동쪽으로 n 점, 점 사이 약 88 m. */
static void straight_east(FsdRoute* r, int n, uint32_t now) {
    uint8_t b[64 * 8];
    for(int i = 0; i < n; i++) put(b, i, LAT0, LON0 + i * 10000);
    fsd_route_feed(r, 0u, (uint16_t)n, b, (size_t)n * 8u, now);
}

static void test_a_route_arrives_in_pieces(void) {
    printf("\n-- 경로가 조각으로 온다 --\n");

    FsdRoute r;
    fsd_route_init(&r);
    CHECK(!fsd_route_complete(&r), "처음에는 경로가 없다");

    uint8_t b[8 * 8];
    for(int i = 0; i < 8; i++) put(b, i, LAT0, LON0 + i * 10000);

    CHECK(fsd_route_feed(&r, 0u, 12u, b, 8u * 8u, 1000u), "첫 조각을 받는다");
    CHECK(!fsd_route_complete(&r), "여덟 점으로는 열둘이 안 된다 — 아직 미완");

    /* 🔴 미완인 경로로는 아무것도 거부하지 않는다. 절반만 온 경로로 거르면
     * 아직 안 온 절반 위의 카메라가 전부 남의 길이 된다. */
    CHECK(!fsd_route_rejects(&r, LAT0, LON0, LAT0 + 100000, LON0),
          "미완이면 거부하지 않는다");

    uint8_t b2[4 * 8];
    for(int i = 0; i < 4; i++) put(b2, i, LAT0, LON0 + (8 + i) * 10000);
    CHECK(fsd_route_feed(&r, 8u, 12u, b2, 4u * 8u, 1100u), "둘째 조각");
    CHECK(fsd_route_complete(&r), "열둘이 다 왔다");
    CHECK(fsd_route_count(&r) == 12, "점 수 %u", (unsigned)fsd_route_count(&r));
}

static void test_out_of_order_is_thrown_away(void) {
    printf("\n-- 순서가 어긋난 조각은 버린다 --\n");

    FsdRoute r;
    fsd_route_init(&r);
    uint8_t b[4 * 8];
    for(int i = 0; i < 4; i++) put(b, i, LAT0, LON0 + i * 10000);

    CHECK(fsd_route_feed(&r, 0u, 8u, b, 4u * 8u, 1000u), "0 번 조각");
    /* 🔴 BLE 는 순서를 지키지만 우리가 그것에 기대면 안 된다. 어긋난 조각을
     * 그냥 이어 붙이면 경로가 **조용히 엉뚱한 모양**이 되고, 그 경로로 거른
     * 결과는 전부 그럴듯해 보인다. */
    CHECK(!fsd_route_feed(&r, 6u, 8u, b, 2u * 8u, 1010u), "6 번은 지금 자리가 아니다");
    CHECK(fsd_route_count(&r) == 4, "버렸으니 그대로 4");
    CHECK(fsd_route_feed(&r, 4u, 8u, b, 4u * 8u, 1020u), "4 번은 맞다");
    CHECK(fsd_route_complete(&r), "이제 완성");
}

static void test_a_new_route_replaces_the_old_one(void) {
    printf("\n-- 새 경로가 옛 경로를 덮는다 --\n");

    FsdRoute r;
    fsd_route_init(&r);
    straight_east(&r, 8, 1000u);
    CHECK(fsd_route_complete(&r), "첫 경로");

    /* seq 0 은 언제나 새 출발이다. 이어 붙이면 목적지를 바꿨을 때 옛 길이
     * 그대로 남아 그 위의 카메라가 계속 통과한다. */
    uint8_t b[2 * 8];
    put(b, 0, LAT0, LON0);
    put(b, 1, LAT0 + 10000, LON0);
    CHECK(fsd_route_feed(&r, 0u, 2u, b, 2u * 8u, 2000u), "새 경로 시작");
    CHECK(fsd_route_count(&r) == 2, "옛 여덟 점이 남지 않았다: %u",
          (unsigned)fsd_route_count(&r));
}

static void test_it_only_rejects_when_it_is_sure(void) {
    printf("\n-- 확신할 때만 버린다 --\n");

    FsdRoute r;
    fsd_route_init(&r);

    /* 경로가 아예 없으면 지금과 똑같이 동작해야 한다 — 필터는 **덤**이지
     * 게이트가 아니다. */
    CHECK(!fsd_route_rejects(&r, LAT0, LON0, LAT0 + 1000000, LON0 + 1000000),
          "경로가 없으면 아무것도 안 버린다");

    straight_east(&r, 12, 1000u);

    /* 길 위의 카메라 — 남긴다. */
    CHECK(!fsd_route_rejects(&r, LAT0, LON0 + 20000, LAT0, LON0 + 60000),
          "길 위의 카메라는 남는다");

    /* 🔴 길에서 한참 벗어난 카메라 — 이것이 버리려던 것이다. 위도 0.005 도는
     * 약 555 m 다. */
    CHECK(fsd_route_rejects(&r, LAT0, LON0 + 20000, LAT0 + 50000, LON0 + 60000),
          "555 m 옆의 카메라는 다른 길이다");

    /* 🔴 우리가 경로를 벗어났으면 필터를 끈다. 다른 길로 접어들었는데 옛
     * 경로로 계속 거르면, 지금 달리는 길의 카메라가 **전부** 남의 길이 된다 —
     * 필터가 정확히 반대로 일한다. */
    CHECK(!fsd_route_rejects(&r, LAT0 + 300000, LON0, LAT0 + 300000, LON0 + 5000),
          "우리가 경로를 벗어나면 아무것도 안 버린다");
}

static void test_a_corridor_wide_enough_to_be_wrong_in(void) {
    printf("\n-- 회랑은 틀려도 되게 넓다 --\n");

    FsdRoute r;
    fsd_route_init(&r);
    straight_east(&r, 12, 1000u);

    /* 폰 GPS 오차는 흔히 5~15 m 이고 나쁘면 50 m 다. 왕복 8차선은 40 m 쯤
     * 된다. 회랑이 그보다 좁으면 **맞은편 차선의 카메라를 버린다.** */
    CHECK(FSD_ROUTE_CORRIDOR_M >= 60.0f,
          "회랑이 %.0f m 다 — 차선 폭과 GPS 오차를 담아야 한다",
          (double)FSD_ROUTE_CORRIDOR_M);

    CHECK(!fsd_route_rejects(&r, LAT0, LON0 + 20000, LAT0 + 4000, LON0 + 60000),
          "44 m 옆은 아직 같은 길이다");
}

static void test_it_survives_rubbish(void) {
    printf("\n-- 쓰레기가 와도 죽지 않는다 --\n");

    FsdRoute r;
    fsd_route_init(&r);
    uint8_t b[8];
    put(b, 0, LAT0, LON0);

    CHECK(!fsd_route_feed(NULL, 0u, 1u, b, 8u, 1000u), "NULL");
    CHECK(!fsd_route_feed(&r, 0u, 1u, NULL, 8u, 1000u), "NULL 바이트");
    CHECK(!fsd_route_feed(&r, 0u, 1u, b, 7u, 1000u), "8 로 안 나누어떨어진다");
    CHECK(!fsd_route_feed(&r, 0u, 0u, b, 8u, 1000u), "총 0 점은 경로가 아니다");
    CHECK(!fsd_route_feed(&r, 0u, (uint16_t)(FSD_ROUTE_MAX_POINTS + 1u), b, 8u, 1000u),
          "담을 수 있는 것보다 많다");
    CHECK(!fsd_route_complete(&r), "하나도 안 받았다");
    CHECK(!fsd_route_rejects(NULL, LAT0, LON0, LAT0, LON0), "NULL 은 안 버린다");
}

int main(void) {
    printf("test_route\n");
    test_a_route_arrives_in_pieces();
    test_out_of_order_is_thrown_away();
    test_a_new_route_replaces_the_old_one();
    test_it_only_rejects_when_it_is_sure();
    test_a_corridor_wide_enough_to_be_wrong_in();
    test_it_survives_rubbish();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
