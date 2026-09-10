/*
 * test_ota_gate.c — 펌웨어를 받기 시작해도 되는가, 그리고 조각이 제대로 오는가.
 *
 * 🔴 왜 이 판정이 `fsd_logic` 에 있나
 * ───────────────────────────────────────────────────────────────────────────
 * 설치 자체는 `esp_ota_*` 를 부르는 일이라 호스트 시험이 못 닿는다. 그런데
 * **거절할지 말지**는 순수한 판단이고, 그것이 ESP32 사본 안에만 있으면 이
 * 저장소의 열 번째 패턴이 된다 — 시험이 도는 코드보다 실제로 도는 코드가 넓은
 * 자리. 그래서 판단만 여기로 꺼냈다.
 *
 * 🔴 순서가 내용이다. 조건이 여럿 참일 때 **무엇을 먼저 말하느냐**가 차 옆에
 * 앉은 사람이 다음에 무엇을 할지를 정한다. 그래서 순서에 시험이 붙어 있다.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_ota_gate.h"

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

/* 여덟 보드 중 우리 것. app0/app1 이 각 6.25 MB 다 (실측 2026-09-10:
 * default_16MB.csv). 지금 이미지는 689 KB. */
#define SLOT 6553600u
#define IMG 688992u

/* 통과하는 판. 하나씩 망가뜨려 무엇이 거절되는지 본다. */
static FsdOtaBeginIn good(void) {
    FsdOtaBeginIn in;
    memset(&in, 0, sizeof(in));
    in.owner_window_open = true;
    in.slot_bytes = SLOT;
    in.declared_bytes = IMG;
    return in;
}

static void test_the_ordinary_case_passes(void) {
    FsdOtaBeginIn in = good();
    CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_OK, "평범한 시작은 통과한다");
}

static void test_each_gate_refuses_on_its_own(void) {
    {
        FsdOtaBeginIn in = good();
        in.transfer_running = true;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_BUSY, "이미 받는 중이면 거절");
    }
    {
        FsdOtaBeginIn in = good();
        in.saving_capture = true;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_SAVING, "캡처를 쓰는 중이면 거절");
    }
    {
        FsdOtaBeginIn in = good();
        in.motion_seen = true;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_MOVING, "움직이면 거절");
    }
    {
        FsdOtaBeginIn in = good();
        in.owner_window_open = false;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_NOT_ARMED, "버튼 창이 닫혀 있으면 거절");
    }
    {
        FsdOtaBeginIn in = good();
        in.slot_bytes = 0;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_NO_SLOT, "받을 칸이 없으면 거절");
    }
    {
        FsdOtaBeginIn in = good();
        in.declared_bytes = SLOT + 1u;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_TOO_BIG, "칸보다 크면 거절");
    }
    {
        FsdOtaBeginIn in = good();
        in.declared_bytes = FSD_OTA_MIN_IMAGE - 1u;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_TOO_SMALL, "이미지일 수 없는 크기는 거절");
    }
    {
        FsdOtaBeginIn in = good();
        in.declared_bytes = 0;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_TOO_SMALL, "0 바이트는 거절");
    }
}

/*
 * 🔴 이 시험이 이 파일의 본체다.
 *
 * 조건이 둘 이상 참일 때 어느 것을 말하느냐가 곧 **차 옆에 앉은 사람에게 주는
 * 지시**다. "버튼을 누르세요" 는 전송이 이미 돌고 있을 때는 틀린 지시이고,
 * 누르는 것으로 아무 일도 안 일어난다.
 *
 * 규칙은 1층과 같다 — **바깥부터**. 모듈의 형편(받는 중 · 쓰는 중) → 차의
 * 형편(움직임) → 권한(창) → 이 요청 자체(칸 · 크기).
 */
static void test_it_answers_with_the_outermost_reason(void) {
    {
        FsdOtaBeginIn in = good();
        in.transfer_running = true;
        in.saving_capture = true;
        in.motion_seen = true;
        in.owner_window_open = false;
        in.slot_bytes = 0;
        in.declared_bytes = 0;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_BUSY,
              "전부 나쁘면 '이미 받는 중' 이 먼저다");
    }
    {
        FsdOtaBeginIn in = good();
        in.saving_capture = true;
        in.motion_seen = true;
        in.owner_window_open = false;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_SAVING, "쓰는 중이 움직임보다 먼저");
    }
    {
        FsdOtaBeginIn in = good();
        in.motion_seen = true;
        in.owner_window_open = false;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_MOVING, "움직임이 창보다 먼저");
    }
    {
        FsdOtaBeginIn in = good();
        in.owner_window_open = false;
        in.slot_bytes = 0;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_NOT_ARMED, "창이 칸보다 먼저");
    }
    {
        FsdOtaBeginIn in = good();
        in.slot_bytes = 0;
        in.declared_bytes = 0;
        CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_NO_SLOT, "칸이 크기보다 먼저");
    }
}

/*
 * 🔴 "안 움직인다" 를 증명하라고 하지 않는다.
 *
 * 차가 자리를 잡으면 구동 인버터가 잠들고 `0x257` 이 **버스에서 사라진다** —
 * 그때가 정확히 사람이 폰으로 굽고 싶은 때다. 그래서 게이트는 *정차를 증명하라*
 * 가 아니라 *움직이는 것을 보았다면 거절한다* 이다.
 *
 * 이것을 반대로 쓰면 미러가 물렸던 그 함정을 그대로 다시 판다 — 기어 신호가
 * 없다고 P 를 증명 못 해서 거절하던 자리.
 */
static void test_a_sleeping_car_is_not_a_moving_car(void) {
    FsdOtaBeginIn in = good();
    in.motion_seen = false; /* 속도 프레임이 아예 안 온다 */
    CHECK(fsd_ota_begin_check(&in) == FSD_OTA_BEGIN_OK,
          "속도 신호가 없는 것은 '움직인다' 가 아니다");
}

static void test_null_is_refused_not_crashed(void) {
    CHECK(fsd_ota_begin_check(NULL) == FSD_OTA_BEGIN_BUSY, "NULL 은 거절이지 통과가 아니다");
}

/* ── 조각 ────────────────────────────────────────────────────────────────── */

static void test_chunks_must_arrive_in_order(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 1000u);

    CHECK(fsd_ota_xfer_take(&x, 1u, 400u) == FSD_OTA_CHUNK_OK, "1번이 먼저");
    CHECK(fsd_ota_xfer_take(&x, 3u, 400u) == FSD_OTA_CHUNK_SEQUENCE, "2번을 건너뛰면 거절");
    /* 거절된 조각은 세지 않는다 — 안 그러면 구멍이 난 채로 완료가 된다. */
    CHECK(x.written == 400u, "거절된 조각은 안 센다");
}

static void test_a_repeat_is_a_sequence_error_too(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 1000u);
    CHECK(fsd_ota_xfer_take(&x, 1u, 400u) == FSD_OTA_CHUNK_OK, "1번");
    CHECK(fsd_ota_xfer_take(&x, 1u, 400u) == FSD_OTA_CHUNK_SEQUENCE, "1번을 또 보내면 거절");
}

static void test_the_header_number_is_not_a_chunk(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 1000u);
    CHECK(fsd_ota_xfer_take(&x, 0u, 400u) == FSD_OTA_CHUNK_SEQUENCE,
          "0 은 머리말 번호다 — 데이터로 오면 안 된다");
}

static void test_it_refuses_more_than_was_declared(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 1000u);
    CHECK(fsd_ota_xfer_take(&x, 1u, 600u) == FSD_OTA_CHUNK_OK, "600");
    CHECK(fsd_ota_xfer_take(&x, 2u, 600u) == FSD_OTA_CHUNK_OVERRUN, "1200 은 넘친다");
    CHECK(x.written == 600u, "넘친 조각은 안 센다");
}

static void test_exactly_the_declared_size_completes(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 1000u);
    CHECK(!fsd_ota_xfer_complete(&x), "시작하자마자 끝난 것이 아니다");
    CHECK(fsd_ota_xfer_take(&x, 1u, 600u) == FSD_OTA_CHUNK_OK, "600");
    CHECK(!fsd_ota_xfer_complete(&x), "아직 아니다");
    CHECK(fsd_ota_xfer_take(&x, 2u, 400u) == FSD_OTA_CHUNK_OK, "400 더");
    CHECK(fsd_ota_xfer_complete(&x), "선언한 만큼 오면 끝이다");
}

/* 🔴 선언이 0 이면 '이미 다 왔다' 가 되어 버린다. 시작 게이트가 그것을 먼저
 * 거절하지만, 이 층이 혼자서도 거짓말을 안 하게 못 박는다. */
static void test_a_zero_length_transfer_is_never_complete(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 0u);
    CHECK(!fsd_ota_xfer_complete(&x), "0 바이트짜리는 완료가 아니다");
}

static void test_an_empty_chunk_says_so(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 1000u);
    CHECK(fsd_ota_xfer_take(&x, 1u, 0u) == FSD_OTA_CHUNK_EMPTY, "빈 조각은 빈 조각이라고 말한다");
    CHECK(fsd_ota_xfer_take(&x, 1u, 400u) == FSD_OTA_CHUNK_OK,
          "빈 조각은 순번을 안 먹는다 — 1번이 아직 기다린다");
}

/*
 * 🔴 순번이 어긋난 **빈** 조각도 '빈 조각' 이다.
 *
 * 이 구분이 왜 중요한가: 부르는 쪽(`esp32/.firmware/ota_store.cpp`)이 SEQUENCE 에는
 * 전송을 **접고** EMPTY 에는 안 접는다. 둘을 헷갈리면 알맹이 없는 프레임 하나가
 * 13초짜리 전송을 죽이고, 폰에는 "조각 순서가 어긋났다" 로만 보인다.
 *
 * 🔴 이 시험은 **돌연변이가 요구해서 생겼다** (2026-09-10). 빈 조각 검사를 순번
 * 검사 뒤로 옮기는 돌연변이가 살아남았는데, 그것은 시험이 순번이 맞는 빈 조각만
 * 물어봤기 때문이다 — 그 경우에는 두 순서가 같은 답을 낸다. 갈리는 자리가
 * 여기다.
 */
static void test_an_empty_chunk_is_empty_even_out_of_order(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 1000u);
    CHECK(fsd_ota_xfer_take(&x, 1u, 400u) == FSD_OTA_CHUNK_OK, "1번");
    CHECK(fsd_ota_xfer_take(&x, 7u, 0u) == FSD_OTA_CHUNK_EMPTY,
          "순번이 틀린 빈 조각도 '빈 조각' 이다 — 전송을 접을 이유가 아니다");
    CHECK(fsd_ota_xfer_take(&x, 2u, 400u) == FSD_OTA_CHUNK_OK, "2번이 그대로 이어진다");
}

/*
 * 🔴 순번이 16비트라 큰 이미지에서 한 바퀴 돈다.
 *
 * 지금 이미지(689 KB)와 지금 MTU(517 → 조각 500 B)로는 1,378 조각이라 **오늘은
 * 도달 불가**다. 그런데 MTU 가 23 으로 떨어지면 조각이 20 B 가 되고, 그러면
 * 34,450 조각이 된다 — 아직 안 돈다. 6 MB 짜리를 20 B 로 보내야 도는데, 그때
 * 처리가 없으면 **멀쩡한 전송이 끝에서 거절된다.**
 *
 * 0 은 머리말이므로 65535 다음은 1 이다.
 */
static void test_the_sequence_wraps_past_the_header_number(void) {
    FsdOtaXfer x;
    fsd_ota_xfer_init(&x, 1000000u);
    x.next_seq = 65535u; /* 여기까지 왔다고 치고 */
    CHECK(fsd_ota_xfer_take(&x, 65535u, 10u) == FSD_OTA_CHUNK_OK, "65535 번");
    CHECK(fsd_ota_xfer_take(&x, 1u, 10u) == FSD_OTA_CHUNK_OK, "그다음은 0 이 아니라 1 이다");
    CHECK(fsd_ota_xfer_take(&x, 2u, 10u) == FSD_OTA_CHUNK_OK, "그리고 2");
}

static void test_names(void) {
    const FsdOtaBeginVerdict all[] = {
        FSD_OTA_BEGIN_OK,       FSD_OTA_BEGIN_BUSY,      FSD_OTA_BEGIN_SAVING,
        FSD_OTA_BEGIN_MOVING,   FSD_OTA_BEGIN_NOT_ARMED, FSD_OTA_BEGIN_NO_SLOT,
        FSD_OTA_BEGIN_TOO_SMALL, FSD_OTA_BEGIN_TOO_BIG,
    };
    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        const char* n = fsd_ota_begin_verdict_str(all[i]);
        CHECK(n && n[0] != '\0', "시작 판정 %u 에 이름이 있다", (unsigned)all[i]);
    }
    CHECK(fsd_ota_begin_verdict_str((FsdOtaBeginVerdict)200)[0] != '\0',
          "모르는 시작 판정에도 이름이 있다");

    const FsdOtaChunkVerdict cs[] = {
        FSD_OTA_CHUNK_OK, FSD_OTA_CHUNK_SEQUENCE, FSD_OTA_CHUNK_OVERRUN, FSD_OTA_CHUNK_EMPTY,
    };
    for (size_t i = 0; i < sizeof(cs) / sizeof(cs[0]); i++) {
        const char* n = fsd_ota_chunk_verdict_str(cs[i]);
        CHECK(n && n[0] != '\0', "조각 판정 %u 에 이름이 있다", (unsigned)cs[i]);
    }
    CHECK(fsd_ota_chunk_verdict_str((FsdOtaChunkVerdict)200)[0] != '\0',
          "모르는 조각 판정에도 이름이 있다");
}


/* ── 시한 ──────────────────────────────────────────────────────────────── */

static void test_stall_needs_the_whole_wait(void) {
    /* 아직 안 됐다 */
    CHECK(!fsd_ota_stalled(1000u, 1000u, 60000u), "막 왔으면 안 접는다");
    CHECK(!fsd_ota_stalled(61000u, 1000u, 60000u), "시한 직전에는 안 접는다");
    /* 딱 넘었다 */
    CHECK(fsd_ota_stalled(61001u, 1000u, 60000u), "시한을 넘으면 접는다");
}

static void test_stall_survives_a_last_seen_from_the_future(void) {
    /*
     * 🔴 **이 시험이 실차에서 물린 것을 그대로 재현한다.**
     *
     * loop() 가 시각을 뜨고 여러 일을 하는 동안 BLE 태스크가 조각을 받는다.
     * 그러면 `last_seen_ms` 가 `now_ms` 보다 **몇 ms 앞선다.** 앞선 것은
     * "방금 왔다" 는 뜻이지 "49 일 전에 왔다" 는 뜻이 아니다.
     */
    CHECK(!fsd_ota_stalled(1000u, 1001u, 60000u), "1 ms 앞선 것은 방금 온 것이다");
    CHECK(!fsd_ota_stalled(1000u, 1200u, 60000u), "200 ms 앞서도 마찬가지");
    /* loop() 한 바퀴가 아무리 길어도 앞서는 폭은 그 한 바퀴다. */
    CHECK(!fsd_ota_stalled(1000u, 1000u + 5000u, 60000u),
          "loop 한 바퀴가 길어도 앞선 것은 앞선 것이다");
}

static void test_stall_still_fires_across_the_millis_wrap(void) {
    /* millis() 는 49 일에 한 번 0 으로 돌아온다. 그때도 판정이 서야 한다. */
    const uint32_t before_wrap = 0xFFFFF000u;
    CHECK(!fsd_ota_stalled(before_wrap + 1000u, before_wrap, 60000u),
          "49 일 넘김 자리에서도 시한 전에는 안 접는다");
    CHECK(fsd_ota_stalled(before_wrap + 61001u, before_wrap, 60000u),
          "49 일 넘김 자리에서도 시한을 넘으면 접는다");
}

int main(void) {
    test_stall_needs_the_whole_wait();
    test_stall_survives_a_last_seen_from_the_future();
    test_stall_still_fires_across_the_millis_wrap();
    printf("test_ota_gate: 받기 시작해도 되는가 · 조각이 제대로 오는가\n");
    test_the_ordinary_case_passes();
    test_each_gate_refuses_on_its_own();
    test_it_answers_with_the_outermost_reason();
    test_a_sleeping_car_is_not_a_moving_car();
    test_null_is_refused_not_crashed();
    test_chunks_must_arrive_in_order();
    test_a_repeat_is_a_sequence_error_too();
    test_the_header_number_is_not_a_chunk();
    test_it_refuses_more_than_was_declared();
    test_exactly_the_declared_size_completes();
    test_a_zero_length_transfer_is_never_complete();
    test_an_empty_chunk_says_so();
    test_an_empty_chunk_is_empty_even_out_of_order();
    test_the_sequence_wraps_past_the_header_number();
    test_names();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
