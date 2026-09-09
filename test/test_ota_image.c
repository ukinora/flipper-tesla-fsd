/*
 * test_ota_image.c — 들어오는 이미지가 이 보드의 것인가.
 *
 * 🔴 이 파일에서 가장 중요한 시험은 **조각 경계에 걸친 표식**이다. 조각 크기는
 * MTU 가 정하므로 표식이 어디에 떨어질지 아무도 못 고르고, 못 찾은 결과는
 * "우리 이미지를 거부" 다 — 즉 실패가 **안전한 쪽이 아니라 쓸모없는 쪽**으로
 * 떨어진다. 그래서 모든 경계를 하나씩 훑는다.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_ota_image.h"

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

#define OUR_BOARD "lilygo-t2can"
#define OUR_STAMP "2026-09-10 04:12:00 6a213985"

/* 아주 작은 가짜 이미지 하나. 진짜 이미지는 673 KB 라 시험에 넣을 것이 못
 * 되고, 이 층이 보는 것은 머리말 24바이트와 어딘가의 표식 72바이트뿐이다.
 *
 * 🔴 머리말의 숫자는 실측이다 — 2026-09-10 에 우리 firmware.bin 을 열어
 * magic 0xE9, chip_id 9 (ESP32-S3) 를 읽었다. 지어낸 값이 아니다. */
static uint32_t make_image(uint8_t* buf, uint32_t cap, uint32_t mark_at,
                           const char* board, const char* stamp, uint8_t magic,
                           uint16_t chip) {
    memset(buf, 0xA5, cap); /* 표식이 아닌 것으로 채운다 */
    buf[0] = magic;
    buf[1] = 5; /* segment_count */
    buf[FSD_OTA_CHIP_OFFSET] = (uint8_t)(chip & 0xFFu);
    buf[FSD_OTA_CHIP_OFFSET + 1u] = (uint8_t)(chip >> 8);

    if (board) {
        FsdOtaMarkWire w;
        memset(&w, 0, sizeof(w));
        w.magic[0] = FSD_OTA_MARK_MAGIC_0;
        w.magic[1] = FSD_OTA_MARK_MAGIC_1;
        w.magic[2] = FSD_OTA_MARK_MAGIC_2;
        w.magic[3] = FSD_OTA_MARK_MAGIC_3;
        w.magic[4] = FSD_OTA_MARK_MAGIC_4;
        w.magic[5] = FSD_OTA_MARK_MAGIC_5;
        w.magic[6] = FSD_OTA_MARK_MAGIC_6;
        w.magic[7] = (char)FSD_OTA_MARK_MAGIC_7;
        strncpy(w.board, board, sizeof(w.board));
        strncpy(w.stamp, stamp, sizeof(w.stamp));
        memcpy(buf + mark_at, &w, sizeof(w));
    }
    return cap;
}

/* 한 조각씩 먹인다. */
static void feed_in_chunks(FsdOtaScan* s, const uint8_t* img, uint32_t len, uint32_t chunk) {
    for (uint32_t i = 0; i < len; i += chunk) {
        uint32_t n = len - i;
        if (n > chunk) n = chunk;
        fsd_ota_scan_feed(s, img + i, n);
    }
}

/* ── 붙는 경우 ─────────────────────────────────────────────────────────── */

static void test_our_own_image_passes(void) {
    printf("\n-- 우리가 구운 이미지는 지난다 --\n");

    uint8_t img[600];
    const uint32_t n = make_image(img, sizeof(img), 300, OUR_BOARD, OUR_STAMP,
                                  FSD_OTA_ESP_MAGIC, FSD_OTA_CHIP_ESP32S3);

    FsdOtaScan s;
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, n, 512);

    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_OK,
          "통과해야 하는데 %s",
          fsd_ota_img_verdict_str(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3)));
    CHECK(s.found, "표식을 찾았다고 표시한다");
    CHECK(strcmp(s.mark.board, OUR_BOARD) == 0, "보드 이름: %s", s.mark.board);
    CHECK(strcmp(s.mark.stamp, OUR_STAMP) == 0, "판번호: %s", s.mark.stamp);
    CHECK(s.seen == n, "%u 바이트를 세었다, got %u", (unsigned)n, (unsigned)s.seen);
}

/* 🔴 이 파일의 심장. 표식이 조각 둘에 걸치는 모든 자리를 훑는다.
 *
 * 겹침을 안 들고 있으면 여기서 무너진다 — 그리고 무너진 결과는 "우리 이미지를
 * 거부" 이므로, 차 옆에서 보면 "왜 안 올라가지" 가 된다. */
static void test_the_mark_may_straddle_a_chunk_boundary(void) {
    printf("\n-- 표식이 조각 경계에 걸쳐도 찾는다 --\n");

    /* 조각 크기 여럿 × 표식 위치 여럿. 표식(72 B)보다 작은 조각도 넣는다 —
     * 그러면 표식이 세 조각에 걸친다. */
    const uint32_t chunks[] = {1u, 7u, 32u, 64u, 71u, 72u, 73u, 128u, 509u};
    unsigned tried = 0, ok = 0;

    for (size_t c = 0; c < sizeof(chunks) / sizeof(chunks[0]); c++) {
        for (uint32_t at = FSD_OTA_HEAD_MIN; at + FSD_OTA_MARK_LEN <= 400u; at++) {
            uint8_t img[400];
            const uint32_t n = make_image(img, sizeof(img), at, OUR_BOARD, OUR_STAMP,
                                          FSD_OTA_ESP_MAGIC, FSD_OTA_CHIP_ESP32S3);
            FsdOtaScan s;
            fsd_ota_scan_init(&s);
            feed_in_chunks(&s, img, n, chunks[c]);
            tried++;
            if (fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_OK &&
                strcmp(s.mark.stamp, OUR_STAMP) == 0) {
                ok++;
            } else if (ok == tried - 1u) {
                /* 처음 실패한 자리만 말한다. 전부 찍으면 수천 줄이 된다. */
                printf("  첫 실패: 조각 %u, 표식 위치 %u\n", (unsigned)chunks[c], (unsigned)at);
            }
        }
    }
    CHECK(tried > 2000u, "충분히 많은 조합을 봤다 (%u)", tried);
    CHECK(ok == tried, "%u/%u 조합에서 찾았다", ok, tried);
}

/* ── 막는 경우 ─────────────────────────────────────────────────────────── */

static void test_what_it_refuses(void) {
    printf("\n-- 거르는 넷 --\n");

    uint8_t img[600];
    FsdOtaScan s;

    /* 1. 이미지가 아니다. 텍스트 파일을 골랐거나 받다가 깨졌거나. */
    make_image(img, sizeof(img), 300, OUR_BOARD, OUR_STAMP, 0x7Fu, FSD_OTA_CHIP_ESP32S3);
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, sizeof(img), 128);
    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_NOT_AN_IMAGE,
          "0xE9 가 아니면 이미지가 아니다");

    /* 2. 다른 칩용. 이 저장소의 다른 일곱 보드는 원래 ESP32 다 — 그것을 우리
     *    보드에 구우면 안 켜진다. 자가진단이 되돌리지만 거기까지 갈 이유가
     *    없다. */
    make_image(img, sizeof(img), 300, OUR_BOARD, OUR_STAMP, FSD_OTA_ESP_MAGIC,
               FSD_OTA_CHIP_ESP32);
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, sizeof(img), 128);
    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_WRONG_CHIP,
          "다른 칩용은 거부한다");

    /* 3. 표식이 없다 — 우리가 구운 것이 아니다.
     *
     * 🔴 이것이 실제로 거르는 것의 대부분이다. 남의 Arduino-ESP32 이미지는
     * 전부 여기서 막힌다. esp_app_desc_t 로는 못 하는 일인데, 그 칸이 남의
     * 이미지에서도 우리와 **글자 그대로 같기** 때문이다
     * ('arduino-lib-builder'). */
    make_image(img, sizeof(img), 300, NULL, NULL, FSD_OTA_ESP_MAGIC, FSD_OTA_CHIP_ESP32S3);
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, sizeof(img), 128);
    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_NO_MARK,
          "우리 표식이 없으면 거부한다");

    /* 4. 우리 것인데 다른 보드용. 칩이 같아도(둘 다 S3) 핀이 다르다. */
    make_image(img, sizeof(img), 300, "waveshare-s3-can", OUR_STAMP, FSD_OTA_ESP_MAGIC,
               FSD_OTA_CHIP_ESP32S3);
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, sizeof(img), 128);
    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_WRONG_BOARD,
          "다른 보드용은 거부한다");
    CHECK(strcmp(s.mark.board, "waveshare-s3-can") == 0,
          "그리고 어느 보드용인지 말해 준다: %s", s.mark.board);
}

/* 🔴 판정 순서가 값이다. 이미지가 아닌 것을 "표식이 없다" 로 답하면, 파일을
 * 잘못 고른 사람이 빌드를 의심하게 된다. 가장 바깥부터 답한다. */
static void test_it_answers_with_the_outermost_reason(void) {
    printf("\n-- 가장 바깥 이유부터 말한다 --\n");

    uint8_t img[600];
    FsdOtaScan s;

    /* 셋이 한꺼번에 틀렸다: 이미지가 아니고, 칩도 다르고, 표식도 없다. */
    make_image(img, sizeof(img), 300, NULL, NULL, 0x00u, FSD_OTA_CHIP_ESP32);
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, sizeof(img), 128);
    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_NOT_AN_IMAGE,
          "이미지가 아닌 것이 먼저다");

    /* 둘이 틀렸다: 칩도 다르고 표식도 없다. */
    make_image(img, sizeof(img), 300, NULL, NULL, FSD_OTA_ESP_MAGIC, FSD_OTA_CHIP_ESP32);
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, sizeof(img), 128);
    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_WRONG_CHIP,
          "칩이 표식보다 먼저다");
}

static void test_a_partial_head_is_not_a_verdict(void) {
    printf("\n-- 머리말도 다 안 왔으면 판정하지 않는다 --\n");

    uint8_t img[600];
    make_image(img, sizeof(img), 300, OUR_BOARD, OUR_STAMP, FSD_OTA_ESP_MAGIC,
               FSD_OTA_CHIP_ESP32S3);

    for (uint32_t got = 0; got < FSD_OTA_HEAD_MIN; got++) {
        FsdOtaScan s;
        fsd_ota_scan_init(&s);
        fsd_ota_scan_feed(&s, img, got);
        CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_TOO_SHORT,
              "%u 바이트로는 판정하지 않는다", (unsigned)got);
    }

    /* 🔴 머리말이 딱 찼으면 칩까지는 답할 수 있다. 표식은 아직 못 봤으므로
     * NO_MARK 인데, 그것은 "없다" 가 아니라 "아직" 이다 — 헤더가 부르는 쪽에
     * 그 계약을 적어 두었고, 이 줄이 그 상태가 실제로 그렇게 보인다는 증거다. */
    FsdOtaScan s;
    fsd_ota_scan_init(&s);
    fsd_ota_scan_feed(&s, img, FSD_OTA_HEAD_MIN);
    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_NO_MARK,
          "머리말만으로는 표식을 못 봤다고 답한다");
}

static void test_strings_always_terminate(void) {
    printf("\n-- 문자열은 언제나 끝난다 --\n");

    /* 칸을 꽉 채운 이름. NUL 이 없다 — 표식이 그렇게 실려 올 수 있다. */
    char full_board[FSD_OTA_MARK_BOARD_LEN + 1];
    char full_stamp[FSD_OTA_MARK_STAMP_LEN + 1];
    memset(full_board, 'B', FSD_OTA_MARK_BOARD_LEN);
    full_board[FSD_OTA_MARK_BOARD_LEN] = '\0';
    memset(full_stamp, 'S', FSD_OTA_MARK_STAMP_LEN);
    full_stamp[FSD_OTA_MARK_STAMP_LEN] = '\0';

    uint8_t img[600];
    make_image(img, sizeof(img), 300, full_board, full_stamp, FSD_OTA_ESP_MAGIC,
               FSD_OTA_CHIP_ESP32S3);
    FsdOtaScan s;
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, sizeof(img), 128);

    /* 🔴 화면에 그대로 나가는 값이다. 안 끝나면 읽는 쪽이 뒤 메모리를 따라간다. */
    CHECK(s.mark.board[FSD_OTA_MARK_BOARD_LEN] == '\0', "보드 이름이 끝난다");
    CHECK(s.mark.stamp[FSD_OTA_MARK_STAMP_LEN] == '\0', "판번호가 끝난다");
    CHECK(strlen(s.mark.board) == FSD_OTA_MARK_BOARD_LEN, "길이 %u", (unsigned)strlen(s.mark.board));
}

static void test_the_first_mark_wins(void) {
    printf("\n-- 표식이 둘이면 앞의 것을 쓴다 --\n");

    /* 🔴 이미지 안에 표식이 둘 있을 수 있나? 우리 빌드는 하나만 넣는다. 그런데
     * 누가 우리 이미지 뒤에 다른 표식을 덧붙이면 둘이 된다. 뒤엣것을 쓰면
     * 앞의 진짜를 덮어쓰는 셈이라, 앞의 것을 쓰고 그 뒤는 안 본다. */
    uint8_t img[600];
    make_image(img, sizeof(img), 100, OUR_BOARD, OUR_STAMP, FSD_OTA_ESP_MAGIC,
               FSD_OTA_CHIP_ESP32S3);
    /* 두 번째 표식을 뒤에 덧쓴다 — 다른 보드 이름으로. */
    {
        FsdOtaMarkWire w;
        memset(&w, 0, sizeof(w));
        w.magic[0] = FSD_OTA_MARK_MAGIC_0; w.magic[1] = FSD_OTA_MARK_MAGIC_1;
        w.magic[2] = FSD_OTA_MARK_MAGIC_2; w.magic[3] = FSD_OTA_MARK_MAGIC_3;
        w.magic[4] = FSD_OTA_MARK_MAGIC_4; w.magic[5] = FSD_OTA_MARK_MAGIC_5;
        w.magic[6] = FSD_OTA_MARK_MAGIC_6; w.magic[7] = (char)FSD_OTA_MARK_MAGIC_7;
        strncpy(w.board, "ttgo-tdisplay", sizeof(w.board));
        strncpy(w.stamp, "9999-99-99 99:99:99 deadbeef", sizeof(w.stamp));
        memcpy(img + 400, &w, sizeof(w));
    }

    FsdOtaScan s;
    fsd_ota_scan_init(&s);
    feed_in_chunks(&s, img, sizeof(img), 128);
    CHECK(strcmp(s.mark.board, OUR_BOARD) == 0, "앞의 표식이 이긴다, got %s", s.mark.board);
    CHECK(fsd_ota_scan_verdict(&s, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_OK,
          "그래서 통과한다");
}

static void test_quiet_cases(void) {
    printf("\n-- NULL 과 빈 것 --\n");

    FsdOtaScan s;
    fsd_ota_scan_init(&s);
    fsd_ota_scan_feed(&s, NULL, 100u); /* 안 죽는다 */
    fsd_ota_scan_feed(&s, (const uint8_t*)"x", 0u);
    CHECK(s.seen == 0u, "아무것도 안 셌다, got %u", (unsigned)s.seen);

    fsd_ota_scan_init(NULL); /* 안 죽는다 */
    CHECK(fsd_ota_scan_verdict(NULL, OUR_BOARD, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_TOO_SHORT,
          "NULL 은 판정하지 않는다");
    CHECK(fsd_ota_scan_verdict(&s, NULL, FSD_OTA_CHIP_ESP32S3) == FSD_OTA_IMG_TOO_SHORT,
          "비교할 보드 이름이 없으면 판정하지 않는다");
}

static void test_names(void) {
    printf("\n-- 판정마다 이름이 있다 --\n");

    const FsdOtaImgVerdict all[] = {
        FSD_OTA_IMG_OK,       FSD_OTA_IMG_TOO_SHORT,   FSD_OTA_IMG_NOT_AN_IMAGE,
        FSD_OTA_IMG_WRONG_CHIP, FSD_OTA_IMG_NO_MARK,   FSD_OTA_IMG_WRONG_BOARD,
    };
    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        const char* n = fsd_ota_img_verdict_str(all[i]);
        CHECK(n && n[0] != '\0', "판정 %u 에 이름이 있다", (unsigned)all[i]);
    }
    /* 🔴 모르는 값도 이름이 있어야 한다. 화면에 숫자가 나가면 안 된다. */
    CHECK(fsd_ota_img_verdict_str((FsdOtaImgVerdict)200)[0] != '\0', "모르는 값에도 이름이 있다");
}

int main(void) {
    printf("test_ota_image: 이 이미지가 이 보드의 것인가\n");
    test_our_own_image_passes();
    test_the_mark_may_straddle_a_chunk_boundary();
    test_what_it_refuses();
    test_it_answers_with_the_outermost_reason();
    test_a_partial_head_is_not_a_verdict();
    test_strings_always_terminate();
    test_the_first_mark_wins();
    test_quiet_cases();
    test_names();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
