#pragma once

/*
 * fsd_ota_image.h — 들어오는 펌웨어 이미지가 **이 보드의 것인가**.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * SECURITY.md 의 "Preconditions if OTA is ever added back" 두 번째 칸은
 * *"Board type and firmware version checked before install"* 이고, 그 문서는
 * 그것을 **이미지 속 `esp_app_desc_t` 를 읽어** 채우면 된다고 적어 두었다.
 *
 * 🔴 그런데 우리 이미지를 열어 보니 그 칸에 우리 것이 없다 (2026-09-10 실측):
 *
 *      project_name   'arduino-lib-builder'
 *      version        'esp-idf: v4.4.7 38eeba213a'
 *
 * 아두이노 프레임워크가 미리 구워 둔 기술자다. **모든 Arduino-ESP32 이미지가
 * 같은 값을 낸다** — 다른 프로젝트의 이미지도, 다른 사람의 이미지도. 그것으로
 * 판번호를 검사하면 검사처럼 보이면서 아무것도 안 거른다. 이 저장소가
 * 반복해서 물린 모양이라(늘 통과하는 게이트) 그렇게 두지 않는다.
 *
 * → 그래서 **우리 표식을 이미지에 심는다.** `gen_build_stamp.py` 가 빌드마다
 *   `FsdOtaMarkWire` 하나를 rodata 에 넣고, 받는 쪽이 그것을 찾는다.
 *
 * WHAT THIS CHECKS, AND WHAT IT DOES NOT
 * --------------------------------------
 * 🟢 막는 것
 *   - 이미지가 아닌 것 (첫 바이트 0xE9)
 *   - 다른 칩용 이미지 (`chip_id`) — 그대로 구우면 안 켜진다
 *   - **우리가 굽지 않은 이미지** (표식 없음) — 남의 아두이노 이미지 전부
 *   - **다른 보드용 우리 이미지** (표식의 보드 이름)
 *
 * 🔴 막지 못하는 것: **일부러 나쁘게 만든 우리 이미지.** 표식은 서명이 아니고
 *   베낄 수 있다. 그것을 막는 것은 서명이고, 차주가 2026-09-10 에 서명 없이
 *   가기로 정했다 — 그 결정과 그 대가는 SECURITY.md 에 적혀 있다.
 *   **이 파일을 "이미지를 검증한다" 로 읽으면 안 된다.** 이것이 거르는 것은
 *   사고이지 공격이 아니다.
 *
 * 이 앞에 서 있는 것은 그대로다: 주인 폰만 · 버튼 창(차 안에 있어야 한다) ·
 * 안 도는 판은 자가진단이 되돌린다.
 *
 * WHY A SCANNER AND NOT A FIXED OFFSET
 * ------------------------------------
 * 표식은 평범한 `const` 배열이라 링커가 rodata 어디에나 놓는다. 자리를
 * 고정하려면 링커 스크립트를 건드려야 하는데, 이 빌드는 아두이노 프레임워크를
 * 라이브러리로 쓰고 여덟 보드가 각자 링커 스크립트를 갖는다. 훑는 편이 싸고
 * 어디서도 안 깨진다 — 그리고 훑기는 **조각 경계에 걸친 표식**을 놓치지
 * 말아야 하므로, 그 겹침이 이 파일에서 시험이 가장 필요한 부분이다.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 표식의 앞머리. 8바이트, NUL 없음 — 이미지 안의 다른 문자열과 우연히 겹칠
 *  확률을 없애려고 마지막 바이트를 인쇄 불가 값으로 둔다. */
#define FSD_OTA_MARK_MAGIC_0 'F'
#define FSD_OTA_MARK_MAGIC_1 'S'
#define FSD_OTA_MARK_MAGIC_2 'D'
#define FSD_OTA_MARK_MAGIC_3 '-'
#define FSD_OTA_MARK_MAGIC_4 'O'
#define FSD_OTA_MARK_MAGIC_5 'T'
#define FSD_OTA_MARK_MAGIC_6 'A'
#define FSD_OTA_MARK_MAGIC_7 0x01u

#define FSD_OTA_MARK_MAGIC_LEN 8u
#define FSD_OTA_MARK_BOARD_LEN 24u /* platformio env 이름. 지금 최장 22자 */
#define FSD_OTA_MARK_STAMP_LEN 40u /* "YYYY-MM-DD HH:MM:SS <rev>[-dirty]" */
#define FSD_OTA_MARK_LEN (FSD_OTA_MARK_MAGIC_LEN + FSD_OTA_MARK_BOARD_LEN + FSD_OTA_MARK_STAMP_LEN)

/** 이미지 안에 실제로 놓이는 모양. 문자열은 NUL 로 채운다. */
typedef struct {
    char magic[FSD_OTA_MARK_MAGIC_LEN];
    char board[FSD_OTA_MARK_BOARD_LEN];
    char stamp[FSD_OTA_MARK_STAMP_LEN];
} FsdOtaMarkWire;

/** 읽어 낸 것. 문자열은 언제나 NUL 로 끝난다 — 화면에 그대로 나가므로. */
typedef struct {
    char board[FSD_OTA_MARK_BOARD_LEN + 1u];
    char stamp[FSD_OTA_MARK_STAMP_LEN + 1u];
} FsdOtaMark;

typedef enum {
    FSD_OTA_IMG_OK = 0,
    /** 머리말도 다 못 받았다. 조각이 더 와야 판정할 수 있다. */
    FSD_OTA_IMG_TOO_SHORT,
    /** 첫 바이트가 0xE9 가 아니다 — ESP32 이미지가 아예 아니다. */
    FSD_OTA_IMG_NOT_AN_IMAGE,
    /** 다른 칩용이다. 구우면 안 켜진다. */
    FSD_OTA_IMG_WRONG_CHIP,
    /** 우리 표식이 없다 — 우리가 구운 이미지가 아니다. */
    FSD_OTA_IMG_NO_MARK,
    /** 우리 것인데 다른 보드용이다. */
    FSD_OTA_IMG_WRONG_BOARD,
} FsdOtaImgVerdict;

/* esp_image_header_t 에서 우리가 보는 두 자리. ESP-IDF 의 배치이고 이 둘은
 * v3 부터 안 움직였다 — 그래도 헤더를 통째로 베끼지 않고 필요한 것만 적는다.
 * 전부 적으면 IDF 가 늘릴 때 우리 사본이 조용히 낡는다. */
#define FSD_OTA_ESP_MAGIC 0xE9u
#define FSD_OTA_CHIP_OFFSET 12u /* uint16 little-endian */
#define FSD_OTA_HEAD_MIN 24u    /* esp_image_header_t 의 크기 */

/** ESP32-S3. 우리 보드(T-2CAN)의 칩이다. */
#define FSD_OTA_CHIP_ESP32S3 9u
/** 원래 ESP32. 이 저장소의 다른 일곱 보드가 쓴다. */
#define FSD_OTA_CHIP_ESP32 0u

/**
 * 흘러 들어오는 이미지를 훑는 상태.
 *
 * 🔴 `tail` 이 이 구조체의 존재 이유다. 표식이 조각 경계에 걸치면, 겹침을
 * 안 들고 있는 훑기는 그것을 못 본다 — 그리고 못 본 결과는 `NO_MARK`,
 * 즉 **멀쩡한 우리 이미지를 거부**하는 것이다. 조각 크기는 MTU 가 정하므로
 * 표식이 어디에 떨어질지는 아무도 못 고른다.
 */
typedef struct {
    uint8_t head[FSD_OTA_HEAD_MIN];
    uint8_t head_len;

    uint8_t tail[FSD_OTA_MARK_LEN - 1u];
    uint8_t tail_len;

    uint32_t seen;
    bool found;
    FsdOtaMark mark;
} FsdOtaScan;

/** 처음으로 되돌린다. 새 업로드마다 부른다. */
void fsd_ota_scan_init(FsdOtaScan* s);

/** 조각 하나를 먹인다. 순서대로, 빠짐없이. NULL 과 0 길이는 무해하다. */
void fsd_ota_scan_feed(FsdOtaScan* s, const uint8_t* data, uint32_t len);

/**
 * 지금까지 본 것으로 판정한다.
 *
 * `want_board` 는 이 보드의 platformio env 이름, `want_chip` 은 칩 id.
 * 둘 다 부르는 쪽이 준다 — 이 파일이 자기가 어느 보드에 있는지 알면
 * 호스트 시험이 한 보드밖에 못 본다.
 *
 * 🔴 표식을 아직 못 찾은 것과 없는 것은 **다르다.** 이미지가 다 오기 전에는
 * `NO_MARK` 가 "아직" 일 수 있으므로, 부르는 쪽은 **전부 받은 뒤에** 이것을
 * 믿어야 한다. 그 계약을 지키게 하려고 판정에 진행률을 안 넣었다 — 넣으면
 * "거의 다 왔으니 통과" 같은 것이 생긴다.
 */
FsdOtaImgVerdict fsd_ota_scan_verdict(const FsdOtaScan* s, const char* want_board,
                                      uint16_t want_chip);

/** 사람 말로. 절대 NULL 이 아니다. */
const char* fsd_ota_img_verdict_str(FsdOtaImgVerdict v);

#ifdef __cplusplus
}
#endif
