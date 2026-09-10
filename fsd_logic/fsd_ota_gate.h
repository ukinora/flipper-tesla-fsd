#pragma once

/*
 * fsd_ota_gate.h — 펌웨어를 **받기 시작해도 되는가**, 그리고 조각이 제대로 오는가.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * 설치 자체(`esp_ota_begin/write/end`)는 ESP-IDF 를 부르는 일이라 호스트 시험이
 * 못 닿는다. 그런데 **거절할지 말지**는 순수한 판단이다. 그 판단이 ESP32 사본
 * 안에만 있으면 이 저장소의 **열 번째 패턴**이 된다 — 실제로 도는 코드가 시험이
 * 도는 코드보다 넓은 자리. 이 저장소는 그 자리에서 이미 두 번 다쳤다(OTA 판정,
 * 배터리 %). 그래서 판단만 여기로 꺼냈고, 글루는 판단을 부르기만 한다.
 *
 * 무엇을 막고 무엇을 안 막나
 * --------------------------
 * 🟢 막는 것: 겹친 전송 · 캡처를 쓰는 중 · **움직이는 차** · 버튼 창이 닫힘 ·
 *    받을 칸이 없음 · 칸보다 큰 이미지 · 이미지일 수 없는 크기 · 순번이 어긋난
 *    조각 · 선언보다 많이 보내는 것.
 *
 * 🔴 안 막는 것: **이미지의 내용**. 그것은 `fsd_ota_image.h` 가 본다(우리가
 *    구운 것인가 · 이 보드 것인가). 그리고 그 층도 서명이 아니다 — 일부러 나쁘게
 *    만든 우리 이미지는 아무도 못 막고, 차주가 2026-09-10 에 서명 없이 가기로
 *    정했다. 안 도는 판은 `fsd_selftest.c` 의 자가진단이 되돌린다.
 *
 * 🔴 순서가 내용이다
 * ------------------
 * 조건이 여럿 참일 때 **무엇을 먼저 말하느냐**가 차 옆에 앉은 사람의 다음 행동을
 * 정한다. *"버튼을 누르세요"* 는 전송이 이미 돌고 있을 때는 틀린 지시다 — 눌러도
 * 아무 일도 안 일어난다. 규칙은 1층과 같다: **바깥부터.**
 *
 *      모듈의 형편 (받는 중 · 쓰는 중)
 *          → 차의 형편 (움직임)
 *              → 권한 (버튼 창)
 *                  → 이 요청 자체 (칸 · 크기)
 *
 * 🔴 "안 움직인다" 를 증명하라고 하지 않는다
 * -------------------------------------------
 * 차가 자리를 잡으면 구동 인버터가 잠들고 `0x257`(속도)이 **버스에서 통째로
 * 사라진다.** 그때가 정확히 사람이 폰으로 굽고 싶은 때다. 그래서 게이트는
 * *정차를 증명하라* 가 아니라 ***움직이는 것을 보았다면 거절한다*** 이다.
 *
 * 반대로 쓰면 미러가 물렸던 그 함정을 그대로 다시 판다 — 기어 신호가 없어서
 * P 를 증명 못 해 거절하던 자리(`axis: no gear signal`, 2026-09-08).
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 이보다 작으면 펌웨어 이미지일 수 없다.
 *
 *  우리 가장 작은 보드 판이 약 630 KB 다. 64 KB 는 넉넉하게 아래에 둔 값이고,
 *  이 검사가 잡으려는 것은 공격이 아니라 **잘못 고른 파일**이다 — 사진이나
 *  텍스트를 보내면 `esp_ota_end` 까지 가기 전에 여기서 끝난다. */
#define FSD_OTA_MIN_IMAGE 65536u

/** 조각이 이만큼 안 오면 전송을 접고 칸을 놓아 준다.
 *
 *  🔴 시한이 없으면 폰이 그냥 사라졌을 때 칸이 영원히 잡혀 있고, 다음 시도가
 *  `BUSY` 로 거절된다 — 그리고 그 증상은 차 옆에서 "왜 안 되지" 로만 보인다.
 *  실측 전송은 689 KB 를 55.8 KB/s 로 약 13 초다. */
#define FSD_OTA_STALL_MS 15000u

typedef struct {
    /** 이미 하나 받고 있다. */
    bool transfer_running;
    /** 블랙박스가 지금 캡처를 플래시에 쓰고 있다. */
    bool saving_capture;
    /** 차가 움직이는 것을 **보았다**. 속도 프레임이 없는 것은 여기에 안 든다. */
    bool motion_seen;
    /** 버튼 창이 열려 있다 — 누군가 차 안에서 버튼을 눌렀다는 뜻. */
    bool owner_window_open;
    /** 다음 OTA 칸의 크기. 0 = 칸이 없다(단일 앱 파티션 표). */
    uint32_t slot_bytes;
    /** 폰이 말한 이미지 크기. */
    uint32_t declared_bytes;
} FsdOtaBeginIn;

typedef enum {
    FSD_OTA_BEGIN_OK = 0,
    /** 이미 하나 받는 중이다. 기다리거나 그것을 먼저 접어야 한다. */
    FSD_OTA_BEGIN_BUSY,
    /** 캡처를 쓰는 중이다. 곧 끝난다. */
    FSD_OTA_BEGIN_SAVING,
    /** 차가 움직인다. */
    FSD_OTA_BEGIN_MOVING,
    /** 버튼 창이 닫혀 있다 — 차 안에서 버튼을 세 번 눌러야 한다. */
    FSD_OTA_BEGIN_NOT_ARMED,
    /** 이 판에는 받을 칸이 없다. */
    FSD_OTA_BEGIN_NO_SLOT,
    /** 펌웨어 이미지일 수 없는 크기다. */
    FSD_OTA_BEGIN_TOO_SMALL,
    /** 칸보다 크다. */
    FSD_OTA_BEGIN_TOO_BIG,
} FsdOtaBeginVerdict;

/** 시작해도 되는가. `in` 이 NULL 이면 **통과가 아니라 거절**이다. */
FsdOtaBeginVerdict fsd_ota_begin_check(const FsdOtaBeginIn* in);

/** 사람이 읽을 이름. 모르는 값에도 이름이 있다 — 화면에 숫자가 나가면 안 된다. */
const char* fsd_ota_begin_verdict_str(FsdOtaBeginVerdict v);

/**
 * 흘러 들어오는 전송의 자리.
 *
 * 🔴 순번이 빠지면 이미지에 구멍이 나고, 구멍 난 이미지는 `esp_ota_end` 의
 * 해시가 잡아 주기는 하지만 **그때는 이미 칸을 다 지우고 다 쓴 뒤**다. 여기서
 * 먼저 잡으면 잘못된 조각 하나에서 끝난다.
 */
typedef struct {
    uint32_t declared;
    uint32_t written;
    /** 다음에 와야 할 번호. 0 은 머리말이라 데이터로는 안 쓴다. */
    uint16_t next_seq;
} FsdOtaXfer;

typedef enum {
    FSD_OTA_CHUNK_OK = 0,
    /** 건너뛰었거나 되돌아왔다. */
    FSD_OTA_CHUNK_SEQUENCE,
    /** 선언한 것보다 많이 보냈다. */
    FSD_OTA_CHUNK_OVERRUN,
    /** 알맹이가 없다. */
    FSD_OTA_CHUNK_EMPTY,
} FsdOtaChunkVerdict;

/** 새 전송을 연다. `declared` 는 폰이 말한 전체 크기. */
void fsd_ota_xfer_init(FsdOtaXfer* x, uint32_t declared);

/**
 * 조각 하나를 받아들일지 정하고, 받아들이면 센다.
 *
 * 🔴 거절된 조각은 **세지 않는다.** 세면 구멍이 난 채로 완료가 되고, 그 이미지가
 * 부팅 파티션이 된다.
 */
FsdOtaChunkVerdict fsd_ota_xfer_take(FsdOtaXfer* x, uint16_t seq, uint32_t n);

/** 선언한 만큼 다 왔는가. 0 바이트 전송은 **결코 완료가 아니다.** */
bool fsd_ota_xfer_complete(const FsdOtaXfer* x);

/** 사람이 읽을 이름. */
const char* fsd_ota_chunk_verdict_str(FsdOtaChunkVerdict v);

#ifdef __cplusplus
}
#endif
