#pragma once
/*
 * ota_store.h — 폰이 보낸 펌웨어를 다음 칸에 굽는다.
 *
 * WHAT THIS IS
 * ------------
 * BLE OTA 의 **2층**. 판정 둘은 이 파일에 없다:
 *
 *   `fsd_logic/fsd_ota_gate.h`  — 시작해도 되는가 (버튼 창 · 움직임 · 크기)
 *   `fsd_logic/fsd_ota_image.h` — 이 이미지가 이 보드의 것인가 (표식)
 *
 * 🔴 일부러 그렇게 갈랐다. 이 파일은 ESP-IDF 를 부르므로 **호스트 시험이 못
 * 닿는다**. 못 닿는 곳에 판단을 두는 것이 이 저장소의 열 번째 패턴이고, 이미
 * 두 번(OTA 판정 · 배터리 %) 값을 치렀다. 여기 있는 것은 순서와 자원 관리뿐이다.
 *
 * 🔴 어느 태스크가 무엇을 하나 — 이것이 이 파일의 설계 전부다
 * -----------------------------------------------------------
 * `esp_ota_begin()` 은 **칸을 지운다.** 689 KB 면 1~3 초 걸리는 블로킹 호출이고,
 * `esp_ota_end()` 는 쓴 것을 전부 다시 읽어 해시를 낸다. 둘 다 BLE 호스트
 * 태스크에서 하면 안 된다.
 *
 *   BLE 태스크  : `ota_store_chunk()` 만 — 조각 하나를 플래시에 쓴다 (~1 ms)
 *   loop()      : `ota_store_begin()` · `ota_store_finish()` · `ota_store_tick()`
 *
 * 부르는 쪽(`ble_server.cpp`)이 머리말을 파킹해서 loop() 로 넘긴다 — 블랙박스
 * 켜기(`BLE_CMD_BB_ENABLE`)가 같은 이유로 이미 같은 모양이다.
 *
 * 🔴 그래서 뮤텍스가 하나 있다. 시한이 끊는 순간과 조각이 오는 순간이 겹치면
 * 핸들이 해제된 뒤에 쓰인다.
 *
 * 🔴 이 층이 막지 못하는 것
 * -------------------------
 * **돌긴 도는데 잘못 동작하는 이미지.** 안 도는 판은 `main.cpp` 의 자가진단이
 * 되돌리지만(`fsd_selftest.c`), 도는 판은 아무도 못 잡는다. USB 로 굽던 동안에는
 * 여덟 보드 빌드와 호스트 시험을 통과한 것만 올라갔고, 폰에서 굽는 길이 생긴
 * 지금 그 규율은 **사람이** 지켜야 한다.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 결과 코드. BLE 결과 프레임의 `detail` 바이트로 그대로 나가고, 앱이 이것을
 * 사람 말로 옮긴다. 구간을 나눠 쓰는 이유는 세 판정(시작 · 조각 · 정체)이
 * 각자 0 부터 세기 때문이다 — 한 줄로 합치면 서로를 덮는다. */
#define OTA_ST_OK 0u
/** 1..9 : 시작 거절. `FsdOtaBeginVerdict` + 1. */
#define OTA_ST_BEGIN_BASE 1u
/** 20..29 : 조각 거절. `FsdOtaChunkVerdict` + 20. */
#define OTA_ST_CHUNK_BASE 20u
/** 30..39 : 이미지 정체 거절. `FsdOtaImgVerdict` + 30. */
#define OTA_ST_IMG_BASE 30u
/* 40.. : 이 층 자신의 실패 — ESP-IDF 가 거절했다. */
#define OTA_ST_ESP_BEGIN 40u   /* 칸을 못 열었다 */
#define OTA_ST_ESP_WRITE 41u   /* 플래시에 못 썼다 */
#define OTA_ST_ESP_END 42u     /* 이미지가 깨졌다 (해시 불일치 등) */
#define OTA_ST_ESP_BOOT 43u    /* 부팅 칸을 못 바꿨다 */
#define OTA_ST_NOT_RUNNING 44u /* 시작하지 않았는데 조각이 왔다 */
#define OTA_ST_STALLED 45u     /* 조각이 끊겼다 — 칸을 놓아 주었다 */

#ifdef __cplusplus
extern "C" {
#endif

/** 한 번만, setup() 에서. 뮤텍스를 만든다. */
void ota_store_init(void);

/**
 * 게이트를 묻고, 통과하면 다음 칸을 열어 지운다. **loop() 에서만.**
 *
 * `motion_seen`·`saving_capture` 는 이 파일이 알 수 없는 바깥 사정이라 넣어
 * 준다 — 전역을 들여다보게 만들면 사본이 늘고 시험이 못 닿는다.
 */
uint8_t ota_store_begin(uint32_t total_bytes, uint8_t xfer_id, bool motion_seen,
                        bool saving_capture);

/**
 * 지금 세션의 번호. 세션이 없으면 0.
 *
 * 🔴🔴 **이것이 없어서 여덟 번을 헤맸다 (2026-09-10).** 결과 프레임에는 *어느
 * 전송의 것인가* 가 없었다. 그래서 앞선 전송이 끝난 뒤 늦게 배달된 실패 통지를
 * **다음 전송이 자기 것으로 읽고 스스로 멈췄다** — 시작 0.9 초 만에. 한 번
 * 실패하면 그다음이 전부 같은 자리에서 죽었고, **실패가 자기를 재생산했다.**
 *
 * 이제 결과의 `extra` 는 `(번호 << 8) | 코드` 이고, 앱은 자기 번호가 아닌 것을
 * 버린다.
 */
uint8_t ota_store_xfer_id(void);

/** 조각 하나를 쓴다. **BLE 태스크에서.** */
uint8_t ota_store_chunk(uint16_t seq, const uint8_t* data, size_t n);

/** 선언한 만큼 다 왔다 — loop() 가 이것을 보고 `ota_store_finish()` 를 부른다. */
bool ota_store_ready_to_finish(void);

/**
 * 닫고, 이미지 정체를 묻고, 통과하면 **부팅 칸을 바꾼다.** loop() 에서만.
 *
 * 🔴 정체 판정은 여기서, 즉 **전부 받은 뒤에** 한다. 1층 헤더가 그 계약을
 * 명시한다: 표식을 *아직 못 찾은 것*과 *없는 것*은 다르다.
 */
uint8_t ota_store_finish(void);

/** 시한을 본다. 끊었으면 true. loop() 에서만. */
bool ota_store_tick(uint32_t now_ms);

/** 지금 받는 중인가. */
bool ota_store_busy(void);

/** 지금까지 쓴 바이트. */
uint32_t ota_store_progress(void);

/** 폰이 사라졌다 — 받던 것을 접는다. loop() 에서만. */
void ota_store_abandon(const char* why);

#ifdef __cplusplus
}
#endif
