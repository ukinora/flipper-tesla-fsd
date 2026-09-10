#include "fsd_ota_gate.h"

/*
 * 🔴 순서가 곧 지시다. 바깥부터 — 헤더의 표를 그대로 코드로 옮긴 것이고,
 * `test_it_answers_with_the_outermost_reason()` 이 그 순서를 못 박는다.
 * 줄을 위아래로 옮기면 빨개진다.
 */
FsdOtaBeginVerdict fsd_ota_begin_check(const FsdOtaBeginIn* in) {
    /* 🔴 NULL 은 "물어볼 것이 없다" 이지 "괜찮다" 가 아니다. 통과로 답하면 이
     * 게이트는 부르는 쪽의 실수 하나로 통째로 사라진다. BUSY 를 고른 이유는
     * 그것이 **아무것도 안 하고 기다리라**는 뜻이라서다. */
    if (!in) return FSD_OTA_BEGIN_BUSY;

    /* ── 모듈의 형편 ── */
    if (in->transfer_running) return FSD_OTA_BEGIN_BUSY;
    if (in->saving_capture) return FSD_OTA_BEGIN_SAVING;

    /* ── 차의 형편 ──
     * "움직이는 것을 보았다" 만 막는다. 속도 프레임이 안 오는 것은 여기에 안
     * 든다 — 헤더의 그 절을 읽을 것. */
    if (in->motion_seen) return FSD_OTA_BEGIN_MOVING;

    /* ── 권한 ── */
    if (!in->owner_window_open) return FSD_OTA_BEGIN_NOT_ARMED;

    /* ── 이 요청 자체 ── */
    if (in->slot_bytes == 0u) return FSD_OTA_BEGIN_NO_SLOT;
    if (in->declared_bytes < FSD_OTA_MIN_IMAGE) return FSD_OTA_BEGIN_TOO_SMALL;
    if (in->declared_bytes > in->slot_bytes) return FSD_OTA_BEGIN_TOO_BIG;

    return FSD_OTA_BEGIN_OK;
}

const char* fsd_ota_begin_verdict_str(FsdOtaBeginVerdict v) {
    switch (v) {
    case FSD_OTA_BEGIN_OK: return "ok";
    case FSD_OTA_BEGIN_BUSY: return "이미 하나 받는 중이다";
    case FSD_OTA_BEGIN_SAVING: return "캡처를 저장하는 중이다";
    case FSD_OTA_BEGIN_MOVING: return "차가 움직이고 있다";
    case FSD_OTA_BEGIN_NOT_ARMED: return "차 안에서 버튼을 세 번 눌러야 한다";
    case FSD_OTA_BEGIN_NO_SLOT: return "이 판에는 받을 칸이 없다";
    case FSD_OTA_BEGIN_TOO_SMALL: return "펌웨어 파일이 아니다 (너무 작다)";
    case FSD_OTA_BEGIN_TOO_BIG: return "칸보다 크다";
    }
    /* 🔴 모르는 값에도 이름이 있어야 한다 — 네 번째 패턴. */
    return "모르는 판정";
}

void fsd_ota_xfer_init(FsdOtaXfer* x, uint32_t declared) {
    if (!x) return;
    x->declared = declared;
    x->written = 0u;
    /* 0 은 머리말 번호다. 데이터는 1 부터. */
    x->next_seq = 1u;
}

/* 다음에 와야 할 번호.
 *
 * 🔴 16비트라 큰 이미지에서 한 바퀴 돈다. 0 은 머리말이 쓰므로 65535 다음은
 * **1** 이다. 오늘 이미지(689 KB)와 오늘 MTU(조각 500 B)로는 1,378 조각이라
 * 도달 불가이고, MTU 23 으로 떨어져도 34,450 조각이라 아직 안 돈다. 그래도
 * 세 줄로 닫아 두는 이유는, 안 닫으면 **멀쩡한 전송이 끝에서 거절**되고 그
 * 증상은 "왜 큰 파일만 안 되지" 로만 보이기 때문이다. */
static uint16_t next_after(uint16_t seq) { return (seq == 0xFFFFu) ? 1u : (uint16_t)(seq + 1u); }

FsdOtaChunkVerdict fsd_ota_xfer_take(FsdOtaXfer* x, uint16_t seq, uint32_t n) {
    if (!x) return FSD_OTA_CHUNK_SEQUENCE;

    /* 🔴 빈 조각을 먼저 본다. 순번보다 먼저인 이유는 **순번을 안 먹게** 하기
     * 위해서다 — 알맹이 없는 프레임 하나가 다음 진짜 조각을 순번 오류로
     * 만들면, 멀쩡한 전송이 프레이밍 사고 하나로 죽는다. */
    if (n == 0u) return FSD_OTA_CHUNK_EMPTY;

    if (seq != x->next_seq) return FSD_OTA_CHUNK_SEQUENCE;

    /* 🔴 넘치는지 먼저 보고 나서 센다. 세고 나서 보면 이미 늦다 —
     * `written` 이 선언을 넘은 채로 남아 완료로 읽힌다. */
    if (n > x->declared - x->written) return FSD_OTA_CHUNK_OVERRUN;

    x->written += n;
    x->next_seq = next_after(seq);
    return FSD_OTA_CHUNK_OK;
}

bool fsd_ota_xfer_complete(const FsdOtaXfer* x) {
    if (!x) return false;
    /* 🔴 0 바이트짜리는 결코 완료가 아니다. `>=` 만 쓰면 `0 >= 0` 이 참이 되어
     * **시작하자마자 끝난 전송**이 되고, 그다음 줄이 부팅 파티션을 바꾼다. */
    if (x->declared == 0u) return false;
    return x->written >= x->declared;
}

const char* fsd_ota_chunk_verdict_str(FsdOtaChunkVerdict v) {
    switch (v) {
    case FSD_OTA_CHUNK_OK: return "ok";
    case FSD_OTA_CHUNK_SEQUENCE: return "조각 순서가 어긋났다";
    case FSD_OTA_CHUNK_OVERRUN: return "말한 크기보다 많이 왔다";
    case FSD_OTA_CHUNK_EMPTY: return "빈 조각이다";
    }
    return "모르는 판정";
}
