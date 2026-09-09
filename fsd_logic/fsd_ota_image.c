#include "fsd_ota_image.h"

#include <string.h>

static const uint8_t MAGIC[FSD_OTA_MARK_MAGIC_LEN] = {
    (uint8_t)FSD_OTA_MARK_MAGIC_0, (uint8_t)FSD_OTA_MARK_MAGIC_1,
    (uint8_t)FSD_OTA_MARK_MAGIC_2, (uint8_t)FSD_OTA_MARK_MAGIC_3,
    (uint8_t)FSD_OTA_MARK_MAGIC_4, (uint8_t)FSD_OTA_MARK_MAGIC_5,
    (uint8_t)FSD_OTA_MARK_MAGIC_6, (uint8_t)FSD_OTA_MARK_MAGIC_7,
};

void fsd_ota_scan_init(FsdOtaScan* s) {
    if(!s) return;
    memset(s, 0, sizeof(*s));
}

/* 이미지 안의 72바이트를 읽어 쓸 수 있는 문자열로 옮긴다.
 *
 * 🔴 `strncpy` 가 아니라 memcpy + 강제 NUL 이다. 표식의 칸은 꽉 찰 수 있고
 * (보드 이름이 정확히 24자), 그러면 원본에 NUL 이 없다. 그 값이 그대로 화면에
 * 나가므로, 안 끝나면 읽는 쪽이 뒤 메모리를 따라간다.
 *
 * ⚠️ 그 두 줄은 **오늘 도달 불가이고, 돌연변이가 그것을 증명했다**
 * (2026-09-10): 지우면 아무 시험도 안 깨진다. `fsd_ota_scan_init()` 이 구조체
 * 전체를 0 으로 채우므로 마지막 칸이 이미 0 이기 때문이다.
 *
 * 🔴 그래도 남긴다. 이 함수의 계약은 "72바이트를 읽어 **끝나는** 문자열을
 * 만든다" 이고, 그 계약이 저 멀리 init 의 memset 에 매달려 있으면 읽는 사람이
 * 그것을 알 길이 없다. 초록 화면을 이 두 줄이 도는 증거로 읽지 말 것. */
static void take_mark(FsdOtaMark* out, const uint8_t* at) {
    memcpy(out->board, at + FSD_OTA_MARK_MAGIC_LEN, FSD_OTA_MARK_BOARD_LEN);
    out->board[FSD_OTA_MARK_BOARD_LEN] = '\0';
    memcpy(out->stamp, at + FSD_OTA_MARK_MAGIC_LEN + FSD_OTA_MARK_BOARD_LEN,
           FSD_OTA_MARK_STAMP_LEN);
    out->stamp[FSD_OTA_MARK_STAMP_LEN] = '\0';
}

/* 창 하나에서 **첫** 표식을 찾는다.
 *
 * 앞의 조각에서 남긴 꼬리(최대 71바이트)와 새 조각을 한 줄로 놓고 훑으므로,
 * 조각 경계에 걸친 표식도 정확히 한 번 보인다.
 *
 * 🔴 부르는 쪽이 `!s->found` 를 보장한다 — 그래서 여기에 그 검사가 없다.
 * 한때 있었는데, **어떤 시험도 그것을 켜고 끌 수 없었다**(돌연변이로 확인,
 * 2026-09-10): 바깥에서 이미 막고 있어 안쪽 가드는 도달 불가였다. 도달 불가인
 * 가드는 게이트처럼 보이면서 아무것도 안 하고, 그것이 이 저장소가 반복해서
 * 물린 자리다. 앞의 것이 이긴다는 규칙은 **바깥의 조기 반환**이 지킨다. */
static void scan_window(FsdOtaScan* s, const uint8_t* win, uint32_t len) {
    if(len < FSD_OTA_MARK_LEN) return;
    for(uint32_t i = 0; i + FSD_OTA_MARK_LEN <= len; i++) {
        if(memcmp(win + i, MAGIC, FSD_OTA_MARK_MAGIC_LEN) != 0) continue;
        take_mark(&s->mark, win + i);
        s->found = true;
        return;
    }
}

void fsd_ota_scan_feed(FsdOtaScan* s, const uint8_t* data, uint32_t len) {
    if(!s || !data || len == 0u) return;

    /* 머리말은 앞에서 딱 한 번 모은다. */
    while(s->head_len < FSD_OTA_HEAD_MIN && len > 0u) {
        s->head[s->head_len++] = *data;
        data++;
        len--;
        s->seen++;
        /* 머리말 바이트도 표식의 일부일 수는 없다(표식은 rodata 에 있고 머리말은
         * 이미지 맨 앞 24바이트다). 그래도 꼬리에 넣어 둔다 — 이 층이 이미지
         * 배치에 대해 아는 것을 하나라도 줄이는 편이 낫다. */
        if(s->tail_len < (uint8_t)(FSD_OTA_MARK_LEN - 1u)) {
            s->tail[s->tail_len++] = s->head[s->head_len - 1u];
        } else {
            memmove(s->tail, s->tail + 1, (size_t)(FSD_OTA_MARK_LEN - 2u));
            s->tail[FSD_OTA_MARK_LEN - 2u] = s->head[s->head_len - 1u];
        }
    }
    if(len == 0u) return;

    s->seen += len;
    if(s->found) {
        /* 이미 찾았으면 더 훑지 않는다. 남은 수십만 바이트를 매 조각마다
         * 비교할 이유가 없고, 앞의 것이 이기는 규칙과도 맞는다. 꼬리는
         * 갱신하지 않아도 되지만, 상태가 반쯤 낡는 것보다 안 쓰는 편이 낫다. */
        return;
    }

    /* 겹침 + 새 조각. 조각이 아주 크면 창을 나눠 돌리는 대신 조각 안을 직접
     * 훑고 경계만 따로 본다 — 큰 버퍼를 스택에 잡지 않기 위해서다. */
    {
        uint8_t win[(FSD_OTA_MARK_LEN - 1u) * 2u];
        const uint32_t carry = s->tail_len;
        uint32_t take = FSD_OTA_MARK_LEN - 1u;
        if(take > len) take = len;

        if(carry > 0u) {
            memcpy(win, s->tail, carry);
            memcpy(win + carry, data, take);
            scan_window(s, win, carry + take);
        }
    }

    if(!s->found) scan_window(s, data, len);

    /* 새 꼬리: 이 조각의 마지막 (MARK_LEN - 1) 바이트. 조각이 그보다 짧으면
     * 앞의 꼬리를 밀어내며 이어 붙인다. */
    {
        const uint32_t keep = FSD_OTA_MARK_LEN - 1u;
        if(len >= keep) {
            memcpy(s->tail, data + (len - keep), keep);
            s->tail_len = (uint8_t)keep;
        } else {
            const uint32_t drop = (uint32_t)s->tail_len + len > keep
                                      ? ((uint32_t)s->tail_len + len) - keep
                                      : 0u;
            if(drop > 0u) {
                memmove(s->tail, s->tail + drop, (size_t)(s->tail_len - drop));
                s->tail_len = (uint8_t)(s->tail_len - drop);
            }
            memcpy(s->tail + s->tail_len, data, len);
            s->tail_len = (uint8_t)(s->tail_len + len);
        }
    }
}

FsdOtaImgVerdict fsd_ota_scan_verdict(const FsdOtaScan* s, const char* want_board,
                                      uint16_t want_chip) {
    if(!s || !want_board) return FSD_OTA_IMG_TOO_SHORT;
    if(s->head_len < FSD_OTA_HEAD_MIN) return FSD_OTA_IMG_TOO_SHORT;

    /* 🔴 가장 바깥부터. "이미지가 아니다" 를 "표식이 없다" 로 답하면, 파일을
     * 잘못 고른 사람이 빌드를 의심하게 된다. */
    if(s->head[0] != (uint8_t)FSD_OTA_ESP_MAGIC) return FSD_OTA_IMG_NOT_AN_IMAGE;

    {
        const uint16_t chip = (uint16_t)((uint16_t)s->head[FSD_OTA_CHIP_OFFSET] |
                                         ((uint16_t)s->head[FSD_OTA_CHIP_OFFSET + 1u] << 8));
        if(chip != want_chip) return FSD_OTA_IMG_WRONG_CHIP;
    }

    if(!s->found) return FSD_OTA_IMG_NO_MARK;
    if(strncmp(s->mark.board, want_board, FSD_OTA_MARK_BOARD_LEN) != 0)
        return FSD_OTA_IMG_WRONG_BOARD;

    return FSD_OTA_IMG_OK;
}

const char* fsd_ota_img_verdict_str(FsdOtaImgVerdict v) {
    switch(v) {
    case FSD_OTA_IMG_OK: return "ok";
    case FSD_OTA_IMG_TOO_SHORT: return "아직 다 안 왔다";
    case FSD_OTA_IMG_NOT_AN_IMAGE: return "펌웨어 파일이 아니다";
    case FSD_OTA_IMG_WRONG_CHIP: return "다른 칩용이다";
    case FSD_OTA_IMG_NO_MARK: return "우리가 구운 파일이 아니다";
    case FSD_OTA_IMG_WRONG_BOARD: return "다른 보드용이다";
    }
    /* 🔴 모르는 값에도 이름이 있어야 한다. 화면에 숫자가 나가면 안 된다 —
     * 이 저장소의 네 번째 패턴이다. */
    return "모르는 판정";
}
