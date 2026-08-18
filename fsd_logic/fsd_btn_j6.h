#pragma once
/*
 * fsd_btn_j6.h — "이 리포트는 어느 버튼인가?"
 *
 * The Yiser J6 is the remote this project uses (차주 결정, 2026-08-18). It is
 * not a keyboard: it pretends to be a phone touchscreen. One physical button
 * becomes a synthesised swipe or tap, and a long press becomes a consumer
 * control code on a different characteristic.
 *
 * So the usual "which bit of which byte" table cannot describe it, and the one
 * that used to live in ble_central.cpp is gone. Seven buttons arrive on the
 * SAME two report characteristics, and what tells them apart is the payload:
 * the DIRECTION a coordinate travelled, or WHERE a tap landed.
 *
 * MEASURED, NOT GUESSED
 * ---------------------
 * Every constant below came off the wire on 2026-08-18. The captures are in
 * test_btn_j6.c as test vectors, and the full table with how it was taken is in
 * 블루투스-버튼-조사.md. This is the FsdSpEncoding discipline, satisfied rather
 * than deferred: the reason the old table had `verified = false` was that no
 * button had been bought.
 *
 * 🔴 THE EARLIER READING OF THIS DEVICE WAS WRONG, and how it was wrong is the
 * reason this file is careful. A swipe arrives as up to NINE notifications. The
 * report path kept one buffer per slot, so eight of them were overwritten
 * before the loop looked, and from the one or two that survived the device
 * appeared to emit nothing usable. Fixing that (fork PR #59) is what made the
 * shape visible. Seeing less is not the same as there being less.
 *
 * WHY NO HANDLE ARGUMENT
 * ----------------------
 * The two talking characteristics were at handles 0x0027 and 0x0032 on the unit
 * measured. Handles are assigned by the PERIPHERAL, we have seen exactly one
 * unit, and a firmware revision that inserts a service renumbers everything
 * after it. Length plus shape says the same thing and cannot go stale:
 *
 *    5 bytes, b[0] in {0x00, 0x03}, both coords <= 1000  -> digitiser
 *    3 bytes, b[0] == 0x00                                -> consumer control
 *
 * Anything else is not this device speaking and decodes to nothing.
 *
 * PRESSES STILL DO NOTHING
 * ------------------------
 * This turns bytes into a button number. What a button number DOES is a
 * separate question, and the only action a button is meant to drive — the
 * speed-profile step — is still locked behind fsd_sp_encoding_ok() and
 * FsdSpInputs.tx_armed, neither of which a capture has unlocked yet.
 */

#include <stdbool.h>
#include <stdint.h>

#include "fsd_button.h" // FSD_BTN_MAX — the logical button index space

#ifdef __cplusplus
extern "C" {
#endif

/* The logical buttons, numbered as the owner numbers them on the remote.
 *
 * 5번 IS DELIBERATELY ABSENT and so are 1번/2번's long presses — see
 * FSD_J6_C_AMBIG_* below. 6번 appears once because its short and long presses
 * are one LEVEL button that fsd_button.h times, not two codes.
 *
 * The value IS the index into FsdButtons, so these must not be renumbered
 * casually: 0 is spent on NONE so that a zeroed struct means "no button". */
typedef enum {
    FSD_J6_NONE = 0,
    FSD_J6_B1,      // 1번 톡 — 아래로 쓸기
    FSD_J6_B2,      // 2번 톡 — 위로 쓸기
    FSD_J6_B3,      // 3번 톡 — 오른쪽으로 쓸기
    FSD_J6_B4,      // 4번 톡 — 왼쪽으로 쓸기
    FSD_J6_B6,      // 6번 — 탭 (420,850). 🟢 유일한 레벨 버튼
    FSD_J6_B7,      // 7번 톡 — 탭 (300,500)
    FSD_J6_B3_LONG, // 3번 길게 — 00 08 00
    FSD_J6_B4_LONG, // 4번 길게 — 00 10 00
    FSD_J6_B7_LONG, // 7번 길게 — 00 40 00
    FSD_J6_COUNT,
} FsdJ6Btn;

/* 🔴 THE SPELLING HAS TO BE SWITCHED, and finding that out cost two builds.
 *
 * This header is compiled THREE ways and no single spelling satisfies all of
 * them:
 *
 *   host tests      C11        _Static_assert ok · static_assert ok (assert.h)
 *   ble_central.cpp C++        _Static_assert FAILS · static_assert is a keyword
 *   fsd_btn_j6.c    ESP32 C    _Static_assert ok · static_assert NOT defined,
 *                              because the Arduino build is below C11 so
 *                              <assert.h> does not declare the macro
 *
 * Reaching for <assert.h> fixes the C++ column and breaks the third. Neither
 * failure is visible from where the code was written: the host tests are C and
 * passed both times. */
#ifdef __cplusplus
#define FSD_J6_STATIC_ASSERT(c, m) static_assert(c, m)
#else
#define FSD_J6_STATIC_ASSERT(c, m) _Static_assert(c, m)
#endif

/* The index space is shared with fsd_button.h. Overflowing it would not crash —
 * fsd_btn_report() bounds-checks and returns NONE — so the top buttons would
 * simply stop producing events with nothing in the log. That is the failure
 * shape this project keeps getting caught by, so it is a build error.
 *
 * ASCII in the message: the toolchain mangles non-ASCII here, and this is read
 * at the moment something is already wrong. */
FSD_J6_STATIC_ASSERT(FSD_J6_COUNT <= FSD_BTN_MAX,
                     "FSD_J6_COUNT exceeds FSD_BTN_MAX (fsd_button.h) - the "
                     "highest buttons would silently stop reporting");

typedef enum {
    FSD_J6_EDGE_NONE = 0,
    FSD_J6_EDGE_PULSE, // 사건형: 제스처 하나가 통째로 끝났다
    FSD_J6_EDGE_DOWN,  // 레벨형: 눌리기 시작했다
    FSD_J6_EDGE_UP,    // 레벨형: 떼었다
} FsdJ6Edge;

typedef struct {
    FsdJ6Btn btn;
    FsdJ6Edge edge;
} FsdJ6Out;

/* Gesture in progress. One per bound remote. */
typedef struct {
    bool down;
    bool level;   // this contact is 6번's hold, not the start of a swipe
    uint16_t x0;  // where the contact started — a tap's identity
    uint16_t y0;
    uint32_t down_ms;
} FsdJ6;

/* Coordinates are normalised to a 1000x1000 surface. */
#define FSD_J6_COORD_MAX 1000u

/* Smallest travel that counts as a swipe. Measured swipes move 300-800; a tap
 * moves 0. Nothing observed lands between, so this is a gap, not a knife edge. */
#define FSD_J6_SWIPE_MIN 100u

/* Tap anchors — matched EXACTLY. Every capture of these was byte-identical, and
 * an unknown coordinate decoding to nothing is the safe direction: a tap we
 * cannot name must not become a button we can act on. */
#define FSD_J6_B6_X 420u
#define FSD_J6_B6_Y 850u
#define FSD_J6_B7_X 300u
#define FSD_J6_B7_Y 500u

/* Consumer control codes, as b[0]<<16 | b[1]<<8 | b[2]. */
#define FSD_J6_C_B3_LONG 0x000800u
#define FSD_J6_C_B4_LONG 0x001000u
#define FSD_J6_C_B7_LONG 0x004000u

/* 🔴 THESE TWO ARE REFUSED ON PURPOSE. DO NOT MAP THEM.
 *
 * ⭐ 5번 IS THE POWER BUTTON AND NOTHING ELSE (차주 결정, 2026-08-18). It is
 * not an input on this remote — it turns the thing on and off, and that is all
 * it is for. That is a DECISION, not a consequence of the collision below: if
 * some future firmware made the two codes tellable apart, 5번 would still not
 * be mapped. Do not read "we could not" as "we would have".
 *
 * The collision is the second reason, and it takes 1번/2번's long presses down
 * with it. Each code is emitted by TWO different gestures, byte for byte, so no
 * amount of decoding separates them:
 *
 *    00 80 00  =  1번 길게  =  5번 톡 (홀수 번째)
 *    00 00 01  =  2번 길게  =  5번 톡 (짝수 번째)
 *
 * Mapping either one means brushing the POWER button steps the speed profile —
 * and the hand reaches for that button every time the remote is switched on.
 *
 * 5번 alternates between the two codes on successive presses (7 in a row,
 * unbroken across sessions), which is why two measurements of it disagreed and
 * both were right. */
#define FSD_J6_C_AMBIG_A 0x008000u
#define FSD_J6_C_AMBIG_B 0x000001u

/* A contact this old is not a gesture any more — the release was lost.
 *
 * 🔴 Without this, a dropped release silently poisons the NEXT swipe: it would
 * be measured from the stale start point, and a swipe up can come out as a
 * swipe down. The user did the opposite of what the module acted on, and
 * nothing in the log says so.
 *
 * 6번's hold is exempt: a person may hold it for as long as they like, and a
 * genuinely wedged button is what fsd_button.h's STUCK is for. */
#define FSD_J6_GESTURE_MAX_MS 2000u

/** Clear to "no contact". */
void fsd_j6_init(FsdJ6* s);

/** Feed one notification's bytes. Returns the button and what happened to it.
 *
 *  Reports that are not this device's shape return NONE, as do the ambiguous
 *  consumer codes and taps at coordinates we have not measured. */
FsdJ6Out fsd_j6_feed(FsdJ6* s, const uint8_t* b, uint8_t len, uint32_t now_ms);

/** Which logical buttons must be registered as FSD_BTN_KIND_LEVEL.
 *
 *  Only 6번. Its hold tracks the hand — 0.12 / 1.97 / 5.43 s measured — so we
 *  time the long press ourselves. Every other button reports its press and
 *  release in the same instant, and registering one of those as LEVEL wedges
 *  it: LONG at 0.6 s, STUCK at 10 s, and STUCK clears only on a release it can
 *  never send. */
bool fsd_j6_is_level(FsdJ6Btn b);

/** ASCII name, for the serial log. Never NULL. */
const char* fsd_j6_name(FsdJ6Btn b);

#ifdef __cplusplus
}
#endif
