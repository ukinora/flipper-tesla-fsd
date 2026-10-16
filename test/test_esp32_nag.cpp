/* 🔴 «NAG 킬러를 컴파일에서 뺐다» 를 **보드 사본**에서 지킨다 (레드팀 2026-10-16, N-1).
 *
 * FSD_NAG_KILLER_ENABLED=0 은 0x370 에코 하나만 막고 있었다. 0x3FD mux1 을
 * 고쳐 다시 보내는 경로(bit19 지우기 · HW4 는 bit47 세우기)는 state 의
 * nag_killer 하나만 봤는데, setup() 이 prefs_load() 앞에서 그 값을 true 로
 * 덮고 prefs_load() 는 저장된 설정이 없으면(첫 부팅·공장 초기화 뒤) 바로
 * 돌아갔다 — 그러면 Active 로 올라온 보드가 고친 0x3FD 를 차에 보냈다.
 * BOOT 3초 누름도 같은 깃발을 켰다.
 *
 * 그래서 이 시험은 **깃발이 켜져 있어도** 아무것도 안 나가는지를 본다.
 * 깃발을 끄는 자리(setup · 버튼)를 아무리 고쳐도, 다음 사람이 깃발을 켜는
 * 새 길을 하나 더 만들면 같은 일이 난다 — 막는 것은 프레임을 고치는 쪽이어야 한다.
 *
 * 🔴 fsd_logic/fsd_handler.c 가 아니라 esp32/.firmware/fsd_handler.cpp 를
 * 컴파일한다. 사본이 둘이고 호스트 시험은 그동안 한쪽만 돌았다(열 번째 패턴).
 * 이 빌드는 보드와 같은 스위치(-DFSD_NAG_KILLER_ENABLED=0)로 굽는다. */
#include "fsd_handler.h"
#include "can_signals.h"
#include <stdio.h>
#include <string.h>

static int g_fail = 0, g_pass = 0;
#define CHECK(c, ...) do { if (c) { g_pass++; } else { g_fail++; \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static bool bit(const CanFrame &f, int b) { return (f.data[b / 8] >> (b % 8)) & 1u; }

/* 0x3FD mux1, bit19 서 있음(차가 «손을 올려라» 를 요청하는 상태). */
static CanFrame ap_mux1(void) {
    CanFrame f;
    memset(&f, 0, sizeof f);
    f.id = CAN_ID_AP_CONTROL;
    f.dlc = 8;
    f.data[0] = 0x01;                                   // mux 1
    f.data[SIG_AP_NAG_CLEAR_BIT / 8] |= (uint8_t)(1u << (SIG_AP_NAG_CLEAR_BIT % 8));
    return f;
}

static void flag_forced_on_does_not_touch_0x3fd(TeslaHWVersion hw, const char *name) {
    FSDState s;
    fsd_state_init(&s, hw);
    CHECK(!s.nag_killer, "%s: init leaves the compiled-out flag off", name);

    s.nag_killer = true;            // what setup() and the BOOT long press used to do
    CanFrame f = ap_mux1();
    CanFrame before = f;
    bool modified = fsd_handle_autopilot_frame(&s, &f);

    CHECK(!modified, "%s: 0x3FD mux1 not modified with nag compiled out", name);
    CHECK(bit(f, SIG_AP_NAG_CLEAR_BIT), "%s: bit19 left as the car sent it", name);
    CHECK(memcmp(f.data, before.data, 8) == 0, "%s: frame bytes unchanged", name);
    CHECK(!s.nag_suppressed, "%s: no nag_suppressed claim", name);
}

static void legacy_path_too(void) {
    FSDState s;
    fsd_state_init(&s, TeslaHW_Legacy);
    s.nag_killer = true;
    CanFrame f = ap_mux1();
    bool modified = fsd_handle_legacy_autopilot(&s, &f);
    CHECK(!modified, "legacy: mux1 not modified with nag compiled out");
    CHECK(bit(f, SIG_AP_NAG_CLEAR_BIT), "legacy: bit19 left as the car sent it");
}

static void echo_still_refused(void) {
    FSDState s;
    fsd_state_init(&s, TeslaHW_HW3);
    s.nag_killer = true;
    CanFrame in, out;
    memset(&in, 0, sizeof in);
    in.id = CAN_ID_EPAS_STATUS;
    in.dlc = 8;
    CHECK(!fsd_handle_nag_killer(&s, &in, &out, 1000u), "0x370 echo refused");
}

int main(void) {
#if FSD_NAG_KILLER_ENABLED
#error "build this test with -DFSD_NAG_KILLER_ENABLED=0 — that is the board's setting"
#endif
    flag_forced_on_does_not_touch_0x3fd(TeslaHW_HW3, "HW3");
    flag_forced_on_does_not_touch_0x3fd(TeslaHW_HW4, "HW4");
    legacy_path_too();
    echo_still_refused();
    printf("test_esp32_nag: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
