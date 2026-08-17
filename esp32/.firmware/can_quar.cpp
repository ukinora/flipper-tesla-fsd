#include "can_quar.h"

#include "../../fsd_logic/fsd_canquar.h"
#include <Preferences.h>
#include <esp_system.h>

static const char *NS = "canquar";

static Preferences  g_nvs;
static FsdCanQuar   g_q      = {};
static uint8_t      g_trying = 0;      // buses enabled this boot, not yet proven
static bool         g_proven = false;  // the mark has been cleared this boot
static uint8_t      g_count  = 0;

/* How long the module must run before a bus is considered vouched for.
 *
 * The observed failure killed the board 0.6 s after the controller came up, so
 * anything comfortably past that is enough to separate "died on bring-up" from
 * "died later, for some other reason". Ten seconds also means a normal session
 * clears the mark almost immediately, so an unrelated panic minutes later is
 * not blamed on a bus. */
#define CAN_QUAR_PROVE_MS 10000u

/* 🔴 Panic only — NOT every software reset.
 *
 * power_log.cpp deliberately lumps ESP_RST_SW in with the panics because for
 * its purpose ("did the power go away?") they are the same. Here they are
 * opposites: ESP_RST_SW is what esp_restart() produces, which is how a factory
 * reset reboots. Treating that as a panic would quarantine a bus every time the
 * operator resets the module. */
static bool was_panic_reset() {
    switch (esp_reset_reason()) {
        case ESP_RST_PANIC:
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
            return true;
        default:
            return false;
    }
}

static void save() {
    if (!g_nvs.begin(NS, /*readOnly=*/false)) return;
    g_nvs.putUChar("quar", g_q.quarantined);
    g_nvs.putUChar("panics", g_q.panics);
    g_nvs.putUChar("trying", g_trying);
    g_nvs.end();
}

uint8_t can_quar_boot(uint8_t bus_count) {
    g_count = bus_count;

    uint8_t prev_trying = 0;
    if (g_nvs.begin(NS, /*readOnly=*/true)) {
        g_q.quarantined = g_nvs.getUChar("quar", 0);
        g_q.panics      = g_nvs.getUChar("panics", 0);
        prev_trying     = g_nvs.getUChar("trying", 0);
        g_nvs.end();
    }

    bool panic = was_panic_reset();
    FsdCanQuarResult r = fsd_canquar_boot(&g_q, panic, prev_trying, bus_count);

    if (r.newly) {
        // Loud, and in the operator's language: this is the module explaining
        // why it is about to run on fewer buses than it has.
        for (uint8_t i = 0; i < bus_count; i++) {
            if (!(r.newly & (1u << i))) continue;
            Serial.printf("[CAN] 🔴 can%u 를 격리했다 — 이 버스를 켜면 보드가 "
                          "패닉으로 재부팅한다 (연속 %u회 확인)\n",
                          (unsigned)i, (unsigned)FSD_CANQUAR_PANICS);
            Serial.println("[CAN]    배선을 확인한다: H/L 반대, 종단, 비트레이트.");
            Serial.println("[CAN]    나머지 버스로 계속 동작한다. 다시 켜려면 'canclear'.");
        }
    } else if (g_q.quarantined) {
        Serial.printf("[CAN] 격리 중: 0x%02X — 'canclear' 로 해제한다\n",
                      (unsigned)g_q.quarantined);
    } else if (panic && prev_trying) {
        Serial.printf("[CAN] 지난 부팅이 패닉이었다 (미검증 버스 0x%02X) — "
                      "%u회 연속이면 격리한다 (지금 %u회)\n",
                      (unsigned)prev_trying, (unsigned)FSD_CANQUAR_PANICS,
                      (unsigned)g_q.panics);
    }

    // A fresh boot starts with nothing on trial; mark_trying() adds them back.
    g_trying = 0;
    g_proven = false;
    save();
    return g_q.quarantined;
}

bool can_quar_blocked(uint8_t index) {
    if (index >= 8u) return false;
    return (g_q.quarantined >> index) & 1u;
}

void can_quar_mark_trying(uint8_t index) {
    if (index >= 8u) return;
    uint8_t bit = (uint8_t)(1u << index);
    if (g_trying & bit) return;
    g_trying |= bit;
    // Written before the bus is enabled, on purpose: if enabling it panics the
    // board, this is the only thing that will still be true afterwards.
    save();
}

void can_quar_prove(uint32_t now_ms) {
    if (g_proven || g_trying == 0u) return;
    if (now_ms < CAN_QUAR_PROVE_MS) return;
    g_proven = true;
    g_trying = 0;
    save();
    Serial.printf("[CAN] 버스가 %u초를 무사히 넘겼다 — 격리 후보에서 뺀다\n",
                  (unsigned)(CAN_QUAR_PROVE_MS / 1000u));
}

void can_quar_clear() {
    fsd_canquar_clear(&g_q);
    g_trying = 0;
    g_proven = true;   // nothing left on trial this boot
    save();
    Serial.println("[CAN] 격리를 전부 해제했다 — 다음 부팅에서 모든 버스를 켠다");
}

void can_quar_print() {
    Serial.printf("[CAN] 격리=0x%02X 연속패닉=%u 미검증=0x%02X%s\n",
                  (unsigned)g_q.quarantined, (unsigned)g_q.panics,
                  (unsigned)g_trying, g_proven ? " (검증됨)" : "");
    for (uint8_t i = 0; i < g_count; i++)
        if (can_quar_blocked(i))
            Serial.printf("[CAN]   can%u 는 켜지 않는다 ('canclear' 로 해제)\n",
                          (unsigned)i);
}
