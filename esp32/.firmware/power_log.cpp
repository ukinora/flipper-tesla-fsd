#include "power_log.h"

#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>

// Its own namespace. prefs.cpp owns operator settings and is written once when
// something changes; this is written every minute, and mixing the two would put
// a periodic writer into the same blob as the settings.
static const char* NS = "pwrlog";

static Preferences g_nvs;
static bool g_ok = false;

static FsdPwrVerdict g_boot_verdict = FSD_PWR_UNKNOWN;
static FsdPwrVerdict g_live_verdict = FSD_PWR_UNKNOWN;
static uint32_t g_last_save_ms = 0;

/* Normal cadence, and the faster one used once the bus falls silent.
 *
 * The fast one matters: if the feed is switched, the window between "bus went
 * quiet" and "we lost power" may be short. Writing only once a minute could
 * leave a final record that still says the bus was busy, and the next boot
 * would then report NO_SLEEP -- the wrong answer, arrived at confidently. */
static const uint32_t SAVE_BUSY_MS = 60000;
static const uint32_t SAVE_QUIET_MS = 10000;

static FsdPwrReset map_reset(esp_reset_reason_t r) {
    switch(r) {
    case ESP_RST_POWERON: return FSD_PWR_RESET_POWERON;
    case ESP_RST_BROWNOUT: return FSD_PWR_RESET_BROWNOUT;
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_SW: return FSD_PWR_RESET_SOFTWARE;
    case ESP_RST_EXT: return FSD_PWR_RESET_EXTERNAL;
    default: return FSD_PWR_RESET_UNKNOWN;
    }
}

static void print_minutes(const char* label, uint32_t ms) {
    Serial.printf("%s %u분 %u초", label, (unsigned)(ms / 60000u),
                  (unsigned)((ms % 60000u) / 1000u));
}

void power_log_init(void) {
    FsdPwrReset reset = map_reset(esp_reset_reason());

    FsdPwrRecord prev;
    prev.valid = false;
    prev.uptime_ms = 0;
    prev.quiet_ms = 0;

    if(g_nvs.begin(NS, /*readOnly=*/true)) {
        if(g_nvs.isKey("up")) {
            prev.valid = true;
            prev.uptime_ms = g_nvs.getULong("up", 0);
            prev.quiet_ms = g_nvs.getULong("quiet", 0);
        }
        g_nvs.end();
    }

    g_boot_verdict = fsd_pwr_verdict(reset, &prev);

    Serial.printf("[PWR] 재시작 사유: %s\n", fsd_pwr_reset_str(reset));
    if(prev.valid) {
        Serial.print("[PWR] 지난번: ");
        print_minutes("켜져 있던 시간", prev.uptime_ms);
        if(prev.quiet_ms == 0) {
            Serial.print(" · CAN 은 끝까지 흐르고 있었다");
        } else {
            Serial.print(" · ");
            print_minutes("CAN 이 조용했던 시간", prev.quiet_ms);
        }
        Serial.println();
    } else {
        Serial.println("[PWR] 지난번 기록 없음 (첫 부팅이거나 NVS 가 비었다)");
    }
    Serial.printf("[PWR] → %s\n", fsd_pwr_verdict_str(g_boot_verdict));

    /* Clear the record now that it has been reported. Without this a session
     * that is cut before its first write would report the session BEFORE it,
     * and the operator would read a stale verdict as a fresh one. */
    if(g_nvs.begin(NS, /*readOnly=*/false)) {
        g_nvs.putULong("up", 0);
        g_nvs.putULong("quiet", 0);
        g_nvs.end();
        g_ok = true;
    } else {
        Serial.println("[PWR] NVS 를 못 열었다 — 이번 세션은 기록되지 않는다");
    }
}

void power_log_tick(uint32_t now_ms, uint32_t last_rx_ms, bool seen_any) {
    if(!g_ok) return;

    uint32_t quiet = fsd_pwr_quiet_ms(now_ms, last_rx_ms, seen_any);

    FsdPwrVerdict live = fsd_pwr_live_verdict(quiet);
    if(live != g_live_verdict) {
        g_live_verdict = live;
        if(live != FSD_PWR_UNKNOWN) {
            Serial.print("[PWR] ");
            print_minutes("CAN 조용해진 지", quiet);
            Serial.printf(" · 아직 살아 있다 → %s\n", fsd_pwr_verdict_str(live));
        }
    }

    uint32_t due = (quiet > 0) ? SAVE_QUIET_MS : SAVE_BUSY_MS;
    if((uint32_t)(now_ms - g_last_save_ms) < due) return;
    g_last_save_ms = now_ms;

    if(!g_nvs.begin(NS, /*readOnly=*/false)) return;
    g_nvs.putULong("up", now_ms);
    g_nvs.putULong("quiet", quiet);
    g_nvs.end();
}

FsdPwrVerdict power_log_verdict(void) {
    /* What this session can prove outranks what the last one suggested: a live
     * ALWAYS-ON is an observation happening now, while the boot verdict is a
     * reading of something already over. */
    return (g_live_verdict != FSD_PWR_UNKNOWN) ? g_live_verdict : g_boot_verdict;
}
