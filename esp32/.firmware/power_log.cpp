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

/* Kept so power_log_print() can re-report the boot picture at any time. The
 * banner is printed once and, on this board, is unreadable over USB after a
 * physical power cycle -- see power_log.h. */
static FsdPwrReset g_boot_reset = FSD_PWR_RESET_UNKNOWN;
static FsdPwrRecord g_prev = {false, 0, 0, false};

/* Where this session stands right now, refreshed every tick. */
static uint32_t g_now_ms = 0;
static uint32_t g_quiet_ms = 0;
static bool g_seen_any = false;

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

/* The boot picture. Separate from init() so `pwr` can print it again later. */
static void report_boot(void) {
    Serial.printf("[PWR] 재시작 사유: %s\n", fsd_pwr_reset_str(g_boot_reset));
    if(g_prev.valid) {
        Serial.print("[PWR] 지난번: ");
        print_minutes("켜져 있던 시간", g_prev.uptime_ms);
        if(!g_prev.seen_any) {
            /* 🔴 Not the same sentence as "the bus was busy". A session that
             * never heard a frame observed the WIRING, not the car, and saying
             * anything about the car from it is how an H/L swap gets reported
             * as "the car never slept". */
            Serial.print(" · CAN 을 한 번도 못 들었다");
        } else if(g_prev.quiet_ms == 0) {
            Serial.print(" · CAN 은 끝까지 흐르고 있었다");
        } else {
            Serial.print(" · ");
            print_minutes("CAN 이 조용했던 시간", g_prev.quiet_ms);
        }
        Serial.println();
    } else {
        Serial.println("[PWR] 지난번 기록 없음 (첫 부팅이거나 NVS 가 비었다)");
    }
    Serial.printf("[PWR] → %s\n", fsd_pwr_verdict_str(g_boot_verdict));
}

void power_log_init(void) {
    g_boot_reset = map_reset(esp_reset_reason());

    g_prev.valid = false;
    g_prev.uptime_ms = 0;
    g_prev.quiet_ms = 0;
    g_prev.seen_any = false;

    if(g_nvs.begin(NS, /*readOnly=*/true)) {
        if(g_nvs.isKey("up")) {
            g_prev.valid = true;
            g_prev.uptime_ms = g_nvs.getULong("up", 0);
            g_prev.quiet_ms = g_nvs.getULong("quiet", 0);
            g_prev.seen_any = g_nvs.getBool("seen", false);
        }
        g_nvs.end();
    }

    g_boot_verdict = fsd_pwr_verdict(g_boot_reset, &g_prev);
    report_boot();

    /* Clear the record now that it has been reported. Without this a session
     * that is cut before its first write would report the session BEFORE it,
     * and the operator would read a stale verdict as a fresh one.
     *
     * 🔴 remove(), not putULong(0). Writing zeroes CREATES the key, and the
     * read above decides validity by whether the key EXISTS -- so after the
     * very first boot every later boot found a record that said "up 0 minutes,
     * CAN flowing to the end". That is not "no record", it is a fabricated one,
     * and fsd_pwr_verdict() turned it into NO_SLEEP: the honest answer (UNKNOWN)
     * replaced by a claim about a car we never observed. Seen on the bench
     * 2026-08-17, on a board whose NVS had just reported "first boot". */
    if(g_nvs.begin(NS, /*readOnly=*/false)) {
        /* Only if there was something to clear. Erasing an absent key is not an
         * error but ESP-IDF logs it as one, three lines every clean boot, and a
         * log that cries wolf is a log nobody reads. */
        if(g_prev.valid) {
            g_nvs.remove("up");
            g_nvs.remove("quiet");
            g_nvs.remove("seen");
        }
        g_nvs.end();
        g_ok = true;
    } else {
        Serial.println("[PWR] NVS 를 못 열었다 — 이번 세션은 기록되지 않는다");
    }
}

void power_log_print(void) {
    report_boot();

    Serial.print("[PWR] 이번 세션: ");
    print_minutes("켜져 있던 시간", g_now_ms);
    if(!g_seen_any) {
        Serial.print(" · CAN 을 아직 한 번도 못 들었다");
    } else if(g_quiet_ms == 0) {
        Serial.print(" · CAN 이 흐르는 중");
    } else {
        Serial.print(" · ");
        print_minutes("CAN 이 조용해진 지", g_quiet_ms);
    }
    Serial.println();

    Serial.printf("[PWR] 지금 판정: %s\n", fsd_pwr_verdict_str(power_log_verdict()));
    if(!g_ok) Serial.println("[PWR] ⚠️ NVS 를 못 열었다 — 이번 세션은 기록되지 않는다");
}

void power_log_tick(uint32_t now_ms, uint32_t last_rx_ms, bool seen_any) {
    uint32_t quiet = fsd_pwr_quiet_ms(now_ms, last_rx_ms, seen_any);

    /* Refreshed even when NVS is unavailable: `pwr` should still be able to
     * describe this session, and "we could not record it" is its own line. */
    g_now_ms = now_ms;
    g_quiet_ms = quiet;
    g_seen_any = seen_any;

    if(!g_ok) return;

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
    /* 🔴 Without this, quiet==0 is ambiguous on the next boot: "the bus was
     * busy to the end" and "we never heard the bus" are the same number. */
    g_nvs.putBool("seen", seen_any);
    g_nvs.end();
}

FsdPwrVerdict power_log_verdict(void) {
    /* What this session can prove outranks what the last one suggested: a live
     * ALWAYS-ON is an observation happening now, while the boot verdict is a
     * reading of something already over. */
    return (g_live_verdict != FSD_PWR_UNKNOWN) ? g_live_verdict : g_boot_verdict;
}
