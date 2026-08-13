#include "ble_owner.h"

#ifdef BLE_SERVER_ENABLED

#include <Arduino.h>
#include <Preferences.h>

#include <string.h>

// Its own namespace, like power_log: prefs.cpp holds operator settings and is
// written when something changes, and mixing an unrelated writer into that blob
// makes both harder to reason about.
static const char* NS = "bleowner";
static const char* KEY = "own";

static Preferences g_nvs;

// The owner as it stands. Read from the BLE host task, written from both, so
// every touch goes through g_mux.
static FsdOwner       g_owner = {};
static portMUX_TYPE   g_mux   = portMUX_INITIALIZER_UNLOCKED;

// Set by the BLE host task, drained by loop(). See the note in the header:
// committing NVS from the BLE host task is the bug this split exists to avoid.
static volatile bool  g_save_pending = false;

static volatile uint32_t g_window_until_ms = 0;
static volatile bool     g_window_active   = false;

static void fmt_addr(char* out, size_t cap, uint8_t type, const uint8_t* a) {
    snprintf(out, cap, "%02X:%02X:%02X:%02X:%02X:%02X (type %u)",
             a[5], a[4], a[3], a[2], a[1], a[0], (unsigned)type);
}

void ble_owner_init(void) {
    FsdOwner loaded = {};

    if(g_nvs.begin(NS, /*readOnly=*/true)) {
        uint8_t blob[1 + FSD_OWNER_ADDR_LEN] = {};
        size_t n = g_nvs.getBytes(KEY, blob, sizeof(blob));
        if(n == sizeof(blob) && fsd_owner_addr_valid(blob[0], &blob[1])) {
            loaded.enrolled = true;
            loaded.type = blob[0];
            memcpy(loaded.addr, &blob[1], FSD_OWNER_ADDR_LEN);
        }
        g_nvs.end();
    }

    portENTER_CRITICAL(&g_mux);
    g_owner = loaded;
    portEXIT_CRITICAL(&g_mux);

    ble_owner_print();
}

void ble_owner_print(void) {
    FsdOwner o;
    portENTER_CRITICAL(&g_mux);
    o = g_owner;
    portEXIT_CRITICAL(&g_mux);

    if(!o.enrolled) {
        Serial.println("[OWNER] 등록된 폰 없음 — 처음 짝짓기하는 폰이 주인이 된다");
        return;
    }
    char buf[48];
    fmt_addr(buf, sizeof(buf), o.type, o.addr);
    Serial.printf("[OWNER] 등록된 폰: %s\n", buf);
}

void ble_owner_on_bond(uint8_t addr_type, const uint8_t* addr) {
    FsdOwner snapshot;
    portENTER_CRITICAL(&g_mux);
    snapshot = g_owner;
    portEXIT_CRITICAL(&g_mux);

    bool window = ble_owner_window_open();
    FsdOwnerVerdict v = fsd_owner_check(&snapshot, addr_type, addr, window);

    char buf[48];
    fmt_addr(buf, sizeof(buf), addr_type, addr);
    Serial.printf("[OWNER] %s — %s\n", buf, fsd_owner_verdict_str(v));

    if(v != FSD_OWNER_ENROLL) return;

    FsdOwner next = {};
    next.enrolled = true;
    next.type = addr_type;
    memcpy(next.addr, addr, FSD_OWNER_ADDR_LEN);

    portENTER_CRITICAL(&g_mux);
    g_owner = next;
    portEXIT_CRITICAL(&g_mux);

    // Close the window immediately. Leaving it open for the rest of its two
    // minutes would let a second phone enrol right behind the one the operator
    // meant to add, off the back of a single button press.
    g_window_active = false;

    g_save_pending = true;   // loop() writes it; see the header
}

bool ble_owner_allows(uint8_t addr_type, const uint8_t* addr) {
    FsdOwner o;
    portENTER_CRITICAL(&g_mux);
    o = g_owner;
    portEXIT_CRITICAL(&g_mux);

    // Nobody enrolled yet: this is the trust-on-first-use case and the bond
    // callback is about to claim this peer. Refusing here would make the very
    // first command after the very first pairing fail for no visible reason.
    if(!o.enrolled) return true;

    return fsd_owner_same(&o, addr_type, addr);
}

void ble_owner_open_window(uint32_t now_ms) {
    g_window_until_ms = now_ms + BLE_OWNER_WINDOW_MS;
    g_window_active   = true;
    Serial.printf("[OWNER] 등록 창을 %u 초 동안 연다 — 지금 짝짓기하는 폰이 새 주인이 된다\n",
                  (unsigned)(BLE_OWNER_WINDOW_MS / 1000u));
}

bool ble_owner_window_open(void) { return g_window_active; }

void ble_owner_forget(void) {
    portENTER_CRITICAL(&g_mux);
    g_owner = FsdOwner{};
    portEXIT_CRITICAL(&g_mux);
    g_save_pending = true;
    Serial.println("[OWNER] 등록을 지웠다 — 다음에 짝짓기하는 폰이 주인이 된다");
}

void ble_owner_tick(uint32_t now_ms) {
    if(g_window_active && (int32_t)(now_ms - g_window_until_ms) >= 0) {
        g_window_active = false;
        Serial.println("[OWNER] 등록 창이 닫혔다");
    }

    if(!g_save_pending) return;
    g_save_pending = false;

    FsdOwner o;
    portENTER_CRITICAL(&g_mux);
    o = g_owner;
    portEXIT_CRITICAL(&g_mux);

    if(!g_nvs.begin(NS, /*readOnly=*/false)) {
        Serial.println("[OWNER] NVS 를 못 열었다 — 이 등록은 재부팅하면 사라진다");
        return;
    }
    if(o.enrolled) {
        uint8_t blob[1 + FSD_OWNER_ADDR_LEN];
        blob[0] = o.type;
        memcpy(&blob[1], o.addr, FSD_OWNER_ADDR_LEN);
        g_nvs.putBytes(KEY, blob, sizeof(blob));
    } else {
        g_nvs.remove(KEY);
    }
    g_nvs.end();
}

#else  // BLE disabled — nothing to own

void ble_owner_init(void) {}
void ble_owner_tick(uint32_t) {}
void ble_owner_on_bond(uint8_t, const uint8_t*) {}
bool ble_owner_allows(uint8_t, const uint8_t*) { return true; }
void ble_owner_open_window(uint32_t) {}
bool ble_owner_window_open(void) { return false; }
void ble_owner_forget(void) {}
void ble_owner_print(void) {}

#endif  // BLE_SERVER_ENABLED
