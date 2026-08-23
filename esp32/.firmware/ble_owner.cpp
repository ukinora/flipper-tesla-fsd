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
    //
    // 🔴 But only for a peer we can actually identify. ble_owner_on_bond()
    // refuses to enrol an unresolved identity (all-zero, see fsd_owner.c) --
    // correctly -- and that left the module permanently unenrolled, which this
    // branch then read as "allow everyone". The two halves disagreed, and the
    // half that decided was the permissive one.
    if(!o.enrolled) return fsd_owner_addr_valid(addr_type, addr);

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

void ble_owner_erase_now(void) {
    ble_owner_forget();
    /* Drain the queue here rather than waiting for a loop() that is not coming:
     * the caller is a factory reset and reboots ~200 ms later.
     *
     * 🔴 Retry, and say so if it never lands. ble_owner_tick() now refuses to
     * clear the pending flag on a failed write, which is right -- but with a
     * reboot already scheduled there is no later loop() to retry in, so a single
     * attempt would turn "erased" into a claim rather than a fact. Failing to
     * erase is at least the safe direction (the OLD owner stays enrolled;
     * nobody new is adopted), so this reports rather than blocks. */
    for(int i = 0; i < 3 && ble_owner_save_pending(); i++) ble_owner_tick(0);
    if(ble_owner_save_pending())
        Serial.println("[OWNER] 🔴 지우지 못했다 — 이전 주인이 그대로 남는다");
}

bool ble_owner_save_pending(void) { return g_save_pending; }

void ble_owner_tick(uint32_t now_ms) {
    if(g_window_active && (int32_t)(now_ms - g_window_until_ms) >= 0) {
        g_window_active = false;
        Serial.println("[OWNER] 등록 창이 닫혔다");
    }

    if(!g_save_pending) return;

    FsdOwner o;
    portENTER_CRITICAL(&g_mux);
    o = g_owner;
    portEXIT_CRITICAL(&g_mux);

    // 🔴 The flag is cleared only after the write lands. Clearing it first
    // meant a failed begin()/put() lost the enrolment silently: the module kept
    // running as if it had an owner and came back after a reboot in
    // trust-on-first-use, ready to adopt whoever paired next.
    if(!g_nvs.begin(NS, /*readOnly=*/false)) {
        Serial.println("[OWNER] NVS 를 못 열었다 — 다음 루프에서 다시 시도한다");
        return;
    }
    /* 🔴 And the write has to have LANDED.
     *
     * putBytes() and remove() both report whether they worked, and both return
     * values were being thrown away -- so a full or worn NVS produced exactly
     * the failure the comment above says it prevents. The flag was cleared
     * unconditionally two lines later, the module kept running with an owner in
     * RAM, and the next boot came up with none: trust-on-first-use, open to
     * whoever pairs next. That gap is invisible until the reboot, which is the
     * worst possible place for it to appear.
     *
     * Read the record back rather than trusting the byte count. A short write
     * is not the only way to end up with the wrong bytes on flash, and the cost
     * of checking is one read of seven bytes, once per enrolment. */
    bool stored = false;
    if(o.enrolled) {
        uint8_t blob[1 + FSD_OWNER_ADDR_LEN];
        blob[0] = o.type;
        memcpy(&blob[1], o.addr, FSD_OWNER_ADDR_LEN);
        if(g_nvs.putBytes(KEY, blob, sizeof(blob)) == sizeof(blob)) {
            uint8_t back[1 + FSD_OWNER_ADDR_LEN] = {};
            stored = g_nvs.getBytes(KEY, back, sizeof(back)) == sizeof(back) &&
                     memcmp(back, blob, sizeof(blob)) == 0;
        }
    } else {
        /* Absent is the goal, however we get there: remove() reports false when
         * the key was not there to begin with, which is success for us. */
        g_nvs.remove(KEY);
        stored = !g_nvs.isKey(KEY);
    }
    g_nvs.end();

    if(!stored) {
        /* Keep g_save_pending set: loop() comes back and tries again. Rate-limit
         * the log so a permanently broken NVS does not bury everything else --
         * but never go silent, because the failure has no other symptom until
         * the module reboots without an owner. */
        static uint32_t s_warn_ms = 0;
        if(!s_warn_ms || (uint32_t)(now_ms - s_warn_ms) >= 5000u) {
            s_warn_ms = now_ms ? now_ms : 1u;
            Serial.println("[OWNER] 🔴 NVS 저장 실패 — 계속 재시도한다. "
                           "지금 재부팅하면 주인 등록이 사라진다");
        }
        return;
    }
    g_save_pending = false;
}

#else  // BLE disabled — nothing to own

void ble_owner_init(void) {}
void ble_owner_tick(uint32_t) {}
void ble_owner_on_bond(uint8_t, const uint8_t*) {}
bool ble_owner_allows(uint8_t, const uint8_t*) { return true; }
void ble_owner_open_window(uint32_t) {}
bool ble_owner_window_open(void) { return false; }
void ble_owner_forget(void) {}
void ble_owner_erase_now(void) {}
bool ble_owner_save_pending(void) { return false; }
void ble_owner_print(void) {}

#endif  // BLE_SERVER_ENABLED
