#include "ble_owner.h"

#ifdef BLE_SERVER_ENABLED

#include <Arduino.h>
#include <Preferences.h>
#include <mbedtls/sha256.h>
#include "nimble/nimble/host/include/host/ble_hs.h"  // ble_store_read_peer_sec

#include <string.h>

// Its own namespace, like power_log: prefs.cpp holds operator settings and is
// written when something changes, and mixing an unrelated writer into that blob
// makes both harder to reason about.
static const char* NS = "bleowner";
static const char* KEY = "own";
/* 🔴 The keys live under their OWN key, and "own" stays the 7-byte record it
 * has always been. Growing "own" to 16 bytes made older firmware read 0
 * (Preferences::getBytes refuses a buffer smaller than the value) -- so an
 * OTA rollback or a USB flash of an older build came up with NO owner and
 * enrolled whoever paired next (review of PR #143). */
static const char* KEY_FP = "ownk";

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

/* NVS: "own" = type + address (7 B, unchanged since it was introduced).
 * "ownk" = the bond fingerprint, present only once the keys are recorded. A
 * board with "own" and no "ownk" is a key-less owner that records its keys on
 * the next reconnect (FSD_OWNER_ADOPT_KEY). */
#define OWNER_BLOB_LEN (1u + FSD_OWNER_ADDR_LEN)

/* 🔴 The fingerprint of the bond NimBLE holds for this identity (red team
 * 2026-10-16, N-3). The address is what the peer claimed; the keys are what
 * the owner's phone and this board made together, and a fresh pairing makes
 * different ones. Hash rather than copy: our record should not be a second
 * place the LTK lives.
 *
 * False when there is no bond to read -- the caller treats that as "not the
 * owner's keys", never as a match. Called from the BLE host task; the store
 * NimBLE reads here is the one it wrote before raising the encryption event
 * (ble_sm.c persists keys, then calls ble_gap_enc_event). */
static bool bond_fp(uint8_t addr_type, const uint8_t* addr,
                    uint8_t out[FSD_OWNER_KEY_FP_LEN]) {
    struct ble_store_key_sec key;
    memset(&key, 0, sizeof key);
    key.peer_addr.type = addr_type;
    memcpy(key.peer_addr.val, addr, FSD_OWNER_ADDR_LEN);
    key.idx = 0;

    struct ble_store_value_sec v;
    memset(&v, 0, sizeof v);
    if(ble_store_read_peer_sec(&key, &v) != 0) return false;
    if(!v.ltk_present && !v.irk_present) return false;

    uint8_t in[2 * 16 + 2];
    memcpy(in, v.ltk, 16);
    memcpy(in + 16, v.irk, 16);
    in[32] = (uint8_t)v.ltk_present;
    in[33] = (uint8_t)v.irk_present;
    uint8_t digest[32];
    /* void in mbedtls 2, int in 3 -- neither can fail on a stack buffer. */
    (void)mbedtls_sha256(in, sizeof in, digest, 0);
    memcpy(out, digest, FSD_OWNER_KEY_FP_LEN);
    return true;
}

static void fmt_addr(char* out, size_t cap, uint8_t type, const uint8_t* a) {
    snprintf(out, cap, "%02X:%02X:%02X:%02X:%02X:%02X (type %u)",
             a[5], a[4], a[3], a[2], a[1], a[0], (unsigned)type);
}

void ble_owner_init(void) {
    FsdOwner loaded = {};

    if(g_nvs.begin(NS, /*readOnly=*/true)) {
        uint8_t blob[OWNER_BLOB_LEN] = {};
        size_t n = g_nvs.getBytes(KEY, blob, sizeof(blob));
        if(n == sizeof(blob) && fsd_owner_addr_valid(blob[0], &blob[1])) {
            loaded.enrolled = true;
            loaded.type = blob[0];
            memcpy(loaded.addr, &blob[1], FSD_OWNER_ADDR_LEN);
            uint8_t fp[FSD_OWNER_KEY_FP_LEN] = {};
            if(g_nvs.getBytes(KEY_FP, fp, sizeof(fp)) == sizeof(fp)) {
                loaded.keyed = true;
                memcpy(loaded.key_fp, fp, sizeof(fp));
            }
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
    Serial.printf("[OWNER] 등록된 폰: %s · 짝 키 %s\n", buf,
                  o.keyed ? "기록됨" : "아직 없음 (다음 연결 때 기록한다)");
}

FsdOwnerVerdict ble_owner_on_bond(uint8_t addr_type, const uint8_t* addr) {
    FsdOwner snapshot;
    portENTER_CRITICAL(&g_mux);
    snapshot = g_owner;
    portEXIT_CRITICAL(&g_mux);

    uint8_t fp[FSD_OWNER_KEY_FP_LEN];
    const bool have_fp = fsd_owner_addr_valid(addr_type, addr) && bond_fp(addr_type, addr, fp);

    bool window = ble_owner_window_open();
    FsdOwnerVerdict v = fsd_owner_check_bond(&snapshot, addr_type, addr,
                                             have_fp ? fp : NULL, window);

    char buf[48];
    fmt_addr(buf, sizeof(buf), addr_type, addr);
    Serial.printf("[OWNER] %s — %s\n", buf, fsd_owner_verdict_str(v));

    if(v == FSD_OWNER_ADOPT_KEY) {
        /* Same owner, same enrolment -- only the keys are new to our record. The
         * window is not touched: nothing was enrolled. */
        FsdOwner next = snapshot;
        next.keyed = true;
        memcpy(next.key_fp, fp, FSD_OWNER_KEY_FP_LEN);
        portENTER_CRITICAL(&g_mux);
        g_owner = next;
        portEXIT_CRITICAL(&g_mux);
        g_save_pending = true;
        return v;
    }

    if(v != FSD_OWNER_ENROLL) return v;

    FsdOwner next = {};
    next.enrolled = true;
    next.type = addr_type;
    memcpy(next.addr, addr, FSD_OWNER_ADDR_LEN);
    /* Without readable keys we enrol key-less, as before, and adopt them on the
     * next reconnect -- refusing would leave a fresh board with no owner. */
    next.keyed = have_fp;
    if(have_fp) memcpy(next.key_fp, fp, FSD_OWNER_KEY_FP_LEN);

    portENTER_CRITICAL(&g_mux);
    g_owner = next;
    portEXIT_CRITICAL(&g_mux);

    // Close the window immediately. Leaving it open for the rest of its two
    // minutes would let a second phone enrol right behind the one the operator
    // meant to add, off the back of a single button press.
    g_window_active = false;

    g_save_pending = true;   // loop() writes it; see the header
    return v;
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
    if(!fsd_owner_same(&o, addr_type, addr)) return false;

    /* 🔴 The address alone is a claim (red team 2026-10-16, N-3). Read the
     * bond in use and ask whether it is the one the owner enrolled with. Per
     * write, like the address check above, so a re-pair in the middle of a
     * session cannot ride on an earlier answer. */
    uint8_t fp[FSD_OWNER_KEY_FP_LEN];
    const bool have_fp = bond_fp(addr_type, addr, fp);
    return fsd_owner_allows(&o, addr_type, addr, have_fp ? fp : NULL);
}

void ble_owner_open_window(uint32_t now_ms) {
    g_window_until_ms = now_ms + BLE_OWNER_WINDOW_MS;
    g_window_active   = true;
    Serial.printf("[OWNER] 등록 창을 %u 초 동안 연다 — 지금 짝짓기하는 폰이 새 주인이 된다\n",
                  (unsigned)(BLE_OWNER_WINDOW_MS / 1000u));
}

bool ble_owner_window_open(void) { return g_window_active; }

bool ble_owner_enrolled(void) {
    portENTER_CRITICAL(&g_mux);
    const bool e = g_owner.enrolled;
    portEXIT_CRITICAL(&g_mux);
    return e;
}

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
        uint8_t blob[OWNER_BLOB_LEN];
        blob[0] = o.type;
        memcpy(&blob[1], o.addr, FSD_OWNER_ADDR_LEN);
        if(g_nvs.putBytes(KEY, blob, sizeof(blob)) == sizeof(blob)) {
            uint8_t back[OWNER_BLOB_LEN] = {};
            stored = g_nvs.getBytes(KEY, back, sizeof(back)) == sizeof(back) &&
                     memcmp(back, blob, sizeof(blob)) == 0;
        }
        /* The fingerprint follows the record: written when keyed, removed when
         * not (a key-less re-enrolment must not inherit the previous phone's
         * keys). Both have to land for the save to count. */
        if(stored && o.keyed) {
            uint8_t back[FSD_OWNER_KEY_FP_LEN] = {};
            stored = g_nvs.putBytes(KEY_FP, o.key_fp, sizeof(o.key_fp)) == sizeof(o.key_fp) &&
                     g_nvs.getBytes(KEY_FP, back, sizeof(back)) == sizeof(back) &&
                     memcmp(back, o.key_fp, sizeof(back)) == 0;
        } else if(stored) {
            g_nvs.remove(KEY_FP);
            stored = !g_nvs.isKey(KEY_FP);
        }
    } else {
        /* Absent is the goal, however we get there: remove() reports false when
         * the key was not there to begin with, which is success for us. */
        g_nvs.remove(KEY);
        g_nvs.remove(KEY_FP);
        stored = !g_nvs.isKey(KEY) && !g_nvs.isKey(KEY_FP);
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
FsdOwnerVerdict ble_owner_on_bond(uint8_t, const uint8_t*) { return FSD_OWNER_MATCH; }
bool ble_owner_allows(uint8_t, const uint8_t*) { return true; }
void ble_owner_open_window(uint32_t) {}
bool ble_owner_window_open(void) { return false; }
bool ble_owner_enrolled(void) { return true; }
void ble_owner_forget(void) {}
void ble_owner_erase_now(void) {}
bool ble_owner_save_pending(void) { return false; }
void ble_owner_print(void) {}

#endif  // BLE_SERVER_ENABLED
