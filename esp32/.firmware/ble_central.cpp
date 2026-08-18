/*
 * ble_central.cpp — see ble_central.h.
 *
 * Threading: NimBLE client callbacks run on the BLE host task, exactly like the
 * server's. So they do the least possible — stamp a byte, bump a counter, set a
 * flag — and every decision, every NVS write and every log line happens in
 * ble_central_tick() on the loop task. Same contract as ble_server.cpp.
 */

#include "ble_central.h"

#ifdef BLE_SERVER_ENABLED

#include "../../fsd_logic/fsd_button.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <string.h>

// ── the device-specific table ────────────────────────────────────────────────

/* Which byte of a notification carries the button, and which bit it is.
 *
 * NOTHING HERE IS MEASURED. It cannot be: no button has been bought. The shape
 * is a guess at the most common case (a one-byte report, one bit per key), and
 * `verified` being false is what stops that guess from ever being treated as a
 * press. Exactly the FsdSpEncoding arrangement, for exactly the same reason.
 *
 * ONE BUTTON PER DEVICE. This used to carry a mask per logical button, for one
 * remote with several keys. The plan changed (2026-08-18): up to eight separate
 * single-button remotes, each its own BLE device. So the table describes one
 * device's report and every bound device is scored through it — buy the same
 * model twice and it just works. A second MODEL would need its own entry, which
 * is a problem to solve when a second model exists.
 *
 * To fill it in: bind the button, watch the raw log through two presses, and
 * the offset and mask fall out. Then set verified.
 */
typedef struct {
    uint8_t report_len;  // expected notification length; 0 = accept any
    uint8_t code_offset;
    uint8_t press_mask;  // this bit set = the button is down
    bool verified;
} FsdBtnMap;

static const FsdBtnMap FSD_BTN_MAP = {
    .report_len = 0,
    .code_offset = 0,
    .press_mask = 0x01u,
    .verified = false,
};

// ── state ────────────────────────────────────────────────────────────────────

#define BLE_CENTRAL_TICK_MS 50u
#define BLE_CENTRAL_RETRY_MS 5000u
#define BLE_CENTRAL_MAX_RETRIES 5u

static Preferences g_prefs;
static bool g_verbose = true;
static uint32_t g_last_tick_ms = 0;

/* One slot per bound remote. Slot index IS the logical button index, which is
 * why BLE_CENTRAL_MAX_BUTTONS and FSD_BTN_MAX are the same number.
 *
 * Each slot keeps its own retry budget and its own clock: eight remotes waking
 * with the car must not all hammer the radio in one tick, and one that is out
 * of range must not stall the others.
 */
typedef struct {
    char addr[20];             // "" = free slot
    NimBLEClient* client;
    volatile bool connected;
    uint8_t retries;
    uint32_t last_try_ms;
} CentralSlot;
static CentralSlot g_slot[BLE_CENTRAL_MAX_BUTTONS];

static FsdButtons g_btns;

/* Last scan's results, kept so the PHONE can see them.
 *
 * Before this the scan printed to serial and threw the list away — fine for a
 * USB bring-up, useless to an app. The phone has no serial port.
 */
typedef struct {
    char addr[18];  // "aa:bb:cc:dd:ee:ff"
    char name[24];
    int8_t rssi;
} CentralFound;
static CentralFound g_found[BLE_CENTRAL_MAX_FOUND];
static uint8_t g_found_n = 0;

/* A scan BLOCKS for whole seconds. It must never run on the BLE host task, so
 * a command parks the request here and loop() performs it — the same rule the
 * mode switch follows (PR #34).
 */
static volatile uint8_t g_scan_req = 0;
static volatile bool g_scanning = false;

/* Written by the BLE task, read by loop(). Per slot, because with eight remotes
 * the identity of the sender IS the signal — a shared buffer would make every
 * remote press the same logical button.
 */
static portMUX_TYPE g_rep_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_rep[BLE_CENTRAL_MAX_BUTTONS][20];
static uint8_t g_rep_len[BLE_CENTRAL_MAX_BUTTONS];
static volatile bool g_rep_pending[BLE_CENTRAL_MAX_BUTTONS];
static volatile uint16_t g_notify_count = 0;

/** Which slot owns this client. -1 when it is not ours. */
static int slot_of(const NimBLEClient* c) {
    if(!c) return -1;
    for(int i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++)
        if(g_slot[i].client == c) return i;
    return -1;
}

// ── callbacks (BLE host task — keep them trivial) ────────────────────────────

class CentralCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* c) override {
        const int i = slot_of(c);
        if(i >= 0) g_slot[i].connected = true;
    }
    void onDisconnect(NimBLEClient* c, int reason) override {
        const int i = slot_of(c);
        if(i >= 0) g_slot[i].connected = false;
        (void)reason; // logged from the loop task, not from here
    }
};
static CentralCB g_cb;

static void on_notify(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len,
                      bool isNotify) {
    (void)isNotify;
    if(!data || len == 0 || !chr) return;
    const NimBLERemoteService* svc = chr->getRemoteService();
    const int i = svc ? slot_of(svc->getClient()) : -1;
    if(i < 0) return; // not one of ours

    portENTER_CRITICAL(&g_rep_mux);
    g_rep_len[i] = (len > sizeof(g_rep[i])) ? (uint8_t)sizeof(g_rep[i]) : (uint8_t)len;
    memcpy(g_rep[i], data, g_rep_len[i]);
    g_rep_pending[i] = true;
    portEXIT_CRITICAL(&g_rep_mux);
    g_notify_count++;
}

// ── connect ──────────────────────────────────────────────────────────────────

/* Subscribe to EVERYTHING that can notify, rather than to a characteristic we
 * think we want. Before the report layout is known, "which one talks when I
 * press it" is the question, and the only way to answer it is to listen to all
 * of them at once.
 *
 * ⚠️ This is why CONFIG_BT_NIMBLE_MAX_CCCDS is raised in platformio.ini: a
 * remote can take two or three subscription slots on its own, and at the stock
 * 8 the fourth remote would simply stop reporting with nothing in the log.
 */
static int subscribe_all(NimBLEClient* c) {
    int n = 0;
    for(auto* svc : c->getServices(true)) {
        for(auto* chr : svc->getCharacteristics(true)) {
            if(!chr->canNotify() && !chr->canIndicate()) continue;
            if(chr->subscribe(chr->canNotify(), on_notify)) {
                n++;
                Serial.printf("[BTN] subscribed %s / %s\n",
                              svc->getUUID().toString().c_str(),
                              chr->getUUID().toString().c_str());
            }
        }
    }
    return n;
}

static bool try_connect(uint8_t i) {
    CentralSlot* sl = &g_slot[i];
    if(!sl->addr[0]) return false;

    if(!sl->client) {
        sl->client = NimBLEDevice::createClient();
        if(!sl->client) {
            Serial.printf("[BTN] %u: no client slot free (radio budget)\n", (unsigned)i);
            return false;
        }
        sl->client->setClientCallbacks(&g_cb, false);
        /* Give up rather than hold the radio: the phone shares it, and a button
         * that is not in the car must not cost the app its link. */
        sl->client->setConnectTimeout(5 * 1000);
    }

    NimBLEAddress addr(sl->addr, BLE_ADDR_PUBLIC);
    if(!sl->client->connect(addr)) {
        NimBLEAddress rnd(sl->addr, BLE_ADDR_RANDOM);
        if(!sl->client->connect(rnd)) return false; // most cheap buttons are random
    }

    const int n = subscribe_all(sl->client);
    Serial.printf("[BTN] %u connected %s, %d notifying characteristic(s)\n",
                  (unsigned)i, sl->addr, n);
    if(n == 0) {
        Serial.println("[BTN] nothing to subscribe to - this may be an "
                       "advertise-only button, which needs a different approach");
    }
    return true;
}

// ── public ───────────────────────────────────────────────────────────────────

/* NVS keys are addr0..addr7. The old single-button build stored "addr"; it is
 * migrated into slot 0 so a module that was already paired stays paired. */
static void slots_save(void) {
    g_prefs.begin("btn", false);
    char key[8];
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        snprintf(key, sizeof(key), "addr%u", (unsigned)i);
        g_prefs.putString(key, g_slot[i].addr);
    }
    g_prefs.end();
}

void ble_central_init(void) {
    fsd_btn_init(&g_btns);
    memset(g_slot, 0, sizeof(g_slot));

    g_prefs.begin("btn", false);
    char key[8];
    uint8_t n = 0;
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        snprintf(key, sizeof(key), "addr%u", (unsigned)i);
        String a = g_prefs.getString(key, "");
        if(a.length() > 0 && a.length() < sizeof(g_slot[i].addr)) {
            strncpy(g_slot[i].addr, a.c_str(), sizeof(g_slot[i].addr) - 1);
            n++;
        }
    }
    String old = g_prefs.getString("addr", "");
    g_prefs.end();
    if(n == 0 && old.length() > 0 && old.length() < sizeof(g_slot[0].addr)) {
        strncpy(g_slot[0].addr, old.c_str(), sizeof(g_slot[0].addr) - 1);
        n = 1;
        slots_save();
        Serial.println("[BTN] migrated the previously bound button into slot 0");
    }

    if(n) Serial.printf("[BTN] %u button(s) bound\n", (unsigned)n);
    else  Serial.println("[BTN] none bound - 'btnscan' to look, 'btnbind <addr>' to pair");
}

bool ble_central_scan(uint8_t secs) {
    NimBLEScan* scan = NimBLEDevice::getScan();
    if(!scan || scan->isScanning()) return false;

    Serial.printf("[BTN] scanning %us...\n", (unsigned)secs);
    scan->setActiveScan(true); // ask for names: an address alone identifies nothing
    NimBLEScanResults r = scan->getResults(secs * 1000, false);
    const int n = r.getCount();
    Serial.printf("[BTN] %d device(s)\n", n);

    g_found_n = 0;
    for(int i = 0; i < n; i++) {
        const NimBLEAdvertisedDevice* d = r.getDevice(i);
        Serial.printf("  %-18s rssi:%-5d %s%s\n", d->getAddress().toString().c_str(),
                      d->getRSSI(), d->haveName() ? d->getName().c_str() : "(no name)",
                      d->haveServiceUUID() ? "  [has services]" : "");
        if(g_found_n < BLE_CENTRAL_MAX_FOUND) {
            CentralFound* f = &g_found[g_found_n++];
            snprintf(f->addr, sizeof(f->addr), "%s", d->getAddress().toString().c_str());
            snprintf(f->name, sizeof(f->name), "%s",
                     d->haveName() ? d->getName().c_str() : "");
            f->rssi = (int8_t)d->getRSSI();
        }
    }
    if(n > BLE_CENTRAL_MAX_FOUND)
        Serial.printf("[BTN] keeping the first %d for the app\n", BLE_CENTRAL_MAX_FOUND);

    scan->clearResults();
    Serial.println("[BTN] 'btnbind <addr>' to use one");
    return true;
}

/* Disconnect and free the radio slot a remote was holding. */
static void slot_drop(uint8_t i) {
    CentralSlot* sl = &g_slot[i];
    if(sl->client) {
        if(sl->client->isConnected()) sl->client->disconnect();
        NimBLEDevice::deleteClient(sl->client);
        sl->client = nullptr;
    }
    sl->connected = false;
    sl->retries = 0;
    sl->addr[0] = 0;
    // The logical button goes with it, or a stuck press would outlive the remote.
    fsd_btn_report(&g_btns, i, false, millis());
}

int ble_central_add(const char* addr_str) {
    if(!addr_str || !addr_str[0]) return -1;
    // Already bound? Say which slot rather than taking a second one.
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++)
        if(strcmp(g_slot[i].addr, addr_str) == 0) return (int)i;

    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        if(g_slot[i].addr[0]) continue;
        strncpy(g_slot[i].addr, addr_str, sizeof(g_slot[i].addr) - 1);
        g_slot[i].retries = 0;
        g_slot[i].last_try_ms = 0;
        slots_save();
        Serial.printf("[BTN] slot %u = %s\n", (unsigned)i, g_slot[i].addr);
        return (int)i;
    }
    Serial.printf("[BTN] all %d slots are taken - forget one first\n",
                  BLE_CENTRAL_MAX_BUTTONS);
    return -1;
}

int ble_central_add_found(uint8_t scan_index) {
    if(scan_index >= g_found_n) return -1;
    return ble_central_add(g_found[scan_index].addr);
}

bool ble_central_forget(uint8_t slot) {
    if(slot >= BLE_CENTRAL_MAX_BUTTONS || !g_slot[slot].addr[0]) return false;
    Serial.printf("[BTN] slot %u forgotten\n", (unsigned)slot);
    slot_drop(slot);
    slots_save();
    return true;
}

void ble_central_forget_all(void) {
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++)
        if(g_slot[i].addr[0]) slot_drop(i);
    slots_save();
    Serial.println("[BTN] all slots cleared");
}

const char* ble_central_slot_addr(uint8_t slot) {
    return (slot < BLE_CENTRAL_MAX_BUTTONS) ? g_slot[slot].addr : "";
}

bool ble_central_slot_connected(uint8_t slot) {
    return (slot < BLE_CENTRAL_MAX_BUTTONS) && g_slot[slot].connected;
}

uint8_t ble_central_bound_count(void) {
    uint8_t n = 0;
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) if(g_slot[i].addr[0]) n++;
    return n;
}

bool ble_central_any_connected(void) {
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++)
        if(g_slot[i].connected) return true;
    return false;
}

uint8_t ble_central_found_count(void) { return g_found_n; }

bool ble_central_found(uint8_t i, const char** addr, const char** name, int8_t* rssi) {
    if(i >= g_found_n) return false;
    if(addr) *addr = g_found[i].addr;
    if(name) *name = g_found[i].name;
    if(rssi) *rssi = g_found[i].rssi;
    return true;
}

void ble_central_request_scan(uint8_t secs) {
    if(secs == 0) secs = BLE_CENTRAL_SCAN_SECS;
    if(secs > 20u) secs = 20u; // the phone shares this radio
    g_scan_req = secs;
}

bool ble_central_scanning(void) { return g_scanning; }

void ble_central_set_verbose(bool on) { g_verbose = on; }
uint16_t ble_central_notify_count(void) { return g_notify_count; }

static uint16_t sum_over_slots(uint16_t (*f)(const FsdButtons*, uint8_t)) {
    uint32_t n = 0;
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) n += f(&g_btns, i);
    return (n > 0xFFFFu) ? 0xFFFFu : (uint16_t)n;
}
uint16_t ble_central_short_presses(void)  { return sum_over_slots(fsd_btn_shorts); }
uint16_t ble_central_long_presses(void)   { return sum_over_slots(fsd_btn_longs); }
uint16_t ble_central_double_presses(void) { return sum_over_slots(fsd_btn_doubles); }

void ble_central_tick(uint32_t now_ms) {
    if((uint32_t)(now_ms - g_last_tick_ms) < BLE_CENTRAL_TICK_MS) return;
    g_last_tick_ms = now_ms;

    /* A parked scan runs HERE, on loop(), never on the BLE host task. It blocks
     * for seconds; doing that in a characteristic callback would stall the
     * phone's link and look like the module died. */
    if(g_scan_req) {
        const uint8_t secs = g_scan_req;
        g_scan_req = 0;
        g_scanning = true;
        ble_central_scan(secs);
        g_scanning = false;
        return; // that took seconds; pick the rest up next tick
    }

    // ── reports that arrived ─────────────────────────────────────────────────
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        uint8_t rep[sizeof(g_rep[0])];
        uint8_t len = 0;
        portENTER_CRITICAL(&g_rep_mux);
        if(g_rep_pending[i]) {
            len = g_rep_len[i];
            memcpy(rep, g_rep[i], len);
            g_rep_pending[i] = false;
        }
        portEXIT_CRITICAL(&g_rep_mux);
        if(len == 0) continue;

        if(g_verbose) {
            Serial.printf("[BTN] %u report", (unsigned)i);
            for(uint8_t k = 0; k < len; k++) Serial.printf(" %02X", rep[k]);
            Serial.println();
        }
        /* THE GATE. Until the layout is confirmed against a real button, a
         * report is a log line and nothing else. Guessing here would mean a
         * random byte from an unknown peripheral counting as a press. */
        if(FSD_BTN_MAP.verified &&
           (FSD_BTN_MAP.report_len == 0 || len == FSD_BTN_MAP.report_len) &&
           FSD_BTN_MAP.code_offset < len) {
            /* Which entry point depends on what this button can say (see
             * fsd_button.h). Feeding a level to an event button, or a pulse to
             * a level one, is refused there rather than guessed — so the branch
             * has to live here, at the only place that knows the kind.
             *
             * EVENT is the default, which is what the TSL remote measured on
             * 2026-08-18 appears to be: it says a press happened and never that
             * it ended. A notification IS the whole gesture. */
            FsdBtnEvent e;
            if(fsd_btn_kind(&g_btns, i) == FSD_BTN_KIND_LEVEL) {
                const bool down =
                    (rep[FSD_BTN_MAP.code_offset] & FSD_BTN_MAP.press_mask) != 0u;
                e = fsd_btn_report(&g_btns, i, down, now_ms);
            } else {
                e = fsd_btn_pulse(&g_btns, i, now_ms);
            }
            if(e != FSD_BTN_EV_NONE)
                Serial.printf("[BTN] %u %s (%ums)\n", (unsigned)i,
                              fsd_btn_event_str(e),
                              (unsigned)fsd_btn_last_hold_ms(&g_btns, i));
        }
    }

    // LONG, DOUBLE and STUCK happen while nothing is being reported.
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        const FsdBtnEvent e = fsd_btn_tick(&g_btns, i, now_ms);
        if(e != FSD_BTN_EV_NONE)
            Serial.printf("[BTN] %u %s\n", (unsigned)i, fsd_btn_event_str(e));
    }

    /* A press is CLASSIFIED and COUNTED here. Nothing acts on it: the only
     * action a button is meant to drive is the speed-profile step, which is
     * locked behind fsd_sp_encoding_ok() and tx_armed. */

    // ── connection upkeep, at most one attempt per tick ───────────────────────
    //
    // 🔴 One per tick, on purpose. Eight remotes waking with the car would
    // otherwise all try in the same pass, and each attempt blocks for up to the
    // connect timeout — the phone's link would stall for seconds.
    for(uint8_t k = 0; k < BLE_CENTRAL_MAX_BUTTONS; k++) {
        CentralSlot* sl = &g_slot[k];
        if(!sl->addr[0] || sl->connected) continue;
        if(sl->retries >= BLE_CENTRAL_MAX_RETRIES) continue; // stop; radio is shared
        if((uint32_t)(now_ms - sl->last_try_ms) < BLE_CENTRAL_RETRY_MS) continue;

        sl->last_try_ms = now_ms;
        if(!try_connect(k)) {
            sl->retries++;
            if(sl->retries >= BLE_CENTRAL_MAX_RETRIES)
                Serial.printf("[BTN] %u giving up on %s - rebind to retry\n",
                              (unsigned)k, sl->addr);
        } else {
            sl->retries = 0;
        }
        break; // at most one connect attempt per tick
    }
}

#endif // BLE_SERVER_ENABLED
