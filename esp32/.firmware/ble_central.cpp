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

/* Which byte of a notification carries the buttons, and which bit is which.
 *
 * NOTHING HERE IS MEASURED. It cannot be: no button has been bought. The shape
 * is a guess at the most common case (a one-byte report, one bit per key), and
 * `verified` being false is what stops that guess from ever being treated as a
 * press. Exactly the FsdSpEncoding arrangement, for exactly the same reason.
 *
 * To fill it in: bind the button, watch the raw log through two presses of each
 * key, and the offset and masks fall out. Then set verified. */
typedef struct {
    uint8_t report_len; // expected notification length; 0 = accept any
    uint8_t code_offset;
    uint8_t mask[FSD_BTN_MAX]; // bit for each logical button, 0 = unused
    bool verified;
} FsdBtnMap;

static const FsdBtnMap FSD_BTN_MAP = {
    .report_len = 0,
    .code_offset = 0,
    .mask = {0x01u, 0x02u, 0x00u, 0x00u},
    .verified = false,
};

// ── state ────────────────────────────────────────────────────────────────────

#define BLE_CENTRAL_TICK_MS 50u
#define BLE_CENTRAL_RETRY_MS 5000u
#define BLE_CENTRAL_MAX_RETRIES 5u

static Preferences g_prefs;
static char g_addr[20] = {0};
static bool g_verbose = true;

static NimBLEClient* g_client = nullptr;
static volatile bool g_connected = false;
static volatile bool g_want_connect = false;
static uint32_t g_last_try_ms = 0;
static uint8_t g_retries = 0;
static uint32_t g_last_tick_ms = 0;

static FsdButtons g_btns;

/* Written by the BLE task, read by loop(). One byte plus a flag; the report is
 * copied out under a critical section rather than parsed in the callback. */
static portMUX_TYPE g_rep_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_rep[20];
static uint8_t g_rep_len = 0;
static volatile bool g_rep_pending = false;
static volatile uint16_t g_notify_count = 0;

// ── callbacks (BLE host task — keep them trivial) ────────────────────────────

class CentralCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient*) override { g_connected = true; }
    void onDisconnect(NimBLEClient*, int reason) override {
        g_connected = false;
        (void)reason; // logged from the loop task, not from here
    }
};
static CentralCB g_cb;

static void on_notify(NimBLERemoteCharacteristic* chr, uint8_t* data, size_t len,
                      bool isNotify) {
    (void)chr;
    (void)isNotify;
    if(!data || len == 0) return;
    portENTER_CRITICAL(&g_rep_mux);
    g_rep_len = (len > sizeof(g_rep)) ? (uint8_t)sizeof(g_rep) : (uint8_t)len;
    memcpy(g_rep, data, g_rep_len);
    g_rep_pending = true;
    portEXIT_CRITICAL(&g_rep_mux);
    g_notify_count++;
}

// ── connect ──────────────────────────────────────────────────────────────────

/* Subscribe to EVERYTHING that can notify, rather than to a characteristic we
 * think we want. Before the report layout is known, "which one talks when I
 * press it" is the question, and the only way to answer it is to listen to all
 * of them at once. */
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

static bool try_connect(void) {
    if(!g_addr[0]) return false;

    if(!g_client) {
        g_client = NimBLEDevice::createClient();
        if(!g_client) {
            Serial.println("[BTN] no client slot free");
            return false;
        }
        g_client->setClientCallbacks(&g_cb, false);
        /* Give up rather than hold the radio: the phone shares it, and a button
         * that is not in the car must not cost the app its link. */
        g_client->setConnectTimeout(5 * 1000);
    }

    NimBLEAddress addr(g_addr, BLE_ADDR_PUBLIC);
    if(!g_client->connect(addr)) {
        NimBLEAddress rnd(g_addr, BLE_ADDR_RANDOM);
        if(!g_client->connect(rnd)) return false; // most cheap buttons are random
    }

    const int n = subscribe_all(g_client);
    Serial.printf("[BTN] connected %s, %d notifying characteristic(s)\n", g_addr, n);
    if(n == 0) {
        Serial.println("[BTN] nothing to subscribe to — this may be an "
                       "advertise-only button, which needs a different approach");
    }
    return true;
}

// ── public ───────────────────────────────────────────────────────────────────

void ble_central_init(void) {
    fsd_btn_init(&g_btns);
    g_prefs.begin("btn", false);
    String a = g_prefs.getString("addr", "");
    g_prefs.end();
    if(a.length() > 0 && a.length() < sizeof(g_addr)) {
        strncpy(g_addr, a.c_str(), sizeof(g_addr) - 1);
        g_want_connect = true;
        Serial.printf("[BTN] bound to %s\n", g_addr);
    } else {
        Serial.println("[BTN] no button bound — 'btnscan' to look, 'btnbind <addr>' to pair");
    }
}

bool ble_central_scan(uint8_t secs) {
    NimBLEScan* scan = NimBLEDevice::getScan();
    if(!scan || scan->isScanning()) return false;

    Serial.printf("[BTN] scanning %us...\n", (unsigned)secs);
    scan->setActiveScan(true); // ask for names: an address alone identifies nothing
    NimBLEScanResults r = scan->getResults(secs * 1000, false);
    const int n = r.getCount();
    Serial.printf("[BTN] %d device(s)\n", n);
    for(int i = 0; i < n; i++) {
        const NimBLEAdvertisedDevice* d = r.getDevice(i);
        Serial.printf("  %-18s rssi:%-5d %s%s\n", d->getAddress().toString().c_str(),
                      d->getRSSI(), d->haveName() ? d->getName().c_str() : "(no name)",
                      d->haveServiceUUID() ? "  [has services]" : "");
    }
    scan->clearResults();
    Serial.println("[BTN] 'btnbind <addr>' to use one");
    return true;
}

bool ble_central_bind(const char* addr_str) {
    if(g_client && g_client->isConnected()) g_client->disconnect();
    memset(g_addr, 0, sizeof(g_addr));
    if(addr_str && addr_str[0]) {
        strncpy(g_addr, addr_str, sizeof(g_addr) - 1);
    }
    /* NVS write on the caller's task. Both callers are the serial console and
     * the loop task, never a BLE callback — see the file header. */
    g_prefs.begin("btn", false);
    g_prefs.putString("addr", g_addr);
    g_prefs.end();

    g_retries = 0;
    g_want_connect = (g_addr[0] != 0);
    Serial.printf("[BTN] %s\n", g_addr[0] ? g_addr : "unbound");
    return true;
}

const char* ble_central_bound_addr(void) { return g_addr; }
bool ble_central_connected(void) { return g_connected; }
void ble_central_set_verbose(bool on) { g_verbose = on; }
uint16_t ble_central_notify_count(void) { return g_notify_count; }
uint16_t ble_central_short_presses(void) { return fsd_btn_shorts(&g_btns, 0) + fsd_btn_shorts(&g_btns, 1); }
uint16_t ble_central_long_presses(void) { return fsd_btn_longs(&g_btns, 0) + fsd_btn_longs(&g_btns, 1); }

void ble_central_tick(uint32_t now_ms) {
    if((uint32_t)(now_ms - g_last_tick_ms) < BLE_CENTRAL_TICK_MS) return;
    g_last_tick_ms = now_ms;

    // ── a report arrived ─────────────────────────────────────────────────────
    uint8_t rep[sizeof(g_rep)];
    uint8_t len = 0;
    portENTER_CRITICAL(&g_rep_mux);
    if(g_rep_pending) {
        len = g_rep_len;
        memcpy(rep, g_rep, len);
        g_rep_pending = false;
    }
    portEXIT_CRITICAL(&g_rep_mux);

    if(len > 0) {
        if(g_verbose) {
            Serial.print("[BTN] report");
            for(uint8_t i = 0; i < len; i++) Serial.printf(" %02X", rep[i]);
            Serial.println();
        }
        /* THE GATE. Until the layout is confirmed against a real button, a
         * report is a log line and nothing else. Guessing here would mean a
         * random byte from an unknown peripheral counting as a press. */
        if(FSD_BTN_MAP.verified && (FSD_BTN_MAP.report_len == 0 || len == FSD_BTN_MAP.report_len) &&
           FSD_BTN_MAP.code_offset < len) {
            const uint8_t code = rep[FSD_BTN_MAP.code_offset];
            for(uint8_t i = 0; i < FSD_BTN_MAX; i++) {
                if(FSD_BTN_MAP.mask[i] == 0u) continue;
                const bool down = (code & FSD_BTN_MAP.mask[i]) != 0u;
                const FsdBtnEvent e = fsd_btn_report(&g_btns, i, down, now_ms);
                if(e != FSD_BTN_EV_NONE)
                    Serial.printf("[BTN] %u %s (%ums)\n", (unsigned)i,
                                  fsd_btn_event_str(e),
                                  (unsigned)fsd_btn_last_hold_ms(&g_btns, i));
            }
        }
    }

    // LONG and STUCK happen while nothing is being reported.
    for(uint8_t i = 0; i < FSD_BTN_MAX; i++) {
        const FsdBtnEvent e = fsd_btn_tick(&g_btns, i, now_ms);
        if(e != FSD_BTN_EV_NONE)
            Serial.printf("[BTN] %u %s\n", (unsigned)i, fsd_btn_event_str(e));
    }

    /* A press is CLASSIFIED and COUNTED here. Nothing acts on it: the only
     * action a button is meant to drive is the speed-profile step, which is
     * locked behind fsd_sp_encoding_ok() and tx_armed. */

    // ── connection upkeep ────────────────────────────────────────────────────
    if(!g_want_connect || g_connected) return;
    if(g_retries >= BLE_CENTRAL_MAX_RETRIES) return; // stop; the radio is shared
    if((uint32_t)(now_ms - g_last_try_ms) < BLE_CENTRAL_RETRY_MS) return;

    g_last_try_ms = now_ms;
    if(!try_connect()) {
        g_retries++;
        if(g_retries >= BLE_CENTRAL_MAX_RETRIES)
            Serial.printf("[BTN] giving up on %s — 'btnbind %s' to retry\n", g_addr, g_addr);
    } else {
        g_retries = 0;
    }
}

#endif // BLE_SERVER_ENABLED
