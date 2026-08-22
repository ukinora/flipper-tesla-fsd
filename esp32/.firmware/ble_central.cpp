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

#include "../../fsd_logic/fsd_btn_j6.h"
#include "../../fsd_logic/fsd_button.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <string.h>

// ── the device-specific table ────────────────────────────────────────────────

/* IT MOVED, AND IT IS MEASURED NOW: fsd_logic/fsd_btn_j6.{h,c}.
 *
 * What used to sit here was an FsdBtnMap — one byte offset, one bit mask, and
 * `verified = false` because no button had been bought. The remote that was
 * eventually chosen (Yiser J6, 차주 결정 2026-08-18) cannot be described that
 * way at all:
 *
 *   - Seven buttons arrive on the SAME two report characteristics, so a mask
 *     cannot tell them apart.
 *   - It is not a keyboard. A press is a synthesised swipe or tap, so what
 *     identifies a button is the DIRECTION a coordinate travelled or WHERE the
 *     tap landed — not a bit.
 *   - A long press comes back as a consumer-control code on the other
 *     characteristic entirely.
 *
 * So the decoder is a real state machine, it lives in fsd_logic/ where it can
 * be tested on the host against bytes captured from the device, and this file
 * only routes what it decides. The 블루투스-버튼-조사.md table is the source. */

// ── state ────────────────────────────────────────────────────────────────────

#define BLE_CENTRAL_TICK_MS 50u
#define BLE_CENTRAL_RETRY_MS 5000u
#define BLE_CENTRAL_MAX_RETRIES 5u

/* After the fast tries, keep looking — just rarely. NEVER give up: a remote
 * that is switched off or asleep is the ordinary case, not a fault. */
#define BLE_CENTRAL_SLOW_RETRY_MS 30000u

/* How long one connect attempt may block loop(). The phone's reads fail for
 * exactly this long, so it is a latency budget, not just a timeout. */
#define BLE_CENTRAL_CONNECT_MS 3000u

static Preferences g_prefs;
static bool g_verbose = true;
static uint32_t g_last_tick_ms = 0;

/* One slot per bound remote.
 *
 * 🔴 SLOT INDEX IS NOT THE BUTTON INDEX ANY MORE. It was, while the plan was
 * several single-button remotes. The J6 puts NINE buttons on ONE link, so the
 * two counts are now separate things:
 *
 *    BLE_CENTRAL_MAX_BUTTONS  links   — a radio budget (ble_central.h)
 *    FSD_BTN_MAX              buttons — an index space (fsd_button.h)
 *
 * The logical button index comes from fsd_btn_j6.h, not from the slot. Two
 * identical remotes therefore drive the SAME buttons, which is the behaviour
 * that was wanted anyway: a spare in the glovebox needs no configuration.
 *
 * Each slot keeps its own retry budget and its own clock: several remotes
 * waking with the car must not all hammer the radio in one tick, and one that
 * is out of range must not stall the others.
 */
typedef struct {
    char addr[20];             // "" = free slot
    NimBLEClient* client;
    volatile bool connected;
    /* Which address form actually worked last time: 0 unknown, 1 public,
     * 2 random.
     *
     * 🔴 A failed attempt costs the CONNECT TIMEOUT TWICE, because both forms
     * are tried. Ten seconds of blocked loop() is ten seconds the phone's GATT
     * reads fail and its link drops — the module going looking for a remote
     * knocks the app off. Remembering which form answered halves that, and
     * most cheap remotes are random, so the public attempt was pure waste. */
    uint8_t addr_kind;

    /* Presses from THIS remote that the decoder could name. Kept per slot
     * rather than as one total: the question the app asks is about one
     * remote ("did the thing I just bound work?"), and a total answers it
     * wrongly the moment a second remote exists. */
    uint16_t decoded;
    uint8_t retries;
    uint32_t last_try_ms;
} CentralSlot;
static CentralSlot g_slot[BLE_CENTRAL_MAX_BUTTONS];

/* Press classification is PER REMOTE, for the same reason the gesture state is.
 *
 * 🔴 IT WAS ONE SHARED STRUCT and that broke the moment a second remote existed
 * (red team, 2026-08-19). 6번 is a LEVEL button, so its press and release are
 * two separate reports with a hold in between — and one shared FsdButtons meant
 * remote B's 6번 press was discarded as a repeated down while remote A held it,
 * and A's release ended B's hold with A's timing. slot_drop() was worse: it
 * released 6번 with no dependence on the slot, so FORGETTING slot 0 let go of a
 * hold in progress on slot 1.
 *
 * 360 bytes each, five slots, 1.8 KB. The blackbox ring lives in PSRAM, so this
 * is not the RAM that competes with the one-shot capture. */
static FsdButtons g_btns[BLE_CENTRAL_MAX_BUTTONS];

/* Which rows are switched on. See ble_central.h for the row encoding. */
static uint32_t g_action_mask = 0;



/* Gesture state is PER REMOTE, not per button: a contact belongs to the device
 * it came from. Sharing one would let two remotes interleave halves of a swipe
 * into a direction neither person made. */
static FsdJ6 g_j6[BLE_CENTRAL_MAX_BUTTONS];

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
/* How many the radio actually saw. `g_found_n` is what we kept — the app must
 * be able to say "8 of 44" rather than pretend eight is all there was. */
static uint16_t g_found_total = 0;

/* Filled by the scan callback on the BLE host task; copied into g_found only
 * when the scan ends. A list being written while it is read is worse than a
 * stale one — devices would appear and vanish under the reader. */
static CentralFound g_stage[BLE_CENTRAL_MAX_FOUND];
static volatile uint8_t g_stage_n = 0;

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
/* Reports go through a RING, not one box per slot.
 *
 * 🔴 It was one box per slot, and that quietly lost reports. A real HID remote
 * (YISHE J6, measured 2026-08-18) sends SIX notifications inside ~300 ms, from
 * different characteristics; the loop drains every 50 ms, so five of the six
 * were overwritten before anyone looked. The log showed a plausible-looking
 * stream and it was a sample, not the traffic.
 *
 * That is fatal for the job this file exists to do — "which characteristic
 * talks when I press which key" cannot be answered from a lossy sample.
 *
 * 🔴 The characteristic handle rides along for the same reason. This device has
 * SEVEN report characteristics (keyboard, consumer, digitizer...) and printing
 * only the bytes makes them indistinguishable. */
typedef struct {
    uint8_t slot;
    uint16_t handle;
    uint8_t len;
    uint8_t b[20];
} CentralRep;

static portMUX_TYPE g_rep_mux = portMUX_INITIALIZER_UNLOCKED;
#define CENTRAL_RING 24
static CentralRep g_ring[CENTRAL_RING];
static volatile uint8_t g_ring_head = 0; // written by the BLE task
static volatile uint8_t g_ring_tail = 0; // read by loop()
/* Silence about loss is what made the old bug invisible. Count it and say it. */
static volatile uint16_t g_ring_dropped = 0;
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
    const uint8_t next = (uint8_t)((g_ring_head + 1) % CENTRAL_RING);
    if(next == g_ring_tail) {
        g_ring_dropped++; // full: the loop is behind. Say so rather than silently lose it.
    } else {
        CentralRep* r = &g_ring[g_ring_head];
        r->slot = (uint8_t)i;
        r->handle = chr->getHandle();
        r->len = (len > sizeof(r->b)) ? (uint8_t)sizeof(r->b) : (uint8_t)len;
        memcpy(r->b, data, r->len);
        g_ring_head = next;
    }
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
    int failed = 0;
    for(auto* svc : c->getServices(true)) {
        for(auto* chr : svc->getCharacteristics(true)) {
            if(!chr->canNotify() && !chr->canIndicate()) continue;
            if(chr->subscribe(chr->canNotify(), on_notify)) {
                n++;
                Serial.printf("[BTN] subscribed %s / %s\n",
                              svc->getUUID().toString().c_str(),
                              chr->getUUID().toString().c_str());
            } else {
                failed++;
                Serial.printf("[BTN] subscribe REFUSED %s / %s\n",
                              svc->getUUID().toString().c_str(),
                              chr->getUUID().toString().c_str());
            }
        }
    }

    /* 🔴 A REFUSED SUBSCRIPTION USED TO BE INVISIBLE. Only successes were
     * logged, and try_connect() warns only when the count is zero — so a remote
     * that got half its characteristics connected, reported a healthy-looking
     * number, and then never notified a press. Its `decoded` stayed at 0 and
     * the app drew it as "connected but broken" with nothing to explain why.
     *
     * The way it happens is the CCCD table filling up: this device alone takes
     * seven, so five remotes want thirty-five of the forty in platformio.ini
     * and there is not much room left over. That is the fourth-remote failure
     * the platformio comment warns about, and this is the line that names it. */
    if(failed)
        Serial.printf("[BTN] %d subscription(s) refused - the CCCD table "
                      "(CONFIG_BT_NIMBLE_MAX_CCCDS) is probably full; this "
                      "remote will not report every press\n", failed);
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
        /* Give up rather than hold the radio: the phone shares it, and a button
         * that is not in the car must not cost the app its link. Three seconds
         * is enough for a remote that IS advertising — the measured connects
         * were well under a second — and every second past that is a second of
         * the phone's link being dead. */
        sl->client->setConnectTimeout(BLE_CENTRAL_CONNECT_MS);
    }

    /* Try the form that worked last time first. On a remote that is asleep this
     * is the whole difference between one timeout and two. */
    const bool random_first = (sl->addr_kind == 2u);
    const uint8_t first = random_first ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
    const uint8_t second = random_first ? BLE_ADDR_PUBLIC : BLE_ADDR_RANDOM;

    if(sl->client->connect(NimBLEAddress(sl->addr, first))) {
        sl->addr_kind = random_first ? 2u : 1u;
    } else if(sl->client->connect(NimBLEAddress(sl->addr, second))) {
        sl->addr_kind = random_first ? 1u : 2u;
    } else {
        return false;
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

/* Give one slot's buttons their settings from scratch.
 *
 * Shared by init and slot_drop so a REUSED slot cannot come back configured
 * differently from its neighbours — the kind above all, because registering a
 * non-level button as LEVEL wedges it permanently: a hold it never ends becomes
 * LONG at 0.6 s and STUCK at 10 s, and STUCK clears only on a release.
 *
 * Order is load-bearing. fsd_btn_set_kind() clears a button's in-progress
 * state, so the kinds go on before the double-press choices, not after. */
static void slot_buttons_reset(uint8_t i) {
    fsd_btn_init(&g_btns[i]);

    /* 🔴 EVENT is the default and stays the default for eight of the nine.
     * Only 6번 reports its release. Asking the table which ones are level —
     * rather than listing them here — keeps that decision next to the
     * measurement that justifies it. */
    for(uint8_t b = 0; b < FSD_J6_COUNT; b++) {
        if(fsd_j6_is_level((FsdJ6Btn)b)) fsd_btn_set_kind(&g_btns[i], b, FSD_BTN_KIND_LEVEL);
        /* The window comes from the GESTURE, not from a setting. A swipe takes
         * a third of a second to finish, so a flat window makes its double
         * press fail one time in six — and a failed double fires the SINGLE
         * action twice. Costs nothing while double-press is off. */
        fsd_btn_set_double_window(&g_btns[i], b, fsd_j6_double_window_ms((FsdJ6Btn)b));
    }

    /* The action mask is the truth; the timing is derived from it, never
     * stored twice. */
    for(uint8_t b = 0; b < FSD_BTN_MAX; b++) {
        const uint8_t row = (uint8_t)(b * FSD_BTN_EVENTS + 1u);
        fsd_btn_set_double(&g_btns[i], b, row < 32u && ((g_action_mask >> row) & 1uL));
    }
}

void ble_central_init(void) {
    memset(g_slot, 0, sizeof(g_slot));

    /* 🔴 THE MASK IS LOADED FIRST because slot_buttons_reset() derives the
     * double-press timing from it. Loading it after would give every slot the
     * settings of an empty mask, and the next reset — a forget — would silently
     * give that one slot different behaviour from its neighbours. */
    g_prefs.begin("btn", true);
    g_action_mask = g_prefs.getUInt("act", 0);
    g_prefs.end();

    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        slot_buttons_reset(i);
        fsd_j6_init(&g_j6[i]);
    }

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

/* ── the scan, and why it does not block ──────────────────────────────────────
 *
 * 🔴 THIS USED TO CALL getResults(), WHICH BLOCKS FOR THE WHOLE SCAN, and that
 * is what made the app's "버튼 찾기" unusable (2026-08-19). Five seconds inside
 * loop() is five seconds the module answers no GATT reads, so the phone's link
 * dropped every time — the scan ran, the module found the remote, and the
 * result could never reach the screen because the screen was no longer there.
 *
 * NimBLE's start() is asynchronous: results arrive on the host task and
 * onScanEnd() fires at the finish. loop() keeps running throughout.
 *
 * Results land in a staging buffer and are published only at the end. Halfway
 * through a scan the list is neither the old answer nor the new one, and a
 * phone reading it then would see devices appear and vanish. */

class CentralScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* d) override {
        if(!d || g_stage_n >= BLE_CENTRAL_MAX_FOUND) return;
        CentralFound* f = &g_stage[g_stage_n++];
        snprintf(f->addr, sizeof(f->addr), "%s", d->getAddress().toString().c_str());
        snprintf(f->name, sizeof(f->name), "%s", d->haveName() ? d->getName().c_str() : "");
        f->rssi = (int8_t)d->getRSSI();
    }
    void onScanEnd(const NimBLEScanResults& r, int reason) override {
        (void)reason;
        const int n = r.getCount();
        /* Publish in one go. Copying under a critical section rather than
         * writing g_found as results arrive: loop() reads this to build the
         * document, and a half-written list is worse than a stale one. */
        portENTER_CRITICAL(&g_rep_mux);
        memcpy(g_found, g_stage, sizeof(g_found));
        g_found_n = g_stage_n;
        g_found_total = (n < 0) ? 0 : (uint16_t)n;
        portEXIT_CRITICAL(&g_rep_mux);
        g_scanning = false;
        NimBLEDevice::getScan()->clearResults();
    }
};
static CentralScanCB g_scan_cb;

bool ble_central_scan(uint8_t secs) {
    NimBLEScan* scan = NimBLEDevice::getScan();
    if(!scan || scan->isScanning() || g_scanning) return false;

    g_stage_n = 0;
    g_scanning = true;
    scan->setScanCallbacks(&g_scan_cb, false);
    scan->setActiveScan(true); // ask for names: an address alone identifies nothing
    if(!scan->start(secs * 1000u, false)) {
        g_scanning = false;
        Serial.println("[BTN] scan refused to start");
        return false;
    }
    Serial.printf("[BTN] scanning %us (loop keeps running)\n", (unsigned)secs);
    return true;
}

/* ── two bring-up instruments ────────────────────────────────────────────────
 *
 * A TSL remote connected on 2026-08-18 and then said nothing through five
 * presses. Three explanations survive and they need different fixes, so the
 * cheapest thing is to measure rather than pick:
 *
 *   1. it wants an encrypted link before it will notify   -> ble_central_secure()
 *   2. its own protocol needs a write first                -> not covered here
 *   3. the ADVERTISEMENT is the event, not a notification  -> ble_central_raw()
 *
 * Both are diagnostics: nothing calls them on its own and neither changes how
 * a bound button behaves. */

bool ble_central_raw(const char* addr_str, uint8_t secs) {
    if(!addr_str || !addr_str[0]) return false;
    if(g_scanning) return false;
    if(secs == 0) secs = BLE_CENTRAL_SCAN_SECS;

    NimBLEScan* scan = NimBLEDevice::getScan();
    if(!scan) return false;

    Serial.printf("[BTN] raw %s for %us — press the button\n", addr_str, (unsigned)secs);
    scan->setActiveScan(true); // scan response too: the name often lives there

    /* 🔴 DETACH THE SCAN CALLBACK FIRST, or this function reports "not seen"
     * for a device that WAS there.
     *
     * ble_central_scan() installs CentralScanCB and never removes it, and that
     * callback's onScanEnd() calls clearResults(). getResults() below runs a
     * scan to completion, so onScanEnd fires and EMPTIES the result set before
     * the loop underneath can read it. Every btnraw after the first btnscan of
     * a boot therefore printed "not seen" -- measured 2026-08-23 against a
     * button that btnscan was finding in the same minute.
     *
     * nullptr is not a crash: NimBLEScan::setScanCallbacks() substitutes its
     * own default instance. `true` asks for duplicates, which is what a raw
     * dump wants -- this remote advertises only while pressed, so every packet
     * of that short burst is worth keeping. */
    scan->setScanCallbacks(nullptr, true);
    NimBLEScanResults r = scan->getResults(secs * 1000, false);

    int hits = 0;
    for(int i = 0; i < r.getCount(); i++) {
        const NimBLEAdvertisedDevice* d = r.getDevice(i);
        if(strcasecmp(d->getAddress().toString().c_str(), addr_str) != 0) continue;
        hits++;
        const std::vector<uint8_t>& p = d->getPayload();
        Serial.printf("[BTN] raw rssi:%d len:%u  ", d->getRSSI(), (unsigned)p.size());
        for(size_t k = 0; k < p.size(); k++) Serial.printf("%02X", p[k]);
        Serial.println();
    }
    /* Silence is a result too — this button only advertises while it is being
     * pressed, so "0 seen" means the press did not reach us at all. */
    if(hits == 0) Serial.println("[BTN] raw: not seen (it may only advertise while pressed)");

    scan->clearResults();
    return true;
}


// ── packet counter (btncount) ────────────────────────────────────────────────
//
// Runs on the BLE host task, so it does the least possible: match the address,
// bump a counter, stamp a time. All printing happens on the loop task after the
// scan returns.

#define CENTRAL_BURST_MAX 12

typedef struct {
    char     name[26];    // the AD name, which is what changes between presses
    uint32_t packets;
    uint32_t first_ms;
    uint32_t last_ms;
} CentralBurst;

static CentralBurst g_burst[CENTRAL_BURST_MAX];
static uint8_t      g_burst_n = 0;
static uint32_t     g_burst_pkts = 0;
static uint32_t     g_burst_dropped = 0;   // bursts past the array

/* Pull the Complete Local Name (AD type 0x09) out of a raw payload.
 *
 * This remote carries its per-press value in the NAME, not in manufacturer
 * data -- the manufacturer field is nine zero bytes. Grouping by name is
 * therefore grouping by press. Falls back to an empty string, which groups
 * everything together rather than splitting a burst into singles. */
static void adv_name(const uint8_t* p, size_t n, char* out, size_t out_sz) {
    out[0] = 0;
    size_t i = 0;
    while(i + 1 < n) {
        const uint8_t len = p[i];
        if(len == 0 || i + 1 + len > n) return;
        if(p[i + 1] == 0x09) {                 // Complete Local Name
            size_t take = len - 1;
            if(take > out_sz - 1) take = out_sz - 1;
            memcpy(out, &p[i + 2], take);
            out[take] = 0;
            return;
        }
        i += 1 + len;
    }
}

class CentralCountCB : public NimBLEScanCallbacks {
public:
    char target[18] = {0};
    void onResult(const NimBLEAdvertisedDevice* d) override {
        if(!d) return;
        if(strcasecmp(d->getAddress().toString().c_str(), target) != 0) return;
        const uint32_t now = millis();
        g_burst_pkts++;

        char nm[26];
        const std::vector<uint8_t>& p = d->getPayload();
        adv_name(p.data(), p.size(), nm, sizeof(nm));

        /* Same name as a burst already open -> same press. Scanning the whole
         * table rather than only the newest: the radio interleaves the three
         * advertising channels and packets do not always arrive in order. */
        for(uint8_t k = 0; k < g_burst_n; k++) {
            if(strcmp(g_burst[k].name, nm) == 0) {
                g_burst[k].packets++;
                g_burst[k].last_ms = now;
                return;
            }
        }
        if(g_burst_n >= CENTRAL_BURST_MAX) { g_burst_dropped++; return; }
        CentralBurst* b = &g_burst[g_burst_n++];
        snprintf(b->name, sizeof(b->name), "%s", nm);
        b->packets = 1;
        b->first_ms = b->last_ms = now;
    }
};
static CentralCountCB g_count_cb;

bool ble_central_count(const char* addr_str, uint8_t secs) {
    if(!addr_str || !addr_str[0]) return false;
    if(g_scanning) return false;
    if(secs == 0) secs = BLE_CENTRAL_SCAN_SECS;

    NimBLEScan* scan = NimBLEDevice::getScan();
    if(!scan) return false;

    snprintf(g_count_cb.target, sizeof(g_count_cb.target), "%s", addr_str);
    g_burst_n = 0; g_burst_pkts = 0; g_burst_dropped = 0;

    Serial.printf("[BTN] count %s for %us -- press ONCE, then do not touch it\n",
                  addr_str, (unsigned)secs);
    scan->setActiveScan(true);
    /* `true` = want duplicates. Without it the controller filters repeats and
     * this function would count exactly what btnraw already counts, which is
     * the thing it exists to go past. */
    scan->setScanCallbacks(&g_count_cb, true);
    scan->getResults(secs * 1000, false);
    scan->setScanCallbacks(nullptr, true);
    scan->clearResults();

    Serial.printf("[BTN] %lu packet(s) in %lu burst(s)\n",
                  (unsigned long)g_burst_pkts, (unsigned long)g_burst_n);
    for(uint8_t k = 0; k < g_burst_n; k++) {
        const CentralBurst* b = &g_burst[k];
        Serial.printf("  burst %u: %lu packet(s) over %lu ms  %s\n",
                      (unsigned)(k + 1), (unsigned long)b->packets,
                      (unsigned long)(b->last_ms - b->first_ms),
                      b->name[0] ? b->name : "(no name)");
    }
    if(g_burst_dropped)
        Serial.printf("  (%lu more burst(s) not shown -- table is %u)\n",
                      (unsigned long)g_burst_dropped, (unsigned)CENTRAL_BURST_MAX);
    if(!g_burst_pkts)
        Serial.println("[BTN] nothing heard -- this remote advertises only while pressed");
    return true;
}

/* A bounded knock on the door.
 *
 * The remote is a transparent-serial bridge (HopeRF HPRDW01): it has write
 * targets and notify sources and no button semantics of its own. Something has
 * to start the conversation, and the vendor handshake is not published.
 *
 * 🔴 This is NOT a search of the space — that space is 2^n and guessing into it
 * is how afternoons disappear. It is a short list of the values these modules
 * conventionally answer to, tried once each. Twenty writes, one verdict:
 * either a notification appears or this approach is done and the answer has to
 * come from watching what TSL actually sends. */
bool ble_central_poke(uint8_t slot) {
    if(slot >= BLE_CENTRAL_MAX_BUTTONS) return false;
    NimBLEClient* c = g_slot[slot].client;
    if(!c || !c->isConnected()) {
        Serial.printf("[BTN] %u: not connected\n", (unsigned)slot);
        return false;
    }

    static const struct { const char* name; uint8_t b[4]; uint8_t len; } TRY[] = {
        {"01",        {0x01},                   1},
        {"FF",        {0xFF},                   1},
        {"00",        {0x00},                   1},
        {"02",        {0x02},                   1},
        {"0100",      {0x01, 0x00},             2},
        {"AA55",      {0xAA, 0x55},             2},
        {"55AA",      {0x55, 0xAA},             2},
        {"A55A",      {0xA5, 0x5A},             2},
        {"FFFF",      {0xFF, 0xFF},             2},
        {"AT",        {0x41, 0x54},             2},
        {"AT\\r\\n",  {0x41, 0x54, 0x0D, 0x0A}, 4},
    };

    /* Collect the write targets once. The table is small; the point is to try
     * every candidate against every door rather than assume which door. */
    NimBLERemoteCharacteristic* w[8];
    int nw = 0;
    for(auto* svc : c->getServices(true)) {
        for(auto* chr : svc->getCharacteristics(true)) {
            if(nw >= (int)(sizeof(w) / sizeof(w[0]))) break;
            if(chr->canWrite() || chr->canWriteNoResponse()) w[nw++] = chr;
        }
    }
    Serial.printf("[BTN] %u: poking %d write target(s) with %d values\n",
                  (unsigned)slot, nw, (int)(sizeof(TRY) / sizeof(TRY[0])));

    for(int t = 0; t < (int)(sizeof(TRY) / sizeof(TRY[0])); t++) {
        for(int k = 0; k < nw; k++) {
            const uint16_t before = ble_central_notify_count();
            const bool sent = w[k]->writeValue(TRY[t].b, TRY[t].len,
                                               w[k]->canWrite());
            delay(500); // give it time to answer
            const uint16_t after = ble_central_notify_count();
            if(after != before) {
                Serial.printf("[BTN] 🔴 HIT: %s -> %s produced %u notification(s)\n",
                              TRY[t].name, w[k]->getUUID().toString().c_str(),
                              (unsigned)(uint16_t)(after - before));
                return true;
            }
            if(!sent)
                Serial.printf("[BTN]   %-8s -> %s  write refused\n", TRY[t].name,
                              w[k]->getUUID().toString().c_str());
        }
    }
    Serial.println("[BTN] no value in the list started it talking");
    Serial.println("[BTN] next step is watching what the real app sends, not more guesses");
    return true;
}

bool ble_central_chars(uint8_t slot) {
    if(slot >= BLE_CENTRAL_MAX_BUTTONS) return false;
    NimBLEClient* c = g_slot[slot].client;
    if(!c || !c->isConnected()) {
        Serial.printf("[BTN] %u: not connected\n", (unsigned)slot);
        return false;
    }
    /* subscribe_all() only ever looked at what NOTIFIES. That was the right
     * first question and it is the wrong second one: a remote that stays silent
     * may be waiting to be written to, and a write target is invisible from
     * here. So print the whole table, properties and all. */
    Serial.printf("[BTN] %u: characteristics\n", (unsigned)slot);
    for(auto* svc : c->getServices(true)) {
        Serial.printf("  service %s\n", svc->getUUID().toString().c_str());
        for(auto* chr : svc->getCharacteristics(true)) {
            char props[24];
            snprintf(props, sizeof(props), "%s%s%s%s%s",
                     chr->canRead() ? "R" : "-",
                     chr->canWrite() ? "W" : "-",
                     chr->canWriteNoResponse() ? "w" : "-",
                     chr->canNotify() ? "N" : "-",
                     chr->canIndicate() ? "I" : "-");
            Serial.printf("    %-38s %s", chr->getUUID().toString().c_str(), props);
            /* The current value of a readable characteristic is free evidence —
             * a key-state byte would show up right here. */
            if(chr->canRead()) {
                NimBLEAttValue v = chr->readValue();
                Serial.print("  ");
                for(size_t k = 0; k < v.length() && k < 20; k++) Serial.printf("%02X", v[k]);
                if(v.length() == 0) Serial.print("(empty)");
            }
            Serial.println();
        }
    }
    Serial.println("[BTN] R=read W=write w=write-no-resp N=notify I=indicate");
    return true;
}

bool ble_central_secure(uint8_t slot) {
    if(slot >= BLE_CENTRAL_MAX_BUTTONS) return false;
    NimBLEClient* c = g_slot[slot].client;
    if(!c || !c->isConnected()) {
        Serial.printf("[BTN] %u: not connected\n", (unsigned)slot);
        return false;
    }
    /* Uses whatever NimBLEDevice::setSecurityAuth() the server role already
     * set — this only starts pairing, it does not change the parameters, so
     * the phone's link is untouched. It does spend one of MAX_BONDS. */
    Serial.printf("[BTN] %u: pairing...\n", (unsigned)slot);
    const bool ok = c->secureConnection();
    Serial.printf("[BTN] %u: %s\n", (unsigned)slot,
                  ok ? "encrypted — press it now and watch for reports"
                     : "pairing refused (so encryption is not what it was waiting for)");
    return ok;
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

    /* The gesture goes with the remote. A half-finished swipe left behind would
     * be measured from its old start point the next time anything connects.
     *
     * 🔴 And the level button has to be let go explicitly. This used to release
     * "button `i`" because the slot WAS the button; now the only button that
     * can be held is 6번, and leaving it down here means the remote walks away
     * mid-press and the module reports STUCK ten seconds later on a device that
     * is no longer there. */
    fsd_j6_init(&g_j6[i]);
    /* 🔴 이 자리에서 지우지 않으면 다음에 묶는 기기가 **남의 증거**로
     * 동작 중처럼 보인다. 슬롯은 재사용된다. */
    g_slot[i].decoded = 0;
    /* 🔴 THIS SLOT's 6번, not every slot's. It used to release the shared
     * struct, so forgetting one remote let go of a hold another was in the
     * middle of. Then wipe the rest: a slot is reused, and the next remote to
     * land here must not inherit a half-finished double press. */
    fsd_btn_report(&g_btns[i], FSD_J6_B6, false, millis());
    slot_buttons_reset(i);
}

int ble_central_add(const char* addr_str) {
    if(!addr_str || !addr_str[0]) return -1;

    /* Already bound? Do not take a second slot — but DO start trying again.
     *
     * 🔴 THIS USED TO RETURN AND NOTHING ELSE, which made the giving-up message
     * a lie. After BLE_CENTRAL_MAX_RETRIES the client stops for good and prints
     * "rebind to retry"; rebinding then hit this line, returned the slot number,
     * and changed nothing. The remote stayed dead until someone thought to
     * forget it first — and the app has no way to show that state at all, so
     * from the phone it looks like a remote that simply never connects.
     *
     * A remote is normally out of retries because it was switched off, which is
     * the ordinary way to use it. Asking to bind it again is exactly the moment
     * to try once more. */
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        if(strcmp(g_slot[i].addr, addr_str) != 0) continue;
        if(g_slot[i].retries >= BLE_CENTRAL_MAX_RETRIES)
            Serial.printf("[BTN] %u retrying %s\n", (unsigned)i, addr_str);
        g_slot[i].retries = 0;
        g_slot[i].last_try_ms = 0; // next tick, not five seconds from now
        return (int)i;
    }

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

uint16_t ble_central_found_total(void) { return g_found_total; }

void ble_central_set_verbose(bool on) { g_verbose = on; }
uint16_t ble_central_notify_count(void) { return g_notify_count; }

uint16_t ble_central_slot_decoded(uint8_t slot) {
    return (slot < BLE_CENTRAL_MAX_BUTTONS) ? g_slot[slot].decoded : 0;
}

/* True: fsd_btn_j6.c is measured against a real remote (블루투스-버튼-조사.md),
 * not a placeholder. It says nothing about whether the BOUND device is one this
 * decoder understands — that is what the per-slot count above answers. */
bool ble_central_decoder_verified(void) { return true; }

uint8_t ble_central_row_events(uint8_t row) {
    const uint8_t btn = (uint8_t)(row / FSD_BTN_EVENTS);
    if(btn >= FSD_BTN_MAX) return 0;
    /* Summed over remotes: the row is what the owner mapped an action to, and
     * it does not matter which one produced it. The per-remote split exists so
     * two of them cannot corrupt each other's timing, not so the screen grows
     * five columns. */
    uint32_t n = 0;
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        switch(row % FSD_BTN_EVENTS) {
        case 0: n += fsd_btn_shorts(&g_btns[i], btn); break;
        case 1: n += fsd_btn_doubles(&g_btns[i], btn); break;
        default: n += fsd_btn_longs(&g_btns[i], btn); break;
        }
    }
    return (n > 0xFFu) ? 0xFFu : (uint8_t)n;
}

/* 🔴 ble_central_set_double() / _double_mask() / the "dbl" NVS key USED TO LIVE
 * HERE and had no caller anywhere in the firmware (red team, 2026-08-19).
 *
 * The live path is ble_central_set_action(), which turns the DOUBLE row on and
 * calls fsd_btn_set_double() itself. Keeping a second store meant two truths
 * about the same thing, and the header already warns that a drifting second
 * truth is invisible from the screen — you cannot see which one the module is
 * actually using. The old "dbl" key is simply left unread.
 */

static void action_save(void) {
    g_prefs.begin("btn", false);
    g_prefs.putUInt("act", g_action_mask);
    g_prefs.end();
}

/* 🔴 THE MASK IS 32 BITS AND THE ROWS MUST FIT IN IT.
 *
 * Thirty rows today. At eleven logical buttons it would be thirty-three, and
 * the `row >= 32u` bound below would start refusing the top rows with nothing
 * in the log — the app would offer a mapping the module quietly ignores. Make
 * it a compile error instead. */
static_assert(FSD_BTN_MAX * FSD_BTN_EVENTS <= 32,
              "the action mask is 32 bits; FSD_BTN_MAX * FSD_BTN_EVENTS no longer fits");

void ble_central_set_action(uint8_t row, bool on) {
    if(row >= FSD_BTN_MAX * FSD_BTN_EVENTS || row >= 32u) return;

    /* 🔴 Rows 0-2 belong to FSD_J6_NONE, which is the decoder's "I could not
     * name this" sentinel — no gesture can ever produce it. Accepting a mapping
     * there burned three of the thirty-two mask bits, left three counters
     * permanently at 00, and printed "[BTN] none short enabled" as if something
     * had been armed. Refuse it: the module should not agree to fire a button
     * that does not exist. */
    if((FsdJ6Btn)(row / FSD_BTN_EVENTS) == FSD_J6_NONE) {
        Serial.printf("[BTN] row %u belongs to no button - ignored\n", (unsigned)row);
        return;
    }
    const uint32_t bit = 1uL << row;
    const uint32_t want = on ? (g_action_mask | bit) : (g_action_mask & ~bit);
    if(want == g_action_mask) return;
    g_action_mask = want;
    action_save();

    /* 🔴 A DOUBLE row is not only a note about intent — switching it on makes
     * the button wait for a twin, and every single press there gets slower.
     * Applying it here is what keeps the mask the ONE truth: a caller cannot
     * set the row and forget the timing, because there is nothing else to set. */
    const uint8_t btn = (uint8_t)(row / FSD_BTN_EVENTS);
    if((row % FSD_BTN_EVENTS) == 1u)
        for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++)
            fsd_btn_set_double(&g_btns[i], btn, on);

    Serial.printf("[BTN] %s %s %s\n", fsd_j6_name((FsdJ6Btn)btn),
                  (row % FSD_BTN_EVENTS) == 0u ? "short"
                      : ((row % FSD_BTN_EVENTS) == 1u ? "double" : "long"),
                  on ? "enabled" : "disabled");
}

uint32_t ble_central_action_mask(void) { return g_action_mask; }

/* Over every logical button. It used to be over slots, back when those were the
 * same thing; with the J6 that would count one button in nine. */
static uint16_t sum_over_buttons(uint16_t (*f)(const FsdButtons*, uint8_t)) {
    uint32_t n = 0;
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++)
        for(uint8_t b = 0; b < FSD_BTN_MAX; b++) n += f(&g_btns[i], b);
    return (n > 0xFFFFu) ? 0xFFFFu : (uint16_t)n;
}
uint16_t ble_central_short_presses(void)  { return sum_over_buttons(fsd_btn_shorts); }
uint16_t ble_central_long_presses(void)   { return sum_over_buttons(fsd_btn_longs); }
uint16_t ble_central_double_presses(void) { return sum_over_buttons(fsd_btn_doubles); }

void ble_central_tick(uint32_t now_ms) {
    if((uint32_t)(now_ms - g_last_tick_ms) < BLE_CENTRAL_TICK_MS) return;
    g_last_tick_ms = now_ms;

    /* A parked scan runs HERE, on loop(), never on the BLE host task. It blocks
     * for seconds; doing that in a characteristic callback would stall the
     * phone's link and look like the module died. */
    if(g_scan_req) {
        const uint8_t secs = g_scan_req;
        g_scan_req = 0;
        /* 🔴 The flag is owned by ble_central_scan() and cleared by onScanEnd()
         * now. Setting it here and clearing it after the call was correct while
         * the call blocked; with an asynchronous scan it would clear the flag
         * the instant the scan STARTED, and everything downstream would think
         * the scan was already over. */
        ble_central_scan(secs);
        return; // let the scan get on with it
    }

    /* Nothing else runs while the radio is scanning. Reconnect attempts in
     * particular: they block for the connect timeout, and doing that mid-scan
     * costs the scan the very seconds it was given. */
    if(g_scanning) return;

    // ── reports that arrived ─────────────────────────────────────────────────
    //
    // Drain the WHOLE ring each tick. Taking one and leaving the rest would put
    // the loss back, just slower.
    for(;;) {
        CentralRep rep;
        bool have = false;
        uint16_t dropped = 0;
        portENTER_CRITICAL(&g_rep_mux);
        if(g_ring_tail != g_ring_head) {
            rep = g_ring[g_ring_tail];
            g_ring_tail = (uint8_t)((g_ring_tail + 1) % CENTRAL_RING);
            have = true;
        }
        dropped = g_ring_dropped;
        g_ring_dropped = 0;
        portEXIT_CRITICAL(&g_rep_mux);

        if(dropped)
            Serial.printf("[BTN] 🔴 %u report(s) dropped — the ring filled\n",
                          (unsigned)dropped);
        if(!have) break;

        const uint8_t i = rep.slot;
        const uint8_t len = rep.len;

        /* slot_of() cannot return an out-of-range index and on_notify() drops
         * anything it does not recognise, so this holds today. It is checked
         * anyway because `i` indexes TWO arrays below and the invariant lives in
         * a different function — the day slot_of() changes, the failure here is
         * memory corruption on the loop task, which is the least debuggable
         * thing this file could produce. */
        if(i >= BLE_CENTRAL_MAX_BUTTONS) continue;

        if(g_verbose) {
            /* The handle is what makes seven report characteristics tellable
             * apart. Without it every key on this remote looks the same. */
            Serial.printf("[BTN] %u h=0x%04X report", (unsigned)i, (unsigned)rep.handle);
            for(uint8_t k = 0; k < len; k++) Serial.printf(" %02X", rep.b[k]);
            Serial.println();
        }
        /* THE GATE, and it is the decoder itself now. A report that is not this
         * device's shape decodes to NONE, so an unknown peripheral's bytes stay
         * a log line — the same protection the old `verified` flag gave, except
         * it comes from recognising the device rather than from a promise. */
        const FsdJ6Out g = fsd_j6_feed(&g_j6[i], rep.b, len, now_ms);
        if(g.edge != FSD_J6_EDGE_NONE) {
            const uint8_t b = (uint8_t)g.btn;
            /* Counted at DECODE, not at event. A level button's press and
             * release are two decodes and one event, and a withheld tap is a
             * decode with no event at all — but both mean "this remote is
             * talking to us", which is what the app is asking. */
            if(g_slot[i].decoded < 0xFFFFu) g_slot[i].decoded++;

            /* Which entry point depends on what the gesture said, and the two
             * are not interchangeable: fsd_button.h refuses a level fed to an
             * event button and a pulse fed to a level one, rather than guessing
             * which was meant. A guess there double-counts a gesture. */
            FsdBtnEvent e = FSD_BTN_EV_NONE;
            switch(g.edge) {
            case FSD_J6_EDGE_PULSE: e = fsd_btn_pulse(&g_btns[i], b, now_ms); break;
            case FSD_J6_EDGE_DOWN:  e = fsd_btn_report(&g_btns[i], b, true, now_ms); break;
            case FSD_J6_EDGE_UP:    e = fsd_btn_report(&g_btns[i], b, false, now_ms); break;
            case FSD_J6_EDGE_NONE:  break; // unreachable; keeps the switch total
            }

            /* The button's NAME, not its number. This line is read while
             * working out which physical key does what, and "6 tap-hold" needs
             * no lookup table where "5" needs the header open. */
            if(e != FSD_BTN_EV_NONE)
                Serial.printf("[BTN] slot%u %s -> %s (%ums)\n", (unsigned)i,
                              fsd_j6_name(g.btn), fsd_btn_event_str(e),
                              (unsigned)fsd_btn_last_hold_ms(&g_btns[i], b));
        }
    }

    /* LONG, DOUBLE and STUCK happen while nothing is being reported.
     *
     * Over REMOTES and then over BUTTONS — both, and they are different counts.
     * BLE_CENTRAL_MAX_BUTTONS is a radio budget; FSD_BTN_MAX is an index space.
     * Getting either bound wrong is quiet in both directions — too low and the
     * top buttons never produce a LONG, too high and it reads past the array —
     * which is why the counts are asserted against each other in fsd_btn_j6.h
     * rather than kept in step by hand. */
    for(uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++)
    for(uint8_t b = 0; b < FSD_BTN_MAX; b++) {
        const FsdBtnEvent e = fsd_btn_tick(&g_btns[i], b, now_ms);
        if(e != FSD_BTN_EV_NONE)
            Serial.printf("[BTN] slot%u %s -> %s\n", (unsigned)i,
                          fsd_j6_name((FsdJ6Btn)b), fsd_btn_event_str(e));
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

        /* 🔴 THIS USED TO GIVE UP FOREVER after five tries, and that made the
         * whole feature unusable (2026-08-19).
         *
         * A remote SLEEPS. That is not a fault, it is how the thing works — it
         * stops advertising, and five attempts over twenty-five seconds is the
         * entire window in which it can be caught. Miss it (the module rebooted
         * while the remote was in a pocket, say) and the button is dead until
         * someone thinks to forget and re-add it. From the phone that looks
         * like a remote that simply never connects, with nothing to click.
         *
         * So the fast lane still backs off — five quick tries then stop
         * hammering — but after that it keeps looking, rarely. One connect
         * attempt a minute is nothing next to a button that never works. */
        const uint32_t wait = (sl->retries >= BLE_CENTRAL_MAX_RETRIES)
                                  ? BLE_CENTRAL_SLOW_RETRY_MS
                                  : BLE_CENTRAL_RETRY_MS;
        if((uint32_t)(now_ms - sl->last_try_ms) < wait) continue;

        sl->last_try_ms = now_ms;
        if(!try_connect(k)) {
            sl->retries++;
            /* Once, at the transition. `>=` here would print every slow retry,
             * forever — a line a minute for a remote sitting in a drawer. */
            if(sl->retries == BLE_CENTRAL_MAX_RETRIES)
                Serial.printf("[BTN] %u %s not answering - slowing to one try a minute\n",
                              (unsigned)k, sl->addr);
        } else {
            sl->retries = 0;
        }
        break; // at most one connect attempt per tick
    }
}

#endif // BLE_SERVER_ENABLED
