/*
 * ble_server.cpp — BLE GATT server (module <-> phone app).
 *
 * See ble_server.h for the wire layout and BLE-GATT-프로토콜.md for rationale.
 *
 * Threading: NimBLE callbacks run on the BLE host task, not loop(). Anything
 * touching FSDState takes g_mux, exactly like web_dashboard.cpp does. Commands
 * are handled inline (they only flip flags); nothing here blocks the BLE task.
 *
 * Safety: this server can put the module into Active mode, which is the same
 * authority the physical button and the web dashboard already have. It cannot
 * transmit CAN by itself — SET_PROFILE is deliberately left unimplemented until
 * the 0x3C2 scroll encoding is confirmed from a real capture.
 */

#include "ble_server.h"

#ifdef BLE_SERVER_ENABLED

#include "../../fsd_logic/fsd_autonomy.h"
#include "../../fsd_logic/fsd_wire.h"
#include "blackbox.h"
#include "ble_owner.h"
#include "camera_store.h"
#include "camera_task.h"
#include "capability.h"
#include "config.h"
#include "mode_switch.h"
#include "prefs.h"
#include <NimBLEDevice.h>
#include <string.h>

// ── UUIDs ────────────────────────────────────────────────────────────────────
// Custom 128-bit base; the 3rd..4th nibble group carries the characteristic id.
#define BLE_UUID_SERVICE "6b1a0000-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_STATE   "6b1a0001-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_COMMAND "6b1a0002-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_RESULT  "6b1a0003-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_CAPAB   "6b1a0005-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_BULK    "6b1a0006-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_UPLOAD  "6b1a0007-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_CAMSTAT "6b1a0008-4b53-4d4f-4432-43414e000001"

#define BLE_STATE_LEN  20u
#define BLE_RESULT_LEN 4u
#define BLE_BULK_HDR   2u   // seq prefix on every bulk frame

// These and fsd_wire.h's must not drift apart. Two sets exist because
// ble_server is the protocol's public face while fsd_wire.h is what the host
// tests and the fixture generator compile against — so the compiler checks that
// the two agree rather than a reviewer having to.
static_assert(BLE_STATE_LEN == FSD_WIRE_STATE_LEN, "State length drifted");
static_assert(BLE_CAMSTAT_LEN == FSD_WIRE_CAMSTAT_LEN, "CamStat length drifted");
static_assert(BLE_RESULT_LEN == FSD_WIRE_RESULT_LEN, "Result length drifted");
static_assert(BLE_PROTO_VERSION == FSD_WIRE_STATE_VERSION, "State version drifted");
static_assert(BLE_CAMSTAT_VERSION == FSD_WIRE_CAMSTAT_VERSION, "CamStat version drifted");

static FSDState     *g_state = nullptr;
static portMUX_TYPE *g_mux   = nullptr;

static NimBLEServer         *g_server   = nullptr;
static NimBLECharacteristic *g_ch_state = nullptr;
static NimBLECharacteristic *g_ch_result = nullptr;
static NimBLECharacteristic *g_ch_cap    = nullptr;
static NimBLECharacteristic *g_ch_bulk   = nullptr;
static NimBLECharacteristic *g_ch_upload = nullptr;
static NimBLECharacteristic *g_ch_camstat = nullptr;

// Whether the radio is advertising, as reported by startAdvertising() -- both
// at init and on every re-advertise after a disconnect. Read by the OTA
// self-test, which must not accept an image whose only control channel is down.
static bool g_adv_ok = false;

static volatile bool g_connected = false;
static volatile bool g_state_subscribed = false;  // client enabled State notifications
static volatile bool g_bulk_subscribed  = false;
static volatile uint16_t g_mtu = 23;              // until the peer negotiates up
static uint32_t      g_last_state_ms = 0;
static uint32_t      g_last_camstat_ms = 0;
// Set by a command handler on the BLE task, consumed by loop(). A single
// word, written true and read-and-cleared in one place, so no lock is needed.
static volatile bool g_prefs_dirty = false;

// ── Bulk transfer state ──────────────────────────────────────────────────────
// Owned by loop() via ble_server_tick(); the BLE task only sets g_bulk_active
// false (STOP) or arms a new transfer, both single-word writes.
static volatile bool g_bulk_active = false;
static bool     g_bulk_json   = false;
// ── Active-mode lease ────────────────────────────────────────────────────────
// This project's rule is "compute on the phone, act on the module": with no
// phone there is nothing to act on, so holding TX authority buys nothing and
// costs safety. When the link drops we hand Active back after a grace period.
//
// Only Active that WE granted is revoked — the button and the web dashboard are
// independent authorities and a BLE disconnect must not override them.
static volatile bool     g_active_by_ble = false;
static volatile uint32_t g_link_down_ms  = 0;  // 0 = link up or nothing to undo

// 🔴 "somebody is connected" and "the OWNER is connected" are different facts,
// and the revoke path needs the second one.
//
// It used to use g_connected, and onConnect() cleared g_link_down_ms outright.
// onConnect fires on the bare GAP connection — before pairing, before identity
// resolution — so ANY radio in range could cancel a pending revoke just by
// connecting, and then hold the revoke off indefinitely simply by staying
// connected. The stranger cannot send commands (ble_owner_allows blocks those),
// but it does not need to: the damage is that the owner's Active permission is
// never taken back, so a car with nobody in it keeps a live transmitter. It also
// occupies the single connection slot, so the owner cannot reconnect to undo it.
//
// Set only where the peer's identity is actually known to be the owner's, and
// cleared on disconnect. Failing closed here means the timer keeps running when
// we are unsure — which ends in Listen-Only, the direction we want to fall.
static volatile bool     g_owner_present  = false;
// SET_MODE arrives on the BLE task but has to be applied from loop(), because
// applying it means switching a CAN controller. Single word each, written by
// the BLE task and read-and-cleared in one place, so no lock is needed.
static volatile bool     g_mode_req_pending = false;
static volatile uint8_t  g_mode_req         = 0;
// Same deal for the recorder. blackbox_set_enabled() allocates a ~114 KB ring,
// which must not happen on the BLE host task, and the heap guard can refuse --
// so like SET_MODE these are applied from loop() and answered with what really
// happened rather than with what was asked for.
static volatile bool     g_bb_req_pending   = false;
static volatile uint8_t  g_bb_req           = 0;
static volatile bool     g_bb_mark_pending  = false;
// Declared size of the camera.bin currently being uploaded (0 = none).
static volatile uint32_t g_upload_expect = 0;
static char     g_bulk_name[40] = {};
static size_t   g_bulk_offset = 0;
static uint16_t g_bulk_seq    = 0;

// ── State serialisation (20 B, little-endian) ────────────────────────────────
// Layout is fixed; the app parses by offset. Bump BLE_PROTO_VERSION on change.
// GATHERS ONLY. Every clamp, scale and byte position lives in
// fsd_logic/fsd_wire.c, which is pure C and host-tested — this file cannot be
// compiled on a host, so anything left here would be untestable on both sides
// of the link. `rx_fps` comes in as an argument because its counter lives here.
static void ble_pack_state(uint8_t *out, uint16_t rx_fps) {
    FSDState s;
    portENTER_CRITICAL(g_mux);
    s = *g_state;
    portEXIT_CRITICAL(g_mux);

    FsdWireState w;
    memset(&w, 0, sizeof(w));
    // rx_count is the same wiring-sanity signal the red LED uses.
    w.rx_seen          = (s.rx_count > 0);
    w.ota_in_progress  = s.tesla_ota_in_progress;
    w.blinker_left     = s.ui_left_blinker;
    w.blinker_right    = s.ui_right_blinker;
    w.brake_applied    = s.driver_brake_applied;
    // The ring, not the wish: blackbox_is_enabled() is false when the heap
    // guard refused, which is exactly the case the operator must not miss.
    w.blackbox_recording = blackbox_is_enabled();
    w.op_mode          = (uint8_t)s.op_mode;
    w.hw_version       = (uint8_t)s.hw_version;
    w.speed_profile    = s.speed_profile;
    w.ap_state         = s.das_ap_state;
    w.speed_kph        = s.vehicle_speed_kph;
    w.soc_percent      = s.soc_percent;
    w.gear             = s.di_gear;
    w.speed_limit_seen = s.speed_limit_seen;
    w.speed_limit_kph  = s.speed_limit_kph;
    w.rx_fps           = rx_fps;
    w.crc_err_count    = s.crc_err_count;
    w.uptime_s         = millis() / 1000u;

    fsd_wire_pack_state(&w, out);
}

// ── camera / autonomy status ─────────────────────────────────────────────────
// v2: the tracker, GPS layer and policy are instantiated in camera_task.cpp, so
// the fields v1 deliberately left out now exist. See ble_server.h for the byte
// map and for the one thing that is still structurally absent.
//
// Every value here is OBSERVED. The policy decision is published, never acted
// on — there is no path in this firmware from a decision to a CAN frame.
static void ble_pack_camstat(uint8_t *out, uint32_t now_ms) {
    FSDState s;
    portENTER_CRITICAL(g_mux);
    s = *g_state;
    portEXIT_CRITICAL(g_mux);

    FsdSupVerdict v = fsd_supervised_drive_why(&s, now_ms);

    // Wait 0 because a status field is never worth blocking for: a missed
    // borrow reports "no database" for one notify and the next one is 1 s away.
    //
    // Note this packer runs on the LOOP task, not the BLE task — both of its
    // callers (ble_server_init from setup(), ble_server_tick from loop()) are
    // there. That is also what makes camera_task's lock-free accessors below
    // safe to read from here.
    const FsdCamDb *db = camera_store_db_acquire(0);
    uint32_t built = db ? db->built_at : 0u;
    bool have_db = (db != nullptr);
    if (db) camera_store_db_release();

    // GATHERS ONLY — the layout is in fsd_logic/fsd_wire.c. See ble_pack_state.
    FsdWireCamStat w;
    memset(&w, 0, sizeof(w));
    w.autonomy_enabled = s.autonomy_enabled;
    w.supervised_ok    = (v == FSD_SUP_OK);
    w.db_loaded        = have_db;
    w.autonomy_allows  = fsd_autonomy_allows(&s, now_ms);
    w.pol_suspended    = camera_task_pol_suspended();
    w.learning_dirty   = camera_task_learning_dirty();
    w.profile_fresh    = camera_task_profile_fresh();
    w.save_failing     = camera_task_save_failing();
    w.sup_verdict      = (uint8_t)v;
    w.op_mode          = (uint8_t)s.op_mode;
    w.camera_count     = camera_store_count();
    w.built_at         = built;
    w.gps_verdict      = camera_task_gps_verdict();
    w.pol_phase        = camera_task_pol_phase();
    w.pol_action       = camera_task_pol_action();
    w.pol_target       = camera_task_pol_target();
    w.nearest_m        = camera_task_nearest_m();
    w.gps_accuracy_raw = camera_task_gps_accuracy_raw();
    w.raw_profile      = camera_task_raw_profile();
    w.learned_count    = camera_task_learned_count();
    w.scan_full_count  = camera_task_scan_full_count();

    fsd_wire_pack_camstat(&w, out);
}

// ── frames/s, derived the same way the dashboard does ────────────────────────
static uint32_t g_fps_last_ms = 0;
static uint32_t g_fps_last_rx = 0;
static uint16_t g_fps         = 0;

static void ble_update_fps(uint32_t now) {
    if (now - g_fps_last_ms < 1000u) return;
    portENTER_CRITICAL(g_mux);
    uint32_t rx = g_state->rx_count;
    portEXIT_CRITICAL(g_mux);
    uint32_t d = (rx >= g_fps_last_rx) ? (rx - g_fps_last_rx) : 0u;
    g_fps = (d > 0xFFFFu) ? 0xFFFFu : (uint16_t)d;
    g_fps_last_rx = rx;
    g_fps_last_ms = now;
}

static void ble_send_result(uint8_t cmd, uint8_t res, uint16_t extra) {
    if (!g_ch_result) return;
    uint8_t b[BLE_RESULT_LEN];
    fsd_wire_pack_result(cmd, res, extra, b);
    g_ch_result->setValue(b, sizeof(b));
    g_ch_result->indicate();  // ACKed: a lost command result would desync the app
}

// ── Bulk download ────────────────────────────────────────────────────────────
// ATT overhead is 3 bytes; our own seq prefix takes 2 more.
static size_t ble_bulk_payload_cap() {
    uint16_t mtu = g_mtu;
    size_t cap = (mtu > (3u + BLE_BULK_HDR)) ? (size_t)(mtu - 3u - BLE_BULK_HDR) : 1u;
    return cap > BLE_BULK_MAX_PAYLOAD ? BLE_BULK_MAX_PAYLOAD : cap;
}

// Arms a transfer of the newest capture. Runs on the BLE task, so it only sets
// up state — the actual pumping happens in ble_server_tick().
static uint8_t ble_bulk_start(bool json) {
    if (g_bulk_active) return BLE_RES_BUSY;
    if (!g_bulk_subscribed) return BLE_RES_REJECTED;  // nobody would hear it
    if (!blackbox_latest_name(g_bulk_name, sizeof(g_bulk_name))) return BLE_RES_NOT_FOUND;

    size_t total = 0;
    if (!blackbox_file_size(g_bulk_name, json, &total) || total == 0) {
        return BLE_RES_NOT_FOUND;
    }

    // seq 0 is the header: total size then the capture name.
    uint8_t hdr[BLE_BULK_HDR + 4 + sizeof(g_bulk_name)];
    hdr[0] = 0; hdr[1] = 0;
    hdr[2] = (uint8_t)(total & 0xFFu);
    hdr[3] = (uint8_t)((total >> 8) & 0xFFu);
    hdr[4] = (uint8_t)((total >> 16) & 0xFFu);
    hdr[5] = (uint8_t)((total >> 24) & 0xFFu);
    size_t nlen = strnlen(g_bulk_name, sizeof(g_bulk_name));
    size_t room = ble_bulk_payload_cap() > 4u ? ble_bulk_payload_cap() - 4u : 0u;
    if (nlen > room) nlen = room;  // a tiny MTU truncates the name, not the data
    memcpy(hdr + BLE_BULK_HDR + 4, g_bulk_name, nlen);

    // If this one notify is dropped the app never learns the total size. It can
    // detect that (no seq 0) and just re-issue DUMP_START — cheaper than
    // carrying a retry path for a single frame sent on an idle queue.
    g_ch_bulk->setValue(hdr, BLE_BULK_HDR + 4 + nlen);
    g_ch_bulk->notify();

    g_bulk_json   = json;
    g_bulk_offset = 0;
    g_bulk_seq    = 1;
    g_bulk_active = true;
    Serial.printf("[BLE] bulk start %s.%s (%u B)\n", g_bulk_name,
                  json ? "json" : "log", (unsigned)total);
    return BLE_RES_OK;
}

// Push a few chunks per loop(). Stops early when notify() reports the queue is
// full — the next tick resumes at the same offset, so nothing is lost.
static void ble_bulk_pump() {
    if (!g_bulk_active) return;
    if (!g_connected || !g_bulk_subscribed) { g_bulk_active = false; return; }

    uint8_t frame[BLE_BULK_HDR + BLE_BULK_MAX_PAYLOAD];
    for (unsigned i = 0; i < BLE_BULK_CHUNKS_PER_TICK && g_bulk_active; i++) {
        size_t n = blackbox_read_chunk(g_bulk_name, g_bulk_json, g_bulk_offset,
                                       frame + BLE_BULK_HDR, ble_bulk_payload_cap());
        frame[0] = (uint8_t)(g_bulk_seq & 0xFFu);
        frame[1] = (uint8_t)(g_bulk_seq >> 8);
        g_ch_bulk->setValue(frame, BLE_BULK_HDR + n);
        if (!g_ch_bulk->notify()) return;  // queue full — retry this offset later

        g_bulk_offset += n;
        g_bulk_seq++;
        if (n == 0) {  // empty payload = EOF
            g_bulk_active = false;
            Serial.printf("[BLE] bulk done (%u B)\n", (unsigned)g_bulk_offset);
            ble_send_result(BLE_CMD_DUMP_START, BLE_RES_OK, (uint16_t)(g_bulk_seq));
        }
    }
}

// Apply a SET_MODE the BLE task parked for us, and answer with the truth.
// Runs from loop(), which is the only task allowed to touch the CAN driver.
static void ble_apply_mode_request(void) {
    if (!g_mode_req_pending) return;
    g_mode_req_pending = false;

    OpMode m = (OpMode)g_mode_req;

    // 🔴 The requester has to still be there.
    //
    // Parking the request for loop() (PR #34) opened a window that did not
    // exist when SET_MODE was handled inline: the phone can disconnect between
    // the park and the apply. onDisconnect only arms the 30 s recovery when
    // g_active_by_ble is already true, and it is not true yet -- so applying
    // Active here granted CAN transmit to a phone that had left, with nothing
    // left to take it back. Permanently.
    if (!g_connected) {
        Serial.printf("[BLE] mode request dropped — phone left before it was applied\n");
        return;
    }

    bool ok = mode_apply(m);

    // Track the lease only when the switch really happened. Claiming an Active
    // we never got would arm the grace timer to "revoke" something that was
    // never granted.
    if (ok) g_active_by_ble = (m == OpMode_Active);

    // The link can also drop DURING mode_apply(), which talks to an MCP2515
    // over SPI and is not instant. onDisconnect would have run while
    // g_active_by_ble was still false, so arm the timer here instead of
    // leaving the grant unowned.
    if (ok && m == OpMode_Active && !g_connected && g_link_down_ms == 0) {
        uint32_t t = millis();
        g_link_down_ms = t ? t : 1u;
        Serial.println("[BLE] phone left while the mode was being applied — recovery armed");
    }

    Serial.printf("[BLE] mode -> %s%s\n",
                  m == OpMode_Active ? "ACTIVE" : "LISTEN-ONLY",
                  ok ? "" : " — FAILED, controller did not switch");
    ble_send_result(BLE_CMD_SET_MODE, ok ? BLE_RES_OK : BLE_RES_REJECTED,
                    (uint16_t)mode_current());
}

// Apply a recorder request parked by the BLE task. Runs from loop(), which is
// where a 114 KB allocation belongs.
static void ble_apply_blackbox_request(void) {
    if (g_bb_req_pending) {
        g_bb_req_pending = false;
        bool want = (g_bb_req != 0);

        blackbox_set_enabled(want);

        // Report what IS, not what was asked. The heap guard can refuse an
        // enable, and a phone told "OK" would then wait for captures that are
        // never going to exist.
        bool now_on = blackbox_is_enabled();
        ble_send_result(BLE_CMD_BB_ENABLE,
                        now_on == want ? BLE_RES_OK : BLE_RES_REJECTED,
                        now_on ? 1u : 0u);
        g_prefs_dirty = true;   // survive the reboot; loop() persists it
    }

    if (g_bb_mark_pending) {
        g_bb_mark_pending = false;
        uint32_t before = blackbox_capture_count();
        blackbox_mark(millis());
        // blackbox_mark() can be suppressed by the event cooldown, and silently
        // doing nothing is exactly what the operator must not be told at the one
        // moment that matters. The count is the honest answer.
        ble_send_result(BLE_CMD_BB_MARK, BLE_RES_OK, (uint16_t)(before + 1u));
    }
}

// Keep the Capability characteristic current. It used to be written once, in
// ble_server_init(), before any frame had arrived -- so the probe's answer to
// "which bus is which" never reached anybody, and CAP_RECHECK re-ran a probe
// whose result went nowhere.
static void ble_refresh_capability(void) {
    static bool s_was_running = true;   // force one refresh after boot
    bool running = capability_running();
    if (running || !s_was_running) { s_was_running = running; return; }
    s_was_running = running;
    if (g_ch_cap) {
        g_ch_cap->setValue(capability_status_json().c_str());
        Serial.println("[CAP] listen window closed — Capability updated");
    }
}

// Hand Active back once the phone has been gone for BLE_ACTIVE_GRACE_MS.
// Runs from loop(), so g_state is touched on the same task as everything else.
static void ble_revoke_active_if_stale(uint32_t now_ms) {
    // g_owner_present, not g_connected: a stranger holding the link open is not
    // a reason to leave the owner's transmit permission standing. See the flag.
    if (g_owner_present || g_link_down_ms == 0) return;
    if ((uint32_t)(now_ms - g_link_down_ms) < BLE_ACTIVE_GRACE_MS) return;

    // 🔴 The bookkeeping is cleared AFTER the switch succeeds, not before.
    //
    // It used to be cleared here, first, and mode_apply()'s return value was
    // discarded. So a revoke that failed — an SPI glitch on the MCP2515, a TWAI
    // reinstall that did not take — left op_mode lowered, the transmitter still
    // physically enabled, and no way back: g_link_down_ms was already 0, so this
    // function was never entered again. One dropped return value and the module
    // sits there claiming Listen-Only with an open transmitter, which is exactly
    // the claim SECURITY.md makes about the hardware register.
    portENTER_CRITICAL(g_mux);
    bool was_active = (g_state->op_mode == OpMode_Active);
    // Not Listen-Only unconditionally any more. The phone leaving withdraws the
    // GENERAL transmit permission it granted, but it is not a reason to stop
    // responding to speed cameras — a phone left at home is the exact case this
    // module exists to cover. The floor grants no general TX either way, and
    // the camera path still has to see a person driving.
    OpMode floor = fsd_autonomy_floor(g_state);
    portEXIT_CRITICAL(g_mux);

    // 🔴 Through mode_apply(), not by writing op_mode here. This path used to
    // lower the variable and leave the controller wherever it was — so a module
    // that had been opened with the button ended up reporting Listen-Only while
    // its transmitter was still enabled. Listen-Only's safety claim is that it
    // is a hardware register; a Listen-Only that is only a variable is not that.
    // Called unconditionally, not only when was_active. mode_apply(floor) is
    // idempotent and only ever narrows permission here, and the retry case needs
    // it: mode_apply() lowers op_mode BEFORE closing the register on the way
    // down, so a failed attempt leaves op_mode already at the floor — was_active
    // would read false on the next tick and the retry would never fire.
    if (!mode_apply(floor)) {
        // Keep g_link_down_ms and g_active_by_ble as they are: they are the
        // reason this function runs, and the next tick will try again. Rate-limit
        // the log so a persistently stuck controller does not drown the console.
        static uint32_t s_warn_ms = 0;
        if (!s_warn_ms || (uint32_t)(now_ms - s_warn_ms) >= 5000u) {
            s_warn_ms = now_ms;
            Serial.println("[BLE] 회수 실패 — 송신기가 아직 열려 있을 수 있다. 계속 재시도한다");
        }
        return;
    }

    g_link_down_ms  = 0;
    g_active_by_ble = false;

    if (was_active) {
        Serial.printf("[BLE] no phone for %u s — falling back to %s\n",
                      (unsigned)(BLE_ACTIVE_GRACE_MS / 1000u),
                      floor == OpMode_Autonomous ? "Autonomous" : "Listen-Only");
    }
}

// ── camera.bin upload (phone -> module) ─────────────────────────────────────
// Writes land on the BLE task. Each chunk is a bounded LittleFS append, which
// is fast enough not to stall the stack; nothing here waits on loop().
class UploadCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo &info) override {
        std::string v = ch->getValue();
        if (v.size() < BLE_UPLOAD_HDR) return;
        if (!info.isEncrypted()) return;  // same bar as Command

        // Same gate as CommandCB. A camera database is what the judgement core
        // reads to decide when to slow the car down; a stranger must not get to
        // choose its contents.
        if (!ble_owner_allows(info.getIdAddress().getType(),
                              info.getIdAddress().getVal())) {
            ble_send_result(BLE_CMD_UPLOAD_ABORT, BLE_RES_NOT_OWNER, 0);
            return;
        }

        uint16_t seq = (uint8_t)v[0] | ((uint16_t)(uint8_t)v[1] << 8);
        const uint8_t *body = (const uint8_t *)v.data() + BLE_UPLOAD_HDR;
        size_t n = v.size() - BLE_UPLOAD_HDR;

        uint8_t res;
        if (seq == 0) {
            // Header frame: total size, so the module knows when it is done.
            if (n < 4) { ble_send_result(BLE_CMD_UPLOAD_ABORT, BLE_RES_REJECTED, 0); return; }
            uint32_t total = (uint32_t)body[0] | ((uint32_t)body[1] << 8) |
                             ((uint32_t)body[2] << 16) | ((uint32_t)body[3] << 24);
            res = camera_store_upload_begin(total);
            // Remember the declared size — completion is implicit, so without
            // this the transfer would never be finalised.
            g_upload_expect = (res == CAM_UP_OK) ? total : 0;
            ble_send_result(BLE_CMD_UPLOAD_ABORT, res == CAM_UP_OK ? BLE_RES_OK : BLE_RES_REJECTED, res);
            return;
        }

        res = camera_store_upload_chunk(seq, body, n);
        if (res != CAM_UP_OK) {
            // Report once and stop; the app restarts rather than sending into
            // a transfer that is already broken.
            camera_store_upload_abort();
            ble_send_result(BLE_CMD_UPLOAD_ABORT, BLE_RES_REJECTED, res);
            return;
        }
        // Completion is implicit — the declared byte count arriving ends it.
        if (camera_store_upload_progress() >= g_upload_expect && g_upload_expect > 0) {
            uint8_t fin = camera_store_upload_end();
            g_upload_expect = 0;
            ble_send_result(BLE_CMD_UPLOAD_ABORT,
                            fin == CAM_UP_OK ? BLE_RES_OK : BLE_RES_REJECTED, fin);
        }
    }
};

// ── Command handling ─────────────────────────────────────────────────────────
class CommandCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo &info) override {
        std::string v = ch->getValue();
        if (v.empty()) return;

        // Only a bonded peer may drive the module. The characteristic is
        // declared WRITE_ENC so the ATT layer should already have refused an
        // unencrypted write, but re-check: this path can enable CAN
        // transmission, and the declaration and this check have disagreed
        // before (WRITE_AUTHEN made the ATT layer stricter than this branch,
        // and the difference was invisible until it was traced by hand).
        if (!info.isEncrypted()) {
            ble_send_result(v[0], BLE_RES_REJECTED, 0);
            return;
        }

        // Encrypted says "nobody is listening in". It does not say "you are the
        // phone this module was set up with" -- this board has no display or
        // keypad, so Just Works is the only pairing available and anyone in
        // radio range of a parked car can complete it. Without this line they
        // could then send SET_MODE(Active) and open CAN transmit.
        if (!ble_owner_allows(info.getIdAddress().getType(),
                              info.getIdAddress().getVal())) {
            ble_send_result(v[0], BLE_RES_NOT_OWNER, 0);
            return;
        }

        const uint8_t cmd = (uint8_t)v[0];
        const uint8_t arg = (v.size() > 1) ? (uint8_t)v[1] : 0u;

        switch (cmd) {
        case BLE_CMD_PING:
            ble_send_result(cmd, BLE_RES_OK, BLE_PROTO_VERSION);
            break;

        case BLE_CMD_SET_MODE: {
            // 🔴 Deliberately NOT answered here.
            //
            // This used to write g_state->op_mode and reply OK on the spot,
            // without touching the CAN controllers -- so the app read "Active"
            // while the hardware listen-only register still refused every
            // transmit. The acknowledgement was a claim nobody had checked.
            //
            // It cannot be checked from here either: this runs on the BLE host
            // task, and switching the driver means talking to an MCP2515 over
            // the same SPI bus loop() is polling. So the request is handed to
            // loop(), which applies it and answers with what actually happened.
            // One slot, so a second request before loop() drained the first
            // would silently replace it -- and both waiters would then match
            // the single answer, so a Listen-Only request could be reported
            // successful by an Active one. Refuse instead; the app retries.
            if (g_mode_req_pending) {
                ble_send_result(cmd, BLE_RES_BUSY, (uint16_t)mode_current());
                break;
            }
            g_mode_req         = arg ? OpMode_Active : OpMode_ListenOnly;
            g_mode_req_pending = true;
            break;
        }

        case BLE_CMD_CAP_RECHECK:
            capability_start(millis());
            ble_send_result(cmd, BLE_RES_OK, 0);
            break;

        case BLE_CMD_BB_ENABLE:
            // Deferred: the enable path allocates the ring.
            if (g_bb_req_pending) { ble_send_result(cmd, BLE_RES_BUSY, 0); break; }
            g_bb_req         = arg ? 1u : 0u;
            g_bb_req_pending = true;
            break;

        case BLE_CMD_BB_MARK:
            // Cheap, but it snapshots FSDState and arms a flush, so it belongs
            // on the same task as the rest of the recorder.
            if (!blackbox_is_enabled()) {
                ble_send_result(cmd, BLE_RES_REJECTED, 0);   // nothing is recording
                break;
            }
            g_bb_mark_pending = true;
            break;

        case BLE_CMD_SET_AUTONOMY: {
            bool on = (arg != 0);
            portENTER_CRITICAL(g_mux);
            g_state->autonomy_enabled = on;
            OpMode cur   = g_state->op_mode;
            OpMode floor = fsd_autonomy_floor(g_state);
            portEXIT_CRITICAL(g_mux);

            // 🔴 op_mode is NOT written here. mode_switch.h calls itself "the one
            // way to change op_mode" and "LOOP TASK ONLY — a BLE callback must
            // NOT call this", and this was the one place still doing it directly.
            //
            // It looked harmless only because fsd_mode_opens_hw_tx(Autonomous) is
            // false, so Autonomous and Listen-Only map to the same register
            // state. The day Autonomous opens the register for a scroll detent,
            // this line would drop Autonomous -> Listen-Only while leaving the
            // transmitter enabled — the software saying "Listen-Only" over an
            // open transmitter, which is the exact defect PR #34 removed.
            //
            // Park it for loop() the same way SET_MODE does. If the phone leaves
            // before it is applied the request is dropped, which is safe here:
            // autonomy_enabled is already false, and fsd_autonomy_allows() checks
            // that flag, so the camera path is shut regardless of what op_mode
            // still says. The floor is recomputed at the next boot anyway.
            if ((cur == OpMode_Autonomous || cur == OpMode_ListenOnly) && cur != floor) {
                g_mode_req         = (uint8_t)floor;
                g_mode_req_pending = true;
            }
            // Persist from loop(), not here. This file's contract is that
            // nothing blocks the BLE host task, and an NVS commit is a flash
            // erase-write that can stall for tens of milliseconds.
            g_prefs_dirty = true;
            Serial.printf("[BLE] autonomy -> %s\n", on ? "ON" : "OFF");
            // The floor we asked for, not a snapshot of op_mode: the switch now
            // happens on loop(), so op_mode has not moved yet at this point.
            // State notifications carry the truth once it has.
            ble_send_result(cmd, BLE_RES_OK, (uint16_t)floor);
            break;
        }

        case BLE_CMD_SET_PROFILE:
        case BLE_CMD_PROFILE_STEP:
            // Deliberately not implemented yet. Scroll emulation injects
            // 0x3C2, and the exact value/timing must come from a real capture
            // (S3XY reference) before anything is transmitted — an Intel HW3
            // emergency-brake incident is on record for this path.
            // fsd_logic/fsd_speed_profile.c holds the converged state machine;
            // it stays gated on FsdSpEncoding.verified until that capture.
            ble_send_result(cmd, BLE_RES_UNSUPPORTED, 0);
            break;

        case BLE_CMD_DUMP_START:
            ble_send_result(cmd, ble_bulk_start(arg != 0), 0);
            break;

        case BLE_CMD_DUMP_STOP:
            g_bulk_active = false;
            ble_send_result(cmd, BLE_RES_OK, 0);
            break;

        case BLE_CMD_UPLOAD_ABORT:
            camera_store_upload_abort();
            g_upload_expect = 0;
            ble_send_result(cmd, BLE_RES_OK, 0);
            break;

        default:
            ble_send_result(cmd, BLE_RES_UNSUPPORTED, 0);
            break;
        }
    }
};

// Tracks whether anyone actually wants State notifications, so the 5 Hz packer
// can be skipped entirely when nothing is listening. NimBLE 2.x dropped
// getSubscribedCount(), so we follow the subscribe callback instead.
class StateCB : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t sub) override {
        g_state_subscribed = (sub != 0);
        Serial.printf("[BLE] state notify %s\n", sub ? "subscribed" : "unsubscribed");
    }
};

class BulkCB : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t sub) override {
        g_bulk_subscribed = (sub != 0);
        if (!sub) g_bulk_active = false;  // no listener -> stop pumping
        Serial.printf("[BLE] bulk notify %s\n", sub ? "subscribed" : "unsubscribed");
    }
};

class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
        g_connected = true;
        // 🔴 The pending revoke is NOT cancelled here. At this point we know a
        // radio connected and nothing else — not who, not whether it can pair.
        // Cancelling on the bare connection let any passer-by keep a departed
        // owner's Active permission alive. The cancel moved to
        // onAuthenticationComplete, where the identity address is resolved.
        Serial.println("[BLE] client connected");
    }
    // Fires when pairing completes and when an existing bond re-encrypts, which
    // is where the peer's identity address becomes known. Runs on the BLE host
    // task, so this only updates RAM — the NVS write is queued for loop().
    void onAuthenticationComplete(NimBLEConnInfo &info) override {
        if (!info.isBonded()) return;
        ble_owner_on_bond(info.getIdAddress().getType(),
                          info.getIdAddress().getVal());

        // Now — and only now — we can say whether the owner is back. A bonded
        // phone re-encrypting lands here too, which is exactly the "owner
        // reconnected within the grace window" case the cancel is meant for.
        if (ble_owner_allows(info.getIdAddress().getType(),
                             info.getIdAddress().getVal())) {
            g_owner_present = true;
            g_link_down_ms  = 0;   // owner is back in time — cancel the revoke
        }
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        g_connected = false;
        g_owner_present = false;  // re-established only by a resolved owner bond
        // A mode change asked for by a phone that is now gone must not be
        // applied. See ble_apply_mode_request() for the window this closes.
        g_mode_req_pending = false;
        g_state_subscribed = false;  // subscriptions die with the link
        g_bulk_subscribed  = false;
        g_bulk_active      = false;  // a half-sent capture is not resumable
        camera_store_upload_abort(); // half-written database is worse than none
        g_upload_expect    = 0;
        g_mtu              = 23;     // next peer renegotiates from the default
        // Start the grace timer only if there is something to take back. millis()
        // can return 0 exactly once at boot, which would read as "no timer" — so
        // clamp to 1; a 1 ms skew on a 30 s window does not matter.
        if (g_active_by_ble) {
            uint32_t t = millis();
            g_link_down_ms = t ? t : 1u;
        }
        // 🔴 Keep the result here too. g_adv_ok used to be written once, at
        // init, so a radio that came up and then failed to advertise again
        // after a disconnect still reported healthy forever -- and the OTA
        // self-test would accept an image whose only control channel was down.
        // "It worked once" is not a health signal.
        g_adv_ok = NimBLEDevice::startAdvertising();
        Serial.printf("[BLE] client disconnected — %s\n",
                      g_adv_ok ? "advertising again"
                               : "ADVERTISING FAILED, the module is not visible");
    }
    void onMTUChange(uint16_t mtu, NimBLEConnInfo &) override {
        g_mtu = mtu;  // bulk chunk size follows this
        Serial.printf("[BLE] MTU %u\n", mtu);
    }
};

static CommandCB g_cmd_cb;
static StateCB   g_state_cb;
static ServerCB  g_srv_cb;

// ── Public API ───────────────────────────────────────────────────────────────
void ble_server_init(FSDState *state, portMUX_TYPE *state_mux) {
    g_state = state;
    g_mux   = state_mux;

    // Name carries the MAC tail only — never the VIN or anything car-identifying.
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char name[20];
    snprintf(name, sizeof(name), "T2CAN-%02X%02X", mac[4], mac[5]);

    NimBLEDevice::init(name);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    // Bonding + Secure Connections, MITM deliberately OFF.
    //
    // The MITM bit used to be set here, and it made the module unusable: this
    // board has no display and no keypad, so NimBLE's default IO capability is
    // BLE_HS_IO_NO_INPUT_OUTPUT. That row of the Secure Connections pairing
    // table (ble_sm_sc.c, ble_sm_sc_resp_ioa[3]) is IOACT_NONE for every peer
    // capability, which selects Just Works, which never sets
    // BLE_SM_PROC_F_AUTHENTICATED. Asking for MITM with no way to prove it
    // does not get you MITM; it gets you an unauthenticated link that the
    // Command characteristic then refuses (see below).
    //
    // Getting real MITM back would need a passkey the operator can read off
    // the device. A hardcoded one is theatre in a public repository, and a
    // generated one has to be displayed somewhere. Owner's decision
    // (2026-08-13): pair without a code. The link is still encrypted and still
    // bonded, so only a phone that has paired can drive the module; what is
    // given up is protection against an attacker who is beside the car during
    // the one pairing exchange.
    NimBLEDevice::setSecurityAuth(true, false, true);

    g_server = NimBLEDevice::createServer();
    g_server->setCallbacks(&g_srv_cb);

    NimBLEService *svc = g_server->createService(BLE_UUID_SERVICE);

    g_ch_state = svc->createCharacteristic(
        BLE_UUID_STATE, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    g_ch_state->setCallbacks(&g_state_cb);

    // WRITE_ENC, not WRITE_AUTHEN. AUTHEN requires sec_state.authenticated,
    // which Just Works pairing never produces (ble_att_svr.c: "if (authen &&
    // !sec_state.authenticated) -> BLE_ATT_ERR_INSUFFICIENT_AUTHEN"). With
    // AUTHEN declared here, every command was rejected inside the ATT layer
    // before CommandCB::onWrite ran, so the module answered nothing at all --
    // status kept streaming and the phone just timed out. ENC requires only
    // sec_state.encrypted, which bonded Just Works does give us.
    NimBLECharacteristic *ch_cmd = svc->createCharacteristic(
        BLE_UUID_COMMAND, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC);
    ch_cmd->setCallbacks(&g_cmd_cb);

    g_ch_result = svc->createCharacteristic(
        BLE_UUID_RESULT, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::INDICATE);

    // Capability verdicts: read once on connect so the app can grey out features
    // this tap cannot do (e.g. no 0x3C2 -> no scroll-based profile control).
    g_ch_cap = svc->createCharacteristic(BLE_UUID_CAPAB, NIMBLE_PROPERTY::READ);
    g_ch_cap->setValue(capability_status_json().c_str());

    // Bulk: notify-only. Unencrypted like the rest — a capture is diagnostic
    // data, and the pairing requirement already gates the link.
    g_ch_bulk = svc->createCharacteristic(BLE_UUID_BULK, NIMBLE_PROPERTY::NOTIFY);
    g_ch_bulk->setCallbacks(new BulkCB());

    // Upload: write-only, no response. The phone pushes camera.bin here.
    g_ch_upload = svc->createCharacteristic(
        BLE_UUID_UPLOAD, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    g_ch_upload->setCallbacks(new UploadCB());

    // Camera / autonomy status. Readable so the app can ask once on connect,
    // and notified at 1 Hz because the supervision verdict changes with the
    // gear and the belt.
    g_ch_camstat = svc->createCharacteristic(
        BLE_UUID_CAMSTAT, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    // Seed it, the way Capability is seeded. Without this the first READ before
    // the first notify returns an empty value, and a client that reads once on
    // connect sees a zero-length payload rather than version 2.
    {
        uint8_t cs[BLE_CAMSTAT_LEN];
        ble_pack_camstat(cs, millis());
        g_ch_camstat->setValue(cs, sizeof(cs));
    }

    // NimBLE 2.x: services start with the server, not individually.
    g_server->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_UUID_SERVICE);
    adv->enableScanResponse(true);
    // Keep the result. Until now nothing checked it, so a radio that failed to
    // start looked exactly like one that worked -- and the OTA self-test needs
    // to tell those apart before it accepts an image whose only control channel
    // this is.
    g_adv_ok = NimBLEDevice::startAdvertising();

    if (g_adv_ok) {
        Serial.printf("[BLE] GATT server up — advertising as \"%s\"\n", name);
    } else {
        Serial.printf("[BLE] GATT server up but ADVERTISING FAILED — \"%s\" is not visible\n", name);
    }
}

bool ble_server_up(void) { return g_adv_ok; }

void ble_server_tick(uint32_t now_ms) {
    if (!g_ch_state) return;
    ble_update_fps(now_ms);

    // Bulk runs every loop(), not on the 5 Hz State cadence — a ~100 KB capture
    // over 5 Hz chunks would take minutes.
    ble_bulk_pump();
    ble_apply_mode_request();      // answers only after the driver moved
    ble_apply_blackbox_request();
    ble_refresh_capability();
    ble_revoke_active_if_stale(now_ms);

    if (g_prefs_dirty) {
        g_prefs_dirty = false;
        FSDState snap;
        portENTER_CRITICAL(g_mux);
        snap = *g_state;
        portEXIT_CRITICAL(g_mux);
        prefs_save(&snap);
    }

    // CamStat first, and gated on the connection alone. It used to sit below the
    // State-subscription check, so a client that subscribed to CamStat but not
    // to State — which the protocol allows, and which is what an app showing
    // only camera status would do — received nothing at all, forever.
    if (g_connected && g_ch_camstat &&
        now_ms - g_last_camstat_ms >= BLE_CAMSTAT_PERIOD_MS) {
        g_last_camstat_ms = now_ms;
        uint8_t cs[BLE_CAMSTAT_LEN];
        ble_pack_camstat(cs, now_ms);
        g_ch_camstat->setValue(cs, sizeof(cs));
        g_ch_camstat->notify();
    }

    if (now_ms - g_last_state_ms < BLE_STATE_PERIOD_MS) return;
    g_last_state_ms = now_ms;

    // Nothing subscribed -> skip the pack entirely; keep the loop cheap.
    if (!g_connected || !g_state_subscribed) return;

    // rx_fps goes in through the packer now rather than being poked into bytes
    // 12-13 afterwards: the whole payload comes from one place, so the fixture
    // the app tests against covers every byte.
    uint8_t buf[BLE_STATE_LEN];
    ble_pack_state(buf, g_fps);

    g_ch_state->setValue(buf, sizeof(buf));
    g_ch_state->notify();
}

bool ble_server_connected(void) { return g_connected; }

#endif  // BLE_SERVER_ENABLED
