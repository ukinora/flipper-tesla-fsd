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
#include "blackbox.h"
#include "camera_store.h"
#include "capability.h"
#include "config.h"
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

static FSDState     *g_state = nullptr;
static portMUX_TYPE *g_mux   = nullptr;

static NimBLEServer         *g_server   = nullptr;
static NimBLECharacteristic *g_ch_state = nullptr;
static NimBLECharacteristic *g_ch_result = nullptr;
static NimBLECharacteristic *g_ch_cap    = nullptr;
static NimBLECharacteristic *g_ch_bulk   = nullptr;
static NimBLECharacteristic *g_ch_upload = nullptr;
static NimBLECharacteristic *g_ch_camstat = nullptr;

static volatile bool g_connected = false;
static volatile bool g_state_subscribed = false;  // client enabled State notifications
static volatile bool g_bulk_subscribed  = false;
static volatile uint16_t g_mtu = 23;              // until the peer negotiates up
static uint32_t      g_last_state_ms = 0;
static uint32_t      g_last_camstat_ms = 0;

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
// Declared size of the camera.bin currently being uploaded (0 = none).
static volatile uint32_t g_upload_expect = 0;
static char     g_bulk_name[40] = {};
static size_t   g_bulk_offset = 0;
static uint16_t g_bulk_seq    = 0;

// ── State serialisation (20 B, little-endian) ────────────────────────────────
// Layout is fixed; the app parses by offset. Bump BLE_PROTO_VERSION on change.
static void ble_pack_state(uint8_t *out) {
    FSDState s;
    portENTER_CRITICAL(g_mux);
    s = *g_state;
    portEXIT_CRITICAL(g_mux);

    uint8_t flags = 0;
    // rx_count is the same wiring-sanity signal the red LED uses.
    if (s.rx_count > 0)             flags |= (1u << 0);  // CAN traffic seen
    if (s.tesla_ota_in_progress)    flags |= (1u << 1);
    if (s.ui_left_blinker)          flags |= (1u << 2);
    if (s.ui_right_blinker)         flags |= (1u << 3);
    // bits 4/5 (blind spot L/R) intentionally unset: DAS_sideCollisionWarning is
    // not extracted on the ESP32 path yet, and its bit position is only confirmed
    // for 0x39B (HW4/Highland) — see CLAUDE.md. Left 0 rather than guessed.
    if (s.driver_brake_applied)     flags |= (1u << 6);
    // bit 7 (profile change in progress) is set by the SET_PROFILE closed loop.

    int prof = s.speed_profile;
    if (prof < 0) prof = 0;
    if (prof > 3) prof = 3;   // FSD v14 Lite: Sloth/Chill/Standard/Hurry

    float soc = s.soc_percent;
    if (soc < 0.0f)   soc = 0.0f;
    if (soc > 100.0f) soc = 100.0f;

    float kph = s.vehicle_speed_kph;
    if (kph < 0.0f) kph = 0.0f;
    uint32_t kph10 = (uint32_t)(kph * 10.0f + 0.5f);
    if (kph10 > 0xFFFFu) kph10 = 0xFFFFu;

    uint32_t lim = (uint32_t)(s.speed_limit_seen ? s.speed_limit_kph + 0.5f : 0.0f);
    if (lim > 0xFFFFu) lim = 0xFFFFu;

    uint32_t crc = s.crc_err_count;
    if (crc > 0xFFFFu) crc = 0xFFFFu;

    memset(out, 0, BLE_STATE_LEN);
    out[0]  = BLE_PROTO_VERSION;
    out[1]  = flags;
    out[2]  = (uint8_t)s.op_mode;
    out[3]  = (uint8_t)s.hw_version;
    out[4]  = (uint8_t)prof;
    out[5]  = s.das_ap_state;
    out[6]  = (uint8_t)(kph10 & 0xFFu);
    out[7]  = (uint8_t)(kph10 >> 8);
    out[8]  = (uint8_t)(soc + 0.5f);
    // Gear, as originally specified. This byte carried the cruise state as a
    // placeholder while the ESP32 had no PRND parser — the protocol note said
    // to put it back once one existed, and fsd_autonomy.c now provides one.
    // Values are the DBC's: 0=INVALID 1=P 2=R 3=N 4=D 7=SNA.
    // Wire meaning changed, so BLE_PROTO_VERSION goes to 2.
    out[9]  = s.di_gear;
    out[10] = (uint8_t)(lim & 0xFFu);
    out[11] = (uint8_t)(lim >> 8);
    // out[12..13] rx_fps — filled by the caller-side counter below.
    out[14] = (uint8_t)(crc & 0xFFu);
    out[15] = (uint8_t)(crc >> 8);
    uint32_t up = millis() / 1000u;
    out[16] = (uint8_t)(up & 0xFFu);
    out[17] = (uint8_t)((up >> 8) & 0xFFu);
    out[18] = (uint8_t)((up >> 16) & 0xFFu);
    out[19] = (uint8_t)((up >> 24) & 0xFFu);
}

// ── camera / autonomy status ─────────────────────────────────────────────────
// Reports only what this build actually knows. The tracker and policy compile
// but are not instantiated — nothing feeds them a position yet — so their
// fields are absent rather than present-and-always-zero.
static void ble_pack_camstat(uint8_t *out, uint32_t now_ms) {
    FSDState s;
    portENTER_CRITICAL(g_mux);
    s = *g_state;
    portEXIT_CRITICAL(g_mux);

    FsdSupVerdict v = fsd_supervised_drive_why(&s, now_ms);
    const FsdCamDb *db = camera_store_db();

    uint8_t flags = 0;
    if (s.autonomy_enabled)  flags |= (1u << 0);
    if (v == FSD_SUP_OK)     flags |= (1u << 1);
    if (db != nullptr)       flags |= (1u << 2);

    memset(out, 0, BLE_CAMSTAT_LEN);
    out[0] = BLE_PROTO_VERSION;
    out[1] = flags;
    out[2] = (uint8_t)v;
    out[3] = (uint8_t)s.op_mode;

    uint32_t n = camera_store_count();
    uint32_t built = db ? db->built_at : 0u;
    for (int i = 0; i < 4; i++) {
        out[4 + i]  = (uint8_t)((n >> (8 * i)) & 0xFFu);
        out[8 + i]  = (uint8_t)((built >> (8 * i)) & 0xFFu);
    }
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
    uint8_t b[BLE_RESULT_LEN] = {
        cmd, res, (uint8_t)(extra & 0xFFu), (uint8_t)(extra >> 8)
    };
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

// Hand Active back once the phone has been gone for BLE_ACTIVE_GRACE_MS.
// Runs from loop(), so g_state is touched on the same task as everything else.
static void ble_revoke_active_if_stale(uint32_t now_ms) {
    if (g_connected || g_link_down_ms == 0) return;
    if ((uint32_t)(now_ms - g_link_down_ms) < BLE_ACTIVE_GRACE_MS) return;

    g_link_down_ms  = 0;
    g_active_by_ble = false;

    portENTER_CRITICAL(g_mux);
    bool was_active = (g_state->op_mode == OpMode_Active);
    // Not Listen-Only unconditionally any more. The phone leaving withdraws the
    // GENERAL transmit permission it granted, but it is not a reason to stop
    // responding to speed cameras — a phone left at home is the exact case this
    // module exists to cover. The floor grants no general TX either way, and
    // the camera path still has to see a person driving.
    OpMode floor = fsd_autonomy_floor(g_state);
    if (was_active) g_state->op_mode = floor;
    portEXIT_CRITICAL(g_mux);

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

        // Only a bonded/authenticated peer may drive the module. The Command
        // characteristic is declared WRITE_AUTHEN, but re-check: this path can
        // enable CAN transmission.
        if (!info.isEncrypted()) {
            ble_send_result(v[0], BLE_RES_REJECTED, 0);
            return;
        }

        const uint8_t cmd = (uint8_t)v[0];
        const uint8_t arg = (v.size() > 1) ? (uint8_t)v[1] : 0u;

        switch (cmd) {
        case BLE_CMD_PING:
            ble_send_result(cmd, BLE_RES_OK, BLE_PROTO_VERSION);
            break;

        case BLE_CMD_SET_MODE: {
            OpMode m = arg ? OpMode_Active : OpMode_ListenOnly;
            portENTER_CRITICAL(g_mux);
            g_state->op_mode = m;
            portEXIT_CRITICAL(g_mux);
            // Remember that this Active is ours, so the grace timer knows what
            // it may revoke (and stop tracking once the phone turns it off).
            g_active_by_ble = (m == OpMode_Active);
            Serial.printf("[BLE] mode -> %s\n", arg ? "ACTIVE" : "LISTEN-ONLY");
            ble_send_result(cmd, BLE_RES_OK, (uint16_t)m);
            break;
        }

        case BLE_CMD_CAP_RECHECK:
            capability_start(millis());
            ble_send_result(cmd, BLE_RES_OK, 0);
            break;

        case BLE_CMD_SET_AUTONOMY: {
            bool on = (arg != 0);
            portENTER_CRITICAL(g_mux);
            g_state->autonomy_enabled = on;
            // Re-derive the floor now. Turning autonomy off while sitting in
            // Autonomous has to drop the module out of it immediately;
            // otherwise the switch would appear to do nothing until a reboot.
            if (g_state->op_mode == OpMode_Autonomous ||
                g_state->op_mode == OpMode_ListenOnly)
                g_state->op_mode = fsd_autonomy_floor(g_state);
            FSDState snap = *g_state;
            portEXIT_CRITICAL(g_mux);
            prefs_save(&snap);
            Serial.printf("[BLE] autonomy -> %s\n", on ? "ON" : "OFF");
            ble_send_result(cmd, BLE_RES_OK, (uint16_t)snap.op_mode);
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
        g_link_down_ms = 0;  // reconnected in time — cancel the pending revoke
        Serial.println("[BLE] client connected");
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        g_connected = false;
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
        Serial.println("[BLE] client disconnected — advertising again");
        NimBLEDevice::startAdvertising();
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
    // Bonding + MITM: pairing is required before commands are accepted.
    NimBLEDevice::setSecurityAuth(true, true, true);

    g_server = NimBLEDevice::createServer();
    g_server->setCallbacks(&g_srv_cb);

    NimBLEService *svc = g_server->createService(BLE_UUID_SERVICE);

    g_ch_state = svc->createCharacteristic(
        BLE_UUID_STATE, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    g_ch_state->setCallbacks(&g_state_cb);

    NimBLECharacteristic *ch_cmd = svc->createCharacteristic(
        BLE_UUID_COMMAND, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_AUTHEN);
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

    // NimBLE 2.x: services start with the server, not individually.
    g_server->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_UUID_SERVICE);
    adv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();

    Serial.printf("[BLE] GATT server up — advertising as \"%s\"\n", name);
}

void ble_server_tick(uint32_t now_ms) {
    if (!g_ch_state) return;
    ble_update_fps(now_ms);

    // Bulk runs every loop(), not on the 5 Hz State cadence — a ~100 KB capture
    // over 5 Hz chunks would take minutes.
    ble_bulk_pump();
    ble_revoke_active_if_stale(now_ms);

    if (now_ms - g_last_state_ms < BLE_STATE_PERIOD_MS) return;
    g_last_state_ms = now_ms;

    // Nothing subscribed -> skip the pack entirely; keep the loop cheap.
    if (!g_connected || !g_state_subscribed) return;

    uint8_t buf[BLE_STATE_LEN];
    ble_pack_state(buf);
    buf[12] = (uint8_t)(g_fps & 0xFFu);
    buf[13] = (uint8_t)(g_fps >> 8);

    g_ch_state->setValue(buf, sizeof(buf));
    g_ch_state->notify();

    if (g_ch_camstat && now_ms - g_last_camstat_ms >= BLE_CAMSTAT_PERIOD_MS) {
        g_last_camstat_ms = now_ms;
        uint8_t cs[BLE_CAMSTAT_LEN];
        ble_pack_camstat(cs, now_ms);
        g_ch_camstat->setValue(cs, sizeof(cs));
        g_ch_camstat->notify();
    }
}

bool ble_server_connected(void) { return g_connected; }

#endif  // BLE_SERVER_ENABLED
