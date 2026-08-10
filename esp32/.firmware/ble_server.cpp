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

#include "capability.h"
#include "config.h"
#include <NimBLEDevice.h>
#include <string.h>

// ── UUIDs ────────────────────────────────────────────────────────────────────
// Custom 128-bit base; the 3rd..4th nibble group carries the characteristic id.
#define BLE_UUID_SERVICE "6b1a0000-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_STATE   "6b1a0001-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_COMMAND "6b1a0002-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_RESULT  "6b1a0003-4b53-4d4f-4432-43414e000001"
#define BLE_UUID_CAPAB   "6b1a0005-4b53-4d4f-4432-43414e000001"

#define BLE_STATE_LEN  20u
#define BLE_RESULT_LEN 4u

static FSDState     *g_state = nullptr;
static portMUX_TYPE *g_mux   = nullptr;

static NimBLEServer         *g_server   = nullptr;
static NimBLECharacteristic *g_ch_state = nullptr;
static NimBLECharacteristic *g_ch_result = nullptr;
static NimBLECharacteristic *g_ch_cap    = nullptr;

static volatile bool g_connected = false;
static volatile bool g_state_subscribed = false;  // client enabled State notifications
static uint32_t      g_last_state_ms = 0;

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
    // Byte 9 was speced as "gear" but the ESP32 state has no PRND field yet
    // (0x286 DI_state is parsed for cruise/park-brake only). Report the cruise
    // state instead so the byte carries something real, and revisit when a gear
    // parser exists. Documented in BLE-GATT-프로토콜.md.
    out[9]  = s.di_cruise_state;
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
            Serial.printf("[BLE] mode -> %s\n", arg ? "ACTIVE" : "LISTEN-ONLY");
            ble_send_result(cmd, BLE_RES_OK, (uint16_t)m);
            break;
        }

        case BLE_CMD_CAP_RECHECK:
            capability_start(millis());
            ble_send_result(cmd, BLE_RES_OK, 0);
            break;

        case BLE_CMD_SET_PROFILE:
        case BLE_CMD_PROFILE_STEP:
            // Deliberately not implemented yet. Scroll emulation injects
            // 0x3C2, and the exact value/timing must come from a real capture
            // (S3XY reference) before anything is transmitted — an Intel HW3
            // emergency-brake incident is on record for this path.
            ble_send_result(cmd, BLE_RES_UNSUPPORTED, 0);
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

class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
        g_connected = true;
        Serial.println("[BLE] client connected");
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        g_connected = false;
        g_state_subscribed = false;  // subscriptions die with the link
        Serial.println("[BLE] client disconnected — advertising again");
        NimBLEDevice::startAdvertising();
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
}

bool ble_server_connected(void) { return g_connected; }

#endif  // BLE_SERVER_ENABLED
