/*
 * capability.cpp — tap capability checker (ESP32, #125).
 *
 * Counts the capability-relevant CAN ids per bus over a short listen window,
 * then renders per-feature verdicts from the pure logic in fsd_capability.h.
 * Pure RX / read-only — never transmits.
 */

#include "ble_central.h"
#include "blackbox.h"
#include "capability.h"
#include "config.h"
#include "../../fsd_logic/fsd_btn_j6.h"      // FSD_J6_COUNT — the logical button count
#include "../../fsd_logic/fsd_capability.h"  // fsd_capability_eval
#include "../../fsd_logic/fsd_cap_json.h"    // the renderers + their size guard

/* The pure renderer and this file must agree about the limit and the shapes, or
 * the host size guard measures a document the board does not build.
 *
 * These live in the .cpp rather than in capability.h on purpose: they name
 * BLE_CENTRAL_MAX_BUTTONS and FSD_BTN_EVENTS, and a static_assert whose macros
 * are not in scope does not fail — the #ifdef around it evaporates and the
 * guard silently stops guarding. Here every include above is unconditional. */
static_assert(CAP_ATTR_MAX == FSD_CAP_JSON_ATTR_MAX,
              "the renderer and the board disagree about the ATT limit");
static_assert(BLE_CENTRAL_MAX_BUTTONS <= FSD_CAP_JSON_MAX_BOUND,
              "a slot the buttons document cannot carry is a slot the app cannot show");
static_assert(FSD_J6_COUNT * FSD_BTN_EVENTS == FSD_CAP_JSON_ROWS,
              "the row count moved; the btn hex string would no longer cover every row");

// Capability-relevant ids, in a fixed slot order. Kept local so the counting
// table and the seen-set construction can't drift.
enum CapId : uint8_t {
    CAP_ID_EPAS = 0,   // 0x370 EPAS_STATUS
    CAP_ID_DAS_HW4,    // 0x39B DAS_STATUS_HW4
    CAP_ID_DAS_HW3,    // 0x399 DAS_STATUS_HW3 / ISA_SPEED-HW4
    CAP_ID_AP_CTRL,    // 0x3FD AP_CONTROL
    CAP_ID_AP_LEGACY,  // 0x3EE AP_LEGACY
    CAP_ID_STEER,      // 0x129 STEER_ANGLE
    CAP_ID_BODY_UI,    // 0x273 UI_vehicleControl   (#128 Vehicle-bus reachability)
    CAP_ID_BODY_DOOR,  // 0x102 VCLEFT_doorStatus
    CAP_ID_BODY_WINDOW,// 0x119 VCSEC_windowRequests
    CAP_ID_BODY_LIGHTS,// 0x3E9 DAS_bodyControls
    CAP_ID_SCROLL,     // 0x3C2 VCLEFT_switchStatus — right scroll wheel (speed profile)
    CAP_ID_COUNT,
};

static const uint32_t kCapCanId[CAP_ID_COUNT] = {
    CAN_ID_EPAS_STATUS, CAN_ID_DAS_STATUS_HW4,  CAN_ID_DAS_STATUS_HW3,
    CAN_ID_AP_CONTROL,  CAN_ID_AP_LEGACY,       CAN_ID_STEER_ANGLE,
    CAN_ID_UI_VEHICLE_CTRL, CAN_ID_VCLEFT_DOOR, CAN_ID_VCSEC_WINDOW, CAN_ID_DAS_BODY,
    CAN_ID_VCLEFT_SWITCH,
};

enum CapRunState : uint8_t { CAP_STATE_IDLE = 0, CAP_STATE_RUNNING, CAP_STATE_DONE };

static FSDState*      g_state     = nullptr;
static portMUX_TYPE*  g_state_mux = nullptr;

static volatile uint8_t  g_cap_state = CAP_STATE_IDLE;
static uint32_t          g_cap_deadline_ms = 0;
static volatile uint16_t g_count[CAN_BUS_COUNT][CAP_ID_COUNT] = {};

void capability_init(FSDState* state, portMUX_TYPE* state_mux) {
    g_state     = state;
    g_state_mux = state_mux;
    g_cap_state = CAP_STATE_IDLE;
}

void capability_start(uint32_t now_ms) {
    for (uint8_t b = 0; b < CAN_BUS_COUNT; b++)
        for (uint8_t i = 0; i < CAP_ID_COUNT; i++) g_count[b][i] = 0;
    g_cap_deadline_ms = now_ms + CAP_WINDOW_MS;
    g_cap_state = CAP_STATE_RUNNING;
}

void capability_record(CanBusId bus, const CanFrame& frame, uint32_t now_ms) {
    if (g_cap_state != CAP_STATE_RUNNING) return;
    if (now_ms >= g_cap_deadline_ms) return;  // tick() will finalize
    if (bus >= CAN_BUS_COUNT) return;
    for (uint8_t i = 0; i < CAP_ID_COUNT; i++) {
        if (frame.id == kCapCanId[i]) {
            if (g_count[bus][i] < 0xFFFF) g_count[bus][i]++;
            break;
        }
    }
}

bool capability_running(void) { return g_cap_state == CAP_STATE_RUNNING; }

void capability_tick(uint32_t now_ms) {
    if (g_cap_state == CAP_STATE_RUNNING && now_ms >= g_cap_deadline_ms)
        g_cap_state = CAP_STATE_DONE;
}

static FSDCapSeen seen_for_bus(uint8_t bus) {
    FSDCapSeen s;
    s.epas       = g_count[bus][CAP_ID_EPAS]      >= CAP_MIN_FRAMES;
    s.das_hw4    = g_count[bus][CAP_ID_DAS_HW4]   >= CAP_MIN_FRAMES;
    s.das_hw3    = g_count[bus][CAP_ID_DAS_HW3]   >= CAP_MIN_FRAMES;
    s.ap_control = g_count[bus][CAP_ID_AP_CTRL]   >= CAP_MIN_FRAMES;
    s.ap_legacy  = g_count[bus][CAP_ID_AP_LEGACY] >= CAP_MIN_FRAMES;
    s.steer      = g_count[bus][CAP_ID_STEER]     >= CAP_MIN_FRAMES;
    s.body_ui     = g_count[bus][CAP_ID_BODY_UI]     >= CAP_MIN_FRAMES;
    s.body_door   = g_count[bus][CAP_ID_BODY_DOOR]   >= CAP_MIN_FRAMES;
    s.body_window = g_count[bus][CAP_ID_BODY_WINDOW] >= CAP_MIN_FRAMES;
    s.body_lights = g_count[bus][CAP_ID_BODY_LIGHTS] >= CAP_MIN_FRAMES;
    s.scroll      = g_count[bus][CAP_ID_SCROLL]      >= CAP_MIN_FRAMES;
    return s;
}

static uint32_t bus_total(uint8_t bus) {
    uint32_t t = 0;
    for (uint8_t i = 0; i < CAP_ID_COUNT; i++) t += g_count[bus][i];
    return t;
}

static const char* hint_str(FSDCapBusHint h) {
    switch (h) {
        case CAP_HINT_PARTY:   return "looks Party-like";
        case CAP_HINT_CHASSIS: return "looks Chassis/Vehicle-like";
        default:               return "";
    }
}

/* ── the two read-only documents ─────────────────────────────────────────────
 *
 * 🔴 THE FORMATTING MOVED OUT, to fsd_logic/fsd_cap_json.c, and the reason is
 * that this document overflowed the 512-byte ATT limit three times and every
 * one of those was found on hardware. It could not be found anywhere else: the
 * builder needed an Arduino String, so nothing on a desk could ask it how large
 * it gets. Now a host test builds the largest document each renderer can ever
 * produce and asserts it fits, and these functions only gather values.
 *
 * 🔴 THE BUTTONS LEFT THE CAPABILITY DOCUMENT for the same reason the scan list
 * did. Five bound remotes are five 17-character addresses plus their state —
 * about four hundred bytes, most of the budget — and an attribute NimBLE
 * refuses holds NOTHING, so an overflow caused by the buttons also erased the
 * bus verdicts the other screens read. They are different questions asked by
 * different screens anyway.
 */
static void fill_status(FsdCapJsonStatus* out) {
    memset(out, 0, sizeof(*out));

    uint32_t now = millis();
    out->state     = g_cap_state;
    out->ms_left   = (g_cap_state == CAP_STATE_RUNNING && g_cap_deadline_ms > now)
                         ? (g_cap_deadline_ms - now) : 0u;
    out->window_ms = CAP_WINDOW_MS;

    TeslaHWVersion hw = TeslaHW_Unknown;
    if (g_state && g_state_mux) {
        portENTER_CRITICAL(g_state_mux);
        hw = g_state->hw_version;
        portEXIT_CRITICAL(g_state_mux);
    }
    out->hw = (uint8_t)hw;

    /* 🔴 The capture disk, which the phone had no way to see. `bbclear` can be
     * pressed from the phone since 2026-09-02, so without these three someone
     * deletes captures without knowing how much room they had — or, worse, sees
     * "받을 것이 없습니다" after a failed mark and cannot tell whether to shoot
     * again or to clear. Before TSL comes out, guessing wrong costs a capture
     * that cannot be retaken.
     *
     * ⚠️ `blackbox_event_count()` returns int and can be negative when the
     * backend cannot be read. Clamp rather than wrap — a count of 65,535 on the
     * phone would be read as a real number. */
    /* 🔴 `blackbox_free_bytes()` answers 0xFFFFFFFF on the RAM backend — "there
     * is no filesystem to run out of". Divided by 1024 that becomes 4,194,303,
     * and the phone would print it as a real number of kilobytes. Pass the
     * sentinel through UNCHANGED and let the app name it; inventing a number
     * here is how a screen starts quietly lying. */
    const uint32_t bb_free = blackbox_free_bytes();
    const int bb_n = blackbox_event_count();
    out->bb_free_kb = (bb_free == 0xFFFFFFFFu) ? 0xFFFFFFFFu : (bb_free / 1024u);
    out->bb_count   = (bb_n > 0) ? (uint16_t)((bb_n > 0xFFFF) ? 0xFFFF : bb_n) : 0u;
    out->bb_lost    = blackbox_last_mark_lost() ? 1u : 0u;

    uint8_t n = 0;
    for (uint8_t b = 0; b < CAN_BUS_COUNT && n < FSD_CAP_JSON_MAX_BUSES; b++) {
        uint32_t total = bus_total(b);
        // Always report can0; report a secondary bus only when it carried frames
        // (single-CAN boards never receive on can1).
        if (b != 0 && total == 0) continue;

        FSDCapSeen seen = seen_for_bus(b);
        FSDCapReport r = fsd_capability_eval(seen, hw);

        FsdCapJsonBus* d = &out->bus[n++];
        snprintf(d->name, sizeof(d->name), "%s", can_bus_name((CanBusId)b));
        d->frames = total;

        /* ── the seen-id set, as a BITMASK ───────────────────────────────────
         *
         * 🔴 THIS USED TO BE ELEVEN NAMED BOOLEANS and that is what broke the
         * whole document the first time. `{"epas":false,"das_hw4":false,...}` is
         * ~190 bytes PER BUS, so two buses spent 400 of the 512 an ATT attribute
         * may hold (2026-08-18). A set of booleans is a bitmask; writing it as
         * prose was the mistake.
         *
         * ⚠️ THE BIT ORDER IS A CONTRACT. wire/Capability.kt decodes it back
         * into the same named fields, so inserting a bit in the middle silently
         * renames every flag above it. Append only; CapabilityTest pins this
         * exact mapping. */
        uint32_t ids = 0;
        if (seen.epas)        ids |= CAP_ID_BIT_EPAS;
        if (seen.das_hw4)     ids |= CAP_ID_BIT_DAS_HW4;
        if (seen.das_hw3)     ids |= CAP_ID_BIT_DAS_HW3;
        if (seen.ap_control)  ids |= CAP_ID_BIT_AP_CTRL;
        if (seen.ap_legacy)   ids |= CAP_ID_BIT_AP_LEGACY;
        if (seen.steer)       ids |= CAP_ID_BIT_STEER;
        if (seen.body_ui)     ids |= CAP_ID_BIT_BODY_UI;
        if (seen.body_door)   ids |= CAP_ID_BIT_BODY_DOOR;
        if (seen.body_window) ids |= CAP_ID_BIT_BODY_WINDOW;
        if (seen.body_lights) ids |= CAP_ID_BIT_BODY_LIGHTS;
        if (seen.scroll)      ids |= CAP_ID_BIT_SCROLL;
        d->ids = ids;

        /* The six verdicts, the HW caveat and the bus hint ride in one packed
         * word — see FSD_CAP_V_*_SHIFT. Spelling them out cost ~160 bytes per
         * bus for what is fifteen bits. */
        d->nag_killer     = (uint8_t)r.nag_killer;
        d->ap_first       = (uint8_t)r.ap_first;
        d->fsd_activation = (uint8_t)r.fsd_activation;
        d->soft_engage    = (uint8_t)r.soft_engage;
        d->body_control   = (uint8_t)r.body_control;
        d->scroll_profile = (uint8_t)r.scroll_profile;
        d->hw_unconfirmed = r.hw_unconfirmed;
        d->hint           = (uint8_t)r.bus_hint;
    }
    out->bus_count = n;
}

String capability_status_json() {
    FsdCapJsonStatus s;
    fill_status(&s);

    char buf[CAP_ATTR_MAX + 1u];
    const size_t n = fsd_cap_json_status(buf, sizeof(buf), &s);

    /* Cannot happen — the host test proves the worst case fits — but say so
     * rather than handing back an empty document in silence if it ever does. */
    if (n == 0u) {
        Serial.println("[CAP] status document did not fit; see test_cap_json");
        return String("");
    }
    return String(buf);
}

/* Which remotes are bound, and what they have pressed.
 *
 * 🔴 `verified` WAS HARDCODED false here, from the months before a button had
 * been bought — and it stayed false after the J6 decoder was measured and
 * shipped, so the app told the owner a working remote did not work. It comes
 * from the firmware now, and cannot freeze again.
 *
 * `verified` answers "does this build have a real decoder"; it says nothing
 * about the device in a given slot. `d` (decoded) does that — presses from THAT
 * remote that we could name. The app needs both: without the count, a remote we
 * do not understand connects and looks fine.
 *
 * The per-row counts are what let a person find which row is the key under
 * their thumb: press it, watch a number move. Without them the screen can only
 * say "something arrived", which names nothing. */
String capability_buttons_json() {
    FsdCapJsonButtons t;
    memset(&t, 0, sizeof(t));

    t.connected = ble_central_any_connected();
    t.verified  = ble_central_decoder_verified();
    t.slots     = (uint8_t)BLE_CENTRAL_MAX_BUTTONS;

    uint8_t n = 0;
    for (uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS && n < FSD_CAP_JSON_MAX_BOUND; i++) {
        const char* a = ble_central_slot_addr(i);
        if (!a || !a[0]) continue;
        FsdCapJsonBound* e = &t.bound[n++];
        e->slot = i;
        snprintf(e->addr, sizeof(e->addr), "%s", a);
        e->connected = ble_central_slot_connected(i);
        e->decoded   = (uint8_t)ble_central_slot_decoded(i);
    }
    t.bound_count = n;

    for (uint8_t r = 0; r < FSD_CAP_JSON_ROWS; r++) t.rows[r] = ble_central_row_events(r);
    t.act = ble_central_action_mask();

    char buf[CAP_ATTR_MAX + 1u];
    const size_t len = fsd_cap_json_buttons(buf, sizeof(buf), &t);
    if (len == 0u) {
        Serial.println("[CAP] buttons document did not fit; see test_cap_json");
        return String("");
    }
    return String(buf);
}

/* ── the scan list, in its own document ──────────────────────────────────────
 *
 * Its own characteristic because it is a different KIND of thing: transient,
 * and as large as the room it was taken in. Eight named devices are about half
 * a kilobyte, which is the whole ATT budget on its own — sharing a document
 * with the module's own facts meant one of them had to lose, and for a while
 * the scan list lost completely (it got zero bytes).
 *
 * Still fitted rather than appended: `kept` says how many are here and `seen`
 * how many the radio actually found. A silent cap reads as "this is all there
 * is", and someone whose remote is missing then looks in the wrong place.
 */
String capability_scan_json() {
    String j;
    j.reserve(CAP_ATTR_MAX + 64);
    j = "{\"scanning\":";
    j += ble_central_scanning() ? "true" : "false";
    j += ",\"seen\":";
    j += (int)ble_central_found_total();
    j += ",\"found\":[";

    uint8_t kept = 0;
    for (uint8_t i = 0; i < ble_central_found_count(); i++) {
        const char* addr = "";
        const char* name = "";
        int8_t rssi = 0;
        if (!ble_central_found(i, &addr, &name, &rssi)) break;

        String e;
        if (kept) e += ',';
        e += "{\"addr\":\""; e += addr;  e += "\",";
        e += "\"name\":\"";  e += name;  e += "\",";
        e += "\"rssi\":";     e += (int)rssi; e += '}';

        // Room for `],"kept":NN}` after the list.
        if (j.length() + e.length() + 20u > CAP_ATTR_MAX) break;
        j += e;
        kept++;
    }
    j += "],\"kept\":"; j += (int)kept; j += '}';
    return j;
}
