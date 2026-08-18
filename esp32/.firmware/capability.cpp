/*
 * capability.cpp — tap capability checker (ESP32, #125).
 *
 * Counts the capability-relevant CAN ids per bus over a short listen window,
 * then renders per-feature verdicts from the pure logic in fsd_capability.h.
 * Pure RX / read-only — never transmits.
 */

#include "ble_central.h"
#include "capability.h"
#include "config.h"
#include "../../fsd_logic/fsd_btn_j6.h"      // FSD_J6_COUNT — the logical button count
#include "../../fsd_logic/fsd_capability.h"  // fsd_capability_eval

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

String capability_status_json() {
    uint8_t st = g_cap_state;
    uint32_t now = millis();
    uint32_t ms_left = (st == CAP_STATE_RUNNING && g_cap_deadline_ms > now)
                           ? (g_cap_deadline_ms - now) : 0;

    TeslaHWVersion hw = TeslaHW_Unknown;
    if (g_state && g_state_mux) {
        portENTER_CRITICAL(g_state_mux);
        hw = g_state->hw_version;
        portEXIT_CRITICAL(g_state_mux);
    }

    String j;
    j.reserve(CAP_ATTR_MAX + 64);
    j  = "{";
    j += "\"state\":";   j += (int)st;       j += ',';
    j += "\"ms_left\":"; j += ms_left;       j += ',';
    j += "\"window_ms\":"; j += CAP_WINDOW_MS; j += ',';
    j += "\"hw\":";      j += (int)hw;       j += ',';
    j += "\"buses\":[";

    bool first = true;
    for (uint8_t b = 0; b < CAN_BUS_COUNT; b++) {
        uint32_t total = bus_total(b);
        // Always report can0; report a secondary bus only when it carried frames
        // (single-CAN boards never receive on can1).
        if (b != 0 && total == 0) continue;

        FSDCapSeen seen = seen_for_bus(b);
        FSDCapReport r = fsd_capability_eval(seen, hw);

        if (!first) j += ',';
        first = false;
        j += "{\"bus\":\""; j += can_bus_name((CanBusId)b); j += "\",";
        j += "\"frames\":"; j += total; j += ',';
        /* ── the seen-id set, as a BITMASK ───────────────────────────────────
         *
         * 🔴 THIS USED TO BE ELEVEN NAMED BOOLEANS and that is what broke the
         * whole document. `{"epas":false,"das_hw4":false,...}` is ~190 bytes
         * PER BUS, so two buses spent 400 of the 512 an ATT attribute may hold
         * (2026-08-18). NimBLE rejected the oversized value outright and the
         * phone read an empty document — for every screen, not just this one.
         *
         * A set of booleans is a bitmask; writing it as prose was the mistake.
         * 190 bytes becomes about 6.
         *
         * ⚠️ THE BIT ORDER IS A CONTRACT. wire/Capability.kt decodes it back
         * into the same named fields, so inserting a bit in the middle silently
         * renames every flag above it. Append only, and the app's CapIdsTest
         * pins this exact mapping. */
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
        j += "\"ids\":"; j += (unsigned long)ids; j += ',';
        j += "\"nag_killer\":";     j += (int)r.nag_killer;     j += ',';
        j += "\"ap_first\":";       j += (int)r.ap_first;       j += ',';
        j += "\"fsd_activation\":"; j += (int)r.fsd_activation; j += ',';
        j += "\"soft_engage\":";    j += (int)r.soft_engage;    j += ',';
        j += "\"body_control\":";   j += (int)r.body_control;   j += ',';
        j += "\"scroll_profile\":"; j += (int)r.scroll_profile; j += ',';
        j += "\"hw_unconfirmed\":"; j += r.hw_unconfirmed ? "true" : "false"; j += ',';
        j += "\"hint\":\"";         j += hint_str(r.bus_hint);  j += "\"}";
    }
    j += "],";

    /* Bluetooth buttons.
     *
     * 🔴 This document is where scan results reach the phone. A separate
     * characteristic would have been tidier, but this one already answers
     * "what does the module see" and the app already re-reads it on demand —
     * and the alternative was the app having no way at all, which is what it
     * had. The scan itself is a command (BLE_CMD_BTN_SCAN); this is the answer.
     *
     * 🔴 `verified` WAS HARDCODED false here, from the months before a button
     * had been bought — and it stayed false after the J6 decoder was measured
     * and shipped, so the app told the owner a working remote did not work.
     * It comes from the firmware now, and cannot freeze again.
     *
     * `verified` answers "does this build have a real decoder"; it says nothing
     * about the device in a given slot. `decoded` does that — presses from THAT
     * remote that we could name. The app needs both: without the count, a
     * remote we do not understand connects and looks fine. */
    j += "\"buttons\":{";
    j += "\"scanning\":"; j += ble_central_scanning() ? "true" : "false"; j += ',';
    j += "\"connected\":"; j += ble_central_any_connected() ? "true" : "false"; j += ',';
    j += "\"verified\":"; j += ble_central_decoder_verified() ? "true" : "false"; j += ',';
    j += "\"bound\":[";
    for (uint8_t i = 0; i < BLE_CENTRAL_MAX_BUTTONS; i++) {
        const char* a = ble_central_slot_addr(i);
        if (!a || !a[0]) continue;
        if (j[j.length() - 1] != '[') j += ',';
        j += "{\"slot\":";      j += (int)i; j += ',';
        j += "\"addr\":\"";    j += a;      j += "\",";
        j += "\"connected\":";  j += ble_central_slot_connected(i) ? "true" : "false"; j += ',';
        j += "\"decoded\":";    j += (int)ble_central_slot_decoded(i);
        j += '}';
    }
    j += "],";
    j += "\"slots\":"; j += (int)BLE_CENTRAL_MAX_BUTTONS; j += ',';

    /* Per LOGICAL button: how many gestures it has produced, and which ones are
     * watching for doubles. Compact on purpose — the app knows the order from
     * fsd_btn_j6.h, and spelling nine of anything out as named objects is
     * exactly what pushed this document past the ATT limit once already.
     *
     * The counts are what let a person find which row is the key under their
     * thumb: press it, watch a number move. Without them the screen can only
     * say "something arrived", which names nothing. */
    /* 🔴 HEX STRING, NOT AN ARRAY, and the reason is size. Thirty rows written
     * as decimal numbers grow with the counts — a document that fits today
     * stops fitting after a hundred presses, and NimBLE answers an oversized
     * value by storing NOTHING. Two hex characters per row is 60 bytes no
     * matter what the counts are, so the document cannot outgrow the ATT limit
     * by being used. */
    j += "\"btn\":\"";
    for (uint8_t r = 0; r < FSD_J6_COUNT * FSD_BTN_EVENTS; r++) {
        const uint8_t n = ble_central_row_events(r);
        static const char kHex[] = "0123456789abcdef";
        j += kHex[(n >> 4) & 0xFu];
        j += kHex[n & 0xFu];
    }
    j += "\",";
    j += "\"act\":"; j += (unsigned long)ble_central_action_mask(); j += ',';

    /* ── the scan list, and the reason it is fitted rather than appended ──────
     *
     * 🔴 512 BYTES IS A SPEC LIMIT, NOT A SETTING. BLE_ATT_ATTR_MAX_LEN is the
     * largest value an ATT attribute may hold, so no amount of MTU raises it.
     * NimBLE does not truncate an oversized setValue() — it REJECTS it, and the
     * characteristic is left holding ZERO BYTES.
     *
     * That happened (2026-08-18). The document grew past 512 the moment the
     * listen window closed and the bus counts filled in; the phone then read an
     * empty document forever, `parseCapability()` returned null, and every
     * screen drew "아직 확인되지 않았습니다" — the same thing it draws before the
     * first read. Nobody saw a failure because nobody looked: the firmware did
     * not check that setValue() took, and the app swallowed the parse in three
     * nested runCatching{}.getOrNull().
     *
     * The scan list is what makes the size unbounded — eight named devices are
     * about half a kilobyte on their own — so entries go in only while they
     * fit, and how many were left out is REPORTED. A silent cap would read as
     * "these are all the devices there are", which is worse than showing fewer.
     */
    String found;
    uint8_t dropped = 0;
    const uint8_t found_n = ble_central_found_count();
    for (uint8_t i = 0; i < found_n; i++) {
        const char* addr = "";
        const char* name = "";
        int8_t rssi = 0;
        if (!ble_central_found(i, &addr, &name, &rssi)) break;

        String e;
        if (found.length()) e += ',';
        e += "{\"addr\":\""; e += addr;  e += "\",";
        e += "\"name\":\"";  e += name;  e += "\",";
        e += "\"rssi\":";     e += (int)rssi; e += '}';

        /* CAP_TAIL_RESERVE covers what still has to be written after the list:
         * `,"found_dropped":NNN,"found":[` plus `]}}`. Kept generous — running
         * out here is the failure this whole block exists to prevent. */
        if (j.length() + found.length() + e.length() + CAP_TAIL_RESERVE > CAP_ATTR_MAX) {
            dropped = (uint8_t)(found_n - i);
            break;
        }
        found += e;
    }
    j += "\"found_dropped\":"; j += (int)dropped; j += ',';
    j += "\"found\":["; j += found; j += "]}}";

    /* Last line of defence. If some future field pushes the fixed part over the
     * limit, an empty characteristic is the worst possible answer — it is
     * indistinguishable from "not read yet". Give up the scan list first, and
     * say so out loud. */
    if (j.length() > CAP_ATTR_MAX) {
        Serial.printf("[CAP] 🔴 %u bytes exceeds the %u-byte ATT limit even without "
                      "the scan list - the fixed part has to shrink\n",
                      (unsigned)j.length(), (unsigned)CAP_ATTR_MAX);
    }
    return j;
}
