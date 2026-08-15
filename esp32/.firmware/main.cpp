/*
 * main.cpp — Tesla FSD Unlock for ESP32
 *
 * Port of hypery11/flipper-tesla-fsd to M5Stack ATOM Lite + ATOMIC CAN Base.
 *
 * Default state: Listen-Only (blue LED).  Press button once to go Active (green).
 *
 * Button:
 *   Single click  → toggle Listen-Only / Active
 *   Long press 3s → toggle NAG Killer on/off
 *   Double click  → toggle BMS serial output
 *
 * Serial 115200 baud.  Status prints every 5 s when Active.
 * BMS output (when enabled): voltage, current, power, SoC, temp every 1 s.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <esp_sleep.h>
#include <esp_ota_ops.h>
#include "config.h"
#include "can_signals.h"
#include "fsd_handler.h"
#include "../../fsd_logic/fsd_autonomy.h"
#include "../../fsd_logic/fsd_selftest.h"
#include "can_driver.h"
#include "led.h"
// FSD_NO_WIFI drops the radio, the dashboard and the HTTP CAN stream from the
// build entirely -- platformio.ini also removes the three .cpp files, so these
// headers would declare functions nothing implements.
#if !defined(FSD_NO_WIFI)
#include "wifi_manager.h"
#include "web_dashboard.h"
#endif
#include "can_dump.h"
#if !defined(FSD_NO_WIFI)
#include "http_can_stream.h"
#endif
#include "blackbox.h"
#include "camera_store.h"
#include "camera_task.h"
#include "body_task.h"
#include "ble_server.h"
#include "ble_owner.h"
#include "ble_central.h"
#include "capability.h"
#include "profile_match.h"
#include "../../fsd_logic/fsd_events.h"
#include "prefs.h"
#include "power_log.h"
#if defined(BOARD_TTGO_DISPLAY)
#include "display.h"
#endif

// ── Globals ───────────────────────────────────────────────────────────────────
#if defined(CAN_DRIVER_T2CAN_DUAL)
#define CAN_ACTIVE_BUS_COUNT 2u
#else
#define CAN_ACTIVE_BUS_COUNT 1u
#endif

static CanDriver *g_can[CAN_ACTIVE_BUS_COUNT] = {};
static bool       g_can_ok[CAN_ACTIVE_BUS_COUNT] = {};       // true once begin() succeeds
static uint32_t   g_can_last_retry_ms[CAN_ACTIVE_BUS_COUNT] = {}; // periodic re-init
#define CAN_REINIT_INTERVAL_MS  30000u

// ── OTA self-test ────────────────────────────────────────────────────────────
//
// 🔴 THIS OVERRIDE IS WHAT MAKES THE REST OF THE FILE RUN AT ALL.
//
// Arduino-ESP32 decides the rollback itself, in initArduino(), BEFORE setup()
// is ever called (cores/esp32/main.cpp: app_main -> initArduino, then loopTask
// -> setup). esp32-hal-misc.c declares two weak hooks:
//
//     bool verifyOta()            __attribute__((weak)) { return true;  }
//     bool verifyRollbackLater()  __attribute__((weak)) { return false; }
//
// and with the defaults it sees PENDING_VERIFY, asks verifyOta(), gets true,
// and calls esp_ota_mark_app_valid_cancel_rollback() on the spot. By the time
// setup() looks at the partition state it is already ESP_OTA_IMG_VALID, so
// g_ota_pending_verify was never set and ota_selftest_tick() returned
// immediately every time. The self-test below was dead code and SECURITY.md's
// claim that images are "accepted only after a self-test" was not true of the
// shipped binary.
//
// Returning true here tells the framework not to decide: the partition stays in
// PENDING_VERIFY and the decision is ours. It is only reachable when the SDK
// has CONFIG_APP_ROLLBACK_ENABLE, which this one does.
#ifdef CONFIG_APP_ROLLBACK_ENABLE
extern "C" bool verifyRollbackLater() { return true; }
#endif

// Set in setup() when we booted an image the bootloader has not accepted yet.
// Cleared by ota_selftest_tick() once the image has proved it runs; until then
// the previous firmware stays as the fallback and a reset restores it.
static bool     g_ota_pending_verify  = false;
static uint32_t g_ota_first_loop_ms   = 0;
static uint32_t g_ota_loops           = 0;
// Thresholds live in fsd_logic/fsd_selftest.h with the decision they belong to.
static FSDState   g_state = {};
static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

static CanBusId bus_id_from_index(uint8_t index) {
    return index == 1 ? CAN_BUS_SECONDARY : CAN_BUS_PRIMARY;
}

static CanBusId configured_bus_from_index(uint8_t index) {
    if (index >= CAN_ACTIVE_BUS_COUNT) index = 0u;
    return bus_id_from_index(index);
}

static uint8_t bus_index(CanBusId bus) {
    return bus == CAN_BUS_SECONDARY ? 1u : 0u;
}

static void state_enter() {
    portENTER_CRITICAL(&g_state_mux);
}

static void state_exit() {
    portEXIT_CRITICAL(&g_state_mux);
}

static FSDState state_snapshot() {
    FSDState s;
    state_enter();
    s = g_state;
    state_exit();
    return s;
}

static bool hw_uses_hw3_das_status(TeslaHWVersion hw) {
    return hw == TeslaHW_Legacy || hw == TeslaHW_HW3;
}

static bool hw_uses_hw4_das_status(TeslaHWVersion hw) {
    return hw == TeslaHW_HW4;
}

static bool frame_looks_like_hw3_das_status(const CanFrame &frame) {
    if (frame.id != CAN_ID_DAS_STATUS_HW3 || frame.dlc != CAN_FRAME_MAX_DATA_LEN) return false;

    uint8_t ap_state = frame.data[SIG_DAS_HW3_AP_STATE_BYTE] & SIG_DAS_HW3_AP_STATE_MASK;
    uint8_t hands_on =
        (frame.data[SIG_DAS_HANDS_ON_STATE_BYTE] >> SIG_DAS_HANDS_ON_STATE_SHIFT) &
        SIG_DAS_HANDS_ON_STATE_MASK;

    return ap_state <= SIG_DAS_HW3_AP_ACTIVE_STATE &&
           hands_on <= SIG_DAS_HANDS_ON_SUSPENDED;
}

static bool serial_cmd_equals(const char *cmd, const char *expected) {
    while (*cmd && *expected) {
        char a = *cmd++;
        char b = *expected++;
        if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
        if (a != b) return false;
    }
    return *cmd == '\0' && *expected == '\0';
}

static void serial_command_tick() {
    // 24 was too small the moment a command took an argument: "btnbind" plus a
    // 17-character BLE address is 25, and the terminator needs one more.
    static char buf[40];
    static uint8_t len = 0;

    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (len == 0) continue;
            buf[len] = '\0';
            len = 0;

            if (serial_cmd_equals(buf, "ip") || serial_cmd_equals(buf, "wifi")) {
#if defined(FSD_NO_WIFI)
                // Answer rather than ignore: the command is in the README and
                // silence would read as a hung serial port, not as a decision.
                Serial.println("[WiFi] not built into this firmware — BLE only");
#else
                wifi_print_status();
#endif
            } else if (serial_cmd_equals(buf, "bbon") ||
                       serial_cmd_equals(buf, "bboff")) {
                // 🔴 The recorder had no reachable switch at all: its only
                // caller outside a self-guarded boot path was the web
                // dashboard, removed in PR #28. Over USB is the right place for
                // the fallback -- if BLE is the problem, a BLE-only switch is
                // no switch.
                bool on = serial_cmd_equals(buf, "bbon");
                blackbox_set_enabled(on);
                FSDState saved = state_snapshot();
                prefs_save(&saved);
                Serial.printf("[BB] serial: %s (actually: %s)\n",
                              on ? "on" : "off",
                              blackbox_is_enabled() ? "recording" : "off");
            } else if (serial_cmd_equals(buf, "mark")) {
                if (!blackbox_is_enabled()) {
                    Serial.println("[BB] not recording — run 'bbon' first");
                } else {
                    uint32_t before = blackbox_capture_count();
                    blackbox_mark(millis());
                    Serial.printf("[BB] mark — captures %u -> %u\n",
                                  (unsigned)before,
                                  (unsigned)blackbox_capture_count());
                }
            } else if (serial_cmd_equals(buf, "owner")) {
                ble_owner_print();
            } else if (serial_cmd_equals(buf, "ownerclear")) {
                // Deliberately reachable only over USB. A lost or replaced
                // phone must not brick the module, and the recovery must need
                // physical access — the same bar as the button.
                ble_owner_forget();
            } else if (serial_cmd_equals(buf, "ownerpair")) {
                ble_owner_open_window(millis());
            } else if (serial_cmd_equals(buf, "btnscan")) {
                // Bring-up: look before writing a parser for a device nobody has.
                if (!ble_central_scan(5)) Serial.println("[SER] scan already running");
            } else if (strncmp(buf, "btnbind", 7) == 0) {
                const char *arg = buf + 7;
                while (*arg == ' ') arg++;
                ble_central_bind(arg); // empty argument forgets the button
            } else if (serial_cmd_equals(buf, "btnstat")) {
                Serial.printf("[BTN] bound:%s connected:%s notifies:%u short:%u long:%u\n",
                              ble_central_bound_addr()[0] ? ble_central_bound_addr() : "(none)",
                              ble_central_connected() ? "yes" : "no",
                              (unsigned)ble_central_notify_count(),
                              (unsigned)ble_central_short_presses(),
                              (unsigned)ble_central_long_presses());
            } else if (serial_cmd_equals(buf, "help") || serial_cmd_equals(buf, "?")) {
                Serial.println("[SER] Commands: ip | btnscan | btnbind <addr> | btnstat");
                Serial.println("[SER]   bbon / bboff  — capture recorder on/off (persisted)");
                Serial.println("[SER]   mark          — record a window around NOW");
                Serial.println("[SER]   owner / ownerpair / ownerclear");
            } else {
                Serial.println("[SER] Unknown command. Type: help");
            }
            continue;
        }

        if (c < 32 || c > 126) continue;
        if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            len = 0;
            Serial.println("[SER] Command too long");
        }
    }
}

static void can_set_all_listen_only(bool listen_only) {
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (g_can[i]) g_can[i]->setListenOnly(listen_only);
    }
}

// Did every initialised controller actually reach the register state we asked
// for? isListenOnly() is not a wish: both drivers only update it after the mode
// change reports success, and an uninstalled MCP2515 reports listen-only, which
// is the safe direction.
static bool can_mode_settled(bool listen_only) {
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (!g_can_ok[i] || !g_can[i]) continue;   // nothing to check on this one
        if (g_can[i]->isListenOnly() != listen_only) return false;
    }
    return true;
}

// A controller that would not take the requested mode has failed its reinstall:
// setListenOnly() uninstalls, then install_and_start() did not bring it back. It
// is now deaf as well as mute, and nothing else notices — g_can_ok is written by
// begin() alone, so the periodic re-init loop below would skip that bus forever
// and it would stay dead for the rest of the session.
//
// Clearing the flag hands it to that loop, which calls begin() again on an
// interval and restores it (and the flag) if the hardware comes back.
static void can_mark_unsettled(bool want_listen_only) {
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (!g_can_ok[i] || !g_can[i]) continue;
        if (g_can[i]->isListenOnly() != want_listen_only) {
            g_can_ok[i] = false;
            Serial.printf("[CAN] %s did not take the mode — flagged for re-init\n",
                          can_bus_name(bus_id_from_index(i)));
        }
    }
}

// The one way to change op_mode. See mode_switch.h for why it exists.
//
// 🔴 Loop task only.
bool mode_apply(OpMode m) {
    // fsd_mode_opens_hw_tx(), not fsd_mode_opens_tx(): this decides the REGISTER.
    // Same value today; separate names so that opening the register for a future
    // Autonomous scroll detent does not also open general TX. See fsd_types.h.
    const bool want_listen_only = !fsd_mode_opens_hw_tx(m);

    // The rule both branches follow: the software gate is never more permissive
    // than the hardware. So the direction that RESTRICTS moves first.
    if (!want_listen_only) {
        // Opening. Register first -- if it will not move we must not raise
        // op_mode, or we advertise a permission the wire cannot honour. That is
        // precisely the bug this function exists to remove.
        can_set_all_listen_only(false);
        if (!can_mode_settled(false)) {
            Serial.println("[MODE] 컨트롤러가 Listen-Only 를 못 벗어났다 — 모드를 바꾸지 않는다");
            can_set_all_listen_only(true);   // never leave it half-open
            can_mark_unsettled(true);        // whichever one is stuck, re-init it
            return false;
        }
    }

    portENTER_CRITICAL(&g_state_mux);
    g_state.op_mode = m;
    portEXIT_CRITICAL(&g_state_mux);

    if (want_listen_only) {
        // Closing. State first, so the software gate shuts immediately; the
        // reverse order would leave a moment where op_mode still said Active.
        can_set_all_listen_only(true);
        if (!can_mode_settled(true)) {
            Serial.println("[MODE] 경고: 컨트롤러가 Listen-Only 로 안 들어갔다");
            can_mark_unsettled(true);
            return false;
        }
    }
    return true;
}

OpMode mode_current(void) {
    portENTER_CRITICAL(&g_state_mux);
    OpMode m = g_state.op_mode;
    portEXIT_CRITICAL(&g_state_mux);
    return m;
}

static bool can_any_ok() {
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (g_can_ok[i]) return true;
    }
    return false;
}

static uint32_t can_total_error_count() {
    uint32_t total = 0;
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (g_can[i]) total += g_can[i]->errorCount();
    }
    return total;
}

static uint32_t can_total_tx_count() {
    uint32_t total = 0;
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (g_can[i]) total += g_can[i]->txCount();
    }
    return total;
}

static CanDriver *can_for_bus(CanBusId bus) {
    uint8_t index = bus_index(bus);
    if (index >= CAN_ACTIVE_BUS_COUNT) return nullptr;
    return g_can[index];
}

static bool send_on_bus(CanBusId bus, const CanFrame &frame) {
    // Body-control IDs never leave this module, in any mode, under any flag.
    // Nothing constructs one — there is no emitter in the whole feature — so
    // this is a backstop against a future call site, not a policy that can be
    // switched. It sits at the single chokepoint because the eight gates that
    // guard TX live at the CALL sites, and a ninth call site could forget one.
    if (body_task_tx_refused(frame.id)) {
        Serial.printf("[BODY] refused TX of 0x%03X — this firmware sends no body frames\n",
                      (unsigned)frame.id);
        return false;
    }

    // 🔴 0x229 — the right stalk, i.e. GEAR SELECTION. Refused here for the same
    // reason as the body IDs: this board has an emitter for it and no business
    // using it.
    //
    // arm_gear_ap_double_press_sequence() really does transmit FULL_DOWN /
    // CENTER / FULL_DOWN / CENTER on 0x229 to re-engage AP, gated only by
    // fsd_can_transmit() and continuous_ap. CLAUDE.md calls 0x229 a "펌웨어 TX
    // denylist" entry, but that denial lives in fsd_profile_id_blocked() — which
    // serves the text-profile parser, and fsd_profile.c is not even in this
    // variant's build_src_filter. The documented protection did not exist here.
    //
    // continuous_ap has no way to be switched on in this build (no BLE command,
    // no serial command, dashboard removed) — but its NVS key "contap" survives
    // reflashing, so a board that once ran a WiFi build with it enabled boots
    // with it true. Cheap backstop, real path.
    if (frame.id == CAN_ID_SCCM_RSTALK) {
        Serial.printf("[TX] refused 0x%03X — gear-lever frames are not sent by this board\n",
                      (unsigned)frame.id);
        return false;
    }

    CanDriver *driver = can_for_bus(bus);
    bool ok = driver ? driver->send(frame) : false;
    // Single TX chokepoint: every injected/modified frame (0x3EE, 0x3FD, the
    // 0x370 nag echo, generated + modified wrappers) routes through here. Record
    // the ones we actually put on the bus as TX so the black-box capture shows
    // our frames alongside the RX bus — same id filter as RX, cheap ring write,
    // no-op when the recorder is off. Recording never gates the send.
    if (ok) blackbox_record_tx(bus, frame, millis());
    return ok;
}

static bool send_generated_frame(CanBusId bus, const CanFrame &frame) {
    return send_on_bus(bus, frame);
}

static bool send_modified_frame(CanBusId bus, const CanFrame &frame) {
    return send_on_bus(bus, frame);
}

static uint8_t g_last_gear_counter = 0;
static bool g_last_gear_counter_valid = false;
static uint32_t g_last_gear_counter_ms = 0;

enum ContinuousApFlowState : uint8_t {
    ContAp_Idle = 0,
    ContAp_WaitSignalOff,
    ContAp_WaitApReady,
    ContAp_Attempting,
};

static ContinuousApFlowState g_cont_ap_state = ContAp_Idle;
static uint32_t g_cont_ap_signal_off_ms = 0;
static uint32_t g_cont_ap_attempt_ms = 0;
static uint32_t g_cont_ap_torque_high_ms = 0;
static uint32_t g_cont_ap_last_brake_ms = 0;
static uint32_t g_cont_ap_last_stalk_full_up_ms = 0;
static uint8_t g_cont_ap_attempts = 0;
static bool g_cont_ap_last_ap_active = false;

static constexpr uint8_t GEAR_SEQUENCE_MAX = 4;
static uint8_t g_gear_sequence[GEAR_SEQUENCE_MAX] = {};
static uint8_t g_gear_sequence_len = 0;
static uint8_t g_gear_sequence_index = 0;
static uint32_t g_gear_sequence_progress_ms = 0;
static uint32_t g_gear_sequence_next_ms = 0;
static bool g_gear_sequence_counter_valid = false;
static uint8_t g_gear_sequence_counter = 0;
static uint32_t g_gear_sequence_send_fail_count = 0;

enum GearSequenceSendResult : uint8_t {
    GearSeqSend_Sent = 0,
    GearSeqSend_Inactive,
    GearSeqSend_WaitStep,
    GearSeqSend_WaitCounter,
    GearSeqSend_TxBlocked,
    GearSeqSend_BuildFailed,
    GearSeqSend_TxFailed,
};

static CanBusId preferred_generated_bus(uint32_t frame_id) {
    if (frame_id == CAN_ID_SCCM_RSTALK)
        return configured_bus_from_index(GEAR_LEVER_TX_BUS_INDEX);
    return CAN_BUS_PRIMARY;
}

static bool gear_sequence_active() {
    return g_gear_sequence_index < g_gear_sequence_len;
}

static void clear_gear_sequence() {
    g_gear_sequence_len = 0;
    g_gear_sequence_index = 0;
    g_gear_sequence_progress_ms = 0;
    g_gear_sequence_next_ms = 0;
    g_gear_sequence_counter_valid = false;
    g_gear_sequence_counter = 0;
    g_gear_sequence_send_fail_count = 0;
}

static bool cached_gear_counter_is_fresh(uint32_t now) {
    return g_last_gear_counter_valid &&
           g_last_gear_counter_ms != 0u &&
           (uint32_t)(now - g_last_gear_counter_ms) <= GEAR_LEVER_CACHED_COUNTER_MAX_AGE_MS;
}

static const char *gear_sequence_send_result_name(GearSequenceSendResult result) {
    switch (result) {
        case GearSeqSend_Sent:        return "sent";
        case GearSeqSend_Inactive:    return "inactive";
        case GearSeqSend_WaitStep:    return "wait_step";
        case GearSeqSend_WaitCounter: return "wait_counter";
        case GearSeqSend_TxBlocked:   return "tx_blocked";
        case GearSeqSend_BuildFailed: return "build_failed";
        case GearSeqSend_TxFailed:    return "tx_failed";
        default:                      return "?";
    }
}

static GearSequenceSendResult send_gear_position_for_sequence(uint8_t gear_pos,
                                                              const char *name) {
    (void)name;
    uint32_t now = millis();
    FSDState s = state_snapshot();
    if (!fsd_can_transmit(&s)) return GearSeqSend_TxBlocked;

    if (!g_gear_sequence_counter_valid) {
        if (!cached_gear_counter_is_fresh(now)) return GearSeqSend_WaitCounter;
        g_gear_sequence_counter = g_last_gear_counter;
        g_gear_sequence_counter_valid = true;
    }

    CanFrame f;
    uint8_t next_counter =
        (uint8_t)((g_gear_sequence_counter + 1u) & SIG_GEAR_LEVER_COUNTER_MASK);
    if (!fsd_build_gear_lever_frame(&f, gear_pos, next_counter)) {
        return GearSeqSend_BuildFailed;
    }

    bool sent = send_generated_frame(preferred_generated_bus(CAN_ID_SCCM_RSTALK), f);
    if (sent) {
        g_gear_sequence_counter = next_counter;
        g_last_gear_counter = next_counter;
        g_last_gear_counter_ms = now;
        g_last_gear_counter_valid = true;
        return GearSeqSend_Sent;
    }
    return GearSeqSend_TxFailed;
}

static GearSequenceSendResult send_next_gear_sequence_frame(const char *name) {
    if (!gear_sequence_active()) return GearSeqSend_Inactive;
    uint8_t gear_pos = g_gear_sequence[g_gear_sequence_index];
    GearSequenceSendResult result = send_gear_position_for_sequence(gear_pos, name);
    if (result != GearSeqSend_Sent) {
        g_gear_sequence_send_fail_count++;
        return result;
    }
    g_gear_sequence_index++;
    uint32_t now = millis();
    g_gear_sequence_progress_ms = now;
    if (!gear_sequence_active()) clear_gear_sequence();
    else g_gear_sequence_next_ms = now + GEAR_SEQUENCE_STEP_MS;
    return GearSeqSend_Sent;
}

static bool gear_sequence_timed_out(uint32_t now) {
    return gear_sequence_active() &&
           g_gear_sequence_progress_ms != 0u &&
           (uint32_t)(now - g_gear_sequence_progress_ms) > GEAR_SEQUENCE_TIMEOUT_MS;
}

static GearSequenceSendResult gear_sequence_tick(uint32_t now, const char *name) {
    if (!gear_sequence_active()) return GearSeqSend_Inactive;
    if (g_gear_sequence_next_ms != 0u &&
        (int32_t)(now - g_gear_sequence_next_ms) < 0) {
        return GearSeqSend_WaitStep;
    }
    return send_next_gear_sequence_frame(name);
}

static GearSequenceSendResult arm_gear_ap_double_press_sequence(uint32_t now) {
    g_gear_sequence[0] = SIG_GEAR_LEVER_FULL_DOWN;
    g_gear_sequence[1] = SIG_GEAR_LEVER_CENTER;
    g_gear_sequence[2] = SIG_GEAR_LEVER_FULL_DOWN;
    g_gear_sequence[3] = SIG_GEAR_LEVER_CENTER;
    g_gear_sequence_len = 4;
    g_gear_sequence_index = 0;
    g_gear_sequence_progress_ms = now;
    g_gear_sequence_next_ms = now;
    return gear_sequence_tick(now, "CONT-AP");
}

#if defined(CAN_DRIVER_T2CAN_DUAL)
static void debug_log_bus_stats() {
    Serial.printf("[CAN] RX can0=%lu can1=%lu TX can0=%lu can1=%lu Err can0=%lu can1=%lu\n",
                  (unsigned long)(g_can[0] ? g_can[0]->rxCount() : 0),
                  (unsigned long)(g_can[1] ? g_can[1]->rxCount() : 0),
                  (unsigned long)(g_can[0] ? g_can[0]->txCount() : 0),
                  (unsigned long)(g_can[1] ? g_can[1]->txCount() : 0),
                  (unsigned long)(g_can[0] ? g_can[0]->errorCount() : 0),
                  (unsigned long)(g_can[1] ? g_can[1]->errorCount() : 0));
}
#endif

static const char *hw_to_str(TeslaHWVersion hw) {
    switch (hw) {
        case TeslaHW_HW4:    return "HW4";
        case TeslaHW_HW3:    return "HW3";
        case TeslaHW_Legacy: return "Legacy";
        default:             return "?";
    }
}

static uint8_t epas_hands_on_level(const CanFrame &frame) {
    return (frame.data[SIG_EPAS_HANDS_ON_BYTE] >> SIG_EPAS_HANDS_ON_SHIFT) &
           SIG_EPAS_HANDS_ON_MASK;
}

static uint8_t epas_counter(const CanFrame &frame) {
    return frame.data[SIG_EPAS_COUNTER_BYTE] & SIG_EPAS_COUNTER_MASK;
}

static void debug_log_das_status(CanBusId bus, uint32_t source_id, const FSDState &state) {
    static bool initialized = false;
    static uint32_t last_ms = 0;
    static CanBusId last_bus = CAN_BUS_PRIMARY;
    static uint32_t last_source_id = 0;
    static TeslaHWVersion last_hw = TeslaHW_Unknown;
    static bool last_ap_active = false;
    static uint8_t last_ap_state = 0;
    static uint8_t last_hands_state = 0;

    uint32_t now = millis();
    bool changed =
        !initialized ||
        last_bus != bus ||
        last_source_id != source_id ||
        last_hw != state.hw_version ||
        last_ap_active != state.ap_active ||
        last_ap_state != state.das_ap_state ||
        last_hands_state != state.das_hands_on_state;

    if (!changed && (now - last_ms) < 5000u) return;

    Serial.printf("[DAS] bus=%s src=0x%03lX hw=%s ap=%s ap_state=%u hands=%u lane=%u cnt=%u chk=0x%02X\n",
                  can_bus_name(bus),
                  (unsigned long)source_id,
                  hw_to_str(state.hw_version),
                  state.ap_active ? "ON" : "wait",
                  state.das_ap_state,
                  state.das_hands_on_state,
                  state.das_lane_change_state,
                  state.das_counter,
                  state.das_checksum);

    initialized = true;
    last_ms = now;
    last_bus = bus;
    last_source_id = source_id;
    last_hw = state.hw_version;
    last_ap_active = state.ap_active;
    last_ap_state = state.das_ap_state;
    last_hands_state = state.das_hands_on_state;
}

static void debug_log_bms_seen(CanBusId bus, uint32_t frame_id, const FSDState &state) {
    static bool seen[CAN_ACTIVE_BUS_COUNT][3] = {};
    uint8_t bus_i = bus_index(bus);
    uint8_t id_i;
    if (frame_id == CAN_ID_BMS_HV_BUS) {
        id_i = 0;
    } else if (frame_id == CAN_ID_BMS_SOC) {
        id_i = 1;
    } else if (frame_id == CAN_ID_BMS_THERMAL) {
        id_i = 2;
    } else {
        return;
    }
    if (bus_i >= CAN_ACTIVE_BUS_COUNT || seen[bus_i][id_i]) return;
    seen[bus_i][id_i] = true;

    Serial.printf("[BMS] first %s frame=0x%03lX hv=%lu soc=%lu thermal=%lu\n",
                  can_bus_name(bus),
                  (unsigned long)frame_id,
                  (unsigned long)state.seen_bms_hv,
                  (unsigned long)state.seen_bms_soc,
                  (unsigned long)state.seen_bms_thermal);
}

typedef enum {
    NagDebug_Disabled = 0,
    NagDebug_ApInactive,
    NagDebug_HandsOk,
    NagDebug_DasSatisfied,
    NagDebug_NotFired,
    NagDebug_TxBlocked,
    NagDebug_BuiltNoTx,
    NagDebug_TxFailed,
    NagDebug_TxEcho,
} NagDebugReason;

static const char *nag_debug_reason_name(NagDebugReason reason) {
    switch (reason) {
        case NagDebug_Disabled:     return "disabled";
        case NagDebug_ApInactive:   return "ap_inactive";
        case NagDebug_HandsOk:      return "hands_ok";
        case NagDebug_DasSatisfied: return "das_satisfied";
        case NagDebug_NotFired:     return "not_fired";
        case NagDebug_TxBlocked:    return "tx_blocked";
        case NagDebug_BuiltNoTx:    return "built_no_tx";
        case NagDebug_TxFailed:     return "tx_failed";
        case NagDebug_TxEcho:       return "tx_echo";
        default:                    return "?";
    }
}

static NagDebugReason nag_debug_reason(const FSDState &state,
                                       uint8_t epas_hands,
                                       bool fired,
                                       bool tx_allowed,
                                       bool sent) {
    if (fired) {
        if (sent) return NagDebug_TxEcho;
        return tx_allowed ? NagDebug_TxFailed : NagDebug_BuiltNoTx;
    }
    if (!tx_allowed) return NagDebug_TxBlocked;
    if (!state.nag_killer) return NagDebug_Disabled;
    if (!state.ap_active) return NagDebug_ApInactive;
    if (epas_hands == SIG_EPAS_HANDS_ON_OK) return NagDebug_HandsOk;
    if (state.das_seen &&
        (state.das_hands_on_state == SIG_DAS_HANDS_ON_NOT_REQUIRED ||
         state.das_hands_on_state == SIG_DAS_HANDS_ON_SUSPENDED)) {
        return NagDebug_DasSatisfied;
    }
    return NagDebug_NotFired;
}

static void debug_log_nag_decision(CanBusId bus,
                                   const CanFrame &frame,
                                   const CanFrame &echo,
                                   bool fired,
                                   bool tx_allowed,
                                   bool sent,
                                   const FSDState &before,
                                   const FSDState &after) {
    static bool initialized = false;
    static uint32_t last_ms = 0;
    static CanBusId last_bus = CAN_BUS_PRIMARY;
    static NagDebugReason last_reason = NagDebug_NotFired;
    static uint8_t last_epas_hands = 0xFFu;
    static uint8_t last_das_hands = 0xFFu;
    static bool last_ap_active = false;

    uint32_t now = millis();
    uint8_t hands = epas_hands_on_level(frame);
    NagDebugReason reason = nag_debug_reason(before, hands, fired, tx_allowed, sent);
    bool nag_relevant = fired || hands != SIG_EPAS_HANDS_ON_OK;
    bool changed =
        !initialized ||
        last_bus != bus ||
        last_reason != reason ||
        last_epas_hands != hands ||
        last_das_hands != before.das_hands_on_state ||
        last_ap_active != before.ap_active;

    if (!nag_relevant && !changed) return;
    if (!changed && (now - last_ms) < 1000u) return;

    if (fired) {
        Serial.printf("[NAG] bus=%s %s epas_lvl=%u cnt=%u tx=%u ap=%u das_seen=%u das=%u echo_cnt=%u echo_chk=0x%02X echoes=%lu\n",
                      can_bus_name(bus),
                      nag_debug_reason_name(reason),
                      hands,
                      epas_counter(frame),
                      tx_allowed ? 1 : 0,
                      before.ap_active ? 1 : 0,
                      before.das_seen ? 1 : 0,
                      before.das_hands_on_state,
                      epas_counter(echo),
                      echo.data[7],
                      (unsigned long)after.nag_echo_count);
    } else {
        Serial.printf("[NAG] bus=%s skip=%s epas_lvl=%u cnt=%u tx=%u ap=%u das_seen=%u das=%u echoes=%lu\n",
                      can_bus_name(bus),
                      nag_debug_reason_name(reason),
                      hands,
                      epas_counter(frame),
                      tx_allowed ? 1 : 0,
                      before.ap_active ? 1 : 0,
                      before.das_seen ? 1 : 0,
                      before.das_hands_on_state,
                      (unsigned long)after.nag_echo_count);
    }

    initialized = true;
    last_ms = now;
    last_bus = bus;
    last_reason = reason;
    last_epas_hands = hands;
    last_das_hands = before.das_hands_on_state;
    last_ap_active = before.ap_active;
}

static void apply_detected_hw(TeslaHWVersion hw, const char *reason) {
    if (hw == TeslaHW_Unknown) return;
    state_enter();
    // Manual HW selection wins (#110): once the owner has pinned a version,
    // auto-detection must never move it — detection can only guess on taps that
    // carry no 0x398, and overriding a deliberate choice is what breaks setups.
    if (g_state.hw_override != TeslaHW_Unknown) {
        state_exit();
        return;
    }
    if (g_state.hw_version == hw) {
        state_exit();
        return;
    }
    fsd_apply_hw_version(&g_state, hw);
    state_exit();

    const char *hw_str =
        (hw == TeslaHW_HW4) ? "HW4" :
        (hw == TeslaHW_HW3) ? "HW3" : "Legacy";
    Serial.printf("[HW] Auto-detected: %s (%s)\n", hw_str, reason);
    can_dump_log("HW  auto-detected: %s (%s)", hw_str, reason);
}

static const char *continuous_ap_state_name(ContinuousApFlowState state) {
    switch (state) {
        case ContAp_WaitSignalOff: return "wait_signal_off";
        case ContAp_WaitApReady:   return "wait_ap_ready";
        case ContAp_Attempting:    return "attempting";
        case ContAp_Idle:
        default:                   return "idle";
    }
}

static void continuous_ap_reset(const char *reason) {
    if (g_cont_ap_state != ContAp_Idle || g_cont_ap_attempts != 0u) {
        Serial.printf("[CONT-AP] stop: %s (state=%s attempts=%u)\n",
                      reason,
                      continuous_ap_state_name(g_cont_ap_state),
                      (unsigned)g_cont_ap_attempts);
    }
    g_cont_ap_state = ContAp_Idle;
    g_cont_ap_signal_off_ms = 0u;
    g_cont_ap_attempt_ms = 0u;
    g_cont_ap_torque_high_ms = 0u;
    g_cont_ap_attempts = 0u;
    clear_gear_sequence();
}

static bool continuous_ap_turn_signal_active(const FSDState &s) {
    return s.turn_status_seen && (s.left_turn_active || s.right_turn_active);
}

static bool continuous_ap_turn_signal_off(const FSDState &s) {
    return s.turn_status_seen && !s.left_turn_active && !s.right_turn_active;
}

static bool continuous_ap_steering_torque_high(const FSDState &s) {
    if (!s.torsion_bar_torque_seen) return false;
    return s.torsion_bar_torque_nm >= CONT_AP_STEERING_TORQUE_ABORT_NM ||
           s.torsion_bar_torque_nm <= -CONT_AP_STEERING_TORQUE_ABORT_NM;
}

static bool continuous_ap_torque_allows(uint32_t now, const FSDState &s) {
    if (!continuous_ap_steering_torque_high(s)) {
        g_cont_ap_torque_high_ms = 0u;
        return true;
    }

    if (g_cont_ap_torque_high_ms == 0u) {
        g_cont_ap_torque_high_ms = now;
        if (g_cont_ap_state != ContAp_Idle) {
            Serial.printf("[CONT-AP] steering torque high %.2f Nm; waiting\n",
                          s.torsion_bar_torque_nm);
        }
    }

    if (g_cont_ap_state != ContAp_Idle &&
        (uint32_t)(now - g_cont_ap_torque_high_ms) > CONT_AP_STEERING_TORQUE_TIMEOUT_MS) {
        Serial.printf("[CONT-AP] steering torque high timeout %.2f Nm\n",
                      s.torsion_bar_torque_nm);
        continuous_ap_reset("steering torque high");
    }
    return false;
}

static bool continuous_ap_brake_recent(uint32_t now, const FSDState &s) {
    if (s.driver_brake_applied) return true;
    return g_cont_ap_last_brake_ms != 0u &&
           (uint32_t)(now - g_cont_ap_last_brake_ms) <= CONT_AP_BRAKE_RECENT_MS;
}

static bool continuous_ap_brake_allows(uint32_t now, const FSDState &s) {
    // Fail closed: the brake kill switch reads 0x145 (ESP_status). Until at least
    // one 0x145 frame is seen we cannot prove the pedal is released, so refuse to
    // engage rather than silently dropping the interlock on a bus without 0x145.
    if (!s.brake_status_seen) {
        if (g_cont_ap_state != ContAp_Idle) {
            Serial.println("[CONT-AP] brake status (0x145) not seen; refusing to engage");
            continuous_ap_reset("brake status unseen");
        }
        return false;
    }
    if (!continuous_ap_brake_recent(now, s)) return true;

    if (g_cont_ap_state != ContAp_Idle) {
        Serial.println("[CONT-AP] brake pedal kill switch");
        continuous_ap_reset("brake pedal");
    }
    return false;
}

static bool continuous_ap_stalk_stop_recent(uint32_t now, const FSDState &s) {
    uint32_t last = s.stalk_full_up_ms;
    if (g_cont_ap_last_stalk_full_up_ms != 0u &&
        (last == 0u || (int32_t)(g_cont_ap_last_stalk_full_up_ms - last) > 0)) {
        last = g_cont_ap_last_stalk_full_up_ms;
    }
    return last != 0u && (uint32_t)(now - last) <= CONT_AP_STALK_STOP_RECENT_MS;
}

static bool continuous_ap_stalk_stop_allows(uint32_t now, const FSDState &s) {
    if (!continuous_ap_stalk_stop_recent(now, s)) return true;

    if (g_cont_ap_state != ContAp_Idle) {
        Serial.println("[CONT-AP] right stalk full-up kill switch");
        continuous_ap_reset("right stalk full-up");
    }
    return false;
}

static bool continuous_ap_hw3_legacy_try_start_attempt(uint32_t now) {
    if (!cached_gear_counter_is_fresh(now)) {
        return false;
    }

    GearSequenceSendResult result = arm_gear_ap_double_press_sequence(now);
    if (result != GearSeqSend_Sent) {
        clear_gear_sequence();
        return false;
    }

    g_cont_ap_attempts++;
    g_cont_ap_attempt_ms = now;
    g_cont_ap_state = ContAp_Attempting;
    Serial.printf("[CONT-AP] attempt %u/%u: 0x229 full-down double press started on %s\n",
                  (unsigned)g_cont_ap_attempts,
                  (unsigned)CONT_AP_MAX_RETRIES,
                  can_bus_name(preferred_generated_bus(CAN_ID_SCCM_RSTALK)));
    return true;
}

static void continuous_ap_tick_hw3_legacy(uint32_t now,
                                          const FSDState &s,
                                          bool ap_disabled_now) {
    bool torque_allows = continuous_ap_torque_allows(now, s);
    bool brake_allows = continuous_ap_brake_allows(now, s);
    bool stalk_stop_allows = continuous_ap_stalk_stop_allows(now, s);

    if (ap_disabled_now) {
        Serial.printf("[CONT-AP] AP disabled torque=%s%.2f Nm brake=%u stalk_stop=%u turn_active=%u left=%u right=%u\n",
                      s.torsion_bar_torque_seen ? "" : "unseen:",
                      s.torsion_bar_torque_seen ? s.torsion_bar_torque_nm : 0.0f,
                      continuous_ap_brake_recent(now, s) ? 1u : 0u,
                      continuous_ap_stalk_stop_recent(now, s) ? 1u : 0u,
                      continuous_ap_turn_signal_active(s) ? 1u : 0u,
                      s.left_turn_active ? 1u : 0u,
                      s.right_turn_active ? 1u : 0u);
    }

    if (ap_disabled_now && continuous_ap_turn_signal_active(s)) {
        if (!brake_allows) {
            Serial.println("[CONT-AP] AP disabled with turn signal active, but brake pedal was pressed");
            return;
        }
        if (!stalk_stop_allows) {
            Serial.println("[CONT-AP] AP disabled with turn signal active, but right stalk full-up was pressed");
            return;
        }
        if (!torque_allows) {
            Serial.printf("[CONT-AP] AP disabled with turn signal active, but steering torque is high %.2f Nm\n",
                          s.torsion_bar_torque_nm);
            return;
        }
        g_cont_ap_state = ContAp_WaitSignalOff;
        g_cont_ap_signal_off_ms = 0u;
        g_cont_ap_attempt_ms = 0u;
        g_cont_ap_attempts = 0u;
        clear_gear_sequence();
        Serial.println("[CONT-AP] AP disabled while turn signal active; waiting for turn signal off");
    }

    switch (g_cont_ap_state) {
        case ContAp_Idle:
            return;

        case ContAp_WaitSignalOff:
            if (s.ap_active) {
                continuous_ap_reset("AP already active");
                return;
            }
            if (!brake_allows) return;
            if (!stalk_stop_allows) return;
            if (!torque_allows) return;
            if (continuous_ap_turn_signal_off(s)) {
                g_cont_ap_signal_off_ms = now;
                g_cont_ap_state = ContAp_WaitApReady;
                Serial.printf("[CONT-AP] turn signal off; waiting %lu ms before AP reengage\n",
                              (unsigned long)CONT_AP_REENGAGE_DELAY_MS);
            }
            return;

        case ContAp_WaitApReady:
            if (s.ap_active) {
                continuous_ap_reset("AP already active");
                return;
            }
            if (g_cont_ap_attempts == 0u &&
                (uint32_t)(now - g_cont_ap_signal_off_ms) > CONT_AP_READY_WAIT_TIMEOUT_MS) {
                continuous_ap_reset("AP ready/counter timeout");
                return;
            }
            if (!brake_allows) return;
            if (!stalk_stop_allows) return;
            if (!torque_allows) return;
            if ((uint32_t)(now - g_cont_ap_signal_off_ms) < CONT_AP_REENGAGE_DELAY_MS) {
                return;
            }
            if (s.ap_ready) {
                continuous_ap_hw3_legacy_try_start_attempt(now);
            }
            return;

        case ContAp_Attempting:
            if (!brake_allows) return;
            if (!stalk_stop_allows) return;
            if (!torque_allows) return;
            {
                GearSequenceSendResult result = gear_sequence_tick(now, "CONT-AP");
                uint32_t sequence_now = millis();
                if (gear_sequence_timed_out(sequence_now)) {
                    Serial.printf("[CONT-AP] 0x229 sequence timeout after %u/%u frames (last=%s fails=%lu)\n",
                                  (unsigned)g_gear_sequence_index,
                                  (unsigned)g_gear_sequence_len,
                                  gear_sequence_send_result_name(result),
                                  (unsigned long)g_gear_sequence_send_fail_count);
                    clear_gear_sequence();
                }
            }
            if (gear_sequence_active()) {
                return;
            }
            if (s.ap_active) {
                continuous_ap_reset("AP active");
                return;
            }
            uint32_t attempt_elapsed = now - g_cont_ap_attempt_ms;
            if (attempt_elapsed < CONT_AP_ATTEMPT_RESULT_MS) {
                return;
            }
            if (g_cont_ap_attempts >= CONT_AP_MAX_RETRIES) {
                continuous_ap_reset("retry limit");
                return;
            }
            if (attempt_elapsed < (CONT_AP_ATTEMPT_RESULT_MS + CONT_AP_RETRY_DELAY_MS)) {
                return;
            }
            if (!s.ap_ready) {
                g_cont_ap_state = ContAp_WaitApReady;
                return;
            }
            continuous_ap_hw3_legacy_try_start_attempt(now);
            return;
    }
}

static void continuous_ap_hw4_start_attempt(uint32_t now) {
    (void)now;
    // TODO: implement HW4 re-engage frame sequence once HW4 control signals are identified.
}

static bool continuous_ap_hw4_reengage_allowed(uint32_t now, const FSDState &s) {
    (void)now;
    (void)s;
    // TODO: implement HW4-specific kill switches and preconditions.
    return false;
}

static void continuous_ap_tick_hw4(uint32_t now,
                                   const FSDState &s,
                                   bool ap_disabled_now) {
    // TODO: implement HW4 Continuous AP using HW4 turn/AP controls instead of stalk frames.
    if (ap_disabled_now &&
        continuous_ap_turn_signal_active(s) &&
        continuous_ap_hw4_reengage_allowed(now, s)) {
        continuous_ap_hw4_start_attempt(now);
    }
}

static void continuous_ap_tick(uint32_t now) {
    FSDState s = state_snapshot();
    bool ap_disabled_now = g_cont_ap_last_ap_active && !s.ap_active;
    g_cont_ap_last_ap_active = s.ap_active;

    if (!s.continuous_ap) {
        continuous_ap_reset("disabled");
        return;
    }

    if (!fsd_can_transmit(&s)) {
        continuous_ap_reset("TX disabled");
        return;
    }

    switch (s.hw_version) {
        case TeslaHW_HW4:
            continuous_ap_tick_hw4(now, s, ap_disabled_now);
            return;
        case TeslaHW_HW3:
        case TeslaHW_Legacy:
            continuous_ap_tick_hw3_legacy(now, s, ap_disabled_now);
            return;
        case TeslaHW_Unknown:
        default:
            continuous_ap_reset("HW unknown");
            return;
    }
}

// ── Button state machine ──────────────────────────────────────────────────────
static uint32_t g_btn_down_ms     = 0;
static uint32_t g_last_release_ms = 0;
static bool     g_btn_down        = false;
static int      g_pending_clicks  = 0;
static bool     g_long_fired      = false;  // prevent double-fire on long press
static bool     g_btn_ignore_boot = true;   // wait for release after boot
static bool     g_factory_reset_window   = false;  // set true on clean boot, clears at 20s
static bool     g_factory_reset_eligible = false;  // latched at leading edge if press was in window
static bool     g_factory_reset_armed    = false;  // blink done, waiting for release

#if defined(BOARD_TTGO_DISPLAY)
static uint32_t g_display_last_wake_ms = 0;
static bool     g_last_fsd_enabled     = false;

static uint32_t g_btn2_down_ms     = 0;
static uint32_t g_btn2_release_ms  = 0;
static bool     g_btn2_down        = false;
static bool     g_btn2_ignore_boot = true;
#endif

#if defined(BOARD_LILYGO)
static uint32_t g_last_can_rx_ms = 0;
static bool     g_sleep_warned   = false;
#endif

static void dispatch_clicks(int n) {
    if (n == 1) {
        // Toggle Listen-Only ↔ Active, through the one API. Only report and
        // persist what actually happened: a controller that refuses the switch
        // used to leave the log and NVS claiming a mode the hardware was not in.
        bool want_active = (mode_current() != OpMode_Active);
        if (!mode_apply(want_active ? OpMode_Active : OpMode_ListenOnly)) {
            Serial.println("[BTN] 모드 전환 실패 — 바뀌지 않았다");
            return;
        }
        bool active = want_active;
        FSDState saved = state_snapshot();
#if !defined(FSD_NO_WIFI)
        http_can_stream_set_enabled(true);  // capture works in both modes now (#108)
#endif
        Serial.println(active ? "[BTN] → Active mode" : "[BTN] → Listen-Only mode");
        can_dump_log(active ? "MODE switched to Active — TX enabled" : "MODE switched to Listen-Only — TX disabled");
        prefs_save(&saved);
#if defined(BLE_SERVER_ENABLED)
    } else if (n == 3) {
        // Let a DIFFERENT phone become the owner. Checked before the n >= 2
        // branch below, which would otherwise swallow it. Compiled out on
        // variants without BLE, so a triple click still toggles BMS there and
        // their behaviour is unchanged.
        //
        // The button is the whole point: pairing on this board is Just Works,
        // so the only thing the hardware can actually prove about a new phone
        // is that somebody was sitting in the car when it was enrolled.
        ble_owner_open_window(millis());
#endif
    } else if (n >= 2) {
        // Toggle BMS serial output
        FSDState saved;
        state_enter();
        g_state.bms_output = !g_state.bms_output;
        bool enabled = g_state.bms_output;
        saved = g_state;
        state_exit();
        Serial.printf("[BTN] BMS output: %s\n", enabled ? "ON" : "OFF");
        prefs_save(&saved);
    }
}

static void button_tick() {
    bool pressed = (digitalRead(PIN_BUTTON) == LOW);
    uint32_t now = millis();

    if (g_btn_ignore_boot) {
        if (!pressed) g_btn_ignore_boot = false;
        return;
    }

    if (pressed && !g_btn_down) {
        // Leading edge — debounce
        if ((now - g_last_release_ms) < BUTTON_DEBOUNCE_MS) return;
        g_btn_down             = true;
        g_btn_down_ms          = now;
        g_long_fired           = false;
        g_factory_reset_eligible = g_factory_reset_window;  // latch at press time
#if defined(BOARD_TTGO_DISPLAY)
        display_wake();
        g_display_last_wake_ms = now;
#endif
    }

    if (g_btn_down && pressed && !g_long_fired) {
        uint32_t held = now - g_btn_down_ms;
        if (g_factory_reset_eligible && held >= FACTORY_RESET_HOLD_MS) {
            g_long_fired           = true;
            g_pending_clicks       = 0;
            g_factory_reset_armed  = true;
            Serial.println("[BTN] Factory reset armed — release to confirm");
            led_factory_blink();
        } else if (!g_factory_reset_eligible && held >= LONG_PRESS_MS) {
            g_long_fired      = true;
            g_pending_clicks  = 0;
            FSDState saved;
            state_enter();
            g_state.nag_killer = !g_state.nag_killer;
            bool enabled = g_state.nag_killer;
            saved = g_state;
            state_exit();
            Serial.printf("[BTN] NAG Killer: %s\n", enabled ? "ON" : "OFF");
            prefs_save(&saved);
        }
        // eligible press with held < FACTORY_RESET_HOLD_MS: suppress 3s NAG killer
    }

    if (!pressed && g_btn_down) {
        // Trailing edge
        g_btn_down               = false;
        g_last_release_ms        = now;
        g_factory_reset_eligible = false;
        if (g_factory_reset_armed) {
            Serial.println("[BTN] Factory reset confirmed — clearing NVS");
            prefs_clear();
            delay(200);
            ESP.restart();
        }
        if (!g_long_fired) {
            g_pending_clicks++;
        }
    }

    // Flush pending clicks after the double-click window closes
    if (g_pending_clicks > 0 && !g_btn_down &&
        (now - g_last_release_ms) >= DOUBLE_CLICK_MS) {
        dispatch_clicks(g_pending_clicks);
        g_pending_clicks = 0;
    }

#if defined(BOARD_TTGO_DISPLAY)
    bool pressed2 = (digitalRead(PIN_BUTTON2) == LOW);
    if (g_btn2_ignore_boot) {
        if (!pressed2) g_btn2_ignore_boot = false;
    } else {
        if (pressed2 && !g_btn2_down) {
            if ((now - g_btn2_release_ms) >= BUTTON_DEBOUNCE_MS) {
                g_btn2_down = true;
                g_btn2_down_ms = now;
            }
        }
        if (!pressed2 && g_btn2_down) {
            g_btn2_down = false;
            g_btn2_release_ms = now;
            if (display_is_awake()) {
                display_sleep();
            } else {
                display_wake();
                g_display_last_wake_ms = now;
            }
        }
    }
#endif
}

// ── LED refresh ───────────────────────────────────────────────────────────────
static void update_led() {
    FSDState s = state_snapshot();
    if (g_factory_reset_armed) {
        led_set(LED_WHITE);
        return;
    }
    if (s.rx_count == 0 && millis() > WIRING_WARN_MS) {
        led_set(LED_RED);
    } else if (s.tesla_ota_in_progress) {
        led_set(LED_YELLOW);
    } else if (s.op_mode == OpMode_Active) {
        led_set(LED_GREEN);
    } else {
        led_set(LED_BLUE);
    }
}

// ── CAN frame dispatcher ──────────────────────────────────────────────────────
static void process_frame(CanBusId bus, const CanFrame &frame) {
    uint32_t now = millis();
    state_enter();
    g_state.rx_count++;
    g_state.last_rx_ms = now;  // bus-liveness stamp; rx_stale is derived in loop()
    // Configurable signal mapping (#122): when set, read DAS/steering from the
    // user-configured positions and disable the auto-parsers for those signals.
    fsd_apply_signal_config(&g_state, &frame, millis());
    bool das_cfg   = (g_state.cfg_das_id != 0);
    bool steer_cfg = (g_state.cfg_steer_id != 0);
    // AP-First stability debounce: stamp the last time AP was not engaged, so
    // fsd_ap_first_allows() can require AP held stable for AP_FIRST_STABLE_MS (#100/#108).
    // < DAS_APSTATE_ENGAGED (3): AVAILABLE(2) is not engaged, so the stability
    // window measures time actually held at 3+, and dropping to AVAILABLE re-requires
    // a centred wheel (a drop to 2 is a disengage).
    if (g_state.das_ap_state < DAS_APSTATE_ENGAGED) {
        g_state.ap_unstable_tick_ms = millis();
        g_state.soft_engage_latched = false;  // re-require centred wheel next engage (#108)
        g_state.ap_inject_count = 0;          // re-arm Minimal Inject burst next engage (#108)
    }
    fsd_abort_guard_update(&g_state);  // latch off injection if the car aborts (#108)
    // Black-box event-core poll (#124): once per frame, reading das_ap_state as
    // of the last DAS parse (same vantage as abort_guard above). Detects the
    // abort transition; the snapshot carries the toggles for the .json summary.
    blackbox_note_ap_state(g_state.das_ap_state, now);
    FSDEventType bb_evt = fsd_events_poll(&g_state, now);
    FSDState bb_snap = g_state;
    if (frame.id == CAN_ID_GTW_CAR_STATE)  g_state.seen_gtw_car_state++;
    if (frame.id == CAN_ID_GTW_CAR_CONFIG) g_state.seen_gtw_car_config++;
    if (frame.id == CAN_ID_AP_CONTROL)     g_state.seen_ap_control++;
    if (frame.id == CAN_ID_BMS_HV_BUS)     g_state.seen_bms_hv++;
    if (frame.id == CAN_ID_BMS_SOC)        g_state.seen_bms_soc++;
    if (frame.id == CAN_ID_BMS_THERMAL)    g_state.seen_bms_thermal++;
    state_exit();

    // Black-box: record key diagnostic ids (all buses, both modes; the filter
    // in blackbox_record keeps the window intact on a busy bus) and arm a
    // capture on an abort transition (#124). Never triggers on a plain
    // disengage — only EVT_ABORT here; bus-off/manual arm from elsewhere.
    blackbox_record(bus, frame, now);
    if (bb_evt == EVT_ABORT) blackbox_arm(BB_TRIG_ABORT, &bb_snap, now);

    // Tap capability checker (#125): count capability-relevant ids per bus during
    // an active listen window. Pure RX — no-op when no check is running.
    capability_record(bus, frame, now);

    can_dump_record(bus, frame);
    // Record to the web stream in BOTH modes so a capture can run *through* an
    // Activate (needed to catch the steer-jerk transient, #108). Recording is a
    // cheap ring-buffer write and only does anything when a client is connected.
    // Injection safety is preserved by never installing the single-ID hardware
    // filter in Active mode (see the acceptance-filter block in loop()).
#if !defined(FSD_NO_WIFI)
    http_can_stream_record(bus, frame);
#endif
#if defined(BOARD_LILYGO)
    g_last_can_rx_ms = millis();
    g_sleep_warned   = false;
#endif

    // DLC sanity: skip zero-length frames
    if (frame.dlc == 0) return;

    // ── HW auto-detect (passive, runs in both modes) ─────────────────────────
    if (frame.id == CAN_ID_GTW_CAR_CONFIG) {
        TeslaHWVersion hw = fsd_detect_hw_version(&frame);
        FSDState s = state_snapshot();
        if (hw != TeslaHW_Unknown && s.hw_version == TeslaHW_Unknown)
            apply_detected_hw(hw, "0x398");
        return;
    }

    // ── OTA monitoring (always, mode-independent) ─────────────────────────────
    if (frame.id == CAN_ID_GTW_CAR_STATE) {
        state_enter();
        bool was_ota = g_state.tesla_ota_in_progress;
        fsd_handle_gtw_car_state(&g_state, &frame);
        bool is_ota = g_state.tesla_ota_in_progress;
        bool ignore_ota = g_state.ignore_ota;
        uint8_t raw = g_state.ota_raw_state;
        state_exit();
        if (!was_ota && is_ota) {
            if (ignore_ota) {
                Serial.printf("[OTA] Update in progress (raw=%u) - TX allowed by Ignore OTA\n", raw);
                can_dump_log("OTA  started - TX allowed by Ignore OTA");
            } else {
                Serial.printf("[OTA] Update in progress (raw=%u) - TX suspended\n", raw);
                can_dump_log("OTA  started - TX suspended");
            }
        } else if (was_ota && !is_ota) {
            Serial.printf("[OTA] Update finished (raw=%u) - TX resumed\n", raw);
            can_dump_log("OTA  finished - TX resumed");
        }
        return;
    }

    // ── BMS sniff (read-only, always) ─────────────────────────────────────────
    if (frame.id == CAN_ID_BMS_HV_BUS)  { state_enter(); fsd_handle_bms_hv(&g_state, &frame);      state_exit(); return; }
    if (frame.id == CAN_ID_BMS_SOC)     { state_enter(); fsd_handle_bms_soc(&g_state, &frame);     state_exit(); return; }
    if (frame.id == CAN_ID_BMS_THERMAL) { state_enter(); fsd_handle_bms_thermal(&g_state, &frame); state_exit(); return; }

    // ── DAS status (read-only, always) — gating for NAG killer ───────────────
    // Skipped when a custom DAS source is configured (#122) — config owns it.
    FSDState das_state = state_snapshot();
    if (!das_cfg && hw_uses_hw3_das_status(das_state.hw_version) && frame.id == CAN_ID_DAS_STATUS_HW3) {
        state_enter();
        fsd_handle_das_status_hw3(&g_state, &frame);
        state_exit();
        return;
    }
    if (!das_cfg && hw_uses_hw4_das_status(das_state.hw_version) && frame.id == CAN_ID_DAS_STATUS_HW4) {
        state_enter();
        fsd_handle_das_status_hw4(&g_state, &frame);
        uint8_t ap = g_state.das_ap_state;         // as read by the std parser
        uint8_t ho = g_state.das_hands_on_state;
        state_exit();
        // Variant-profile auto-suggest (#126): feed the raw frame + what the std
        // parser made of it, so a stuck parser can be detected and matched.
        profile_match_record(frame, ap, ho, now);
        return;
    }
    // HW4 trims that never broadcast 0x39B carry the hands-on field on 0x399
    // (same byte5[5:2]); read it as a fallback so the nag gate isn't starved (#100).
    // Read-only and non-returning — the ISA chime-suppress path still handles 0x399.
    if (!das_cfg && hw_uses_hw4_das_status(das_state.hw_version) &&
        frame.id == CAN_ID_DAS_STATUS_HW3 && !das_state.das_hw4_status_seen) {
        state_enter();
        fsd_handle_das_handsonly_399(&g_state, &frame);
        state_exit();
    }

    // ── Continuous AP HW3/Legacy state parsers (read-only, always) ──────────
    if (frame.id == CAN_ID_SCCM_RSTALK) {
        FSDState s = state_snapshot();
        if (s.hw_version != TeslaHW_HW3 && s.hw_version != TeslaHW_Legacy) return;
        uint32_t now_ms = millis();
        if (frame.dlc > SIG_GEAR_LEVER_POS_BYTE) {
            uint8_t gear_pos =
                (frame.data[SIG_GEAR_LEVER_POS_BYTE] >> SIG_GEAR_LEVER_POS_SHIFT) &
                SIG_GEAR_LEVER_POS_MASK;
            if (gear_pos == SIG_GEAR_LEVER_FULL_UP) {
                g_cont_ap_last_stalk_full_up_ms = now_ms;
            }
        }
        state_enter();
        fsd_handle_gear_lever(&g_state, &frame, now_ms);
        if (g_state.stalk_full_up_ms != 0u &&
            (g_cont_ap_last_stalk_full_up_ms == 0u ||
             (int32_t)(g_state.stalk_full_up_ms - g_cont_ap_last_stalk_full_up_ms) > 0)) {
            g_cont_ap_last_stalk_full_up_ms = g_state.stalk_full_up_ms;
        }
        state_exit();
        if (frame.dlc > SIG_GEAR_LEVER_COUNTER_BYTE) {
            g_last_gear_counter =
                frame.data[SIG_GEAR_LEVER_COUNTER_BYTE] & SIG_GEAR_LEVER_COUNTER_MASK;
            g_last_gear_counter_valid = true;
            g_last_gear_counter_ms = now_ms;
        }
        if (frame.dlc > SIG_GEAR_LEVER_COUNTER_BYTE) {
            if (gear_sequence_active() && fsd_can_transmit(&s)) gear_sequence_tick(now_ms, "CONT-AP");
        }
        return;
    }
    if (frame.id == CAN_ID_UI_MAP_DATA) {
        state_enter();
        fsd_handle_ui_map_data(&g_state, &frame, millis());
        state_exit();
        return;
    }
    if (frame.id == CAN_ID_DAS_STATUS2) {
        state_enter();
        fsd_handle_das_status2(&g_state, &frame, millis());
        state_exit();
        return;
    }
    if (frame.id == CAN_ID_DAS_CONTROL) {
        state_enter();
        fsd_handle_das_control(&g_state, &frame);
        state_exit();
        return;
    }
    if (frame.id == CAN_ID_VCFRONT_LIGHT) {
        state_enter();
        fsd_handle_vcfront_lighting(&g_state, &frame);
        state_exit();
        return;
    }
    // Supervision inputs (fsd_autonomy.h): is a person driving right now?
    // Neither frame was parsed on this build before — the ESP32 does not compile
    // fsd_logic/fsd_handler.c, so its DI_state / UI_warning parsers never ran
    // here and di_cruise_state, ui_buckle_status and the blinker flags have all
    // been structurally zero. These two observers are shared with the Flipper.
    // Gear is on 0x118 DI_systemStatus. It was dispatched from 0x286 until
    // 2026-08-12, which does not carry DI_gear at all — the gate was reading
    // the top bits of DI_digitalSpeed and refusing at every ordinary speed.
    if (frame.id == CAN_ID_DI_SYS_STATUS) {
        uint32_t now_ms = millis();
        state_enter();
        fsd_drive_observe_gear(&g_state, &frame, now_ms);
        state_exit();
        return;
    }
    if (frame.id == CAN_ID_DI_STATE) {
        state_enter();
        fsd_drive_observe_cruise(&g_state, &frame);
        state_exit();
        return;
    }
    if (frame.id == CAN_ID_UI_WARNING) {
        uint32_t now_ms = millis();
        state_enter();
        fsd_drive_observe_belt(&g_state, &frame, now_ms);
        state_exit();
        return;
    }
    // Car GPS (0x3D8 position, 0x2F8 heading/speed) and the drivetrain speed
    // (0x257) that proves the position is not frozen. Read-only, and ABOVE the
    // TX boundary on purpose: the freeze detector needs three consecutive
    // position frames, and fsd_can_transmit() is false in OpMode_Autonomous —
    // the exact mode this feature exists for. On builds without the camera core
    // the shim returns false and this whole line folds away.
    // 0x257 speed -> FSDState. NON-RETURNING on purpose: camera_task_observe()
    // below needs this same frame for the freeze detector, and every other
    // observer in this function returns as soon as it handles one. Intercepting
    // 0x257 here would have blinded the camera path without a symptom.
    if (frame.id == CAN_ID_DI_SPEED) {
        uint32_t now_ms = millis();
        state_enter();
        fsd_drive_observe_speed(&g_state, &frame, now_ms);
        state_exit();
    }

    if (camera_task_observe(frame.id, frame.data, frame.dlc, millis())) return;

    // Body detectors (0x102, 0x103, 0x3C2). NON-RETURNING on purpose: 0x3C2 is
    // shared with the scroll path and the door frames may gain other readers,
    // so this offers the frame and gets out of the way. Read-only — the whole
    // feature has no emitter, and send_on_bus() refuses these IDs outright.
    (void)body_task_observe(frame.id, frame.data, frame.dlc, millis());

    if (frame.id == CAN_ID_ESP_STATUS) {
        uint32_t now_ms = millis();
        state_enter();
        fsd_handle_esp_status(&g_state, &frame);
        if (g_state.driver_brake_applied) g_cont_ap_last_brake_ms = now_ms;
        state_exit();
        return;
    }
    // Steering angle (0x129) — read-only, feeds the Soft Engage gate (#108).
    if (!steer_cfg && frame.id == CAN_ID_STEER_ANGLE) {
        state_enter();
        fsd_handle_steering_angle(&g_state, &frame);
        state_exit();
        return;
    }

    // Speed-profile read-back off 0x3FD. Non-returning — the AP-control handler
    // below still needs this frame.
    //
    // Reads the ORIGINAL `frame`, never the copy the injection path works on:
    // that copy is where our own writes go, and feeding it back would look like
    // the car agreeing with us. FSDState.speed_profile cannot serve here either
    // — it is write INTENT, derived from the follow-distance and stalk parsers,
    // and it even takes the value 4. Feeding intent to the policy's never-raise
    // clamp would be a lie in the one direction that matters.
    if (frame.id == CAN_ID_AP_CONTROL) {
        camera_task_observe_profile(hw_uses_hw4_das_status(das_state.hw_version),
                                    frame.data, frame.dlc, millis());
    }

    // ── Beyond here only run when TX is allowed ───────────────────────────────
    state_enter();
    bool tx = fsd_can_transmit(&g_state);
    // AP-First (#100/#108): when enabled, hold AP/FSD/nag injection until AP is
    // engaged and stable. Gates 0x3FD / 0x3EE / 0x370 below; off by default.
    bool ap_ok = fsd_ap_first_allows(&g_state, millis()) &&
                 fsd_soft_engage_allows(&g_state) &&   // Soft Engage holds until wheel centred (#108)
                 fsd_abort_guard_allows(&g_state);     // Abort Guard cuts injection on an abort (#108)
    state_exit();

    // NAG killer — build echo only when TX is currently allowed.
    if (frame.id == CAN_ID_EPAS_STATUS) {
        CanFrame echo;
        state_enter();
        fsd_handle_epas_status(&g_state, &frame);
        bool fired = (tx && ap_ok) ? fsd_handle_nag_killer(&g_state, &frame, &echo, millis()) : false;
        state_exit();
        if (fired) {
            uint8_t lvl     = (frame.data[4] >> 6) & 0x03;
            uint8_t cnt_in  = frame.data[6] & 0x0F;
            uint8_t cnt_out = echo.data[6] & 0x0F;
            // fired is already gated on tx above, so the send is unconditional
            // here; route through the bus-aware helper from the dual-CAN work.
            can_dump_log("NAG 0x370 hands_off lvl=%u cnt=%u->%u TX echo",
                         lvl, cnt_in, cnt_out);
            send_on_bus(bus, echo);
        }
        return;
    }

    // Legacy stalk (0x045) — updates speed_profile, no TX
    if (frame.id == CAN_ID_STW_ACTN_RQ && state_snapshot().hw_version == TeslaHW_Legacy) {
        state_enter();
        fsd_handle_legacy_stalk(&g_state, &frame);
        state_exit();
        return;
    }

    // Legacy autopilot control (0x3EE)
    if (frame.id == CAN_ID_AP_LEGACY && state_snapshot().hw_version == TeslaHW_Legacy) {
        CanFrame f = frame;
        state_enter();
        // Minimal Inject (#108): once this engagement's burst budget is spent, stop
        // modifying the 0x3EE frame so injection stays at engage onset, off the abort
        // edge. ap_inject_count counts injected frames and resets to 0 on disengage.
        bool minimal_ok = !(g_state.ap_first_minimal &&
                            g_state.ap_inject_count >= AP_MINIMAL_INJECT_FRAMES);
        bool modified = minimal_ok && fsd_handle_legacy_autopilot(&g_state, &f);
        if (modified && ap_ok && g_state.ap_first_minimal) g_state.ap_inject_count++;
        state_exit();
        if (modified && tx && ap_ok) send_on_bus(bus, f);
        return;
    }

    // Auto-upgrade Legacy→HW3: Palladium S/X with HW3 reports das_hw=0
    // (→Legacy) but actually uses 0x3FD. True Legacy never broadcasts 0x3FD.
    if (state_snapshot().hw_version == TeslaHW_Legacy && frame.id == CAN_ID_AP_CONTROL) {
        apply_detected_hw(TeslaHW_HW3, "upgrade:Legacy→HW3(0x3FD seen)");
    }

    // Fallback HW detection when 0x398 is unavailable on the tapped bus.
    // Prefer explicit HW4 DAS_status (0x39B) when present. If the tap only sees
    // HW3-style DAS_status on 0x399, classify as HW3 after repeated plausible
    // samples so 0x399 can be parsed for AP/NAG gating.
    static uint32_t hw_fallback_3fd_count = 0;
    static uint32_t hw_fallback_399_count = 0;
    // NOTE (#110/#122): beta.20 upgraded a locked-in HW3 guess to HW4 whenever a
    // valid 0x39B appeared, to fix HW4 cars whose tap carries no 0x398. That
    // regressed cars where the HW3 path was already working: switching to the HW4
    // DAS parser left AP-state unread ("Waiting") and HW4-style injection produced
    // TX errors. Reverted — detecting HW4 must not change a DAS-read/injection
    // path that already works. See #110 for the reworked approach.
    if (state_snapshot().hw_version == TeslaHW_Unknown) {
        if (frame.id == CAN_ID_AP_LEGACY) {
            apply_detected_hw(TeslaHW_Legacy, "fallback:0x3EE");
        } else if (frame.id == CAN_ID_DAS_STATUS_HW4 && frame.dlc == CAN_FRAME_MAX_DATA_LEN) {
            apply_detected_hw(TeslaHW_HW4, "fallback:0x39B");
            hw_fallback_3fd_count = 0;
            hw_fallback_399_count = 0;
        } else if (frame_looks_like_hw3_das_status(frame)) {
            hw_fallback_399_count++;
            if (hw_fallback_399_count >= 2u)
                apply_detected_hw(TeslaHW_HW3, "fallback:0x399(DAS status)");
        } else if (frame.id == CAN_ID_AP_CONTROL) {
            hw_fallback_3fd_count++;
            if (hw_fallback_3fd_count >= 10u)
                apply_detected_hw(TeslaHW_HW3, "fallback:0x3FD(confirmed)");
        }
    }

    // ISA speed chime (0x399, HW4 only)
    FSDState s = state_snapshot();
    if (frame.id == CAN_ID_ISA_SPEED &&
        s.hw_version == TeslaHW_HW4 &&
        s.suppress_speed_chime) {
        CanFrame f = frame;
        if (fsd_handle_isa_speed_chime(&f) && tx)
            send_on_bus(bus, f);
        return;
    }

    // Follow distance → speed_profile (0x3F8). Parse is read-only; the RHD
    // driving-side override (#66) may modify a copy and re-TX when enabled.
    if (frame.id == CAN_ID_FOLLOW_DIST) {
        CanFrame f = frame;
        state_enter();
        fsd_handle_follow_distance(&g_state, &frame);
        bool modified = fsd_handle_driver_assist_override(&g_state, &f);
        state_exit();
        if (modified && tx) send_on_bus(bus, f);
        return;
    }

    // TLSSC Restore (0x331) — DAS config spoof
    if (frame.id == CAN_ID_DAS_AP_CONFIG) {
        CanFrame f = frame;
        state_enter();
        bool modified = fsd_handle_tlssc_restore(&g_state, &f);
        state_exit();
        if (modified && tx) send_on_bus(bus, f);
        return;
    }

    // HW3/HW4 autopilot control (0x3FD) — main FSD activation frame
    if (frame.id == CAN_ID_AP_CONTROL) {
        CanFrame f = frame;
        state_enter();
        // Minimal Inject (#108): stop modifying 0x3FD once the burst budget is spent
        // this engagement, keeping injection off the abort edge. Resets on disengage.
        bool minimal_ok = !(g_state.ap_first_minimal &&
                            g_state.ap_inject_count >= AP_MINIMAL_INJECT_FRAMES);
        bool modified = minimal_ok && fsd_handle_autopilot_frame(&g_state, &f);
        if (modified && ap_ok && g_state.ap_first_minimal) g_state.ap_inject_count++;
        state_exit();
        if (modified && tx && ap_ok) send_on_bus(bus, f);
        return;
    }
}

#if defined(BOARD_LILYGO)
// ── Deep-sleep watchdog (Lilygo only) ────────────────────────────────────────
static void sleep_tick(uint32_t now) {
    if (now < g_last_can_rx_ms) return;
    uint32_t idle_ms = now - g_last_can_rx_ms;
    FSDState s = state_snapshot();

    if (idle_ms >= s.sleep_idle_ms) {
        Serial.printf("[SLEEP] Entering deep sleep after %lu ms CAN silence\n",
                      (unsigned long)idle_ms);
        can_dump_stop();
        sd_syslog_close();
        led_set(LED_SLEEP);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_CAN_RX, 0);
        esp_deep_sleep_start();
        // never returns
    } else if (!g_sleep_warned && idle_ms >= (s.sleep_idle_ms - SLEEP_WARN_MS)) {
        g_sleep_warned = true;
        uint32_t remaining_ms = s.sleep_idle_ms - idle_ms;
        Serial.printf("[SLEEP] Warning: %lu ms idle, sleeping in %lu ms\n",
                      (unsigned long)idle_ms, (unsigned long)remaining_ms);
    }
}
#endif

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
#if defined(BOARD_LILYGO)
    g_last_can_rx_ms = millis();
#endif
    Serial.begin(115200);
    delay(300);

    Serial.println("\n============================");
    Serial.println(" Tesla FSD Unlock — ESP32   ");
    Serial.println("============================");
    Serial.printf("[FSD] Build: %s %s\n", __DATE__, __TIME__);

    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running) {
        Serial.printf("[OTA] Running from: %s\n", running->label);

        esp_ota_img_states_t ota_state;
        if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                // Do NOT mark valid here. This line runs before CAN, storage and
                // BLE are up, so calling esp_ota_mark_app_valid_cancel_rollback()
                // now would throw away the rollback for an image we have not
                // watched run for even one loop. An image that boots and then
                // dies in CAN init would already have cancelled its own way back.
                // ESP-IDF's OTA docs say to self-test first; ota_selftest_tick()
                // below does that. CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y in the
                // Arduino SDK, so this is live, not theoretical.
                g_ota_pending_verify = true;
                Serial.println("[OTA] First boot after update - self-test running");
            } else if (ota_state == ESP_OTA_IMG_VALID) {
                Serial.println("[OTA] Running verified firmware");
            }
        }
    }

#if defined(CAN_DRIVER_T2CAN_DUAL)
    Serial.println("[CAN] Driver: LilyGO T-2CAN dual CAN (TWAI can0 + MCP2515 can1)");
#elif defined(CAN_DRIVER_TWAI)
  #if defined(BOARD_WAVESHARE_S3)
    Serial.println("[CAN] Driver: ESP32-S3 TWAI (Waveshare ESP32-S3-RS485-CAN)");
  #elif defined(BOARD_LILYGO)
    Serial.println("[CAN] Driver: ESP32 TWAI (LilyGO T-CAN485)");
  #else
    Serial.println("[CAN] Driver: ESP32 TWAI (M5Stack ATOM Lite + ATOMIC CAN Base)");
  #endif
#elif defined(CAN_DRIVER_MCP2515)
    Serial.println("[CAN] Driver: MCP2515 via SPI");
#endif

#if defined(BOARD_LILYGO)
    pinMode(ME2107_EN, OUTPUT);
    digitalWrite(ME2107_EN, HIGH);
    delay(100); // Wait for 5V rail to stabilize (SD power)
    // CAN transceiver slope/mode pin — must be LOW for normal TX+RX operation.
    // Floating or HIGH puts the SN65HVD230/TJA1051 into standby (RX-only),
    // which causes the TWAI controller to go bus-off the first time it tries to TX.
    pinMode(PIN_CAN_SPEED_MODE, OUTPUT);
    digitalWrite(PIN_CAN_SPEED_MODE, LOW);
#endif

#if defined(CAN_DRIVER_T2CAN_DUAL)
    Serial.printf("[CFG] pins: LED=%d BUTTON=%d can0_TX=%d can0_RX=%d can1_CS=%d can1_SCK=%d can1_MISO=%d can1_MOSI=%d\n",
                  PIN_LED, PIN_BUTTON, PIN_CAN_TX, PIN_CAN_RX,
                  PIN_MCP_CS, PIN_MCP_SCK, PIN_MCP_MISO, PIN_MCP_MOSI);
    Serial.printf("[CFG] TX route: modified frames -> source bus, generated precondition -> %s\n",
                  can_bus_name(configured_bus_from_index(PRECONDITION_TX_BUS_INDEX)));
    Serial.printf("[CFG] Continuous AP gear TX route: 0x229 -> %s\n",
                  can_bus_name(preferred_generated_bus(CAN_ID_SCCM_RSTALK)));
#elif defined(CAN_DRIVER_TWAI)
    Serial.printf("[CFG] pins: LED=%d BUTTON=%d CAN_TX=%d CAN_RX=%d\n",
                  PIN_LED, PIN_BUTTON, PIN_CAN_TX, PIN_CAN_RX);
#else
    Serial.printf("[CFG] pins: LED=%d BUTTON=%d MCP_CS=%d MCP_SCK=%d\n",
                  PIN_LED, PIN_BUTTON, PIN_MCP_CS, PIN_MCP_SCK);
#endif

    pinMode(PIN_BUTTON, INPUT_PULLUP);
#if defined(BOARD_TTGO_DISPLAY)
    pinMode(PIN_BUTTON2, INPUT_PULLUP);
#endif
    led_init();
#if defined(BOARD_TTGO_DISPLAY)
    display_init();
#endif

    fsd_state_init(&g_state, TeslaHW_Unknown);
    // Explicit safe defaults — will be overridden after HW auto-detect.
    // Listen-Only until prefs are loaded; the floor is applied afterwards,
    // once autonomy_enabled is actually known.
    g_state.op_mode               = OpMode_ListenOnly;
    g_state.nag_killer            = true;
    g_state.suppress_speed_chime  = true;
    g_state.ignore_ota            = false;
    g_state.fsd_unlock            = false;
    g_state.emergency_vehicle_detect = false;
    g_state.force_fsd             = false;
    g_state.china_mode            = false;
    g_state.bms_output            = false;
    g_state.blackbox_enabled      = BLACKBOX_DEFAULT_ENABLED;  // OFF everywhere; opt in with 'bbon' or BLE BB_ENABLE

    prefs_load(&g_state);
    // 지난 세션이 어떻게 끝났는지 읽어 판정을 찍는다. prefs_load() 뒤인 이유는
    // 없다 — 서로 다른 네임스페이스다. 부팅 배너 근처에 나오게 하려는 것뿐이다.
    power_log_init();
    ble_owner_init();
    // Now that autonomy_enabled is known, settle where the module sits with
    // nothing else raising it. op_mode itself is still never read from NVS
    // (PERSIST_OP_MODE); this is derived from the operator's intent, and
    // Autonomous transmits nothing on its own — the camera path additionally
    // needs a gear of D and a latched belt.
    g_state.op_mode = fsd_autonomy_floor(&g_state);
    if (g_state.op_mode == OpMode_Autonomous)
        Serial.println("[MODE] Autonomous — camera response only, no general TX");
    // Apply a saved manual HW selection immediately (#110) so the correct
    // handlers are live from the first frame, without waiting on detection.
    if (g_state.hw_override != TeslaHW_Unknown) {
        fsd_apply_hw_version(&g_state, g_state.hw_override);
        Serial.printf("[HW] Manual override: %s\n",
                      (g_state.hw_override == TeslaHW_HW4)    ? "HW4" :
                      (g_state.hw_override == TeslaHW_HW3)    ? "HW3" : "Legacy");
    }
#if defined(BOARD_TTGO_DISPLAY)
    display_set_enabled(g_state.display_enabled);
#endif

    {
        esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
        g_factory_reset_window = (wakeup == ESP_SLEEP_WAKEUP_UNDEFINED);
        if (g_factory_reset_window)
            Serial.println("[BTN] Factory reset window active — hold button 5s within 20s");
    }

    if (state_snapshot().op_mode == OpMode_Active) {
        // Will be re-applied after g_can is created; record intent here only
        Serial.println("[NVS] Restored Active mode from NVS");
    }

    led_set(LED_BLUE);

    can_dump_init();
    blackbox_init(&g_state, &g_state_mux);  // ring + storage (after SD is mounted)
#ifdef BLE_SERVER_ENABLED
    // Camera database lives in the same filesystem blackbox_init() mounts.
    // Absent on a fresh flash — the app uploads one over BLE.
    camera_store_init();
#endif
    // Instances + learning restore. After camera_store_init() (it needs the
    // filesystem), before ble_server_init() (its packer reads the accessors).
    camera_task_init(&g_state, &g_state_mux);
    body_task_init(&g_state, &g_state_mux);   // T1/T2 detectors — read-only
    capability_init(&g_state, &g_state_mux);  // tap capability checker (#125)
    ble_server_init(&g_state, &g_state_mux);  // GATT server for the phone app
    // Client role, for a generic button. AFTER the server: NimBLEDevice::init()
    // happens there and must not be called twice.
    ble_central_init();
    profile_match_init(&g_state, &g_state_mux);  // variant-profile auto-suggest (#126)

#if defined(BOARD_LILYGO)
    {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_EXT0) {
            Serial.printf("[WAKE] Woken by CAN activity (EXT0 GPIO %d)\n", PIN_CAN_RX);
        } else if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
            Serial.printf("[WAKE] Wakeup cause=%d\n", (int)cause);
        }
        g_last_can_rx_ms = millis();
    }
#endif

    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        CanBusId bus = bus_id_from_index(i);
        g_can[i] = can_driver_create(bus);
        g_can_ok[i] = g_can[i] && g_can[i]->begin(true);
        g_can_last_retry_ms[i] = millis();
    }
    if (!can_any_ok()) {
        Serial.println("[ERR] All CAN driver init failed — check wiring");
#if defined(BOARD_TTGO_DISPLAY)
        Serial.printf("[ERR] Continuing in NO-CAN mode (will retry every %lu ms)\n",
                      (unsigned long)CAN_REINIT_INTERVAL_MS);
        led_set(LED_RED);
#elif defined(CAN_DRIVER_T2CAN_DUAL)
        Serial.printf("[ERR] Continuing in NO-CAN mode (will retry every %lu ms)\n",
                      (unsigned long)CAN_REINIT_INTERVAL_MS);
        led_set(LED_RED);
#else
        // Halt: signal error via blinking red indefinitely.
        while (true) {
            led_set(LED_RED);   delay(200);
            led_set(LED_OFF);   delay(200);
        }
#endif
    } else {
        // Through mode_apply() like every other caller, so a controller that
        // refuses to leave Listen-Only cannot leave the module claiming Active
        // at boot. PERSIST_OP_MODE=0 on the car board means this is always
        // Listen-Only there; the other variants can restore Active.
        OpMode restored = state_snapshot().op_mode;
        if (!mode_apply(restored)) {
            Serial.println("[CAN] 500 kbps — mode switch failed, staying Listen-Only");
        } else if (restored == OpMode_Active) {
            Serial.println("[CAN] 500 kbps — Active (restored from NVS)");
        } else {
            Serial.println("[CAN] 500 kbps — Listen-Only");
        }
    }
    Serial.println("[BTN] Single click : toggle Listen-Only / Active");
    Serial.println("[BTN] Long press 3s: toggle NAG Killer");
    Serial.println("[BTN] Double click : toggle BMS serial output");
    Serial.println("[LED] Blue=Listen  Green=Active  Yellow=OTA  Red=Error");

    // ── WiFi + Web dashboard (non-fatal if WiFi fails) ───────────────────────
#if defined(FSD_NO_WIFI)
    // Not built. This board lives permanently in a car and talks to its phone
    // app over BLE, so the AP was pure attack surface: the default password is
    // printed in this repo's README, and ws_event() in web_dashboard.cpp has no
    // authentication of any kind -- one message on that socket flips the module
    // to Active and opens CAN transmit on every bus. blackbox.h already said we
    // "do not intend to bring WiFi up at all" here; this makes that true.
    Serial.println("[WiFi] not built into this firmware — BLE only");
#else
    if (wifi_init(&g_state)) {
        web_dashboard_init(&g_state, g_can, CAN_ACTIVE_BUS_COUNT, &g_state_mux);
        http_can_stream_set_enabled(true);  // capture works in both modes now (#108)
        Serial.println("[SER] Type 'ip' in the serial monitor to print WiFi URLs again");
    }
#endif

    // Ring is allocated lazily on enable, so a persisted-ON device re-applies it
    // here. The heap guard refuses when memory is too tight, so this can never
    // re-trigger the boot loop (#124).
    //
    // 🔴 This used to sit INSIDE the WiFi block above. On a build without WiFi
    // that is one of only two callers gone, and the other one is the dashboard
    // -- so the module would have booted with its capture ring permanently
    // unallocated and no way left to allocate it. BLE can read captures
    // (DUMP_START) but has no command to switch recording on. The one-shot
    // capture we take before the TSL comes out runs on this ring.
    if (g_state.blackbox_enabled) blackbox_set_enabled(true);
}

// ── OTA self-test ─────────────────────────────────────────────────────────────
// Accept a freshly installed image only after watching it run, per ESP-IDF's OTA
// rollback guidance. Until this calls mark_app_valid, a reset boots the previous
// firmware -- which is the whole point of the pending-verify state.
//
// WHAT IS AND IS NOT A HEALTH SIGNAL
// ----------------------------------
// The test has to prove OUR CODE works, never that the CAR is there. Gate on the
// wrong thing and a good image is rolled back for sitting on a bench:
//
//   ✗ frames received      -- needs a live bus
//   ✗ camera_store_ready() -- false until a database has been uploaded, so a
//                             fresh board would NEVER accept its own firmware
//   ✗ ble_server_connected() -- needs a phone in range
//   ✓ we reached loop()    -- catches a crash or hang anywhere in setup()
//   ✓ the loop keeps running -- catches boot loops and watchdog resets
//   ✓ EVERY CAN controller came up -- on this board the transceivers are onboard,
//                                 so begin() talks to the chip, not to the car.
//                                 "every", not "any": this variant runs two, and
//                                 an image that kept can0 alive while breaking
//                                 can1 would otherwise be accepted with half its
//                                 hardware dead
//   ✓ storage mounted      -- the blackbox ring and camera.bin both live there
//   ✓ BLE started advertising -- the only control channel this board has
//
// Returns why the image is not acceptable yet, or nullptr when it is.
static const char* ota_selftest_failure(void) {
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (!g_can_ok[i]) return "a CAN controller did not come up";
    }
    if (!blackbox_storage_ok()) return "storage did not mount";
    if (!ble_server_up())       return "BLE did not start advertising";
    return nullptr;
}

static void ota_selftest_tick(uint32_t now) {
    if (!g_ota_pending_verify) return;

    if (g_ota_first_loop_ms == 0) {
        g_ota_first_loop_ms = now;
        // 🔴 The deadline below lives inside loop(). If loop() stops entirely --
        // hangs in a call that never returns -- nothing here runs, no clock
        // advances, and the promised rollback never happens: the image sits in
        // PENDING_VERIFY until something else reboots the board. A timer cannot
        // fix that either; only a watchdog can.
        //
        // The Arduino core sets loopTaskWDTEnabled = false at startup, and the
        // Task WDT watches CPU0's idle task while this loop runs on CPU1, so a
        // hung loop was unwatched. enableLoopWDT() subscribes it; the core feeds
        // it once per iteration, so a loop that stops panics and reboots in 5 s,
        // and the bootloader rolls back because we never marked the image valid.
        //
        // ⚠️ ONLY while a self-test is pending, and switched off the moment it
        // resolves. do_flush() writes a 15-second capture window to LittleFS in
        // one blocking call -- on a busy bus that is hundreds of kilobytes and
        // will exceed the 5 s timeout. An always-on loop WDT would reboot the
        // module while it was saving the one-shot capture taken before the TSL
        // comes out, which is a far worse outcome than the hang it guards.
        enableLoopWDT();
        Serial.println("[OTA] self-test armed — loop watchdog on for the duration");
    }
    if (g_ota_loops < UINT32_MAX) g_ota_loops++;

    uint32_t elapsed = (uint32_t)(now - g_ota_first_loop_ms);
    const char* why = ota_selftest_failure();

    // The ordering of the thresholds is in fsd_selftest.c, where a host test
    // pins it down. It used to be three ifs here, and they were in the wrong
    // order: the loop-count gate came before the deadline, so an image too
    // broken to run its loop was never rolled back at all.
    switch (fsd_selftest_decide(elapsed, g_ota_loops, why == nullptr)) {
    case FSD_SELFTEST_WAIT: {
        if (why == nullptr) return;   // just warming up; nothing to report
        // A CAN controller can still come back on the 30 s re-init retry, so
        // this is not a verdict -- but say it out loud, once a minute, because
        // it is also what a real failure looks like on its way to the deadline.
        static uint32_t s_last_warn_ms = 0;
        if (s_last_warn_ms == 0 || (uint32_t)(now - s_last_warn_ms) >= 60000u) {
            s_last_warn_ms = now;
            Serial.printf("[OTA] self-test waiting: %s — image NOT accepted "
                          "(%u s until rollback)\n",
                          why, (unsigned)((FSD_SELFTEST_DEADLINE_MS - elapsed) / 1000u));
        }
        return;
    }

    case FSD_SELFTEST_ACCEPT:
        g_ota_pending_verify = false;
        disableLoopWDT();   // see the note where it was enabled
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            Serial.printf("[OTA] self-test passed (%u loops, %u ms) — firmware accepted\n",
                          (unsigned)g_ota_loops, (unsigned)elapsed);
        } else {
            Serial.println("[OTA] WARNING: self-test passed but could not mark firmware valid");
        }
        return;

    case FSD_SELFTEST_ROLLBACK:
        // Waiting forever is its own failure mode: the image runs indefinitely
        // in PENDING_VERIFY, and the rollback then happens whenever something
        // else reboots the board -- on a car that sleeps, possibly hours later,
        // with nobody watching and no reason recorded.
        Serial.printf("[OTA] self-test FAILED after %u s (%u loops): %s\n",
                      (unsigned)(elapsed / 1000u), (unsigned)g_ota_loops,
                      why ? why : "unknown");
        Serial.println("[OTA] rolling back to the previous firmware and rebooting");
        Serial.flush();
        g_ota_pending_verify = false;      // do not retry if the call returns
        disableLoopWDT();                  // the reboot below should be ours
        esp_ota_mark_app_invalid_rollback_and_reboot();
        // Only reached when there is no valid image to roll back to.
        Serial.println("[OTA] WARNING: rollback refused — staying on this image");
        return;
    }
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();
    ota_selftest_tick(now);

    if (g_factory_reset_window && now >= FACTORY_RESET_WINDOW_MS) {
        g_factory_reset_window = false;
        Serial.println("[BTN] Factory reset window closed");
    }

    button_tick();
    serial_command_tick();

    // Drain all available CAN frames in one shot
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (!g_can_ok[i] || !g_can[i]) continue;
        CanBusId bus = bus_id_from_index(i);
        CanFrame frame;
        while (g_can[i]->receive(frame)) {
            process_frame(bus, frame);
        }
        // Recover a bus-off controller so RX resumes without a manual toggle (#108).
        g_can[i]->serviceHealth();
        // Bus-off just fired → arm a black-box capture via the event-core (#124).
        if (g_can[i]->busOffEvent()) blackbox_busoff(now);
    }

    // Bus liveness, derived once per loop — process_frame() only runs when a
    // frame arrives, so a bus that went quiet would never be noticed there.
    {
        state_enter();
        bool was = g_state.rx_stale;
        g_state.rx_stale = fsd_rx_is_stale(&g_state, now);
        bool changed = (was != g_state.rx_stale);
        bool stale = g_state.rx_stale;
        bool heard = (g_state.rx_count > 0);  // suppress the boot-time edge
        state_exit();
        if (changed && heard) {
            Serial.printf("[CAN] bus %s\n", stale ? "quiet — TX held off" : "back — TX allowed");
        }
    }

    // ── Periodic error counter refresh (~every 250 ms) ────────────────────────
    static uint32_t last_err_ms = 0;
    if ((now - last_err_ms) >= 250u) {
        state_enter();
        g_state.crc_err_count = can_total_error_count();
        g_state.tx_count      = can_total_tx_count();
        state_exit();
        last_err_ms = now;
    }

    continuous_ap_tick(now);

    // ── Precondition frame injection ──────────────────────────────────────────
    static uint32_t last_precond_ms = 0;
    FSDState s = state_snapshot();
    if (s.precondition && fsd_can_transmit(&s) &&
        (now - last_precond_ms) >= PRECOND_INTERVAL_MS) {
        CanFrame pf;
        fsd_build_precondition_frame(&pf);
        send_generated_frame(configured_bus_from_index(PRECONDITION_TX_BUS_INDEX), pf);
        last_precond_ms = now;
    }

    // ── BMS serial output ─────────────────────────────────────────────────────
    static uint32_t last_bms_ms = 0;
    s = state_snapshot();
    if (s.bms_output && s.bms_seen &&
        (now - last_bms_ms) >= BMS_PRINT_MS) {
        float kw = s.pack_voltage_v * s.pack_current_a / 1000.0f;
        Serial.printf("[BMS] %.1fV  %.1fA  %.2fkW  SoC:%.1f%%  Temp:%d~%d°C\n",
            s.pack_voltage_v,
            s.pack_current_a,
            kw,
            s.soc_percent,
            (int)s.batt_temp_min_c,
            (int)s.batt_temp_max_c);
        last_bms_ms = now;
    }

    // ── Active-mode status line ───────────────────────────────────────────────
    static uint32_t last_status_ms = 0;
    s = state_snapshot();
    if (s.op_mode == OpMode_Active &&
        (now - last_status_ms) >= STATUS_PRINT_MS) {
        const char *hw_str =
            (s.hw_version == TeslaHW_HW4)    ? "HW4"    :
            (s.hw_version == TeslaHW_HW3)    ? "HW3"    :
            (s.hw_version == TeslaHW_Legacy)  ? "Legacy" : "?";
        Serial.printf(
            "[STA] HW:%-6s AP:%-4s FSD_UI:%-4s Unlock:%-3s NAG:%-3s Echo:%lu OTA:%-3s "
            "Profile:%d  RX:%lu TX:%lu Mod:%lu Err:%lu\n",
            hw_str,
            s.ap_active       ? "ON"         : "wait",
            s.fsd_enabled     ? "ON"         : "wait",
            s.fsd_unlock      ? "ON"         : "off",
            s.nag_killer      ? "ON"         : "off",
            (unsigned long)s.nag_echo_count,
            s.tesla_ota_in_progress ? "YES"  : "no",
            s.speed_profile,
            (unsigned long)s.rx_count,
            (unsigned long)s.tx_count,
            (unsigned long)s.frames_modified,
            (unsigned long)s.crc_err_count);
        last_status_ms = now;
    }

    // ── Headroom, deliberately NOT gated on Active ────────────────────────────
    // Neither number is sampled anywhere else in this firmware, and both became
    // load-bearing with the camera core: ~12.2 KB of new .bss on this variant
    // comes out of the same pool the black box asks ~175 KB of free heap for,
    // and fsd_trk_update() puts a 24-entry scan buffer on the loop task's stack.
    // A bring-up drive runs in Listen-Only, so gating this on Active — the way
    // the line above is — would print it exactly when nobody is measuring.
    static uint32_t last_mem_ms = 0;
    if ((now - last_mem_ms) >= 60000u) {
        // The multiply is a no-op on ESP-IDF (StackType_t is a byte there, and
        // the high-water mark already comes back in bytes rather than the words
        // vanilla FreeRTOS returns). Kept because it is what makes the line
        // read correctly on either.
        Serial.printf("[MEM] heap:%lu min:%lu stack_free:%lu\n",
                      (unsigned long)ESP.getFreeHeap(),
                      (unsigned long)ESP.getMinFreeHeap(),
                      (unsigned long)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
        last_mem_ms = now;
    }

    // ── Periodic re-init when a CAN driver failed at boot ────────────────────
    for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
        if (!g_can_ok[i] && g_can[i] &&
            (now - g_can_last_retry_ms[i]) >= CAN_REINIT_INTERVAL_MS) {
            g_can_last_retry_ms[i] = now;
            Serial.printf("[CAN] Retrying %s driver init...\n",
                          can_bus_name(bus_id_from_index(i)));
            bool listen_only = (state_snapshot().op_mode != OpMode_Active);
            g_can_ok[i] = g_can[i]->begin(listen_only);
            if (g_can_ok[i]) {
                Serial.printf("[CAN] %s re-init SUCCESS — %s mode\n",
                              can_bus_name(bus_id_from_index(i)),
                              listen_only ? "Listen-Only" : "Active");
            }
        }
    }

    // ── Wiring / hardware sanity warning ─────────────────────────────────────
    static uint32_t last_warn_ms = 0;
    s = state_snapshot();
    if (now > WIRING_WARN_MS && (now - last_warn_ms) >= 5000u) {
        if (!can_any_ok()) {
            // Driver init failed — distinguish chip-not-detected from other.
            // Skip the "no CAN traffic" warn entirely (it's never going to
            // arrive without a working driver).
            Serial.println("[WARN] CAN drivers not initialised — "
                           "no CAN traffic possible");
            last_warn_ms = now;
        } else if (s.rx_count == 0) {
            Serial.println("[WARN] No CAN traffic after 5 s — check wiring");
            Serial.println("[WARN] Verify CAN-H on OBD pin 6, CAN-L on pin 14");
            last_warn_ms = now;
        }
    }

    can_dump_tick(now);
    blackbox_tick(now);  // post-roll countdown + flush (#124)
    capability_tick(now);  // finalize the capability listen window (#125)
    camera_task_tick(now); // 1 Hz camera judgement — reads only, never transmits
    // 12V 가 스위치드인지 상시인지 기록한다. NVS 쓰기가 여기(loop)에서만
    // 일어나야 한다는 계약은 prefs.cpp 와 같다.
    power_log_tick(now, g_state.last_rx_ms, g_state.last_rx_ms != 0);
    ble_owner_tick(now);   // owns the NVS write; see ble_owner.h
    // The body detectors need to know whether the transceiver could transmit at
    // all; only main.cpp owns the drivers. Fail-closed: any doubt reports shut.
    {
        bool tx_open = false;
        for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
            if (g_can_ok[i] && g_can[i] && !g_can[i]->isListenOnly()) tx_open = true;
        }
        body_task_set_bus_tx_open(tx_open);
    }
    body_task_tick(now);   // T1/T2 detectors — measures, logs, sends nothing
    ble_server_tick(now);  // push State notifications to the phone app
    ble_central_tick(now); // button link + press classification — no TX

#if defined(BOARD_LILYGO)
    sleep_tick(now);
#endif

#if !defined(FSD_NO_WIFI)
    // ── Web dashboard (after CAN to preserve CAN frame latency) ──────────────
    web_dashboard_update();

    // ── Full-rate single-ID capture: drive the hardware acceptance filter ─────
    // When a /stream is opened with exactly one ?ids= value, restrict the CAN
    // controller to that id so its RX queue never overflows and every frame is
    // captured at true full rate. If ?bus=can0/can1 is present, scope the
    // hardware filter to that controller only. Restore accept-all when the
    // stream ends. Only ever single in Listen-Only (stream is disabled in Active).
    {
        static bool     s_hw_filter_single[CAN_ACTIVE_BUS_COUNT] = {};
        static uint32_t s_hw_filter_id[CAN_ACTIVE_BUS_COUNT] = {};
        static bool     s_hw_filter_initialized = false;
        if (!s_hw_filter_initialized) {
            for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
                s_hw_filter_id[i] = 0xFFFFFFFFu;
            }
            s_hw_filter_initialized = true;
        }

        uint32_t want_id = 0;
        bool want_single = http_can_stream_single_filter(&want_id);
        CanBusId want_bus = CAN_BUS_PRIMARY;
        bool want_bus_filter = http_can_stream_bus_filter(&want_bus);

        // Never install the single-ID hardware filter in Active mode — it would
        // restrict the controller to one id and starve injection's RX path. A
        // single-ID capture during Active silently falls back to software
        // filtering (lower rate, but injection stays intact). Multi-ID captures
        // are software-filtered anyway and run at full effect in both modes.
        bool allow_hw_filter = (state_snapshot().op_mode == OpMode_ListenOnly);
        for (uint8_t i = 0; i < CAN_ACTIVE_BUS_COUNT; i++) {
            CanBusId bus = bus_id_from_index(i);
            bool bus_want_single = want_single && allow_hw_filter && (!want_bus_filter || bus == want_bus);
            uint32_t bus_want_id = bus_want_single ? want_id : 0u;
            if (bus_want_single != s_hw_filter_single[i] ||
                (bus_want_single && bus_want_id != s_hw_filter_id[i])) {
                if (g_can_ok[i] && g_can[i]) {
                    g_can[i]->setAcceptanceFilter(bus_want_single, bus_want_id);
                    s_hw_filter_single[i] = bus_want_single;
                    s_hw_filter_id[i] = bus_want_id;
                }
            }
        }
    }
#endif  // !FSD_NO_WIFI — dashboard + the stream's acceptance-filter driver

#if defined(BOARD_TTGO_DISPLAY)
    s = state_snapshot();
    display_set_enabled(s.display_enabled);
    display_set_brightness(s.display_brightness);

    if (s.fsd_enabled && !g_last_fsd_enabled) {
        display_wake();
        g_display_last_wake_ms = now;
    }
    g_last_fsd_enabled = s.fsd_enabled;

    if (display_is_awake() && s.display_timeout_s > 0) {
        if (now - g_display_last_wake_ms >= s.display_timeout_s * 1000) {
            display_sleep();
        }
    }

    display_update(&s);
#endif

    update_led();
}
