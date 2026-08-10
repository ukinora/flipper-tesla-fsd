#pragma once
/*
 * ble_server.h — BLE GATT server (module <-> phone app).
 *
 * The WiFi dashboard stays as-is for desk debugging; this is a parallel
 * transport for the phone app, which is the intended in-car interface.
 * Both read the same FSDState, so neither owns the state.
 *
 * Layout and rationale: docs BLE-GATT-프로토콜.md (draft v0.1).
 *
 *   State      notify   20 B fixed, 5 Hz   — vehicle/module state
 *   Command    write    1..20 B            — app -> module
 *   Result     indicate 4 B                — command ack (must not be lost)
 *   Capability read     JSON               — per-bus verdicts, read once on connect
 *
 * Only compiled when BLE_SERVER_ENABLED is defined (see platformio.ini), so
 * boards that don't need it keep their flash footprint unchanged.
 */

#include "../../fsd_logic/fsd_state.h"
#include <freertos/FreeRTOS.h>
#include <stdint.h>

// Wire protocol version, first byte of every State notification.
#define BLE_PROTO_VERSION 1

// State notify cadence. Slowed automatically while no client is subscribed.
#define BLE_STATE_PERIOD_MS 200u  // 5 Hz

// ── Command codes (Command characteristic, byte 0) ───────────────────────────
#define BLE_CMD_SET_MODE     0x01u  // arg: 0 = Listen-Only, 1 = Active
#define BLE_CMD_SET_PROFILE  0x10u  // arg: target profile 0..3   (closed loop; TODO)
#define BLE_CMD_PROFILE_STEP 0x11u  // arg: int8 +1 / -1          (closed loop; TODO)
#define BLE_CMD_DUMP_START   0x30u  // arg: bus mask              (TODO)
#define BLE_CMD_DUMP_STOP    0x31u
#define BLE_CMD_CAP_RECHECK  0x40u  // re-run the capability listen window
#define BLE_CMD_PING         0x50u

// ── Result codes (Result characteristic, byte 1) ─────────────────────────────
#define BLE_RES_OK          0u
#define BLE_RES_REJECTED    1u
#define BLE_RES_TIMEOUT     2u
#define BLE_RES_UNSUPPORTED 3u
#define BLE_RES_SAFETY      4u  // blocked by a safety gate (e.g. moving vehicle)

#ifdef BLE_SERVER_ENABLED

/** Start advertising and register the GATT service. Safe to call once from setup(). */
void ble_server_init(FSDState *state, portMUX_TYPE *state_mux);

/** Pump from loop(): pushes State notifications on schedule. Cheap when idle. */
void ble_server_tick(uint32_t now_ms);

/** True while a phone is connected AND subscribed to State. */
bool ble_server_connected(void);

#else  // BLE disabled — no-op shims so main.cpp needs no #ifdef

static inline void ble_server_init(FSDState *, portMUX_TYPE *) {}
static inline void ble_server_tick(uint32_t) {}
static inline bool ble_server_connected(void) { return false; }

#endif
