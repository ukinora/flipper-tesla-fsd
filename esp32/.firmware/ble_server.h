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

// How long Active survives a dropped BLE link before it reverts to Listen-Only.
// Long enough to ride out an app switch or a screen blank, short enough that a
// phone left behind does not leave the module transmit-capable.
#define BLE_ACTIVE_GRACE_MS 30000u  // 30 s

// ── Command codes (Command characteristic, byte 0) ───────────────────────────
#define BLE_CMD_SET_MODE     0x01u  // arg: 0 = Listen-Only, 1 = Active
#define BLE_CMD_SET_PROFILE  0x10u  // arg: target profile 0..3   (closed loop; TODO)
#define BLE_CMD_PROFILE_STEP 0x11u  // arg: int8 +1 / -1          (closed loop; TODO)
#define BLE_CMD_DUMP_START   0x30u  // arg: 0 = .log (candump), 1 = .json summary
#define BLE_CMD_DUMP_STOP    0x31u
#define BLE_CMD_CAP_RECHECK  0x40u  // re-run the capability listen window
#define BLE_CMD_PING         0x50u

// ── Result codes (Result characteristic, byte 1) ─────────────────────────────
#define BLE_RES_OK          0u
#define BLE_RES_REJECTED    1u
#define BLE_RES_TIMEOUT     2u
#define BLE_RES_UNSUPPORTED 3u
#define BLE_RES_SAFETY      4u  // blocked by a safety gate (e.g. moving vehicle)
#define BLE_RES_NOT_FOUND   5u  // nothing to download
#define BLE_RES_BUSY        6u  // a transfer is already running

// ── Bulk framing (Bulk characteristic, notify) ───────────────────────────────
// On a T-2CAN there is no SD card and we do not bring WiFi up, so this is the
// only way a capture leaves the module.
//
//   [0..1] seq, LE16.  0 = header, 1.. = data
//   header : [2..5] total bytes LE32, [6..] capture name (no NUL)
//   data   : [2..]  file bytes at the running offset
//   EOF    : a frame with no payload (length 2)
//
// Chunk size follows the negotiated MTU, so a phone that negotiates 517 moves
// ~500 B per notification and a 23-byte default still works, just slowly.
#define BLE_BULK_MAX_PAYLOAD   500u
#define BLE_BULK_CHUNKS_PER_TICK 4u

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
