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
 *   Bulk       notify   variable           — black-box capture download
 *   Upload     write    variable           — camera.bin, phone -> module
 *   CamStat    read+notify 12 B, 1 Hz      — camera database + autonomy gate
 *
 * There is no Config characteristic. The draft reserved 0004 for one; settings
 * go through Command instead, which costs one opcode rather than a second
 * serialisation format, and keeps every acknowledgement on Result.
 *
 * Only compiled when BLE_SERVER_ENABLED is defined (see platformio.ini), so
 * boards that don't need it keep their flash footprint unchanged.
 */

#include "../../fsd_logic/fsd_state.h"
#include <freertos/FreeRTOS.h>
#include <stdint.h>

// Wire protocol version, first byte of every State notification.
//   1 — initial layout; byte 9 carried DI_cruiseState as a stand-in
//   2 — byte 9 is the gear (PRND), which is what it was specified as; the
//       stand-in existed only because this build had no gear parser
#define BLE_PROTO_VERSION 2

// State notify cadence. With nobody subscribed the tick returns before the
// serialisation — it stops rather than slowing down, which is worth saying
// because the protocol draft claimed a 1 Hz background rate that never existed.
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
#define BLE_CMD_UPLOAD_ABORT 0x33u  // give up on an in-flight camera.bin upload
#define BLE_CMD_CAP_RECHECK  0x40u  // re-run the capability listen window
#define BLE_CMD_SET_AUTONOMY 0x41u  // arg: 0/1 — operator intent, persisted
#define BLE_CMD_PING         0x50u

// ── Camera / autonomy status (read + notify) ─────────────────────────────────
// A separate characteristic rather than more bytes in State: State is a fixed
// 20 and full, and widening it would bump the wire version for every client.
//
// It reports ONLY what the firmware actually knows today. The tracker and
// policy are compiled but not instantiated — nothing feeds them a position yet
// — so their fields are absent rather than present-and-always-zero. Sending a
// zero that means "not implemented" is indistinguishable from a real zero, and
// this project has already been bitten by exactly that (di_cruise_state and the
// blinker flags were structurally zero on this build for months).
//
//   [0]     protocol version
//   [1]     flags: bit0 autonomy enabled (operator intent, persisted)
//                  bit1 supervised drive right now
//                  bit2 camera database loaded
//   [2]     FsdSupVerdict — which gate is refusing (0 = none)
//   [3]     OpMode
//   [4..7]  cameras in the database, LE32
//   [8..11] database build time, Unix seconds LE32 (0 = unknown/old file)
#define BLE_CAMSTAT_LEN 12u
#define BLE_CAMSTAT_PERIOD_MS 1000u

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

// ── Upload framing (Upload characteristic, write) ────────────────────────────
// The mirror image of Bulk: the phone pushes camera.bin to the module. The
// module cannot judge cameras without a database, and 163 KB will not fit in
// any other channel we have.
//
//   [0..1] seq, LE16.  0 = header, 1.. = data
//   header : [2..5] total bytes LE32
//   data   : [2..]  file bytes, in order
//
// Sequence numbers must not skip. A gap would punch a hole in the database
// that nothing downstream could detect, so the module rejects the transfer
// instead of storing something quietly damaged. Completion is implicit: once
// the declared byte count has arrived the module verifies and swaps it in.
//
// Written without response for throughput; the module never blocks on a chunk.
#define BLE_UPLOAD_HDR 2u

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
