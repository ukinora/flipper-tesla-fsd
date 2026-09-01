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
#define BLE_PROTO_VERSION 5

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
// 🔴 The recorder had no reachable switch and no reachable trigger.
//
// blackbox_set_enabled() was only ever called from the web dashboard and from a
// boot path guarded by the flag it sets -- so on this board it could only be
// turned on if it was already on. blackbox_mark() had exactly one caller, also
// the dashboard. Removing the dashboard (PR #28) therefore left the module
// unable to record anything, which is the one thing the schedule depends on:
// the capture taken before the TSL comes out cannot be retaken.
#define BLE_CMD_BB_ENABLE    0x34u  // arg: 0 = off, 1 = on. Persisted.
#define BLE_CMD_BB_MARK      0x35u  // record a window around NOW
/* Delete every stored capture. arg MUST be 1; anything else is REJECTED.
 *
 * 🔴 The confirmation is the point, and it is the same one the serial command
 * demands ("bbclear yes"). This deletes the capture taken before the TSL comes
 * out and that capture cannot be taken again, so a stray byte must not be
 * enough to do it.
 *
 * It exists because the disk holds ONE capture at a time and clearing it was
 * only possible over USB. At the car the laptop is the thing most likely to be
 * missing, and without this the second capture of the visit is refused with no
 * way forward — which would mean putting the TSL back.
 *
 * Answered from loop(): blackbox_delete_all() walks the filesystem deleting
 * files, and the BLE host task must not do that. `extra` carries the number of
 * captures STILL stored afterwards, read back rather than assumed, so 0 is the
 * only shape of success. */
#define BLE_CMD_BB_CLEAR     0x36u  // arg: 1 = do it. Any other value refuses.
#define BLE_CMD_CAP_RECHECK  0x40u  // re-run the capability listen window
#define BLE_CMD_SET_AUTONOMY 0x41u  // arg: 0/1 — operator intent, persisted
#define BLE_CMD_PING         0x50u

/* ── Bluetooth button (BLE Central) ───────────────────────────────────────────
 *
 * 🔴 These exist because the button was reachable only over USB. ble_central.cpp
 * has had scan/bind since PR #21, but only as SERIAL commands — and the phone
 * has no serial port, so the app's button screen had nothing to call.
 *
 * The results are published in the capability JSON rather than a new
 * characteristic: that document already answers "what does the module see",
 * and the app already re-reads it on demand. */
#define BLE_CMD_BTN_SCAN     0x60u  // arg: seconds (0 = default). Runs on loop().
#define BLE_CMD_BTN_BIND     0x61u  // arg: index into the last scan's results
#define BLE_CMD_BTN_FORGET   0x62u  // arg: slot to drop, 0xFF = all

/* Turn ONE row on or off. A row is (logical button, event) — see ble_central.h.
 *
 *    arg = (row << 1) | (1 = on, 0 = off)
 *
 * There is no separate double-press command: enabling the DOUBLE row IS turning
 * double-press on. One truth, so the two cannot drift apart.
 *
 * 🔴 It is per button because the COST is per button. Watching for a second tap
 * means the first one cannot be reported until the window closes, so every
 * single press on that button is delayed by FSD_BTN_DOUBLE_MS. On a control
 * where the feel matters — stepping the speed profile — that latency is the
 * control. fsd_button.h refused double-press outright for exactly this reason
 * before the cost was made opt-in.
 *
 * Persisted: it is a mapping decision, not a session one. */
#define BLE_CMD_BTN_ACTION   0x63u

/* ── The owner's rules ───────────────────────────────────────────────────────
 *
 * A rule is thirteen bytes ([slot][12]) and the Command characteristic carries
 * one argument byte, so writing a rule goes to the RULES characteristic
 * instead. These two codes are what the RESULT is tagged with.
 *
 * 🔴 0x70 IS A RESULT TAG, NOT A COMMAND. Sent on the Command characteristic it
 * falls through to UNSUPPORTED, which is correct — there is nowhere in one byte
 * to put a rule. It exists so the answer to a RULES write is a Result like every
 * other answer, instead of a second reply channel with its own rules.
 *
 * `extra` on both is (slot << 8) | FsdRuleVerdict. The slot is echoed for the
 * reason BTN_BIND echoes its index: one parking slot means an answer can arrive
 * after the app has moved on, and a reply that does not say what it is about can
 * be credited to the wrong request. 0xFF in the high byte means "all".
 *
 * A verdict rather than a bare rejection because the refusals are things the
 * owner has to be told: fsd_rules_set() turns away a rule that could never fire
 * (a switch cannot ENTER a state, a state cannot be PRESSED), and a control that
 * silently did nothing is the failure that validator exists to prevent. */
#define BLE_CMD_RULE_SET     0x70u  // result tag for a RULES write. Never sent to us.
#define BLE_CMD_RULE_CLEAR   0x71u  // arg: rule number, 0xFF = all

// ── Camera / autonomy status (read + notify) ─────────────────────────────────
// A separate characteristic rather than more bytes in State: State is a fixed
// 20 and full, and widening it would bump the wire version for every client.
//
// v2 (20 B). The tracker, the GPS layer and the policy now RUN — camera_task.cpp
// instantiates them — so the fields that were deliberately absent in v1 exist.
// Bytes 0..11 keep their exact v1 meaning, so a reader that ignores the version
// and parses the first 12 is still correct; the bump is for length-strict
// clients, which is what this version number was split off for.
//
// 20 B is the ceiling a default 23-byte ATT MTU carries, the same limit State
// already respects.
//
// Everything here is OBSERVED, not commanded. The policy's decision is
// published and never acted on: this firmware has no path from a decision to a
// CAN frame — see camera_task.h.
//
//   [0]      protocol version (2)
//   [1]      flags: bit0 autonomy enabled (operator intent, persisted)
//                   bit1 supervised drive right now
//                   bit2 camera database loaded
//                   bit3 fsd_autonomy_allows() — the camera path's only door
//                   bit4 policy suspended by DRIVER OVERRIDE (see below)
//                   bit5 learning dirty — passes recorded, not yet written
//                   bit6 0x3FD read-back fresh AND inside 0..3
//                   bit7 learning writes are failing
//   [2]      FsdSupVerdict — which supervision gate is refusing (0 = none)
//   [3]      OpMode
//   [4..7]   cameras in the database, LE32
//   [8..11]  database build time, Unix seconds LE32 (0 = unknown/old file)
//   [12]     FsdGpsVerdict — why there is no fix (0 = OK)
//   [13]     FsdPolPhase bits0-2 | FsdPolAction bits3-4 | target profile bits5-6
//   [14..15] metres to the nearest tracked camera, LE16
//            (0xFFFF = none, 0xFFFE = 65534 m or more — unreachable in practice,
//             the scan radius is 600 m, but the field cannot say "65535 m")
//   [16]     MCU_gpsAccuracy, raw 0.2 m units (0xFF = unknown)
//   [17]     last decoded 0x3FD profile, RAW 0..7 (0xFF = never decoded)
//   [18]     cameras with stored learning
//   [19]     scans that came back full — possible truncation, saturating
//
// bit4 covers only HALF of suspension, on purpose. The other half —
// fsd_pol_on_convergence_failed(), the 페일세이프 §5-E guard against an
// unattended module retrying forever — has no caller anywhere in this firmware,
// because there is no convergence machine to fail. Reporting it as if it could
// fire would be the same "zero that means not implemented" mistake v1's comment
// warned about.
//
// Its own version, not BLE_PROTO_VERSION. Sharing one constant meant that
// bumping State to 2 also announced a CamStat change that never happened, and
// an app checking the byte would have re-parsed for nothing. Two independent
// wire formats need two independent version numbers.
#define BLE_CAMSTAT_VERSION 2u
#define BLE_CAMSTAT_LEN 20u
#define BLE_CAMSTAT_PERIOD_MS 1000u

// ── Result codes (Result characteristic, byte 1) ─────────────────────────────
#define BLE_RES_OK          0u
#define BLE_RES_REJECTED    1u
#define BLE_RES_TIMEOUT     2u
#define BLE_RES_UNSUPPORTED 3u
#define BLE_RES_SAFETY      4u  // blocked by a safety gate (e.g. moving vehicle)
#define BLE_RES_NOT_FOUND   5u  // nothing to download
#define BLE_RES_BUSY        6u  // a transfer is already running, or a capture is being written

/* DUMP_START 가 BUSY 일 때 `extra` 에 실리는 이유 (2026-08-31 레드팀 ⑥).
 *
 * 🔴 둘 다 "잠시 뒤 다시 물어라" 지만 **사람에게는 전혀 다른 소식**이다:
 *   TRANSFER = 이미 받고 있다 (잘못 눌렀다)
 *   SAVING   = 모듈이 캡처를 쓰고 있다 (정상이니 기다려라) */
#define BLE_BUSY_NONE       0u
#define BLE_BUSY_TRANSFER   1u
#define BLE_BUSY_SAVING     2u
/* 🔴 A BB_CLEAR is already parked and waiting for loop() to run it. This used to
 * answer BLE_BUSY_TRANSFER, which the phone renders as "another download is
 * running" -- and nothing is downloading. The reasons exist precisely because
 * the app tells the owner different things: one means "you pressed the wrong
 * thing", the other means "this is normal, wait". Reusing one for a third
 * meaning gives back the distinction the field was added for. */
#define BLE_BUSY_CLEARING   3u
// The link is encrypted and bonded, but this phone is not the one the module
// was set up with. Distinct from REJECTED so the app can say so instead of
// showing a generic failure — the fix is a button press in the car, which the
// user has no way to guess from "rejected". See ble_owner.h.
#define BLE_RES_NOT_OWNER   7u

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
/* Minimum gap between bursts, in ms — the capture download's speed limit.
 *
 * BLE_BULK_CHUNKS_PER_TICK notifications go out per burst, so this sets the
 * notification RATE, which is what actually breaks: Android silently drops
 * notifications above a few hundred per second and there is no ATT ack to
 * notice it. Measured 2026-08-17 — unpaced (~650/s) lost frames mid-transfer;
 * 4 per 20 ms is ~200/s.
 *
 * Raise it if a capture still arrives with a sequence gap. Lower it only with a
 * full download to prove it, and remember the traffic that matters is a capture
 * that cannot be recorded a second time. */
#define BLE_BULK_TICK_MS         20u

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

/** True once the GATT server started and the radio began advertising.
 *
 *  This is "the stack came up", not "someone is talking to us" -- deliberately.
 *  It is a health signal for the OTA self-test, which has to be answerable on a
 *  bench with no phone in the room. ble_server_connected() cannot do that job:
 *  it would make accepting a firmware depend on who happens to be nearby. */
bool ble_server_up(void);

#else  // BLE disabled — no-op shims so main.cpp needs no #ifdef

static inline void ble_server_init(FSDState *, portMUX_TYPE *) {}
static inline void ble_server_tick(uint32_t) {}
static inline bool ble_server_connected(void) { return false; }
// Nothing to bring up on this variant, so nothing can have failed to. Returning
// false here would make the OTA self-test unpassable on every non-BLE board.
static inline bool ble_server_up(void) { return true; }

#endif
