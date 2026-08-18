#pragma once
/*
 * capability.h — tap capability checker (ESP32, #125).
 *
 * Listens passively for a few seconds, counts capability-relevant CAN ids per
 * id per bus, then calls the pure verdict logic (fsd_logic/fsd_capability.h) to
 * answer per FEATURE — not per bus name — "will the nag killer / AP-First / FSD
 * activation / Soft Engage work on the tap this device is plugged into?".
 *
 * Pure RX / read-only: this module never transmits. It only reads the same RX
 * frames process_frame() already handles. An id counts as "present" once it has
 * been seen >= CAP_MIN_FRAMES times in the window (ignores a single spurious
 * frame). A check runs automatically when the dashboard connects and on demand
 * via the "capability_recheck" WS command.
 *
 * Single-threaded counters: capability_record() is called only from the CAN
 * path; capability_status_json() reads from the web task. Counters are small and
 * monotonic within a window, so a torn read at the >= CAP_MIN_FRAMES threshold
 * is harmless.
 */

#include <Arduino.h>
#include "can_driver.h"   // CanBusId, CAN_BUS_COUNT
#include "fsd_handler.h"  // FSDState, CanFrame

// Listen window and presence threshold (see fsd_logic/fsd_capability.h).
#define CAP_WINDOW_MS    4000u
#define CAP_MIN_FRAMES   3u

// Wire up the shared state (for hw_version) + its mux. Call once from setup().
void capability_init(FSDState* state, portMUX_TYPE* state_mux);

// Count one RX frame toward the active window. Cheap; no-op when not running.
void capability_record(CanBusId bus, const CanFrame& frame, uint32_t now_ms);

// Begin (or restart) a listen window: zero the counters and arm the deadline.
void capability_start(uint32_t now_ms);

// Finalize the window once CAP_WINDOW_MS has elapsed. Call once per loop().
void capability_tick(uint32_t now_ms);

// Dashboard payload: {"state":..,"ms_left":..,"hw":..,"buses":[{...}]}.
/* ── which capability-relevant CAN ids were seen, as bit positions ───────────
 *
 * 🔴 APPEND ONLY. android/wire/Capability.kt decodes this back into named
 * fields in exactly this order; inserting a bit in the middle renames every
 * flag above it, and nothing would say so — the document still parses and the
 * app still greys out features, just the wrong ones.
 *
 * These were eleven named booleans in the JSON until 2026-08-18. That cost
 * ~190 bytes per bus and pushed the document past the 512-byte ATT limit,
 * which NimBLE answers by storing NOTHING. */
#define CAP_ID_BIT_EPAS         (1u << 0)
#define CAP_ID_BIT_DAS_HW4      (1u << 1)
#define CAP_ID_BIT_DAS_HW3      (1u << 2)
#define CAP_ID_BIT_AP_CTRL      (1u << 3)
#define CAP_ID_BIT_AP_LEGACY    (1u << 4)
#define CAP_ID_BIT_STEER        (1u << 5)
#define CAP_ID_BIT_BODY_UI      (1u << 6)
#define CAP_ID_BIT_BODY_DOOR    (1u << 7)
#define CAP_ID_BIT_BODY_WINDOW  (1u << 8)
#define CAP_ID_BIT_BODY_LIGHTS  (1u << 9)
#define CAP_ID_BIT_SCROLL       (1u << 10)

/* The largest value a GATT attribute may hold.
 *
 * 🔴 SPEC, NOT SETTING. BLE_ATT_ATTR_MAX_LEN is 512 in the Bluetooth core
 * specification, so raising the MTU does not raise this. NimBLE REJECTS an
 * oversized setValue() rather than truncating it, leaving the characteristic
 * empty — which a reader cannot tell apart from "not written yet".
 *
 * The Capability document outgrew this on 2026-08-18 (566 bytes) and the phone
 * read nothing for as long as that build was on the board. */
#define CAP_ATTR_MAX 512u

/* How often the Capability document is rebuilt.
 *
 * 🔴 It was built ONCE, ten seconds after boot, and never again — so anything
 * that changed later (a remote connecting, presses arriving, a scan finishing)
 * could not reach the phone at all.
 *
 * 500 ms pairs with the app's read cadence: together they put a press on the
 * screen within a second. The document is ~400 bytes, so rebuilding it twice a
 * second is a String build and nothing else.
 *
 * ⚠️ This is SCREEN latency only. The module classifies a press within about
 * 100 ms of the report arriving (BLE interval + a 50 ms tick) and acts on it
 * there; the phone is a spectator. Do not read this number as reaction time. */
#define CAP_REFRESH_MS 500u

/* Room kept for what is written after the scan list. */
#define CAP_TAIL_RESERVE 48u

/** The last scan's results, as their own document.
 *
 * 🔴 THEY USED TO RIDE IN THE CAPABILITY DOCUMENT and there is no longer room —
 * that document reached 483 of the 512 bytes an ATT attribute may hold, so the
 * scan list got nothing and the app's "find buttons" could not have shown a
 * result even once the scan itself was fixed.
 *
 * They never belonged there anyway: everything else in that document is a fact
 * about the module, bounded and slow-changing. A scan list is transient and its
 * size depends on what happens to be in the room. */
String capability_scan_json();

String capability_status_json();

/**
 * True while the listen window is still counting.
 *
 * Exists so a caller can notice the window CLOSING. The BLE Capability
 * characteristic used to be filled once in ble_server_init() -- at boot, before
 * a single frame had arrived -- and never again, so CAP_RECHECK re-ran the
 * probe and the result went nowhere. The dashboard had been rebuilding the JSON
 * live; nothing replaced that when it was removed.
 */
bool capability_running(void);
