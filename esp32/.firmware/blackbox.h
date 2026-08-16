#pragma once
/*
 * blackbox.h — black-box incident recorder (ESP32, #124).
 *
 * A RAM ring buffer continuously records raw CAN frames — by default only the
 * key diagnostic ids (fsd_blackbox_filter.h), across all buses, so a busy full
 * bus (~3300 f/s) can't fill the small ring in ~1.8 s and truncate the window;
 * define BLACKBOX_CAPTURE_ALL to record every id instead. When the shared
 * event-core (fsd_events.h) reports an abort, a
 * bus-off, or a dashboard "Mark", the recorder freezes a window of pre+post
 * frames and writes two files: a pure candump .log (drops straight into the
 * CRC cracker) and a decoded .json summary (fsd_blackbox_summary.h).
 *
 * The ring lives in PSRAM when present (large window) and falls back to a
 * smaller internal-RAM window otherwise — PSRAM is never required.
 *
 * Storage is one of three backends chosen at compile time by board (see
 * blackbox.cpp): LittleFS (real data partition, persistent, retention N=5),
 * SD (LILYGO), or volatile RAM + dashboard download (min_spiffs boards).
 *
 * Default OFF; the enable toggle is persisted in NVS. The ring is allocated
 * lazily (heap-guarded) on enable — boot never grabs it, so WiFi/web always
 * get the heap first. Single-threaded: every entry point is called from the
 * Arduino loop()/CAN path.
 */

#include <Arduino.h>
#if !defined(FSD_NO_WIFI)
// Only for the WiFiClient in blackbox_stream_body() below. Including it anyway
// on a no-WiFi build is not inert: PlatformIO's dependency finder reads these
// includes, so libWiFi.a gets built and handed to the linker. No member is
// adopted and the ELF stays clean, but "we removed WiFi" and "the WiFi library
// is on the link line" should not both be true.
#include <WiFi.h>
#endif
#include "can_driver.h"   // CanBusId, CanFrame
#include "fsd_handler.h"  // FSDState

// ── Storage backend selection (single source of truth) ───────────────────────
// Chosen by board, like can_dump.cpp's per-board #if. blackbox.cpp implements
// the matching path; the enable-default below keys off this too.
#if defined(BOARD_LILYGO)
  #define BLACKBOX_BACKEND_SD       1
  #define BLACKBOX_BACKEND_NAME     "sd"
#elif defined(BOARD_WAVESHARE_S3) || defined(BOARD_LILYGO_T2CAN)
  #define BLACKBOX_BACKEND_LITTLEFS 1
  #define BLACKBOX_BACKEND_NAME     "littlefs"
#else
  #define BLACKBOX_BACKEND_RAM      1
  #define BLACKBOX_BACKEND_NAME     "ram"
#endif

// Default enable state: OFF on every backend. A fresh flash must boot with the
// recorder off so WiFi/dashboard always come up — the ~108 KB ring on a
// no-PSRAM S3 can starve the WiFi/web stack if grabbed at boot (#124). The heap
// guard in blackbox_set_enabled() protects the allocation.
//
// 🔴 "The user opts in from the dashboard" is what this used to say, and on
// lilygo-t2can that stopped being true when the dashboard was removed (PR #28):
// the only other caller of blackbox_set_enabled() is a boot path guarded by the
// very flag it sets, so the recorder could only be switched on if it was
// already on. Opting in now goes through BLE (BB_ENABLE) or serial (bbon).
#define BLACKBOX_DEFAULT_ENABLED  false

enum BBTrigger : uint8_t {
    BB_TRIG_ABORT = 0,
    BB_TRIG_BUSOFF,
    BB_TRIG_MANUAL,
};

// Wire up state + storage backend and make the PSRAM/size decision. Does NOT
// allocate the ring — that happens lazily, heap-guarded, on enable (#124), so
// boot never starves the WiFi/web stack. Pass the shared state + mux so marks
// can inject through the event-core and flushes can snapshot toggles.
void blackbox_init(FSDState* state, portMUX_TYPE* state_mux);

// Record one RX frame into the ring. Cheap (id filter + memcpy); no-op when
// disabled or when the id isn't a key diagnostic id (see fsd_blackbox_filter.h).
void blackbox_record(CanBusId bus, const CanFrame& frame, uint32_t now_ms);

// Record one injected TX frame into the ring — same id filter and ring path as
// blackbox_record, tagged TX so the capture distinguishes our frames from the
// bus. Called centrally from send_on_bus() so every injection (0x3EE / 0x3FD /
// the 0x370 nag echo) is captured without touching each call site.
void blackbox_record_tx(CanBusId bus, const CanFrame& frame, uint32_t now_ms);

// Note the current das_ap_state for the mini-timeline (call once per frame).
void blackbox_note_ap_state(uint8_t ap_state, uint32_t now_ms);

// Arm a capture from an abort/bus-off detected on the CAN path. `snap` is a
// snapshot of FSDState at trigger time (toggles, hw, evt_last_*). Ignored when
// disabled or a capture is already in flight.
void blackbox_arm(BBTrigger trig, const FSDState* snap, uint32_t now_ms);

// Inject a bus-off / manual mark through the event-core (applies the cooldown)
// and arm a capture if it fires. Used from the loop (bus-off) and dashboard.
void blackbox_busoff(uint32_t now_ms);
/** Arm a manual capture.
 *
 * @return milliseconds until the capture becomes a FILE, or 0 when nothing was
 *         armed (the event cooldown swallowed it, or the recorder is off).
 *
 * Non-zero does NOT mean the capture is downloadable yet — it means "wait this
 * long, then it will be". Asking for a download before then serves the PREVIOUS
 * capture and reports success, which is how the wrong file gets taken to the car. */
uint32_t blackbox_mark(uint32_t now_ms);

// Post-roll countdown + flush. Call once per loop().
void blackbox_tick(uint32_t now_ms);

// Enable/disable + persist; off stops recording and clears any armed capture.
void blackbox_set_enabled(bool enabled);
bool blackbox_is_enabled();

/** Captures written since boot. The honest answer to "did my MARK take?" --
 *  blackbox_mark() can be swallowed by the event cooldown, and at the one
 *  moment that matters the operator must not be told it worked when it did
 *  not. */
uint32_t blackbox_capture_count();

/** True when the storage this backend needs is mounted and usable.
 *
 *  Distinct from blackbox_is_enabled(), which is the operator's choice. This is
 *  whether the filesystem came up at all -- a health signal for the OTA
 *  self-test, and the same filesystem the camera database lives on. Always true
 *  on the RAM backend, which has nothing to mount. */
bool blackbox_storage_ok();

// Dashboard helpers.
String blackbox_status_json();              // {"enabled":..,"backend":..,...}
String blackbox_list_json();                // [{"name":..,"summary":{...}},..]
// Download: report the byte size of an event's .log/.json (false if missing),
// then stream just the body to the client. The caller owns the HTTP headers.
bool   blackbox_file_size(const char* name, bool json, size_t* size_out);
#if !defined(FSD_NO_WIFI)
void   blackbox_stream_body(WiFiClient& client, const char* name, bool json);
#endif

// Transport-independent download. blackbox_stream_body() is bound to a
// WiFiClient, which BLE cannot use — and on a T-2CAN we do not intend to bring
// WiFi up at all, so this is the only way a capture leaves the module.
//
// Reads up to `cap` bytes starting at `offset`; returns the byte count (0 =
// EOF or missing). Stateless, so a caller can pump one chunk per tick and
// resume at the next offset when the notify queue backs up.
size_t blackbox_read_chunk(const char* name, bool json, size_t offset,
                           uint8_t* out, size_t cap);

// Name of the most recent capture (false when there is none). Enough for the
// common case — grab the dump that was just taken — without making the client
// parse the listing first.
bool   blackbox_latest_name(char* out, size_t cap);
bool   blackbox_delete(const char* name);
void   blackbox_delete_all();

/** Free bytes left for captures, 0xFFFFFFFF when the backend does not police it.
 *
 * 🔴 One capture is about 3 MB against a 3.5 MB partition (measured
 * 2026-08-17), so "how much room is left" is not a curiosity — it is the
 * difference between the next `mark` working and being refused. */
uint32_t blackbox_free_bytes();

/** How many captures are stored on disk right now. */
int blackbox_event_count();
