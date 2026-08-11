#pragma once
/*
 * camera_store.h — holds camera.bin on the module and takes it over BLE.
 *
 * The module judges cameras by itself now, so the database has to live here.
 * It is 163 KB, which rules out RAM (see fsd_camera.h), so it sits in LittleFS
 * and fsd_camera reads it a few hundred bytes at a time.
 *
 * REPLACEMENT IS ATOMIC
 * ---------------------
 * An upload writes to camera.tmp and only becomes camera.bin after the size
 * and CRC check out. A transfer that dies halfway leaves the previous database
 * untouched — the alternative is a car driving around with a half-written file
 * and no cameras in it, which is worse than an out-of-date one.
 *
 * The open database file is closed for the duration of an upload and reopened
 * afterwards, so a lookup never reads a file that is being replaced.
 *
 * TWO TASKS TOUCH THIS FILE
 * -------------------------
 * The upload path runs on the NimBLE host task; the judgement path (camera_task)
 * runs on the Arduino loop task and reads the database once a second. That was
 * harmless while nothing on the loop task did file I/O, and stopped being
 * harmless the moment camera_task started calling fsd_cam_near().
 *
 * The sharp edge is not the upload itself but camera_store_upload_abort(): it is
 * called from ServerCB::onDisconnect, so it runs on EVERY ordinary disconnect —
 * the phone walking out of range, the app going to the background. It used to
 * close and reopen the database file every time. A lookup in flight would have
 * had the File closed underneath it, and fsd_cam_open() memsets the FsdCamDb
 * while fsd_cam_near() is reading rec_offset out of it.
 *
 * So: an abort with nothing to abort now returns immediately, and the
 * transitions that genuinely do replace the database take a mutex that the
 * borrow API below also takes.
 */

#include "../../fsd_logic/fsd_cam_track.h"
#include "../../fsd_logic/fsd_camera.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Upload result codes, reported back through the BLE Result characteristic.
#define CAM_UP_OK 0u
#define CAM_UP_BUSY 1u        // an upload is already running
#define CAM_UP_NO_FS 2u       // filesystem unavailable
#define CAM_UP_TOO_BIG 3u     // will not fit
#define CAM_UP_BAD_SEQ 4u     // chunk out of order — a gap means silent holes
#define CAM_UP_WRITE_FAIL 5u  // flash write failed
#define CAM_UP_SIZE_MISMATCH 6u
#define CAM_UP_BAD_CRC 7u
#define CAM_UP_BAD_FORMAT 8u  // not a camera.bin
#define CAM_UP_NOT_ACTIVE 9u

// Leave room for the black box and its rotation; refuse anything larger.
#define CAM_MAX_BYTES (512u * 1024u)

/** Mount and open the database if one is present. Safe to call once from
 *  setup(); it does not fail the boot when there is no database yet. */
void camera_store_init(void);

/** True when a valid database is open and can be queried. */
bool camera_store_ready(void);

/** Borrow the open database for ONE bounded read burst.
 *
 *  Takes an internal lock and holds it until the matching release. Returns NULL
 *  — with the lock already given back — when there is no usable database or the
 *  wait expired, so the contract is exactly:
 *
 *      const FsdCamDb* db = camera_store_db_acquire(10);
 *      ... reads ...
 *      if(db) camera_store_db_release();
 *
 *  Release if and only if it returned non-NULL. There is no hidden ownership
 *  flag to get out of step with.
 *
 *  Do NOT cache the pointer across calls. close_db() clears FsdCamDb.ok, which
 *  is the only self-check fsd_cam_near() makes, so a stale borrow degrades to
 *  "no cameras anywhere" — quiet, plausible, and wrong. */
const FsdCamDb* camera_store_db_acquire(uint32_t wait_ms);
void camera_store_db_release(void);

/** How many cameras are loaded (0 when none). */
uint32_t camera_store_count(void);

// ── Upload (phone -> module) ────────────────────────────────────────────────

/** Begin receiving a new database of `total` bytes. Closes the current one. */
uint8_t camera_store_upload_begin(uint32_t total);

/** Append one chunk. `seq` must be the next expected sequence number: a gap
 *  would punch an invisible hole in the database, so it is an error rather
 *  than something to paper over. */
uint8_t camera_store_upload_chunk(uint16_t seq, const uint8_t* data, size_t len);

/** Finish: verify size, CRC and format, then swap it in. */
uint8_t camera_store_upload_end(void);

/** Give up. Removes the temp file and reopens whatever was there before. */
void camera_store_upload_abort(void);

/** True while an upload is in flight (lookups are unavailable then). */
bool camera_store_uploading(void);

/** Bytes received so far — for progress reporting. */
uint32_t camera_store_upload_progress(void);

// ── Learned lane discrimination (fsd_cam_track.h) ───────────────────────────
// Kept next to the database because it is the same kind of thing: a file the
// judgement core needs and cannot rebuild by itself. Without it the module
// forgets every pass when the car sleeps, so each drive restarts at the wide
// default limit and warns on the opposite carriageway all over again.

/** Read learning back at boot. Leaves the tracker empty when there is no file
 *  or it does not check out — see fsd_trk_load(). */
bool camera_store_load_learning(FsdTracker* t);

/** Write it out, atomically. Cheap enough to call at the end of a drive; do not
 *  call it on every pass, since it rewrites the whole file. Clears t->dirty
 *  only when the bytes are actually in place. */
bool camera_store_save_learning(FsdTracker* t);
