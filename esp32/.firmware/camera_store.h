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
 */

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

/** The open database, or NULL. Borrowed — do not keep it across an upload. */
const FsdCamDb* camera_store_db(void);

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
