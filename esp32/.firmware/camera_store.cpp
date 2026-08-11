/*
 * camera_store.cpp — see camera_store.h.
 */

#include "camera_store.h"

// Only the T-2CAN build carries the camera core (see platformio.ini
// build_src_filter). Guarding the whole file keeps the other variants linking
// and their flash footprint unchanged — the same approach ble_server.cpp uses.
#ifdef BLE_SERVER_ENABLED

#include <Arduino.h>
#include <LittleFS.h>
// Explicit rather than inherited through Arduino.h: this file's correctness now
// depends on the mutex existing, and an include that arrives by accident is one
// framework bump away from not arriving.
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define CAM_PATH "/camera.bin"
#define CAM_TMP "/camera.tmp"
#define LEARN_PATH "/learn.bin"
#define LEARN_TMP "/learn.tmp"

static bool g_fs_ok = false;
static File g_db_file;    // held open so lookups are just seek + read
static FsdCamDb g_db;
static bool g_db_ok = false;

/* Guards g_db_file and g_db against the two tasks that reach them: the upload
 * path on the NimBLE host task and the judgement path on the loop task. A mutex
 * rather than a binary semaphore, because the two tasks run at different
 * priorities and this one wants priority inheritance. */
static SemaphoreHandle_t g_db_lock = nullptr;

/* Set while learning is being written. Both files live on the same LittleFS, and
 * an upload arriving mid-save would interleave a multi-megabyte write with a
 * rename; the upload is the one that can be told to come back later. */
static volatile bool g_saving = false;

static File g_up_file;
static bool g_up_active = false;
static uint32_t g_up_total = 0;
static uint32_t g_up_written = 0;
static uint16_t g_up_next_seq = 1; // seq 0 is the header frame
static uint32_t g_up_crc = 0;      // over the body, header excluded

// zlib-compatible CRC-32. Bitwise so there is no 1 KB table to carry; a 163 KB
// upload is a one-off, and it is computed while the bytes stream past anyway.
static uint32_t crc32_update(uint32_t crc, const uint8_t* d, size_t n) {
    crc = ~crc;
    while(n--) {
        crc ^= *d++;
        for(int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

/** Reader handed to fsd_camera. The file stays open; a lookup is seek + read. */
static size_t db_read(void* ctx, uint32_t offset, void* buf, size_t len) {
    (void)ctx;
    if(!g_db_file) return 0;
    if(!g_db_file.seek(offset)) return 0;
    int n = g_db_file.read((uint8_t*)buf, len);
    return n > 0 ? (size_t)n : 0;
}

static void close_db() {
    if(g_db_file) g_db_file.close();
    g_db_ok = false;
    /* Clear the struct's own flag too, not just ours. fsd_cam_near() checks
     * db->ok once on entry (fsd_camera.c) and nothing else; a borrow that
     * somehow outlived a close would otherwise go on seeking a released File
     * instead of returning "no cameras". */
    g_db.ok = false;
}

static bool open_db() {
    close_db();
    if(!g_fs_ok || !LittleFS.exists(CAM_PATH)) return false;
    g_db_file = LittleFS.open(CAM_PATH, "r");
    if(!g_db_file) return false;
    g_db_ok = fsd_cam_open(&g_db, db_read, nullptr);
    if(!g_db_ok) {
        Serial.println("[CAM] camera.bin present but unreadable");
        g_db_file.close();
    }
    return g_db_ok;
}

void camera_store_init(void) {
    if(!g_db_lock) g_db_lock = xSemaphoreCreateMutex();
    if(!g_db_lock) {
        Serial.println("[CAM] mutex alloc failed — database disabled");
        return; // g_fs_ok stays false: no lock, no shared file
    }

    // begin(true) formats on failure. blackbox_init() usually mounts first;
    // calling again is harmless and removes the ordering dependency.
    g_fs_ok = LittleFS.begin(true);
    if(!g_fs_ok) {
        Serial.println("[CAM] LittleFS mount failed");
        return;
    }
    // A leftover temp file means a previous upload died. It is not a database.
    if(LittleFS.exists(CAM_TMP)) LittleFS.remove(CAM_TMP);

    /* Learning gets the opposite treatment, because it is not interchangeable
     * with the database. learn.tmp is only ever a COMPLETE, CRC-covered file
     * that was written and closed successfully — the save path removes it on
     * any write failure — so a learn.tmp with no learn.bin means power vanished
     * in the gap between the remove and the rename. That gap is a real hazard
     * here: the accessory feed is switched and dies without warning. Finishing
     * the rename recovers the whole file; deleting it would throw away every
     * pass ever recorded. fsd_trk_load() still has to accept it. */
    if(!LittleFS.exists(LEARN_PATH) && LittleFS.exists(LEARN_TMP)) {
        if(LittleFS.rename(LEARN_TMP, LEARN_PATH))
            Serial.println("[CAM] recovered learning from an interrupted save");
    }

    if(open_db()) {
        Serial.printf("[CAM] %u cameras loaded (%u cells)\n",
                      (unsigned)g_db.rec_count, (unsigned)g_db.cell_count);
    } else {
        Serial.println("[CAM] no camera database — upload one from the app");
    }
}

bool camera_store_ready(void) {
    return g_db_ok && !g_up_active;
}

const FsdCamDb* camera_store_db_acquire(uint32_t wait_ms) {
    if(!g_db_lock) return nullptr;
    if(xSemaphoreTake(g_db_lock, pdMS_TO_TICKS(wait_ms)) != pdTRUE) return nullptr;
    if(!camera_store_ready()) {
        xSemaphoreGive(g_db_lock); // give it straight back: nothing to borrow
        return nullptr;
    }
    return &g_db;
}

void camera_store_db_release(void) {
    if(g_db_lock) xSemaphoreGive(g_db_lock);
}

uint32_t camera_store_count(void) {
    return g_db_ok ? g_db.rec_count : 0;
}

bool camera_store_uploading(void) {
    return g_up_active;
}

uint32_t camera_store_upload_progress(void) {
    return g_up_written;
}

uint8_t camera_store_upload_begin(uint32_t total) {
    if(g_up_active) return CAM_UP_BUSY;
    if(g_saving) return CAM_UP_BUSY; // learning is being written to the same FS
    if(!g_fs_ok) return CAM_UP_NO_FS;
    if(total < FSD_CAM_HEADER_SIZE || total > CAM_MAX_BYTES) return CAM_UP_TOO_BIG;

    // Stop serving lookups from a file we are about to replace. Under the lock:
    // a lookup may be mid-read on the other task.
    if(!g_db_lock) return CAM_UP_NO_FS;
    if(xSemaphoreTake(g_db_lock, pdMS_TO_TICKS(200)) != pdTRUE) return CAM_UP_BUSY;
    close_db();
    xSemaphoreGive(g_db_lock);

    LittleFS.remove(CAM_TMP);
    g_up_file = LittleFS.open(CAM_TMP, "w");
    if(!g_up_file) return CAM_UP_WRITE_FAIL;

    g_up_total = total;
    g_up_written = 0;
    g_up_next_seq = 1;
    g_up_crc = 0;
    g_up_active = true;
    Serial.printf("[CAM] upload started, %u B\n", (unsigned)total);
    return CAM_UP_OK;
}

uint8_t camera_store_upload_chunk(uint16_t seq, const uint8_t* data, size_t len) {
    if(!g_up_active) return CAM_UP_NOT_ACTIVE;
    if(g_saving) return CAM_UP_BUSY;
    if(!data || len == 0) return CAM_UP_OK; // nothing to do, not an error
    // A gap would leave a hole we could never see again. Refuse and let the
    // app restart rather than storing a database with invisible damage.
    if(seq != g_up_next_seq) return CAM_UP_BAD_SEQ;
    if(g_up_written + len > g_up_total) return CAM_UP_SIZE_MISMATCH;

    if(g_up_file.write(data, len) != len) {
        camera_store_upload_abort();
        return CAM_UP_WRITE_FAIL;
    }

    // CRC covers the body only — the header carries the expected value, so it
    // cannot cover itself.
    if(g_up_written + len > FSD_CAM_HEADER_SIZE) {
        size_t skip = (g_up_written < FSD_CAM_HEADER_SIZE)
                          ? (size_t)(FSD_CAM_HEADER_SIZE - g_up_written)
                          : 0;
        g_up_crc = crc32_update(g_up_crc, data + skip, len - skip);
    }

    g_up_written += (uint32_t)len;
    g_up_next_seq++;
    return CAM_UP_OK;
}

uint8_t camera_store_upload_end(void) {
    if(!g_up_active) return CAM_UP_NOT_ACTIVE;
    g_up_file.close();
    g_up_active = false;

    uint8_t err = CAM_UP_OK;
    if(g_up_written != g_up_total) {
        err = CAM_UP_SIZE_MISMATCH;
    } else {
        // Re-read the header from flash rather than trusting what was sent —
        // this also proves the file is actually readable back.
        File f = LittleFS.open(CAM_TMP, "r");
        if(!f) {
            err = CAM_UP_WRITE_FAIL;
        } else {
            uint8_t h[FSD_CAM_HEADER_SIZE];
            bool read_ok = f.read(h, sizeof(h)) == (int)sizeof(h);
            f.close();
            if(!read_ok) {
                err = CAM_UP_WRITE_FAIL;
            } else {
                uint32_t magic = (uint32_t)h[0] | ((uint32_t)h[1] << 8) |
                                 ((uint32_t)h[2] << 16) | ((uint32_t)h[3] << 24);
                uint32_t want = (uint32_t)h[22] | ((uint32_t)h[23] << 8) |
                                ((uint32_t)h[24] << 16) | ((uint32_t)h[25] << 24);
                if(magic != FSD_CAM_MAGIC || h[4] != FSD_CAM_VERSION) {
                    err = CAM_UP_BAD_FORMAT;
                } else if(want != g_up_crc) {
                    Serial.printf("[CAM] CRC %08X != %08X\n",
                                  (unsigned)g_up_crc, (unsigned)want);
                    err = CAM_UP_BAD_CRC;
                }
            }
        }
    }

    // Everything past here reopens the database, so it runs under the lock.
    if(!g_db_lock) return CAM_UP_NO_FS;
    if(xSemaphoreTake(g_db_lock, pdMS_TO_TICKS(200)) != pdTRUE) return CAM_UP_BUSY;

    if(err != CAM_UP_OK) {
        LittleFS.remove(CAM_TMP);
        open_db(); // put the previous database back into service
        xSemaphoreGive(g_db_lock);
        Serial.printf("[CAM] upload rejected (%u)\n", err);
        return err;
    }

    // Swap. Removing first is required — rename() will not overwrite.
    LittleFS.remove(CAM_PATH);
    if(!LittleFS.rename(CAM_TMP, CAM_PATH)) {
        LittleFS.remove(CAM_TMP);
        xSemaphoreGive(g_db_lock);
        Serial.println("[CAM] rename failed — no database now");
        return CAM_UP_WRITE_FAIL;
    }
    bool opened = open_db();
    xSemaphoreGive(g_db_lock);
    if(!opened) return CAM_UP_BAD_FORMAT;
    Serial.printf("[CAM] upload OK — %u cameras\n", (unsigned)g_db.rec_count);
    return CAM_UP_OK;
}

// ── learned lane discrimination ──────────────────────────────────────────────

static size_t learn_write(void* ctx, const void* buf, size_t len) {
    File* f = (File*)ctx;
    return f->write((const uint8_t*)buf, len);
}

static size_t learn_read(void* ctx, void* buf, size_t len) {
    File* f = (File*)ctx;
    int n = f->read((uint8_t*)buf, len);
    return n > 0 ? (size_t)n : 0;
}

bool camera_store_load_learning(FsdTracker* t) {
    if(!t) return false;
    fsd_trk_init(t);
    if(!g_fs_ok || !LittleFS.exists(LEARN_PATH)) {
        Serial.println("[CAM] no learning yet — limits start wide");
        return false;
    }
    File f = LittleFS.open(LEARN_PATH, "r");
    if(!f) return false;
    bool ok = fsd_trk_load(t, learn_read, &f);
    f.close();
    if(!ok) {
        /* Unreadable is not the same as absent, and keeping it around means
         * failing the same way at every boot. Drop it and relearn. */
        Serial.println("[CAM] learning file rejected — discarding");
        LittleFS.remove(LEARN_PATH);
        return false;
    }
    unsigned n = 0;
    for(int i = 0; i < FSD_TRK_CAM_MAX; i++)
        if(t->mem[i].used) n++;
    Serial.printf("[CAM] learning restored for %u cameras\n", n);
    return true;
}

bool camera_store_save_learning(FsdTracker* t) {
    if(!t || !g_fs_ok) return false;

    /* Announced for the whole call so an upload arriving on the other task is
     * told to come back rather than interleaving with the rename below. */
    g_saving = true;

    /* Same atomic replacement the database gets. A power cut mid-write must
     * leave the previous learning intact, not a truncated file that fails its
     * CRC and throws away every pass ever recorded. */
    LittleFS.remove(LEARN_TMP);
    File f = LittleFS.open(LEARN_TMP, "w");
    if(!f) { g_saving = false; return false; }
    bool ok = fsd_trk_save(t, learn_write, &f);
    f.close();
    if(!ok) {
        LittleFS.remove(LEARN_TMP);
        Serial.println("[CAM] learning write failed");
        g_saving = false;
        return false;
    }

    /* The gap. Between these two calls there is no learn.bin, only a complete
     * learn.tmp — which is why camera_store_init() finishes the rename instead
     * of deleting the leftover. */
    LittleFS.remove(LEARN_PATH); // rename() will not overwrite
    if(!LittleFS.rename(LEARN_TMP, LEARN_PATH)) {
        /* Leave learn.tmp in place: it is the only complete copy left, and boot
         * recovery will pick it up. Removing it here would turn a failed rename
         * into permanent loss. */
        Serial.println("[CAM] learning rename failed — tmp kept for recovery");
        g_saving = false;
        return false;
    }
    t->dirty = false; // only now: the bytes are actually in place
    g_saving = false;
    return true;
}

void camera_store_upload_abort(void) {
    /* The load-bearing line. This is called from ServerCB::onDisconnect, so it
     * runs on EVERY ordinary disconnect — the phone leaving, the app going to
     * the background — not only when an upload is actually in flight. Without
     * this it fell through to open_db() every time, closing the database file
     * out from under a lookup running on the loop task and memsetting the
     * FsdCamDb that lookup was reading. With nothing to abort there is nothing
     * to do, and doing nothing touches no shared state at all. */
    if(!g_up_active && !LittleFS.exists(CAM_TMP)) return;

    if(g_up_file) g_up_file.close();
    if(g_up_active) Serial.println("[CAM] upload aborted");
    g_up_active = false;
    g_up_written = 0;
    LittleFS.remove(CAM_TMP);

    if(!g_db_lock) return;
    if(xSemaphoreTake(g_db_lock, pdMS_TO_TICKS(200)) != pdTRUE) return; // skip the reopen
    open_db();
    xSemaphoreGive(g_db_lock);
}

#endif  // BLE_SERVER_ENABLED
