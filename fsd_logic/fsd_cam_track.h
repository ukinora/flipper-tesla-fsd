#pragma once
/*
 * fsd_cam_track.h — follow cameras past, and learn which ones are ours.
 *
 * fsd_camera.h answers "how close would I pass this camera if I keep going
 * straight". That number separates lanes arithmetically — same lane 0 m,
 * next lane 4 m, opposite carriageway 11 m — but it does not say where to cut.
 * GPS error is 3-10 m, comparable to the gap being measured, so on a camera we
 * have never driven past the opposite carriageway cannot be ruled out and the
 * limit has to stay wide (FSD_CAM_DEFAULT_CPA_M).
 *
 * Driving past is what settles it. Every pass records
 *
 *      (heading at the closest point, how close we actually came)
 *
 * and after a couple of passes "coming this way I always graze it at 3 m" is a
 * fact, which makes an 11 m candidate demonstrably not our lane. Repeated
 * routes get sharper; new roads stay conservative.
 *
 * WHY TRACKING IS SEPARATE FROM SCANNING
 * -------------------------------------
 * The bearing to a camera grows as you close on it, so the moment you actually
 * graze it is the moment it leaves the forward cone. A tracker driven only by
 * scan results would lose exactly the samples it needs and would overestimate
 * every minimum. Once a camera is picked up it is followed until it is behind
 * us, independent of the cone.
 *
 * NO ALLOCATION
 * -------------
 * Fixed arrays, sized at compile time. The learning store is the only sizeable
 * thing here (~12 KB at the defaults) and it lives in RAM because it is written
 * on every pass; persisting it is the caller's business, and `dirty` says when
 * that is worth doing.
 *
 * DIVERGENCE FROM THE PROTOTYPE (camera-db/track.py)
 * --------------------------------------------------
 * Python keeps every sample ever taken. Here each direction keeps the last
 * FSD_TRK_SAMPLES in a ring. That bounds memory, and it also lets the learned
 * limit follow a road that changed — a resurfaced junction that moved the lane
 * two metres would otherwise be averaged against years of stale samples.
 * The percentile is identical while the count is within the ring.
 */

#include "fsd_camera.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Cameras being followed at once. Four is generous: they have to be inside the
 * forward cone AND within the CPA limit at the same time. A fifth is ignored
 * rather than displacing one we are already measuring. */
#ifndef FSD_TRK_ACTIVE_MAX
#define FSD_TRK_ACTIVE_MAX 4
#endif

/* Cameras we retain learning for. A commute passes a few dozen; this holds a
 * few hundred kilometres of habit. When it fills, the least recently used slot
 * goes — losing the learning for a road driven once a year is the right thing
 * to lose. */
#ifndef FSD_TRK_CAM_MAX
#define FSD_TRK_CAM_MAX 128
#endif

/* Directions per camera. Two carriageways is the case this exists for; a third
 * approach (a junction) evicts the least-used of the two. */
#ifndef FSD_TRK_DIRS
#define FSD_TRK_DIRS 2
#endif

/* Passes retained per direction. Beyond about five the limit stops moving, so
 * eight is already headroom. */
#ifndef FSD_TRK_SAMPLES
#define FSD_TRK_SAMPLES 8
#endif

/* Records pulled from the database per scan.
 *
 * Measured against the real national database: within FSD_TRK_SCAN_RADIUS_M the
 * densest spot in the country holds 17 cameras (37.6075, 127.1343), and four
 * locations exceed 16. The previous value of 16 therefore truncated in actual
 * Korean traffic — and fsd_cam_near() fills in GRID ORDER, south-west cell
 * first, not by distance, so what gets dropped is whatever lies north-east.
 * Driving north-east through one of those junctions would silently lose the
 * camera being approached.
 *
 * 24 clears the measured maximum with room to spare, at 288 bytes of stack —
 * FsdCamRecord is 12 bytes once aligned (int32, int32, uint8, uint8), not the
 * 10-byte FSD_CAM_REC_SIZE that goes on the wire. */
#ifndef FSD_TRK_SCAN_MAX
#define FSD_TRK_SCAN_MAX 24
#endif

/* How far ahead to look. Must exceed the policy's largest lead distance
 * (FSD_POL_MAX_LEAD_M, 400 m) or a camera would be triggered on before it was
 * ever found. */
#define FSD_TRK_SCAN_RADIUS_M 600.0f

/* Two headings count as the same approach within this. Wide enough for road
 * curvature and GPS bearing noise on a single carriageway, narrow enough that
 * the opposite direction (180 deg away) never merges in. */
#define FSD_TRK_DIRECTION_TOLERANCE_DEG 35.0f

/* Added to the learned percentile. GPS error differs pass to pass, so the
 * limit sits above what we have actually seen, never on it. */
#define FSD_TRK_LEARNED_MARGIN_M 6.0f

/* Passes before a learned limit is trusted. One could be a fluke. */
#define FSD_TRK_MIN_PASSES 2

/* Farther than this at the closest point and it was not a pass — we drove near
 * it, not past it, and learning from it would widen the limit for nothing. */
#define FSD_TRK_MAX_PASS_CPA_M 60.0f

/* Receding by this much confirms a pass. Without hysteresis a single GPS
 * excursion would declare one. */
#define FSD_TRK_PASS_HYSTERESIS_M 25.0f

typedef enum {
    FSD_TRK_APPROACH = 0, // entered our path
    FSD_TRK_PASS,         // went by close enough to learn from
    FSD_TRK_DROP,         // left our path without a real pass
} FsdTrkEventKind;

typedef struct {
    FsdTrkEventKind kind;
    FsdCamRecord cam;
    uint64_t key;      // stable identity, see fsd_trk_key()
    float distance_m;  // straight-line at the moment of the event
    float cpa_m;       // closest approach: predicted on APPROACH, measured after
    float eta_s;       // seconds to the camera, or -1 when speed is too low
    bool learned;      // judged with a trusted profile rather than the default
    uint16_t passes;   // how many times this direction has been driven
} FsdTrkEvent;

typedef struct {
    float bearing_deg;                 // representative heading for this approach
    float samples[FSD_TRK_SAMPLES];    // ring of measured closest approaches
    uint8_t count;                     // valid entries (saturates)
    uint8_t next;                      // ring write cursor
    uint16_t passes;                   // total ever, not just what is retained
} FsdTrkDirection;

typedef struct {
    uint64_t key;
    FsdTrkDirection dir[FSD_TRK_DIRS];
    uint8_t dir_count;
    bool used;
    uint32_t last_used; // tracker tick counter, for eviction
} FsdTrkCamera;

typedef struct {
    FsdCamRecord cam;
    uint64_t key;
    float min_dist_m;      // closest approach seen so far, segment-interpolated
    float bearing_at_min;
    float last_dist_m;     // most recent point-to-point distance
    int32_t prev_lat_e7;
    int32_t prev_lon_e7;
    bool has_prev;
    bool used;
} FsdTrkActive;

typedef struct {
    FsdTrkActive active[FSD_TRK_ACTIVE_MAX];
    FsdTrkCamera mem[FSD_TRK_CAM_MAX];
    uint32_t tick;   // monotonic, drives eviction
    bool dirty;      // learning changed since the last persist
    /* Scans that came back exactly full, i.e. possibly truncated. A full buffer
     * is indistinguishable from "there were more", and a dropped record is a
     * camera we never even judged. Counting it makes the condition reportable
     * instead of silent — the same reason the upload path refuses a sequence
     * gap rather than papering over it. */
    uint16_t scan_full_count;
} FsdTracker;

/** Stable identity for a camera. The database has no id column, but the
 *  coordinates are the identity — they are what the record is. */
uint64_t fsd_trk_key(const FsdCamRecord* cam);

/** Clear everything, learning included. */
void fsd_trk_init(FsdTracker* t);

/** The CPA limit to judge this camera by, coming in on this heading, and
 *  whether it came from learning. Falls back to FSD_CAM_DEFAULT_CPA_M. */
float fsd_trk_cpa_limit(const FsdTracker* t, uint64_t key, float bearing_deg,
                        bool* learned_out);

/** Feed one fix. Returns how many events were written to `out` (never more
 *  than `max_events`; excess events are dropped, oldest first is not a concern
 *  because a single fix cannot produce many).
 *
 *  `db` may be NULL, in which case nothing new is picked up but cameras already
 *  being followed are still measured — which is what should happen while a
 *  database upload is replacing the file underneath us. */
int fsd_trk_update(FsdTracker* t, const FsdCamDb* db, const FsdCamFix* fix,
                   FsdTrkEvent* out, int max_events);

/** Forget every camera currently being measured. Learning is untouched.
 *
 *  Call this after a gap in fixes — a tunnel, a reboot, a spell where the fix
 *  was refused. FsdTrkActive.prev_lat/prev_lon otherwise survive the gap, and
 *  the segment interpolation in fsd_trk_update() then draws one chord across
 *  the WHOLE of it. A chord kilometres long that happens to graze a camera the
 *  car never went near collapses min_dist_m to a few metres, and retirement
 *  branches on min_dist_m rather than on how far the car appeared to jump — so
 *  the gap is recorded as a PASS and NARROWS that camera's learned limit.
 *
 *  That is the worst direction to be wrong in: a limit learned from a fiction
 *  is narrower than the truth, and a too-narrow limit silently stops warning
 *  about a camera that really is ours. */
void fsd_trk_reset_active(FsdTracker* t);

/** The nearest camera currently being followed, or false when there is none.
 *  This is what the policy layer reads: it wants a level, not an edge. */
bool fsd_trk_nearest(const FsdTracker* t, FsdCamRecord* cam_out, uint64_t* key_out,
                     float* distance_out);

/** Passes recorded for a camera in this direction (0 when unknown). */
uint16_t fsd_trk_passes(const FsdTracker* t, uint64_t key, float bearing_deg);

// ── persistence ──────────────────────────────────────────────────────────────
// Learning that does not survive a power cycle is decorative: the car sleeps
// between every drive, so every drive would start back at the wide default and
// warn on the opposite carriageway again. The value of this layer is entirely
// in the repetition, which means it has to be written down.
//
// Streamed through callbacks rather than a buffer, for the same reason the
// database reader is: a few kilobytes of scratch is a lot on this part, and the
// firmware wants to go straight to a file. Values are stored as fixed-point
// integers so no float layout assumption crosses the wire — closest approaches
// in centimetres, headings in hundredths of a degree.

#define FSD_TRK_MAGIC 0x4E524C54u /* "TLRN" little-endian */
#define FSD_TRK_FORMAT_VERSION 1u
#define FSD_TRK_HEADER_SIZE 16u
#define FSD_TRK_REC_SIZE (8u + (2u + 2u + 1u + 1u + 2u * FSD_TRK_SAMPLES) * FSD_TRK_DIRS)

/** Returns bytes actually written; anything short is a failure. */
typedef size_t (*FsdTrkWriteFn)(void* ctx, const void* buf, size_t len);
/** Returns bytes actually read; anything short ends the load. */
typedef size_t (*FsdTrkReadFn)(void* ctx, void* buf, size_t len);

/** Write the learning out. Only cameras that have actually been passed are
 *  stored, so a fresh module writes a 20-byte file rather than 7 KB of zeros.
 *  Does not clear `dirty` — the caller owns that, because only the caller knows
 *  whether the bytes reached the flash. */
bool fsd_trk_save(const FsdTracker* t, FsdTrkWriteFn write, void* ctx);

/** Read it back, replacing whatever is in `t`. On ANY problem — bad magic,
 *  wrong version, geometry from a different build, truncation, CRC mismatch —
 *  the tracker is left empty rather than half-filled. Starting over is cheap;
 *  trusting a corrupt limit is not, because a too-narrow one silently stops
 *  warning about a camera that is genuinely ours. */
bool fsd_trk_load(FsdTracker* t, FsdTrkReadFn read, void* ctx);

#ifdef __cplusplus
}
#endif
