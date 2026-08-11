/*
 * fsd_cam_track.c — see fsd_cam_track.h.
 */

#include "fsd_cam_track.h"

#include <math.h>
#include <string.h>

uint64_t fsd_trk_key(const FsdCamRecord* cam) {
    if(!cam) return 0;
    /* Cast through uint32_t first: a negative latitude sign-extends to all-ones
     * in the high half otherwise, and every southern-hemisphere camera would
     * collide. Korea is northern and eastern, but a key collision is a silent
     * wrong answer and costs nothing to prevent. */
    return ((uint64_t)(uint32_t)cam->lat_e7 << 32) | (uint64_t)(uint32_t)cam->lon_e7;
}

void fsd_trk_init(FsdTracker* t) {
    if(!t) return;
    memset(t, 0, sizeof(*t));
}

// ── learning store ───────────────────────────────────────────────────────────

static FsdTrkCamera* find_cam(FsdTracker* t, uint64_t key) {
    for(int i = 0; i < FSD_TRK_CAM_MAX; i++) {
        if(t->mem[i].used && t->mem[i].key == key) return &t->mem[i];
    }
    return NULL;
}

static const FsdTrkCamera* find_cam_const(const FsdTracker* t, uint64_t key) {
    for(int i = 0; i < FSD_TRK_CAM_MAX; i++) {
        if(t->mem[i].used && t->mem[i].key == key) return &t->mem[i];
    }
    return NULL;
}

/* Existing slot, else a free one, else the least recently used. Eviction is by
 * last_used rather than by pass count on purpose: a camera passed a hundred
 * times last year is worth less than one passed twice last week, because the
 * point of the learning is the route being driven now. */
static FsdTrkCamera* claim_cam(FsdTracker* t, uint64_t key) {
    FsdTrkCamera* c = find_cam(t, key);
    if(c) return c;

    FsdTrkCamera* victim = NULL;
    for(int i = 0; i < FSD_TRK_CAM_MAX; i++) {
        if(!t->mem[i].used) {
            victim = &t->mem[i];
            break;
        }
        if(!victim || t->mem[i].last_used < victim->last_used) victim = &t->mem[i];
    }
    if(!victim) return NULL; // only when FSD_TRK_CAM_MAX is 0

    memset(victim, 0, sizeof(*victim));
    victim->key = key;
    victim->used = true;
    return victim;
}

static const FsdTrkDirection* match_dir_const(const FsdTrkCamera* c, float bearing_deg) {
    if(!c) return NULL;
    const FsdTrkDirection* best = NULL;
    float best_d = FSD_TRK_DIRECTION_TOLERANCE_DEG;
    for(uint8_t i = 0; i < c->dir_count && i < FSD_TRK_DIRS; i++) {
        float d = fabsf(fsd_cam_angle_diff(bearing_deg, c->dir[i].bearing_deg));
        if(d <= best_d) {
            best = &c->dir[i];
            best_d = d;
        }
    }
    return best;
}

static FsdTrkDirection* match_dir(FsdTrkCamera* c, float bearing_deg) {
    /* Same search, non-const. Casting away const on the result would be the
     * short version but this stays readable and the array is at most 2. */
    FsdTrkDirection* best = NULL;
    float best_d = FSD_TRK_DIRECTION_TOLERANCE_DEG;
    for(uint8_t i = 0; i < c->dir_count && i < FSD_TRK_DIRS; i++) {
        float d = fabsf(fsd_cam_angle_diff(bearing_deg, c->dir[i].bearing_deg));
        if(d <= best_d) {
            best = &c->dir[i];
            best_d = d;
        }
    }
    return best;
}

/* Upper limit for "this is my lane", from the samples we have.
 *
 * The 80th percentile, not the maximum. GPS throws the occasional large
 * excursion, and letting one of those set the limit would undo the learning.
 * With few samples the percentile lands on the maximum anyway, so early
 * behaviour is conservative without a special case.
 *
 * Integer arithmetic for the index: (int)(count * 0.8f) is not reliable at
 * count == 5, where 0.8f rounds the product just under 4.0 and the index drops
 * a place. */
static float dir_limit(const FsdTrkDirection* d) {
    if(!d || d->count == 0) return FSD_CAM_DEFAULT_CPA_M;

    float s[FSD_TRK_SAMPLES];
    uint8_t n = d->count;
    if(n > FSD_TRK_SAMPLES) n = FSD_TRK_SAMPLES;
    for(uint8_t i = 0; i < n; i++) s[i] = d->samples[i];

    for(uint8_t i = 1; i < n; i++) { // insertion sort; n <= 8
        float v = s[i];
        int j = (int)i - 1;
        while(j >= 0 && s[j] > v) {
            s[j + 1] = s[j];
            j--;
        }
        s[j + 1] = v;
    }

    int idx = ((int)n * 8) / 10;
    if(idx > (int)n - 1) idx = (int)n - 1;
    return s[idx] + FSD_TRK_LEARNED_MARGIN_M;
}

float fsd_trk_cpa_limit(const FsdTracker* t, uint64_t key, float bearing_deg,
                        bool* learned_out) {
    if(learned_out) *learned_out = false;
    if(!t) return FSD_CAM_DEFAULT_CPA_M;

    const FsdTrkDirection* d = match_dir_const(find_cam_const(t, key), bearing_deg);
    if(!d || d->passes < FSD_TRK_MIN_PASSES) return FSD_CAM_DEFAULT_CPA_M;

    if(learned_out) *learned_out = true;
    return dir_limit(d);
}

uint16_t fsd_trk_passes(const FsdTracker* t, uint64_t key, float bearing_deg) {
    if(!t) return 0;
    const FsdTrkDirection* d = match_dir_const(find_cam_const(t, key), bearing_deg);
    return d ? d->passes : 0;
}

/* Fold to [0, 360). fmodf alone keeps the sign of the dividend. */
static float wrap360(float deg) {
    float v = fmodf(deg, 360.0f);
    if(v < 0.0f) v += 360.0f;
    return v;
}

static uint16_t learn(FsdTracker* t, uint64_t key, float bearing_deg, float cpa_m) {
    FsdTrkCamera* c = claim_cam(t, key);
    if(!c) return 0;
    c->last_used = t->tick;

    FsdTrkDirection* d = match_dir(c, bearing_deg);
    if(!d) {
        if(c->dir_count < FSD_TRK_DIRS) {
            d = &c->dir[c->dir_count++];
            memset(d, 0, sizeof(*d));
            d->bearing_deg = wrap360(bearing_deg);
        } else {
            /* Out of direction slots. Reuse the least-driven one rather than
             * discarding the observation: a third approach that keeps recurring
             * should eventually win, and one that does not will be pushed back
             * out by the two that do. */
            d = &c->dir[0];
            for(uint8_t i = 1; i < FSD_TRK_DIRS; i++)
                if(c->dir[i].passes < d->passes) d = &c->dir[i];
            memset(d, 0, sizeof(*d));
            d->bearing_deg = wrap360(bearing_deg);
        }
    }

    /* Pull the representative heading a fraction of the way toward this
     * observation — a running circular mean. Divided by passes+1 BEFORE the
     * count is incremented, so the first observation moves it by half and later
     * ones barely at all. */
    d->bearing_deg = wrap360(d->bearing_deg +
                             fsd_cam_angle_diff(bearing_deg, d->bearing_deg) /
                                 (float)(d->passes + 1));

    d->samples[d->next] = cpa_m;
    d->next = (uint8_t)((d->next + 1u) % FSD_TRK_SAMPLES);
    if(d->count < FSD_TRK_SAMPLES) d->count++;
    if(d->passes < 0xFFFFu) d->passes++;

    t->dirty = true;
    return d->passes;
}

// ── tracking ─────────────────────────────────────────────────────────────────

static FsdTrkActive* find_active(FsdTracker* t, uint64_t key) {
    for(int i = 0; i < FSD_TRK_ACTIVE_MAX; i++)
        if(t->active[i].used && t->active[i].key == key) return &t->active[i];
    return NULL;
}

static FsdTrkActive* free_active(FsdTracker* t) {
    for(int i = 0; i < FSD_TRK_ACTIVE_MAX; i++)
        if(!t->active[i].used) return &t->active[i];
    return NULL;
}

static float eta_seconds(float along_m, float speed_kph) {
    if(speed_kph < 1.0f) return -1.0f; // not moving enough to extrapolate
    return along_m / (speed_kph / 3.6f);
}

static void push(FsdTrkEvent* out, int max, int* n, const FsdTrkEvent* e) {
    if(*n < max) out[(*n)++] = *e;
}

int fsd_trk_update(FsdTracker* t, const FsdCamDb* db, const FsdCamFix* fix,
                   FsdTrkEvent* out, int max_events) {
    if(!t || !fix || !out || max_events <= 0) return 0;
    t->tick++;

    int n = 0;

    // 1. Measure everything already being followed, cone or no cone.
    for(int i = 0; i < FSD_TRK_ACTIVE_MAX; i++) {
        FsdTrkActive* st = &t->active[i];
        if(!st->used) continue;

        float d = fsd_cam_distance_m(fix->lat_e7, fix->lon_e7, st->cam.lat_e7,
                                     st->cam.lon_e7);
        if(st->has_prev) {
            /* At 1 Hz and 60 km/h consecutive fixes are 16 m apart, so the
             * nearest fix is never the nearest point — the error is wider than
             * a lane, which is the whole quantity being measured. */
            float seg = fsd_cam_segment_distance_m(st->prev_lat_e7, st->prev_lon_e7,
                                                   fix->lat_e7, fix->lon_e7,
                                                   st->cam.lat_e7, st->cam.lon_e7);
            if(seg < d) d = seg;
        }
        if(d < st->min_dist_m) {
            st->min_dist_m = d;
            st->bearing_at_min = fix->bearing_deg;
        }
        st->prev_lat_e7 = fix->lat_e7;
        st->prev_lon_e7 = fix->lon_e7;
        st->has_prev = true;
        st->last_dist_m = fsd_cam_distance_m(fix->lat_e7, fix->lon_e7,
                                             st->cam.lat_e7, st->cam.lon_e7);
    }

    // 2. Pick up anything new that is on our path.
    if(db && db->ok) {
        FsdCamRecord found[FSD_TRK_SCAN_MAX];
        int found_n = fsd_cam_near(db, fix->lat_e7, fix->lon_e7,
                                   FSD_TRK_SCAN_RADIUS_M, found, FSD_TRK_SCAN_MAX);
        for(int i = 0; i < found_n; i++) {
            uint64_t key = fsd_trk_key(&found[i]);
            if(find_active(t, key)) continue;

            FsdCamHit hit;
            if(!fsd_cam_evaluate(fix, &found[i], &hit)) continue; // behind or beside

            bool learned = false;
            float limit = fsd_trk_cpa_limit(t, key, fix->bearing_deg, &learned);
            if(hit.cpa_m > limit) continue; // learning says: not our lane

            FsdTrkActive* st = free_active(t);
            /* Full. Skipping is deliberate — displacing a camera mid-measurement
             * would corrupt its learning, and four simultaneous candidates
             * inside the cone is already an unusual junction. */
            if(!st) continue;

            memset(st, 0, sizeof(*st));
            st->cam = found[i];
            st->key = key;
            st->min_dist_m = hit.distance_m;
            st->bearing_at_min = fix->bearing_deg;
            st->last_dist_m = hit.distance_m;
            st->prev_lat_e7 = fix->lat_e7;
            st->prev_lon_e7 = fix->lon_e7;
            st->has_prev = true;
            st->used = true;

            FsdTrkCamera* c = find_cam(t, key);
            if(c) c->last_used = t->tick;

            FsdTrkEvent e;
            memset(&e, 0, sizeof(e));
            e.kind = FSD_TRK_APPROACH;
            e.cam = found[i];
            e.key = key;
            e.distance_m = hit.distance_m;
            e.cpa_m = hit.cpa_m;
            e.eta_s = eta_seconds(hit.along_m, fix->speed_kph);
            e.learned = learned;
            e.passes = fsd_trk_passes(t, key, fix->bearing_deg);
            push(out, max_events, &n, &e);
        }
    }

    // 3. Retire anything that has receded convincingly past its minimum.
    for(int i = 0; i < FSD_TRK_ACTIVE_MAX; i++) {
        FsdTrkActive* st = &t->active[i];
        if(!st->used) continue;
        if(st->last_dist_m <= st->min_dist_m + FSD_TRK_PASS_HYSTERESIS_M) continue;

        FsdTrkEvent e;
        memset(&e, 0, sizeof(e));
        e.cam = st->cam;
        e.key = st->key;
        e.distance_m = st->min_dist_m;
        e.cpa_m = st->min_dist_m;
        e.eta_s = -1.0f;

        if(st->min_dist_m <= FSD_TRK_MAX_PASS_CPA_M) {
            e.kind = FSD_TRK_PASS;
            e.passes = learn(t, st->key, st->bearing_at_min, st->min_dist_m);
            e.learned = true;
        } else {
            /* Near, but never close enough to have been the same road. Learning
             * from it would widen the limit on evidence of nothing. */
            e.kind = FSD_TRK_DROP;
            e.passes = fsd_trk_passes(t, st->key, st->bearing_at_min);
        }
        push(out, max_events, &n, &e);
        memset(st, 0, sizeof(*st));
    }

    return n;
}

// ── persistence ──────────────────────────────────────────────────────────────

/* zlib-compatible CRC-32, bitwise. A table would cost 1 KB of flash to save a
 * few microseconds on a file written once per drive. */
static uint32_t crc32_update(uint32_t crc, const uint8_t* d, size_t n) {
    crc = ~crc;
    while(n--) {
        crc ^= *d++;
        for(int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return ~crc;
}

static void put_u16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t get_u16(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

/* Metres to centimetres, saturating. Closest approaches never exceed
 * FSD_TRK_MAX_PASS_CPA_M, so the ceiling is unreachable in practice; it is here
 * so a NaN or a wild value truncates instead of wrapping to something small and
 * plausible-looking. */
static uint16_t m_to_cm(float m) {
    if(!(m > 0.0f)) return 0; // also catches NaN
    float cm = m * 100.0f + 0.5f;
    if(cm > 65535.0f) return 65535u;
    return (uint16_t)cm;
}

static uint16_t deg_to_cdeg(float deg) {
    float v = fmodf(deg, 360.0f);
    if(!(v == v)) v = 0.0f; // NaN
    if(v < 0.0f) v += 360.0f;
    uint32_t c = (uint32_t)(v * 100.0f + 0.5f);
    if(c > 35999u) c = 0u; // 360.00 rounds up into the next turn
    return (uint16_t)c;
}

/* One camera on the wire. Fixed size, so a record can be skipped or counted
 * without parsing it. */
static void pack_record(const FsdTrkCamera* c, uint8_t* rec) {
    memset(rec, 0, FSD_TRK_REC_SIZE);
    put_u32(rec + 0, (uint32_t)(c->key & 0xFFFFFFFFu));
    put_u32(rec + 4, (uint32_t)(c->key >> 32));

    uint8_t* p = rec + 8;
    for(uint8_t i = 0; i < FSD_TRK_DIRS; i++) {
        const FsdTrkDirection* d = &c->dir[i];
        bool live = i < c->dir_count;
        put_u16(p + 0, live ? deg_to_cdeg(d->bearing_deg) : 0u);
        put_u16(p + 2, live ? d->passes : 0u);
        p[4] = live ? d->count : 0u;
        p[5] = live ? d->next : 0u;
        for(uint8_t s = 0; s < FSD_TRK_SAMPLES; s++)
            put_u16(p + 6 + 2 * s, live ? m_to_cm(d->samples[s]) : 0u);
        p += 6 + 2 * FSD_TRK_SAMPLES;
    }
}

static void unpack_record(FsdTrkCamera* c, const uint8_t* rec) {
    memset(c, 0, sizeof(*c));
    c->key = (uint64_t)get_u32(rec + 0) | ((uint64_t)get_u32(rec + 4) << 32);
    c->used = true;

    const uint8_t* p = rec + 8;
    for(uint8_t i = 0; i < FSD_TRK_DIRS; i++) {
        FsdTrkDirection* d = &c->dir[i];
        d->bearing_deg = (float)get_u16(p + 0) / 100.0f;
        d->passes = get_u16(p + 2);
        d->count = p[4];
        d->next = p[5];
        /* Clamp what indexes an array. A corrupt count would read past the ring
         * in dir_limit(); the CRC should have caught it, but a bounds check is
         * the thing that actually prevents the read. */
        if(d->count > FSD_TRK_SAMPLES) d->count = FSD_TRK_SAMPLES;
        if(d->next >= FSD_TRK_SAMPLES) d->next = 0;
        for(uint8_t s = 0; s < FSD_TRK_SAMPLES; s++)
            d->samples[s] = (float)get_u16(p + 6 + 2 * s) / 100.0f;
        if(d->passes > 0) c->dir_count = (uint8_t)(i + 1);
        p += 6 + 2 * FSD_TRK_SAMPLES;
    }
}

bool fsd_trk_save(const FsdTracker* t, FsdTrkWriteFn write, void* ctx) {
    if(!t || !write) return false;

    uint16_t n = 0;
    for(int i = 0; i < FSD_TRK_CAM_MAX; i++)
        if(t->mem[i].used && t->mem[i].dir_count > 0) n++;

    uint8_t hdr[FSD_TRK_HEADER_SIZE];
    memset(hdr, 0, sizeof(hdr));
    put_u32(hdr + 0, FSD_TRK_MAGIC);
    hdr[4] = (uint8_t)FSD_TRK_FORMAT_VERSION;
    hdr[5] = 0; // flags
    put_u16(hdr + 6, n);
    /* Geometry goes in the header so a firmware built with different constants
     * rejects an old file instead of misreading it into the wrong fields. */
    hdr[8] = (uint8_t)FSD_TRK_DIRS;
    hdr[9] = (uint8_t)FSD_TRK_SAMPLES;
    if(write(ctx, hdr, sizeof(hdr)) != sizeof(hdr)) return false;

    /* CRC covers the records only, and rides at the END of the file — it cannot
     * be in the header, because writing it there would mean buffering
     * everything first, which is exactly what streaming avoids. */
    uint32_t crc = 0;
    for(int i = 0; i < FSD_TRK_CAM_MAX; i++) {
        if(!t->mem[i].used || t->mem[i].dir_count == 0) continue;
        uint8_t rec[FSD_TRK_REC_SIZE];
        pack_record(&t->mem[i], rec);
        crc = crc32_update(crc, rec, sizeof(rec));
        if(write(ctx, rec, sizeof(rec)) != sizeof(rec)) return false;
    }

    uint8_t tail[4];
    put_u32(tail, crc);
    return write(ctx, tail, sizeof(tail)) == sizeof(tail);
}

bool fsd_trk_load(FsdTracker* t, FsdTrkReadFn read, void* ctx) {
    if(!t || !read) return false;
    fsd_trk_init(t);

    uint8_t hdr[FSD_TRK_HEADER_SIZE];
    if(read(ctx, hdr, sizeof(hdr)) != sizeof(hdr)) return false;
    if(get_u32(hdr + 0) != FSD_TRK_MAGIC) return false;
    if(hdr[4] != FSD_TRK_FORMAT_VERSION) return false;
    if(hdr[8] != (uint8_t)FSD_TRK_DIRS || hdr[9] != (uint8_t)FSD_TRK_SAMPLES)
        return false;

    uint16_t n = get_u16(hdr + 6);
    if(n > FSD_TRK_CAM_MAX) return false;

    uint32_t crc = 0;
    for(uint16_t i = 0; i < n; i++) {
        uint8_t rec[FSD_TRK_REC_SIZE];
        if(read(ctx, rec, sizeof(rec)) != sizeof(rec)) {
            fsd_trk_init(t);
            return false;
        }
        crc = crc32_update(crc, rec, sizeof(rec));
        unpack_record(&t->mem[i], rec);
    }

    uint8_t tail[4];
    if(read(ctx, tail, sizeof(tail)) != sizeof(tail) || get_u32(tail) != crc) {
        /* Half a learning store is worse than none: a limit that is too narrow
         * stops warning about a camera that really is ours, and nothing
         * downstream can tell that from a legitimately learned one. */
        fsd_trk_init(t);
        return false;
    }
    return true;
}

bool fsd_trk_nearest(const FsdTracker* t, FsdCamRecord* cam_out, uint64_t* key_out,
                     float* distance_out) {
    if(!t) return false;
    const FsdTrkActive* best = NULL;
    for(int i = 0; i < FSD_TRK_ACTIVE_MAX; i++) {
        const FsdTrkActive* st = &t->active[i];
        if(!st->used) continue;
        if(!best || st->last_dist_m < best->last_dist_m) best = st;
    }
    if(!best) return false;
    if(cam_out) *cam_out = best->cam;
    if(key_out) *key_out = best->key;
    if(distance_out) *distance_out = best->last_dist_m;
    return true;
}
