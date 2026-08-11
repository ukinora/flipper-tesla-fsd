/*
 * test_camera.c — host tests for the on-module camera database and approach
 * judgement (fsd_logic/fsd_camera.c).
 *
 * The fixture below is a real camera.bin produced by camera-db/pack.py, pasted
 * in as bytes. That makes this a **format compatibility test**: if the Python
 * writer and the C reader ever drift apart, this fails. A hand-built C fixture
 * would not catch that.
 *
 * Fixture contents (4 cameras, 2 cells):
 *   35.1000000, 129.0000000  110 km/h          <- Busan, own cell
 *   37.5000000, 127.0000000   30 km/h          <- Seoul, shared cell
 *   37.5001000, 127.0002000   30 km/h
 *   37.5100000, 127.0100000   60 km/h  SECTION
 *
 * Expected geometry values come from the Python prototype (camera-db/), so the
 * two implementations are checked against each other rather than against
 * themselves.
 *
 * Build + run:  make -C test check
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fsd_cam_policy.h"
#include "fsd_cam_track.h"
#include "fsd_camera.h"
#include "fsd_gps.h"
#include "fsd_speed_profile.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (cond) {                                                             \
            g_pass++;                                                           \
        } else {                                                                \
            g_fail++;                                                           \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                       \
            printf(__VA_ARGS__);                                                \
            printf("\n");                                                       \
        }                                                                       \
    } while (0)

#define NEAR(a, b, tol) (fabsf((float)(a) - (float)(b)) <= (float)(tol))

// camera.bin, 92 bytes, straight out of pack.py
static const unsigned char FIXTURE[] = {
    0x54, 0x43, 0x41, 0x4D, 0x01, 0x00, 0x20, 0x4E, 0x00, 0x00, 0x02, 0x00,
    0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0xF4, 0xD9,
    0xBE, 0xE6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x19, 0xDB, 0x06,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xCE, 0x18, 0x53, 0x07, 0x01, 0x00,
    0x00, 0x00, 0x03, 0x00, 0xC0, 0xD5, 0xEB, 0x14, 0x80, 0xD6, 0xE3, 0x4C,
    0x6E, 0x00, 0xC0, 0x0B, 0x5A, 0x16, 0x80, 0xA9, 0xB2, 0x4B, 0x1E, 0x00,
    0xA8, 0x0F, 0x5A, 0x16, 0x50, 0xB1, 0xB2, 0x4B, 0x1E, 0x00, 0x60, 0x92,
    0x5B, 0x16, 0x20, 0x30, 0xB4, 0x4B, 0x3C, 0x01,
};

typedef struct {
    const unsigned char* data;
    size_t len;
    int reads; // how many flash reads a lookup costs
} MemSrc;

static size_t mem_read(void* ctx, uint32_t offset, void* buf, size_t len) {
    MemSrc* s = (MemSrc*)ctx;
    s->reads++;
    if (offset >= s->len) return 0;
    size_t n = s->len - offset;
    if (n > len) n = len;
    memcpy(buf, s->data + offset, n);
    return n;
}

static MemSrc make_src(void) {
    MemSrc s = {FIXTURE, sizeof(FIXTURE), 0};
    return s;
}

// ── header ──────────────────────────────────────────────────────────────────
static void test_open(void) {
    MemSrc s = make_src();
    FsdCamDb db;
    CHECK(fsd_cam_open(&db, mem_read, &s), "open must succeed");
    CHECK(db.ok, "db marked ok");
    CHECK(db.grid_e6 == 20000, "grid=%u exp 20000", db.grid_e6);
    CHECK(db.cell_count == 2, "cells=%u exp 2", db.cell_count);
    CHECK(db.rec_count == 4, "records=%u exp 4", db.rec_count);
    CHECK(db.rec_offset == 32 + 2 * FSD_CAM_CELL_SIZE,
          "rec_offset=%u exp %u", db.rec_offset, 32 + 2 * FSD_CAM_CELL_SIZE);

    // built_at went into what used to be reserved bytes, with NO version bump.
    // This fixture predates the field, so reading it must give 0 — "unknown",
    // which is exactly true — rather than failing to open or reporting noise.
    CHECK(db.built_at == 0, "pre-timestamp file reads as unknown, got %u",
          db.built_at);

    // And a file that does carry one is read back intact. Written at offset 26,
    // so every byte an older reader looks at is untouched.
    unsigned char stamped[sizeof(FIXTURE)];
    memcpy(stamped, FIXTURE, sizeof(stamped));
    const uint32_t WHEN = 1754870400u;
    for (int i = 0; i < 4; i++)
        stamped[26 + i] = (unsigned char)((WHEN >> (8 * i)) & 0xFFu);
    MemSrc s2 = {stamped, sizeof(stamped), 0};
    FsdCamDb db2;
    CHECK(fsd_cam_open(&db2, mem_read, &s2), "stamped file still opens");
    CHECK(db2.built_at == WHEN, "built_at=%u exp %u", db2.built_at, WHEN);
    CHECK(db2.crc == db.crc && db2.rec_count == db.rec_count,
          "the fields an older reader uses are unchanged");
}

static void test_open_rejects_garbage(void) {
    unsigned char bad[sizeof(FIXTURE)];
    memcpy(bad, FIXTURE, sizeof(bad));
    bad[0] = 'X'; // break the magic
    MemSrc s = {bad, sizeof(bad), 0};
    FsdCamDb db;
    CHECK(!fsd_cam_open(&db, mem_read, &s), "bad magic must be refused");

    memcpy(bad, FIXTURE, sizeof(bad));
    bad[4] = 99; // unknown version
    MemSrc s2 = {bad, sizeof(bad), 0};
    CHECK(!fsd_cam_open(&db, mem_read, &s2), "unknown version must be refused");

    // A truncated file must not be read past its end.
    MemSrc s3 = {FIXTURE, 8, 0};
    CHECK(!fsd_cam_open(&db, mem_read, &s3), "truncated header must be refused");

    CHECK(!fsd_cam_open(&db, NULL, &s), "NULL reader must be refused");
}

// ── lookup ──────────────────────────────────────────────────────────────────
static void test_near(void) {
    MemSrc s = make_src();
    FsdCamDb db;
    fsd_cam_open(&db, mem_read, &s);
    FsdCamRecord out[8];

    // Standing on the Seoul pair: both are within 50 m, the 60 km/h one is not.
    s.reads = 0;
    int n = fsd_cam_near(&db, 375000000, 1270000000, 50.0f, out, 8);
    CHECK(n == 2, "50 m around Seoul -> %d exp 2", n);
    CHECK(s.reads < 40, "lookup should be cheap, took %d reads", s.reads);

    // Widen and the 60 km/h camera (~1.3 km away) joins.
    n = fsd_cam_near(&db, 375000000, 1270000000, 2000.0f, out, 8);
    CHECK(n == 3, "2 km around Seoul -> %d exp 3", n);
    bool saw_section = false;
    for (int i = 0; i < n; i++) {
        if (out[i].flags & FSD_CAM_FLAG_SECTION) {
            saw_section = true;
            CHECK(out[i].limit_kph == 60, "section camera limit=%u exp 60",
                  out[i].limit_kph);
        }
    }
    CHECK(saw_section, "the SECTION flag must survive the round trip");

    // Busan sits in a different cell and must not leak into a Seoul lookup.
    for (int i = 0; i < n; i++) {
        CHECK(out[i].lat_e7 > 370000000,
              "Busan record leaked into a Seoul lookup: lat=%d", out[i].lat_e7);
    }

    // ...but is found from Busan.
    n = fsd_cam_near(&db, 351000000, 1290000000, 100.0f, out, 8);
    CHECK(n == 1, "Busan lookup -> %d exp 1", n);
    CHECK(n == 1 && out[0].limit_kph == 110, "Busan limit=%u exp 110",
          n == 1 ? out[0].limit_kph : 0);

    // Empty ocean.
    n = fsd_cam_near(&db, 360000000, 1280000000, 500.0f, out, 8);
    CHECK(n == 0, "middle of nowhere -> %d exp 0", n);
}

static void test_near_respects_max(void) {
    MemSrc s = make_src();
    FsdCamDb db;
    fsd_cam_open(&db, mem_read, &s);
    FsdCamRecord out[8];
    int n = fsd_cam_near(&db, 375000000, 1270000000, 2000.0f, out, 1);
    CHECK(n == 1, "max=1 must cap the result, got %d", n);
    CHECK(fsd_cam_near(&db, 375000000, 1270000000, 50.0f, out, 0) == -1,
          "max=0 is a programming error, not an empty result");
    CHECK(fsd_cam_near(&db, 375000000, 1270000000, 50.0f, NULL, 8) == -1,
          "NULL out must be refused");
}

// ── geometry (expected values from the Python prototype) ────────────────────
static void test_geometry(void) {
    const int32_t A_LAT = 375000000, A_LON = 1270000000;
    const int32_t B_LAT = 375001000, B_LON = 1270002000;

    float d = fsd_cam_distance_m(A_LAT, A_LON, B_LAT, B_LON);
    CHECK(NEAR(d, 20.8785f, 0.01f), "distance=%.4f exp 20.8785", d);

    float b = fsd_cam_bearing_deg(A_LAT, A_LON, B_LAT, B_LON);
    CHECK(NEAR(b, 57.7794f, 0.01f), "bearing=%.4f exp 57.7794", b);

    // Reverse bearing is 180 degrees away.
    float rb = fsd_cam_bearing_deg(B_LAT, B_LON, A_LAT, A_LON);
    CHECK(NEAR(fabsf(fsd_cam_angle_diff(b, rb)), 180.0f, 0.05f),
          "reverse bearing diff = %.2f exp 180", fsd_cam_angle_diff(b, rb));

    CHECK(NEAR(fsd_cam_distance_m(A_LAT, A_LON, A_LAT, A_LON), 0.0f, 1e-3f),
          "distance to self must be 0");

    // Wrap-around must fold to the short way, not 350 degrees.
    CHECK(NEAR(fsd_cam_angle_diff(10.0f, 350.0f), 20.0f, 0.01f),
          "angle_diff(10,350)=%.2f exp 20", fsd_cam_angle_diff(10.0f, 350.0f));
    CHECK(NEAR(fsd_cam_angle_diff(350.0f, 10.0f), -20.0f, 0.01f),
          "angle_diff(350,10)=%.2f exp -20", fsd_cam_angle_diff(350.0f, 10.0f));
}

// The reason segment interpolation exists: at 1 Hz the closest sample is not
// the closest approach, and the gap is wider than a lane.
static void test_segment_distance(void) {
    // Drive due north past a camera sitting 5 m to the east, sampling 16 m
    // apart (60 km/h at 1 Hz) so that no sample lands beside it.
    const int32_t CAM_LAT = 375000000;
    const int32_t CAM_LON = 1270000000 + (int32_t)(5.0 / 88000.0 * 1e7);
    const int32_t STEP = (int32_t)(16.0 / 111320.0 * 1e7);

    // Straddle the camera: one sample 8 m before it, one 8 m after, neither
    // beside it. The segment between them passes the camera's latitude, so the
    // true closest approach is the 5 m lateral offset while the best sample is
    // sqrt(8^2 + 5^2) = 9.4 m.
    int32_t a_lat = CAM_LAT - STEP / 2;
    int32_t b_lat = CAM_LAT + STEP / 2;
    float best_sample = fsd_cam_distance_m(a_lat, 1270000000, CAM_LAT, CAM_LON);
    float s2 = fsd_cam_distance_m(b_lat, 1270000000, CAM_LAT, CAM_LON);
    if (s2 < best_sample) best_sample = s2;

    float seg = fsd_cam_segment_distance_m(a_lat, 1270000000, b_lat, 1270000000,
                                           CAM_LAT, CAM_LON);
    CHECK(seg < best_sample,
          "segment (%.1f m) must beat the best sample (%.1f m)", seg, best_sample);
    CHECK(NEAR(seg, 5.0f, 1.5f), "segment=%.2f exp ~5 (the lateral offset)", seg);

    // Degenerate segment falls back to point distance.
    float deg = fsd_cam_segment_distance_m(a_lat, 1270000000, a_lat, 1270000000,
                                           CAM_LAT, CAM_LON);
    CHECK(NEAR(deg, fsd_cam_distance_m(a_lat, 1270000000, CAM_LAT, CAM_LON), 0.01f),
          "zero-length segment must equal point distance");
}

// ── approach judgement ──────────────────────────────────────────────────────
static void test_evaluate(void) {
    FsdCamRecord cam = {375000000, 1270000000, 30, 0};
    FsdCamHit hit;
    // 200 m south of the camera, heading north: dead ahead.
    const int32_t BACK = (int32_t)(200.0 / 111320.0 * 1e7);
    FsdCamFix fix = {375000000 - BACK, 1270000000, 0.0f, 60.0f, 0.0f};

    CHECK(fsd_cam_evaluate(&fix, &cam, &hit), "camera ahead must be accepted");
    CHECK(NEAR(hit.distance_m, 200.0f, 1.0f), "distance=%.1f exp 200",
          hit.distance_m);
    CHECK(NEAR(hit.along_m, 200.0f, 1.0f), "along=%.1f exp 200", hit.along_m);
    CHECK(NEAR(hit.cpa_m, 0.0f, 1.0f), "cpa=%.1f exp 0 (same lane)", hit.cpa_m);

    // Same spot, driving away: must be rejected outright.
    fix.bearing_deg = 180.0f;
    CHECK(!fsd_cam_evaluate(&fix, &cam, &hit), "camera behind must be rejected");

    // Offset 11 m to the side (the far carriageway) while still heading at it.
    fix.bearing_deg = 0.0f;
    fix.lon_e7 = 1270000000 - (int32_t)(11.0 / 88000.0 * 1e7);
    CHECK(fsd_cam_evaluate(&fix, &cam, &hit), "still ahead");
    CHECK(NEAR(hit.cpa_m, 11.0f, 1.5f), "cpa=%.1f exp ~11 (opposite lane)",
          hit.cpa_m);
    CHECK(hit.cpa_m <= FSD_CAM_DEFAULT_CPA_M,
          "before learning, the opposite lane still warns — over-warning is the "
          "safe direction");

    // Far off to the side: outside the forward cone.
    fix.lon_e7 = 1270000000 - (int32_t)(400.0 / 88000.0 * 1e7);
    CHECK(!fsd_cam_evaluate(&fix, &cam, &hit),
          "a camera 400 m to the side is not on our path");

    CHECK(!fsd_cam_evaluate(NULL, &cam, &hit), "NULL fix refused");
    CHECK(!fsd_cam_evaluate(&fix, NULL, &hit), "NULL cam refused");
    CHECK(!fsd_cam_evaluate(&fix, &cam, NULL), "NULL out refused");
}

// ─────────────────────────────────────────────────────────────────────────────
// Tracking + learning (fsd_cam_track.c)
// ─────────────────────────────────────────────────────────────────────────────

static FsdCamFix at(double lat, double lon, float bearing, float kph) {
    FsdCamFix f;
    memset(&f, 0, sizeof(f));
    f.lat_e7 = (int32_t)(lat * 1e7);
    f.lon_e7 = (int32_t)(lon * 1e7);
    f.bearing_deg = bearing;
    f.speed_kph = kph;
    return f;
}

// Drive north along the 127.0 meridian past the Seoul camera at 37.5, holding a
// constant lateral offset. Returns the PASS event's measured closest approach,
// or -1 if no pass was reported.
static float drive_past(FsdTracker* t, const FsdCamDb* db, double offset_m,
                        int* passes_out) {
    float measured = -1.0f;
    if (passes_out) *passes_out = 0;
    // 20 m steps — 1 Hz at 72 km/h. Deliberately offset by 10 m so no sample
    // lands on the camera's latitude: the nearest fixes are then 10 m either
    // side, point-to-point says 10.4 m, and only segment interpolation recovers
    // the real 3 m. A grid that happened to line up would pass this test
    // without the interpolation working at all.
    for (int m = -410; m <= 210; m += 20) {
        double lat = 37.5 + (double)m / 111320.0;
        double lon = 127.0 + offset_m / 88000.0;
        FsdCamFix f = at(lat, lon, 0.0f, 72.0f);
        FsdTrkEvent ev[4];
        int n = fsd_trk_update(t, db, &f, ev, 4);
        for (int i = 0; i < n; i++) {
            if (ev[i].kind == FSD_TRK_PASS && ev[i].cam.lat_e7 == 375000000 &&
                ev[i].cam.lon_e7 == 1270000000) {
                measured = ev[i].cpa_m;
                if (passes_out) *passes_out = (int)ev[i].passes;
            }
        }
    }
    return measured;
}

static void test_track_pass_and_learn(void) {
    printf("\n-- tracking: pass detection and learning --\n");

    FsdCamDb db;
    MemSrc src = make_src();
    CHECK(fsd_cam_open(&db, mem_read, &src), "fixture opens");

    FsdTracker t;
    fsd_trk_init(&t);

    FsdCamRecord seoul = {375000000, 1270000000, 30, 0};
    uint64_t key = fsd_trk_key(&seoul);

    bool learned = true;
    CHECK(NEAR(fsd_trk_cpa_limit(&t, key, 0.0f, &learned), FSD_CAM_DEFAULT_CPA_M,
               0.01f),
          "an unlearned camera uses the wide default");
    CHECK(!learned, "and reports that it is not learned");

    // Same lane: 3 m to the side.
    int passes = 0;
    float measured = drive_past(&t, &db, 3.0, &passes);
    CHECK(measured >= 0.0f, "a pass was reported");
    // Fixes are 20 m apart and the camera sits between two of them, so only
    // segment interpolation recovers a 3 m approach. Nearest-sample would say
    // about 10 m — wider than a lane, which is the quantity being measured.
    CHECK(NEAR(measured, 3.0f, 2.0f), "measured %.1f m, expected ~3",
          (double)measured);
    CHECK(passes == 1, "one pass recorded");
    CHECK(t.dirty, "learning marked dirty for the caller to persist");

    CHECK(NEAR(fsd_trk_cpa_limit(&t, key, 0.0f, &learned), FSD_CAM_DEFAULT_CPA_M,
               0.01f),
          "one pass could be a fluke, so the default still applies");
    CHECK(!learned, "not trusted after one pass");

    measured = drive_past(&t, &db, 3.0, &passes);
    CHECK(passes == 2, "two passes recorded");
    float lim = fsd_trk_cpa_limit(&t, key, 0.0f, &learned);
    CHECK(learned, "trusted after two passes");
    CHECK(lim < FSD_CAM_DEFAULT_CPA_M, "learned limit %.1f m narrows the default",
          (double)lim);
    // The point of the whole layer: 11 m is the opposite carriageway, and after
    // learning it must no longer count as ours.
    CHECK(lim < 11.0f, "learned limit %.1f m excludes the opposite carriageway",
          (double)lim);
}

static void test_track_direction_is_separate(void) {
    printf("\n-- tracking: directions stay separate --\n");

    FsdCamDb db;
    MemSrc src = make_src();
    fsd_cam_open(&db, mem_read, &src);

    FsdTracker t;
    fsd_trk_init(&t);
    FsdCamRecord seoul = {375000000, 1270000000, 30, 0};
    uint64_t key = fsd_trk_key(&seoul);

    drive_past(&t, &db, 3.0, NULL);
    drive_past(&t, &db, 3.0, NULL);

    bool learned = false;
    CHECK(fsd_trk_cpa_limit(&t, key, 0.0f, &learned) < 11.0f && learned,
          "northbound is learned");

    // The other carriageway is a different road surface and starts from scratch.
    CHECK(NEAR(fsd_trk_cpa_limit(&t, key, 180.0f, &learned), FSD_CAM_DEFAULT_CPA_M,
               0.01f),
          "southbound falls back to the default");
    CHECK(!learned, "southbound learns nothing from northbound passes");

    CHECK(fsd_trk_cpa_limit(&t, key, 20.0f, &learned) < 11.0f && learned,
          "20 deg off counts as the same approach");
    CHECK(NEAR(fsd_trk_cpa_limit(&t, key, 80.0f, &learned), FSD_CAM_DEFAULT_CPA_M,
               0.01f),
          "80 deg off does not");
}

static void test_track_key(void) {
    printf("\n-- tracking: camera identity --\n");
    FsdCamRecord a = {375000000, 1270000000, 30, 0};
    FsdCamRecord b = {375000001, 1270000000, 30, 0};
    FsdCamRecord c = {375000000, 1270000000, 60, 1}; // same place, other fields
    CHECK(fsd_trk_key(&a) != fsd_trk_key(&b),
          "1e-7 deg apart are different cameras");
    CHECK(fsd_trk_key(&a) == fsd_trk_key(&c), "identity is the position");

    // Negative coordinates sign-extend if cast straight to uint64_t, which would
    // collapse every southern/western camera onto one high half.
    FsdCamRecord s1 = {-375000000, -1270000000, 30, 0};
    FsdCamRecord s2 = {-375000000, -1270000001, 30, 0};
    CHECK(fsd_trk_key(&s1) != fsd_trk_key(&s2),
          "negative coordinates do not collide");
}

// ─────────────────────────────────────────────────────────────────────────────
// Policy (fsd_cam_policy.c)
// ─────────────────────────────────────────────────────────────────────────────

static FsdPolTarget target(uint64_t key, uint8_t limit, float dist) {
    FsdPolTarget t;
    memset(&t, 0, sizeof(t));
    t.valid = true;
    t.key = key;
    t.limit_kph = limit;
    t.distance_m = dist;
    return t;
}

static void test_policy_lead(void) {
    printf("\n-- policy: lead distance and profile table --\n");
    // 12 seconds of travel, clamped at both ends.
    CHECK(NEAR(fsd_pol_lead_distance_m(30.0f), 100.0f, 0.5f), "30 km/h -> 100 m");
    CHECK(NEAR(fsd_pol_lead_distance_m(60.0f), 200.0f, 0.5f), "60 km/h -> 200 m");
    CHECK(NEAR(fsd_pol_lead_distance_m(10.0f), FSD_POL_MIN_LEAD_M, 0.5f),
          "floor at low speed");
    CHECK(NEAR(fsd_pol_lead_distance_m(200.0f), FSD_POL_MAX_LEAD_M, 0.5f),
          "ceiling at high speed");
    CHECK(NEAR(fsd_pol_lead_distance_m(-5.0f), FSD_POL_MIN_LEAD_M, 0.5f),
          "a negative speed does not produce a negative lead");

    // A camera has to be findable before it is actionable.
    CHECK(FSD_TRK_SCAN_RADIUS_M > FSD_POL_MAX_LEAD_M,
          "scan radius covers the longest lead distance");

    CHECK(fsd_pol_profile_for_limit(30) == FSD_POL_PROFILE_CHILL, "30 -> Chill");
    CHECK(fsd_pol_profile_for_limit(50) == FSD_POL_PROFILE_STANDARD, "50 -> Standard");
    CHECK(fsd_pol_profile_for_limit(110) == FSD_POL_PROFILE_STANDARD,
          "110 -> Standard, never higher");
}

static void test_policy_never_raises(void) {
    printf("\n-- policy: never raises the profile --\n");

    FsdPolicy p;
    fsd_pol_init(&p);

    // Driver is in Sloth and a 60 km/h camera maps to Standard, which is
    // FASTER. The prototype returned Standard here — speeding the car up next
    // to a speed camera is a defect, not a trade-off.
    FsdPolTarget tg = target(1, 60, 50.0f);
    FsdPolDecision d =
        fsd_pol_tick(&p, &tg, FSD_POL_PROFILE_SLOTH, 60.0f, 20.0f, 1.0f);
    CHECK(d.action == FSD_POL_ACT_LOWER, "engaged");
    CHECK(d.target_profile == FSD_POL_PROFILE_SLOTH,
          "clamped to Sloth rather than raised to Standard");

    // From Hurry the same camera genuinely lowers.
    fsd_pol_init(&p);
    d = fsd_pol_tick(&p, &tg, FSD_POL_PROFILE_HURRY, 60.0f, 20.0f, 1.0f);
    CHECK(d.target_profile == FSD_POL_PROFILE_STANDARD, "Hurry -> Standard");
}

static void test_policy_arc(void) {
    printf("\n-- policy: approach, hold, restore --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    const uint8_t entry = FSD_POL_PROFILE_HURRY;
    uint8_t observed = entry;

    // 60 km/h means a 200 m trigger, so 350 m is armed but silent.
    FsdPolTarget tg = target(7, 30, 350.0f);
    FsdPolDecision d = fsd_pol_tick(&p, &tg, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ARMED, "armed while still far off");
    CHECK(d.action == FSD_POL_ACT_NONE, "asking for nothing yet");
    CHECK(NEAR(d.trigger_m, 200.0f, 1.0f), "trigger distance reported");

    tg = target(7, 30, 180.0f);
    d = fsd_pol_tick(&p, &tg, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ACTIVE, "engaged inside the trigger");
    CHECK(d.action == FSD_POL_ACT_LOWER && d.target_profile == FSD_POL_PROFILE_CHILL,
          "a 30 km/h camera asks for Chill");
    observed = d.target_profile; // the car complies

    // Passed. Release is not immediate — every camera is assumed to shoot the
    // rear plate, because the public data does not say which ones do.
    fsd_pol_on_pass(&p, 7);
    d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_HOLDING, "holding past the camera");
    CHECK(d.action == FSD_POL_ACT_LOWER, "still lowered");

    // 20 m per tick, 120 m to release: six ticks in total.
    for (int i = 0; i < 4; i++) {
        d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
        CHECK(d.action == FSD_POL_ACT_LOWER, "still holding at tick %d", i);
    }
    d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_RESTORING, "released after 120 m");
    CHECK(d.action == FSD_POL_ACT_RESTORE && d.target_profile == entry,
          "asks for the driver's original profile back");

    // The request is a level, not an edge: a caller that misses one tick does
    // not lose the restore.
    d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.action == FSD_POL_ACT_RESTORE, "restore request repeats");

    observed = entry; // the car arrives
    d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_IDLE && d.action == FSD_POL_ACT_NONE,
          "finished once the car is back");
}

static void test_policy_hold_is_not_interrupted(void) {
    printf("\n-- policy: a new camera cannot cut the hold short --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    uint8_t observed = FSD_POL_PROFILE_HURRY;

    FsdPolTarget a = target(1, 30, 100.0f);
    FsdPolDecision d = fsd_pol_tick(&p, &a, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ACTIVE, "engaged on the first camera");
    observed = d.target_profile;

    fsd_pol_on_pass(&p, 1);

    // The prototype adopted the new camera here and left HOLDING, releasing the
    // profile while still inside the first camera's rear-facing range — exactly
    // what the hold exists to prevent.
    FsdPolTarget b = target(2, 60, 300.0f);
    d = fsd_pol_tick(&p, &b, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_HOLDING, "still holding, not re-armed on the new one");
    CHECK(d.action == FSD_POL_ACT_LOWER, "profile stays down");
}

static void test_policy_entry_does_not_ratchet(void) {
    printf("\n-- policy: cameras in sequence keep one original --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    const uint8_t entry = FSD_POL_PROFILE_HURRY;
    uint8_t observed = entry;

    // Camera A: engage, pass, run the hold out, begin restoring.
    FsdPolTarget a = target(1, 30, 100.0f);
    FsdPolDecision d = fsd_pol_tick(&p, &a, observed, 60.0f, 20.0f, 1.0f);
    observed = d.target_profile;
    fsd_pol_on_pass(&p, 1);
    for (int i = 0; i < 7; i++)
        d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_RESTORING, "restoring after A");

    // B arrives before the car has climbed back.
    FsdPolTarget b = target(2, 30, 100.0f);
    d = fsd_pol_tick(&p, &b, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ACTIVE, "engaged on B");

    fsd_pol_on_pass(&p, 2);
    for (int i = 0; i < 7; i++)
        d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_RESTORING, "restoring after B");
    // The guarded bug: B captures the profile WE set for A as the "original",
    // so a road with cameras in a row ratchets it down every time and the car
    // ends the drive slow with nothing left to restore towards.
    CHECK(d.target_profile == entry, "restores the driver's value, not ours");
}

static void test_policy_abandon(void) {
    printf("\n-- policy: abandon does not restore --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    uint8_t observed = FSD_POL_PROFILE_HURRY;
    FsdPolTarget tg = target(1, 30, 100.0f);
    FsdPolDecision d = fsd_pol_tick(&p, &tg, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ACTIVE, "engaged");
    observed = d.target_profile;

    // Driver override, or authority withdrawn. Forcing the old value back would
    // be overruling someone who has just told us otherwise.
    fsd_pol_abandon(&p);
    d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_IDLE, "idle after abandon");
    CHECK(d.action == FSD_POL_ACT_NONE, "asks for nothing, restore included");

    // A later camera may still engage — abandoning is not a permanent stop.
    d = fsd_pol_tick(&p, &tg, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ACTIVE, "a fresh engagement is still allowed");
}

static void test_policy_standstill(void) {
    printf("\n-- policy: standstill and null safety --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    uint8_t observed = FSD_POL_PROFILE_HURRY;

    FsdPolTarget tg = target(1, 30, 100.0f);
    FsdPolDecision d = fsd_pol_tick(&p, &tg, observed, 0.0f, 0.0f, 1.0f);
    CHECK(d.action == FSD_POL_ACT_NONE, "stopped and not engaged: no decision");

    d = fsd_pol_tick(&p, &tg, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ACTIVE, "engaged while moving");
    observed = d.target_profile;
    // A queue at a school-zone camera is the last place to let the profile up.
    d = fsd_pol_tick(&p, &tg, observed, 0.0f, 0.0f, 1.0f);
    CHECK(d.action == FSD_POL_ACT_LOWER, "stays lowered while stopped");

    CHECK(fsd_pol_phase_str(FSD_POL_HOLDING) != NULL, "phase names exist");
    fsd_pol_tick(NULL, NULL, 0, 0.0f, 0.0f, 0.0f); // must not crash
    fsd_pol_abandon(NULL);
    fsd_trk_init(NULL);
    CHECK(fsd_trk_key(NULL) == 0, "a NULL record has no key");
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistence
// ─────────────────────────────────────────────────────────────────────────────

typedef struct {
    unsigned char buf[8192];
    size_t len;
    size_t pos;
    size_t fail_after; // stop short once this many bytes have moved (0 = never)
} Stream;

static size_t st_write(void* ctx, const void* buf, size_t len) {
    Stream* s = (Stream*)ctx;
    if (s->fail_after && s->len >= s->fail_after) return 0;
    if (s->len + len > sizeof(s->buf)) return 0;
    memcpy(s->buf + s->len, buf, len);
    s->len += len;
    return len;
}

static size_t st_read(void* ctx, void* buf, size_t len) {
    Stream* s = (Stream*)ctx;
    size_t n = s->len - s->pos;
    if (n > len) n = len;
    memcpy(buf, s->buf + s->pos, n);
    s->pos += n;
    return n;
}

static void test_learning_persistence(void) {
    printf("\n-- persistence: learning survives a power cycle --\n");

    FsdCamDb db;
    MemSrc src = make_src();
    fsd_cam_open(&db, mem_read, &src);

    FsdTracker t;
    fsd_trk_init(&t);
    drive_past(&t, &db, 3.0, NULL);
    drive_past(&t, &db, 3.0, NULL);

    FsdCamRecord seoul = {375000000, 1270000000, 30, 0};
    uint64_t key = fsd_trk_key(&seoul);
    bool learned = false;
    float before = fsd_trk_cpa_limit(&t, key, 0.0f, &learned);
    CHECK(learned && before < 11.0f, "learned before saving");

    Stream s;
    memset(&s, 0, sizeof(s));
    CHECK(fsd_trk_save(&t, st_write, &s), "save succeeds");
    CHECK(s.len > FSD_TRK_HEADER_SIZE + 4, "something was written (%u B)",
          (unsigned)s.len);
    CHECK(t.dirty, "save does not clear dirty — only the caller knows it landed");

    // Power cycle.
    FsdTracker t2;
    fsd_trk_init(&t2);
    CHECK(!fsd_trk_cpa_limit(&t2, key, 0.0f, &learned) || !learned,
          "a fresh tracker knows nothing");

    s.pos = 0;
    CHECK(fsd_trk_load(&t2, st_read, &s), "load succeeds");
    float after = fsd_trk_cpa_limit(&t2, key, 0.0f, &learned);
    CHECK(learned, "still learned after the round trip");
    // Samples are stored in centimetres, so the limit comes back to 1 cm.
    CHECK(NEAR(after, before, 0.02f), "limit %.3f -> %.3f survives", (double)before,
          (double)after);
    CHECK(fsd_trk_passes(&t2, key, 0.0f) == 2, "pass count survives");

    // Direction separation has to survive too, or the whole thing collapses to
    // one profile and the opposite carriageway gets excluded by mistake.
    CHECK(NEAR(fsd_trk_cpa_limit(&t2, key, 180.0f, &learned), FSD_CAM_DEFAULT_CPA_M,
               0.01f),
          "southbound is still unlearned after loading");

    // A tracker that has learned nothing writes a header and a trailer, not
    // seven kilobytes of zeros.
    FsdTracker empty;
    fsd_trk_init(&empty);
    Stream es;
    memset(&es, 0, sizeof(es));
    CHECK(fsd_trk_save(&empty, st_write, &es), "empty save succeeds");
    CHECK(es.len == FSD_TRK_HEADER_SIZE + 4, "empty file is %u B, expected %u",
          (unsigned)es.len, (unsigned)(FSD_TRK_HEADER_SIZE + 4));
}

static void test_learning_rejects_damage(void) {
    printf("\n-- persistence: damaged files are refused, not half-read --\n");

    FsdCamDb db;
    MemSrc src = make_src();
    fsd_cam_open(&db, mem_read, &src);

    FsdTracker t;
    fsd_trk_init(&t);
    drive_past(&t, &db, 3.0, NULL);
    drive_past(&t, &db, 3.0, NULL);

    Stream good;
    memset(&good, 0, sizeof(good));
    fsd_trk_save(&t, st_write, &good);

    FsdCamRecord seoul = {375000000, 1270000000, 30, 0};
    uint64_t key = fsd_trk_key(&seoul);
    bool learned = false;

    // A flipped bit in the body. Half a learning store is worse than none: a
    // limit that came back too narrow silently stops warning about a camera
    // that really is ours, and nothing downstream can tell it from a real one.
    Stream bad = good;
    bad.pos = 0;
    bad.buf[FSD_TRK_HEADER_SIZE + 9] ^= 0x20;
    FsdTracker t2;
    CHECK(!fsd_trk_load(&t2, st_read, &bad), "corrupt body refused");
    fsd_trk_cpa_limit(&t2, key, 0.0f, &learned);
    CHECK(!learned, "and the tracker is left empty, not half-filled");

    // Truncated — a write that ran out of space.
    Stream cut = good;
    cut.pos = 0;
    cut.len = good.len - 6;
    CHECK(!fsd_trk_load(&t2, st_read, &cut), "truncated file refused");

    Stream bm = good;
    bm.pos = 0;
    bm.buf[0] = 'X';
    CHECK(!fsd_trk_load(&t2, st_read, &bm), "bad magic refused");

    Stream bv = good;
    bv.pos = 0;
    bv.buf[4] = 99;
    CHECK(!fsd_trk_load(&t2, st_read, &bv), "unknown version refused");

    // Rebuilt with different ring geometry: the records would parse into the
    // wrong fields and look perfectly valid, so the shape is checked explicitly.
    Stream bg = good;
    bg.pos = 0;
    bg.buf[9] = (unsigned char)(FSD_TRK_SAMPLES + 1);
    CHECK(!fsd_trk_load(&t2, st_read, &bg), "different sample geometry refused");

    // A write that fails partway must report failure rather than a short file.
    Stream fw;
    memset(&fw, 0, sizeof(fw));
    fw.fail_after = FSD_TRK_HEADER_SIZE;
    CHECK(!fsd_trk_save(&t, st_write, &fw), "a failing writer is reported");

    CHECK(!fsd_trk_save(NULL, st_write, &fw), "NULL tracker refused");
    CHECK(!fsd_trk_load(&t2, NULL, &fw), "NULL reader refused");
}

// ─────────────────────────────────────────────────────────────────────────────
// 0x3FD decode
// ─────────────────────────────────────────────────────────────────────────────

static void test_profile_decode(void) {
    printf("\n-- 0x3FD: reading the profile back --\n");

    uint8_t f[8];
    uint8_t out = 0xFF;

    // HW3: mux 0, byte 6 bits [2:1]. Built from the DBC position, then checked
    // against what the write path in fsd_handler.c actually emits.
    for (uint8_t v = 0; v <= 3; v++) {
        memset(f, 0, sizeof(f));
        f[0] = 0; // mux 0
        f[6] = (uint8_t)(v << 1);
        f[6] |= 0xF9; // every neighbouring bit set
        CHECK(fsd_sp_decode_profile(f, 8, false, &out), "hw3 mux0 decodes");
        CHECK(out == v, "hw3 profile %u decoded as %u", v, out);
    }

    // Other muxes on the same ID carry different fields entirely.
    memset(f, 0, sizeof(f));
    f[0] = 1;
    f[6] = 0x06;
    CHECK(!fsd_sp_decode_profile(f, 8, false, &out), "hw3 mux1 is not the profile");
    f[0] = 2;
    CHECK(!fsd_sp_decode_profile(f, 8, false, &out), "hw3 mux2 is not the profile");

    // HW4: mux 2, byte 7 bits [7:5], three bits wide.
    for (uint8_t v = 0; v <= 7; v++) {
        memset(f, 0, sizeof(f));
        f[0] = 2;
        f[7] = (uint8_t)(v << 5);
        f[7] |= 0x1F;
        CHECK(fsd_sp_decode_profile(f, 8, true, &out), "hw4 mux2 decodes");
        CHECK(out == v, "hw4 profile %u decoded as %u", v, out);
    }
    memset(f, 0, sizeof(f));
    f[0] = 0;
    CHECK(!fsd_sp_decode_profile(f, 8, true, &out), "hw4 mux0 is not the profile");

    // Short frames must be refused rather than read past.
    memset(f, 0, sizeof(f));
    f[0] = 0;
    CHECK(!fsd_sp_decode_profile(f, 6, false, &out), "hw3 needs 7 bytes");
    f[0] = 2;
    CHECK(!fsd_sp_decode_profile(f, 7, true, &out), "hw4 needs 8 bytes");
    CHECK(!fsd_sp_decode_profile(NULL, 8, false, &out), "NULL data refused");
    CHECK(!fsd_sp_decode_profile(f, 8, false, NULL), "NULL out refused");
    CHECK(!fsd_sp_decode_profile(f, 0, false, &out), "zero dlc refused");
}

// ─────────────────────────────────────────────────────────────────────────────
// Override detection and session budgets
// ─────────────────────────────────────────────────────────────────────────────

static void engage(FsdPolicy* p, uint8_t observed) {
    FsdPolTarget tg = target(1, 30, 100.0f);
    fsd_pol_tick(p, &tg, observed, 60.0f, 20.0f, 1.0f);
}

static void test_override_detection(void) {
    printf("\n-- override: the driver always wins --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    fsd_pol_new_drive(&p);

    // Ask to go from Hurry (3) down to Chill (1).
    engage(&p, FSD_POL_PROFILE_HURRY);
    fsd_pol_observe_profile(&p, FSD_POL_PROFILE_HURRY);

    // Our own convergence steps 3 -> 2 -> 1. The intermediate 2 is not the
    // value we asked for, and a naive "not what we requested" test would call
    // it an override and abandon every multi-step change we ever make.
    CHECK(!fsd_pol_observe_profile(&p, FSD_POL_PROFILE_STANDARD),
          "stepping toward the target is us, not the driver");
    CHECK(!fsd_pol_observe_profile(&p, FSD_POL_PROFILE_CHILL), "arriving is us");
    CHECK(fsd_pol_suspended(&p) == false, "not suspended");

    // Now the driver scrolls it back up. That moves AWAY from the request.
    CHECK(fsd_pol_observe_profile(&p, FSD_POL_PROFILE_STANDARD),
          "moving away from the request is the driver");

    FsdPolDecision d = fsd_pol_tick(&p, NULL, FSD_POL_PROFILE_STANDARD, 60.0f, 20.0f,
                                    1.0f);
    CHECK(d.action == FSD_POL_ACT_NONE, "target abandoned");
    CHECK(d.phase == FSD_POL_IDLE, "and no restore is attempted");

    // Overshoot counts as away too: if the car sailed past what we asked for,
    // something is wrong and letting go is the safe response.
    fsd_pol_init(&p);
    fsd_pol_new_drive(&p);
    engage(&p, FSD_POL_PROFILE_STANDARD);
    fsd_pol_observe_profile(&p, FSD_POL_PROFILE_STANDARD);
    CHECK(!fsd_pol_observe_profile(&p, FSD_POL_PROFILE_CHILL), "arrived at the target");
    CHECK(fsd_pol_observe_profile(&p, FSD_POL_PROFILE_SLOTH), "overshoot is flagged");

    // Nothing requested: the driver moving the profile is simply their business.
    fsd_pol_init(&p);
    fsd_pol_new_drive(&p);
    CHECK(!fsd_pol_observe_profile(&p, FSD_POL_PROFILE_HURRY), "first sample, no history");
    CHECK(!fsd_pol_observe_profile(&p, FSD_POL_PROFILE_SLOTH),
          "with nothing requested, a change is not an override");
}

static void test_session_budgets(void) {
    printf("\n-- budgets: stop arguing after three --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    fsd_pol_new_drive(&p);

    for (unsigned i = 0; i < FSD_POL_MAX_OVERRIDES; i++) {
        engage(&p, FSD_POL_PROFILE_HURRY);
        fsd_pol_observe_profile(&p, FSD_POL_PROFILE_HURRY);
        fsd_pol_observe_profile(&p, FSD_POL_PROFILE_CHILL);   // us, arriving
        CHECK(fsd_pol_observe_profile(&p, FSD_POL_PROFILE_HURRY),
              "override %u detected", i + 1);
    }
    CHECK(fsd_pol_suspended(&p), "suspended after %u overrides", FSD_POL_MAX_OVERRIDES);

    FsdPolTarget tg = target(2, 30, 50.0f);
    FsdPolDecision d = fsd_pol_tick(&p, &tg, FSD_POL_PROFILE_HURRY, 60.0f, 20.0f, 1.0f);
    CHECK(d.action == FSD_POL_ACT_NONE, "suspended: offers nothing even with a camera");

    // A new drive lifts it — a budget that never resets latches the feature off
    // permanently after one bad day.
    fsd_pol_new_drive(&p);
    CHECK(!fsd_pol_suspended(&p), "a new drive lifts the suspension");
    d = fsd_pol_tick(&p, &tg, FSD_POL_PROFILE_HURRY, 60.0f, 20.0f, 1.0f);
    CHECK(d.action == FSD_POL_ACT_LOWER, "and it works again");

    // Convergence failures. Unattended, a car that stopped listening to 0x3C2
    // would otherwise be scrolled at for the entire drive.
    fsd_pol_init(&p);
    fsd_pol_new_drive(&p);
    for (unsigned i = 0; i < FSD_POL_MAX_FAILURES - 1; i++) {
        fsd_pol_on_convergence_failed(&p);
        CHECK(!fsd_pol_suspended(&p), "still trying after %u failures", i + 1);
    }
    fsd_pol_on_convergence_failed(&p);
    CHECK(fsd_pol_suspended(&p), "suspended after %u failures", FSD_POL_MAX_FAILURES);

    // Consecutive, not cumulative: one success clears the count.
    fsd_pol_init(&p);
    fsd_pol_new_drive(&p);
    fsd_pol_on_convergence_failed(&p);
    fsd_pol_on_convergence_failed(&p);
    fsd_pol_on_convergence_ok(&p);
    fsd_pol_on_convergence_failed(&p);
    fsd_pol_on_convergence_failed(&p);
    CHECK(!fsd_pol_suspended(&p), "a success in between resets the count");

    fsd_pol_new_drive(NULL);
    fsd_pol_on_convergence_ok(NULL);
    fsd_pol_on_convergence_failed(NULL);
    CHECK(!fsd_pol_observe_profile(NULL, 0), "NULL policy is not an override");
    CHECK(!fsd_pol_suspended(NULL), "NULL policy is not suspended");
}

static void test_entry_does_not_survive_a_drive(void) {
    printf("\n-- policy: a new drive forgets the last one --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    fsd_pol_new_drive(&p);

    // Drive 1: driver is in Hurry. Engage, pass, run the hold out, start the
    // restore — then park before the car has climbed back.
    uint8_t observed = FSD_POL_PROFILE_HURRY;
    FsdPolTarget a = target(1, 30, 100.0f);
    FsdPolDecision d = fsd_pol_tick(&p, &a, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ACTIVE, "drive 1: engaged");
    observed = d.target_profile; // Chill
    fsd_pol_on_pass(&p, 1);
    for (int i = 0; i < 7; i++)
        d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_RESTORING, "drive 1: restoring when the drive ends");

    // Key goes off. Next drive, the driver has deliberately chosen Sloth — the
    // slowest profile — and sets off.
    fsd_pol_new_drive(&p);
    observed = FSD_POL_PROFILE_SLOTH;

    FsdPolTarget b = target(2, 30, 100.0f);
    d = fsd_pol_tick(&p, &b, observed, 60.0f, 20.0f, 1.0f);
    CHECK(d.phase == FSD_POL_ACTIVE, "drive 2: engaged");
    CHECK(d.target_profile == FSD_POL_PROFILE_SLOTH, "drive 2: never raises");

    fsd_pol_on_pass(&p, 2);
    for (int i = 0; i < 7; i++)
        d = fsd_pol_tick(&p, NULL, observed, 60.0f, 20.0f, 1.0f);

    // The bug: the entry profile from drive 1 survives the parking cycle, the
    // "capture once" guard sees it as already valid, and the restore therefore
    // aims at LAST drive's value. The car is pulled from the Sloth the driver
    // chose up to the Hurry they left behind yesterday — us overruling a
    // person, which is the one thing this layer must never do.
    if (d.phase == FSD_POL_RESTORING) {
        CHECK(d.target_profile == FSD_POL_PROFILE_SLOTH,
              "restores this drive's value (%u), not the last drive's (%u)",
              d.target_profile, FSD_POL_PROFILE_HURRY);
    } else {
        CHECK(d.action == FSD_POL_ACT_NONE,
              "nothing to restore when the drive began at the target");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// End to end
//
// Every piece above is checked on its own. This is the first time a position
// goes in one end and a scroll detent comes out the other:
//
//   fix -> fsd_cam_near -> evaluate -> tracker -> policy -> convergence -> car
//
// The glue in the middle is what the firmware will have to write, so writing it
// here first is the cheapest place to find out it is wrong. Integration is also
// where the interesting failures live: two modules that each behave correctly
// can still disagree about who moved the profile.
// ─────────────────────────────────────────────────────────────────────────────

static void test_end_to_end(void) {
    printf("\n-- end to end: a drive past a school-zone camera --\n");

    FsdCamDb db;
    MemSrc src = make_src();
    CHECK(fsd_cam_open(&db, mem_read, &src), "database opens");

    FsdTracker trk;
    FsdPolicy pol;
    FsdSpeedProfile sp;
    fsd_trk_init(&trk);
    fsd_pol_init(&pol);
    fsd_pol_new_drive(&pol);
    fsd_sp_init(&sp);

    // Stand in for the state after a capture has settled the wire encoding.
    // Everything else on the safety interlock stays as it ships.
    sp.enc.verified = true;

    FsdSpInputs in;
    memset(&in, 0, sizeof(in));
    in.tx_armed = true;
    in.scroll_bus_present = true;
    in.status_fresh = true;

    const uint8_t ENTRY = FSD_POL_PROFILE_HURRY;
    uint8_t car = ENTRY; // the profile the simulated car is actually on

    // 800 m before the camera to 400 m past, 3 m to the side, 60 km/h.
    const double SPEED_MPS = 60.0 / 3.6;
    double pos_m = -800.0;
    uint32_t now = 0;

    int lowered_at_m = 0;
    bool lowered = false;
    // Asking and arriving never coincide: the policy stops emitting RESTORE on
    // the very tick it sees the car has arrived, so "RESTORE while already
    // there" is a state that cannot occur. The two halves are watched
    // separately — that it was asked for, and that the car ended up back.
    bool restore_asked = false;
    int restore_asked_at_m = 0;
    uint8_t highest_asked = 0;
    bool asked = false;
    FsdSpPhase prev_sp = sp.phase;
    double prev_pos = pos_m;

    // 100 ms steps: the convergence machine settles in 400 ms, so a 1 Hz-only
    // loop would never see it move.
    for (int step = 0; step < 1200; step++, now += 100) {
        if (now % 1000 == 0) {
            pos_m = -800.0 + SPEED_MPS * ((double)now / 1000.0);
            FsdCamFix fix = at(37.5 + pos_m / 111320.0, 127.0 + 3.0 / 88000.0, 0.0f,
                               60.0f);

            FsdTrkEvent ev[4];
            int n = fsd_trk_update(&trk, &db, &fix, ev, 4);
            for (int i = 0; i < n; i++) {
                if (ev[i].kind != FSD_TRK_APPROACH) fsd_pol_on_pass(&pol, ev[i].key);
            }

            FsdPolTarget ahead;
            memset(&ahead, 0, sizeof(ahead));
            FsdCamRecord cam;
            uint64_t key;
            float dist;
            if (fsd_trk_nearest(&trk, &cam, &key, &dist)) {
                ahead.valid = true;
                ahead.key = key;
                ahead.limit_kph = cam.limit_kph;
                ahead.distance_m = dist;
            }

            FsdPolDecision d = fsd_pol_tick(&pol, ahead.valid ? &ahead : NULL, car,
                                            60.0f, (float)(pos_m - prev_pos), 1.0f);
            prev_pos = pos_m;

            if (d.action != FSD_POL_ACT_NONE) {
                if (!asked || d.target_profile > highest_asked)
                    highest_asked = d.target_profile;
                asked = true;
                // Only start a convergence when the ask differs from where the
                // car already is, and only when the machine is free.
                if (!fsd_sp_busy(&sp) && car != d.target_profile) {
                    in.observed_profile = car;
                    fsd_sp_request(&sp, &in, d.target_profile, now);
                }
            }
            if (d.action == FSD_POL_ACT_RESTORE && !restore_asked) {
                restore_asked = true;
                restore_asked_at_m = (int)pos_m;
                CHECK(d.target_profile == ENTRY,
                      "restore asks for the driver's value (%u), not ours",
                      d.target_profile);
            }
        }

        // Convergence, and a car that obeys.
        in.observed_profile = car;
        FsdSpAction act = fsd_sp_poll(&sp, &in, now);
        if (act == FSD_SP_ACT_TICK_UP && car < FSD_SP_PROFILE_MAX) car++;
        if (act == FSD_SP_ACT_TICK_DOWN && car > FSD_SP_PROFILE_MIN) car--;
        if (act != FSD_SP_ACT_NONE) {
            fsd_sp_observe(&sp, car, now);
            // The policy watches the same read-back. If it mistook our own
            // stepping for the driver it would abandon here, which is exactly
            // the sort of thing only an integration run catches.
            fsd_pol_observe_profile(&pol, car);
        }

        if (sp.phase != prev_sp) {
            if (sp.phase == FSD_SP_DONE) fsd_pol_on_convergence_ok(&pol);
            if (sp.phase == FSD_SP_FAILED) fsd_pol_on_convergence_failed(&pol);
            prev_sp = sp.phase;
        }

        if (!lowered && car == FSD_POL_PROFILE_CHILL) {
            lowered = true;
            lowered_at_m = (int)pos_m;
        }
    }

    CHECK(lowered, "the car actually reached Chill");
    // Before the camera, and not absurdly early: the trigger at 60 km/h is
    // 200 m, plus whatever the convergence takes.
    CHECK(lowered_at_m < 0, "lowered at %d m — before the camera, not after",
          lowered_at_m);
    CHECK(lowered_at_m > -260, "lowered at %d m — not needlessly early", lowered_at_m);

    CHECK(!fsd_pol_suspended(&pol),
          "no override or failure was misdetected during a clean run");
    CHECK(highest_asked <= ENTRY, "never asked for anything faster than the driver had");
    CHECK(restore_asked, "the original profile was asked for again after the camera");
    // Not at the camera: the release distance exists because public data does
    // not mark which cameras shoot the rear plate, so all of them are assumed to.
    CHECK(restore_asked_at_m >= (int)FSD_POL_RELEASE_DIST_M,
          "restore asked at %d m past — at or beyond the release distance (%d m)",
          restore_asked_at_m, (int)FSD_POL_RELEASE_DIST_M);
    CHECK(car == ENTRY, "and the car got back to it");

    // The drive taught the tracker something, and it is the kind of thing that
    // narrows the limit next time.
    FsdCamRecord seoul = {375000000, 1270000000, 30, 0};
    CHECK(fsd_trk_passes(&trk, fsd_trk_key(&seoul), 0.0f) >= 1,
          "the pass was recorded for next time");
}

// ── what the firmware wiring leans on ────────────────────────────────────────
// camera_task.cpp calls fsd_pol_abandon() on EVERY tick that refuses, which is
// most of them when a fix is missing. That is only safe because abandon does not
// spend the drive's budgets — true today by reading the code, and nothing else
// stops a future edit from adding a budget reset there and silently defeating
// FSD_POL_MAX_OVERRIDES. These assertions are what would notice.

static void test_abandon_is_cheap_to_repeat(void) {
    printf("\n-- abandon: drops the request, keeps the budgets --\n");

    FsdPolicy p;
    fsd_pol_init(&p);
    fsd_pol_new_drive(&p);

    // Spend one override so there is a budget to lose.
    engage(&p, FSD_POL_PROFILE_HURRY);
    fsd_pol_observe_profile(&p, FSD_POL_PROFILE_HURRY);
    fsd_pol_observe_profile(&p, FSD_POL_PROFILE_CHILL);
    CHECK(fsd_pol_observe_profile(&p, FSD_POL_PROFILE_HURRY), "one override spent");
    const uint8_t spent = p.overrides;
    CHECK(spent == 1, "overrides = %u", (unsigned)spent);

    engage(&p, FSD_POL_PROFILE_HURRY);
    CHECK(p.phase == FSD_POL_ACTIVE, "engaged again");

    // The call the firmware makes on a refused tick.
    for (int i = 0; i < 5; i++) fsd_pol_abandon(&p);

    CHECK(p.phase == FSD_POL_IDLE, "phase cleared");
    CHECK(!p.entry_valid, "entry profile forgotten");
    CHECK(!p.has_target, "target dropped");
    CHECK(!p.requested_valid, "request dropped");
    CHECK(p.overrides == spent, "overrides untouched: %u", (unsigned)p.overrides);
    CHECK(p.failures == 0, "failures untouched");
    CHECK(!p.suspended, "suspension untouched");

    // And the entry profile is recaptured from the CURRENT car, not restored
    // from what it was before the refusal — the driver may have moved it while
    // we were blind.
    FsdPolTarget tg = target(9, 30, 50.0f);
    FsdPolDecision d =
        fsd_pol_tick(&p, &tg, FSD_POL_PROFILE_CHILL, 60.0f, 20.0f, 1.0f);
    CHECK(d.action == FSD_POL_ACT_LOWER, "engages again after abandon");
    CHECK(p.entry_profile == FSD_POL_PROFILE_CHILL,
          "entry recaptured from the car now (%u), not from before",
          (unsigned)p.entry_profile);
}

static void test_policy_out_of_range_readback(void) {
    printf("\n-- an out-of-range read-back would defeat the clamp --\n");

    // HW4 carries three bits, and a wrong mux or a wrong car decodes above
    // Hurry. lower_only() compares the request against the observed value, and
    // a request is never above 3 — so an observed 5 makes every clamp inert and
    // the policy would happily RAISE the car next to a camera.
    //
    // This pins the reason camera_task_observe_profile() refuses to store such a
    // value, which would otherwise live only in a comment.
    FsdPolicy p;
    fsd_pol_init(&p);
    FsdPolTarget tg = target(1, 60, 50.0f);
    FsdPolDecision d = fsd_pol_tick(&p, &tg, 5, 60.0f, 20.0f, 1.0f);
    CHECK(d.action == FSD_POL_ACT_LOWER, "it does engage");
    CHECK(d.target_profile == FSD_POL_PROFILE_STANDARD,
          "and asks for Standard (%u) — above an observed 5 the clamp does nothing",
          (unsigned)d.target_profile);
}

static void test_trk_reset_active(void) {
    printf("\n-- reset_active: a gap must not forge a pass --\n");

    FsdCamDb db;
    MemSrc src = make_src();
    fsd_cam_open(&db, mem_read, &src);

    // Learn something first, so we can prove the reset does not touch it.
    FsdTracker t;
    fsd_trk_init(&t);
    for (int i = 0; i < FSD_TRK_MIN_PASSES; i++) drive_past(&t, &db, 3.0, NULL);
    FsdCamRecord seoul = {375000000, 1270000000, 30, 0};
    const uint64_t key = fsd_trk_key(&seoul);
    bool learned = false;
    const float limit_before = fsd_trk_cpa_limit(&t, key, 0.0f, &learned);
    CHECK(learned, "learning is in place before the reset");
    const uint16_t passes_before = fsd_trk_passes(&t, key, 0.0f);

    // Open a track, then reset.
    FsdTrkEvent ev[4];
    FsdCamFix f = at(37.5 - 400.0 / 111320.0, 127.0 + 3.0 / 88000.0, 0.0f, 72.0f);
    fsd_trk_update(&t, &db, &f, ev, 4);
    bool any_active = false;
    for (int i = 0; i < FSD_TRK_ACTIVE_MAX; i++)
        if (t.active[i].used) any_active = true;
    CHECK(any_active, "a camera is being followed");

    const bool dirty_before = t.dirty;
    fsd_trk_reset_active(&t);
    for (int i = 0; i < FSD_TRK_ACTIVE_MAX; i++)
        CHECK(!t.active[i].used, "active[%d] cleared", i);
    CHECK(t.dirty == dirty_before, "dirty untouched");
    CHECK(fsd_trk_passes(&t, key, 0.0f) == passes_before, "pass count untouched");
    bool learned2 = false;
    CHECK(NEAR(fsd_trk_cpa_limit(&t, key, 0.0f, &learned2), limit_before, 0.001f) &&
              learned2,
          "the learned limit survives");
}

static void test_gap_forges_a_pass_without_a_reset(void) {
    printf("\n-- the gap this exists for --\n");

    FsdCamDb db;
    MemSrc src = make_src();
    fsd_cam_open(&db, mem_read, &src);

    // Approach the Seoul camera, then jump 3 km up the same meridian — a tunnel,
    // a reboot, a spell of refused fixes. The interpolation draws ONE chord
    // across the whole gap, and that chord runs straight over the camera.
    FsdTrkEvent ev[4];
    FsdTracker t;
    fsd_trk_init(&t);

    FsdCamFix a = at(37.5 - 400.0 / 111320.0, 127.0 + 3.0 / 88000.0, 0.0f, 72.0f);
    fsd_trk_update(&t, &db, &a, ev, 4);

    FsdCamFix b = at(37.5 + 3000.0 / 111320.0, 127.0 + 3.0 / 88000.0, 0.0f, 72.0f);
    int n = fsd_trk_update(&t, &db, &b, ev, 4);
    bool forged = false;
    for (int i = 0; i < n; i++)
        if (ev[i].kind == FSD_TRK_PASS && ev[i].cam.lat_e7 == 375000000) forged = true;
    // Documents the CURRENT behaviour, which is why the reset is needed. A car
    // that was never there gets a pass recorded, and the pass NARROWS the limit.
    CHECK(forged, "without a reset the gap is recorded as a pass");

    // With the reset the track is gone, and at 3 km the camera is far outside
    // FSD_TRK_SCAN_RADIUS_M, so nothing is picked up and nothing is learned.
    fsd_trk_init(&t);
    fsd_trk_update(&t, &db, &a, ev, 4);
    fsd_trk_reset_active(&t);
    n = fsd_trk_update(&t, &db, &b, ev, 4);
    bool still = false;
    for (int i = 0; i < n; i++)
        if (ev[i].kind == FSD_TRK_PASS) still = true;
    CHECK(!still, "with a reset the gap produces no pass");
    CHECK(!t.dirty, "and nothing was learned from it");
}

// ── the chain, from CAN bytes ────────────────────────────────────────────────
// Every test above starts from a hand-built FsdCamFix, which is fine for the
// geometry but leaves the one question that actually blocked this feature
// unanswered: nothing had ever PRODUCED a fix, so the tracker, the learning
// store and the policy were all unreachable on a car. This drive starts from
// raw 0x3D8 / 0x2F8 / 0x257 frames instead.

static void gps_pos_frame(uint8_t* d, double lat_deg, double lon_deg) {
    const uint64_t lat = (uint64_t)((uint32_t)(int32_t)llround(lat_deg * 1e6) & 0x0FFFFFFFu);
    const uint64_t lon = (uint64_t)((uint32_t)(int32_t)llround(lon_deg * 1e6) & 0x1FFFFFFFu);
    const uint64_t w = lat | (lon << 28) | ((uint64_t)15u << 57); // 3.0 m accuracy
    for (int i = 0; i < 8; i++) d[i] = (uint8_t)((w >> (8 * i)) & 0xFFu);
}

static void gps_vel_frame(uint8_t* d, float heading_deg, float kph) {
    const uint16_t h = (uint16_t)lrintf(heading_deg / 0.0078125f);
    const uint16_t s = (uint16_t)lrintf(kph / 0.00390625f);
    memset(d, 0, 8);
    d[0] = 10; // HDOP 1.0
    d[1] = (uint8_t)(h & 0xFFu);
    d[2] = (uint8_t)(h >> 8);
    d[3] = (uint8_t)(s & 0xFFu);
    d[4] = (uint8_t)(s >> 8);
}

static void di_speed_frame(uint8_t* d, float kph) {
    const uint16_t raw = (uint16_t)lrintf((kph + 40.0f) / 0.08f);
    memset(d, 0, 8);
    d[1] = (uint8_t)((raw & 0x0Fu) << 4);
    d[2] = (uint8_t)(raw >> 4);
}

static void test_gps_feeds_the_tracker(void) {
    printf("\n-- the chain: CAN bytes -> fix -> tracker -> learning --\n");

    FsdCamDb db;
    MemSrc src = make_src();
    CHECK(fsd_cam_open(&db, mem_read, &src), "database opens");

    FsdTracker t;
    FsdGps g;
    fsd_trk_init(&t);
    fsd_gps_init(&g);

    // North along the 127.0 meridian past the Seoul camera at 37.5, 3 m to the
    // side — our own lane. Same geometry as drive_past(), but the fixes come
    // out of the decoder rather than being written by hand.
    bool approached = false, passed = false;
    int fixes = 0, pass_count = 0;
    uint32_t now = 1000;
    for (int m = -410; m <= 210; m += 20, now += 1000) {
        uint8_t d[8];
        gps_pos_frame(d, 37.5 + (double)m / 111320.0, 127.0 + 3.0 / 88000.0);
        fsd_gps_observe_position(&g, d, 8, now);
        gps_vel_frame(d, 0.0f, 72.0f);
        fsd_gps_observe_velocity(&g, d, 8, now);
        di_speed_frame(d, 72.0f);
        fsd_gps_observe_di_speed(&g, d, 8, now);

        FsdCamFix f;
        if (!fsd_gps_fix(&g, now, &f)) continue;
        fixes++;

        FsdTrkEvent ev[4];
        const int n = fsd_trk_update(&t, &db, &f, ev, 4);
        for (int i = 0; i < n; i++) {
            if (ev[i].cam.lat_e7 != 375000000 || ev[i].cam.lon_e7 != 1270000000) continue;
            if (ev[i].kind == FSD_TRK_APPROACH) approached = true;
            if (ev[i].kind == FSD_TRK_PASS) {
                passed = true;
                pass_count = (int)ev[i].passes;
            }
        }
    }

    CHECK(fixes >= 30, "the decoder produced usable fixes, got %d", fixes);
    CHECK(approached, "the camera was picked up from decoded frames");
    CHECK(passed, "and the pass was measured");
    CHECK(pass_count == 1, "one drive past is one pass, got %d", pass_count);
    // The point of the whole exercise: learning now has something to persist.
    // Before this producer existed, `dirty` could never become true on a car.
    CHECK(t.dirty, "the pass reached the learning store");
}

int main(void) {
    printf("test_camera\n");
    test_open();
    test_open_rejects_garbage();
    test_near();
    test_near_respects_max();
    test_geometry();
    test_segment_distance();
    test_evaluate();
    test_track_pass_and_learn();
    test_track_direction_is_separate();
    test_track_key();
    test_policy_lead();
    test_policy_never_raises();
    test_policy_arc();
    test_policy_hold_is_not_interrupted();
    test_policy_entry_does_not_ratchet();
    test_policy_abandon();
    test_policy_standstill();
    test_learning_persistence();
    test_learning_rejects_damage();
    test_profile_decode();
    test_override_detection();
    test_session_budgets();
    test_entry_does_not_survive_a_drive();
    test_end_to_end();
    test_gps_feeds_the_tracker();
    test_abandon_is_cheap_to_repeat();
    test_policy_out_of_range_readback();
    test_trk_reset_active();
    test_gap_forges_a_pass_without_a_reset();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
