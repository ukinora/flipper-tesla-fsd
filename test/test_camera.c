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
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
