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

    int32_t a_lat = CAM_LAT - STEP / 2 - STEP;
    int32_t b_lat = a_lat + STEP;
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

int main(void) {
    printf("test_camera\n");
    test_open();
    test_open_rejects_garbage();
    test_near();
    test_near_respects_max();
    test_geometry();
    test_segment_distance();
    test_evaluate();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
