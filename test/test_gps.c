/*
 * test_gps.c — host tests for fsd_logic/fsd_gps.c.
 *
 * The frames are built here from engineering units and read back through the
 * decoder, so an error in the bit layout shows up as a wrong number rather than
 * as a crash. That matters more than usual for this file: every value it
 * produces is plausible. A latitude that is ten times too small is still a
 * latitude, a heading folded modulo 360 is still a heading, and nothing
 * downstream can tell that it is wrong — the tracker would simply decide there
 * are no cameras nearby, forever, and look like it was working.
 *
 * The freeze tests are the other half. Staleness is easy and boring; the
 * failure this layer exists for is a fresh, plausible, unchanging position, and
 * it is only detectable against an independent witness that the car is moving.
 *
 * Build + run:  make -C test check
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fsd_gps.h"

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

static bool near_f(float a, float b, float eps) { return fabsf(a - b) <= eps; }

// ── frame builders ───────────────────────────────────────────────────────────
// Deliberately written from the DBC spec independently of the decoder: these
// shift values INTO the word, the decoder shifts them out. A shared helper
// would let one mistake cancel itself.

static void mk_pos(uint8_t* d, int32_t lat_e6, int32_t lon_e6, uint32_t acc_raw) {
    uint64_t w = 0;
    w |= (uint64_t)((uint32_t)lat_e6 & 0x0FFFFFFFu);        //  0|28@1-
    w |= (uint64_t)((uint32_t)lon_e6 & 0x1FFFFFFFu) << 28;  // 28|29@1-
    w |= (uint64_t)(acc_raw & 0x7Fu) << 57;                 // 57|7@1+
    for (int i = 0; i < 8; i++) d[i] = (uint8_t)((w >> (8 * i)) & 0xFFu);
}

static void mk_vel(uint8_t* d, float heading_deg, float speed_kph, uint8_t hdop_raw,
                   bool mia) {
    memset(d, 0, 8);
    const uint16_t h = (uint16_t)lrintf(heading_deg / 0.0078125f);
    const uint16_t s = (uint16_t)lrintf(speed_kph / 0.00390625f);
    d[0] = hdop_raw;
    d[1] = (uint8_t)(h & 0xFFu);
    d[2] = (uint8_t)(h >> 8);
    d[3] = (uint8_t)(s & 0xFFu);
    d[4] = (uint8_t)(s >> 8);
    if (mia) d[6] |= (uint8_t)(1u << 5); // 53|1@1+
}

static void mk_di_speed(uint8_t* d, float kph) {
    memset(d, 0, 8);
    const uint16_t raw = (uint16_t)lrintf((kph + 40.0f) / 0.08f); // 12|12@1+
    d[1] = (uint8_t)((raw & 0x0Fu) << 4);
    d[2] = (uint8_t)(raw >> 4);
}

// ── position decoding ────────────────────────────────────────────────────────

static void test_position_decode(void) {
    printf("\n-- 0x3D8: position --\n");

    uint8_t d[8];
    int32_t lat = 0, lon = 0;
    float acc = -1.0f;

    // Seoul. The frame carries 1e-6 degrees; everything above this layer is
    // 1e-7. Dropping the x10 leaves a perfectly well-formed coordinate 400 km
    // south-west of here, so this assertion is the one that catches it.
    mk_pos(d, 37566500, 126978000, 25);
    CHECK(fsd_gps_decode_position(d, 8, &lat, &lon, &acc), "Seoul decodes");
    CHECK(lat == 375665000, "lat 1e-6 -> 1e-7: want 375665000, got %d", lat);
    CHECK(lon == 1269780000, "lon 1e-6 -> 1e-7: want 1269780000, got %d", lon);
    CHECK(near_f(acc, 5.0f, 0.001f), "accuracy 25*0.2 = 5.0 m, got %.3f", acc);

    // Southern and western hemispheres: both fields are signed and both cross a
    // byte boundary, so a sign-extension slip is invisible on a Korean fixture.
    mk_pos(d, -33868800, -151209300, 0);
    CHECK(fsd_gps_decode_position(d, 8, &lat, &lon, &acc), "southwest decodes");
    CHECK(lat == -338688000, "negative lat: want -338688000, got %d", lat);
    CHECK(lon == -1512093000, "negative lon: want -1512093000, got %d", lon);
    CHECK(near_f(acc, 0.0f, 0.001f), "accuracy 0 = unknown, got %.3f", acc);

    mk_pos(d, 37566500, 126978000, 127);
    fsd_gps_decode_position(d, 8, &lat, &lon, &acc);
    CHECK(near_f(acc, 25.4f, 0.001f), "accuracy saturates at 25.4 m, got %.3f", acc);

    // Extremes of the coordinate system are legal.
    mk_pos(d, 90000000, 180000000, 5);
    CHECK(fsd_gps_decode_position(d, 8, &lat, &lon, &acc), "90N/180E is a place");
    CHECK(lat == 900000000 && lon == 1800000000, "extremes survive the scaling");
}

static void test_position_rejects(void) {
    printf("\n-- 0x3D8: refusals --\n");

    uint8_t d[8];
    int32_t lat = 12345, lon = 67890;
    float acc = 9.0f;

    mk_pos(d, 37566500, 126978000, 25);
    CHECK(!fsd_gps_decode_position(d, 7, &lat, &lon, &acc), "short frame refused");
    CHECK(!fsd_gps_decode_position(NULL, 8, &lat, &lon, &acc), "NULL refused");
    CHECK(lat == 12345 && lon == 67890 && near_f(acc, 9.0f, 0.001f),
          "a refusal must not touch the outputs");

    // Null Island: the receiver has produced nothing, and the value is a real
    // coordinate 400 km off Ghana. Letting it through reads as "no cameras
    // nearby" rather than as an error.
    mk_pos(d, 0, 0, 0);
    CHECK(!fsd_gps_decode_position(d, 8, &lat, &lon, &acc), "(0,0) refused");

    // ...but only exactly (0,0). A metre away is the Atlantic, not a sentinel.
    mk_pos(d, 0, 1, 0);
    CHECK(fsd_gps_decode_position(d, 8, &lat, &lon, &acc), "(0, 1e-6) is a place");

    mk_pos(d, 100000000, 1000000, 0); // 100 deg north
    CHECK(!fsd_gps_decode_position(d, 8, &lat, &lon, &acc), "lat > 90 refused");

    mk_pos(d, 1000000, 200000000, 0); // 200 deg east
    CHECK(!fsd_gps_decode_position(d, 8, &lat, &lon, &acc), "lon > 180 refused");

    mk_pos(d, -100000000, -1000000, 0);
    CHECK(!fsd_gps_decode_position(d, 8, &lat, &lon, &acc), "lat < -90 refused");
}

// ── velocity decoding ────────────────────────────────────────────────────────

static void test_velocity_decode(void) {
    printf("\n-- 0x2F8: heading and speed --\n");

    uint8_t d[8];
    float bearing = -1.0f, speed = -1.0f, hdop = -1.0f;
    bool mia = true;

    mk_vel(d, 123.5f, 88.0f, 12, false);
    CHECK(fsd_gps_decode_velocity(d, 8, &bearing, &speed, &hdop, &mia), "decodes");
    CHECK(near_f(bearing, 123.5f, 0.01f), "heading 123.5, got %.4f", bearing);
    CHECK(near_f(speed, 88.0f, 0.01f), "speed 88.0 km/h, got %.4f", speed);
    CHECK(near_f(hdop, 1.2f, 0.01f), "HDOP 1.2, got %.4f", hdop);
    CHECK(!mia, "NMEA MIA clear");

    mk_vel(d, 0.0f, 0.0f, 0, true);
    CHECK(fsd_gps_decode_velocity(d, 8, &bearing, &speed, &hdop, &mia), "MIA decodes");
    CHECK(mia, "NMEA MIA set");

    // The field holds 511.99 deg. Anything past a full circle means we are not
    // looking at this signal, and folding it would produce a plausible heading
    // pointing somewhere the car is not.
    mk_vel(d, 400.0f, 10.0f, 1, false);
    CHECK(!fsd_gps_decode_velocity(d, 8, &bearing, &speed, &hdop, &mia),
          "heading > 360 refused");

    mk_vel(d, 360.0f, 10.0f, 1, false);
    CHECK(fsd_gps_decode_velocity(d, 8, &bearing, &speed, &hdop, &mia), "360 accepted");
    CHECK(near_f(bearing, 0.0f, 0.001f), "360 normalises to 0, got %.4f", bearing);

    mk_vel(d, 90.0f, 10.0f, 1, false);
    CHECK(!fsd_gps_decode_velocity(d, 7, &bearing, &speed, &hdop, &mia),
          "short frame refused");
}

static void test_di_speed_decode(void) {
    printf("\n-- 0x257: the motion reference --\n");

    FsdGps g;
    uint8_t d[8];

    fsd_gps_init(&g);
    mk_di_speed(d, 62.4f);
    CHECK(fsd_gps_observe_di_speed(&g, d, 8, 1000), "0x257 decodes");
    CHECK(near_f(g.ref_speed_kph, 62.4f, 0.05f), "62.4 km/h, got %.3f", g.ref_speed_kph);

    // The raw field has a -40 offset, so a stationary car sits mid-scale and a
    // low raw value would decode negative. Speed is clamped, not wrapped.
    fsd_gps_init(&g);
    memset(d, 0, 8);
    CHECK(fsd_gps_observe_di_speed(&g, d, 8, 1000), "raw 0 decodes");
    CHECK(near_f(g.ref_speed_kph, 0.0f, 0.001f), "clamped to 0, got %.3f",
          g.ref_speed_kph);

    fsd_gps_init(&g);
    CHECK(!fsd_gps_observe_di_speed(&g, d, 2, 1000), "short frame refused");
    CHECK(!g.ref_seen, "a refused frame is not a sighting");
}

// ── the verdict ladder ───────────────────────────────────────────────────────

// Everything a good fix needs, at time `t`.
static void feed_all(FsdGps* g, int32_t lat_e6, int32_t lon_e6, float heading,
                     float kph, uint32_t t) {
    uint8_t d[8];
    mk_pos(d, lat_e6, lon_e6, 15);
    fsd_gps_observe_position(g, d, 8, t);
    mk_vel(d, heading, kph, 10, false);
    fsd_gps_observe_velocity(g, d, 8, t);
    mk_di_speed(d, kph);
    fsd_gps_observe_di_speed(g, d, 8, t);
}

static void test_verdict_ladder(void) {
    printf("\n-- verdicts: every unknown is a refusal --\n");

    FsdGps g;
    uint8_t d[8];

    fsd_gps_init(&g);
    CHECK(fsd_gps_fix_why(&g, 1000, NULL) == FSD_GPS_NO_POSITION, "fresh: no position");

    mk_pos(d, 37500000, 127000000, 15);
    fsd_gps_observe_position(&g, d, 8, 1000);
    CHECK(fsd_gps_fix_why(&g, 1000, NULL) == FSD_GPS_NO_HEADING, "position alone");

    mk_vel(d, 0.0f, 60.0f, 10, false);
    fsd_gps_observe_velocity(&g, d, 8, 1000);
    // The interesting one: both GPS frames are present and fresh, and the fix
    // still refuses, because with no independent speed a frozen position is
    // indistinguishable from a parked car.
    CHECK(fsd_gps_fix_why(&g, 1000, NULL) == FSD_GPS_NO_MOTION_REF,
          "GPS alone is not enough");

    mk_di_speed(d, 60.0f);
    fsd_gps_observe_di_speed(&g, d, 8, 1000);

    FsdCamFix fix;
    memset(&fix, 0, sizeof(fix));
    CHECK(fsd_gps_fix_why(&g, 1000, &fix) == FSD_GPS_OK, "complete: OK");
    CHECK(fix.lat_e7 == 375000000 && fix.lon_e7 == 1270000000, "fix carries position");
    CHECK(near_f(fix.accuracy_m, 3.0f, 0.01f), "fix carries accuracy");

    // Speed comes from the drivetrain, not from GPS. They agree here on
    // purpose — the test that they are different sources is below.
    CHECK(near_f(fix.speed_kph, 60.0f, 0.05f), "fix carries speed");
}

static void test_fix_speed_is_the_drivetrain(void) {
    printf("\n-- the fix takes speed from 0x257, not from GPS --\n");

    FsdGps g;
    uint8_t d[8];
    fsd_gps_init(&g);

    mk_pos(d, 37500000, 127000000, 15);
    fsd_gps_observe_position(&g, d, 8, 1000);
    mk_vel(d, 0.0f, 95.0f, 10, false); // GPS claims 95
    fsd_gps_observe_velocity(&g, d, 8, 1000);
    mk_di_speed(d, 40.0f); // the wheels say 40
    fsd_gps_observe_di_speed(&g, d, 8, 1000);

    FsdCamFix fix;
    memset(&fix, 0, sizeof(fix));
    CHECK(fsd_gps_fix(&g, 1000, &fix), "fix is usable");
    // In a tunnel the GPS number is the one still reporting the speed the car
    // had on the way in. The ETA the policy acts on must not come from it.
    CHECK(near_f(fix.speed_kph, 40.0f, 0.05f), "want the drivetrain's 40, got %.2f",
          fix.speed_kph);
}

static void test_staleness(void) {
    printf("\n-- staleness --\n");

    FsdGps g;
    uint8_t d[8];

    fsd_gps_init(&g);
    feed_all(&g, 37500000, 127000000, 0.0f, 60.0f, 1000);
    CHECK(fsd_gps_fix_why(&g, 1000, NULL) == FSD_GPS_OK, "fresh");

    // The motion reference has the tightest window, so it expires first.
    CHECK(fsd_gps_fix_why(&g, 1000 + FSD_GPS_REF_FRESH_MS, NULL) == FSD_GPS_NO_MOTION_REF,
          "0x257 expires first");
    CHECK(fsd_gps_fix_why(&g, 1000 + FSD_GPS_POS_FRESH_MS, NULL) == FSD_GPS_POSITION_STALE,
          "then the position");

    // A rejected frame must not look like a heartbeat. Feed a good frame, then
    // rubbish, then check that the clock still dates from the good one.
    fsd_gps_init(&g);
    feed_all(&g, 37500000, 127000000, 0.0f, 60.0f, 1000);
    mk_pos(d, 0, 0, 0); // Null Island
    CHECK(!fsd_gps_observe_position(&g, d, 8, 2000), "rubbish refused");
    CHECK(g.rejects == 1 && g.pos_frames == 1, "counted as a reject, not a frame");
    CHECK(fsd_gps_fix_why(&g, 1000 + FSD_GPS_POS_FRESH_MS, NULL) == FSD_GPS_POSITION_STALE,
          "a refused frame does not refresh the clock");
}

static void test_no_fix_is_not_staleness(void) {
    printf("\n-- NMEA MIA is its own answer --\n");

    FsdGps g;
    uint8_t d[8];

    fsd_gps_init(&g);
    feed_all(&g, 37500000, 127000000, 0.0f, 60.0f, 1000);
    mk_vel(d, 0.0f, 0.0f, 0, true);
    fsd_gps_observe_velocity(&g, d, 8, 1000);
    CHECK(fsd_gps_fix_why(&g, 1000, NULL) == FSD_GPS_NO_FIX,
          "receiver reporting nothing is not the same as no frame");

    // Once that report is old it is history, and saying NO_FIX would send
    // someone looking at the sky when the problem is the bus. Keep the position
    // fresh so this is unambiguously about the heading frame ageing out.
    mk_pos(d, 37500000, 127000000, 15);
    fsd_gps_observe_position(&g, d, 8, 3000);
    CHECK(fsd_gps_fix_why(&g, 1000 + FSD_GPS_VEL_FRESH_MS, NULL) == FSD_GPS_HEADING_STALE,
          "a stale MIA reports as staleness");
}

// ── the frozen fix ───────────────────────────────────────────────────────────

/* Drive north from a starting point, one fix per second. `move` false pins the
 * position where it started while still reporting the speed — a tunnel. */
static void drive(FsdGps* g, int seconds, float kph, bool move, uint32_t* t) {
    uint8_t d[8];
    const int32_t lat0 = 37500000, lon0 = 127000000;
    for (int i = 0; i < seconds; i++) {
        *t += 1000;
        // The frame carries 1e-6 degrees, where one unit is 11 cm of latitude,
        // so 180 units is the ~20 m a car at 72 km/h covers in a second.
        const int32_t lat = move ? lat0 + (int32_t)(i + 1) * 180 : lat0;
        mk_pos(d, lat, lon0, 15);
        fsd_gps_observe_position(g, d, 8, *t);
        mk_vel(d, 0.0f, kph, 10, false);
        fsd_gps_observe_velocity(g, d, 8, *t);
        mk_di_speed(d, kph);
        fsd_gps_observe_di_speed(g, d, 8, *t);
    }
}

static void test_freeze_detection(void) {
    printf("\n-- the frozen fix --\n");

    FsdGps g;
    uint32_t t = 1000;

    // Ordinary driving: fresh, moving, never frozen.
    fsd_gps_init(&g);
    drive(&g, 10, 72.0f, true, &t);
    CHECK(fsd_gps_fix_why(&g, t, NULL) == FSD_GPS_OK, "a real drive stays OK");

    // Tunnel: every frame fresh, every value plausible, position pinned.
    // Staleness cannot see this; only the drivetrain can.
    fsd_gps_init(&g);
    t = 1000;
    drive(&g, 6, 72.0f, false, &t);
    CHECK(fsd_gps_fix_why(&g, t, NULL) == FSD_GPS_FROZEN, "pinned at speed is frozen");

    // ...and it clears the moment the position moves again.
    drive(&g, 3, 72.0f, true, &t);
    CHECK(fsd_gps_fix_why(&g, t, NULL) == FSD_GPS_OK, "coming out of the tunnel");
}

static void test_freeze_does_not_fire_when_stopped(void) {
    printf("\n-- a red light is not a tunnel --\n");

    FsdGps g;
    uint32_t t = 1000;

    // Stopped: position pinned for a long time, and correctly so.
    fsd_gps_init(&g);
    drive(&g, 30, 0.0f, false, &t);
    CHECK(fsd_gps_fix_why(&g, t, NULL) == FSD_GPS_OK, "parked is not frozen");

    // Crawling below the threshold: at 5 km/h the car covers 4 m in three
    // seconds, close enough to GPS noise that "it did not move" proves nothing.
    fsd_gps_init(&g);
    t = 1000;
    drive(&g, 30, 5.0f, false, &t);
    CHECK(fsd_gps_fix_why(&g, t, NULL) == FSD_GPS_OK, "a crawl is not evidence");

    // Pulling away after a long wait must not inherit the stationary window:
    // the car has only been fast for a second, which proves nothing yet.
    fsd_gps_init(&g);
    t = 1000;
    drive(&g, 30, 0.0f, false, &t);
    drive(&g, 1, 72.0f, false, &t);
    CHECK(fsd_gps_fix_why(&g, t, NULL) == FSD_GPS_OK,
          "one second of speed is not enough to call it frozen");
    drive(&g, 3, 72.0f, false, &t);
    CHECK(fsd_gps_fix_why(&g, t, NULL) == FSD_GPS_FROZEN, "...but three more is");
}

static void test_freeze_needs_frames_not_just_time(void) {
    printf("\n-- silence is staleness, not freezing --\n");

    FsdGps g;
    uint8_t d[8];
    fsd_gps_init(&g);

    // We have never measured how often this car sends 0x3D8. If it turned out
    // to be slower than the freeze window, a time-only test would call every
    // normal gap a freeze. Two frames five seconds apart, at the same place, at
    // speed, is exactly that case: both the fast stretch and the no-movement
    // window are well past FSD_GPS_FREEZE_MS, and only the sample floor stands
    // between it and a false FROZEN.
    uint32_t t = 1000;
    for (int i = 0; i < 2; i++) {
        if (i) t += 5000;
        mk_pos(d, 37500000, 127000000, 15);
        fsd_gps_observe_position(&g, d, 8, t);
        mk_vel(d, 0.0f, 72.0f, 10, false);
        fsd_gps_observe_velocity(&g, d, 8, t);
        mk_di_speed(d, 72.0f);
        fsd_gps_observe_di_speed(&g, d, 8, t);
    }
    // Asked at the instant the second frame lands, so nothing is stale and the
    // sample floor is the only thing answering.
    CHECK(g.still_samples == 2, "two frames arrived, got %u",
          (unsigned)g.still_samples);
    CHECK(fsd_gps_fix_why(&g, t, NULL) == FSD_GPS_OK,
          "two frames can never satisfy the sample floor");

    // Let the same data age, and it reads as what it is: silence.
    CHECK(fsd_gps_fix_why(&g, t + FSD_GPS_POS_FRESH_MS, NULL) == FSD_GPS_POSITION_STALE,
          "a slow frame rate reads as stale, not frozen");
}

static void test_verdict_strings(void) {
    printf("\n-- verdict strings --\n");

    // Every value needs a string: these go on the BLE Result characteristic,
    // and "?" in the app is indistinguishable from a bug in the app.
    const FsdGpsVerdict all[] = {
        FSD_GPS_OK,          FSD_GPS_NO_POSITION,   FSD_GPS_POSITION_STALE,
        FSD_GPS_NO_HEADING,  FSD_GPS_HEADING_STALE, FSD_GPS_NO_FIX,
        FSD_GPS_NO_MOTION_REF, FSD_GPS_FROZEN,
    };
    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        const char* s = fsd_gps_verdict_str(all[i]);
        CHECK(s && s[0] && strcmp(s, "?") != 0, "verdict %d has a name", (int)all[i]);
    }
}

static void test_null_safety(void) {
    printf("\n-- NULL --\n");

    uint8_t d[8];
    mk_pos(d, 37500000, 127000000, 15);

    fsd_gps_init(NULL);
    CHECK(!fsd_gps_observe_position(NULL, d, 8, 1000), "NULL tracker refused");
    CHECK(!fsd_gps_observe_velocity(NULL, d, 8, 1000), "NULL tracker refused");
    CHECK(!fsd_gps_observe_di_speed(NULL, d, 8, 1000), "NULL tracker refused");
    fsd_gps_observe_motion_ref(NULL, 10.0f, 1000);
    CHECK(fsd_gps_fix_why(NULL, 1000, NULL) == FSD_GPS_NO_POSITION, "NULL has no fix");
    CHECK(!fsd_gps_fix(NULL, 1000, NULL), "NULL has no fix");

    // Partial outputs: a caller that wants only the position must not have to
    // supply somewhere for the accuracy to go.
    int32_t lat = 0;
    CHECK(fsd_gps_decode_position(d, 8, &lat, NULL, NULL), "partial outputs are fine");
    CHECK(lat == 375000000, "and the one asked for is still filled");
    float bearing = 0.0f;
    mk_vel(d, 45.0f, 30.0f, 5, false);
    CHECK(fsd_gps_decode_velocity(d, 8, &bearing, NULL, NULL, NULL), "partial outputs");
    CHECK(near_f(bearing, 45.0f, 0.01f), "and filled correctly");
}

int main(void) {
    printf("test_gps\n");
    test_position_decode();
    test_position_rejects();
    test_velocity_decode();
    test_di_speed_decode();
    test_verdict_ladder();
    test_fix_speed_is_the_drivetrain();
    test_staleness();
    test_no_fix_is_not_staleness();
    test_freeze_detection();
    test_freeze_does_not_fire_when_stopped();
    test_freeze_needs_frames_not_just_time();
    test_verdict_strings();
    test_null_safety();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
