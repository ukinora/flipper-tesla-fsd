/*
 * fsd_gps.c — see fsd_gps.h.
 *
 * Pure arithmetic and three timestamps. No allocation, no platform calls, no
 * dependency on FSDState — the same isolation fsd_camera.c and
 * fsd_speed_profile.c keep, so the host tests exercise exactly the code the
 * firmware runs.
 */

#include "fsd_gps.h"

#include <string.h>

// ── bit plumbing ─────────────────────────────────────────────────────────────

/* Every signal here is little-endian (@1 in the DBC), which means the whole
 * frame is one 64-bit little-endian word and each signal is a shift and a mask.
 * Built byte by byte rather than cast, so host endianness never enters. */
static uint64_t le64(const uint8_t* d) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | (uint64_t)d[i];
    return v;
}

/* Sign-extend the low `bits` of `v`. Done through int64_t rather than the usual
 * (v ^ m) - m trick: that one relies on unsigned wrap then a narrowing cast to
 * a signed type, which C11 leaves implementation-defined. This is plainly
 * correct and the compiler folds it to the same instruction. */
static int32_t sext(uint32_t v, unsigned bits) {
    if (bits == 0u || bits >= 32u) return (int32_t)v;
    const uint32_t sign = 1u << (bits - 1u);
    if (v & sign) return (int32_t)((int64_t)v - ((int64_t)1 << bits));
    return (int32_t)v;
}

static bool stale(uint32_t now_ms, uint32_t stamp_ms, uint32_t window_ms) {
    return (uint32_t)(now_ms - stamp_ms) >= window_ms;
}

// ── decoding ─────────────────────────────────────────────────────────────────

/* Bounds in 1e-7 degrees, i.e. what FsdCamFix and camera.bin use. The frame
 * carries 1e-6, so everything below is scaled by ten on the way out — the one
 * conversion in this file that a reader has to see, because getting it wrong
 * puts the car a tenth of the way to the equator without any value looking
 * obviously wrong. */
#define LAT_E7_MAX 900000000
#define LON_E7_MAX 1800000000

bool fsd_gps_decode_position(const uint8_t* data, uint8_t dlc, int32_t* lat_e7,
                             int32_t* lon_e7, float* accuracy_m) {
    if (!data || dlc < 8) return false;

    const uint64_t w = le64(data);
    const int32_t lat_e6 = sext((uint32_t)(w & 0x0FFFFFFFu), 28u);        //  0|28@1-
    const int32_t lon_e6 = sext((uint32_t)((w >> 28) & 0x1FFFFFFFu), 29u); // 28|29@1-
    const uint32_t acc_raw = (uint32_t)((w >> 57) & 0x7Fu);                // 57|7@1+

    const int64_t lat = (int64_t)lat_e6 * 10;
    const int64_t lon = (int64_t)lon_e6 * 10;
    if (lat > LAT_E7_MAX || lat < -LAT_E7_MAX) return false;
    if (lon > LON_E7_MAX || lon < -LON_E7_MAX) return false;

    /* Null Island. Exactly (0, 0) is the classic "receiver has produced nothing
     * yet" value and it is 400 km off the coast of Ghana, so on a car in Korea
     * it is never a position — it is a fix that has not happened. Letting it
     * through would put every camera in the country several thousand kilometres
     * away, which reads as "no cameras nearby" rather than as an error. */
    if (lat == 0 && lon == 0) return false;

    if (lat_e7) *lat_e7 = (int32_t)lat;
    if (lon_e7) *lon_e7 = (int32_t)lon;
    /* MCU_gpsAccuracy is already metres (0.2 m per count). Zero is reported as
     * unknown rather than as perfect accuracy — FsdCamFix documents <= 0 that
     * way, and a receiver claiming zero error is describing itself, not the
     * world. UI_gpsHDOP is deliberately NOT folded in here: it is a unitless
     * dilution figure, and turning it into metres needs a UERE assumption we
     * have no basis for. */
    if (accuracy_m) *accuracy_m = (float)acc_raw * 0.2f;
    return true;
}

bool fsd_gps_decode_velocity(const uint8_t* data, uint8_t dlc, float* bearing_deg,
                             float* speed_kph, float* hdop, bool* nmea_mia) {
    if (!data || dlc < 8) return false;

    const uint16_t head_raw = (uint16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8));
    const float heading = (float)head_raw * 0.0078125f; // 8|16@1+

    /* The field holds up to 511.99 deg, so anything past 360 is not a compass
     * bearing. Folding it modulo 360 would turn nonsense into a plausible
     * heading and quietly point the forward cone the wrong way; refusing sends
     * it down the staleness path instead, which is the direction that costs a
     * warning rather than causing one. */
    if (heading > 360.0f) return false;

    if (bearing_deg) *bearing_deg = (heading >= 360.0f) ? 0.0f : heading;
    if (speed_kph) {
        const uint16_t s_raw = (uint16_t)((uint16_t)data[3] | ((uint16_t)data[4] << 8));
        *speed_kph = (float)s_raw * 0.00390625f; // 24|16@1+
    }
    if (hdop) *hdop = (float)data[0] * 0.1f; // 0|8@1+
    if (nmea_mia) *nmea_mia = ((data[6] >> 5) & 0x01u) != 0u; // 53|1@1+
    return true;
}

// ── observers ────────────────────────────────────────────────────────────────

void fsd_gps_init(FsdGps* g) {
    if (!g) return;
    memset(g, 0, sizeof(*g));
}

/* Restart the no-movement window at this position. */
static void still_restart(FsdGps* g, int32_t lat_e7, int32_t lon_e7, uint32_t now_ms) {
    g->still_lat_e7 = lat_e7;
    g->still_lon_e7 = lon_e7;
    g->still_since_ms = now_ms;
    g->still_samples = 1u;
}

bool fsd_gps_observe_position(FsdGps* g, const uint8_t* data, uint8_t dlc, uint32_t now_ms) {
    if (!g) return false;

    int32_t lat = 0, lon = 0;
    float acc = 0.0f;
    if (!fsd_gps_decode_position(data, dlc, &lat, &lon, &acc)) {
        g->rejects++;
        return false; // no stamp: a receiver emitting rubbish must go stale
    }

    if (!g->pos_seen) {
        still_restart(g, lat, lon, now_ms);
    } else {
        const float moved = fsd_cam_distance_m(g->still_lat_e7, g->still_lon_e7, lat, lon);
        if (moved > FSD_GPS_FREEZE_MOVED_M) {
            still_restart(g, lat, lon, now_ms);
        } else if (g->still_samples < 0xFFFF) {
            g->still_samples++;
        }
    }

    g->lat_e7 = lat;
    g->lon_e7 = lon;
    g->accuracy_m = acc;
    g->pos_seen = true;
    g->pos_ms = now_ms;
    g->pos_frames++;
    return true;
}

bool fsd_gps_observe_velocity(FsdGps* g, const uint8_t* data, uint8_t dlc, uint32_t now_ms) {
    if (!g) return false;

    float bearing = 0.0f, speed = 0.0f, hdop = 0.0f;
    bool mia = false;
    if (!fsd_gps_decode_velocity(data, dlc, &bearing, &speed, &hdop, &mia)) {
        g->rejects++;
        return false;
    }

    g->bearing_deg = bearing;
    g->gps_speed_kph = speed;
    g->hdop = hdop;
    g->nmea_mia = mia;
    g->vel_seen = true;
    g->vel_ms = now_ms;
    g->vel_frames++;
    return true;
}

void fsd_gps_observe_motion_ref(FsdGps* g, float kph, uint32_t now_ms) {
    if (!g) return;

    const bool fast = kph >= FSD_GPS_FREEZE_MIN_KPH;
    if (fast && !g->fast) g->fast_since_ms = now_ms; // start of a fast stretch
    g->fast = fast;

    g->ref_speed_kph = kph;
    g->ref_seen = true;
    g->ref_ms = now_ms;
}

bool fsd_gps_observe_di_speed(FsdGps* g, const uint8_t* data, uint8_t dlc, uint32_t now_ms) {
    if (!g || !data || dlc < 3) return false;

    // Shared with fsd_drive_observe_speed() through fsd_types.h. Two copies of
    // this arithmetic would drift, and the freeze detector and the speedometer
    // disagreeing about how fast the car is going is the worst possible way for
    // them to drift.
    float kph;
    if (!fsd_decode_di_speed_kph(data, dlc, &kph)) return false;

    fsd_gps_observe_motion_ref(g, kph, now_ms);
    return true;
}

// ── the verdict ──────────────────────────────────────────────────────────────

static bool frozen(const FsdGps* g, uint32_t now_ms) {
    if (!g->fast) return false;
    if (!stale(now_ms, g->fast_since_ms, FSD_GPS_FREEZE_MS)) return false;
    if (!stale(now_ms, g->still_since_ms, FSD_GPS_FREEZE_MS)) return false;
    return g->still_samples >= FSD_GPS_FREEZE_MIN_SAMPLES;
}

FsdGpsVerdict fsd_gps_fix_why(const FsdGps* g, uint32_t now_ms, FsdCamFix* out) {
    if (!g) return FSD_GPS_NO_POSITION;

    if (!g->pos_seen) return FSD_GPS_NO_POSITION;
    if (!g->vel_seen) return FSD_GPS_NO_HEADING;
    if (stale(now_ms, g->pos_ms, FSD_GPS_POS_FRESH_MS)) return FSD_GPS_POSITION_STALE;
    if (stale(now_ms, g->vel_ms, FSD_GPS_VEL_FRESH_MS)) return FSD_GPS_HEADING_STALE;

    /* Checked only once the frame is known fresh: a receiver that reported no
     * data and then went silent is stale news, and reporting NO_FIX for it
     * would send someone looking at the sky instead of at the bus. */
    if (g->nmea_mia) return FSD_GPS_NO_FIX;

    if (!g->ref_seen || stale(now_ms, g->ref_ms, FSD_GPS_REF_FRESH_MS)) {
        return FSD_GPS_NO_MOTION_REF;
    }
    if (frozen(g, now_ms)) return FSD_GPS_FROZEN;

    if (out) {
        out->lat_e7 = g->lat_e7;
        out->lon_e7 = g->lon_e7;
        out->bearing_deg = g->bearing_deg;
        out->speed_kph = g->ref_speed_kph; // see the header: the tunnel-proof one
        out->accuracy_m = g->accuracy_m;
    }
    return FSD_GPS_OK;
}

bool fsd_gps_fix(const FsdGps* g, uint32_t now_ms, FsdCamFix* out) {
    return fsd_gps_fix_why(g, now_ms, out) == FSD_GPS_OK;
}

const char* fsd_gps_verdict_str(FsdGpsVerdict v) {
    switch (v) {
    case FSD_GPS_OK: return "ok";
    case FSD_GPS_NO_POSITION: return "no 0x3D8";
    case FSD_GPS_POSITION_STALE: return "0x3D8 stale";
    case FSD_GPS_NO_HEADING: return "no 0x2F8";
    case FSD_GPS_HEADING_STALE: return "0x2F8 stale";
    case FSD_GPS_NO_FIX: return "receiver reports no fix";
    case FSD_GPS_NO_MOTION_REF: return "no 0x257 speed";
    case FSD_GPS_FROZEN: return "position frozen";
    }
    return "?";
}
