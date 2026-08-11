#pragma once
/*
 * fsd_camera.h — speed-camera lookup and approach judgement, on the module.
 *
 * WHY THIS LIVES ON THE MODULE
 * ----------------------------
 * The requirement is that camera handling still works when the phone is left
 * behind. That only closes because the car broadcasts its own GPS on CAN
 * (0x3D8 MCU_latitude/longitude, 0x2F8 UI_gpsVehicleHeading), so no extra GPS
 * hardware is needed — the module already has every input it needs.
 *
 * MEMORY IS THE CONSTRAINT
 * ------------------------
 * A T-2CAN has 320 KB of RAM, ~129 KB already in use, and the black-box ring
 * wants another 108 KB. The nationwide database is 163 KB. **It cannot be held
 * in RAM.** So nothing here allocates: the database stays in flash and is read
 * a few hundred bytes at a time through a caller-supplied reader.
 *
 * A lookup is a binary search over the sorted cell array (~13 reads of 10 B)
 * plus the records of the ~9 cells that cover the search radius. At 1 Hz that
 * is nothing.
 *
 * ON PLATFORM INDEPENDENCE
 * ------------------------
 * The reader is a function pointer so the host tests can use fopen() while the
 * firmware uses LittleFS. Everything else is pure arithmetic, which is what
 * lets the Python prototype (camera-db/) and this file be checked against each
 * other.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// camera.bin — see camera-db/pack.py for the writer.
#define FSD_CAM_MAGIC 0x4D414354u /* "TCAM" little-endian */
#define FSD_CAM_VERSION 1u
#define FSD_CAM_HEADER_SIZE 32u
#define FSD_CAM_CELL_SIZE 10u
#define FSD_CAM_REC_SIZE 10u

// Record flags
#define FSD_CAM_FLAG_SECTION (1u << 0) /* section (average-speed) enforcement */
#define FSD_CAM_FLAG_ZONE (1u << 1)    /* protected zone */

// Coordinates are 1e-7 degrees. 1e-7 deg of latitude is ~1.1 cm, so integers
// keep the source precision without dragging floats through the hot path.
#define FSD_CAM_E7 10000000

/** Pulls bytes out of camera.bin. Returns bytes actually read. */
typedef size_t (*FsdCamReadFn)(void* ctx, uint32_t offset, void* buf, size_t len);

typedef struct {
    FsdCamReadFn read;
    void* ctx;
} FsdCamReader;

typedef struct {
    FsdCamReader rd;
    uint32_t grid_e6;    // cell size in 1e-6 degrees
    uint32_t cell_count;
    uint32_t rec_count;
    uint32_t rec_offset; // byte offset of the record array
    uint32_t crc;
    bool ok;
} FsdCamDb;

typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
    uint8_t limit_kph;
    uint8_t flags;
} FsdCamRecord;

/** Where we are and where we are pointed. Filled from CAN, not from a phone. */
typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
    float bearing_deg; // 0 = north, clockwise
    float speed_kph;
    float accuracy_m;  // MCU_gpsAccuracy / UI_gpsHDOP; <= 0 means unknown
} FsdCamFix;

typedef struct {
    FsdCamRecord cam;
    float distance_m;    // straight-line
    float along_m;       // forward component
    float cpa_m;         // how close we pass if we keep going straight
    float off_angle_deg; // heading vs bearing-to-camera
} FsdCamHit;

/** Read and validate the header. Does not read the body. */
bool fsd_cam_open(FsdCamDb* db, FsdCamReadFn read, void* ctx);

/** Records within `radius_m`. Returns how many were written to `out`, or -1.
 *  Never allocates; stops at `max`. */
int fsd_cam_near(const FsdCamDb* db, int32_t lat_e7, int32_t lon_e7,
                 float radius_m, FsdCamRecord* out, int max);

/** Metres between two 1e-7 degree points (planar approximation — good to
 *  centimetres over the few hundred metres we care about). */
float fsd_cam_distance_m(int32_t a_lat, int32_t a_lon, int32_t b_lat, int32_t b_lon);

/** Bearing from a to b, degrees, 0 = north. */
float fsd_cam_bearing_deg(int32_t a_lat, int32_t a_lon, int32_t b_lat, int32_t b_lon);

/** Difference between two bearings, folded to -180..180. */
float fsd_cam_angle_diff(float a, float b);

/** Shortest distance from segment a->b to point c, in metres.
 *
 *  Sampling at 1 Hz and 60 km/h puts fixes 16 m apart, so the closest sample
 *  is never the closest approach — the error is larger than a lane is wide.
 *  Interpolating along the segment removes the dependency on sample spacing.
 */
float fsd_cam_segment_distance_m(int32_t a_lat, int32_t a_lon, int32_t b_lat,
                                 int32_t b_lon, int32_t c_lat, int32_t c_lon);

/** Judge one camera against the current fix. False when it is behind us or too
 *  far off to the side to be on our path. */
bool fsd_cam_evaluate(const FsdCamFix* fix, const FsdCamRecord* cam,
                      FsdCamHit* out);

// A camera is only "ours" if we would pass within this much of it. Wide on
// purpose: GPS error is comparable to lane width, so before the learning layer
// narrows it, over-warning is the safe direction.
#define FSD_CAM_DEFAULT_CPA_M 25.0f

// Beyond this angle the camera is beside or behind us, not ahead.
#define FSD_CAM_FORWARD_CONE_DEG 60.0f

#ifdef __cplusplus
}
#endif
