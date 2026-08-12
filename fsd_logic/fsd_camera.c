/*
 * fsd_camera.c — see fsd_camera.h.
 *
 * Nothing here allocates and nothing here caches: every lookup goes back to
 * flash. That is deliberate — the database is 163 KB and there is no RAM to
 * spare. A lookup costs a ~13-step binary search plus the records of the cells
 * covering the radius, which is a few hundred bytes at 1 Hz.
 */

#include "fsd_camera.h"

#include <math.h>
#include <string.h>

#define M_PER_DEG_LAT 111320.0f

// M_PI is a POSIX extension, not C11 — the build is -std=c11, so define our own
// rather than loosening the standard for one constant.
#define FSD_CAM_PI 3.14159265358979323846f

// Little-endian readers. The file is written little-endian by pack.py and both
// ESP32 and the host test machines are little-endian, but going through these
// keeps the parsing explicit rather than depending on struct layout.
static uint16_t rd16(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static int32_t rdi32(const uint8_t* p) {
    return (int32_t)rd32(p);
}

static float m_per_deg_lon(float lat_deg) {
    float c = cosf(lat_deg * FSD_CAM_PI / 180.0f);
    if(c < 0.05f) c = 0.05f; // never divide the world away near the poles
    return M_PER_DEG_LAT * c;
}

bool fsd_cam_open(FsdCamDb* db, FsdCamReadFn read, void* ctx) {
    if(!db || !read) return false;
    memset(db, 0, sizeof(*db));
    db->rd.read = read;
    db->rd.ctx = ctx;

    uint8_t h[FSD_CAM_HEADER_SIZE];
    if(read(ctx, 0, h, sizeof(h)) != sizeof(h)) return false;
    if(rd32(h) != FSD_CAM_MAGIC) return false;
    if(h[4] != FSD_CAM_VERSION) return false;

    db->grid_e6 = rd32(h + 6);
    db->cell_count = rd32(h + 10);
    db->rec_count = rd32(h + 14);
    db->rec_offset = rd32(h + 18);
    db->crc = rd32(h + 22);
    db->built_at = rd32(h + 26); // 0 on files written before this field existed
    // A zero grid would divide by zero in every lookup; treat as corrupt.
    bool sane = (db->grid_e6 > 0) && (db->cell_count > 0) &&
                (db->rec_offset >= FSD_CAM_HEADER_SIZE);

    /* Counts have to fit the arithmetic that uses them. Nothing bounded them
     * before, and both products are computed in uint32: fsd_cam_total_bytes()
     * does rec_offset + rec_count*10, and find_cell() does
     * HEADER + index*10 with index < cell_count. A corrupt count wraps the
     * product to a small number, which turns "this file is enormous" into
     * "this file is tiny" — the wrong direction for a corruption check.
     *
     * These are exact overflow guards, not invented caps: refuse only what the
     * later arithmetic could not represent. */
    if(sane && db->rec_count > (UINT32_MAX - db->rec_offset) / FSD_CAM_REC_SIZE) sane = false;
    if(sane && db->cell_count > (UINT32_MAX - FSD_CAM_HEADER_SIZE) / FSD_CAM_CELL_SIZE) sane = false;

    db->ok = sane;
    return db->ok;
}

/* Bitwise, so there is no 1 KB table to carry in flash. This runs once per
 * boot over the whole file and once per upload as the bytes stream past;
 * neither is in a loop that cares. */
uint32_t fsd_cam_crc32(uint32_t crc, const void* data, size_t len) {
    const uint8_t* d = (const uint8_t*)data;
    crc = ~crc;
    while(len--) {
        crc ^= *d++;
        for(int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return ~crc;
}

uint32_t fsd_cam_total_bytes(const FsdCamDb* db) {
    if(!db || !db->ok) return 0;
    return db->rec_offset + db->rec_count * FSD_CAM_REC_SIZE;
}

bool fsd_cam_verify(const FsdCamDb* db) {
    if(!db || !db->ok || !db->rd.read) return false;

    uint32_t total = fsd_cam_total_bytes(db);
    /* A header claiming the file ends inside itself is corrupt on its face.
     * Treating it as "nothing to check" would pass it. */
    if(total <= FSD_CAM_HEADER_SIZE) return false;

    uint32_t crc = 0;
    uint32_t at = FSD_CAM_HEADER_SIZE; /* the header carries the expected value,
                                        * so it cannot cover itself */

    /* 256 is a compromise, not a measurement. A 163 KB file is 650 reads at
     * this size and 2,600 at 64; but this also runs on the BLE host task
     * (open_db() is reached from the upload paths), whose stack is smaller
     * than the loop task's. The caller logs how long the pass took, so the
     * first bench run replaces this guess with a number. */
    uint8_t buf[256];
    while(at < total) {
        uint32_t want = total - at;
        if(want > sizeof(buf)) want = sizeof(buf);

        size_t got = db->rd.read(db->rd.ctx, at, buf, want);
        /* A short read means the file is smaller than the header says. That is
         * a truncated file, which is exactly the case a CRC alone would miss:
         * the bytes that ARE there could still sum correctly. */
        if(got != want) return false;

        crc = fsd_cam_crc32(crc, buf, got);
        at += (uint32_t)got;
    }

    return crc == db->crc;
}

/** Binary search the sorted cell array. Returns false when the cell is empty. */
static bool find_cell(const FsdCamDb* db, uint32_t key, uint32_t* rec_index,
                      uint16_t* rec_count) {
    int32_t lo = 0, hi = (int32_t)db->cell_count - 1;
    uint8_t buf[FSD_CAM_CELL_SIZE];
    while(lo <= hi) {
        int32_t mid = lo + (hi - lo) / 2;
        uint32_t off = FSD_CAM_HEADER_SIZE + (uint32_t)mid * FSD_CAM_CELL_SIZE;
        if(db->rd.read(db->rd.ctx, off, buf, sizeof(buf)) != sizeof(buf)) return false;
        uint32_t k = rd32(buf);
        if(k == key) {
            *rec_index = rd32(buf + 4);
            *rec_count = rd16(buf + 8);
            return true;
        }
        if(k < key)
            lo = mid + 1;
        else
            hi = mid - 1;
    }
    return false;
}

int fsd_cam_near(const FsdCamDb* db, int32_t lat_e7, int32_t lon_e7,
                 float radius_m, FsdCamRecord* out, int max) {
    if(!db || !db->ok || !out || max <= 0) return -1;

    const int32_t grid = (int32_t)(db->grid_e6 * 10u); // to 1e-7 degrees
    const float lat_deg = (float)lat_e7 / (float)FSD_CAM_E7;

    int32_t span_lat = (int32_t)(radius_m / M_PER_DEG_LAT * (float)FSD_CAM_E7);
    int32_t span_lon = (int32_t)(radius_m / m_per_deg_lon(lat_deg) * (float)FSD_CAM_E7);

    // Floor division — C truncates toward zero, which is wrong for negatives.
    // Korea is entirely positive, but a wrong cell key is a silent miss so it
    // is worth being right about.
    int32_t li0 = (int32_t)floorf((float)(lat_e7 - span_lat) / (float)grid);
    int32_t li1 = (int32_t)floorf((float)(lat_e7 + span_lat) / (float)grid);
    int32_t lo0 = (int32_t)floorf((float)(lon_e7 - span_lon) / (float)grid);
    int32_t lo1 = (int32_t)floorf((float)(lon_e7 + span_lon) / (float)grid);

    int n = 0;
    for(int32_t li = li0; li <= li1; li++) {
        for(int32_t lo = lo0; lo <= lo1; lo++) {
            if(li < 0 || li > 0xFFFF || lo < 0 || lo > 0xFFFF) continue;
            uint32_t key = ((uint32_t)li << 16) | (uint32_t)lo;
            uint32_t ri = 0;
            uint16_t rc = 0;
            if(!find_cell(db, key, &ri, &rc)) continue;

            for(uint16_t j = 0; j < rc && n < max; j++) {
                uint8_t b[FSD_CAM_REC_SIZE];
                uint32_t off = db->rec_offset + (ri + j) * FSD_CAM_REC_SIZE;
                if(db->rd.read(db->rd.ctx, off, b, sizeof(b)) != sizeof(b)) return -1;
                int32_t clat = rdi32(b);
                int32_t clon = rdi32(b + 4);
                if(fsd_cam_distance_m(lat_e7, lon_e7, clat, clon) > radius_m) continue;
                out[n].lat_e7 = clat;
                out[n].lon_e7 = clon;
                out[n].limit_kph = b[8];
                out[n].flags = b[9];
                n++;
            }
        }
    }
    return n;
}

float fsd_cam_distance_m(int32_t a_lat, int32_t a_lon, int32_t b_lat, int32_t b_lon) {
    float mid_lat = (float)(a_lat / 2 + b_lat / 2) / (float)FSD_CAM_E7;
    float dy = (float)(b_lat - a_lat) / (float)FSD_CAM_E7 * M_PER_DEG_LAT;
    float dx = (float)(b_lon - a_lon) / (float)FSD_CAM_E7 * m_per_deg_lon(mid_lat);
    return sqrtf(dx * dx + dy * dy);
}

float fsd_cam_bearing_deg(int32_t a_lat, int32_t a_lon, int32_t b_lat, int32_t b_lon) {
    float mid_lat = (float)(a_lat / 2 + b_lat / 2) / (float)FSD_CAM_E7;
    float dy = (float)(b_lat - a_lat) / (float)FSD_CAM_E7 * M_PER_DEG_LAT;
    float dx = (float)(b_lon - a_lon) / (float)FSD_CAM_E7 * m_per_deg_lon(mid_lat);
    float deg = atan2f(dx, dy) * 180.0f / FSD_CAM_PI;
    return deg < 0.0f ? deg + 360.0f : deg;
}

float fsd_cam_angle_diff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f);
    if(d < 0.0f) d += 360.0f;
    return d - 180.0f;
}

float fsd_cam_segment_distance_m(int32_t a_lat, int32_t a_lon, int32_t b_lat,
                                 int32_t b_lon, int32_t c_lat, int32_t c_lon) {
    // Work in metres RELATIVE TO `a`, never in absolute metres.
    //
    // Latitude 37.5 deg is 4,174,500 m from the equator. A float carries ~7
    // significant digits, so absolute metres quantise to about 0.25 m — coarser
    // than the lane widths this function exists to distinguish. Subtracting the
    // integer coordinates first keeps every value small and the precision
    // intact. (The Python prototype used doubles and never showed this.)
    float mid_lat = (float)(a_lat / 2 + b_lat / 2) / (float)FSD_CAM_E7;
    float mlon = m_per_deg_lon(mid_lat);
    float bx = (float)(b_lon - a_lon) / (float)FSD_CAM_E7 * mlon;
    float by = (float)(b_lat - a_lat) / (float)FSD_CAM_E7 * M_PER_DEG_LAT;
    float cx = (float)(c_lon - a_lon) / (float)FSD_CAM_E7 * mlon;
    float cy = (float)(c_lat - a_lat) / (float)FSD_CAM_E7 * M_PER_DEG_LAT;
    // `a` is the origin, so the segment direction is simply (bx, by).

    float den = bx * bx + by * by;
    if(den <= 0.0f) return sqrtf(cx * cx + cy * cy);
    float t = (cx * bx + cy * by) / den;
    if(t < 0.0f) t = 0.0f;
    if(t > 1.0f) t = 1.0f;
    float px = cx - t * bx, py = cy - t * by;
    return sqrtf(px * px + py * py);
}

bool fsd_cam_evaluate(const FsdCamFix* fix, const FsdCamRecord* cam,
                      FsdCamHit* out) {
    if(!fix || !cam || !out) return false;

    float d = fsd_cam_distance_m(fix->lat_e7, fix->lon_e7, cam->lat_e7, cam->lon_e7);
    memset(out, 0, sizeof(*out));
    out->cam = *cam;
    if(d <= 0.0f) {
        out->distance_m = 0.0f;
        return true; // standing on it
    }

    float to_cam =
        fsd_cam_bearing_deg(fix->lat_e7, fix->lon_e7, cam->lat_e7, cam->lon_e7);
    float off = fsd_cam_angle_diff(to_cam, fix->bearing_deg);
    if(fabsf(off) > FSD_CAM_FORWARD_CONE_DEG) return false;

    float rad = off * FSD_CAM_PI / 180.0f;
    out->distance_m = d;
    out->along_m = d * cosf(rad);
    out->cpa_m = fabsf(d * sinf(rad));
    out->off_angle_deg = off;
    return true;
}
