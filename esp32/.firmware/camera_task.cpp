/*
 * camera_task.cpp — see camera_task.h.
 *
 * Threading: every entry point runs on the Arduino loop task. process_frame()
 * has exactly one caller, inside loop(); camera_task_tick() is called from
 * loop(); the accessors are read from ble_server_tick(), which loop() also
 * calls. Nothing here is guarded, and nothing here may be called from a NimBLE
 * callback. The camera database is the one object two tasks reach, and it is
 * borrowed through camera_store's lock rather than held.
 */

#include "camera_task.h"

#ifdef BLE_SERVER_ENABLED

#include "../../fsd_logic/fsd_autonomy.h"
#include "../../fsd_logic/fsd_cam_policy.h"
#include "../../fsd_logic/fsd_cam_track.h"
#include "../../fsd_logic/fsd_camera.h"
#include "../../fsd_logic/fsd_gps.h"
// Explicit, not inherited through main.cpp's include order: this file needs
// CAN_ID_DI_SPEED and CAN_ID_AP_CONTROL and must not depend on who included
// what first.
#include "../../fsd_logic/fsd_handler.h"
#include "../../fsd_logic/fsd_speed_profile.h"
#include "camera_store.h"

#include <Arduino.h>
#include <string.h>

// ── cadence and windows ──────────────────────────────────────────────────────

/* One judgement per second. The fixes themselves arrive no faster than the car
 * sends them, and fsd_cam_near() costs a handful of LittleFS seeks. */
#define CAMERA_TICK_MS 1000u

/* A tick this late means the pipeline lost time — a long BLE operation, a
 * flash write, a reboot. GUESS: loop() jitter has never been measured on this
 * board. It only has to be comfortably above the normal tick and below the
 * point where a straight-line chord stops resembling the road. */
#define CAMERA_MAX_GAP_MS 2500u

/* How long a 0x3FD read-back counts for. GUESS: the frame's rate has never been
 * measured on this car. Too tight and the policy simply never runs, which is
 * visible on CamStat rather than silent. */
#define CAMERA_PROFILE_FRESH_MS 2000u

/* Consecutive refusing ticks before a DATA refusal drops the policy. A fix that
 * blinks out for one second in a cutting should not throw away a restore we
 * still owe the driver. A withdrawal of AUTHORITY gets no such grace. */
#define CAMERA_REFUSE_GRACE 3u

/* How long to wait for the database lock. The BLE task only holds it across a
 * database swap; missing one second of lookups is nothing. */
#define CAMERA_DB_WAIT_MS 10u

/* Learning writes. The debounce RATE-LIMITS; it does not bound data loss to two
 * minutes, because it is conjoined with a standstill test below. On a drive
 * with no stops the real bound is one drive. */
#define CAMERA_SAVE_DEBOUNCE_MS 120000u
#define CAMERA_SAVE_MAX_FAILURES 3u

// ── instances ────────────────────────────────────────────────────────────────

static FsdGps g_gps;
static FsdTracker g_trk;   // ~12.2 KB, the only large object here
static FsdPolicy g_pol;

static FSDState* g_state = nullptr;
static portMUX_TYPE* g_mux = nullptr;

static uint32_t g_last_tick_ms = 0;

static int32_t g_prev_lat_e7 = 0;
static int32_t g_prev_lon_e7 = 0;
static bool g_have_prev = false;

/* Never structurally zero: FSD_GPS_OK is 0, so a field that was merely never
 * written would report a good fix. */
static uint8_t g_gps_verdict = (uint8_t)FSD_GPS_NO_POSITION;

static FsdPolDecision g_decision;
static uint16_t g_nearest_m = 0xFFFFu;

static uint8_t g_observed_profile = 0;
static uint8_t g_raw_profile = 0xFFu; // published before the range check
static uint32_t g_prof_ms = 0;
static bool g_prof_seen = false;

/* Starts true so a module that boots with the car already in D does not mistake
 * the first tick for the start of a drive it did not see. */
static bool g_was_driving = true;
static bool g_drive_edge_seen = false;
static uint8_t g_refuse_ticks = 0;

// persistence
static bool g_want_save = false;
static bool g_was_rx_stale = false;
static uint32_t g_last_save_ms = 0;
static uint8_t g_save_failures = 0;

// ── setup ────────────────────────────────────────────────────────────────────

void camera_task_init(FSDState* state, portMUX_TYPE* mux) {
    g_state = state;
    g_mux = mux;
    fsd_gps_init(&g_gps);
    fsd_pol_init(&g_pol);
    memset(&g_decision, 0, sizeof(g_decision));

    /* Unconditionally initialises the tracker first, so `false` means "start
     * wide", not "the tracker is undefined". */
    camera_store_load_learning(&g_trk);
}

// ── observers ────────────────────────────────────────────────────────────────

bool camera_task_observe(uint32_t id, const uint8_t* data, uint8_t dlc, uint32_t now_ms) {
    switch(id) {
    case FSD_CAN_ID_MCU_LOCATION:
        fsd_gps_observe_position(&g_gps, data, dlc, now_ms);
        return true;
    case FSD_CAN_ID_UI_GPS:
        fsd_gps_observe_velocity(&g_gps, data, dlc, now_ms);
        return true;
    case CAN_ID_DI_SPEED:
        /* The independent witness the freeze detector needs. This build has
         * never parsed 0x257 — fsd_logic/fsd_handler.c is not compiled here —
         * so FSDState.vehicle_speed_kph has always been structurally zero. */
        fsd_gps_observe_di_speed(&g_gps, data, dlc, now_ms);
        return true;
    default:
        return false;
    }
}

void camera_task_observe_profile(bool hw4, const uint8_t* data, uint8_t dlc, uint32_t now_ms) {
    uint8_t v = 0;
    if(!fsd_sp_decode_profile(data, dlc, hw4, &v)) return;

    /* Published whatever it is, so a bring-up drive can see a mis-decode
     * instead of just seeing the policy never run. */
    g_raw_profile = v;

    /* HW4 carries three bits, and a wrong mux or a wrong car gives values above
     * Hurry. Storing one would defeat the never-raise clamp in fsd_pol_tick():
     * it compares the request against this number, and a request is always <= 3,
     * so an observed 5 would make every clamp inert. Not stored, not stamped —
     * the tick then sees no read-back and releases. */
    if(v > FSD_POL_PROFILE_HURRY) return;

    g_observed_profile = v;
    g_prof_ms = now_ms;
    g_prof_seen = true;

    /* Override detection is direction-based and needs EVERY sample, not one per
     * judgement tick: at 1 Hz a driver's turn of the wheel and our own
     * convergence become indistinguishable. */
    fsd_pol_observe_profile(&g_pol, v);
}

// ── the tick ─────────────────────────────────────────────────────────────────

/* Drop what is in flight. `hard` is a withdrawal of authority, which must take
 * effect on the same tick (페일세이프-정책.md §7); a data refusal gets a few
 * ticks of grace so one blink in a cutting does not discard a restore we still
 * owe. Safe to call repeatedly: fsd_pol_abandon() clears the target and the
 * entry profile but leaves the drive's override and failure budgets alone. */
static void camera_release(bool hard) {
    if(hard) {
        g_refuse_ticks = 0;
        fsd_pol_abandon(&g_pol);
    } else if(g_refuse_ticks < CAMERA_REFUSE_GRACE) {
        if(++g_refuse_ticks >= CAMERA_REFUSE_GRACE) fsd_pol_abandon(&g_pol);
    }

    /* The measurements go either way. A track carried across a refusal would be
     * interpolated over the gap when fixes come back. */
    fsd_trk_reset_active(&g_trk);
    g_have_prev = false;
    memset(&g_decision, 0, sizeof(g_decision));
    g_nearest_m = 0xFFFFu;
}

static void camera_judge(const FsdCamFix* fix, float dt_s) {
    const float moved_m =
        g_have_prev ? fsd_cam_distance_m(g_prev_lat_e7, g_prev_lon_e7,
                                         fix->lat_e7, fix->lon_e7)
                    : 0.0f;
    g_prev_lat_e7 = fix->lat_e7;
    g_prev_lon_e7 = fix->lon_e7;
    g_have_prev = true;

    /* Borrowed for this burst only. Caching it across ticks would survive a
     * database swap and quietly report "no cameras anywhere". */
    const FsdCamDb* db = camera_store_db_acquire(CAMERA_DB_WAIT_MS);
    FsdTrkEvent ev[4];
    const int n = fsd_trk_update(&g_trk, db, fix, ev, 4);
    if(db) camera_store_db_release();

    for(int i = 0; i < n; i++) {
        if(ev[i].kind == FSD_TRK_PASS || ev[i].kind == FSD_TRK_DROP)
            fsd_pol_on_pass(&g_pol, ev[i].key);
    }

    FsdPolTarget ahead;
    memset(&ahead, 0, sizeof(ahead));
    FsdCamRecord cam;
    uint64_t key = 0;
    float dist = 0.0f;
    if(fsd_trk_nearest(&g_trk, &cam, &key, &dist)) {
        ahead.valid = true;
        ahead.key = key;
        ahead.limit_kph = cam.limit_kph;
        ahead.distance_m = dist;
        g_nearest_m = (dist >= 65535.0f || dist < 0.0f) ? 0xFFFEu : (uint16_t)dist;
    } else {
        g_nearest_m = 0xFFFFu;
    }

    /* PUBLISHED, NEVER ACTED ON. There is no send path in this file — see the
     * header. Wiring this to fsd_sp_request() is a separate decision that needs
     * the 0x3C2 encoding captured from the car first. */
    g_decision = fsd_pol_tick(&g_pol, &ahead, g_observed_profile,
                              fix->speed_kph, moved_m, dt_s);
}

/* Learning is worthless if it does not survive the car sleeping, and the
 * accessory feed is switched — power vanishes without warning. Three triggers,
 * all reached from loop(), never from a BLE callback. */
static void camera_persist(uint32_t now_ms, bool rx_stale) {
    /* The edge first, before any return: rx_stale is a LEVEL, recomputed every
     * loop iteration and true for as long as the bus stays quiet. Sampling it
     * after an early return would either miss the transition or fire on every
     * subsequent tick. */
    const bool quiet_edge = rx_stale && !g_was_rx_stale;
    g_was_rx_stale = rx_stale;

    if(!g_trk.dirty) { g_want_save = false; return; }
    if(camera_store_uploading()) return;  // keep g_want_save LATCHED for later
    if(g_save_failures >= CAMERA_SAVE_MAX_FAILURES) return;

    /* Speed comes from the drivetrain reference, never from a fix. A fix only
     * exists on the FSD_GPS_OK path, so "no fix" must not be read as
     * "stationary" — that is precisely the tunnel, at 100 km/h. */
    const bool slow = g_gps.ref_seen && g_gps.ref_speed_kph < FSD_POL_MIN_SPEED_KPH;
    const bool ref_fresh =
        g_gps.ref_seen && (uint32_t)(now_ms - g_gps.ref_ms) < FSD_GPS_REF_FRESH_MS;
    const bool due = (uint32_t)(now_ms - g_last_save_ms) >= CAMERA_SAVE_DEBOUNCE_MS;

    const bool go = g_want_save                  // D/R -> P/N: the drive ended
                 || (quiet_edge && slow)         // the bus going quiet at a stop
                 || (due && ref_fresh && slow);  // periodic, only at a standstill
    if(!go) return;

    /* Stamped on the ATTEMPT. Stamping on success would turn a failing write
     * into a full-file rewrite every second. camera_store_save_learning()
     * ignores t->dirty, so the entire rate policy is this function. */
    g_last_save_ms = now_ms;
    if(camera_store_save_learning(&g_trk)) {
        g_want_save = false;
        g_save_failures = 0;
    } else {
        g_save_failures++;
        Serial.printf("[CAM] learning save failed (%u)\n", (unsigned)g_save_failures);
    }
}

void camera_task_tick(uint32_t now_ms) {
    if((uint32_t)(now_ms - g_last_tick_ms) < CAMERA_TICK_MS) return;
    const uint32_t elapsed = (uint32_t)(now_ms - g_last_tick_ms);
    g_last_tick_ms = now_ms;
    const float dt_s = (float)elapsed / 1000.0f; // measured, never assumed 1.0

    bool allowed;
    bool rx_stale;
    uint8_t gear;
    bool gear_seen;
    portENTER_CRITICAL(g_mux);
    allowed = fsd_autonomy_allows(g_state, now_ms);
    rx_stale = g_state->rx_stale;
    gear = g_state->di_gear;
    gear_seen = g_state->di_gear_seen;
    portEXIT_CRITICAL(g_mux);

    // Drive edges. A budget that never resets would latch the feature off.
    const bool driving = gear_seen && (gear == FSD_GEAR_D || gear == FSD_GEAR_R);
    if(driving && !g_was_driving) {
        fsd_pol_new_drive(&g_pol);
        g_drive_edge_seen = true;
        g_save_failures = 0; // a new drive is worth retrying a failing write for
    } else if(!driving && g_was_driving) {
        g_want_save = true;  // the car is stopped: the best moment to write
    }
    g_was_driving = driving;

    /* A late tick means the chord between fixes no longer resembles the road.
     * Interpolating across it can graze a camera the car never went near, and
     * that would be recorded as a pass and NARROW its learned limit. */
    if(elapsed >= CAMERA_MAX_GAP_MS) {
        fsd_trk_reset_active(&g_trk);
        g_have_prev = false;
    }

    FsdCamFix fix;
    memset(&fix, 0, sizeof(fix));
    g_gps_verdict = (uint8_t)fsd_gps_fix_why(&g_gps, now_ms, &fix);
    const bool prof_fresh =
        g_prof_seen && (uint32_t)(now_ms - g_prof_ms) < CAMERA_PROFILE_FRESH_MS;

    /* ONE TAIL. Every branch falls through to camera_persist() — an early
     * return here would skip persistence on exactly the ticks that need it,
     * because fsd_autonomy_allows() is false when the bus goes quiet and when
     * the gear leaves D, which are two of the three save triggers. */
    if(!allowed || !g_drive_edge_seen) {
        camera_release(true);
    } else if(g_gps_verdict != (uint8_t)FSD_GPS_OK || !prof_fresh) {
        camera_release(false);
    } else {
        g_refuse_ticks = 0;
        camera_judge(&fix, dt_s);
    }

    camera_persist(now_ms, rx_stale);
}

// ── accessors ────────────────────────────────────────────────────────────────

uint8_t camera_task_gps_verdict(void) { return g_gps_verdict; }
uint8_t camera_task_pol_phase(void) { return (uint8_t)g_decision.phase; }
uint8_t camera_task_pol_action(void) { return (uint8_t)g_decision.action; }
uint8_t camera_task_pol_target(void) { return g_decision.target_profile; }
bool camera_task_pol_suspended(void) { return fsd_pol_suspended(&g_pol); }
uint16_t camera_task_nearest_m(void) { return g_nearest_m; }
uint8_t camera_task_raw_profile(void) { return g_raw_profile; }
bool camera_task_learning_dirty(void) { return g_trk.dirty; }
bool camera_task_save_failing(void) { return g_save_failures > 0; }
uint16_t camera_task_scan_full_count(void) { return g_trk.scan_full_count; }

uint8_t camera_task_gps_accuracy_raw(void) {
    if(!g_gps.pos_seen || g_gps.accuracy_m <= 0.0f) return 0xFFu;
    const float raw = g_gps.accuracy_m / 0.2f;
    return (raw >= 254.0f) ? 254u : (uint8_t)(raw + 0.5f);
}

bool camera_task_profile_fresh(void) {
    // Same test the tick makes, so the app sees the reason the policy is idle.
    return g_prof_seen && (uint32_t)(millis() - g_prof_ms) < CAMERA_PROFILE_FRESH_MS;
}

uint16_t camera_task_learned_count(void) {
    uint16_t n = 0;
    for(int i = 0; i < FSD_TRK_CAM_MAX; i++)
        if(g_trk.mem[i].used) n++;
    return n;
}

#endif  // BLE_SERVER_ENABLED
