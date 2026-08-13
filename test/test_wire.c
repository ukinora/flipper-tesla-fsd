/*
 * test_wire.c — host tests for fsd_logic/fsd_wire.c, and the generator for the
 * byte vectors the Android app is tested against.
 *
 * TWO JOBS, AND THE SECOND IS THE POINT
 * -------------------------------------
 * 1. Assert the packers. Until now nothing did: they lived inside
 *    ble_server.cpp, which cannot be compiled on a host, so every clamp and
 *    scale factor the phone depends on was unverified on both sides.
 *
 * 2. Write test/fixtures/wire_vectors.json. The app's JVM tests parse the SAME
 *    file and assert they recover the same field values. That makes the two
 *    implementations check EACH OTHER rather than both checking the prose in
 *    BLE-GATT-프로토콜.md.
 *
 * Which matters because the wire has already moved twice — State byte 9 went
 * from cruise state to gear, and CamStat went from 12 bytes to 20. An app
 * written from the document would have gone quietly wrong both times, and
 * "quietly" is the part that costs a day of debugging in a car park.
 *
 * The fixture is REGENERATED on every run, so a change to a packer shows up as a
 * diff in review rather than as a surprise in the app. The firmware is the
 * source of truth; if the app disagrees with this file, the app is wrong.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_wire.h"

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

static uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

// ── State ────────────────────────────────────────────────────────────────────

static void test_state_layout(void) {
    printf("\n-- State: every byte, from the spec not the code --\n");

    FsdWireState w;
    memset(&w, 0, sizeof(w));
    w.rx_seen = true;
    w.blinker_right = true;
    w.brake_applied = true;
    w.blackbox_recording = true;
    w.op_mode = 1;      // Active
    w.hw_version = 2;   // HW3
    w.speed_profile = 2;
    w.ap_state = 6;
    w.speed_kph = 63.4f;
    w.soc_percent = 78.0f;
    w.gear = 4; // D
    w.speed_limit_seen = true;
    w.speed_limit_kph = 60.0f;
    w.rx_fps = 1200;
    w.crc_err_count = 3;
    w.uptime_s = 86400;

    uint8_t b[FSD_WIRE_STATE_LEN];
    fsd_wire_pack_state(&w, b);

    CHECK(b[0] == 2, "ver 2, got %u", b[0]);
    // bit0 rx, bit3 right blinker, bit4 recording, bit6 brake = 0x59
    CHECK(b[1] == 0x59u, "flags 0x59, got 0x%02X", b[1]);
    CHECK(b[2] == 1, "op_mode");
    CHECK(b[3] == 2, "hw");
    CHECK(b[4] == 2, "profile");
    CHECK(b[5] == 6, "ap_state");
    CHECK(le16(&b[6]) == 634u, "speed x10 = 634, got %u", le16(&b[6]));
    CHECK(b[8] == 78, "soc");
    CHECK(b[9] == 4, "gear D");
    CHECK(le16(&b[10]) == 60u, "speed limit");
    CHECK(le16(&b[12]) == 1200u, "rx_fps");
    CHECK(le16(&b[14]) == 3u, "crc errors");
    CHECK(le32(&b[16]) == 86400u, "uptime");
}

static void test_state_structural_zeros(void) {
    printf("\n-- State: the bits that are always zero stay zero --\n");

    // Bits 5 (blind spot R) and 7 (profile change) have no input field at all,
    // deliberately: giving them one would invite someone to fill it, and
    // neither is extracted or emitted on this build. The app renders them as
    // unavailable, and this pins that they cannot be set by accident.
    //
    // Bit 4 used to be in this list. It now carries blackbox_recording, which is
    // why that line below is an assertion about the FIELD rather than about the
    // bit: this test failing is exactly what should happen when a spare bit
    // stops being spare, and it did.
    FsdWireState w;
    memset(&w, 0xFF, sizeof(w)); // every bool true, every number huge
    w.speed_kph = 0.0f;
    w.soc_percent = 0.0f;
    w.speed_limit_kph = 0.0f;
    w.speed_profile = 0;

    uint8_t b[FSD_WIRE_STATE_LEN];
    fsd_wire_pack_state(&w, b);
    CHECK((b[1] & (1u << 5)) == 0, "blind spot R never set");
    CHECK((b[1] & (1u << 7)) == 0, "profile-change never set");

    // ...and bit 4 tracks its field in BOTH directions, so it can neither be
    // stuck on nor silently dropped.
    w.blackbox_recording = false;
    fsd_wire_pack_state(&w, b);
    CHECK((b[1] & (1u << 4)) == 0, "recording clear when not recording");
    w.blackbox_recording = true;
    fsd_wire_pack_state(&w, b);
    CHECK((b[1] & (1u << 4)) != 0, "recording set when recording");
}

static void test_state_clamps(void) {
    printf("\n-- State: clamps and saturation --\n");

    FsdWireState w;
    uint8_t b[FSD_WIRE_STATE_LEN];

    // A negative speed is a parser artefact, not a reversing car.
    memset(&w, 0, sizeof(w));
    w.speed_kph = -12.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[6]) == 0, "negative speed clamps to 0");

    // x10 overflows the field above 6553.5 km/h — impossible, but a garbage
    // parse must saturate rather than wrap into a plausible small number.
    memset(&w, 0, sizeof(w));
    w.speed_kph = 99999.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[6]) == 0xFFFFu, "absurd speed saturates, got %u", le16(&b[6]));

    memset(&w, 0, sizeof(w));
    w.soc_percent = 140.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(b[8] == 100, "soc clamps to 100, got %u", b[8]);
    w.soc_percent = -5.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(b[8] == 0, "soc clamps to 0");

    memset(&w, 0, sizeof(w));
    w.speed_profile = 9;
    fsd_wire_pack_state(&w, b);
    CHECK(b[4] == 3, "profile clamps to 3, got %u", b[4]);
    w.speed_profile = -4;
    fsd_wire_pack_state(&w, b);
    CHECK(b[4] == 0, "profile clamps to 0");

    memset(&w, 0, sizeof(w));
    w.crc_err_count = 70000u;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[14]) == 0xFFFFu, "crc count saturates");

    // An unseen limit is zero, never a stale value — the app must show blank
    // rather than a wrong number.
    memset(&w, 0, sizeof(w));
    w.speed_limit_seen = false;
    w.speed_limit_kph = 110.0f;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[10]) == 0, "unseen limit is 0, got %u", le16(&b[10]));

    // Rounding, not truncation: 63.45 -> 635, not 634.
    memset(&w, 0, sizeof(w));
    w.speed_kph = 63.45f;
    fsd_wire_pack_state(&w, b);
    CHECK(le16(&b[6]) == 635u, "speed rounds, got %u", le16(&b[6]));
}

// ── CamStat ──────────────────────────────────────────────────────────────────

static void test_camstat_layout(void) {
    printf("\n-- CamStat: v2, and v1 still parses the first 12 --\n");

    FsdWireCamStat w;
    memset(&w, 0, sizeof(w));
    w.autonomy_enabled = true;
    w.db_loaded = true;
    w.learning_dirty = true;
    w.sup_verdict = 6; // BELT_UNLATCHED
    w.op_mode = 3;     // Autonomous
    w.camera_count = 8123;
    w.built_at = 1754870400u;
    w.gps_verdict = 7; // FROZEN
    w.pol_phase = 2;   // ACTIVE
    w.pol_action = 1;  // LOWER
    w.pol_target = 1;
    w.nearest_m = 143;
    w.gps_accuracy_raw = 15;
    w.raw_profile = 3;
    w.learned_count = 42;
    w.scan_full_count = 2;

    uint8_t b[FSD_WIRE_CAMSTAT_LEN];
    fsd_wire_pack_camstat(&w, b);

    CHECK(b[0] == 2, "ver 2");
    CHECK(b[1] == (0x01u | 0x04u | 0x20u), "flags 0x25, got 0x%02X", b[1]);
    CHECK(b[2] == 6, "sup_verdict");
    CHECK(b[3] == 3, "op_mode");
    CHECK(le32(&b[4]) == 8123u, "camera count");
    CHECK(le32(&b[8]) == 1754870400u, "built_at");
    CHECK(b[12] == 7, "gps_verdict");
    // phase b0-2 | action b3-4 | target b5-6  ->  2 | (1<<3) | (1<<5) = 0x2A
    CHECK(b[13] == 0x2Au, "packed phase/action/target 0x2A, got 0x%02X", b[13]);
    CHECK(le16(&b[14]) == 143u, "nearest metres");
    CHECK(b[16] == 15, "accuracy raw");
    CHECK(b[17] == 3, "raw profile");
    CHECK(b[18] == 42, "learned count");
    CHECK(b[19] == 2, "scan full count");
}

static void test_camstat_packing_cannot_bleed(void) {
    printf("\n-- CamStat: byte 13 fields cannot bleed into each other --\n");

    // Three fields share one byte. If any mask is wrong the neighbours corrupt,
    // and the app would show a plausible wrong phase. Drive each to its maximum
    // with the others also at maximum.
    FsdWireCamStat w;
    memset(&w, 0, sizeof(w));
    w.pol_phase = 7;
    w.pol_action = 3;
    w.pol_target = 3;

    uint8_t b[FSD_WIRE_CAMSTAT_LEN];
    fsd_wire_pack_camstat(&w, b);
    CHECK((b[13] & 0x07u) == 7, "phase survives");
    CHECK(((b[13] >> 3) & 0x03u) == 3, "action survives");
    CHECK(((b[13] >> 5) & 0x03u) == 3, "target survives");

    // Out-of-range inputs are masked, not allowed to overflow into the next
    // field: a policy that grew a sixth phase must not corrupt the action.
    memset(&w, 0, sizeof(w));
    w.pol_phase = 0xFFu;
    fsd_wire_pack_camstat(&w, b);
    CHECK(((b[13] >> 3) & 0x03u) == 0, "an over-range phase does not reach action");
    CHECK(((b[13] >> 5) & 0x03u) == 0, "nor target");
}

static void test_camstat_saturates(void) {
    printf("\n-- CamStat: counts that do not fit a byte --\n");

    FsdWireCamStat w;
    memset(&w, 0, sizeof(w));
    w.learned_count = 500;   // FSD_TRK_CAM_MAX is 128 today, but the field is 16-bit
    w.scan_full_count = 999;

    uint8_t b[FSD_WIRE_CAMSTAT_LEN];
    fsd_wire_pack_camstat(&w, b);
    CHECK(b[18] == 255, "learned count saturates, got %u", b[18]);
    CHECK(b[19] == 255, "scan full count saturates, got %u", b[19]);
}

static void test_result(void) {
    printf("\n-- Result --\n");

    uint8_t b[FSD_WIRE_RESULT_LEN];
    fsd_wire_pack_result(0x41u, 3u, 8123u, b); // SET_AUTONOMY, UNSUPPORTED
    CHECK(b[0] == 0x41u, "command echoed");
    CHECK(b[1] == 3u, "result code");
    CHECK(le16(&b[2]) == 8123u, "extra LE16");
}

static void test_null_safety(void) {
    printf("\n-- NULL --\n");

    uint8_t b[FSD_WIRE_STATE_LEN];
    memset(b, 0xAA, sizeof(b));
    fsd_wire_pack_state(NULL, b);
    CHECK(b[0] == 0xAAu, "NULL input leaves the buffer alone");
    FsdWireState w;
    memset(&w, 0, sizeof(w));
    fsd_wire_pack_state(&w, NULL); // must not crash
    fsd_wire_pack_camstat(NULL, b);
    fsd_wire_pack_result(1, 2, 3, NULL);
    CHECK(1, "NULL outputs do not crash");
}

// ── the fixture the app is tested against ────────────────────────────────────

static void emit_hex(FILE* f, const uint8_t* b, size_t n) {
    fputc('"', f);
    for(size_t i = 0; i < n; i++) fprintf(f, "%02x", b[i]);
    fputc('"', f);
}

/* One case per line-ish, with BOTH the inputs and the resulting bytes. The app
 * parses `hex` and asserts it recovers `fields` — so the assertion is about
 * agreement between two implementations, not about either one's idea of the
 * spec. */
static void emit_fixture(FILE* f) {
    fprintf(f, "{\n");
    fprintf(f, "  \"_generated_by\": \"test/test_wire.c — do not hand-edit\",\n");
    fprintf(f, "  \"state_version\": %u,\n", (unsigned)FSD_WIRE_STATE_VERSION);
    fprintf(f, "  \"camstat_version\": %u,\n", (unsigned)FSD_WIRE_CAMSTAT_VERSION);
    fprintf(f, "  \"state_len\": %u,\n", (unsigned)FSD_WIRE_STATE_LEN);
    fprintf(f, "  \"camstat_len\": %u,\n", (unsigned)FSD_WIRE_CAMSTAT_LEN);
    fprintf(f, "  \"state\": [\n");

    struct {
        const char* name;
        FsdWireState w;
    } states[] = {
        {"zeroed", {0}},
        {"driving_hw3",
         {.rx_seen = true, .blinker_right = true, .brake_applied = true, .op_mode = 1,
          .hw_version = 2, .speed_profile = 2, .ap_state = 6, .speed_kph = 63.4f,
          .soc_percent = 78.0f, .gear = 4, .speed_limit_seen = true,
          .speed_limit_kph = 60.0f, .rx_fps = 1200, .crc_err_count = 3,
          .uptime_s = 86400}},
        {"parked_listen_only",
         {.rx_seen = true, .op_mode = 0, .hw_version = 2, .gear = 1,
          .soc_percent = 55.5f, .rx_fps = 980, .uptime_s = 42}},
        {"autonomous_no_phone",
         {.rx_seen = true, .op_mode = 3, .hw_version = 2, .speed_profile = 1,
          .speed_kph = 88.8f, .soc_percent = 31.0f, .gear = 4,
          .speed_limit_seen = true, .speed_limit_kph = 100.0f, .rx_fps = 1150,
          .uptime_s = 3600}},
        {"clamped_extremes",
         {.speed_kph = 99999.0f, .soc_percent = 140.0f, .speed_profile = 9,
          .crc_err_count = 70000u, .uptime_s = 0xFFFFFFFFu}},
        {"ota_running", {.rx_seen = true, .ota_in_progress = true, .op_mode = 0,
                         .hw_version = 2, .gear = 1, .rx_fps = 1000}},
        /* The bit the phone reads before taking a capture that cannot be
         * retaken. Its own vector so a packer change that drops it fails on
         * both sides of the link rather than on neither. */
        {"recording", {.rx_seen = true, .blackbox_recording = true, .op_mode = 0,
                       .hw_version = 2, .gear = 1, .rx_fps = 1000, .uptime_s = 60}},
    };

    const size_t ns = sizeof(states) / sizeof(states[0]);
    for(size_t i = 0; i < ns; i++) {
        uint8_t b[FSD_WIRE_STATE_LEN];
        fsd_wire_pack_state(&states[i].w, b);
        const FsdWireState* w = &states[i].w;
        fprintf(f, "    { \"name\": \"%s\", \"hex\": ", states[i].name);
        emit_hex(f, b, sizeof(b));
        fprintf(f,
                ", \"fields\": { \"ver\": %u, \"flags\": %u, \"op_mode\": %u, "
                "\"hw\": %u, \"speed_profile\": %u, \"ap_state\": %u, "
                "\"speed_kph_x10\": %u, \"soc\": %u, \"gear\": %u, "
                "\"speed_limit\": %u, \"rx_fps\": %u, \"crc_err\": %u, "
                "\"uptime_s\": %u } }%s\n",
                (unsigned)b[0], (unsigned)b[1], (unsigned)w->op_mode,
                (unsigned)w->hw_version, (unsigned)b[4], (unsigned)w->ap_state,
                (unsigned)le16(&b[6]), (unsigned)b[8], (unsigned)w->gear,
                (unsigned)le16(&b[10]), (unsigned)w->rx_fps,
                (unsigned)le16(&b[14]), (unsigned)le32(&b[16]),
                (i + 1 < ns) ? "," : "");
    }
    fprintf(f, "  ],\n  \"camstat\": [\n");

    struct {
        const char* name;
        FsdWireCamStat w;
    } cams[] = {
        {"zeroed", {0}},
        {"no_gps_yet",
         {.autonomy_enabled = true, .db_loaded = true, .sup_verdict = 1,
          .op_mode = 3, .camera_count = 8123, .built_at = 1754870400u,
          .gps_verdict = 1, .nearest_m = 0xFFFFu, .gps_accuracy_raw = 0xFFu,
          .raw_profile = 0xFFu}},
        {"lowering_for_camera",
         {.autonomy_enabled = true, .supervised_ok = true, .db_loaded = true,
          .autonomy_allows = true, .learning_dirty = true, .profile_fresh = true,
          .sup_verdict = 0, .op_mode = 3, .camera_count = 8123,
          .built_at = 1754870400u, .gps_verdict = 0, .pol_phase = 2,
          .pol_action = 1, .pol_target = 1, .nearest_m = 143,
          .gps_accuracy_raw = 15, .raw_profile = 3, .learned_count = 42}},
        {"gps_frozen_in_tunnel",
         {.autonomy_enabled = true, .supervised_ok = true, .db_loaded = true,
          .autonomy_allows = true, .profile_fresh = true, .op_mode = 3,
          .camera_count = 8123, .gps_verdict = 7, .nearest_m = 0xFFFFu,
          .gps_accuracy_raw = 10, .raw_profile = 2}},
        {"save_failing",
         {.autonomy_enabled = true, .db_loaded = true, .learning_dirty = true,
          .save_failing = true, .sup_verdict = 6, .op_mode = 3,
          .gps_verdict = 6, .nearest_m = 0xFFFFu, .gps_accuracy_raw = 0xFFu,
          .raw_profile = 0xFFu, .learned_count = 500, .scan_full_count = 999}},
        {"field_maxima",
         {.pol_phase = 7, .pol_action = 3, .pol_target = 3, .camera_count = 0xFFFFFFFFu,
          .built_at = 0xFFFFFFFFu, .nearest_m = 0xFFFFu, .gps_accuracy_raw = 0xFFu,
          .raw_profile = 0xFFu}},
    };

    const size_t nc = sizeof(cams) / sizeof(cams[0]);
    for(size_t i = 0; i < nc; i++) {
        uint8_t b[FSD_WIRE_CAMSTAT_LEN];
        fsd_wire_pack_camstat(&cams[i].w, b);
        fprintf(f, "    { \"name\": \"%s\", \"hex\": ", cams[i].name);
        emit_hex(f, b, sizeof(b));
        fprintf(f,
                ", \"fields\": { \"ver\": %u, \"flags\": %u, \"sup_verdict\": %u, "
                "\"op_mode\": %u, \"camera_count\": %u, \"built_at\": %u, "
                "\"gps_verdict\": %u, \"pol_phase\": %u, \"pol_action\": %u, "
                "\"pol_target\": %u, \"nearest_m\": %u, \"gps_accuracy_raw\": %u, "
                "\"raw_profile\": %u, \"learned_count\": %u, "
                "\"scan_full_count\": %u } }%s\n",
                (unsigned)b[0], (unsigned)b[1], (unsigned)b[2], (unsigned)b[3],
                (unsigned)le32(&b[4]), (unsigned)le32(&b[8]), (unsigned)b[12],
                (unsigned)(b[13] & 0x07u), (unsigned)((b[13] >> 3) & 0x03u),
                (unsigned)((b[13] >> 5) & 0x03u), (unsigned)le16(&b[14]),
                (unsigned)b[16], (unsigned)b[17], (unsigned)b[18], (unsigned)b[19],
                (i + 1 < nc) ? "," : "");
    }
    fprintf(f, "  ],\n  \"result\": [\n");

    struct { const char* name; uint8_t cmd, res; uint16_t extra; } res[] = {
        {"set_mode_ok", 0x01u, 0u, 0u},
        {"set_profile_unsupported", 0x10u, 3u, 0u},
        {"dump_not_found", 0x30u, 5u, 0u},
        {"ping_ok_extra", 0x50u, 0u, 65535u},
    };
    const size_t nr = sizeof(res) / sizeof(res[0]);
    for(size_t i = 0; i < nr; i++) {
        uint8_t b[FSD_WIRE_RESULT_LEN];
        fsd_wire_pack_result(res[i].cmd, res[i].res, res[i].extra, b);
        fprintf(f, "    { \"name\": \"%s\", \"hex\": ", res[i].name);
        emit_hex(f, b, sizeof(b));
        fprintf(f, ", \"fields\": { \"cmd\": %u, \"result\": %u, \"extra\": %u } }%s\n",
                (unsigned)res[i].cmd, (unsigned)res[i].res, (unsigned)res[i].extra,
                (i + 1 < nr) ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
    (void)ns;
    (void)nc;
    (void)nr;
}

static void write_fixture(void) {
    const char* path = "fixtures/wire_vectors.json";
    FILE* f = fopen(path, "w");
    if(f) {
        emit_fixture(f);
        fclose(f);
        printf("  wrote %s\n", path);
    } else {
        printf("  NOTE could not write %s — run from test/\n", path);
    }

    /* Also to stdout, between markers. The committed copy has to come FROM the
     * packers rather than from anyone's idea of them, and this is how it gets
     * out of a CI runner and into the repository. */
    printf("----8<---- wire_vectors.json ----8<----\n");
    emit_fixture(stdout);
    printf("---->8---- wire_vectors.json ---->8----\n");
}

int main(void) {
    printf("test_wire\n");
    test_state_layout();
    test_state_structural_zeros();
    test_state_clamps();
    test_camstat_layout();
    test_camstat_packing_cannot_bleed();
    test_camstat_saturates();
    test_result();
    test_null_safety();

    printf("\n-- fixture --\n");
    write_fixture();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
