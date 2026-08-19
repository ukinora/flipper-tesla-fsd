/*
 * test_cap_json.c — host tests for fsd_logic/fsd_cap_json.c.
 *
 * 🔴 THE POINT OF THIS FILE IS ONE ASSERTION: the largest document the module
 * can ever publish still fits in the 512 bytes a GATT attribute holds.
 *
 * That document has overflowed three times. Each time NimBLE refused the
 * oversized value outright, the characteristic held zero bytes, and the phone
 * read an empty document — indistinguishable from "the module has not answered
 * yet". Each time it was found on hardware, and the third time it would have
 * been found IN THE CAR, because the second bus only enters the document once
 * it has seen a capability id and the bench replay files carry none.
 *
 * So the guard cannot be "it fits with the values we have". It has to be "it
 * fits with the largest values these fields can hold", which is what
 * fsd_cap_json_worst_*() build.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_cap_json.h"

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

/* Roomy enough that a document which does NOT fit the real limit is still
 * rendered in full — otherwise a failure would report 0 and hide by how much. */
static char g_buf[4096];

/* ── the size guard ──────────────────────────────────────────────────────── */

static void test_worst_status_fits_the_att_limit(void) {
    FsdCapJsonStatus s;
    fsd_cap_json_worst_status(&s);

    const size_t n = fsd_cap_json_status(g_buf, sizeof(g_buf), &s);

    CHECK(n > 0u, "worst-case status rendered nothing");
    CHECK(n <= FSD_CAP_JSON_ATTR_MAX,
          "worst-case status is %u bytes, %u over the %u-byte ATT limit",
          (unsigned)n, (unsigned)(n - FSD_CAP_JSON_ATTR_MAX),
          (unsigned)FSD_CAP_JSON_ATTR_MAX);
}

static void test_worst_buttons_fits_the_att_limit(void) {
    FsdCapJsonButtons b;
    fsd_cap_json_worst_buttons(&b);

    const size_t n = fsd_cap_json_buttons(g_buf, sizeof(g_buf), &b);

    CHECK(n > 0u, "worst-case buttons rendered nothing");
    CHECK(n <= FSD_CAP_JSON_ATTR_MAX,
          "worst-case buttons is %u bytes, %u over the %u-byte ATT limit",
          (unsigned)n, (unsigned)(n - FSD_CAP_JSON_ATTR_MAX),
          (unsigned)FSD_CAP_JSON_ATTR_MAX);
}

/* Build the documents the car actually produces, so the numbers land in the
 * log next to the limit. Not an assertion about a specific size — that would
 * pin a value nobody chose — but the margin is the thing a reader wants. */
static void report_realistic_sizes(void) {
    FsdCapJsonStatus s;
    memset(&s, 0, sizeof(s));
    s.state = 2u; s.window_ms = 4000u; s.hw = 2u; s.bus_count = 2u;
    memcpy(s.bus[0].name, "can0", 5);
    memcpy(s.bus[1].name, "can1", 5);
    s.bus[1].frames = 39999u; s.bus[1].ids = 2047u; s.bus[1].hint = 2u;
    s.bus[1].hw_unconfirmed = true;
    const size_t ns = fsd_cap_json_status(g_buf, sizeof(g_buf), &s);

    FsdCapJsonButtons b;
    memset(&b, 0, sizeof(b));
    b.connected = true; b.verified = true; b.slots = 5u; b.bound_count = 1u;
    memcpy(b.bound[0].addr, "2f:95:e6:54:51:84", 18);
    b.bound[0].connected = true; b.bound[0].decoded = 42u;
    b.act = 5u;
    const size_t nb = fsd_cap_json_buttons(g_buf, sizeof(g_buf), &b);

    FsdCapJsonStatus ws;  fsd_cap_json_worst_status(&ws);
    FsdCapJsonButtons wb; fsd_cap_json_worst_buttons(&wb);
    const size_t xs = fsd_cap_json_status(g_buf, sizeof(g_buf), &ws);
    const size_t xb = fsd_cap_json_buttons(g_buf, sizeof(g_buf), &wb);

    printf("  [size] limit %u | status %u B car / %u B worst | buttons %u B car / %u B worst\n",
           (unsigned)FSD_CAP_JSON_ATTR_MAX,
           (unsigned)ns, (unsigned)xs, (unsigned)nb, (unsigned)xb);
}

/* The worst case must actually be the worst case. A realistic document has to
 * be no larger — if it is, worst_*() has stopped covering some field and the
 * guards above are measuring the wrong thing. */
static void test_worst_case_is_not_smaller_than_a_real_one(void) {
    FsdCapJsonStatus worst_s;
    fsd_cap_json_worst_status(&worst_s);
    const size_t big_s = fsd_cap_json_status(g_buf, sizeof(g_buf), &worst_s);

    FsdCapJsonStatus real_s;
    memset(&real_s, 0, sizeof(real_s));
    real_s.state = 2u; real_s.ms_left = 3999u; real_s.window_ms = 4000u;
    real_s.hw = 2u; real_s.bus_count = 2u;
    memcpy(real_s.bus[0].name, "can0", 5);
    memcpy(real_s.bus[1].name, "can1", 5);
    real_s.bus[1].frames = 4000000000u; real_s.bus[1].ids = 2047u;
    real_s.bus[1].hint = 2u; real_s.bus[1].hw_unconfirmed = true;
    real_s.bus[1].nag_killer = 2u; real_s.bus[1].scroll_profile = 2u;
    const size_t small_s = fsd_cap_json_status(g_buf, sizeof(g_buf), &real_s);

    CHECK(small_s > 0u, "a realistic status rendered nothing");
    CHECK(small_s <= big_s, "a realistic status (%u B) is LARGER than the worst case (%u B)",
          (unsigned)small_s, (unsigned)big_s);

    FsdCapJsonButtons worst_b;
    fsd_cap_json_worst_buttons(&worst_b);
    const size_t big_b = fsd_cap_json_buttons(g_buf, sizeof(g_buf), &worst_b);

    FsdCapJsonButtons real_b;
    memset(&real_b, 0, sizeof(real_b));
    real_b.connected = true; real_b.verified = true;
    real_b.slots = 5u; real_b.bound_count = 5u;
    for (uint8_t i = 0; i < 5u; i++) {
        real_b.bound[i].slot = i;
        memcpy(real_b.bound[i].addr, "2f:95:e6:54:51:84", 18);
        real_b.bound[i].connected = true;
        real_b.bound[i].decoded = 255u;
    }
    for (uint8_t r = 0; r < FSD_CAP_JSON_ROWS; r++) real_b.rows[r] = 255u;
    real_b.act = 1073741823u;
    const size_t small_b = fsd_cap_json_buttons(g_buf, sizeof(g_buf), &real_b);

    CHECK(small_b > 0u, "a realistic buttons rendered nothing");
    CHECK(small_b <= big_b, "a realistic buttons (%u B) is LARGER than the worst case (%u B)",
          (unsigned)small_b, (unsigned)big_b);
}

/* ── the packed verdict word ─────────────────────────────────────────────────
 *
 * 🔴 EVERY FIELD PINNED SEPARATELY, and on purpose. A single combined vector
 * would let a shifted field pass as long as the total matched — that is exactly
 * how the J6 tap anchors ended up half-tested (either coordinate alone rejected
 * the only negative vector, so half of each comparison could be deleted). Here
 * each field is set alone, so moving any one of them fails its own assertion.
 */
static void test_each_verdict_sits_at_its_own_bits(void) {
    struct { const char* name; size_t off; uint8_t shift; } f[] = {
        { "nag_killer",     offsetof(FsdCapJsonBus, nag_killer),     FSD_CAP_V_NAG_KILLER_SHIFT },
        { "ap_first",       offsetof(FsdCapJsonBus, ap_first),       FSD_CAP_V_AP_FIRST_SHIFT },
        { "fsd_activation", offsetof(FsdCapJsonBus, fsd_activation), FSD_CAP_V_FSD_ACTIVATION_SHIFT },
        { "soft_engage",    offsetof(FsdCapJsonBus, soft_engage),    FSD_CAP_V_SOFT_ENGAGE_SHIFT },
        { "body_control",   offsetof(FsdCapJsonBus, body_control),   FSD_CAP_V_BODY_CONTROL_SHIFT },
        { "scroll_profile", offsetof(FsdCapJsonBus, scroll_profile), FSD_CAP_V_SCROLL_PROFILE_SHIFT },
    };

    for (size_t i = 0; i < sizeof(f) / sizeof(f[0]); i++) {
        for (uint8_t val = 0; val <= 2u; val++) {
            FsdCapJsonBus b;
            memset(&b, 0, sizeof(b));
            *((uint8_t*)((char*)&b + f[i].off)) = val;
            const uint32_t v = fsd_cap_json_pack_verdicts(&b);
            CHECK(v == ((uint32_t)val << f[i].shift),
                  "%s=%u packed to %lu, expected %lu — bit positions moved",
                  f[i].name, val, (unsigned long)v, (unsigned long)((uint32_t)val << f[i].shift));
        }
    }

    FsdCapJsonBus hw;
    memset(&hw, 0, sizeof(hw));
    hw.hw_unconfirmed = true;
    CHECK(fsd_cap_json_pack_verdicts(&hw) == (1u << FSD_CAP_V_HW_UNCONFIRMED_SHIFT),
          "hw_unconfirmed is not on bit %u", FSD_CAP_V_HW_UNCONFIRMED_SHIFT);

    for (uint8_t val = 0; val <= 2u; val++) {
        FsdCapJsonBus h;
        memset(&h, 0, sizeof(h));
        h.hint = val;
        CHECK(fsd_cap_json_pack_verdicts(&h) == ((uint32_t)val << FSD_CAP_V_HINT_SHIFT),
              "hint=%u is not at bit %u", val, FSD_CAP_V_HINT_SHIFT);
    }
}

/* A field wider than its slot must not bleed into its neighbour — a verdict
 * that overflowed would silently rewrite every field above it. */
static void test_an_out_of_range_value_cannot_reach_the_next_field(void) {
    FsdCapJsonBus b;
    memset(&b, 0, sizeof(b));
    b.nag_killer = 0xFFu;
    const uint32_t v = fsd_cap_json_pack_verdicts(&b);
    CHECK(v == (FSD_CAP_V_VERDICT_MASK << FSD_CAP_V_NAG_KILLER_SHIFT),
          "an out-of-range verdict reached beyond its two bits (got %lu)", (unsigned long)v);
}

/* ── the sink's contract ─────────────────────────────────────────────────── */

static void test_a_document_that_does_not_fit_yields_nothing(void) {
    FsdCapJsonStatus s;
    fsd_cap_json_worst_status(&s);
    char tight[64];
    const size_t n = fsd_cap_json_status(tight, sizeof(tight), &s);
    CHECK(n == 0u, "expected 0 for a status that does not fit, got %u", (unsigned)n);
    CHECK(tight[0] == '\0', "a status that did not fit left a half-written body");

    FsdCapJsonButtons b;
    fsd_cap_json_worst_buttons(&b);
    char tight2[64];
    const size_t m = fsd_cap_json_buttons(tight2, sizeof(tight2), &b);
    CHECK(m == 0u, "expected 0 for a buttons doc that does not fit, got %u", (unsigned)m);
    CHECK(tight2[0] == '\0', "a buttons doc that did not fit left a half-written body");
}

static void test_zero_capacity_and_null_are_safe(void) {
    FsdCapJsonStatus s;
    fsd_cap_json_worst_status(&s);
    CHECK(fsd_cap_json_status(NULL, 0u, &s) == 0u, "NULL/0 status should render nothing");
    CHECK(fsd_cap_json_status(g_buf, sizeof(g_buf), NULL) == 0u, "NULL input should render nothing");

    FsdCapJsonButtons b;
    fsd_cap_json_worst_buttons(&b);
    CHECK(fsd_cap_json_buttons(NULL, 0u, &b) == 0u, "NULL/0 buttons should render nothing");
    CHECK(fsd_cap_json_buttons(g_buf, sizeof(g_buf), NULL) == 0u, "NULL input should render nothing");
}

/* ── the shape the app parses ────────────────────────────────────────────── */

static void test_btn_is_always_sixty_hex_characters(void) {
    FsdCapJsonButtons b;
    memset(&b, 0, sizeof(b));
    for (uint8_t r = 0; r < FSD_CAP_JSON_ROWS; r++) b.rows[r] = (uint8_t)(r * 8u);

    const size_t n = fsd_cap_json_buttons(g_buf, sizeof(g_buf), &b);
    CHECK(n > 0u, "rendered nothing");

    const char* p = strstr(g_buf, "\"btn\":\"");
    CHECK(p != NULL, "no btn field");
    if (p == NULL) return;
    p += strlen("\"btn\":\"");
    const char* end = strchr(p, '"');
    CHECK(end != NULL, "btn field is unterminated");
    if (end == NULL) return;
    CHECK((size_t)(end - p) == FSD_CAP_JSON_ROWS * 2u,
          "btn is %u characters, expected %u",
          (unsigned)(end - p), (unsigned)(FSD_CAP_JSON_ROWS * 2u));
}

/* The row counts have to survive the hex round trip in order, or a press moves
 * a number on the wrong row and the screen names the wrong button. */
static void test_rows_render_in_order(void) {
    FsdCapJsonButtons b;
    memset(&b, 0, sizeof(b));
    b.rows[0]  = 0x00u;
    b.rows[1]  = 0x0Fu;
    b.rows[2]  = 0xA5u;
    b.rows[29] = 0xFFu;

    const size_t n = fsd_cap_json_buttons(g_buf, sizeof(g_buf), &b);
    CHECK(n > 0u, "rendered nothing");
    CHECK(strstr(g_buf, "\"btn\":\"000fa5") != NULL, "first three rows are wrong");
    CHECK(strstr(g_buf, "ff\",\"act\"") != NULL, "the last row is wrong");
}

/* No trailing comma before a closing brace or bracket — the document was
 * invalid JSON for a whole build once, and 455 perfectly good bytes were
 * thrown away by the parser with nothing to say why. */
static void test_no_trailing_separators(void) {
    FsdCapJsonStatus ws;
    fsd_cap_json_worst_status(&ws);
    CHECK(fsd_cap_json_status(g_buf, sizeof(g_buf), &ws) > 0u, "rendered nothing");
    CHECK(strstr(g_buf, ",}") == NULL, "status: a trailing comma before }");
    CHECK(strstr(g_buf, ",]") == NULL, "status: a trailing comma before ]");
    CHECK(strstr(g_buf, "[,") == NULL, "status: a leading comma after [");

    FsdCapJsonButtons wb;
    fsd_cap_json_worst_buttons(&wb);
    CHECK(fsd_cap_json_buttons(g_buf, sizeof(g_buf), &wb) > 0u, "rendered nothing");
    CHECK(strstr(g_buf, ",}") == NULL, "buttons: a trailing comma before }");
    CHECK(strstr(g_buf, ",]") == NULL, "buttons: a trailing comma before ]");
    CHECK(strstr(g_buf, "[,") == NULL, "buttons: a leading comma after [");

    /* Empty collections still have to be valid — a module with no bus traffic
     * and no remote bound is the state every board boots into. */
    FsdCapJsonStatus es;
    memset(&es, 0, sizeof(es));
    CHECK(fsd_cap_json_status(g_buf, sizeof(g_buf), &es) > 0u, "the empty status rendered nothing");
    CHECK(strstr(g_buf, "\"buses\":[]") != NULL, "an empty bus list is malformed");
    CHECK(strstr(g_buf, ",}") == NULL, "empty status: a trailing comma before }");

    FsdCapJsonButtons eb;
    memset(&eb, 0, sizeof(eb));
    CHECK(fsd_cap_json_buttons(g_buf, sizeof(g_buf), &eb) > 0u, "the empty buttons rendered nothing");
    CHECK(strstr(g_buf, "\"bound\":[]") != NULL, "an empty bound list is malformed");
    CHECK(strstr(g_buf, ",}") == NULL, "empty buttons: a trailing comma before }");
}

/* The buses each carry their own verdict word; one bus's values must not appear
 * on the other's line. */
static void test_buses_do_not_share_a_verdict_word(void) {
    FsdCapJsonStatus s;
    memset(&s, 0, sizeof(s));
    s.bus_count = 2u;
    memcpy(s.bus[0].name, "can0", 5);
    memcpy(s.bus[1].name, "can1", 5);
    s.bus[1].scroll_profile = 2u;   /* only can1 sees the scroll wheel */

    CHECK(fsd_cap_json_status(g_buf, sizeof(g_buf), &s) > 0u, "rendered nothing");
    CHECK(strstr(g_buf, "\"bus\":\"can0\",\"frames\":0,\"ids\":0,\"v\":0}") != NULL,
          "can0 picked up can1's verdicts: %s", g_buf);
    CHECK(strstr(g_buf, "\"bus\":\"can1\",\"frames\":0,\"ids\":0,\"v\":2048}") != NULL,
          "can1's verdict word is wrong: %s", g_buf);
}

int main(void) {
    test_worst_status_fits_the_att_limit();
    test_worst_buttons_fits_the_att_limit();
    test_worst_case_is_not_smaller_than_a_real_one();
    test_each_verdict_sits_at_its_own_bits();
    test_an_out_of_range_value_cannot_reach_the_next_field();
    test_a_document_that_does_not_fit_yields_nothing();
    test_zero_capacity_and_null_are_safe();
    test_btn_is_always_sixty_hex_characters();
    test_rows_render_in_order();
    test_no_trailing_separators();
    test_buses_do_not_share_a_verdict_word();
    report_realistic_sizes();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
