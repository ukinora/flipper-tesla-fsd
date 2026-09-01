/*
 * test_body_wire.c — host tests for the bit-granularity TX chokepoint.
 *
 * The assertions that matter most, in order:
 *
 *   - two actions sharing 0x3C2 cannot reach each other's bits. This is THE
 *     reason the file exists, and it is asserted mechanically over every pair
 *     rather than read off the table by eye.
 *   - an action whose command frame is unknown is refused, whatever the
 *     capability table says.
 *   - a bus we are not hearing is a bus we do not write to.
 *   - the multiplex cannot be crossed, in either the outgoing frame or the
 *     reference.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_body_wire.h"

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

// The car's 0x3C2 mux 0x29 frame, verbatim from 유휴-A.
static const uint8_t MUX29[8] = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40};
// The car's 0x3C2 mux 0 frame, verbatim from 운전석앞으로-A1.
static const uint8_t MUX00[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x65, 0x85};
// The car's 0x229, verbatim from 기어D-A1 at t=6.440.
static const uint8_t STALK[3] = {0xDD, 0x06, 0x00};

static FsdBodyRef ref_of(const uint8_t *d, uint8_t dlc, uint32_t ms) {
    FsdBodyRef r;
    memset(&r, 0, sizeof(r));
    r.seen = true;
    r.ms = ms;
    r.dlc = dlc;
    memcpy(r.data, d, dlc);
    return r;
}

/* ── the reason this file exists ─────────────────────────────────────────── */

// Camera, seat and scroll share one CAN ID. If any two payload masks touched,
// a bug in one emitter could reach another action's bits — which is exactly
// what frame-granularity refusal could not prevent.
static void test_masks_never_overlap(void) {
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const FsdBodyWire *x = fsd_body_wire((FsdBodyAction)a);
        if (!x) continue;
        for (int b = a + 1; b < FSD_ACT_COUNT; b++) {
            const FsdBodyWire *y = fsd_body_wire((FsdBodyAction)b);
            if (!y) continue;
            if (x->can_id != y->can_id) continue;
            // Different multiplexes of one ID are different frames entirely.
            if (x->mux_byte != FSD_BODY_WIRE_NO_MUX && x->mux_byte == y->mux_byte &&
                x->mux_value != y->mux_value)
                continue;
            for (int i = 0; i < FSD_BODY_WIRE_MAX_DLC; i++)
                CHECK((x->payload[i] & y->payload[i]) == 0,
                      "%s and %s both claim byte %d (0x%02X & 0x%02X)",
                      fsd_body_action_str((FsdBodyAction)a),
                      fsd_body_action_str((FsdBodyAction)b), i, x->payload[i],
                      y->payload[i]);
        }
    }

    // And the multiplex byte belongs to nobody, which is what keeps a camera
    // write from ever carrying the seat frame.
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const FsdBodyWire *w = fsd_body_wire((FsdBodyAction)a);
        if (!w || w->mux_byte == FSD_BODY_WIRE_NO_MUX) continue;
        CHECK((w->payload[w->mux_byte] & w->mux_mask) == 0,
              "%s must not be able to rewrite its own multiplex",
              fsd_body_action_str((FsdBodyAction)a));
    }
}

// An action whose command frame we never measured has no row, and no row means
// refused — independently of fsd_body_allows().
static void test_unknown_frames_have_no_row(void) {
    CHECK(fsd_body_wire(FSD_ACT_MAP_LIGHT) == NULL,
          "map light: 0x273 bit 59 correlates with two actions, so it selects neither");
    CHECK(fsd_body_wire(FSD_ACT_DOOR_OPEN) == NULL, "door: command frame unknown");
    CHECK(fsd_body_wire(FSD_ACT_COUNT) == NULL, "out of range is NULL");

    FsdBodyRef r = ref_of(MUX00, 8, 1000);
    CHECK(fsd_body_wire_check(FSD_ACT_MAP_LIGHT, 0x3C2u, MUX00, 8, &r, 1000) == FSD_WIRE_NO_ROW,
          "no row refuses even a frame that is byte-identical to the car's");

    // The passenger seat HAS a row but an empty payload: its frame is known,
    // its bits are not. It may change nothing.
    const FsdBodyWire *p = fsd_body_wire(FSD_ACT_SEAT_PASSENGER);
    CHECK(p != NULL, "passenger row exists");
    for (int i = 0; p && i < FSD_BODY_WIRE_MAX_DLC; i++)
        CHECK(p->payload[i] == 0, "passenger payload byte %d is empty (rules 2/11 uncaptured)", i);

    uint8_t out[8];
    memcpy(out, MUX00, 8);
    out[1] = 0x59; // the driver-seat value
    r = ref_of(MUX00, 8, 1000);
    CHECK(fsd_body_wire_check(FSD_ACT_SEAT_PASSENGER, 0x3C2u, out, 8, &r, 1000)
              == FSD_WIRE_OUT_OF_MASK,
          "the passenger action cannot borrow the driver seat's bits");
}

/* ── the reference ───────────────────────────────────────────────────────── */

static void test_no_reference_no_write(void) {
    uint8_t out[8];
    memcpy(out, MUX29, 8);
    out[6] |= 0x08u;

    FsdBodyRef r;
    memset(&r, 0, sizeof(r));
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_NO_REF,
          "a bus we have never heard is a bus we do not write to");

    r = ref_of(MUX29, 8, 1000);
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_OK,
          "a fresh reference passes");

    r = ref_of(MUX29, 8, 1000 - (FSD_BODY_WIRE_REF_FRESH_MS - 1));
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_OK,
          "one millisecond inside the window still passes");

    r = ref_of(MUX29, 8, 1000 - FSD_BODY_WIRE_REF_FRESH_MS);
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_REF_STALE,
          "exactly the window is already too old");
}

static void test_multiplex_cannot_be_crossed(void) {
    uint8_t out[8];
    FsdBodyRef r;

    // Outgoing frame carries the other variant.
    memcpy(out, MUX00, 8);
    r = ref_of(MUX29, 8, 1000);
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_WRONG_MUX,
          "camera cannot send the switch-pack variant");

    // Correct outgoing frame, wrong reference. Caught rather than obeyed.
    memcpy(out, MUX29, 8);
    out[6] |= 0x08u;
    r = ref_of(MUX00, 8, 1000);
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_REF_MUX,
          "the wrong reference is a refusal, not a comparison");

    // Seat is the other way round.
    memcpy(out, MUX29, 8);
    r = ref_of(MUX00, 8, 1000);
    CHECK(fsd_body_wire_check(FSD_ACT_SEAT_DRIVER, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_WRONG_MUX,
          "the seat cannot send the scroll variant");
}

/* ── per-action shape, against the captured bytes ────────────────────────── */

static void test_camera_toggle(void) {
    uint8_t out[8];
    FsdBodyRef r = ref_of(MUX29, 8, 1000);

    memcpy(out, MUX29, 8);
    out[6] = 0x08u; // exactly what TSL sent
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_OK,
          "the frame TSL actually sent passes");

    memcpy(out, MUX29, 8); // unchanged frame is also inside the mask
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_OK,
          "changing nothing is trivially inside the mask");

    memcpy(out, MUX29, 8);
    out[3] = 0x01u; // the scroll detent
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_OUT_OF_MASK,
          "🔴 the camera cannot move the speed profile — the whole point");

    memcpy(out, MUX29, 8);
    out[6] = 0x10u; // a neighbouring bit in its own byte
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 8, &r, 1000) == FSD_WIRE_OUT_OF_MASK,
          "one bit means one bit, not one byte");

    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x229u, out, 8, &r, 1000) == FSD_WIRE_WRONG_ID,
          "wrong frame refuses");
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, out, 7, &r, 1000) == FSD_WIRE_WRONG_DLC,
          "wrong length refuses");
}

static void test_seat_and_scroll(void) {
    uint8_t out[8];
    FsdBodyRef r00 = ref_of(MUX00, 8, 1000);
    FsdBodyRef r29 = ref_of(MUX29, 8, 1000);

    memcpy(out, MUX00, 8);
    out[1] = 0x59u; // forward, as captured
    CHECK(fsd_body_wire_check(FSD_ACT_SEAT_DRIVER, 0x3C2u, out, 8, &r00, 1000) == FSD_WIRE_OK,
          "seat forward, exactly as TSL sent it");
    out[1] = 0x56u; // back, as captured
    CHECK(fsd_body_wire_check(FSD_ACT_SEAT_DRIVER, 0x3C2u, out, 8, &r00, 1000) == FSD_WIRE_OK,
          "seat back — one action, direction is an argument");

    memcpy(out, MUX00, 8);
    out[6] = 0x69u; // buckle switch: a safety INPUT
    CHECK(fsd_body_wire_check(FSD_ACT_SEAT_DRIVER, 0x3C2u, out, 8, &r00, 1000)
              == FSD_WIRE_OUT_OF_MASK,
          "🔴 the seat cannot restate whether someone is belted");

    memcpy(out, MUX29, 8);
    out[3] = 0x3Fu; // -1 detent
    CHECK(fsd_body_wire_check(FSD_ACT_SCROLL, 0x3C2u, out, 8, &r29, 1000) == FSD_WIRE_OK,
          "one detent down");
    out[3] = 0x08u; // +8, a fast roll
    CHECK(fsd_body_wire_check(FSD_ACT_SCROLL, 0x3C2u, out, 8, &r29, 1000) == FSD_WIRE_OK,
          "detents accumulate, so a burst is one frame");
    out[3] = 0x40u; // above the 6-bit field
    CHECK(fsd_body_wire_check(FSD_ACT_SCROLL, 0x3C2u, out, 8, &r29, 1000) == FSD_WIRE_OUT_OF_MASK,
          "bits 6-7 of that byte were zero in every capture and are not ours");

    memcpy(out, MUX29, 8);
    out[6] = 0x08u; // the camera bit
    CHECK(fsd_body_wire_check(FSD_ACT_SCROLL, 0x3C2u, out, 8, &r29, 1000) == FSD_WIRE_OUT_OF_MASK,
          "🔴 the scroll emitter cannot kill the dashcam — the whole point");
}

static void test_gear_is_isolated_by_its_frame(void) {
    uint8_t out[3];
    FsdBodyRef r = ref_of(STALK, 3, 1000);

    memcpy(out, STALK, 3);
    out[0] = 0x08u;
    out[1] = 0x47u; // command 4, counter 7 = the car's next — as captured
    CHECK(fsd_body_wire_check(FSD_ACT_GEAR_D, 0x229u, out, 3, &r, 1000) == FSD_WIRE_OK,
          "the frame that actually shifted the car passes");

    // byte2 is the only frozen byte on this frame. It still has to hold.
    out[2] = 0x01u;
    CHECK(fsd_body_wire_check(FSD_ACT_GEAR_D, 0x229u, out, 3, &r, 1000) == FSD_WIRE_OUT_OF_MASK,
          "the one frozen byte on 0x229 is checked");

    // The mask really is wide here, and the test says so rather than implying
    // the check is stronger than it is.
    const FsdBodyWire *w = fsd_body_wire(FSD_ACT_GEAR_D);
    CHECK(w->payload[0] == 0xFFu && w->payload[1] == 0xFFu && w->payload[2] == 0x00u,
          "gear's mask is deliberately wide: CRC and counter cannot be compared");
    CHECK(w->mux_byte == FSD_BODY_WIRE_NO_MUX, "0x229 has no multiplex");

    CHECK(fsd_body_wire_check(FSD_ACT_SCROLL, 0x229u, out, 3, &r, 1000) == FSD_WIRE_WRONG_ID,
          "no other action may touch the gear frame");
}

// 🔴 The masks, written out by hand, from the capture.
//
// The exhaustive sweep below reads each mask FROM THE ROW IT IS TESTING, so it
// follows a mutation instead of catching it: widen a mask and the sweep widens
// its expectation with it. Proven, not theorised — widening the seat mask from
// the low nibble to the whole byte turned nothing red until this test existed.
//
// It is exactly the trap this repo hit with the J6 swipe threshold: a test that
// computed its vector from the constant it was supposed to pin. The fix is the
// same one. Changing a row now requires changing this table too, and that is
// the entire point of it.
static void test_masks_are_pinned(void) {
    static const struct {
        FsdBodyAction a;
        uint32_t id;
        uint8_t dlc;
        uint8_t payload[FSD_BODY_WIRE_MAX_DLC];
    } expect[] = {
        // camera: byte6 bit3, the one bit TSL toggled
        {FSD_ACT_CAMERA, 0x3C2u, 8, {0, 0, 0, 0, 0, 0, 0x08, 0}},
        // seat: byte1 LOW nibble only. The upper nibble is two more 2-bit
        // switch fields on the same pack — other buttons, not ours.
        {FSD_ACT_SEAT_DRIVER, 0x3C2u, 8, {0, 0x0F, 0, 0, 0, 0, 0, 0}},
        // passenger: frame known, bits not (TSL rules 2/11 uncaptured)
        {FSD_ACT_SEAT_PASSENGER, 0x3C2u, 8, {0, 0, 0, 0, 0, 0, 0, 0}},
        // scroll: byte3 bits 0-5, 6-bit signed. Bits 6-7 stayed 0 in every
        // capture and are not ours.
        {FSD_ACT_SCROLL, 0x3C2u, 8, {0, 0, 0, 0x3F, 0, 0, 0, 0}},
        // gear: CRC and (command|counter). Wide on purpose — neither byte can
        // be compared against the reference.
        {FSD_ACT_GEAR_D, 0x229u, 3, {0xFF, 0xFF, 0, 0, 0, 0, 0, 0}},
    };
    const int n = (int)(sizeof(expect) / sizeof(expect[0]));

    for (int k = 0; k < n; k++) {
        const FsdBodyWire *w = fsd_body_wire(expect[k].a);
        CHECK(w != NULL, "%s has a row", fsd_body_action_str(expect[k].a));
        if (!w) continue;
        CHECK(w->can_id == expect[k].id, "%s frame is 0x%03X",
              fsd_body_action_str(expect[k].a), (unsigned)expect[k].id);
        CHECK(w->dlc == expect[k].dlc, "%s length", fsd_body_action_str(expect[k].a));
        for (int i = 0; i < FSD_BODY_WIRE_MAX_DLC; i++)
            CHECK(w->payload[i] == expect[k].payload[i],
                  "%s byte %d mask: row 0x%02X, pinned 0x%02X",
                  fsd_body_action_str(expect[k].a), i, w->payload[i], expect[k].payload[i]);
    }

    // And the set of actions that HAVE a row is exactly the set pinned above,
    // so adding a row without pinning it is a failing test rather than an
    // unreviewed bit mask.
    int rows = 0;
    for (int a = 0; a < FSD_ACT_COUNT; a++)
        if (fsd_body_wire((FsdBodyAction)a)) rows++;
    CHECK(rows == n, "%d rows exist, %d pinned", rows, n);
}

// Hand-picked cases missed a real hole: widening the seat's mask from the low
// nibble to the whole byte turned no test red. The upper nibble of that byte is
// two more 2-bit switch fields (idle 0101 each) — other buttons on the same
// switch pack — so a wider mask would let the seat action press them.
//
// So do it exhaustively instead. For every row, every byte, every bit: flipping
// it must be allowed exactly when the payload mask says so. Widening any mask
// by one bit now turns this red.
static void test_every_bit_is_accounted_for(void) {
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const FsdBodyWire *w = fsd_body_wire((FsdBodyAction)a);
        if (!w) continue;

        const uint8_t *car =
            (w->can_id == 0x229u) ? STALK : (w->mux_value == 0x29u) ? MUX29 : MUX00;
        FsdBodyRef ref = ref_of(car, w->dlc, 1000);

        for (uint8_t i = 0; i < w->dlc; i++) {
            for (uint8_t b = 0; b < 8; b++) {
                uint8_t out[FSD_BODY_WIRE_MAX_DLC];
                memcpy(out, car, w->dlc);
                out[i] ^= (uint8_t)(1u << b);

                const FsdBodyWireVerdict v =
                    fsd_body_wire_check((FsdBodyAction)a, w->can_id, out, w->dlc, &ref, 1000);

                // Flipping a multiplex bit is caught earlier and by a more
                // specific name; either refusal is fine, an OK is not.
                const bool is_mux = (w->mux_byte != FSD_BODY_WIRE_NO_MUX && i == w->mux_byte &&
                                     (w->mux_mask & (1u << b)) != 0);
                const bool mine = (w->payload[i] & (1u << b)) != 0;

                if (mine) {
                    CHECK(v == FSD_WIRE_OK, "%s: byte %u bit %u is its own and must pass",
                          fsd_body_action_str((FsdBodyAction)a), i, b);
                } else if (is_mux) {
                    CHECK(v == FSD_WIRE_WRONG_MUX, "%s: byte %u bit %u is the multiplex",
                          fsd_body_action_str((FsdBodyAction)a), i, b);
                } else {
                    CHECK(v == FSD_WIRE_OUT_OF_MASK,
                          "%s: byte %u bit %u belongs to something else and must refuse",
                          fsd_body_action_str((FsdBodyAction)a), i, b);
                }
            }
        }
    }
}

static void test_verdicts_are_nameable(void) {
    for (int v = 0; v <= FSD_WIRE_OUT_OF_MASK; v++)
        CHECK(fsd_body_wire_verdict_str((FsdBodyWireVerdict)v)[0] != '?',
              "verdict %d has a name", v);

    FsdBodyRef r = ref_of(MUX29, 8, 1000);
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, NULL, 8, &r, 1000) == FSD_WIRE_BAD_ARGS,
          "NULL frame");
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, MUX29, 8, NULL, 1000) == FSD_WIRE_BAD_ARGS,
          "NULL reference");
    CHECK(fsd_body_wire_check(FSD_ACT_CAMERA, 0x3C2u, MUX29, 9, &r, 1000) == FSD_WIRE_BAD_ARGS,
          "a length this file cannot hold");
}

int main(void) {
    printf("test_body_wire\n");
    test_masks_never_overlap();
    test_unknown_frames_have_no_row();
    test_no_reference_no_write();
    test_multiplex_cannot_be_crossed();
    test_camera_toggle();
    test_seat_and_scroll();
    test_gear_is_isolated_by_its_frame();
    test_masks_are_pinned();
    test_every_bit_is_accounted_for();
    test_verdicts_are_nameable();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
