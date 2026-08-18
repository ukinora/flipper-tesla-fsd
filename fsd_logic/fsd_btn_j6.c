/*
 * fsd_btn_j6.c — see fsd_btn_j6.h. Bytes in, button out; no radio, no time
 * source of its own.
 */

#include "fsd_btn_j6.h"

#include <string.h>

void fsd_j6_init(FsdJ6* s) {
    if(!s) return;
    memset(s, 0, sizeof(*s));
}

static FsdJ6Out none(void) {
    FsdJ6Out o = {FSD_J6_NONE, FSD_J6_EDGE_NONE};
    return o;
}

static FsdJ6Out made(FsdJ6Btn b, FsdJ6Edge e) {
    FsdJ6Out o = {b, e};
    return o;
}

/* ── consumer control: a 3-byte bitmap, stateless ─────────────────────────── */

static FsdJ6Out decode_consumer(const uint8_t* b) {
    if(b[0] != 0x00u) return none(); // not this device's shape

    const uint32_t code =
        ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];

    switch(code) {
    case FSD_J6_C_B3_LONG: return made(FSD_J6_B3_LONG, FSD_J6_EDGE_PULSE);
    case FSD_J6_C_B4_LONG: return made(FSD_J6_B4_LONG, FSD_J6_EDGE_PULSE);
    case FSD_J6_C_B7_LONG: return made(FSD_J6_B7_LONG, FSD_J6_EDGE_PULSE);

    /* Named rather than left to the default, so that adding them looks like
     * what it is: undoing a decision, not filling a gap. See the header. */
    case FSD_J6_C_AMBIG_A: return none(); // 1번 길게 = 5번 톡
    case FSD_J6_C_AMBIG_B: return none(); // 2번 길게 = 5번 톡

    default: return none(); // 00 00 00 = 놓임, 그 밖은 모르는 코드
    }
}

/* ── digitiser: a contact that has to be watched from press to release ────── */

/* Where the contact ended up, relative to where it started. */
static FsdJ6Out classify(const FsdJ6* s, uint16_t x, uint16_t y) {
    const int32_t dx = (int32_t)x - (int32_t)s->x0;
    const int32_t dy = (int32_t)y - (int32_t)s->y0;
    const int32_t ax = (dx < 0) ? -dx : dx;
    const int32_t ay = (dy < 0) ? -dy : dy;

    /* Vertical first, and ties go to vertical. The two axes never compete in
     * practice — a swipe moves on one and holds the other — so the rule only
     * has to be deterministic. */
    if(ay >= (int32_t)FSD_J6_SWIPE_MIN && ay >= ax)
        return made((dy > 0) ? FSD_J6_B1 : FSD_J6_B2, FSD_J6_EDGE_PULSE);

    if(ax >= (int32_t)FSD_J6_SWIPE_MIN)
        return made((dx > 0) ? FSD_J6_B3 : FSD_J6_B4, FSD_J6_EDGE_PULSE);

    /* It did not travel, so it is a tap, and a tap's identity is WHERE IT
     * STARTED. Using the start rather than the end costs nothing and survives
     * a pixel of jitter on the release report.
     *
     * 6번 never reaches here — it is recognised at press time so that its hold
     * can be timed. */
    if(s->x0 == FSD_J6_B7_X && s->y0 == FSD_J6_B7_Y)
        return made(FSD_J6_B7, FSD_J6_EDGE_PULSE);

    return none(); // a tap somewhere we have not measured
}

static bool gesture_expired(const FsdJ6* s, uint32_t now_ms) {
    return !s->level && (uint32_t)(now_ms - s->down_ms) > FSD_J6_GESTURE_MAX_MS;
}

static FsdJ6Out decode_touch(FsdJ6* s, const uint8_t* b, uint32_t now_ms) {
    const uint8_t st = b[0];
    if(st != 0x00u && st != 0x03u) return none();

    const uint16_t x = (uint16_t)((uint16_t)b[1] | ((uint16_t)b[2] << 8));
    const uint16_t y = (uint16_t)((uint16_t)b[3] | ((uint16_t)b[4] << 8));
    if(x > FSD_J6_COORD_MAX || y > FSD_J6_COORD_MAX) return none();

    if(st == 0x03u) {
        if(s->down) {
            /* Mid-gesture. Either it is the same swipe still moving — nothing
             * to say — or the release was lost and this is a new gesture
             * starting on top of a corpse. */
            if(!gesture_expired(s, now_ms)) return none();
        }
        s->down = true;
        s->x0 = x;
        s->y0 = y;
        s->down_ms = now_ms;
        s->level = (x == FSD_J6_B6_X && y == FSD_J6_B6_Y);

        /* 🔴 The level button is announced HERE, not on release. Waiting for
         * the release would collapse a five-second hold into one instant and
         * there would be nothing left to time. */
        return s->level ? made(FSD_J6_B6, FSD_J6_EDGE_DOWN) : none();
    }

    /* st == 0x00 — the contact lifted. */
    if(!s->down) return none(); // the device repeats this row; it is not a second press

    s->down = false;
    if(s->level) {
        s->level = false;
        return made(FSD_J6_B6, FSD_J6_EDGE_UP);
    }
    if(gesture_expired(s, now_ms)) return none(); // whatever this was, it is stale

    return classify(s, x, y);
}

/* ── entry point ─────────────────────────────────────────────────────────── */

FsdJ6Out fsd_j6_feed(FsdJ6* s, const uint8_t* b, uint8_t len, uint32_t now_ms) {
    if(!s || !b) return none();
    if(len == 3u) return decode_consumer(b);
    if(len == 5u) return decode_touch(s, b, now_ms);
    return none(); // battery, service-changed, anything else
}

bool fsd_j6_is_level(FsdJ6Btn b) {
    return b == FSD_J6_B6;
}

const char* fsd_j6_name(FsdJ6Btn b) {
    /* ASCII: the serial console is ASCII throughout and this is read when
     * something is already wrong.
     *
     * 🔴 ONE WORD, NO SPACES, and that is not cosmetic. The caller prints this
     * next to fsd_btn_event_str(), so a name containing "long" produced
     *
     *     [BTN] 0 3 long short (0ms)
     *
     * on the bench — three words that a reader cannot split into "which button"
     * and "what happened". Keeping the name a single token, and separating the
     * two with an arrow at the call site, makes the line parseable by eye. */
    switch(b) {
    case FSD_J6_NONE: return "none";
    case FSD_J6_B1: return "btn1-down";
    case FSD_J6_B2: return "btn2-up";
    case FSD_J6_B3: return "btn3-right";
    case FSD_J6_B4: return "btn4-left";
    case FSD_J6_B6: return "btn6";
    case FSD_J6_B7: return "btn7";
    case FSD_J6_B3_LONG: return "btn3-hold";
    case FSD_J6_B4_LONG: return "btn4-hold";
    case FSD_J6_B7_LONG: return "btn7-hold";
    case FSD_J6_COUNT: break;
    }
    return "?";
}
