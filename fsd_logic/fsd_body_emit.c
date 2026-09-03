#include "fsd_body_emit.h"

#include <string.h>

bool fsd_emit_supported(FsdBodyAction action) {
    /* One action, and it matches the one armable row in fsd_body.c. Written as
     * a switch rather than `== FSD_ACT_MAP_LIGHT` so that adding an action to
     * the enum without deciding about it here is a compiler warning, not a
     * silent "no". */
    switch(action) {
    case FSD_ACT_MAP_LIGHT:
        return true;
    case FSD_ACT_DOOR_OPEN:      /* command never observed -- see 차량-캡처-2026-09-03 §5 */
    case FSD_ACT_CAMERA:
    case FSD_ACT_SEAT_DRIVER:
    case FSD_ACT_SEAT_PASSENGER:
    case FSD_ACT_SCROLL:
    case FSD_ACT_GEAR_D:
    case FSD_ACT_COUNT:
        return false;
    }
    return false;
}

FsdEmitResult fsd_emit_build(FsdBodyAction action, const FsdEmitTemplate* t,
                             uint32_t now_ms, FsdEmitFrame* out) {
    if(!t || !out) return FSD_EMIT_BAD_TEMPLATE;
    if(!fsd_emit_supported(action)) return FSD_EMIT_NO_ENCODING;

    /* Map light. The only one, so the checks below are written inline rather
     * than behind a table -- a one-row table reads like there are more. */
    if(!t->seen) return FSD_EMIT_NO_TEMPLATE;

    /* 🔴 The id is checked, not assumed. The caller hands us "the last frame we
     * saw"; if its plumbing ever feeds the wrong one we must not stamp our bit
     * into a stranger's payload and put it on the bus under this id. This
     * project has already shipped a parser that read the right BITS out of the
     * wrong FRAME (0x286 vs 0x118, PR #18) and it failed closed by luck. */
    if(t->id != FSD_EMIT_MAP_LIGHT_ID) return FSD_EMIT_BAD_TEMPLATE;
    if(t->dlc != FSD_EMIT_MAP_LIGHT_DLC) return FSD_EMIT_BAD_TEMPLATE;

    /* Unsigned subtraction so the millisecond clock wrapping past 2^32 does not
     * make a fresh template look ancient -- same idiom as fsd_autonomy.c. */
    if((uint32_t)(now_ms - t->seen_ms) >= FSD_EMIT_TEMPLATE_MAX_AGE_MS)
        return FSD_EMIT_STALE_TEMPLATE;

    memset(out, 0, sizeof(*out));
    out->id = FSD_EMIT_MAP_LIGHT_ID;
    out->dlc = FSD_EMIT_MAP_LIGHT_DLC;
    memcpy(out->data, t->data, FSD_EMIT_MAP_LIGHT_DLC);

    /* The whole command. Everything else is the car's own bytes, unchanged --
     * which is the point, and what the test compares against the real TSL
     * frame byte for byte. */
    out->data[FSD_EMIT_MAP_LIGHT_BYTE] |= FSD_EMIT_MAP_LIGHT_MASK;
    return FSD_EMIT_OK;
}

const char* fsd_emit_result_str(FsdEmitResult r) {
    switch(r) {
    case FSD_EMIT_OK: return "ok";
    case FSD_EMIT_NO_TEMPLATE: return "no template";
    case FSD_EMIT_STALE_TEMPLATE: return "stale template";
    case FSD_EMIT_BAD_TEMPLATE: return "bad template";
    case FSD_EMIT_NO_ENCODING: return "no encoding";
    }
    return "?";
}
