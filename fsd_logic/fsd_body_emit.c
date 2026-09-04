#include "fsd_body_emit.h"

#include <string.h>

bool fsd_emit_supported(FsdBodyAction action) {
    /* Two actions, and they match the two armable rows in fsd_body.c. Written
     * as a switch rather than a comparison so that adding an action to the enum
     * without deciding about it here is a compiler warning, not a silent "no".
     *
     * A host test asserts this function and armable_at_runtime agree for every
     * action. That is deliberate: it makes "we can build the frame" and "the
     * axis will let it through" one decision instead of two that drift. */
    switch(action) {
    case FSD_ACT_MAP_LIGHT:
    case FSD_ACT_DOOR_OPEN:
        return true;
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

    /* Both commands have the same shape -- copy the car's frame, set one field,
     * put it back on the same id -- so the id, length and bit live in three
     * variables and the checks below are written once. When a third command
     * does NOT fit this shape it gets its own branch rather than a fourth
     * variable; 0x3E9 (hazards) is already known to be that case, because it
     * carries a counter and a check field and cannot be copied at all. */
    uint32_t want_id;
    uint8_t want_dlc, byte_ix, bits;
    switch(action) {
    case FSD_ACT_DOOR_OPEN:
        want_id = FSD_EMIT_DOOR_ID;
        want_dlc = FSD_EMIT_DOOR_DLC;
        byte_ix = FSD_EMIT_DOOR_BYTE;
        bits = FSD_EMIT_DOOR_MASK;
        break;
    default: /* fsd_emit_supported() already refused everything else */
        want_id = FSD_EMIT_MAP_LIGHT_ID;
        want_dlc = FSD_EMIT_MAP_LIGHT_DLC;
        byte_ix = FSD_EMIT_MAP_LIGHT_BYTE;
        bits = FSD_EMIT_MAP_LIGHT_MASK;
        break;
    }

    if(!t->seen) return FSD_EMIT_NO_TEMPLATE;

    /* 🔴 The id is checked, not assumed. The caller hands us "the last frame we
     * saw"; if its plumbing ever feeds the wrong one we must not stamp our bit
     * into a stranger's payload and put it on the bus under this id. This
     * project has already shipped a parser that read the right BITS out of the
     * wrong FRAME (0x286 vs 0x118, PR #18) and it failed closed by luck. */
    if(t->id != want_id) return FSD_EMIT_BAD_TEMPLATE;
    if(t->dlc != want_dlc) return FSD_EMIT_BAD_TEMPLATE;

    /* Unsigned subtraction so the millisecond clock wrapping past 2^32 does not
     * make a fresh template look ancient -- same idiom as fsd_autonomy.c. */
    if((uint32_t)(now_ms - t->seen_ms) >= FSD_EMIT_TEMPLATE_MAX_AGE_MS)
        return FSD_EMIT_STALE_TEMPLATE;

    memset(out, 0, sizeof(*out));
    out->id = want_id;
    out->dlc = want_dlc;
    memcpy(out->data, t->data, want_dlc);

    /* The whole command. Everything else is the car's own bytes, unchanged --
     * which is the point, and what the tests compare against the real TSL
     * frames byte for byte, for both commands. */
    out->data[byte_ix] |= bits;
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
