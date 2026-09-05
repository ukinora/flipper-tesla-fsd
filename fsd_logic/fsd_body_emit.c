#include "fsd_body_emit.h"

#include <string.h>

bool fsd_emit_supported(FsdBodyAction action) {
    /* Three actions, and they match the three armable rows in fsd_body.c. Written
     * as a switch rather than a comparison so that adding an action to the enum
     * without deciding about it here is a compiler warning, not a silent "no".
     *
     * A host test asserts this function and armable_at_runtime agree for every
     * action. That is deliberate: it makes "we can build the frame" and "the
     * axis will let it through" one decision instead of two that drift. */
    switch(action) {
    case FSD_ACT_MAP_LIGHT:
    case FSD_ACT_DOOR_OPEN:
    case FSD_ACT_HAZARDS:
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

/* Door selector -> byte1 value. A switch, not a table lookup, so that adding
 * a door to FsdEmitDoor without measuring its bits is a compiler warning here
 * rather than an index into whatever follows the array. */
bool fsd_emit_door_bits(int32_t door, uint8_t* bits_out) {
    if(!bits_out) return false;
    switch((FsdEmitDoor)door) {
    case FSD_EMIT_DOOR_RIGHT_FRONT:
        *bits_out = FSD_EMIT_DOOR_RF_BITS;
        return true;
    case FSD_EMIT_DOOR_RIGHT_REAR:
        *bits_out = FSD_EMIT_DOOR_RR_BITS;
        return true;
    case FSD_EMIT_DOOR_COUNT:
        break;
    }
    /* 🔴 Includes the two LEFT doors. The bit pattern makes 0x0C and 0x30 look
     * obvious, and obvious is not measured. Guessing wrong here opens a door on
     * the other side of the car. */
    return false;
}

const char* fsd_emit_door_str(int32_t door) {
    switch((FsdEmitDoor)door) {
    case FSD_EMIT_DOOR_RIGHT_FRONT: return "right front";
    case FSD_EMIT_DOOR_RIGHT_REAR: return "right rear";
    case FSD_EMIT_DOOR_COUNT: break;
    }
    return "?";
}

/* byte 7 = (sum of bytes 0..6 + 0xEC) & 0xFF.
 *
 * Checked against every distinct 0x3E9 payload we hold -- 162, zero
 * exceptions -- rather than against the handful the hazard capture happened to
 * contain. A rule that fits four samples and a rule that fits 162 look the
 * same until the day they do not. */
static uint8_t hazard_check(const uint8_t* d) {
    unsigned sum = 0;
    for(unsigned i = 0; i < 7u; i++) sum += d[i];
    return (uint8_t)((sum + FSD_EMIT_HAZARD_SUM_ADD) & 0xFFu);
}

static FsdEmitResult emit_hazards(const FsdEmitTemplate* t, uint32_t now_ms,
                                  FsdEmitFrame* out) {
    if(!t->seen) return FSD_EMIT_NO_TEMPLATE;
    if(t->id != FSD_EMIT_HAZARD_ID) return FSD_EMIT_BAD_TEMPLATE;
    if(t->dlc != FSD_EMIT_HAZARD_DLC) return FSD_EMIT_BAD_TEMPLATE;
    if((uint32_t)(now_ms - t->seen_ms) >= FSD_EMIT_TEMPLATE_MAX_AGE_MS)
        return FSD_EMIT_STALE_TEMPLATE;

    /* 🔴 Staleness matters MORE here than for the other two. A copied
     * light frame that is a second old is still a valid light frame; a hazard
     * frame that is a second old carries a counter the car has already moved
     * past, and the receiver has every reason to drop it. The shared bound is
     * three of the car's own periods, which is the right order for a frame the
     * car sends about every 495 ms. */

    memset(out, 0, sizeof(*out));
    out->id = FSD_EMIT_HAZARD_ID;
    out->dlc = FSD_EMIT_HAZARD_DLC;
    memcpy(out->data, t->data, FSD_EMIT_HAZARD_DLC);

    out->data[FSD_EMIT_HAZARD_BYTE] |= FSD_EMIT_HAZARD_MASK;

    /* The counter is the HIGH nibble; the low nibble is not ours and is copied
     * through untouched (it is 0 or 2 in everything we have seen, and we do not
     * know which of those means what). */
    uint8_t cnt = (uint8_t)((out->data[FSD_EMIT_HAZARD_CNT_BYTE] >> 4) & 0x0Fu);
    cnt = (uint8_t)((cnt + 1u) & 0x0Fu);
    out->data[FSD_EMIT_HAZARD_CNT_BYTE] =
        (uint8_t)((cnt << 4) | (out->data[FSD_EMIT_HAZARD_CNT_BYTE] & 0x0Fu));

    /* Last, because the counter is inside the summed range. Doing it in the
     * other order produces a frame that looks right and checks wrong. */
    out->data[7] = hazard_check(out->data);
    return FSD_EMIT_OK;
}

FsdEmitResult fsd_emit_build(FsdBodyAction action, int32_t arg,
                             const FsdEmitTemplate* t, uint32_t now_ms,
                             FsdEmitFrame* out) {
    if(!t || !out) return FSD_EMIT_BAD_TEMPLATE;
    if(!fsd_emit_supported(action)) return FSD_EMIT_NO_ENCODING;

    /* Two of the three commands have the same shape -- copy the car's frame,
     * set one field, put it back on the same id -- so the id, length and bit
     * live in four variables and the checks below are written once.
     *
     * Hazards do NOT fit that shape and take their own branch: 0x3E9 carries a
     * counter and a check field, so the frame is not a copy with a bit set, it
     * is a copy REWRITTEN. Kept separate so nobody has to read the shared path
     * wondering which of its steps apply. */
    /* Hazards take no argument. Ignored, not refused -- see the header. */
    if(action == FSD_ACT_HAZARDS) return emit_hazards(t, now_ms, out);

    /* 🔴 NO `default:` HERE, AND THAT IS THE WHOLE POINT.
     *
     * fsd_emit_supported() above is written as a bare switch precisely so that
     * adding an action to the enum without deciding about it is a compiler
     * warning rather than a silent "no". This switch used to end in
     *
     *      default:   // fsd_emit_supported() already refused everything else
     *          want_id = FSD_EMIT_MAP_LIGHT_ID; ...
     *
     * which left the safety net one-sided: the day somebody builds the camera
     * emitter, adds FSD_ACT_CAMERA to fsd_emit_supported() and forgets this
     * switch, `default:` hands them the MAP LIGHT encoding and nothing
     * complains. With a per-action template that fails closed on the id check.
     * With the call shape this file actually recommends -- driven by reception,
     * "copy the frame that just arrived" -- a 0x273 arrives, the rule for the
     * new action fires, and we put a MAP LIGHT COMMAND on the bus and report
     * FSD_EMIT_OK. A wrong command that reports success is the failure this
     * whole file is written to avoid.
     *
     * That is not hypothetical: it is exactly the door today's hazard action
     * would have walked through, had its own branch above not been written by
     * hand first.
     *
     * The unreachable cases are listed rather than collapsed so the warning
     * fires. Initialised at the declaration only to keep -Wmaybe-uninitialized
     * quiet; the guard below is what actually stands there. */
    uint32_t want_id = 0;
    uint8_t want_dlc = 0, byte_ix = 0, bits = 0;
    switch(action) {
    case FSD_ACT_DOOR_OPEN:
        /* 🔴 The ONLY place a door is chosen, and it refuses before it knows
         * the template is good. An unmeasured selector must not get as far as
         * "the template was stale" -- that reads like a retryable problem, and
         * this one is not: no amount of waiting will make us know which bits
         * open the driver's door. */
        if(!fsd_emit_door_bits(arg, &bits)) return FSD_EMIT_NO_ENCODING;
        want_id = FSD_EMIT_DOOR_ID;
        want_dlc = FSD_EMIT_DOOR_DLC;
        byte_ix = FSD_EMIT_DOOR_BYTE;
        break;
    case FSD_ACT_MAP_LIGHT:
        /* Takes no argument. Ignored, not refused -- see the header. */
        want_id = FSD_EMIT_MAP_LIGHT_ID;
        want_dlc = FSD_EMIT_MAP_LIGHT_DLC;
        byte_ix = FSD_EMIT_MAP_LIGHT_BYTE;
        bits = FSD_EMIT_MAP_LIGHT_MASK;
        break;
    case FSD_ACT_HAZARDS: /* returned above; listed so the switch is complete */
    case FSD_ACT_CAMERA:
    case FSD_ACT_SEAT_DRIVER:
    case FSD_ACT_SEAT_PASSENGER:
    case FSD_ACT_SCROLL:
    case FSD_ACT_GEAR_D:
    case FSD_ACT_COUNT:
        return FSD_EMIT_NO_ENCODING;
    }

    /* Belt to the braces above: an action outside the enum cannot reach here
     * (fsd_emit_supported refuses it), but if one ever does it leaves without
     * an encoding instead of borrowing map light's. */
    if(want_id == 0) return FSD_EMIT_NO_ENCODING;

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
