#include "fsd_body_emit.h"

#include <string.h>

bool fsd_emit_supported(FsdBodyAction action) {
    /* Four actions, and they match the four armable rows in fsd_body.c. Written
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
    case FSD_ACT_TURN_SIGNAL:
    case FSD_ACT_MIRROR:
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

/* Door selector -> (byte, mask). A switch, not a table lookup, so that adding
 * a door to FsdEmitDoor without measuring its field is a compiler warning here
 * rather than an index into whatever follows the array.
 *
 * 🔴 Neither output is written unless BOTH are known. A partial answer here is
 * a measured mask at an unmeasured offset, which is a different door. */
bool fsd_emit_door_field(int32_t door, uint8_t* byte_ix_out, uint8_t* bits_out) {
    if(!byte_ix_out || !bits_out) return false;
    switch((FsdEmitDoor)door) {
    case FSD_EMIT_DOOR_RIGHT_FRONT:
        *byte_ix_out = FSD_EMIT_DOOR_RF_BYTE;
        *bits_out = FSD_EMIT_DOOR_RF_BITS;
        return true;
    case FSD_EMIT_DOOR_RIGHT_REAR:
        *byte_ix_out = FSD_EMIT_DOOR_RR_BYTE;
        *bits_out = FSD_EMIT_DOOR_RR_BITS;
        return true;
    case FSD_EMIT_DOOR_LEFT_FRONT:
        *byte_ix_out = FSD_EMIT_DOOR_LF_BYTE;
        *bits_out = FSD_EMIT_DOOR_LF_BITS;
        return true;
    case FSD_EMIT_DOOR_LEFT_REAR:
        *byte_ix_out = FSD_EMIT_DOOR_LR_BYTE;
        *bits_out = FSD_EMIT_DOOR_LR_BITS;
        return true;
    case FSD_EMIT_DOOR_COUNT:
        break;
    }
    /* 🔴 Still the honest answer for anything past the fourth door. The frame
     * carries more than four fields -- the frunk switch is in its back bytes --
     * and "three bits apart" predicts a fifth at offset 17. Predicting is not
     * measuring, and this is the file where that distinction opens a door. */
    return false;
}

const char* fsd_emit_door_str(int32_t door) {
    switch((FsdEmitDoor)door) {
    case FSD_EMIT_DOOR_RIGHT_FRONT: return "right front";
    case FSD_EMIT_DOOR_RIGHT_REAR: return "right rear";
    case FSD_EMIT_DOOR_LEFT_FRONT: return "left front";
    case FSD_EMIT_DOOR_LEFT_REAR: return "left rear";
    case FSD_EMIT_DOOR_COUNT: break;
    }
    return "?";
}

/* Mirror selector -> byte3 field value. A switch for the same reason the door
 * is one. */
bool fsd_emit_mirror_bits(int32_t mirror, uint8_t* bits_out) {
    if(!bits_out) return false;
    switch((FsdEmitMirror)mirror) {
    case FSD_EMIT_MIRROR_FOLD:
        *bits_out = FSD_EMIT_MIRROR_FOLD_BITS;
        return true;
    case FSD_EMIT_MIRROR_UNFOLD:
        *bits_out = FSD_EMIT_MIRROR_UNFOLD_BITS;
        return true;
    case FSD_EMIT_MIRROR_COUNT:
        break;
    }
    /* 🔴 Includes 3, the value OR-ing the two directions would produce. The
     * field is two bits wide so it exists on the wire; it has never been seen,
     * and "the encoding allows it" is not "the car was asked for it". */
    return false;
}

const char* fsd_emit_mirror_str(int32_t mirror) {
    switch((FsdEmitMirror)mirror) {
    case FSD_EMIT_MIRROR_FOLD: return "fold";
    case FSD_EMIT_MIRROR_UNFOLD: return "unfold";
    case FSD_EMIT_MIRROR_COUNT: break;
    }
    return "?";
}

/* Turn-signal selector -> byte2 stalk field. A switch for the same reason the
 * door is one: an unmeasured selector must be a compiler warning here, not an
 * index into whatever follows. */
bool fsd_emit_turn_bits(int32_t turn, uint8_t* bits_out) {
    if(!bits_out) return false;
    switch((FsdEmitTurn)turn) {
    case FSD_EMIT_TURN_LEFT:
        *bits_out = FSD_EMIT_TURN_LEFT_BITS;
        return true;
    case FSD_EMIT_TURN_RIGHT:
        *bits_out = FSD_EMIT_TURN_RIGHT_BITS;
        return true;
    case FSD_EMIT_TURN_CANCEL:
        *bits_out = FSD_EMIT_TURN_CANCEL_BITS;
        return true;
    case FSD_EMIT_TURN_COUNT:
        break;
    }
    /* 🔴 Includes DOWN_1 (0x06), the other half tap. It is in every capture --
     * a person passes through it on the way to a left indicator -- but nobody
     * has sent it ALONE, so whether it cancels the way UP_1 does is unknown.
     * Seen is not measured. */
    return false;
}

const char* fsd_emit_turn_str(int32_t turn) {
    switch((FsdEmitTurn)turn) {
    case FSD_EMIT_TURN_LEFT: return "left";
    case FSD_EMIT_TURN_RIGHT: return "right";
    case FSD_EMIT_TURN_CANCEL: return "cancel";
    case FSD_EMIT_TURN_COUNT: break;
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

/* ── the stalk check table ───────────────────────────────────────────────────
 *
 * GENERATED. tools/derive_stalk_check.py reads every 0x249 frame in every
 * capture we hold and prints exactly these numbers. Do not edit by hand; run
 * the script, and if it disagrees with what is here then the captures changed
 * and this table is the thing that is wrong.
 *
 * byte0 = BASE[counter] ^ DELTA[stalk]. See the header for why a table and not
 * a formula, and for the leave-one-out cross-validation. */
static const uint8_t FSD_EMIT_TURN_CHECK_BASE[16] = {
    0x9Bu, 0xE8u, 0x2Au, 0xD3u,
    0xD3u, 0x83u, 0x4Cu, 0x5Eu,
    0x3Fu, 0x5Eu, 0xE2u, 0x28u,
    0x3Au, 0x13u, 0xAFu, 0xCEu,
};

/* Written as a switch, not an array indexed by the stalk value, so that a
 * stalk position nobody measured cannot reach the table at all. `false` here
 * and FSD_EMIT_NO_CHECK at the call site. */
static bool turn_check_delta(uint8_t stalk, uint8_t* delta_out) {
    switch(stalk) {
    case 0x00u: *delta_out = 0x00u; return true; /* idle */
    case 0x02u: *delta_out = 0x1Cu; return true; /* UP_1   -- cancel */
    case 0x04u: *delta_out = 0x38u; return true; /* UP_2   -- right */
    case 0x06u: *delta_out = 0x24u; return true; /* DOWN_1 */
    case 0x08u: *delta_out = 0x70u; return true; /* DOWN_2 -- left */
    default: return false;
    }
}

static FsdEmitResult emit_turn_signal(int32_t arg, const FsdEmitTemplate* t,
                                      uint32_t now_ms, FsdEmitFrame* out) {
    /* 🔴 The selector is judged BEFORE the template, same as the door. An
     * unmeasured direction must not get as far as "the template was stale",
     * which reads like something waiting would fix. */
    uint8_t stalk = 0;
    if(!fsd_emit_turn_bits(arg, &stalk)) return FSD_EMIT_NO_ENCODING;

    if(!t->seen) return FSD_EMIT_NO_TEMPLATE;
    if(t->id != FSD_EMIT_TURN_ID) return FSD_EMIT_BAD_TEMPLATE;
    if(t->dlc != FSD_EMIT_TURN_DLC) return FSD_EMIT_BAD_TEMPLATE;
    if((uint32_t)(now_ms - t->seen_ms) >= FSD_EMIT_TEMPLATE_MAX_AGE_MS)
        return FSD_EMIT_STALE_TEMPLATE;

    /* 🔴 THE EDGE OF THE MEASURED REGION, checked before anything is built.
     * Every frame the table was derived from had the high beams and the washer
     * idle, and the CRC covers both. A template with either one set is a frame
     * we have never seen the check byte for. */
    if(t->data[FSD_EMIT_TURN_CNT_BYTE] & FSD_EMIT_TURN_B1_UNKNOWN) return FSD_EMIT_NO_CHECK;
    if(t->data[FSD_EMIT_TURN_STALK_BYTE] & FSD_EMIT_TURN_B2_UNKNOWN) return FSD_EMIT_NO_CHECK;
    if(t->data[3] != 0u) return FSD_EMIT_NO_CHECK;

    uint8_t cnt = (uint8_t)((t->data[FSD_EMIT_TURN_CNT_BYTE] + 1u) & FSD_EMIT_TURN_CNT_MASK);

    uint8_t delta = 0;
    /* Cannot fail for the three selectors above; checked anyway, because the
     * day somebody adds one the failure has to be a refusal and not a frame
     * built out of an uninitialised byte. */
    if(!turn_check_delta(stalk, &delta)) return FSD_EMIT_NO_CHECK;

    memset(out, 0, sizeof(*out));
    out->id = FSD_EMIT_TURN_ID;
    out->dlc = FSD_EMIT_TURN_DLC;
    memcpy(out->data, t->data, FSD_EMIT_TURN_DLC);

    out->data[FSD_EMIT_TURN_CNT_BYTE] = cnt;
    /* Only our three bits. The rest of byte 2 is opendbc's leftStalkReserved1,
     * and the guard above has already established it is zero -- but writing the
     * whole byte would still be a claim about a field we do not read. */
    out->data[FSD_EMIT_TURN_STALK_BYTE] =
        (uint8_t)((out->data[FSD_EMIT_TURN_STALK_BYTE] & (uint8_t)~FSD_EMIT_TURN_STALK_MASK) |
                  (stalk & FSD_EMIT_TURN_STALK_MASK));

    /* Last, and from the bytes we just wrote -- not from the template's. */
    out->data[FSD_EMIT_TURN_CHK_BYTE] = (uint8_t)(FSD_EMIT_TURN_CHECK_BASE[cnt] ^ delta);
    return FSD_EMIT_OK;
}

FsdEmitResult fsd_emit_build(FsdBodyAction action, int32_t arg,
                             const FsdEmitTemplate* t, uint32_t now_ms,
                             FsdEmitFrame* out) {
    if(!t || !out) return FSD_EMIT_BAD_TEMPLATE;
    if(!fsd_emit_supported(action)) return FSD_EMIT_NO_ENCODING;

    /* Three of the five commands have the same shape -- copy the car's frame,
     * set one field, put it back on the same id -- so the id, length, byte and
     * field live in five variables and the checks below are written once.
     *
     * The other two do NOT fit that shape and take their own branches. 0x3E9
     * and 0x249 both carry a counter and a check field, so the frame is not a
     * copy with a bit set, it is a copy REWRITTEN -- and even those two differ,
     * because one check is a formula and the other is a lookup. Kept separate
     * so nobody has to read the shared path wondering which of its steps
     * apply. */
    /* Hazards take no argument. Ignored, not refused -- see the header. */
    if(action == FSD_ACT_HAZARDS) return emit_hazards(t, now_ms, out);
    if(action == FSD_ACT_TURN_SIGNAL) return emit_turn_signal(arg, t, now_ms, out);

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
    /* 🔴 A MASK AND A VALUE, NOT ONE SET OF BITS, since 2026-09-07.
     *
     * This used to be a single `bits` OR-ed into the byte, which is right for
     * a flag and wrong for a small enumerated field. The mirror is the second
     * kind: byte 3 holds 1 for fold and 2 for unfold, so OR-ing 2 onto a
     * template that already reads 1 yields 3 -- unmeasured, on a frame that
     * also carries the locks, the wipers and the horn.
     *
     * 🟢 Nothing changed for the two that were here first: the map light and
     * every door pass mask == value, and clearing a field before setting it to
     * the same bits is the frame the OR produced. The tests that compare
     * against TSL's captured bytes are unchanged and still pass, which is the
     * point of writing them against the car instead of against the code. */
    uint32_t want_id = 0;
    uint8_t want_dlc = 0, byte_ix = 0, mask = 0, value = 0;
    switch(action) {
    case FSD_ACT_DOOR_OPEN:
        /* 🔴 The ONLY place a door is chosen, and it refuses before it knows
         * the template is good. An unmeasured selector must not get as far as
         * "the template was stale" -- that reads like a retryable problem, and
         * this one is not: no amount of waiting will make us know which bits
         * open the driver's door. */
        if(!fsd_emit_door_field(arg, &byte_ix, &mask)) return FSD_EMIT_NO_ENCODING;
        /* The door's field IS its value: 3 in a 3-bit field, and the two bits
         * that carry it are the mask. Written out so that the day a door needs
         * a different value the two stop being the same variable. */
        value = mask;
        want_id = FSD_EMIT_DOOR_ID;
        want_dlc = FSD_EMIT_DOOR_DLC;
        break;
    case FSD_ACT_MIRROR:
        /* Judged before the template, same as the door and the stalk: an
         * unmeasured direction must not get as far as "the template was
         * stale", which reads like something waiting would fix. */
        if(!fsd_emit_mirror_bits(arg, &value)) return FSD_EMIT_NO_ENCODING;
        want_id = FSD_EMIT_MIRROR_ID;
        want_dlc = FSD_EMIT_MIRROR_DLC;
        byte_ix = FSD_EMIT_MIRROR_BYTE;
        mask = FSD_EMIT_MIRROR_MASK;
        break;
    case FSD_ACT_MAP_LIGHT:
        /* Takes no argument. Ignored, not refused -- see the header. */
        want_id = FSD_EMIT_MAP_LIGHT_ID;
        want_dlc = FSD_EMIT_MAP_LIGHT_DLC;
        byte_ix = FSD_EMIT_MAP_LIGHT_BYTE;
        mask = FSD_EMIT_MAP_LIGHT_MASK;
        value = FSD_EMIT_MAP_LIGHT_MASK;
        break;
    case FSD_ACT_HAZARDS:     /* returned above; listed so the switch is complete */
    case FSD_ACT_TURN_SIGNAL: /* likewise */
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

    /* The byte index is a variable now that the door carries its own -- so it
     * is bounded here rather than by inspection. Every value in the table is 0
     * or 1 against a dlc of 8, so this cannot fire today; it is here because
     * the day a short frame gets an emitter, the failure would be a write past
     * the end of out->data rather than a refusal. */
    if(byte_ix >= want_dlc) return FSD_EMIT_NO_ENCODING;

    /* A value that does not fit its own field is a table error, and the frame
     * it would build claims bits belonging to something else on the same
     * byte. Refused rather than truncated: a silently narrowed command is a
     * command that means something different. */
    if((uint8_t)(value & (uint8_t)~mask) != 0u) return FSD_EMIT_NO_ENCODING;

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
     * frames byte for byte, for all three commands. Clear the field, then set
     * it: for the light and the doors mask == value so this is the byte the
     * old OR produced, and for the mirror it is the difference between 2 and
     * 3. */
    out->data[byte_ix] =
        (uint8_t)((out->data[byte_ix] & (uint8_t)~mask) | value);
    return FSD_EMIT_OK;
}

/* See the header. Measured only; 1 is the "not measured" value. */
uint8_t fsd_emit_repeat(FsdBodyAction a) {
    if(a == FSD_ACT_TURN_SIGNAL) return 4u;
    return 1u;
}

const char* fsd_emit_result_str(FsdEmitResult r) {
    switch(r) {
    case FSD_EMIT_OK: return "ok";
    case FSD_EMIT_NO_TEMPLATE: return "no template";
    case FSD_EMIT_STALE_TEMPLATE: return "stale template";
    case FSD_EMIT_BAD_TEMPLATE: return "bad template";
    case FSD_EMIT_NO_ENCODING: return "no encoding";
    case FSD_EMIT_NO_CHECK: return "check byte unmeasured here";
    }
    return "?";
}
