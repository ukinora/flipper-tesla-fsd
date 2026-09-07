/*
 * fsd_body_wire.c — see fsd_body_wire.h. One predicate, no emitter.
 */

#include "fsd_body_wire.h"

/* The wire rows.
 *
 * Every mask below is a bit TSL was OBSERVED changing in a capture -- the
 * 2026-09-01 one for the four original rows (차량-캡처-2026-09-01.md §8), the
 * 2026-09-05 afternoon one for the stalk (차량-캡처-2026-09-05-4차.md §5-②).
 * Nothing here is inferred from a DBC, and nothing here has been transmitted.
 *
 * 🔴🔴 EVERY ACTION HAS A ROW NOW (owner decision, 2026-09-07).
 *
 * Five were deliberately absent until today -- MAP_LIGHT, DOOR_OPEN, HAZARDS,
 * MIRROR, LIGHT_HORN. Their frames were all measured; what was missing was a
 * decision, and this file said so: "refused is the correct state until the
 * first write test in the car."
 *
 * ⚠️ THAT CONDITION IS NOT MET, AND THE OWNER OPENED THEM KNOWING IT.
 * The first car write went out on 2026-09-07 with Err=0 and the car did not
 * act; the fix (four frames instead of one) is verified on the bench and the
 * retest has not happened. That, and the door's residual risk below, were put
 * to the owner in those words and the answer was to open all five. Written
 * down because a row that opens on someone's say-so should record whose and on
 * what information -- the same discipline the capability table keeps.
 *
 * 🔴 WHAT NO MASK HERE CAN COVER. DOOR_OPEN swings a door OUTWARD and nothing
 * on this bus reports what is beside the car -- not a person, not a wall, not
 * a passing cyclist. These rows decide which BITS may move; they cannot decide
 * whether it is safe to move them. What stands in front of that is
 * fsd_body.c's row (standstill, park, driver present, drive session), the
 * session-only arm flag, and the owner having written the rule on purpose.
 *
 * 🔴 AND OPENING ALL OF THEM COST A LAYER. FSD_WIRE_NO_ROW is now unreachable
 * from fsd_pipe_one() and fsd_pipe_release(): every action in the enum has a
 * row, and an action OUTSIDE the enum is refused by the axis first
 * (FSD_BODY_UNKNOWN_ACTION). The check stays, because it is what greets the
 * NEXT action added to the enum -- but it is no longer a live gate, and a
 * comment implying otherwise would be the kind of thing this file exists to
 * prevent. test_body_wire.c pins it at the unit level instead.
 *
 * Every mask below is a bit TSL was OBSERVED changing. Nothing is inferred
 * from a DBC, and nothing here has been transmitted by us. */
static const FsdBodyWire FSD_BODY_WIRES[] = {
    [FSD_ACT_MAP_LIGHT] =
        {
            .action = FSD_ACT_MAP_LIGHT,
            /* 273#81E1000044023001 -> ..09. One bit, byte 7 bit 3 = bit 59.
             * Measured 2026-09-03; six capture files agree, and the frame
             * carries no counter and no checksum, so everything else is the
             * car's own bytes unchanged.
             *
             * 🔴 SHARES 0x273 WITH THE MIRROR. Two actions, one id, and the
             * masks must not touch -- test_masks_never_overlap() asserts that
             * rather than leaving it to review. */
            .can_id = 0x273u,
            .dlc = 8u,
            .mux_byte = FSD_BODY_WIRE_NO_MUX,
            .payload = {[7] = 0x08u},
        },

    [FSD_ACT_DOOR_OPEN] =
        {
            .action = FSD_ACT_DOOR_OPEN,
            /* 1F9#0000000000000000 -> one 3-bit field set to 3. Measured over
             * two visits, one door at a time:
             *
             *      left  front   6000...   3 << 5    byte 0
             *      right front   0003...   3 << 8    byte 1
             *      left  rear    0018...   3 << 11   byte 1
             *      right rear    00C0...   3 << 14   byte 1
             *
             * The mask is the UNION of all four, because WHICH door is the
             * emitter's ARGUMENT and not a separate permission -- the same
             * reasoning the seat row gives for covering both directions. A bug
             * that picks the wrong door is caught by fsd_emit_door_field(),
             * which refuses any selector nobody measured; it is not this row's
             * job and this row could not do it.
             *
             * 🔴 THE WIDEST-CONSEQUENCE ROW IN THIS TABLE, and this mask is
             * the narrowest part of what guards it. See the header. */
            .can_id = 0x1F9u,
            .dlc = 8u,
            .mux_byte = FSD_BODY_WIRE_NO_MUX,
            .payload = {[0] = 0x60u, [1] = 0xDBu},
        },

    [FSD_ACT_CAMERA] =
        {
            .action = FSD_ACT_CAMERA,
            /* 2955000000000040 -> 2955000000000840. One bit, and the same bit
             * for on and for off: the car treats it as a toggle. */
            .can_id = 0x3C2u,
            .dlc = 8u,
            .mux_byte = 0u,
            .mux_mask = 0xFFu,
            .mux_value = 0x29u,
            .payload = {[6] = 0x08u},
        },

    [FSD_ACT_SEAT_DRIVER] =
        {
            .action = FSD_ACT_SEAT_DRIVER,
            /* 0055555500006585 -> ..59.. (forward) and ..56.. (back). Two
             * 2-bit fields in one byte: bits 2-3 forward, bits 0-1 back, idle
             * value 1 in each. Both directions are one action, so the mask
             * covers both fields — direction is the emitter's argument, not a
             * separate permission. */
            .can_id = 0x3C2u,
            .dlc = 8u,
            .mux_byte = 0u,
            .mux_mask = 0xFFu,
            .mux_value = 0x00u,
            .payload = {[1] = 0x0Fu},
        },

    [FSD_ACT_SEAT_PASSENGER] =
        {
            .action = FSD_ACT_SEAT_PASSENGER,
            /* 🔴 NOT MEASURED. TSL rules 2 and 11 drive the passenger seat but
             * we did not capture them (no time in the car). The remaining four
             * bits of the same byte are the obvious candidate and that is
             * exactly why they are not written here: a guess in a wire table is
             * a guess that reaches the bus. Empty payload = may change nothing,
             * so this row refuses everything until rules 2/11 are captured. */
            .can_id = 0x3C2u,
            .dlc = 8u,
            .mux_byte = 0u,
            .mux_mask = 0xFFu,
            .mux_value = 0x00u,
        },

    [FSD_ACT_SCROLL] =
        {
            .action = FSD_ACT_SCROLL,
            /* byte3, 6-bit signed detent count. Measured: +1 = 0x01,
             * -1 = 0x3F, and a fast roll accumulates (+8 = 0x08, -9 = 0x37),
             * so one frame carries the whole movement. Bits 6-7 of that byte
             * are NOT ours — they stayed 0 in every capture. */
            .can_id = 0x3C2u,
            .dlc = 8u,
            .mux_byte = 0u,
            .mux_mask = 0xFFu,
            .mux_value = 0x29u,
            .payload = {[3] = 0x3Fu},
        },

    [FSD_ACT_GEAR_D] =
        {
            .action = FSD_ACT_GEAR_D,
            /* 229#DD0600 -> 084700. byte1 is (command << 4) | counter and the
             * counter must be the car's NEXT value, so it cannot be compared
             * against the reference — the whole byte is payload. byte0 is a
             * CRC over a frame that just changed, so likewise.
             *
             * That leaves byte2 as the only frozen byte, which is a weak check
             * on its own. It is not on its own: 0x229 belongs to no other
             * action, the ID check already isolates it, and every gate in
             * fsd_body.c stands in front. Written down rather than glossed
             * over, because a mask this wide should be visible. */
            .can_id = 0x229u,
            .dlc = 3u,
            .mux_byte = FSD_BODY_WIRE_NO_MUX,
            .payload = {[0] = 0xFFu, [1] = 0xFFu},
        },

    [FSD_ACT_HAZARDS] =
        {
            .action = FSD_ACT_HAZARDS,
            /* 3E9#F18802000000C027 -> F58802000000D03B. Measured 2026-09-05,
             * four consecutive injections while the car was in reverse.
             *
             * Same shape as 0x249 below: byte 6's HIGH nibble is a counter
             * that must be the car's NEXT value and byte 7 is a check over a
             * frame that just changed, so neither can be compared against the
             * reference and both are payload. byte 6's low nibble is NOT ours
             * -- it was 0 in everything we hold and the emitter copies it
             * through -- so the mask is 0xF0 rather than 0xFF.
             *
             * That leaves bytes 1..5 frozen plus the one bit that is the
             * command. Written out because a mask this wide should be visible
             * rather than glossed. */
            .can_id = 0x3E9u,
            .dlc = 8u,
            .mux_byte = FSD_BODY_WIRE_NO_MUX,
            .payload = {[0] = 0x04u, [6] = 0xF0u, [7] = 0xFFu},
        },

    [FSD_ACT_MIRROR] =
        {
            .action = FSD_ACT_MIRROR,
            /* 273#81E110000B023001 -> ..01.. (fold) and ..02.. (unfold).
             * Measured 2026-09-06; the car never sends a non-zero byte 3, and
             * both values arrive 1 ms behind a car frame.
             *
             * Two bits, because the field holds 1 or 2 and which one is the
             * emitter's argument. 🔴 Disjoint from the map light's byte 7 by
             * construction, and asserted by test_masks_never_overlap(). */
            .can_id = 0x273u,
            .dlc = 8u,
            .mux_byte = FSD_BODY_WIRE_NO_MUX,
            .payload = {[3] = 0x03u},
        },

    [FSD_ACT_LIGHT_HORN] =
        {
            .action = FSD_ACT_LIGHT_HORN,
            /* 3C2#0055555500006985 -> 04....., and back 12 ms later.
             * Measured 2026-09-06. One bit, byte 0 bit 2.
             *
             * 🔴 THE MULTIPLEX MASK IS 0x03 HERE AND 0xFF ON ITS NEIGHBOURS,
             * AND THAT IS NOT A TYPO. 0x3C2 selects its variant with byte 0
             * bits [1:0] -- and byte 0 also carries this action's own bit. With
             * a 0xFF mask our outgoing frame (byte 0 = 0x04) would fail this
             * row's own multiplex check, and the feature could not work at all.
             * The signal table hit the same wall and narrowed to 0x03 for the
             * same reason (fsd_signal.c, MUX_MASK).
             *
             * ⚠️ The camera, seat and scroll rows still use 0xFF. That fails
             * CLOSED -- they refuse a reference frame in which somebody is
             * pressing the horn or the hazard button -- so it is a missed write
             * rather than a wrong one, and narrowing them is a change with its
             * own measurement to do. Left alone deliberately.
             *
             * 🟢 The release frame differs from the reference in NOTHING, so it
             * passes this row by construction. That is what routing it through
             * here buys: nothing today, and everything the day the frame
             * changes shape. */
            .can_id = 0x3C2u,
            .dlc = 8u,
            .mux_byte = 0u,
            .mux_mask = 0x03u,
            .mux_value = 0x00u,
            .payload = {[0] = 0x04u},
        },

    [FSD_ACT_TURN_SIGNAL] =
        {
            .action = FSD_ACT_TURN_SIGNAL,
            /* 249#5E090000 -> 920A0800. Measured 2026-09-05 (4th visit).
             *
             * 🔴 THIS ROW IS THE WHOLE ANSWER TO "MAY WE SEND THE STALK
             * FRAME". The owner said yes to 0x249 on 2026-09-06, and the ID is
             * not the unit of that permission: 0x249 is the LEFT stalk, which
             * carries the high beams (12|2) and the washer/wiper (14|2)
             * alongside the indicator. Allowing the id would let a bug in the
             * indicator emitter flash the high beams at oncoming traffic or
             * start the wipers. Allowing three bits cannot.
             *
             * byte0 is the CRC over a frame that just changed and byte1 holds
             * the counter, which must be the car's NEXT value -- neither can be
             * compared against the reference, so both are payload. Same shape
             * as 0x229 above, and written out for the same reason: a mask this
             * wide should be visible rather than glossed.
             *
             * 🟢 What that leaves is exactly the check that matters. byte1's
             * high nibble is inside the payload mask, so this file does not
             * catch a high beam being flipped -- but fsd_body_emit.c refuses
             * to build such a frame at all (FSD_EMIT_NO_CHECK), because the
             * check table was never measured there. Two different mechanisms,
             * and the emitter's is the tighter one for once.
             *
             * byte2 is masked to bits [3:1], the indicator field. Bit 0 and
             * bits [7:4] are opendbc's leftStalkReserved1 and were zero in all
             * 51 payloads; they must equal the car's. byte3 likewise. */
            .can_id = 0x249u,
            .dlc = 4u,
            .mux_byte = FSD_BODY_WIRE_NO_MUX,
            .payload = {[0] = 0xFFu, [1] = 0xFFu, [2] = 0x0Eu},
        },
};

_Static_assert(sizeof(FSD_BODY_WIRES) / sizeof(FSD_BODY_WIRES[0]) <= FSD_ACT_COUNT,
               "a wire row landed past the end of the action enum");

const FsdBodyWire* fsd_body_wire(FsdBodyAction a) {
    if(a >= FSD_ACT_COUNT) return NULL;
    if((size_t)a >= sizeof(FSD_BODY_WIRES) / sizeof(FSD_BODY_WIRES[0])) return NULL;
    const FsdBodyWire* w = &FSD_BODY_WIRES[a];
    /* Designated initialisers leave the gaps zero-filled, and a zero row has
     * can_id 0 and action 0 — which would otherwise masquerade as row 0. Both
     * checks are needed: the action check catches a transposed row, the can_id
     * check catches an absent one. */
    if(w->can_id == 0u) return NULL;
    if(w->action != a) return NULL;
    return w;
}

static bool mux_matches(const FsdBodyWire* w, const uint8_t* d, uint8_t dlc) {
    if(w->mux_byte == FSD_BODY_WIRE_NO_MUX) return true;
    if(w->mux_byte >= dlc) return false;
    return (uint8_t)(d[w->mux_byte] & w->mux_mask) == w->mux_value;
}

FsdBodyWireVerdict fsd_body_wire_check(FsdBodyAction a, uint32_t can_id, const uint8_t* out,
                                       uint8_t out_dlc, const FsdBodyRef* ref, uint32_t now_ms) {
    if(!out || !ref) return FSD_WIRE_BAD_ARGS;
    if(out_dlc > FSD_BODY_WIRE_MAX_DLC) return FSD_WIRE_BAD_ARGS;

    const FsdBodyWire* w = fsd_body_wire(a);
    if(!w) return FSD_WIRE_NO_ROW;

    if(can_id != w->can_id) return FSD_WIRE_WRONG_ID;
    if(out_dlc != w->dlc) return FSD_WIRE_WRONG_DLC;
    if(!mux_matches(w, out, out_dlc)) return FSD_WIRE_WRONG_MUX;

    /* Fail-closed on a bus we are not hearing. This is not a nicety: without a
     * reference there is nothing to compare the frozen bits against, so the
     * only honest answer is no. */
    if(!ref->seen) return FSD_WIRE_NO_REF;
    if((uint32_t)(now_ms - ref->ms) >= FSD_BODY_WIRE_REF_FRESH_MS) return FSD_WIRE_REF_STALE;
    if(ref->dlc != w->dlc) return FSD_WIRE_WRONG_DLC;
    if(!mux_matches(w, ref->data, ref->dlc)) return FSD_WIRE_REF_MUX;

    /* The whole point, in one loop: outside your payload, be exactly what the
     * car just said. */
    for(uint8_t i = 0; i < w->dlc; i++) {
        const uint8_t frozen = (uint8_t)~w->payload[i];
        if((uint8_t)(out[i] & frozen) != (uint8_t)(ref->data[i] & frozen)) {
            return FSD_WIRE_OUT_OF_MASK;
        }
    }
    return FSD_WIRE_OK;
}

const char* fsd_body_wire_verdict_str(FsdBodyWireVerdict v) {
    switch(v) {
    case FSD_WIRE_OK: return "ok";
    case FSD_WIRE_NO_ROW: return "no wire row (frame unknown)";
    case FSD_WIRE_BAD_ARGS: return "bad args";
    case FSD_WIRE_WRONG_ID: return "wrong CAN id";
    case FSD_WIRE_WRONG_DLC: return "wrong length";
    case FSD_WIRE_WRONG_MUX: return "wrong multiplex";
    case FSD_WIRE_NO_REF: return "no reference frame";
    case FSD_WIRE_REF_STALE: return "reference stale";
    case FSD_WIRE_REF_MUX: return "reference is the other multiplex";
    case FSD_WIRE_OUT_OF_MASK: return "changed bits outside this action";
    }
    return "?";
}
