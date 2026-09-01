/*
 * fsd_body_wire.c — see fsd_body_wire.h. One predicate, no emitter.
 */

#include "fsd_body_wire.h"

/* The wire rows.
 *
 * Every mask below is a bit TSL was OBSERVED changing in the 2026-09-01
 * capture, decoded in 차량-캡처-2026-09-01.md §8. Nothing here is inferred from
 * a DBC, and nothing here has been transmitted yet.
 *
 * 🔴 Two actions are absent on purpose: MAP_LIGHT and DOOR_OPEN. Their command
 * frame is not known — 0x273 bit 59 correlates with both, which means it
 * selects neither. An action with no row is refused by this file no matter what
 * the capability table says. */
static const FsdBodyWire FSD_BODY_WIRES[] = {
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
