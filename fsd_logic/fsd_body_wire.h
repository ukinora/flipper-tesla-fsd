#pragma once
/*
 * fsd_body_wire.h — the TX chokepoint, at BIT granularity.
 *
 * WHY THIS EXISTS
 * ---------------
 * The old chokepoint refused by CAN ID. That worked while the safety story was
 * "the only thing we ever write is a 0x3C2 scroll detent". The 2026-09-01
 * capture killed that sentence: camera (byte6 bit3) and seat (byte1 bits 0-3)
 * are the SAME FRAME as the scroll detent (byte3). So at frame granularity:
 *
 *   allow 0x3C2  -> one bug in the scroll emitter moves a seat or kills the
 *                   dashcam
 *   refuse 0x3C2 -> the scroll detent dies, and with it the owner's priority
 *                   feature
 *
 * Both answers are wrong, so the question has to change: not "may this action
 * write this frame" but "may this action have changed THESE BITS".
 *
 * 🔴 THE REFERENCE IS THE CAR'S OWN LAST FRAME, NOT A REST TABLE
 * --------------------------------------------------------------
 * 권한축-재설계.md §6 originally said we would build each frame from a fixed
 * "at rest" template and set only our own bits. Re-reading the capture says
 * that is the more dangerous of the two options, so this file does the other
 * one — the one TSL does.
 *
 * 0x3C2 mux 0 carries frontBuckleSwitch (48|2) and frontOccupancySwitch (50|2).
 * A rest template has to put SOME value in those bits, and any value we choose
 * is a claim about whether a person is belted and seated. Getting it wrong is
 * spoofing a safety input. Echoing the car's own most recent bytes cannot be
 * wrong in that way: it repeats what the car just said.
 *
 * So the rule is:
 *
 *      every bit outside this action's payload mask must equal
 *      the car's most recent frame of the same ID and multiplex
 *
 * and the consequences fall out of it:
 *   - the multiplex byte is outside every payload mask, so a camera write can
 *     never carry the seat frame's mux (and vice versa) -- for free.
 *   - two actions on one frame cannot reach each other's bits, because the
 *     masks do not overlap. That is asserted, not reviewed.
 *   - 🟢 WE CANNOT INJECT ONTO A BUS WE ARE NOT HEARING. No recent frame means
 *     no reference means refused. Fail-closed, and it needs no extra gate.
 *
 * WHAT THIS IS NOT
 * ----------------
 * Not permission. fsd_can_transmit() and fsd_body_allows() both have to say yes
 * first. This is the last denial before the wire, and it can only subtract.
 *
 * 🔴 THE ROWS ARE NOT VERIFIED YET
 * --------------------------------
 * The masks come from the 2026-09-01 capture -- bits TSL was observed changing.
 * Whether a frame WE build that way is accepted by the car is unmeasured; that
 * is step 3 of the redesign (first write test, camera toggle). Two actions have
 * no row at all because their command frame is unknown, and an action with no
 * row is refused here regardless of what the capability table says. Two
 * independent locks, on purpose.
 */

#include "fsd_body.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Longest frame this file handles. CAN classic. */
#define FSD_BODY_WIRE_MAX_DLC 8u

/* How old the car's frame may be and still serve as the reference. 0x3C2 mux 0
 * arrives at 20 Hz and 0x229 at 10 Hz, so 200 ms is several frames of slack
 * while still refusing on a bus that has gone quiet. */
#define FSD_BODY_WIRE_REF_FRESH_MS 200u

/* No multiplex on this frame. */
#define FSD_BODY_WIRE_NO_MUX 0xFFu

typedef struct {
    FsdBodyAction action; // must equal its own index; checked at runtime
    uint32_t can_id;
    uint8_t dlc;

    /* Which byte selects the multiplex, and the value this action's variant
     * carries. FSD_BODY_WIRE_NO_MUX in mux_byte means the frame has none.
     * Checked on BOTH the outgoing frame and the reference, so a caller that
     * hands us the wrong variant is caught rather than obeyed. */
    uint8_t mux_byte;
    uint8_t mux_mask;
    uint8_t mux_value;

    /* Bits this action is allowed to differ from the reference in. Everything
     * else must match exactly. Zero-filled means "may change nothing", which
     * is the tightest row — same convention as FsdBodyCaps. */
    uint8_t payload[FSD_BODY_WIRE_MAX_DLC];
} FsdBodyWire;

/* The car's most recent frame of the relevant ID and multiplex. Supplied by the
 * caller for the same reason FsdBodyInputs is: the predicate holds no state and
 * the host tests need no fixture. */
typedef struct {
    bool seen;
    uint32_t ms;
    uint8_t dlc;
    uint8_t data[FSD_BODY_WIRE_MAX_DLC];
} FsdBodyRef;

typedef enum {
    FSD_WIRE_OK = 0,
    FSD_WIRE_NO_ROW,      // this action has no measured frame — always refused
    FSD_WIRE_BAD_ARGS,    // NULL, or a row that drifted from its index
    FSD_WIRE_WRONG_ID,    // not the frame this action writes
    FSD_WIRE_WRONG_DLC,
    FSD_WIRE_WRONG_MUX,   // outgoing frame is the other variant
    FSD_WIRE_NO_REF,      // never heard the car send this — cannot build one
    FSD_WIRE_REF_STALE,   // heard it, but not recently enough
    FSD_WIRE_REF_MUX,     // the reference is the other variant
    FSD_WIRE_OUT_OF_MASK, // changed a bit that belongs to something else
} FsdBodyWireVerdict;

/** The row for an action, or NULL if it has none (its frame is unknown) or the
 *  index is out of range. */
const FsdBodyWire* fsd_body_wire(FsdBodyAction a);

/** May this exact frame go on the wire as this action?
 *
 *  Holds no state. Refuses on anything it cannot check, including a reference
 *  it does not have — a bus we are not hearing is a bus we do not write to. */
FsdBodyWireVerdict fsd_body_wire_check(FsdBodyAction a, uint32_t can_id, const uint8_t* out,
                                       uint8_t out_dlc, const FsdBodyRef* ref, uint32_t now_ms);

/** Human-readable verdict, for logs and the BLE surface. */
const char* fsd_body_wire_verdict_str(FsdBodyWireVerdict v);

#ifdef __cplusplus
}
#endif
