/* fsd_body_emit — the emitter slot fsd_body.h leaves open.
 *
 *      rule engine     trigger -> action
 *          |
 *      authority axis  may this action happen right now   (fsd_body.c)
 *          |
 *      THIS FILE       builds the frame
 *          |
 *      TX chokepoint   the last denial
 *
 * 🔴 IT STILL TRANSMITS NOTHING. This file returns bytes; no caller anywhere
 * puts them on a bus. The first actual write is a decision to be made in the
 * car, with something reversible, and it is not made here.
 *
 *
 * WHY THIS EXISTS NOW
 * -------------------
 * fsd_body.h has said "THERE IS STILL NO EMITTER ANYWHERE ... no encoding table
 * exists" since the axis was written, and the reason was honest: we did not know
 * what TSL actually sends. On 2026-09-03 we found out for exactly one action.
 *
 *      captures/2026-09-03/맵등 켜기
 *
 *      car   0x273  81E1000044023001      every 500 ms
 *      TSL   0x273  81E1000044023009      0-1 ms later, the SAME bytes
 *                                          with one bit set
 *
 * Six files agree: the extra frame appears in the three captures where the map
 * lights came on and in none of the three where they did not. A whole-bus scan
 * of 279 ids found nothing else injected. See 차량-캡처-2026-09-03.md §2-3.
 *
 *
 * 🔴 WE COPY. WE DO NOT SYNTHESISE.
 * ---------------------------------
 * 0x273 is UI_vehicleControl: mirrors, locks, wipers, horn, seat heaters. The
 * only field we understand is bit 59. Every other bit is a live statement about
 * something else on the car, and we have no business inventing values for them.
 *
 * So the emitter needs the car's own most recent 0x273 as a template. Without
 * one it refuses. That is not a convenience — building a frame from zeros would
 * be asserting "mirrors folded, horn off, heaters off" on no evidence.
 *
 * ⚠️ This also means the emitter is DRIVEN BY RECEPTION. The natural call site
 * is the moment a 0x273 arrives: copy it, set the bit, send. That reproduces
 * TSL's 0-1 ms spacing for free, and it is why there is no timer here.
 *
 * 🟢 Copying is safe on this frame specifically because it carries NO counter
 * and NO checksum -- 20 consecutive car frames in the idle capture were byte
 * identical. A frame with a rolling counter could not be replayed this way, and
 * test_body_emit.c pins that observation so a future car that adds one shows up
 * as a failing test rather than as a command the car ignores.
 *
 *
 * ⚠️ WE INHERIT THE FLICKER
 * -------------------------
 * This method puts our frame and the car's own frame on the bus 1 ms apart,
 * both claiming the same field. TSL's own documentation warns in three places
 * that "some cars flicker" while it holds a map light. That is this, and doing
 * it the same way inherits it. Known, not discovered later.
 *
 * "Off" is not a command. Stop sending and the car's own 0x01 wins on its next
 * 500 ms tick. TSL's menu calls it 顶灯 关闭(跟随车机) -- "give control back to
 * the car" -- which is exactly what it is.
 */

#ifndef FSD_BODY_EMIT_H
#define FSD_BODY_EMIT_H

#include <stdbool.h>
#include <stdint.h>

#include "fsd_body.h"

#ifdef __cplusplus
extern "C" {
#endif

/* How old the template may be. One bound for all three ids, because all three
 * arrive at about the same rate -- 0x273 every 500 ms, 0x3E9 every 495 ms --
 * so this is three periods: enough to ride out one or two dropped frames, short
 * enough that the fields we copy still describe the car as it is now.
 *
 * 🔴 Staleness is not a transmission problem, it is a TRUTH problem. An old
 * template is an old statement about the mirrors and the horn. */
#define FSD_EMIT_TEMPLATE_MAX_AGE_MS 1500u

/* 0x273 UI_vehicleControl -- measured, see the header comment. */
/* THE DOOR-OPEN COMMAND. Measured 2026-09-05, third visit.
 *
 *      (8.127) 1F9#0000000000000000     <- the car
 *      (8.127) 1F9#0003000000000000     <- TSL, same millisecond
 *      (8.240)                          <- the right front door opens
 *
 * 🔴 0x1F9 IS IN NONE OF OUR FIVE DBCs. It has no name and we do not know
 * what the frame is for -- only what setting byte 1 to 0x03 does. Two visits
 * failed to catch it for exactly that reason: a filter assembled from ids we
 * can name cannot hold an id nobody has named.
 *
 * Across three unfiltered control captures the payload is all zeroes in 273
 * frames with no exception; in the door capture exactly two carry 0x03, 300 ms
 * apart, and the door moves between them.
 *
 * A template is required even though we have seen only zeroes. Knowing every
 * byte was zero on THIS car is not knowing what the other seven bytes mean,
 * and synthesising them would be a claim about fields we cannot read. Same
 * rule as 0x273, for a weaker-looking but identical reason. */
/* THE HAZARD COMMAND. Measured 2026-09-05, when TSL turned the hazards on by
 * itself because the car went into reverse -- a rule the owner had configured,
 * and a fact only the owner could supply. Without that sentence these two
 * injections were an unexplained pair.
 *
 *      (6.199) gear -> R
 *      (6.881) 3E9#F18802000000C027     <- the car,  counter 0xC
 *      (6.881) 3E9#F58802000000D03B     <- TSL, same ms, counter 0xD
 *
 * 🔴 THIS ONE CANNOT BE COPIED, AND THAT MAKES IT A DIFFERENT KIND OF
 * EMITTER FROM THE OTHER TWO. 0x273 and 0x1F9 carry no counter and no check, so
 * "copy the car's frame and set one field" produces a frame the car accepts.
 * 0x3E9 carries both. A copied frame is a stale counter and a wrong check, and
 * a receiver that validates either will drop it -- silently, which is the worst
 * way for a hazard command to fail.
 *
 * Both rules were derived from captures, not assumed:
 *
 *   COUNTER  byte 6, high nibble. TSL sends the car's value PLUS ONE, wrapping
 *            F -> 0. Four consecutive injections, all +1, no exception.
 *
 *   CHECK    byte 7 = (sum of bytes 0..6 + 0xEC) & 0xFF. Verified against
 *            EVERY distinct 0x3E9 payload in every capture we hold -- 162 of
 *            them, zero exceptions. The counter lives inside the summed range,
 *            which is why advancing it and recomputing the check is one step.
 *
 * The 0xEC is this frame's constant and nothing else's. It is not a CRC
 * polynomial and must not be reused for another id. */
#define FSD_EMIT_HAZARD_ID       0x3E9u
#define FSD_EMIT_HAZARD_BYTE     0u
#define FSD_EMIT_HAZARD_MASK     0x04u
#define FSD_EMIT_HAZARD_DLC      8u
#define FSD_EMIT_HAZARD_CNT_BYTE 6u
#define FSD_EMIT_HAZARD_SUM_ADD  0xECu

#define FSD_EMIT_DOOR_ID         0x1F9u
#define FSD_EMIT_DOOR_BYTE       1u
#define FSD_EMIT_DOOR_DLC        8u

/* WHICH DOOR. Measured, one door per visit:
 *
 *      right FRONT   byte1 = 0x03    3rd visit, 2026-09-05
 *      right REAR    byte1 = 0xC0    4th visit, 2026-09-05 afternoon
 *
 * The 4th visit caught the rear one TWICE in two different captures — TSL's
 * own menu entry, and the three-window-up gesture that fires the same rule —
 * and both produced 0xC0. That is the internal cross-check; neither reading
 * rests on the other.
 *
 *      (7.458) 1F9#0000000000000000     <- the car
 *      (7.459) 1F9#00C0000000000000     <- TSL, +1 ms
 *      (7.53x)                          <- 0x103 latch moves, 70-81 ms later
 *
 * ⚠️ TWO POINTS, AND EVERYTHING ELSE IS INFERENCE. 0x03 is bits[1:0] and 0xC0
 * is bits[7:6], so "four 2-bit fields, value 3 = open" fits both. It also fits
 * a plain bitmask. Either reading predicts 0x0C and 0x30 for the two LEFT
 * doors — and NEITHER HAS BEEN SEEN. TSL has no left-door rule, so no capture
 * can contain one.
 *
 * So the left doors are NOT in this enum. A guess here does not fail loudly:
 * it opens a door on the other side of the car, next to whatever is standing
 * there. FSD_EMIT_NO_ENCODING is the honest answer until somebody measures it.
 */
typedef enum {
    /* 0 is the right front, which is what every rule stored before this enum
     * existed already meant. A stored rule must not quietly change which door
     * it opens because the emitter learned a second one. */
    FSD_EMIT_DOOR_RIGHT_FRONT = 0,
    FSD_EMIT_DOOR_RIGHT_REAR = 1,
    FSD_EMIT_DOOR_COUNT,
} FsdEmitDoor;

#define FSD_EMIT_DOOR_RF_BITS    0x03u
#define FSD_EMIT_DOOR_RR_BITS    0xC0u

/** byte1 value for a door selector. False — *bits_out untouched — for a
 *  selector this car has never been measured to accept. */
bool fsd_emit_door_bits(int32_t door, uint8_t* bits_out);

/** Name for logs and the serial console. Never returns NULL; an unmeasured
 *  selector reads as "?" rather than as some door. */
const char* fsd_emit_door_str(int32_t door);

#define FSD_EMIT_MAP_LIGHT_ID    0x273u
#define FSD_EMIT_MAP_LIGHT_BYTE  7u
#define FSD_EMIT_MAP_LIGHT_MASK  0x08u   /* bit 3 of byte 7 = bit 59 */
#define FSD_EMIT_MAP_LIGHT_DLC   8u

typedef enum {
    FSD_EMIT_OK = 0,
    /** No frame for this action's id has been received; nothing to copy. */
    FSD_EMIT_NO_TEMPLATE,
    /** One was received but it is too old to still describe the car. */
    FSD_EMIT_STALE_TEMPLATE,
    /** The template is the wrong id or too short to hold the field. */
    FSD_EMIT_BAD_TEMPLATE,
    /** No encoding for this request. Either the action has no emitter yet
     *  (five of the eight), or it has one but the argument selects something
     *  nobody has measured — a door on the left, say. Both are gaps, not
     *  gates, and a gap must never be filled by guessing. */
    FSD_EMIT_NO_ENCODING,
} FsdEmitResult;

/** The car's most recent frame for the id this action writes. */
typedef struct {
    bool seen;
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t seen_ms;
} FsdEmitTemplate;

/** What to put on the bus. Filled only when the result is FSD_EMIT_OK. */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} FsdEmitFrame;

/**
 * Build the frame for one action, or say why not.
 *
 * `arg` is the rule's own argument (FsdRule.arg / FsdRuleDecision.arg), carried
 * here unchanged. Today only FSD_ACT_DOOR_OPEN reads it, as an FsdEmitDoor.
 *
 * 🔴 An action that takes no argument IGNORES it rather than refusing. That is
 * deliberate: rules stored before an action had an argument carry whatever was
 * in the field, and a refusal would break them for a value that means nothing.
 * The door is the opposite case — there 0 has always meant the right front, so
 * ignoring is exactly what keeps a stored rule pointing at the same door.
 *
 * 🔴 This answers "what would the bytes be", NOT "may this happen". The
 * authority axis (fsd_body_allows) is a separate question asked separately, and
 * this function deliberately does not call it -- a builder that also decides is
 * a builder nobody can test in isolation, and this one has to be testable
 * against bytes copied out of a real capture.
 *
 * Returns FSD_EMIT_NO_ENCODING for every action but FSD_ACT_MAP_LIGHT,
 * FSD_ACT_DOOR_OPEN and FSD_ACT_HAZARDS. That mirrors fsd_body.c, where those
 * three are the rows with armable_at_runtime = true -- two independent
 * statements of the same fact, so widening one without the other does nothing,
 * and a host test asserts they agree for every action.
 */
FsdEmitResult fsd_emit_build(FsdBodyAction action, int32_t arg,
                             const FsdEmitTemplate* t, uint32_t now_ms,
                             FsdEmitFrame* out);

/** Names for logs and the serial console. Never returns NULL. */
const char* fsd_emit_result_str(FsdEmitResult r);

/**
 * Does this action have an emitter at all?
 *
 * Exposed so a caller can tell "not allowed right now" from "we do not know how
 * to do this yet" without building a frame to find out. The two need different
 * words on a screen: one is a gate, the other is a gap.
 */
bool fsd_emit_supported(FsdBodyAction action);

#ifdef __cplusplus
}
#endif

#endif /* FSD_BODY_EMIT_H */
