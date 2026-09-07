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

/* How old the template may be. One bound for all four ids -- 0x273 every
 * 500 ms and 0x3E9 every 495 ms, so this is three of their periods: enough to
 * ride out one or two dropped frames, short enough that the fields we copy
 * still describe the car as it is now.
 *
 * ⚠️ 0x249 arrives every 50 ms, so for that frame the same bound is thirty
 * periods rather than three. Left alone deliberately: what a stale stalk
 * template costs is a stale COUNTER, and a receiver that checks the counter
 * drops the frame rather than misreading it. The bound is about the fields we
 * COPY still being true, and on 0x249 those are the high beams and the washer,
 * which do not change faster than the light on the ceiling.
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

/* THE TURN SIGNAL COMMAND. Measured 2026-09-05, fourth visit.
 *
 * 🔴 IT IS NOT 0x3E9, AND THAT PREDICTION WAS WRITTEN DOWN BEFORE THE
 * CAPTURE. The hazards live in 0x3E9 and the indicators looked like they must
 * too. They do not: TSL REPLAYS THE STALK. Across the three indicator captures
 * 0x3E9 byte 0 held at 0xF1 for 19-21 frames with its 495 ms period unbroken --
 * not one injection. So the same device drives the same lamps two different
 * ways, and the only reason we know which is which is that somebody looked.
 *
 *      (7.955) 249#5E090000     <- the car,  counter 9,  stalk 00
 *      (7.956) 249#920A0800     <- TSL, +1 ms, counter A, stalk 08
 *      (8.006) 249#580B0800     <- ... and again, 50 ms later
 *      (8.056) 249#4A0C0800
 *      (8.106) 249#630D0800     <- four frames, then it stops
 *      (8.133) 3F5#02000B38...  <- 27 ms later the left lamp is on
 *
 * The stalk field is byte 2, and it is the position TIMES TWO (17|3, one bit
 * left of where opendbc puts it -- see the DBC comment). A person's stalk goes
 * through detent 1 on the way to detent 2; TSL sends detent 2 directly:
 *
 *      person, right   02 02 04 04 04 04 04 04 00      UP_1 then UP_2
 *      person, left    06 06 08 08 08 08 00            DOWN_1 then DOWN_2
 *      TSL right       04 04 04                        UP_2
 *      TSL left        08 08 08 08                     DOWN_2
 *      TSL cancel      02 02 02                        UP_1, the half tap
 *
 * 🟢 Left-is-down and right-is-up is confirmed from OUTSIDE this frame: 0x3F5
 * lights its right pair after 04 and its left pair after 08.
 *
 * ⚠️ "Off" IS A COMMAND HERE, unlike the map light and the hazards. Ceasing to
 * send does not cancel an indicator -- the car latches it. TSL sends UP_1, the
 * half tap a driver uses to cancel. Whether DOWN_1 cancels too is unmeasured,
 * so it is not in the enum.
 */
#define FSD_EMIT_TURN_ID         0x249u
#define FSD_EMIT_TURN_DLC        4u
#define FSD_EMIT_TURN_CHK_BYTE   0u
#define FSD_EMIT_TURN_CNT_BYTE   1u
#define FSD_EMIT_TURN_STALK_BYTE 2u
/* SCCM_turnIndicatorStalkStatus, 17|3 = byte 2 bits [3:1]. The bits we own,
 * and the only ones we change. */
#define FSD_EMIT_TURN_STALK_MASK 0x0Eu

/* ---- WHAT WE MAY WRITE, AND WHERE THE CHECK TABLE STOPS -------------------
 *
 * 🔴 THIS FRAME CAN BE NEITHER COPIED NOR COMPUTED, WHICH MAKES IT A THIRD
 * KIND OF EMITTER.
 *
 *   0x273, 0x1F9    copy the car's frame, set a field.       No counter, no
 *                                                            check.
 *   0x3E9           copy, advance the counter, RECOMPUTE
 *                   the check from a formula we derived.
 *   0x249           copy, advance the counter, and LOOK THE
 *                   CHECK UP -- because there is no formula.
 *
 * byte 0 is SCCM_leftStalkCrc. The whole 8-bit CRC space (255 polynomials x
 * 256 seeds x 4 reflection combinations x 2 xorouts, over eight byte layouts)
 * was searched and nothing fits -- and nothing can, because two of the 51
 * distinct payloads we hold are
 *
 *      D3 03 00 00   and   D3 04 00 00      different counter, same check
 *      5E 07 00 00   and   5E 09 00 00      likewise
 *
 * A check that is a function of the bytes cannot do that. So we do not derive
 * it; we carry what the car was measured to send.
 *
 * 🟢 BUT NOT AS 51 PAIRS. The observations factor exactly:
 *
 *      byte0 = BASE[counter] ^ DELTA[stalk]
 *
 * 16 + 4 = 20 numbers explaining 51 observations with no exception, and the
 * BASE row for every one of the 16 counters comes from an IDLE frame, so it is
 * derived without reference to any command. tools/derive_stalk_check.py
 * regenerates both tables and cross-validates them leave-one-out: take DELTA
 * from a single counter, predict every other counter. 300 out-of-sample
 * predictions, zero misses.
 *
 * 🔴 WHY THAT MATTERS RATHER THAN BEING TIDY. The emitter does not get to
 * choose its counter -- it is whatever the car last sent, plus one -- so which
 * pair we need is decided by the millisecond the rule fires. A literal table of
 * the 51 observed pairs covers 29 of the 48 (counter, stalk) combinations the
 * three commands need. The indicator would work about 60% of the time, at
 * random, with nothing on screen to explain the other 40%.
 *
 * 🔴 THE TABLE STILL HAS AN EDGE, AND IT IS REAL. Every frame we hold was
 * captured with the high beams and the washer idle. Those live in the same
 * byte as the counter (highBeamStalkStatus 12|2, washWipeButtonStatus 14|2)
 * and the CRC covers them, so a template with either one set is outside
 * everything we measured. FSD_EMIT_NO_CHECK, not a guess: flash the high beams
 * while an indicator rule fires and the command is refused with a name.
 */
#define FSD_EMIT_TURN_CNT_MASK   0x0Fu  /* SCCM_leftStalkCounter, 8|4 */
/* Bits that must be zero in the template for the check table to apply. byte 1
 * outside the counter is the high beam and washer; byte 2 outside our field is
 * opendbc's leftStalkReserved1; byte 3 was zero in all 51. */
#define FSD_EMIT_TURN_B1_UNKNOWN 0xF0u
#define FSD_EMIT_TURN_B2_UNKNOWN 0xF1u

/** Which way, or cancel. The raw byte-2 values are deliberately NOT the enum
 *  values: a rule's stored arg must keep meaning the same thing if the field
 *  encoding is ever re-read, and 0 must not be a live command by accident. */
typedef enum {
    /* 0 is LEFT because a rule stored with arg 0 -- which is what the app
     * writes when it has never offered a choice -- has to mean something
     * measured. All three of these are measured; none is a fallback. */
    FSD_EMIT_TURN_LEFT = 0,
    FSD_EMIT_TURN_RIGHT = 1,
    FSD_EMIT_TURN_CANCEL = 2,
    FSD_EMIT_TURN_COUNT,
} FsdEmitTurn;

#define FSD_EMIT_TURN_LEFT_BITS   0x08u /* DOWN_2 */
#define FSD_EMIT_TURN_RIGHT_BITS  0x04u /* UP_2 */
#define FSD_EMIT_TURN_CANCEL_BITS 0x02u /* UP_1, the half tap */

/** byte2 stalk field for a turn-signal selector. False — *bits_out untouched —
 *  for a selector this car has never been measured to accept. */
bool fsd_emit_turn_bits(int32_t turn, uint8_t* bits_out);

/** Name for logs and the serial console. Never returns NULL. */
const char* fsd_emit_turn_str(int32_t turn);

typedef enum {
    FSD_EMIT_OK = 0,
/** No frame for this action's id has been received; nothing to copy. */
    FSD_EMIT_NO_TEMPLATE,
    /** One was received but it is too old to still describe the car. */
    FSD_EMIT_STALE_TEMPLATE,
    /** The template is the wrong id or too short to hold the field. */
    FSD_EMIT_BAD_TEMPLATE,
    /** No encoding for this request. Either the action has no emitter yet
     *  (five of the nine), or it has one but the argument selects something
     *  nobody has measured — a door on the left, say. Both are gaps, not
     *  gates, and a gap must never be filled by guessing. */
    FSD_EMIT_NO_ENCODING,
    /** 🔴 We know the command and we have the template, but the template is
     *  outside the region where the check byte was measured — the high beams
     *  or the washer are in use, and 0x249's CRC covers them.
     *
     *  Distinct from NO_ENCODING on purpose. NO_ENCODING is permanent: no
     *  amount of waiting teaches us which bits open the driver's door. This
     *  one clears by itself the moment the stalk goes back to rest, so a
     *  caller may retry and a screen should say something different. */
    FSD_EMIT_NO_CHECK,
} FsdEmitResult;

    /** How many CONSECUTIVE frames this command has to occupy before the car
 *  believes it. One is not always enough, and the first car test proved it.
 *
 * 🔴 MEASURED ON 2026-09-07, IN THE CAR, THE HARD WAY. Eleven perfectly formed
 * 0x249 frames went out -- right counter, right check byte, byte-identical to
 * what TSL sends -- and the turn signal never came on. The capture said why:
 *
 *      7.955  5E09 0000   car,  counter 09, stalk idle
 *      7.956  920A 0800   TSL,  counter 0A, stalk left    +1 ms
 *      8.006  E20A 0000   car,  counter 0A, idle
 *      8.006  580B 0800   TSL,  counter 0B, stalk left
 *      8.055  280B 0000   car
 *      8.056  4A0C 0800   TSL,  counter 0C
 *      8.106  3A0C 0000   car
 *      8.106  630D 0800   TSL,  counter 0D
 *      8.133  0x3F5 byte0  00 -> 02        the lamp comes on
 *
 * TSL gets in front of the car's own frame FOUR TIMES RUNNING, 50 ms apart,
 * for 200 ms. We sent one. The car's own next idle frame -- 13 to 43 ms later
 * -- said "stalk released", and a lever held for 15 ms is not a lever push.
 * The human capture agrees: a real stalk produces six consecutive frames.
 *
 * ⚠️ ONE MEANS "NOT MEASURED", NOT "ONE IS RIGHT". Every other action keeps 1
 * because nobody has watched it fail yet. The map light TSL holds by re-sending
 * for as long as the light is on, which is a different shape again -- a hold,
 * not a burst -- and this number does not describe it. Fill a row in when the
 * car has answered, the way this one was.
 */
uint8_t fsd_emit_repeat(FsdBodyAction a);


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
 * here unchanged. Two actions read it: FSD_ACT_DOOR_OPEN as an FsdEmitDoor and
 * FSD_ACT_TURN_SIGNAL as an FsdEmitTurn.
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
 * FSD_ACT_DOOR_OPEN, FSD_ACT_HAZARDS and FSD_ACT_TURN_SIGNAL. That mirrors
 * fsd_body.c, where those four are the rows with armable_at_runtime = true --
 * two independent statements of the same fact, so widening one without the
 * other does nothing, and a host test asserts they agree for every action.
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
