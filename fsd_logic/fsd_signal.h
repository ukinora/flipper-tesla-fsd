#pragma once
/*
 * fsd_signal.h — named values pulled out of CAN frames, and nothing else.
 *
 * WHERE THIS SITS
 * ---------------
 *      CAN frames
 *          |
 *      THIS FILE        raw bytes -> named signals          (no state)
 *          |
 *      triggers         signal changes -> "the owner pressed something"
 *          |
 *      rule engine      trigger -> action                   (owner's rules)
 *          |
 *      fsd_body.c       may this action happen right now
 *          |
 *      emitter          builds the frame
 *          |
 *      fsd_body_wire.c  the last denial
 *
 * WHY A TABLE AND NOT A PARSER PER FRAME
 * --------------------------------------
 * The rule engine has to let the owner point at any input. A hand-written
 * parser per feature makes "which inputs exist" a property of how much code
 * somebody wrote; a table makes it a property of what we measured. The app can
 * enumerate this table and the answer is the truth.
 *
 * 🔴 EVERY BIT POSITION HERE WAS MEASURED ON THIS CAR, 2026-09-01 AND -09-05
 * --------------------------------------------------------------------------
 * Not read off a DBC. The owner pressed each map light and each window switch
 * in a fixed order, one per second, so the capture shows one field moving at a
 * time — 차량-캡처-2026-09-01.md §8-C. Where a borrowed DBC had a prediction it
 * agreed, four times out of four, but the prediction is not the source.
 *
 * The one thing NOT measured is the passenger-seat command, so it is absent
 * rather than guessed.
 *
 * BIT NUMBERING
 * -------------
 * bit = byte * 8 + bit_within_byte, LSB first. The same convention the DBCs
 * use for start bits, so a signal written here can be compared to one written
 * there without a mental transform.
 *
 * NO STATE
 * --------
 * This file remembers nothing. "Was it pressed a moment ago" is the trigger
 * layer's question, not this one — keeping the split means a signal can be
 * added without touching anything that latches.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* No multiplex on this frame. */
#define FSD_SIG_NO_MUX 0xFFu

typedef enum {
    /* Map light switches. One bit each, and 🔴 THEY ARE LEVEL: the bit stays
     * set while a finger is down. Measured 2026-09-01 — a tap held 160-360 ms
     * and a deliberate hold 1.2-3.7 s, following the finger both times.
     *
     * This is the opposite of the J6 remote, where six buttons of seven send
     * press and release together. Mixing the two kinds in one table is how one
     * of them breaks, which is why the trigger layer asks each source what it
     * is instead of assuming. */
    FSD_SIG_MAP_SW_FL = 0,
    FSD_SIG_MAP_SW_FR,
    FSD_SIG_MAP_SW_RL,
    FSD_SIG_MAP_SW_RR,

    /* Map light state: is the lamp on. Two bits each. Separate from the switch
     * because "someone pressed it" and "it is lit" are different questions —
     * and because a rule that triggers on the STATE will also see a lamp we
     * turned on ourselves. */
    FSD_SIG_MAP_ON_FL,
    FSD_SIG_MAP_ON_FR,
    FSD_SIG_MAP_ON_RL,
    FSD_SIG_MAP_ON_RR,

    /* Window switches on the driver's armrest. Two bits each: 0 idle, 1 first
     * detent, 2 second (one-touch). Eight fields, and the capture walked all
     * eight in order. */
    FSD_SIG_WIN_UP_FL,
    FSD_SIG_WIN_DN_FL,
    FSD_SIG_WIN_UP_RL,
    FSD_SIG_WIN_DN_RL,
    FSD_SIG_WIN_UP_FR, // 🔴 the T2 gesture's input, at bit 40 as predicted
    FSD_SIG_WIN_DN_FR,
    FSD_SIG_WIN_UP_RR,
    FSD_SIG_WIN_DN_RR,

    FSD_SIG_BELT_FRONT, // 1 unlatched, 2 latched
    FSD_SIG_DRIVER_PRESENT,

    /* Door latches. 2 = shut, 1 = open — the values, not a guess at polarity:
     * the driver's door was opened and closed on camera. */
    FSD_SIG_DOOR_FL_LATCH,
    FSD_SIG_DOOR_RR_LATCH,

    FSD_SIG_GEAR, // 1 P, 2 R, 3 N, 4 D
    FSD_SIG_SCROLL_TICKS, // signed detents, accumulating

    /* ---- measured on the 4th visit, 2026-09-05 -----------------------------
     *
     * 🔴 APPENDED, NEVER INSERTED. These numbers are what a stored rule keeps
     * in NVS and what the app matches on. Slot one in the middle and every
     * rule the owner already wrote points at a different input, silently.
     */

    /* 0x3C2 mux 0, byte 0. The same byte that carries the multiplex selector,
     * which is why the mask over that selector had to narrow to two bits
     * before these could be read at all -- see fsd_signal.c.
     *
     * Three button presses are the only non-idle byte-0 values in 32 captures
     * across three visits, and all three land where the borrowed DBC says:
     * horn 2|1, hazard button 3|1, driver present 4|1. */
    FSD_SIG_HORN_SW,
    FSD_SIG_HAZARD_BTN,

    /* 0x249 SCCM_leftStalk, byte 2 bits [3:1].
     *   0 idle, 1 UP_1, 2 UP_2 (right), 3 DOWN_1, 4 DOWN_2 (left)
     *
     * ⚠️ ONE BIT LEFT OF WHERE THE DBC PUTS IT. opendbc writes 16|3, which
     * makes the captured 0x08 read as 0 (idle) at the exact moment the left
     * indicator came on. At 17|3 the four captured values are 1,2,3,4 -- the
     * value table, exactly. The frames decide, not the DBC.
     *
     * 🔴 STATE, not SWITCH, for two reasons and the second one is the one that
     * matters. First, direction is the whole point: a SWITCH row would report
     * "non-zero" and lose left-vs-right. Second, THIS FILE'S DEFINITION OF
     * STATE IS "we can cause it" -- the commercial device turns the indicators
     * on by replaying this very field, so the day we build that emitter our own
     * frame lands here. STATE gets the suppression machinery; SWITCH is exempt
     * from it on the grounds that "we cannot press one", which would stop being
     * true. */
    FSD_SIG_TURN_STALK,

    /* 0x3F5 byte 0 bit 4. The hazards, as the car reports them.
     *
     * 🟢 This bit does NOT blink. The two indicator fields beside it alternate
     * (0x02/0x01 left, 0x08/0x04 right, about three times a second) but bit 4
     * held through both phases in every hazard capture we have -- 0x1A/0x15
     * when a person pressed the button, 0x7A/0x75 when the device turned them
     * on for reverse. So it is a state a rule can watch without firing on
     * every flash.
     *
     * 🔴 The indicator fields themselves are deliberately NOT here. They are
     * measured just as well, but a STATE rule on a blinking field fires on
     * every flash, and whether that is what an owner means is a design
     * question rather than a reading. */
    FSD_SIG_HAZARD_ON,

    FSD_SIG_COUNT,
} FsdSignal;

typedef enum {
    /* A finger is on it right now. Edges are presses. */
    FSD_SIGK_SWITCH,
    /* Something about the car. Edges are events, but 🔴 WE CAN CAUSE THEM. */
    FSD_SIGK_STATE,
    /* Signed, and it accumulates: one frame can carry eight detents. */
    FSD_SIGK_DELTA,
} FsdSignalKind;

typedef struct {
    FsdSignal signal; // must equal its own index; checked at runtime
    const char* name; // what the app shows the owner
    uint32_t can_id;
    uint8_t mux_byte; // FSD_SIG_NO_MUX if the frame has none
    uint8_t mux_mask;
    uint8_t mux_value;
    uint8_t start_bit;
    uint8_t bit_len;
    FsdSignalKind kind;
} FsdSignalDef;

typedef enum {
    FSD_SIGV_OK = 0,
    FSD_SIGV_UNKNOWN,   // out of range, or a table row that drifted
    FSD_SIGV_WRONG_ID,  // this frame does not carry this signal
    FSD_SIGV_WRONG_MUX, // right ID, other variant
    FSD_SIGV_SHORT,     // the frame is too short to hold it
    FSD_SIGV_BAD_ARGS,
} FsdSignalVerdict;

/** The definition, or NULL for an out-of-range or drifted index. */
const FsdSignalDef* fsd_signal_def(FsdSignal s);

/** Pull one signal out of one frame.
 *
 *  Refuses rather than guesses: a frame of the wrong ID, the wrong multiplex or
 *  too few bytes yields a verdict, never a value. `out` is untouched unless the
 *  verdict is OK, so a caller that ignores the verdict gets its own stale value
 *  back rather than a zero that looks like a reading. */
FsdSignalVerdict fsd_signal_extract(FsdSignal s, uint32_t can_id, const uint8_t* data,
                                    uint8_t dlc, int32_t* out);

/** Human-readable verdict, for logs and the BLE surface. */
const char* fsd_signal_verdict_str(FsdSignalVerdict v);

#ifdef __cplusplus
}
#endif
