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

/* How old the template may be. The car sends 0x273 every 500 ms, so this is
 * three periods: enough to ride out one or two dropped frames, short enough
 * that the fields we copy still describe the car as it is now.
 *
 * 🔴 Staleness is not a transmission problem, it is a TRUTH problem. An old
 * template is an old statement about the mirrors and the horn. */
#define FSD_EMIT_TEMPLATE_MAX_AGE_MS 1500u

/* 0x273 UI_vehicleControl -- measured, see the header comment. */
#define FSD_EMIT_MAP_LIGHT_ID    0x273u
#define FSD_EMIT_MAP_LIGHT_BYTE  7u
#define FSD_EMIT_MAP_LIGHT_MASK  0x08u   /* bit 3 of byte 7 = bit 59 */
#define FSD_EMIT_MAP_LIGHT_DLC   8u

typedef enum {
    FSD_EMIT_OK = 0,
    /** No 0x273 has been received, so there is nothing to copy. */
    FSD_EMIT_NO_TEMPLATE,
    /** One was received but it is too old to still describe the car. */
    FSD_EMIT_STALE_TEMPLATE,
    /** The template is the wrong id or too short to hold the field. */
    FSD_EMIT_BAD_TEMPLATE,
    /** This action has no emitter yet. Six of the seven are here. */
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
 * 🔴 This answers "what would the bytes be", NOT "may this happen". The
 * authority axis (fsd_body_allows) is a separate question asked separately, and
 * this function deliberately does not call it -- a builder that also decides is
 * a builder nobody can test in isolation, and this one has to be testable
 * against bytes copied out of a real capture.
 *
 * Returns FSD_EMIT_NO_ENCODING for every action but FSD_ACT_MAP_LIGHT. That
 * mirrors fsd_body.c, where map light is the only row with
 * armable_at_runtime = true -- two independent statements of the same fact, so
 * widening one without the other does nothing.
 */
FsdEmitResult fsd_emit_build(FsdBodyAction action, const FsdEmitTemplate* t,
                             uint32_t now_ms, FsdEmitFrame* out);

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
