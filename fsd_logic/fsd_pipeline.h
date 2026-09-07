#pragma once

/*
 * fsd_pipeline.h — the four layers, joined, with nothing transmitted.
 *
 * WHAT THIS IS FOR
 * ----------------
 * The rule engine, the permission axis, the emitter and the TX chokepoint have
 * all existed for a while and NOTHING CALLED THEM. Every one was built, tested
 * and merged as a piece that could not reach the bus, which is why the board
 * has been able to sit in Active for minutes at a time and still report TX=0.
 *
 * That is the right way to have built them and the wrong way to leave them.
 * This file is the join. It takes one trigger event and pushes it all the way
 * to A FRAME IN A STRUCT — and stops there.
 *
 * 🔴 fsd_pipe_run() DOES NOT TRANSMIT. It cannot: it has no bus, no driver and
 * no way to reach one. The caller gets bytes and decides. That split is the
 * whole design, for two reasons:
 *
 *   1. Everything a person would want to check about "should this frame go
 *      out, and what exactly would it be" is decidable on a desk. The host
 *      tests run the entire chain, on real captured frames, with no hardware.
 *   2. The one line that actually writes to the bus stays in ESP32 code, next
 *      to the mode gate and the ID refusal, where it is easy to find and hard
 *      to reach by accident.
 *
 * WHAT STILL GUARDS THE BUS
 * -------------------------
 * Nothing here loosens anything. A decision has to survive, in order:
 *
 *   the rule       the owner has to have written it (fsd_rules_match)
 *   the axis       fsd_body_allows: mode, bus, OTA, RX freshness, driver,
 *                  gear, speed, drive session, min interval, and the
 *                  per-action enable the caller sets
 *   the emitter    fsd_emit_build: a template from the car, fresh, right ID,
 *                  right multiplex, inside the measured region
 *   the chokepoint fsd_body_wire_check: the outgoing frame may differ from the
 *                  car's most recent one ONLY in this action's bits
 *
 * Four independent refusals, each with its own name in the result. A decision
 * that fails any of them yields no frame at all.
 */

#include "fsd_body.h"
#include "fsd_body_emit.h"
#include "fsd_body_wire.h"
#include "fsd_rules.h"
#include "fsd_trigger.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** How many decisions one trigger event can produce in a single call.
 *
 *  Two rules on one trigger is a case the owner asked for by name ("a switch
 *  press turns on the map light AND flashes the hazards"), so this is not 1.
 *  It is small because the rule table is 24 and a single event matching more
 *  than a handful of them is a mistake, not a feature. */
#define FSD_PIPE_MAX_OUT 4u

/** Where a decision stopped. Anything but OK means NO FRAME WAS BUILT. */
typedef enum {
    FSD_PIPE_OK = 0,
    FSD_PIPE_BLOCKED_BODY, /* reason is FsdBodyVerdict */
    FSD_PIPE_BLOCKED_EMIT, /* reason is FsdEmitResult */
    FSD_PIPE_BLOCKED_WIRE, /* reason is FsdBodyWireVerdict */
} FsdPipeStage;

typedef struct {
    uint8_t rule_index; /* so a refusal names the owner's rule */
    FsdBodyAction action;
    int32_t arg;

    FsdPipeStage stage;
    uint8_t reason; /* the verdict of whichever layer stopped it */

    /* 🔴 Meaningful ONLY when stage == FSD_PIPE_OK. Zeroed otherwise, so a
     * caller that forgets to check the stage sends id 0 rather than something
     * plausible. */
    FsdEmitFrame frame;
} FsdPipeResult;

/* The car's most recent frame for each action, kept per action rather than per
 * CAN id. Two actions can share an id -- the map light and the mirror are both
 * 0x273 -- and giving each its own copy costs 24 bytes and removes a lookup
 * that would otherwise have to answer "which of these two did I mean". */
typedef struct {
    FsdEmitTemplate tpl[FSD_ACT_COUNT];
} FsdPipeFrames;

/** Clear every template. Nothing is "seen" until the car says so. */
void fsd_pipe_init(FsdPipeFrames* f);

/** Feed one received CAN frame.
 *
 *  Updates the template of every action whose wire row names this id AND whose
 *  multiplex matches. Returns how many were updated -- 0 for the overwhelming
 *  majority of frames, which is the point: this is cheap enough to call on
 *  every frame on the bus.
 *
 *  🔴 The action->id mapping is fsd_body_wire()'s table, not a second list.
 *  A copy would be one more thing to keep in step, and this repo has paid for
 *  that four times (CAN ids, rule layout, allow-lists, the OTA raw value). */
uint8_t fsd_pipe_observe(FsdPipeFrames* f, uint32_t can_id, const uint8_t* data, uint8_t dlc,
                         uint32_t now_ms);

/**
 * The RELEASE half of a gesture, through the same gates the press faced --
 * minus one, on purpose.
 *
 * 🔴 THE PERMISSION AXIS IS SKIPPED, AND THE SHAPE OF THIS FUNCTION IS THE
 * PROOF: there is no FsdBodyInputs argument, so it is not something a caller
 * can forget to pass. Three reasons, and the third is the one that carries it:
 *
 *   1. THE AXIS IS THE WRONG QUESTION FOR THIS FRAME. It answers "may this
 *      action happen"; it was asked and answered for the press 12 ms ago.
 *      Asking again would let a rate limit -- min_interval_ms is 1000 for the
 *      light horn, and the release is owed after 12 -- refuse the second half
 *      of a gesture whose first half already went out. That leaves the car
 *      holding a button down, which is the exact failure the release exists to
 *      prevent.
 *
 *      ⚠️ It would not refuse it TODAY, because FsdBodyInputs.last_act_ms has
 *      no producer and every min_interval_ms is currently decorative. That is
 *      a defect, not a design, and this function must not depend on which way
 *      it gets resolved.
 *
 *   2. Stopping half way through one gesture is strictly worse than not
 *      starting it. There is no reading of "safer" under which it is better.
 *
 *   3. THE FRAME CANNOT ASSERT ANYTHING, and that is enforced:
 *      fsd_emit_build_release() refuses with FSD_EMIT_FIELD_IN_USE unless the
 *      frame it builds is byte-identical to the car's own most recent one.
 *
 * 🟢 THE CHOKEPOINT IS NOT SKIPPED. fsd_body_wire.h calls itself "the last
 * denial before the wire" and fsd_pipeline.c says why it is separate from the
 * emitter -- "the emitter is trusted to build the frame; it is NOT trusted to
 * have changed only what it was allowed to." A release is a write like any
 * other and faces it, including the row check that refuses an action nobody
 * has opened on purpose. An earlier version of this went from the emitter
 * straight to the bus with a hand-written memcmp in the ESP32 glue; that
 * compared the frame against the template the emitter had just used as its own
 * input, never looked at the id, the length or the multiplex, and sat where no
 * host test could reach it.
 *
 * TRANSMITS NOTHING, same as fsd_pipe_one(). Safe with any NULL.
 */
void fsd_pipe_release(FsdBodyAction action, int32_t arg, uint8_t rule_index,
                      const FsdPipeFrames* f, uint32_t now_ms, FsdPipeResult* out);

/** Push one trigger event through all four layers.
 *
 *  Writes at most `max_out` results and returns how many. A result with
 *  stage == FSD_PIPE_OK carries a frame the caller may transmit; every other
 *  stage carries the reason it may not.
 *
 *  🔴 TRANSMITS NOTHING. See the header comment.
 *
 *  Safe with any NULL: returns 0. */
/** One decision through the same four gates, without matching a rule.
 *
 * fsd_pipe_run() is this in a loop. It exists separately because a BURST needs
 * to re-run a decision the rules already made: the turn signal is not one frame
 * but three or four consecutive ones, and the second through fourth arrive on
 * the car's clock rather than on a trigger.
 *
 * 🔴 IT IS NOT A SHORTCUT PAST ANYTHING. Same axis, same emitter, same
 * chokepoint, same order. Take the belt off in the middle of a burst and the
 * remaining frames are refused with a name, exactly as the first would have
 * been.
 */
void fsd_pipe_one(FsdBodyAction action, int32_t arg, uint8_t rule_index,
                  const FsdBodyInputs* in, const FsdPipeFrames* f,
                  uint32_t now_ms, FsdPipeResult* out);

uint8_t fsd_pipe_run(const FsdRules* rules, const FsdTriggerEvent* ev, const FsdBodyInputs* in,
                     const FsdPipeFrames* f, uint32_t now_ms, FsdPipeResult* out,
                     uint8_t max_out);

/** Human-readable stage, for logs and the app. */
const char* fsd_pipe_stage_str(FsdPipeStage s);

/** The reason, named by whichever layer produced it. Never NULL. */
const char* fsd_pipe_reason_str(FsdPipeStage s, uint8_t reason);

#ifdef __cplusplus
}
#endif
