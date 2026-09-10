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
 * Nothing here loosens anything on its own. A decision has to survive, in
 * order:
 *
 *   the rule       the owner has to have written it and switched it on
 *                  (fsd_rules_match)
 *   the row        fsd_body_wire: this action has to have a measured row at all
 *   the emitter    fsd_emit_build: a template from the car, fresh, right ID,
 *                  right multiplex, inside the measured region
 *   the chokepoint fsd_body_wire_check: the outgoing frame may differ from the
 *                  car's most recent one ONLY in this action's bits
 *
 * 🔴🔴 THERE WAS A PERMISSION AXIS BETWEEN THE RULE AND THE EMITTER AND IT IS
 * GONE (owner's instruction, 2026-09-10):
 *
 *     "차의 모든 안전게이트관련사항을 삭제해라.
 *      필요하다면 추후 내가 하나씩 추가하겠다."
 *
 * It asked about mode, bus, OTA, RX freshness, gear, speed, the drive session,
 * the rate limit and a per-action enable. All of it went -- not defaulted to
 * "allow", DELETED, so there is nothing left to default. What remains asks
 * nothing about the car's situation: it asks whether we know how to build
 * this frame and whether the bytes we built are the bytes we were allowed to
 * change.
 *
 * ⚠️ SAY IT PLAINLY: A RULE THAT IS SWITCHED ON ACTS. Driving, parked, empty
 * or occupied, the moment its trigger fires and the car's frame arrives.
 *
 * Three independent refusals, each with its own name in the result. A decision
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

/** No channel: this action's template has never arrived.
 *
 *  🔴 0 IS A REAL CHANNEL (can0), so it cannot carry the absence -- the same
 *  argument as the profile sentinel and the battery percentage's. A caller
 *  that forgets to check the stage must not be handed a plausible bus, for
 *  exactly the reason the frame is zeroed. */
#define FSD_PIPE_BUS_NONE 0xFFu

/** Where a decision stopped. Anything but OK means NO FRAME WAS BUILT. */
typedef enum {
    FSD_PIPE_OK = 0,
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

    /* Which channel this frame must go out on: the one its TEMPLATE arrived
     * on, not the one that spoke most recently.
     *
     * 🔴 rule_task.h has always promised "a command goes back out the way it
     * came in", and that promise was written after it was broken on the bench:
     * a write went out on can0 while every frame we read arrived on can1,
     * nothing answered, the error counters ran away and the isolator took the
     * bus down. The implementation kept the promise only because ONE channel
     * was wired -- it was a global set on every received frame. With two, a
     * 0x273 template from Vehicle and a Party frame a millisecond later put
     * the command on Party, and the chokepoint cannot catch that: it compares
     * bytes, not channels.
     *
     * FSD_PIPE_BUS_NONE when no template has arrived. */
    uint8_t bus;
} FsdPipeResult;

/* The car's most recent frame for each action, kept per action rather than per
 * CAN id. Two actions can share an id -- the map light and the mirror are both
 * 0x273 -- and giving each its own copy costs 24 bytes and removes a lookup
 * that would otherwise have to answer "which of these two did I mean". */
typedef struct {
    FsdEmitTemplate tpl[FSD_ACT_COUNT];
    /* Which channel each template arrived on. Parallel to tpl[] rather than
     * inside FsdEmitTemplate because the emitter has no business knowing about
     * buses -- it builds bytes. */
    uint8_t bus[FSD_ACT_COUNT];
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
uint8_t fsd_pipe_observe(FsdPipeFrames* f, uint8_t bus, uint32_t can_id,
                         const uint8_t* data, uint8_t dlc, uint32_t now_ms);

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
 *      ⚠️ It could not have refused it before 2026-09-08 either, because
 *      last_act_ms had no producer and every min_interval_ms was decorative.
 *      That was a defect, not a design, and this function deliberately did not
 *      depend on which way it got resolved -- it is resolved now (the column
 *      counts commands, and a release is the second half of one), and this
 *      function still does not depend on it.
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

/** ONE DECISION THROUGH ALL FOUR GATES, WITH A FRAME AT THE END. THE SEND.
 *
 * Called on the car's own arrival of this action's id, which is where TSL puts
 * its frames and the only instant at which the template is not old. Every
 * frame the board sends comes from here -- the first one of a burst and the
 * repeats alike, since 2026-09-09.
 *
 * 🔴 IT IS NOT A SHORTCUT PAST ANYTHING. The axis is asked again, in full, at
 * every arrival: close the transmission switch in the middle of a burst and
 * the remaining frames are refused with a name. The press's permission is not
 * a licence that outlives the conditions it was granted under.
 *
 * 🔴 TRANSMITS NOTHING. See the header comment. */
void fsd_pipe_one(FsdBodyAction action, int32_t arg, uint8_t rule_index,
                  const FsdPipeFrames* f, uint32_t now_ms, FsdPipeResult* out);

/**
 * WHICH RULES MATCHED, AND MAY EACH OF THEM ACT -- WITHOUT BUILDING A FRAME.
 *
 * This is the PRESS half. fsd_pipe_one() is the SEND half, and the split is
 * not a refactor: it is a measurement.
 *
 * 🔴 THE CAR, 2026-09-09. The mirror worked "sometimes" -- 19 presses, 7
 * frames, 37 %. The press used to run all four layers, and the chokepoint's
 * question is "did the car's own frame land within
 * FSD_BODY_WIRE_REF_FRESH_MS (200)". 0x273 arrives every 500. A finger lands
 * where it lands, so 200/500 = 40 % of presses found a fresh template and the
 * rest were refused before anything was even armed. The 3 points between
 * predicted and measured are 19 presses' worth of rounding.
 *
 * 🟢 THE FRAMES THAT DID GO OUT WERE CORRECT and the mirror moved every time.
 * The send was never the problem and cannot be: a send IS an arrival, and at
 * an arrival the template is 0 ms old.
 *
 * So the freshness question belongs to the send, and asking it at the press is
 * asking a question whose answer is about a moment that has not happened yet.
 *
 * WHAT IS STILL ASKED HERE
 *   the rule    the owner has to have written it and switched it on
 *   the axis    fsd_body_allows in full: mode, bus, OTA, RX freshness, gear,
 *               speed, min interval, and the per-action session enable
 *   the row     the chokepoint has to have a row for this action at all
 *
 * WHAT IS NOT, AND WHY IT CANNOT BE
 *   the emitter and the byte-level chokepoint both need the car's template.
 *   There is no way to ask their STRUCTURAL questions (right mux, inside the
 *   mask) while skipping their TIMING one, because without a fresh template
 *   there is no frame to ask them about. So this function does not try.
 *
 * 🔴 THE SHAPE IS THE PROOF: there is no FsdPipeFrames parameter, so this
 * cannot consult a template even by mistake -- the same argument
 * fsd_pipe_release() makes by not taking FsdBodyInputs.
 *
 * 🔴 A RESULT WITH stage == FSD_PIPE_OK IS PERMISSION, NOT A FRAME. bus stays
 * FSD_PIPE_BUS_NONE and frame stays zeroed, so a caller that ships one anyway
 * is refused by name rather than putting id 0 on the bus.
 *
 * Safe with any NULL: returns 0.
 */
uint8_t fsd_pipe_decide(const FsdRules* rules, const FsdTriggerEvent* ev,
                        FsdPipeResult* out, uint8_t max_out);

/** Human-readable stage, for logs and the app. */
const char* fsd_pipe_stage_str(FsdPipeStage s);

/** The reason, named by whichever layer produced it. Never NULL. */
const char* fsd_pipe_reason_str(FsdPipeStage s, uint8_t reason);

#ifdef __cplusplus
}
#endif
