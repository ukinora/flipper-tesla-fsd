#include "fsd_pipeline.h"

#include <string.h>

void fsd_pipe_init(FsdPipeFrames* f) {
    if(!f) return;
    memset(f, 0, sizeof(*f));
}

/* Does this frame carry the multiplex this action's row wants?
 *
 * A row with no multiplex accepts anything. A row with one accepts only its
 * own variant -- 0x3C2 mux 1 is the scroll wheel and mux 0 is the windows and
 * the belt, and storing one as the other would hand the emitter a template
 * describing a completely different set of fields. */
static bool mux_matches(const FsdBodyWire* w, const uint8_t* data, uint8_t dlc) {
    if(w->mux_byte == FSD_BODY_WIRE_NO_MUX) return true;
    if(w->mux_byte >= dlc) return false;
    return (uint8_t)(data[w->mux_byte] & w->mux_mask) == w->mux_value;
}

uint8_t fsd_pipe_observe(FsdPipeFrames* f, uint32_t can_id, const uint8_t* data, uint8_t dlc,
                         uint32_t now_ms) {
    if(!f || !data) return 0;
    if(dlc == 0u || dlc > FSD_BODY_WIRE_MAX_DLC) return 0;

    uint8_t n = 0;
    for(uint8_t a = 0; a < (uint8_t)FSD_ACT_COUNT; a++) {
        const FsdBodyWire* w = fsd_body_wire((FsdBodyAction)a);
        /* No row means no known frame for this action -- the camera and the
         * seats are here today. Nothing to store, and storing "some frame"
         * would be worse than storing none. */
        if(!w) continue;
        if(w->can_id != can_id) continue;
        /* A row states the length the car uses. A frame of another length is
         * not the frame the row describes. */
        if(w->dlc != dlc) continue;
        if(!mux_matches(w, data, dlc)) continue;

        FsdEmitTemplate* t = &f->tpl[a];
        t->seen = true;
        t->id = can_id;
        t->dlc = dlc;
        memcpy(t->data, data, dlc);
        if(dlc < sizeof(t->data)) memset(t->data + dlc, 0, sizeof(t->data) - dlc);
        t->seen_ms = now_ms;
        n++;
    }
    return n;
}

/* The template doubles as the chokepoint's reference: both mean "the car's
 * most recent frame of this id and multiplex", and keeping two copies of the
 * same bytes is how they drift. */
static void ref_from_template(const FsdEmitTemplate* t, FsdBodyRef* ref) {
    memset(ref, 0, sizeof(*ref));
    if(!t->seen) return;
    ref->seen = true;
    ref->ms = t->seen_ms;
    ref->dlc = t->dlc;
    uint8_t n = t->dlc;
    if(n > FSD_BODY_WIRE_MAX_DLC) n = FSD_BODY_WIRE_MAX_DLC;
    memcpy(ref->data, t->data, n);
}

uint8_t fsd_pipe_run(const FsdRules* rules, const FsdTriggerEvent* ev, const FsdBodyInputs* in,
                     const FsdPipeFrames* f, uint32_t now_ms, FsdPipeResult* out,
                     uint8_t max_out) {
    if(!rules || !ev || !in || !f || !out || max_out == 0u) return 0;

    FsdRuleDecision dec[FSD_PIPE_MAX_OUT];
    uint8_t want = max_out;
    if(want > FSD_PIPE_MAX_OUT) want = FSD_PIPE_MAX_OUT;

    const uint8_t n = fsd_rules_match(rules, ev, dec, want);

    for(uint8_t i = 0; i < n; i++)
        fsd_pipe_one(dec[i].action, dec[i].arg, dec[i].rule_index, in, f,
                     now_ms, &out[i]);

    return n;
}

/* One decision, four gates. Split out of fsd_pipe_run() on 2026-09-07 so a
 * BURST can reuse it.
 *
 * The first frame of a burst comes from a rule match; the rest come from the
 * car's next 0x249 arrivals. Every one of them faces the same four refusals.
 * A burst that skipped the axis would be a rule that keeps acting after the
 * belt comes off -- which is precisely the gate the car proved this morning. */
void fsd_pipe_one(FsdBodyAction action, int32_t arg, uint8_t rule_index,
                  const FsdBodyInputs* in, const FsdPipeFrames* f,
                  uint32_t now_ms, FsdPipeResult* out) {
    if(!in || !f || !out) return;
    {
        FsdRuleDecision dec[1];
        FsdPipeResult* r = out;
        const uint8_t i = 0;
        dec[0].action = action;
        dec[0].arg = arg;
        dec[0].rule_index = rule_index;
        memset(r, 0, sizeof(*r));
        r->rule_index = dec[i].rule_index;
        r->action = dec[i].action;
        r->arg = dec[i].arg;

        /* 1. The permission axis. Asked FIRST, before a frame is built, so a
         * refusal never depends on whether the bytes happened to work out. */
        const FsdBodyVerdict bv = fsd_body_allows(in, dec[i].action, now_ms);
        if(bv != FSD_BODY_OK) {
            r->stage = FSD_PIPE_BLOCKED_BODY;
            r->reason = (uint8_t)bv;
            return;
        }

        /* An action out of range cannot index the template array. The axis
         * above already refuses it (FSD_BODY_UNKNOWN_ACTION), so this is a
         * second check on the same fact -- kept because the one thing it
         * guards is a read past the end of a stack array. */
        if((uint8_t)dec[i].action >= (uint8_t)FSD_ACT_COUNT) {
            r->stage = FSD_PIPE_BLOCKED_BODY;
            r->reason = (uint8_t)FSD_BODY_UNKNOWN_ACTION;
            return;
        }

        /* 2. Does the chokepoint have a row for this action?
         *
         * 🔴 ASKED HERE, BEFORE THE EMITTER, AND THE ORDER IS THE POINT.
         * An action with no row never gets a template stored either -- the
         * store is keyed off the same table -- so leaving this until step 4
         * means the emitter refuses first, with NO_TEMPLATE, which reads as
         * "the car has not sent that frame". For the map light, the door and
         * the hazards that is exactly wrong: the car sends those frames
         * constantly and the emitter can build all three. What is missing is a
         * row, i.e. a deliberate decision to open them, which is a thing a
         * person does and not a thing a bus does. Sending someone to look at
         * their wiring over it would be this file's fault. */
        if(!fsd_body_wire(dec[i].action)) {
            r->stage = FSD_PIPE_BLOCKED_WIRE;
            r->reason = (uint8_t)FSD_WIRE_NO_ROW;
            return;
        }

        const FsdEmitTemplate* tpl = &f->tpl[(uint8_t)dec[i].action];

        /* 3. The emitter. Copies the car's frame and sets this action's bits.
         * Refuses without a template, which is why fsd_pipe_observe() has to
         * be fed: we do not invent frames the car has never sent. */
        FsdEmitFrame frame;
        const FsdEmitResult er = fsd_emit_build(dec[i].action, dec[i].arg, tpl, now_ms, &frame);
        if(er != FSD_EMIT_OK) {
            r->stage = FSD_PIPE_BLOCKED_EMIT;
            r->reason = (uint8_t)er;
            return;
        }

        /* 4. The chokepoint, at bit granularity. The emitter is trusted to
         * build the frame; it is NOT trusted to have changed only what it was
         * allowed to. Same reference the emitter copied from, so the only
         * thing being compared is what the emitter did to it. */
        FsdBodyRef ref;
        ref_from_template(tpl, &ref);
        const FsdBodyWireVerdict wv =
            fsd_body_wire_check(dec[i].action, frame.id, frame.data, frame.dlc, &ref, now_ms);
        if(wv != FSD_WIRE_OK) {
            r->stage = FSD_PIPE_BLOCKED_WIRE;
            r->reason = (uint8_t)wv;
            return;
        }

        r->stage = FSD_PIPE_OK;
        r->reason = 0u;
        r->frame = frame;
    }
}

const char* fsd_pipe_stage_str(FsdPipeStage s) {
    switch(s) {
    case FSD_PIPE_OK: return "ok";
    case FSD_PIPE_BLOCKED_BODY: return "axis";
    case FSD_PIPE_BLOCKED_EMIT: return "emitter";
    case FSD_PIPE_BLOCKED_WIRE: return "chokepoint";
    }
    return "?";
}

const char* fsd_pipe_reason_str(FsdPipeStage s, uint8_t reason) {
    switch(s) {
    case FSD_PIPE_OK: return "ok";
    case FSD_PIPE_BLOCKED_BODY: return fsd_body_verdict_str((FsdBodyVerdict)reason);
    case FSD_PIPE_BLOCKED_EMIT: return fsd_emit_result_str((FsdEmitResult)reason);
    case FSD_PIPE_BLOCKED_WIRE: return fsd_body_wire_verdict_str((FsdBodyWireVerdict)reason);
    }
    return "?";
}
