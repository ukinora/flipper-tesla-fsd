/*
 * fsd_rules.c — see fsd_rules.h. A table, a validator, and a matcher.
 */

#include "fsd_rules.h"

/* Which signals each action disturbs.
 *
 * Only the map light has any: it is the one action that changes something this
 * firmware watches as a STATE, so it is the only one that can trigger its own
 * rules. Doors change 0x102/0x103, which we do watch — but no action opens a
 * door yet, and an entry for an action that cannot happen would be a guess
 * about which door.
 *
 * 🔴 An empty row is correct, not missing. The gear moves DI_gear and the
 * camera moves nothing we read, so neither can feed itself. */
typedef struct {
    FsdBodyAction action;
    uint8_t n;
    FsdSignal sig[FSD_RULE_MAX_AFFECTS];
} FsdRuleAffects;

static const FsdRuleAffects FSD_AFFECTS[] = {
    {FSD_ACT_MAP_LIGHT,
     4,
     {FSD_SIG_MAP_ON_FL, FSD_SIG_MAP_ON_FR, FSD_SIG_MAP_ON_RL, FSD_SIG_MAP_ON_RR}},
    /* The scroll changes the speed profile, which we do not read as a signal
     * yet — but the detent field itself is a DELTA signal, and our own write
     * lands in it. Without this, "on scroll up, scroll up" would run away. */
    {FSD_ACT_SCROLL, 1, {FSD_SIG_SCROLL_TICKS}},
    {FSD_ACT_GEAR_D, 1, {FSD_SIG_GEAR}},
};

uint8_t fsd_rule_affects(FsdBodyAction a, FsdSignal* out, uint8_t max_out) {
    if(!out || max_out == 0) return 0;
    for(size_t i = 0; i < sizeof(FSD_AFFECTS) / sizeof(FSD_AFFECTS[0]); i++) {
        if(FSD_AFFECTS[i].action != a) continue;
        uint8_t n = FSD_AFFECTS[i].n;
        if(n > max_out) n = max_out;
        for(uint8_t k = 0; k < n; k++) out[k] = FSD_AFFECTS[i].sig[k];
        return n;
    }
    return 0;
}

void fsd_rules_init(FsdRules* r) {
    if(!r) return;
    for(int i = 0; i < FSD_RULE_MAX; i++) {
        r->rule[i].enabled = false;
        r->rule[i].signal = FSD_SIG_MAP_SW_FL;
        r->rule[i].kind = FSD_TRIG_NONE;
        r->rule[i].value = 0;
        r->rule[i].action = FSD_ACT_MAP_LIGHT;
        r->rule[i].arg = 0;
    }
}

FsdRuleVerdict fsd_rule_valid(const FsdRule* rule) {
    if(!rule) return FSD_RULE_BAD_ARGS;

    const FsdSignalDef* d = fsd_signal_def(rule->signal);
    if(!d) return FSD_RULE_BAD_SIGNAL;
    if(rule->action >= FSD_ACT_COUNT || !fsd_body_caps(rule->action)) {
        return FSD_RULE_BAD_ACTION;
    }

    /* STUCK says a switch has been held past all reason — a wedged contact, a
     * peripheral that stopped reporting. It is a fault, and a fault must not be
     * a way to make the car do something. */
    if(rule->kind == FSD_TRIG_STUCK) return FSD_RULE_NOT_A_TRIGGER;

    switch(rule->kind) {
    case FSD_TRIG_PRESS:
    case FSD_TRIG_LONG:
    case FSD_TRIG_DOUBLE:
        return (d->kind == FSD_SIGK_SWITCH) ? FSD_RULE_OK : FSD_RULE_KIND_MISMATCH;

    case FSD_TRIG_STATE_ENTER:
    case FSD_TRIG_STATE_LEAVE:
        return (d->kind == FSD_SIGK_STATE) ? FSD_RULE_OK : FSD_RULE_KIND_MISMATCH;

    case FSD_TRIG_DELTA:
        return (d->kind == FSD_SIGK_DELTA) ? FSD_RULE_OK : FSD_RULE_KIND_MISMATCH;

    case FSD_TRIG_NONE:
    case FSD_TRIG_STUCK: break;
    }
    return FSD_RULE_BAD_KIND;
}

FsdRuleVerdict fsd_rules_set(FsdRules* r, uint8_t idx, const FsdRule* rule) {
    if(!r || !rule) return FSD_RULE_BAD_ARGS;
    if(idx >= FSD_RULE_MAX) return FSD_RULE_BAD_INDEX;

    /* A disabled rule is stored without validation on purpose: the app builds
     * one field at a time, and refusing a half-finished rule would mean it
     * could not be saved and come back to. It cannot fire while disabled, and
     * enabling it goes through the check. */
    if(rule->enabled) {
        const FsdRuleVerdict v = fsd_rule_valid(rule);
        if(v != FSD_RULE_OK) return v;
    }
    r->rule[idx] = *rule;
    return FSD_RULE_OK;
}

const FsdRule* fsd_rules_get(const FsdRules* r, uint8_t idx) {
    if(!r || idx >= FSD_RULE_MAX) return NULL;
    return &r->rule[idx];
}

/* Does this rule's `value` accept this event's? */
static bool value_matches(const FsdRule* rule, const FsdTriggerEvent* ev) {
    switch(rule->kind) {
    case FSD_TRIG_STATE_ENTER:
    case FSD_TRIG_STATE_LEAVE:
        return rule->value == ev->value;

    case FSD_TRIG_DELTA:
        /* Direction, not magnitude. Zero means either way, which is the only
         * reading that lets "on any scroll" be expressed at all. */
        if(rule->value == 0) return true;
        return (rule->value > 0) == (ev->value > 0);

    default: return true; // presses carry no value
    }
}

uint8_t fsd_rules_match(const FsdRules* r, const FsdTriggerEvent* ev, FsdRuleDecision* out,
                        uint8_t max_out) {
    uint8_t n = 0;
    if(!r || !ev || !out || max_out == 0) return 0;

    /* Nothing matches a fault or a non-event, whatever a rule says.
     *
     * ⚠️ Mutation testing (2026-09-02) shows this line is REDUNDANT: deleting it
     * turns nothing red, because fsd_rule_valid() below refuses the only rules
     * that could reach it. Kept anyway, and said out loud rather than left to
     * look load-bearing — if the validator is ever relaxed, a STUCK is a wedged
     * contact and must not become a way to make the car do something. */
    if(ev->kind == FSD_TRIG_NONE || ev->kind == FSD_TRIG_STUCK) return 0;

    for(uint8_t i = 0; i < FSD_RULE_MAX && n < max_out; i++) {
        const FsdRule* rule = &r->rule[i];
        if(!rule->enabled) continue;
        if(rule->signal != ev->signal) continue;
        if(rule->kind != ev->kind) continue;
        if(!value_matches(rule, ev)) continue;

        /* Re-validated at match time, not trusted from storage. A rule can
         * arrive from NVS written by an older build whose signal table meant
         * something else — and this is the last place to catch that before an
         * action is requested. */
        if(fsd_rule_valid(rule) != FSD_RULE_OK) continue;

        out[n].rule_index = i;
        out[n].action = rule->action;
        out[n].arg = rule->arg;
        n++;
    }
    return n;
}

const char* fsd_rule_verdict_str(FsdRuleVerdict v) {
    switch(v) {
    case FSD_RULE_OK: return "ok";
    case FSD_RULE_BAD_SIGNAL: return "no such input";
    case FSD_RULE_BAD_ACTION: return "no such action";
    case FSD_RULE_BAD_KIND: return "no such trigger";
    case FSD_RULE_KIND_MISMATCH: return "this input cannot do that";
    case FSD_RULE_NOT_A_TRIGGER: return "a stuck switch is a fault, not a trigger";
    case FSD_RULE_BAD_INDEX: return "no such rule slot";
    case FSD_RULE_BAD_ARGS: return "bad args";
    }
    return "?";
}
