#pragma once
/*
 * fsd_rules.h — "when this happens, do that". The owner's rules.
 *
 * WHERE THIS SITS
 * ---------------
 *      fsd_signal.c     bytes -> named values
 *      fsd_trigger.c    values -> named events
 *          |
 *      THIS FILE        event -> which actions the owner asked for
 *          |
 *      fsd_body.c       may that action happen right now
 *      fsd_body_wire.c  may this frame carry it
 *      emitter          the frame
 *
 * 🔴 IT DECIDES. IT DOES NOT DO.
 * ------------------------------
 * fsd_rules_match() returns a list and changes nothing. The caller walks that
 * list through fsd_body_allows() and the emitter, and either may refuse.
 *
 * That is not tidiness. A rule engine that dispatched would be a second path to
 * the bus, and this project has already been bitten by exactly that shape: the
 * replay tool was a second transmitter that did not pass the firmware's TX
 * chokepoint, so the safety argument was true of one sender and not the other.
 * Here there is one path, and rules feed into it rather than around it.
 *
 * WHY A RULE IS SO SMALL
 * ----------------------
 * A trigger and an action, and that is all. No conditions, no scripting, no
 * "and only if". The owner asked to connect arbitrary inputs to arbitrary
 * outputs and to overlap them (2026-08-20); overlap is what buys expressiveness
 * here, not a language inside one rule. Two rules on one trigger both fire.
 *
 * 🔴 WHICH MEANS RULES CAN CONTRADICT EACH OTHER, AND THAT IS ALLOWED
 * ------------------------------------------------------------------
 * Two rules that both fire on one press and ask for opposite things will both
 * be requested, and the car will do whichever arrives last. We do not resolve
 * that here — resolving it would mean inventing a priority the owner never
 * asked for, and hiding a mistake is worse than performing it visibly.
 *
 * What we DO prevent is a rule that can never fire at all: a trigger kind the
 * signal cannot produce. fsd_rule_valid() refuses those, so the app cannot
 * offer a mapping the module would silently ignore.
 *
 * 🔴 AND WHAT WE FIRED, WE MUST NOT HEAR
 * --------------------------------------
 * A state trigger fires for whoever changed the state, us included. After an
 * action actually happens the caller tells the trigger layer, using
 * fsd_rule_affects() to know which signals to silence. That mapping lives here
 * rather than in fsd_body.h because fsd_body.h knows nothing about signals and
 * should not start.
 */

#include "fsd_body.h"
#include "fsd_signal.h"
#include "fsd_trigger.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TSL, the thing being replaced, ships fifteen. Twenty-four leaves room without
 * making the table something that has to be paged. Each rule is a handful of
 * bytes. */
#define FSD_RULE_MAX 24

/* Most an event can produce in one go. Equal to FSD_RULE_MAX because every rule
 * is allowed to watch the same trigger — capping it lower would silently drop
 * the owner's later rules, which is the failure this file exists to avoid. */
#define FSD_RULE_MAX_DECISIONS FSD_RULE_MAX

/* Most signals one action can disturb. The map light action touches four lamps;
 * nothing touches more. */
#define FSD_RULE_MAX_AFFECTS 4

typedef struct {
    bool enabled;

    /* The trigger. `value` means: the state for ENTER/LEAVE, and for DELTA the
     * DIRECTION — negative matches a roll down, positive a roll up, zero either
     * way. Not the magnitude: "scroll up" is a rule, "scroll up exactly three"
     * is a wish nobody expressed. */
    FsdSignal signal;
    FsdTriggerKind kind;
    int32_t value;

    /* The action, and whatever it needs. Seat direction, detent count. The
     * emitter interprets it; this file only carries it. */
    FsdBodyAction action;
    int32_t arg;
} FsdRule;

typedef struct {
    FsdRule rule[FSD_RULE_MAX];
} FsdRules;

typedef struct {
    uint8_t rule_index; // so a refusal can be reported against the owner's rule
    FsdBodyAction action;
    int32_t arg;
} FsdRuleDecision;

typedef enum {
    FSD_RULE_OK = 0,
    FSD_RULE_BAD_SIGNAL,
    FSD_RULE_BAD_ACTION,
    FSD_RULE_BAD_KIND,      // a kind at all outside the trigger enum
    FSD_RULE_KIND_MISMATCH, // a kind this signal can never produce
    FSD_RULE_NOT_A_TRIGGER, // STUCK: a fault report, not an intent
    FSD_RULE_BAD_INDEX,
    FSD_RULE_BAD_ARGS,
} FsdRuleVerdict;

/** Every rule disabled. */
void fsd_rules_init(FsdRules* r);

/** Would this rule ever be able to fire?
 *
 *  🔴 The point is not tidiness. A switch signal can never produce ENTER, and a
 *  state signal can never produce PRESS, so a rule pairing them is a control
 *  the owner sets and then watches do nothing — with no way to tell that from a
 *  gate refusing it. Refuse it where it is built instead. */
FsdRuleVerdict fsd_rule_valid(const FsdRule* rule);

/** Store a rule. Refuses rather than storing something that cannot fire. */
FsdRuleVerdict fsd_rules_set(FsdRules* r, uint8_t idx, const FsdRule* rule);

/** The stored rule, or NULL for a bad index. */
const FsdRule* fsd_rules_get(const FsdRules* r, uint8_t idx);

/** One event in, the actions the owner asked for out.
 *
 *  Order is rule order, so two rules on one trigger fire in the order the owner
 *  wrote them — the only ordering that is not invented. */
uint8_t fsd_rules_match(const FsdRules* r, const FsdTriggerEvent* ev, FsdRuleDecision* out,
                        uint8_t max_out);

/** Which signals this action disturbs, so the trigger layer can stay deaf to
 *  our own work. Returns how many were written.
 *
 *  An action with no entry returns 0 — correct, not a gap: the gear and the
 *  camera change nothing this firmware watches as a state. */
uint8_t fsd_rule_affects(FsdBodyAction a, FsdSignal* out, uint8_t max_out);

/** Human-readable verdict, for the app. */
const char* fsd_rule_verdict_str(FsdRuleVerdict v);

/* ── THE WIRE FORM ───────────────────────────────────────────────────────────
 *
 * Twelve bytes, little-endian. FsdRule itself is 24 bytes on the two compilers
 * that build it today and would be something else under -fshort-enums: three of
 * its fields are enums and one is a bool, none of which has a fixed width. NVS
 * and BLE both need a shape that does not depend on how the file was compiled,
 * so neither of them ever sees FsdRule.
 *
 *   [0]      flags   bit0 = enabled. Bits 1..7 RESERVED — written as zero and
 *                    IGNORED on receipt, so a flag added later cannot be
 *                    honoured by accident by a build that predates it.
 *   [1]      signal  FsdSignal
 *   [2]      kind    FsdTriggerKind
 *   [3..6]   value   int32
 *   [7]      action  FsdBodyAction
 *   [8..11]  arg     int32
 *
 * 🔴 UNPACK DOES NOT VALIDATE, DELIBERATELY.
 * It fills the struct and reports whether the ARGUMENTS were usable, nothing
 * more. fsd_rule_valid() is the one authority on whether a rule can ever fire,
 * and a second validator here would be a second thing to keep in step — this
 * file already re-validates at match time for exactly that reason. So an
 * out-of-range signal survives unpack and is then refused by fsd_rules_set() or
 * skipped by fsd_rules_match(); both of those bounds-check their own lookups.
 */
#define FSD_RULE_WIRE_LEN 12u

/* The whole table, as the BLE characteristic carries it. THE POSITION IS THE
 * RULE NUMBER — there is no index byte inside a record, so a short read cannot
 * silently renumber whatever did arrive. */
#define FSD_RULES_WIRE_LEN (FSD_RULE_WIRE_LEN * FSD_RULE_MAX)

/* 🔴 The spelling has to be switched. This header is compiled three ways — C11
 * on the host, pre-C11 C in the ESP32 build (where <assert.h> does not declare
 * `static_assert`), and C++ in ble_server.cpp (where `_Static_assert` is not a
 * keyword). fsd_btn_j6.h paid two broken builds to learn that, and neither
 * failure is visible from here: the host tests are C and pass either way. */
#ifdef __cplusplus
#define FSD_RULE_STATIC_ASSERT(c, m) static_assert(c, m)
#else
#define FSD_RULE_STATIC_ASSERT(c, m) _Static_assert(c, m)
#endif

/* One byte each on the wire. A table that grew past this would be truncated
 * SILENTLY, and 256 lands as 0 — which is a perfectly valid signal, so the rule
 * would come back pointing somewhere else rather than being refused. */
FSD_RULE_STATIC_ASSERT(FSD_SIG_COUNT <= 255, "signal id must fit one wire byte");
FSD_RULE_STATIC_ASSERT(FSD_ACT_COUNT <= 255, "action id must fit one wire byte");
FSD_RULE_STATIC_ASSERT(FSD_TRIG_DELTA <= 255, "trigger kind must fit one wire byte");

/** One rule -> twelve bytes. `out` is fully written; nothing is read back. */
void fsd_rule_pack(const FsdRule* rule, uint8_t out[FSD_RULE_WIRE_LEN]);

/** Twelve bytes -> one rule. False only for a NULL argument.
 *
 *  See the note above: `true` means the bytes were decoded, NOT that the rule
 *  is one that could ever fire. Ask fsd_rule_valid() for that. */
bool fsd_rule_unpack(const uint8_t in[FSD_RULE_WIRE_LEN], FsdRule* out);

/** The whole table, in slot order. What the RULES characteristic returns. */
void fsd_rules_pack_all(const FsdRules* r, uint8_t out[FSD_RULES_WIRE_LEN]);

#ifdef __cplusplus
}
#endif
