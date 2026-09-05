/*
 * test_rules.c — host tests for the rule engine, and for the four layers
 * together.
 *
 * The centrepiece is test_the_owners_two_examples(). The owner wrote these down
 * on 2026-08-20, before any of this existed:
 *
 *   (1) tap the driver's map-light switch twice  -> all four map lights on
 *   (2) when the driver's map light comes on     -> something else happens
 *
 * and CLAUDE.md says plainly that they are examples of a SHAPE, not a feature
 * list — "어떤 규칙을 실제로 만들지는 정해진 적이 없다". So they are not built
 * in anywhere. They are built here, out of real captured frames, to prove the
 * shape can hold them.
 *
 * (2) is the dangerous one and it is why the suppression exists: it fires for
 * whoever lit the lamp, and we are one of the whoevers.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_rules.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (cond) {                                                             \
            g_pass++;                                                           \
        } else {                                                                \
            g_fail++;                                                           \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                       \
            printf(__VA_ARGS__);                                                \
            printf("\n");                                                       \
        }                                                                       \
    } while (0)

#define EVBUF 8
#define DECBUF 8

static FsdRule mk(FsdSignal s, FsdTriggerKind k, int32_t v, FsdBodyAction a, int32_t arg) {
    FsdRule r;
    memset(&r, 0, sizeof(r));
    r.enabled = true;
    r.signal = s;
    r.kind = k;
    r.value = v;
    r.action = a;
    r.arg = arg;
    return r;
}

// fsd_rules_set() wants a pointer and mk() returns a value; C will not let you
// take the address of the latter.
static FsdRuleVerdict set_rule(FsdRules *r, uint8_t i, FsdRule rule) {
    return fsd_rules_set(r, i, &rule);
}

// 0x3E2, verbatim shape from 맵등손으로-A1.
static void light_frame(uint8_t *d, bool fl_switch, bool fl_lamp) {
    d[0] = (uint8_t)(0x02 | (fl_lamp ? 0x40 : 0x00));
    d[1] = (uint8_t)(fl_switch ? 0x40 : 0x00);
    d[2] = 0x80; d[3] = 0x0A; d[4] = 0x00; d[5] = 0xC0; d[6] = 0x04;
}

/* ── the owner's two examples, through all four layers ───────────────────── */

static void test_the_owners_two_examples(void) {
    FsdTriggers t;
    FsdRules r;
    FsdTriggerEvent ev[EVBUF];
    FsdRuleDecision dec[DECBUF];
    uint8_t d[7];

    fsd_trig_init(&t);
    fsd_rules_init(&r);

    // (1) Two taps on the driver's map-light switch -> map lights on.
    //     Double press costs that switch's tap the width of the window, so it
    //     is opt-in, and a rule using it has to turn it on.
    CHECK(fsd_rules_set(&r, 0,
                        &(FsdRule){.enabled = true,
                                   .signal = FSD_SIG_MAP_SW_FL,
                                   .kind = FSD_TRIG_DOUBLE,
                                   .action = FSD_ACT_MAP_LIGHT,
                                   .arg = 1}) == FSD_RULE_OK,
          "rule 1 stores");
    fsd_trig_set_double(&t, FSD_SIG_MAP_SW_FL, true);

    // (2) when that lamp comes on -> the camera, standing in for "something
    //     else". The point is the trigger, not the action.
    CHECK(fsd_rules_set(&r, 1,
                        &(FsdRule){.enabled = true,
                                   .signal = FSD_SIG_MAP_ON_FL,
                                   .kind = FSD_TRIG_STATE_ENTER,
                                   .value = 1,
                                   .action = FSD_ACT_CAMERA}) == FSD_RULE_OK,
          "rule 2 stores");

    light_frame(d, false, false);
    fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1000, ev, EVBUF); // establish

    // Two taps inside the window.
    light_frame(d, true, false);
    fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1100, ev, EVBUF);
    light_frame(d, false, false);
    fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1250, ev, EVBUF); // withheld: twin may come
    light_frame(d, true, false);
    fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1350, ev, EVBUF);
    light_frame(d, false, false);
    uint8_t n = fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1500, ev, EVBUF);

    CHECK(n == 1 && ev[0].kind == FSD_TRIG_DOUBLE, "two taps are one DOUBLE");
    uint8_t m = fsd_rules_match(&r, &ev[0], dec, DECBUF);
    CHECK(m == 1 && dec[0].action == FSD_ACT_MAP_LIGHT && dec[0].rule_index == 0,
          "example (1): the double press asks for the map light");

    // 🔴 Now the light actually comes on — and example (2) watches for exactly
    // that. Without suppression, our own action would fire the second rule.
    FsdSignal touched[FSD_RULE_MAX_AFFECTS];
    const uint8_t na = fsd_rule_affects(dec[0].action, touched, FSD_RULE_MAX_AFFECTS);
    CHECK(na == 4, "the map light action disturbs four lamps, got %u", na);
    for (uint8_t k = 0; k < na; k++) fsd_trig_disturbed(&t, touched[k], 1500);

    light_frame(d, false, true); // the lamp we asked for
    n = fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1550, ev, EVBUF);
    CHECK(n == 0, "🔴 our own light does not fire example (2)");

    // A person pressing the physical switch still does, once the window has
    // passed. The rule is not broken — it is deaf only to us.
    light_frame(d, false, false);
    fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1500 + FSD_TRIG_SUPPRESS_MS + 10, ev, EVBUF);
    light_frame(d, false, true);
    n = fsd_trig_on_frame(&t, 0x3E2u, d, 7, 1500 + FSD_TRIG_SUPPRESS_MS + 60, ev, EVBUF);
    CHECK(n == 2, "LEAVE then ENTER once the window has passed");
    m = fsd_rules_match(&r, &ev[1], dec, DECBUF);
    CHECK(m == 1 && dec[0].action == FSD_ACT_CAMERA && dec[0].rule_index == 1,
          "example (2): a lamp lit by someone else still fires it");
}

/* ── overlap ─────────────────────────────────────────────────────────────── */

// The owner asked to overlap rules. Two on one trigger both fire, in the order
// they were written — the only ordering that is not invented.
static void test_two_rules_on_one_trigger(void) {
    FsdRules r;
    FsdRuleDecision dec[DECBUF];
    fsd_rules_init(&r);

    set_rule(&r, 3, mk(FSD_SIG_MAP_SW_RL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0));
    set_rule(&r, 1, mk(FSD_SIG_MAP_SW_RL, FSD_TRIG_PRESS, 0, FSD_ACT_SEAT_DRIVER, 7));
    set_rule(&r, 7, mk(FSD_SIG_MAP_SW_RL, FSD_TRIG_LONG, 0, FSD_ACT_MAP_LIGHT, 0));

    FsdTriggerEvent ev = {FSD_SIG_MAP_SW_RL, FSD_TRIG_PRESS, 0, 1000};
    const uint8_t n = fsd_rules_match(&r, &ev, dec, DECBUF);
    CHECK(n == 2, "both press rules fire, got %u", n);
    CHECK(dec[0].rule_index == 1 && dec[0].action == FSD_ACT_SEAT_DRIVER && dec[0].arg == 7,
          "rule order, and the argument rides along");
    CHECK(dec[1].rule_index == 3 && dec[1].action == FSD_ACT_CAMERA, "then the later one");

    // The LONG rule on the same switch is a different trigger and stays put.
    ev.kind = FSD_TRIG_LONG;
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 1 && dec[0].rule_index == 7,
          "a long press is not a press");
}

/* ── rules that could never fire are refused where they are built ────────── */

static void test_impossible_rules_are_refused(void) {
    FsdRules r;
    fsd_rules_init(&r);

    // A switch cannot "become" a value; a state cannot be "pressed". Either
    // mapping is a control the owner sets and then watches do nothing.
    FsdRule bad = mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_STATE_ENTER, 1, FSD_ACT_CAMERA, 0);
    CHECK(fsd_rule_valid(&bad) == FSD_RULE_KIND_MISMATCH, "a switch does not enter a state");
    CHECK(fsd_rules_set(&r, 0, &bad) == FSD_RULE_KIND_MISMATCH, "and it is not stored");
    CHECK(!fsd_rules_get(&r, 0)->enabled, "slot 0 is untouched");

    bad = mk(FSD_SIG_MAP_ON_FL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0);
    CHECK(fsd_rule_valid(&bad) == FSD_RULE_KIND_MISMATCH, "a lamp is not pressed");

    bad = mk(FSD_SIG_SCROLL_TICKS, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0);
    CHECK(fsd_rule_valid(&bad) == FSD_RULE_KIND_MISMATCH, "the wheel is not pressed");

    bad = mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_DELTA, 0, FSD_ACT_CAMERA, 0);
    CHECK(fsd_rule_valid(&bad) == FSD_RULE_KIND_MISMATCH, "a switch has no detents");

    // 🔴 STUCK is a wedged contact, and a fault must not be a way to make the
    // car do something.
    bad = mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_STUCK, 0, FSD_ACT_CAMERA, 0);
    CHECK(fsd_rule_valid(&bad) == FSD_RULE_NOT_A_TRIGGER, "a stuck switch is not a trigger");

    bad = mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_NONE, 0, FSD_ACT_CAMERA, 0);
    CHECK(fsd_rule_valid(&bad) == FSD_RULE_BAD_KIND, "no trigger at all");

    bad = mk(FSD_SIG_COUNT, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0);
    CHECK(fsd_rule_valid(&bad) == FSD_RULE_BAD_SIGNAL, "no such input");

    bad = mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, FSD_ACT_COUNT, 0);
    CHECK(fsd_rule_valid(&bad) == FSD_RULE_BAD_ACTION, "no such action");

    CHECK(fsd_rule_valid(NULL) == FSD_RULE_BAD_ARGS, "NULL");
    CHECK(fsd_rules_set(&r, FSD_RULE_MAX, &bad) == FSD_RULE_BAD_INDEX, "no such slot");
    CHECK(fsd_rules_get(&r, FSD_RULE_MAX) == NULL, "and reading it is NULL");

    // A half-built rule can be saved while disabled — the app builds one field
    // at a time and must be able to come back to it. It cannot fire.
    FsdRule half = mk(FSD_SIG_MAP_ON_FL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0);
    half.enabled = false;
    CHECK(fsd_rules_set(&r, 2, &half) == FSD_RULE_OK, "a disabled rule stores unvalidated");
    FsdTriggerEvent ev = {FSD_SIG_MAP_ON_FL, FSD_TRIG_PRESS, 0, 1000};
    FsdRuleDecision dec[DECBUF];
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 0, "and never fires");
}

/* ── two gaps mutation testing found ─────────────────────────────────────────
 *
 * Both of these looked covered and were not. The disabled-rule test used a rule
 * the validator rejected anyway, and every overlap test used rules that shared
 * a signal as well as a kind — so deleting either check turned nothing red.
 */
static void test_a_valid_rule_that_is_off_stays_off(void) {
    FsdRules r;
    FsdRuleDecision dec[DECBUF];
    fsd_rules_init(&r);

    // Perfectly good rule. The only thing wrong with it is that the owner
    // switched it off.
    FsdRule off = mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0);
    off.enabled = false;
    CHECK(fsd_rules_set(&r, 0, &off) == FSD_RULE_OK, "it stores");

    FsdTriggerEvent ev = {FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, 1000};
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 0, "and it does not fire");

    off.enabled = true;
    CHECK(fsd_rules_set(&r, 0, &off) == FSD_RULE_OK, "switched on");
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 1, "and now it does");
}

static void test_the_signal_has_to_match_too(void) {
    FsdRules r;
    FsdRuleDecision dec[DECBUF];
    fsd_rules_init(&r);

    // Four rules, one trigger kind, four different switches. A press on one
    // must not fire the other three — which is the whole reason the map lights
    // are four signals and not one.
    set_rule(&r, 0, mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 10));
    set_rule(&r, 1, mk(FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 11));
    set_rule(&r, 2, mk(FSD_SIG_MAP_SW_RL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 12));
    set_rule(&r, 3, mk(FSD_SIG_WIN_UP_FR, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 13));

    const FsdSignal which[4] = {FSD_SIG_MAP_SW_FL, FSD_SIG_MAP_SW_FR, FSD_SIG_MAP_SW_RL,
                                FSD_SIG_WIN_UP_FR};
    for (int i = 0; i < 4; i++) {
        FsdTriggerEvent ev = {which[i], FSD_TRIG_PRESS, 0, 1000};
        const uint8_t n = fsd_rules_match(&r, &ev, dec, DECBUF);
        CHECK(n == 1, "signal %d fires exactly one rule, got %u", i, n);
        CHECK(n == 1 && dec[0].rule_index == (uint8_t)i, "and it is rule %d", i);
        CHECK(n == 1 && dec[0].arg == 10 + i, "carrying that rule's argument");
    }

    // A state rule on a different state signal likewise.
    fsd_rules_init(&r);
    set_rule(&r, 0, mk(FSD_SIG_MAP_ON_FL, FSD_TRIG_STATE_ENTER, 1, FSD_ACT_CAMERA, 0));
    FsdTriggerEvent ev = {FSD_SIG_MAP_ON_RR, FSD_TRIG_STATE_ENTER, 1, 1000};
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 0,
          "the rear-right lamp does not fire the front-left rule");
}

/* ── a rule from an older build ──────────────────────────────────────────── */

// Rules will come back from NVS. One written by a build whose signal table
// meant something else must not be honoured just because it was stored.
static void test_stored_rules_are_revalidated(void) {
    FsdRules r;
    FsdRuleDecision dec[DECBUF];
    fsd_rules_init(&r);

    // Reach around fsd_rules_set() the way a load from flash would.
    r.rule[0] = mk(FSD_SIG_MAP_ON_FL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0);
    FsdTriggerEvent ev = {FSD_SIG_MAP_ON_FL, FSD_TRIG_PRESS, 0, 1000};
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 0,
          "an impossible rule that got into storage still does not fire");

    r.rule[0] = mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_STUCK, 0, FSD_ACT_CAMERA, 0);
    ev = (FsdTriggerEvent){FSD_SIG_MAP_SW_FL, FSD_TRIG_STUCK, 0, 1000};
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 0, "nor does a stored STUCK rule");
}

/* ── values ──────────────────────────────────────────────────────────────── */

static void test_value_matching(void) {
    FsdRules r;
    FsdRuleDecision dec[DECBUF];
    fsd_rules_init(&r);

    // A state rule wants one state, not any change.
    set_rule(&r, 0, mk(FSD_SIG_GEAR, FSD_TRIG_STATE_ENTER, 4, FSD_ACT_CAMERA, 0));
    FsdTriggerEvent ev = {FSD_SIG_GEAR, FSD_TRIG_STATE_ENTER, 4, 1000};
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 1, "entering D fires it");
    ev.value = 1;
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 0, "entering P does not");
    ev.value = 4;
    ev.kind = FSD_TRIG_STATE_LEAVE;
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 0, "leaving D is a different trigger");

    // 🔴 A scroll rule wants a DIRECTION. The magnitude is whatever the wrist
    // did — a fast roll carries eight detents in one event, and a rule that
    // demanded exactly one would fire on slow rolls only.
    fsd_rules_init(&r);
    set_rule(&r, 0, mk(FSD_SIG_SCROLL_TICKS, FSD_TRIG_DELTA, 1, FSD_ACT_SCROLL, 1));
    set_rule(&r, 1, mk(FSD_SIG_SCROLL_TICKS, FSD_TRIG_DELTA, -1, FSD_ACT_SCROLL, -1));
    set_rule(&r, 2, mk(FSD_SIG_SCROLL_TICKS, FSD_TRIG_DELTA, 0, FSD_ACT_CAMERA, 0));

    ev = (FsdTriggerEvent){FSD_SIG_SCROLL_TICKS, FSD_TRIG_DELTA, 8, 1000};
    uint8_t n = fsd_rules_match(&r, &ev, dec, DECBUF);
    CHECK(n == 2 && dec[0].rule_index == 0 && dec[1].rule_index == 2,
          "eight up matches 'up' and 'either', not 'down'");

    ev.value = -9;
    n = fsd_rules_match(&r, &ev, dec, DECBUF);
    CHECK(n == 2 && dec[0].rule_index == 1 && dec[1].rule_index == 2,
          "nine down matches 'down' and 'either'");
}

/* ── the loop map ────────────────────────────────────────────────────────── */

static void test_affects_is_honest(void) {
    FsdSignal out[FSD_RULE_MAX_AFFECTS];

    CHECK(fsd_rule_affects(FSD_ACT_MAP_LIGHT, out, FSD_RULE_MAX_AFFECTS) == 4,
          "the map light disturbs all four lamps");
    CHECK(fsd_rule_affects(FSD_ACT_SCROLL, out, FSD_RULE_MAX_AFFECTS) == 1 &&
              out[0] == FSD_SIG_SCROLL_TICKS,
          "🔴 the scroll lands in the detent field we read — without this, "
          "'on scroll up, scroll up' runs away");
    CHECK(fsd_rule_affects(FSD_ACT_GEAR_D, out, FSD_RULE_MAX_AFFECTS) == 1 &&
              out[0] == FSD_SIG_GEAR,
          "the gear moves the gear");
    CHECK(fsd_rule_affects(FSD_ACT_HAZARDS, out, FSD_RULE_MAX_AFFECTS) == 1 &&
              out[0] == FSD_SIG_HAZARD_ON,
          "🔴 the hazards land in 0x3F5 bit 4, which we now read — the car "
          "cannot tell our command from the owner's button, so 'on hazards, "
          "hazards' would hold itself on");

    /* 🔴 THE RULE THAT MADE THAT ROW NECESSARY, AND WHY IT IS NOT OPTIONAL.
     *
     * An action that changes a STATE signal we watch must name it. The row
     * above appeared the day FSD_SIG_HAZARD_ON did; before that the hazards
     * changed nothing this firmware read, and an empty row was correct.
     *
     * Stated as a loop rather than as three more lines, so the day somebody
     * adds a signal for something an action already does, this turns red
     * instead of staying quiet. */
    struct {
        FsdBodyAction action;
        FsdSignal must_name;
        const char *why;
    } feedback[] = {
        {FSD_ACT_MAP_LIGHT, FSD_SIG_MAP_ON_FL, "the map light lights the lamp we read"},
        {FSD_ACT_HAZARDS, FSD_SIG_HAZARD_ON, "the hazards set the bit we read"},
        {FSD_ACT_SCROLL, FSD_SIG_SCROLL_TICKS, "the scroll writes the field we read"},
        {FSD_ACT_GEAR_D, FSD_SIG_GEAR, "the gear moves the gear we read"},
    };
    for (unsigned i = 0; i < sizeof(feedback) / sizeof(feedback[0]); i++) {
        const uint8_t n = fsd_rule_affects(feedback[i].action, out, FSD_RULE_MAX_AFFECTS);
        int found = 0;
        for (uint8_t k = 0; k < n; k++)
            if (out[k] == feedback[i].must_name) found = 1;
        CHECK(found, "action %d must name signal %d: %s", (int)feedback[i].action,
              (int)feedback[i].must_name, feedback[i].why);
    }

    // An empty answer is correct, not a gap: the camera changes nothing this
    // firmware reads as a state, so it cannot feed itself.
    CHECK(fsd_rule_affects(FSD_ACT_CAMERA, out, FSD_RULE_MAX_AFFECTS) == 0,
          "the camera disturbs nothing we watch");
    CHECK(fsd_rule_affects(FSD_ACT_SEAT_DRIVER, out, FSD_RULE_MAX_AFFECTS) == 0, "nor the seat");

    // Every signal named as disturbed must actually be a state we watch — a
    // switch cannot be disturbed by us, and fsd_trig_disturbed() ignores those,
    // so naming one here would be a suppression that silently does nothing.
    for (int a = 0; a < FSD_ACT_COUNT; a++) {
        const uint8_t n = fsd_rule_affects((FsdBodyAction)a, out, FSD_RULE_MAX_AFFECTS);
        for (uint8_t k = 0; k < n; k++) {
            const FsdSignalDef *d = fsd_signal_def(out[k]);
            CHECK(d && d->kind != FSD_SIGK_SWITCH,
                  "action %d names %d, which is a switch and cannot be suppressed", a, out[k]);
        }
    }

    CHECK(fsd_rule_affects(FSD_ACT_MAP_LIGHT, NULL, 4) == 0, "NULL out");
    CHECK(fsd_rule_affects(FSD_ACT_MAP_LIGHT, out, 0) == 0, "no room");
    CHECK(fsd_rule_affects(FSD_ACT_MAP_LIGHT, out, 2) == 2, "capped at the room given");
}

/* ── bounds ──────────────────────────────────────────────────────────────── */

static void test_bounds(void) {
    FsdRules r;
    fsd_rules_init(&r);
    for (uint8_t i = 0; i < FSD_RULE_MAX; i++)
        set_rule(&r, i, mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, i));

    FsdTriggerEvent ev = {FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, 1000};
    FsdRuleDecision all[FSD_RULE_MAX_DECISIONS];
    CHECK(fsd_rules_match(&r, &ev, all, FSD_RULE_MAX_DECISIONS) == FSD_RULE_MAX,
          "every rule may watch one trigger");

    FsdRuleDecision two[2];
    CHECK(fsd_rules_match(&r, &ev, two, 2) == 2, "capped at the buffer");
    CHECK(two[0].rule_index == 0 && two[1].rule_index == 1, "and it is the first two");

    CHECK(fsd_rules_match(NULL, &ev, two, 2) == 0, "NULL rules");
    CHECK(fsd_rules_match(&r, NULL, two, 2) == 0, "NULL event");
    CHECK(fsd_rules_match(&r, &ev, NULL, 2) == 0, "NULL out");
    CHECK(fsd_rules_match(&r, &ev, two, 0) == 0, "no room");

    for (int v = 0; v <= FSD_RULE_BAD_ARGS; v++)
        CHECK(fsd_rule_verdict_str((FsdRuleVerdict)v)[0] != '?', "verdict %d is named", v);
}

/* ── the wire form ───────────────────────────────────────────────────────────
 *
 * 🔴 The expected bytes are LITERALS, including the enum ids.
 *
 * Writing `out[1] == FSD_SIG_WIN_UP_FR` would pass no matter what that constant
 * became, which is the trap this repo has already been caught by twice. The ids
 * are now a WIRE CONTRACT: the phone stores rules by them, and inserting a
 * signal in the middle of the enum would silently repoint every stored rule at
 * its neighbour. So they are nailed down here, and moving one turns this red
 * with a message that says which. */

static void test_wire_layout_is_nailed_down(void) {
    // enabled · WIN_UP_FR · double press · value 0 · door-open · arg -2
    FsdRule r = mk(FSD_SIG_WIN_UP_FR, FSD_TRIG_DOUBLE, 0, FSD_ACT_DOOR_OPEN, -2);
    const uint8_t want[FSD_RULE_WIRE_LEN] = {
        0x01,                   // [0]     flags: enabled, nothing reserved set
        0x0C,                   // [1]     signal  FSD_SIG_WIN_UP_FR   = 12
        0x03,                   // [2]     kind    FSD_TRIG_DOUBLE     = 3
        0x00, 0x00, 0x00, 0x00, // [3..6]  value   0
        0x01,                   // [7]     action  FSD_ACT_DOOR_OPEN   = 1
        0xFE, 0xFF, 0xFF, 0xFF, // [8..11] arg     -2, little-endian
    };
    uint8_t got[FSD_RULE_WIRE_LEN];
    fsd_rule_pack(&r, got);
    CHECK(memcmp(got, want, sizeof want) == 0, "the twelve bytes are exactly these");

    // Say which id moved, rather than only that some byte differs.
    CHECK(got[1] == 0x0C, "FSD_SIG_WIN_UP_FR is signal id 12, got %u", got[1]);
    CHECK(got[2] == 0x03, "FSD_TRIG_DOUBLE is kind id 3, got %u", got[2]);
    CHECK(got[7] == 0x01, "FSD_ACT_DOOR_OPEN is action id 1, got %u", got[7]);

    // Little-endian, and both fields, not just the one that happened to be set.
    FsdRule big = mk(FSD_SIG_SCROLL_TICKS, FSD_TRIG_DELTA, 0x12345678, FSD_ACT_SCROLL,
                     0x0A0B0C0D);
    fsd_rule_pack(&big, got);
    CHECK(got[3] == 0x78 && got[4] == 0x56 && got[5] == 0x34 && got[6] == 0x12,
          "value is little-endian");
    CHECK(got[8] == 0x0D && got[9] == 0x0C && got[10] == 0x0B && got[11] == 0x0A,
          "arg is little-endian");
    CHECK(got[1] == 0x15, "FSD_SIG_SCROLL_TICKS is signal id 21, got %u", got[1]);
    CHECK(got[2] == 0x07, "FSD_TRIG_DELTA is kind id 7, got %u", got[2]);
    CHECK(got[7] == 0x05, "FSD_ACT_SCROLL is action id 5, got %u", got[7]);

    CHECK(FSD_RULE_WIRE_LEN == 12u, "a record is twelve bytes");
    CHECK(FSD_RULES_WIRE_LEN == 288u, "the table is 288 bytes");
}

static bool same_rule(const FsdRule *a, const FsdRule *b) {
    return a->enabled == b->enabled && a->signal == b->signal && a->kind == b->kind &&
           a->value == b->value && a->action == b->action && a->arg == b->arg;
}

static void test_wire_roundtrip(void) {
    // The extremes are the point: the sign bit is where an int32 that is
    // rebuilt with the wrong cast goes wrong, and it goes wrong quietly.
    const int32_t vals[] = {0, 1, -1, 2, -2, 127, -128, 32767, -32768,
                            2147483647, -2147483647 - 1, 0x12345678};

    for (size_t i = 0; i < sizeof vals / sizeof vals[0]; i++) {
        FsdRule in = mk(FSD_SIG_GEAR, FSD_TRIG_STATE_ENTER, vals[i], FSD_ACT_CAMERA,
                        vals[(sizeof vals / sizeof vals[0]) - 1 - i]);
        uint8_t b[FSD_RULE_WIRE_LEN];
        FsdRule out;
        memset(&out, 0xAA, sizeof out);
        fsd_rule_pack(&in, b);
        CHECK(fsd_rule_unpack(b, &out), "unpack accepts its own output");
        CHECK(same_rule(&in, &out), "roundtrip keeps value %ld / arg %ld",
              (long)in.value, (long)in.arg);
    }

    // A disabled rule keeps every other field. The app builds a rule one field
    // at a time and has to be able to save a half-finished one and come back.
    FsdRule half = mk(FSD_SIG_MAP_SW_RR, FSD_TRIG_LONG, 3, FSD_ACT_SEAT_PASSENGER, 9);
    half.enabled = false;
    uint8_t b[FSD_RULE_WIRE_LEN];
    FsdRule out;
    fsd_rule_pack(&half, b);
    CHECK(b[0] == 0x00, "the enabled bit is the only thing in the flags byte");
    CHECK(fsd_rule_unpack(b, &out) && same_rule(&half, &out),
          "a disabled rule survives the round trip whole");
}

static void test_reserved_flag_bits_are_ignored(void) {
    // A record from a module that learned a second flag. We must act on the one
    // bit we understand and NOT on the ones we do not.
    uint8_t b[FSD_RULE_WIRE_LEN] = {0xFF, 0x00, 0x01, 0, 0, 0, 0, 0x02, 0, 0, 0, 0};
    FsdRule out;
    CHECK(fsd_rule_unpack(b, &out), "it decodes");
    CHECK(out.enabled, "bit 0 still means enabled");

    uint8_t back[FSD_RULE_WIRE_LEN];
    fsd_rule_pack(&out, back);
    CHECK(back[0] == 0x01, "and we do not echo a flag we did not honour");

    b[0] = 0xFE; // every reserved bit, enabled clear
    CHECK(fsd_rule_unpack(b, &out) && !out.enabled,
          "a record with only reserved bits set is still disabled");
}

// 🔴 Unpack is not a validator, and must not become one: fsd_rule_valid() is
// the single authority. What matters is that garbage from flash cannot reach an
// action — it is refused on the way in and skipped on the way out.
static void test_garbage_bytes_cannot_become_an_action(void) {
    uint8_t b[FSD_RULE_WIRE_LEN] = {0x01, 0xFE, 0x01, 0, 0, 0, 0, 0xFD, 0, 0, 0, 0};
    FsdRule out;
    CHECK(fsd_rule_unpack(b, &out), "unpack itself does not judge");
    CHECK(out.signal == (FsdSignal)0xFE, "it hands over what was written");
    CHECK(fsd_rule_valid(&out) == FSD_RULE_BAD_SIGNAL, "the validator judges");

    FsdRules r;
    fsd_rules_init(&r);
    CHECK(fsd_rules_set(&r, 0, &out) == FSD_RULE_BAD_SIGNAL, "and storing it is refused");

    // Now the flash-corruption path: straight into the table, past set().
    r.rule[0] = out;
    FsdTriggerEvent ev = {(FsdSignal)0xFE, FSD_TRIG_PRESS, 0, 1000};
    FsdRuleDecision dec[DECBUF];
    CHECK(fsd_rules_match(&r, &ev, dec, DECBUF) == 0, "and it still cannot fire");
}

static void test_pack_all(void) {
    FsdRules r;
    uint8_t buf[FSD_RULES_WIRE_LEN];

    // A blank table is 288 zero bytes. Worth pinning: it is what the phone gets
    // from a module nobody has configured, and "all zero" has to mean "nothing
    // set" rather than "rule 0 is signal 0 doing action 0".
    fsd_rules_init(&r);
    memset(buf, 0xAA, sizeof buf);
    fsd_rules_pack_all(&r, buf);
    bool all_zero = true;
    for (size_t i = 0; i < sizeof buf; i++)
        if (buf[i] != 0) all_zero = false;
    CHECK(all_zero, "an untouched table packs to 288 zeros");

    // The position IS the rule number. Put a distinguishable rule in the last
    // slot and check it lands at the last twelve bytes and nowhere else.
    set_rule(&r, FSD_RULE_MAX - 1,
             mk(FSD_SIG_BELT_FRONT, FSD_TRIG_STATE_ENTER, 2, FSD_ACT_GEAR_D, 0));
    fsd_rules_pack_all(&r, buf);
    const size_t last = (size_t)(FSD_RULE_MAX - 1) * FSD_RULE_WIRE_LEN;
    CHECK(last == 276u, "slot 23 starts at byte 276, got %u", (unsigned)last);
    CHECK(buf[last] == 0x01, "slot 23 is enabled");
    CHECK(buf[last + 1] == 0x10, "FSD_SIG_BELT_FRONT is signal id 16, got %u",
          buf[last + 1]);
    CHECK(buf[last + 7] == 0x06, "FSD_ACT_GEAR_D is action id 6, got %u", buf[last + 7]);
    bool rest_zero = true;
    for (size_t i = 0; i < last; i++)
        if (buf[i] != 0) rest_zero = false;
    CHECK(rest_zero, "and it did not smear into any earlier slot");

    // Every slot roundtrips through its own offset.
    fsd_rules_init(&r);
    for (uint8_t i = 0; i < FSD_RULE_MAX; i++)
        set_rule(&r, i, mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, i, FSD_ACT_CAMERA, -(int32_t)i));
    fsd_rules_pack_all(&r, buf);
    for (uint8_t i = 0; i < FSD_RULE_MAX; i++) {
        FsdRule out;
        CHECK(fsd_rule_unpack(&buf[(size_t)i * FSD_RULE_WIRE_LEN], &out), "slot %u decodes", i);
        CHECK(out.value == (int32_t)i && out.arg == -(int32_t)i,
              "slot %u kept its own numbers (%ld/%ld)", i, (long)out.value, (long)out.arg);
    }

    // A NULL table renders as a blank one rather than refusing: 24 empty slots
    // is the truth about what would fire.
    memset(buf, 0xAA, sizeof buf);
    fsd_rules_pack_all(NULL, buf);
    all_zero = true;
    for (size_t i = 0; i < sizeof buf; i++)
        if (buf[i] != 0) all_zero = false;
    CHECK(all_zero, "a NULL table packs to 288 zeros");
}

static void test_wire_null_args(void) {
    FsdRule r = mk(FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0);
    uint8_t b[FSD_RULE_WIRE_LEN];

    memset(b, 0xAA, sizeof b);
    fsd_rule_pack(NULL, b);
    bool blanked = true;
    for (size_t i = 0; i < sizeof b; i++)
        if (b[i] != 0) blanked = false;
    CHECK(blanked, "packing nothing writes an empty slot, not leftovers");

    fsd_rule_pack(&r, NULL);        // must not crash
    fsd_rules_pack_all(NULL, NULL); // 〃
    CHECK(!fsd_rule_unpack(NULL, &r), "NULL input");
    CHECK(!fsd_rule_unpack(b, NULL), "NULL output");
}

int main(void) {
    printf("test_rules\n");
    test_the_owners_two_examples();
    test_two_rules_on_one_trigger();
    test_impossible_rules_are_refused();
    test_a_valid_rule_that_is_off_stays_off();
    test_the_signal_has_to_match_too();
    test_stored_rules_are_revalidated();
    test_value_matching();
    test_affects_is_honest();
    test_bounds();
    test_wire_layout_is_nailed_down();
    test_wire_roundtrip();
    test_reserved_flag_bits_are_ignored();
    test_garbage_bytes_cannot_become_an_action();
    test_pack_all();
    test_wire_null_args();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
