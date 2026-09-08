/*
 * test_pipeline.c — the four layers joined, on real captured frames.
 *
 * fsd_pipeline.c is the file that finally connects the rule engine to a set of
 * bytes. Everything it can get wrong is a thing that would otherwise be found
 * in a car, so all of it is checked here: which layer refuses, in what order,
 * and — for the one action that can get all the way through today — that the
 * bytes it produces are the bytes TSL sent.
 *
 * Every frame below is copied out of a capture. A table written to match the
 * implementation would only prove the table matches the implementation.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_autonomy.h"
#include "fsd_burst.h"
#include "fsd_pipeline.h"

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

/* 0x249 as the car sends it — captures/2026-09-05-4차/TSL좌깜빡이켜기 at 7.955.
 * CRC, counter, stalk position, and a byte the car keeps at zero. */
static const uint8_t LSTALK[4] = {0x5E, 0x09, 0x00, 0x00};

/* What TSL sent one millisecond later, same capture. The counter went 9 -> A,
 * the stalk field took DOWN_2, and byte0 is the check value for that pair. */
static const uint8_t TSL_LEFT[4] = {0x92, 0x0A, 0x08, 0x00};

/* 0x273 as the car sends it, every 500 ms — the map light's frame. Present so
 * a test can prove the pipeline refuses the map light for the RIGHT reason:
 * the frame is there and the emitter works; what is missing is a wire row. */
static const uint8_t BODY273[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* 0x3C2 multiplex 0 as the car sends it, from
 * captures/2026-09-06-5차/가벼운경적1회. The light horn's frame, and the one
 * whose release has to face the same denial the press does. */
static const uint8_t HORN_MUX0[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x69, 0x85};

/* Inputs with every gate satisfied. Individual tests spoil one at a time. */
static FsdBodyInputs good_inputs(uint32_t now_ms) {
    FsdBodyInputs in;
    memset(&in, 0, sizeof(in));
    in.op_mode = OpMode_Active;
    in.bus_tx_open = true;
    in.ota_in_progress = false;
    in.rx_stale = false;


    in.gear_seen = true;
    in.gear = FSD_GEAR_D;
    in.gear_ms = now_ms;

    in.speed_seen = true;
    in.speed_kph = 0;
    in.speed_ms = now_ms;

    in.belt_seen = true;
    in.belt_latched = true;
    in.belt_ms = now_ms;

    in.passenger_seen = true;
    in.passenger_present = false;
    in.passenger_ms = now_ms;

    for (unsigned a = 0; a < FSD_ACT_COUNT; a++) in.action_enabled[a] = true;
    return in;
}

/* One enabled rule at index 0: this signal + kind fires this action. */
static void one_rule(FsdRules *r, FsdSignal sig, FsdTriggerKind kind, int32_t value,
                     FsdBodyAction act, int32_t arg) {
    fsd_rules_init(r);
    FsdRule rule;
    memset(&rule, 0, sizeof(rule));
    rule.enabled = true;
    rule.signal = sig;
    rule.kind = kind;
    rule.value = value;
    rule.action = act;
    rule.arg = arg;
    const FsdRuleVerdict v = fsd_rules_set(r, 0, &rule);
    CHECK(v == FSD_RULE_OK, "rule accepted, got %u", (unsigned)v);
}

/* Fill the result array with something that is not zero.
 *
 * 🔴 Every "and no frame was built" assertion below is worthless without
 * this. fsd_pipe_run() zeroes each result before it decides, and if it stopped
 * doing that a refused decision would carry whatever was in the caller's
 * memory -- most often a plausible-looking frame from the previous decision.
 * With the array left uninitialised those assertions passed anyway, because
 * the stack happened to hold zeros. Found by deleting the memset and watching
 * nothing go red. */
static void dirty(FsdPipeResult *out) {
    memset(out, 0xAB, sizeof(FsdPipeResult) * FSD_PIPE_MAX_OUT);
}

static FsdTriggerEvent ev_of(FsdSignal sig, FsdTriggerKind kind, int32_t value, uint32_t at) {
    FsdTriggerEvent e;
    memset(&e, 0, sizeof(e));
    e.signal = sig;
    e.kind = kind;
    e.value = value;
    e.at_ms = at;
    return e;
}

/* ── the template store ────────────────────────────────────────────────── */

static void test_observe(void) {
    printf("\n-- templates come from the car, keyed off the wire table --\n");

    FsdPipeFrames f;
    fsd_pipe_init(&f);
    CHECK(!f.tpl[FSD_ACT_TURN_SIGNAL].seen, "nothing seen after init");

    /* The stalk frame reaches the turn signal's slot. */
    CHECK(fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 4u, 1000u) == 1u, "0x249 stored once");
    CHECK(f.tpl[FSD_ACT_TURN_SIGNAL].seen, "turn signal has a template");
    CHECK(f.tpl[FSD_ACT_TURN_SIGNAL].id == 0x249u, "id kept");
    CHECK(f.tpl[FSD_ACT_TURN_SIGNAL].dlc == 4u, "dlc kept");
    CHECK(memcmp(f.tpl[FSD_ACT_TURN_SIGNAL].data, LSTALK, 4) == 0, "bytes kept verbatim");
    CHECK(f.tpl[FSD_ACT_TURN_SIGNAL].seen_ms == 1000u, "stamped");

    /* ⚠️ THIS USED TO ASSERT THAT 0x273 WAS STORED NOWHERE, because the map
     * light had no wire row. The owner opened all five rows on 2026-09-07, so
     * the claim moves to the one that still holds and is stronger:
     *
     * 🔴 ONE FRAME, TWO ACTIONS, TWO SEPARATE COPIES. The map light and the
     * mirror both write 0x273, which is exactly why the template store is
     * keyed by ACTION and not by CAN id -- a single slot would make "which of
     * these two did I mean" a question this layer cannot answer. */
    CHECK(fsd_pipe_observe(&f, 1u, 0x273u, BODY273, 8u, 1000u) == 2u,
          "0x273 feeds the map light AND the mirror");
    CHECK(f.tpl[FSD_ACT_MAP_LIGHT].seen && f.tpl[FSD_ACT_MIRROR].seen,
          "both actions have a template");
    CHECK(f.tpl[FSD_ACT_MAP_LIGHT].data != f.tpl[FSD_ACT_MIRROR].data,
          "and they are two objects, not one shared one");

    /* Wrong length is not the frame the row describes. */
    fsd_pipe_init(&f);
    CHECK(fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 3u, 1000u) == 0u, "dlc 3 refused");
    CHECK(!f.tpl[FSD_ACT_TURN_SIGNAL].seen, "and nothing was written");

    /* The multiplex is checked. 0x3C2 mux 1 is the scroll wheel; mux 0 is the
     * windows, the belt and the seats. Storing one as the other would describe
     * a completely different set of fields. */
    fsd_pipe_init(&f);
    const uint8_t mux29[8] = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40};
    const uint8_t mux00[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x65, 0x85};
    (void)fsd_pipe_observe(&f, 1u, 0x3C2u, mux29, 8u, 1000u);
    CHECK(f.tpl[FSD_ACT_CAMERA].seen, "mux 0x29 reached the camera slot");
    CHECK(!f.tpl[FSD_ACT_SEAT_DRIVER].seen, "and not the driver seat slot");
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 1u, 0x3C2u, mux00, 8u, 1000u);
    CHECK(f.tpl[FSD_ACT_SEAT_DRIVER].seen, "mux 0 reached the driver seat slot");
    CHECK(!f.tpl[FSD_ACT_CAMERA].seen, "and not the camera slot");

    /* NULL is survivable: this runs on every frame on the bus. */
    CHECK(fsd_pipe_observe(NULL, 1u, 0x249u, LSTALK, 4u, 1000u) == 0u, "NULL store");
    CHECK(fsd_pipe_observe(&f, 1u, 0x249u, NULL, 4u, 1000u) == 0u, "NULL data");
    CHECK(fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 0u, 1000u) == 0u, "dlc 0");
    CHECK(fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 9u, 1000u) == 0u, "dlc 9");
}

/* ── the whole chain, on the one action that can complete it ───────────── */

static void test_turn_signal_end_to_end(void) {
    printf("\n-- a left indicator, rule to bytes --\n");

    const uint32_t now = 5000u;

    FsdRules rules;
    one_rule(&rules, FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, FSD_ACT_TURN_SIGNAL,
             FSD_EMIT_TURN_LEFT);

    FsdPipeFrames f;
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 4u, now);

    const FsdBodyInputs in = good_inputs(now);
    const FsdTriggerEvent ev = ev_of(FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, now);

    FsdPipeResult out[FSD_PIPE_MAX_OUT];
    dirty(out);
    const uint8_t n = fsd_pipe_run(&rules, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT);

    CHECK(n == 1u, "one decision, got %u", (unsigned)n);
    if (n != 1u) return;

    CHECK(out[0].stage == FSD_PIPE_OK, "reached the wire, stopped at %s (%s)",
          fsd_pipe_stage_str(out[0].stage),
          fsd_pipe_reason_str(out[0].stage, out[0].reason));
    CHECK(out[0].rule_index == 0u, "names the owner's rule");
    CHECK(out[0].action == FSD_ACT_TURN_SIGNAL, "action carried through");

    /* 🟢 THE ASSERTION THIS FILE EXISTS FOR: our bytes are TSL's bytes.
     * Not "a plausible frame" — the same four bytes, from the same capture. */
    CHECK(out[0].frame.id == 0x249u, "id 0x249, got 0x%03X", (unsigned)out[0].frame.id);
    CHECK(out[0].frame.dlc == 4u, "dlc 4, got %u", (unsigned)out[0].frame.dlc);
    CHECK(memcmp(out[0].frame.data, TSL_LEFT, 4) == 0,
          "bytes match TSL: got %02X %02X %02X %02X, want %02X %02X %02X %02X",
          out[0].frame.data[0], out[0].frame.data[1], out[0].frame.data[2],
          out[0].frame.data[3], TSL_LEFT[0], TSL_LEFT[1], TSL_LEFT[2], TSL_LEFT[3]);
}

/* ── each layer refuses, and says which one it was ─────────────────────── */

static void test_each_layer_refuses(void) {
    printf("\n-- four layers, four names --\n");

    const uint32_t now = 5000u;
    FsdRules rules;
    FsdPipeFrames f;
    FsdPipeResult out[FSD_PIPE_MAX_OUT];
    const FsdTriggerEvent ev = ev_of(FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, now);

    /* 1. The axis. Listen-Only is the state the board sits in by default, and
     * it has to refuse before anything is built. */
    one_rule(&rules, FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, FSD_ACT_TURN_SIGNAL,
             FSD_EMIT_TURN_LEFT);
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 4u, now);
    FsdBodyInputs in = good_inputs(now);
    in.op_mode = OpMode_ListenOnly;
    in.bus_tx_open = false;
    dirty(out);
    CHECK(fsd_pipe_run(&rules, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 1u, "one result");
    CHECK(out[0].stage == FSD_PIPE_BLOCKED_BODY, "listen-only stops at the axis, got %s",
          fsd_pipe_stage_str(out[0].stage));
    CHECK(out[0].frame.id == 0u, "and no frame was built");

    /* The per-action enable is the session switch. Off means off even in
     * Active with every other gate satisfied. */
    in = good_inputs(now);
    in.action_enabled[FSD_ACT_TURN_SIGNAL] = false;
    dirty(out);
    CHECK(fsd_pipe_run(&rules, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 1u, "one result");
    CHECK(out[0].stage == FSD_PIPE_BLOCKED_BODY, "the enable is a real gate");
    CHECK(out[0].reason == (uint8_t)FSD_BODY_NOT_ENABLED, "named NOT_ENABLED, got %s",
          fsd_pipe_reason_str(out[0].stage, out[0].reason));

    /* 2. The map light, all the way through.
     *
     * ⚠️ THIS WAS THE ORDERING TEST -- "a missing row must say NO_ROW, not
     * NO_TEMPLATE, or it sends a person to look at their wiring". The map
     * light was the action it used, because its frame arrives every 500 ms and
     * its emitter works, so only the ORDER could produce the right answer.
     *
     * 🔴 THAT CLAIM IS NO LONGER TESTABLE WITH ANY ACTION. The owner opened
     * every row on 2026-09-07, so FSD_WIRE_NO_ROW is unreachable from here: an
     * action inside the enum has a row, and one outside it is refused by the
     * axis first (FSD_BODY_UNKNOWN_ACTION). The branch stays as what greets
     * the next action added to the enum, and test_body_wire.c pins the
     * predicate at the unit level.
     *
     * So the case is kept and turned into what it CAN prove, which is more
     * than it proved before: the whole chain produces the exact bytes TSL put
     * on the wire. */
    one_rule(&rules, FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, FSD_ACT_MAP_LIGHT, 0);
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 1u, 0x273u, BODY273, 8u, now);
    in = good_inputs(now);
    const FsdTriggerEvent map_ev = ev_of(FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, now);
    dirty(out);
    CHECK(fsd_pipe_run(&rules, &map_ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 1u, "one result");
    CHECK(out[0].stage == FSD_PIPE_OK, "map light goes through, got %s (%s)",
          fsd_pipe_stage_str(out[0].stage),
          fsd_pipe_reason_str(out[0].stage, out[0].reason));
    CHECK(out[0].frame.id == 0x273u, "on 0x273, got 0x%X", (unsigned)out[0].frame.id);
    /* One bit from the car's own frame, and it is byte 7 bit 3. */
    for(unsigned i = 0; i < 8u; i++) {
        const uint8_t want = (i == 7u) ? (uint8_t)(BODY273[i] | 0x08u) : BODY273[i];
        CHECK(out[0].frame.data[i] == want, "byte %u: expected 0x%02X, got 0x%02X", i, want,
              out[0].frame.data[i]);
    }

    /* 3. The emitter, with no template. A bus we are not hearing is a bus we
     * do not write to. */
    one_rule(&rules, FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, FSD_ACT_TURN_SIGNAL,
             FSD_EMIT_TURN_LEFT);
    fsd_pipe_init(&f); /* nothing observed */
    in = good_inputs(now);
    dirty(out);
    CHECK(fsd_pipe_run(&rules, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 1u, "one result");
    CHECK(out[0].stage == FSD_PIPE_BLOCKED_EMIT, "no template stops at the emitter, got %s",
          fsd_pipe_stage_str(out[0].stage));
    CHECK(out[0].reason == (uint8_t)FSD_EMIT_NO_TEMPLATE, "named NO_TEMPLATE, got %s",
          fsd_pipe_reason_str(out[0].stage, out[0].reason));

    /* A template that has gone stale is the same refusal with a different
     * name: the car stopped talking, so the counter we would build on is old. */
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 4u, 0u);
    dirty(out);
    CHECK(fsd_pipe_run(&rules, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 1u, "one result");
    CHECK(out[0].stage == FSD_PIPE_BLOCKED_EMIT, "a stale template is refused");
    CHECK(out[0].reason == (uint8_t)FSD_EMIT_STALE_TEMPLATE, "named STALE_TEMPLATE, got %s",
          fsd_pipe_reason_str(out[0].stage, out[0].reason));

    /* 3b. THE CHOKEPOINT ACTUALLY REFUSING SOMETHING.
     *
     * 🔴 Without this case the chokepoint could be deleted outright and
     * every other test in this file would still pass -- found by mutation, not
     * by reading. Every other path either stops earlier or sails through, so
     * this is the only place fsd_body_wire_check() is load-bearing here.
     *
     * The two layers keep different windows: the emitter will build on a
     * template up to 1500 ms old, the chokepoint will not compare against a
     * reference older than 200 ms. In between, the emitter says yes and the
     * chokepoint says no -- THE TIGHTER WINDOW WINS, which is the behaviour
     * you want when a bus goes quiet for a moment mid-command. */
    CHECK(FSD_BODY_WIRE_REF_FRESH_MS < FSD_EMIT_TEMPLATE_MAX_AGE_MS,
          "the chokepoint's window is the tighter one -- if this flips, the "
          "test below stops testing anything");
    one_rule(&rules, FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, FSD_ACT_TURN_SIGNAL,
             FSD_EMIT_TURN_LEFT);
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 4u, now - 500u);
    in = good_inputs(now);
    dirty(out);
    CHECK(fsd_pipe_run(&rules, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 1u, "one result");
    CHECK(out[0].stage == FSD_PIPE_BLOCKED_WIRE,
          "500 ms old: the emitter builds, the chokepoint refuses, got %s (%s)",
          fsd_pipe_stage_str(out[0].stage),
          fsd_pipe_reason_str(out[0].stage, out[0].reason));
    CHECK(out[0].reason == (uint8_t)FSD_WIRE_REF_STALE, "named REF_STALE, got %s",
          fsd_pipe_reason_str(out[0].stage, out[0].reason));
    CHECK(out[0].frame.id == 0u, "and the built frame was dropped");

    /* 4. A wire row is not permission.
     *
     * The camera HAS a row -- its bit was measured on 2026-09-01 -- and its
     * frame arrives constantly, so both the store and the chokepoint are ready
     * for it. It still does not go out, because fsd_body.c marks it
     * armable_at_runtime = false. The two tables are independent and BOTH have
     * to open; this is the half that is easy to forget, because the row being
     * there looks like a yes.
     *
     * 🔴 This assertion started life as "the camera has no encoder, so the
     * emitter refuses". That was wrong in an instructive way: the run never
     * reaches the emitter, because the axis is asked first. Refusing earlier
     * than expected is the safe direction, and the test now says what actually
     * happens rather than what I assumed. */
    one_rule(&rules, FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, FSD_ACT_CAMERA, 0);
    fsd_pipe_init(&f);
    const uint8_t mux29[8] = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40};
    (void)fsd_pipe_observe(&f, 1u, 0x3C2u, mux29, 8u, now);
    CHECK(f.tpl[FSD_ACT_CAMERA].seen, "the camera's template really is there");
    CHECK(fsd_body_wire(FSD_ACT_CAMERA) != NULL, "and so is its wire row");
    in = good_inputs(now);
    dirty(out);
    CHECK(fsd_pipe_run(&rules, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 1u, "one result");
    CHECK(out[0].stage == FSD_PIPE_BLOCKED_BODY, "the axis refuses it anyway, got %s",
          fsd_pipe_stage_str(out[0].stage));
    CHECK(out[0].reason == (uint8_t)FSD_BODY_NOT_ARMABLE, "named NOT_ARMABLE, got %s",
          fsd_pipe_reason_str(out[0].stage, out[0].reason));
    CHECK(out[0].frame.id == 0u, "and built nothing");
}

/* ── nothing matched, and other quiet cases ────────────────────────────── */

static void test_quiet_cases(void) {
    printf("\n-- silence is a result too --\n");

    const uint32_t now = 5000u;
    FsdRules rules;
    FsdPipeFrames f;
    FsdPipeResult out[FSD_PIPE_MAX_OUT];

    one_rule(&rules, FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, FSD_ACT_TURN_SIGNAL,
             FSD_EMIT_TURN_LEFT);
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, 4u, now);
    const FsdBodyInputs in = good_inputs(now);

    /* An event no rule wants produces nothing at all — not a refusal. */
    const FsdTriggerEvent other = ev_of(FSD_SIG_MAP_SW_RR, FSD_TRIG_PRESS, 0, now);
    CHECK(fsd_pipe_run(&rules, &other, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 0u,
          "an unmatched event yields no results");

    /* A disabled rule is not a rule. */
    FsdRules off;
    fsd_rules_init(&off);
    const FsdTriggerEvent ev = ev_of(FSD_SIG_MAP_SW_FR, FSD_TRIG_PRESS, 0, now);
    CHECK(fsd_pipe_run(&off, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 0u,
          "an empty table yields no results");

    /* Every NULL. This is called from the RX path. */
    CHECK(fsd_pipe_run(NULL, &ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 0u, "NULL rules");
    CHECK(fsd_pipe_run(&rules, NULL, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 0u, "NULL event");
    CHECK(fsd_pipe_run(&rules, &ev, NULL, &f, now, out, FSD_PIPE_MAX_OUT) == 0u, "NULL inputs");
    CHECK(fsd_pipe_run(&rules, &ev, &in, NULL, now, out, FSD_PIPE_MAX_OUT) == 0u, "NULL frames");
    CHECK(fsd_pipe_run(&rules, &ev, &in, &f, now, NULL, FSD_PIPE_MAX_OUT) == 0u, "NULL out");
    CHECK(fsd_pipe_run(&rules, &ev, &in, &f, now, out, 0u) == 0u, "no room");
}

/* ── the names ─────────────────────────────────────────────────────────── */

static void test_names(void) {
    printf("\n-- every stage and reason has a name --\n");

    CHECK(strcmp(fsd_pipe_stage_str(FSD_PIPE_OK), "ok") == 0, "ok");
    CHECK(strcmp(fsd_pipe_stage_str(FSD_PIPE_BLOCKED_BODY), "axis") == 0, "axis");
    CHECK(strcmp(fsd_pipe_stage_str(FSD_PIPE_BLOCKED_EMIT), "emitter") == 0, "emitter");
    CHECK(strcmp(fsd_pipe_stage_str(FSD_PIPE_BLOCKED_WIRE), "chokepoint") == 0, "chokepoint");

    /* Each stage routes to the layer that produced the number. A reason of 1
     * means three different things depending on the stage, and reading it
     * against the wrong table is how a log sends someone the wrong way. */
    CHECK(strcmp(fsd_pipe_reason_str(FSD_PIPE_BLOCKED_BODY, (uint8_t)FSD_BODY_NOT_ENABLED),
                 fsd_body_verdict_str(FSD_BODY_NOT_ENABLED)) == 0,
          "axis reasons come from the axis");
    CHECK(strcmp(fsd_pipe_reason_str(FSD_PIPE_BLOCKED_EMIT, (uint8_t)FSD_EMIT_NO_TEMPLATE),
                 fsd_emit_result_str(FSD_EMIT_NO_TEMPLATE)) == 0,
          "emitter reasons come from the emitter");
    CHECK(strcmp(fsd_pipe_reason_str(FSD_PIPE_BLOCKED_WIRE, (uint8_t)FSD_WIRE_NO_ROW),
                 fsd_body_wire_verdict_str(FSD_WIRE_NO_ROW)) == 0,
          "chokepoint reasons come from the chokepoint");

    /* An out-of-range number must not walk off the end of a table. */
    CHECK(fsd_pipe_reason_str(FSD_PIPE_BLOCKED_BODY, 250u) != NULL, "axis, absurd value");
    CHECK(fsd_pipe_reason_str(FSD_PIPE_BLOCKED_EMIT, 250u) != NULL, "emitter, absurd value");
    CHECK(fsd_pipe_reason_str(FSD_PIPE_BLOCKED_WIRE, 250u) != NULL, "chokepoint, absurd value");
    CHECK(fsd_pipe_stage_str((FsdPipeStage)99) != NULL, "absurd stage");
}

/* 🔴 ONE FRAME WAS NOT THE COMMAND, AND THE CAR SAID SO.
 *
 * 2026-09-07, first real write: eleven byte-perfect 0x249 frames, right
 * counter, right check byte, and the lamp never lit. TSL gets in front of
 * the car FOUR TIMES RUNNING, 50 ms apart; we sent one, and the car's own
 * idle frame 13-43 ms later said the stalk had been released. A lever held
 * for 15 ms is not a lever push.
 *
 * These pin the two halves of the fix: the number itself, and the fact that
 * a repeated frame still faces every gate. */
static void test_turn_signal_needs_a_burst(void) {
    /* Measured. Not a tuning knob -- 4 is what turned the lamp on. */
    CHECK(fsd_emit_repeat(FSD_ACT_TURN_SIGNAL) == 4u,
          "the turn signal is four consecutive frames, measured 2026-09-07");

    /* Everything else says 1, which means "nobody has watched it fail".
     * A number filled in without a capture behind it is what this catches. */
    for(unsigned a = 0; a < (unsigned)FSD_ACT_COUNT; a++) {
        if((FsdBodyAction)a == FSD_ACT_TURN_SIGNAL) continue;
        CHECK(fsd_emit_repeat((FsdBodyAction)a) == 1u,
              "action %u claims a burst length nobody measured", a);
    }
}

static void test_a_repeat_faces_every_gate(void) {
    const uint32_t now = 500000;

    FsdPipeFrames f;
    fsd_pipe_init(&f);
    fsd_pipe_observe(&f, 1u, 0x249u, LSTALK, sizeof(LSTALK), now);

    FsdBodyInputs in = good_inputs(now);
    in.action_enabled[FSD_ACT_TURN_SIGNAL] = true;

    /* dirty() poisons FSD_PIPE_MAX_OUT results, so the array has to be
     * that long. A single struct here overruns it and smashes `in` --
     * which is exactly what happened the first time this was written,
     * and the axis answered NOT_ENABLED because the enable array had
     * been overwritten with 0xAB. */
    FsdPipeResult res[FSD_PIPE_MAX_OUT];
    dirty(res);
    fsd_pipe_one(FSD_ACT_TURN_SIGNAL, 0, 3, &in, &f, now, &res[0]);
    CHECK(res[0].stage == FSD_PIPE_OK, "a repeat of an allowed action should build");
    CHECK(res[0].rule_index == 3, "the repeat carries the rule that started it");
    CHECK(res[0].frame.id == 0x249u, "and it is still the stalk frame");

    /* 🔴 THE HALF THAT MATTERS. Shut the bus between frame two and frame
     * three of a burst and the rest must stop -- with a name. A burst that
     * bypassed the axis would be a rule that keeps writing after the thing
     * that let it write went away.
     *
     * ⚠️ This used to unlatch the belt instead. The occupancy gate it leaned
     * on was removed on 2026-09-08 (owner's instruction), so the test now
     * takes away the gate that IS still there and still session-scoped: the
     * transmit unlock, which dies with the power. The subject is unchanged --
     * a repeat faces the axis -- only the gate it is asked about. */
    in.bus_tx_open = false;
    dirty(res);
    fsd_pipe_one(FSD_ACT_TURN_SIGNAL, 0, 3, &in, &f, now, &res[0]);
    CHECK(res[0].stage == FSD_PIPE_BLOCKED_BODY,
          "a repeat must face the axis, not skip it");
    CHECK(res[0].reason == (uint8_t)FSD_BODY_BUS_SHUT,
          "and it must say which gate, got %s",
          fsd_pipe_reason_str(res[0].stage, res[0].reason));
    in.bus_tx_open = true;

    /* NULL in, nothing out -- same contract as fsd_pipe_run(). */
    fsd_pipe_one(FSD_ACT_TURN_SIGNAL, 0, 3, NULL, &f, now, &res[0]);
    fsd_pipe_one(FSD_ACT_TURN_SIGNAL, 0, 3, &in, NULL, now, &res[0]);
    fsd_pipe_one(FSD_ACT_TURN_SIGNAL, 0, 3, &in, &f, now, NULL);
}

/* 🔴 THE RELEASE IS A WRITE LIKE ANY OTHER, AND IT FACES THE SAME LAST DENIAL.
 *
 * When the light horn's release was first built it went straight from the
 * emitter to the bus in rule_task.cpp -- skipping fsd_body_wire_check()
 * entirely. The comment there argued at length for skipping the PERMISSION
 * AXIS and never mentioned the chokepoint, which is a different thing with a
 * different job: fsd_body_wire.h calls itself "the last denial before the
 * wire", and fsd_pipeline.c says why it is separate from the emitter -- "the
 * emitter is trusted to build the frame; it is NOT trusted to have changed
 * only what it was allowed to."
 *
 * A hand-written memcmp in the ESP32 glue is not that check. It compares the
 * frame against the same template the emitter just used as its input, it never
 * looks at the id, the length or the multiplex, and it lives where no host
 * test can reach it. So the release goes through fsd_pipe_release() now, and
 * these assertions are what that buys.
 *
 * ⚠️ The axis is STILL skipped, deliberately, and fsd_pipe_release() has no
 * FsdBodyInputs argument at all -- so it is not something a caller can forget
 * to pass, it is a property of the function's shape. */
static void test_release_faces_the_chokepoint(void) {
    printf("\n-- 놓기도 초크포인트를 지난다 --\n");

    FsdPipeFrames f;
    fsd_pipe_init(&f);
    FsdPipeResult r;

    /* No template at all: refused before anything is built. */
    memset(&r, 0, sizeof(r));
    fsd_pipe_release(FSD_ACT_LIGHT_HORN, 0, 3u, &f, 1000u, &r);
    CHECK(r.stage != FSD_PIPE_OK, "no template -> no release");

    /* ⚠️ THIS USED TO ASSERT NO_ROW FOR BOTH HALVES, because the light horn
     * had no wire row. The owner opened it on 2026-09-07, so the case becomes
     * the one it was written to make possible: the release REACHES the
     * chokepoint and passes it, and what it passes with is the claim.
     *
     * 🟢 A RELEASE DIFFERS FROM THE CAR'S OWN FRAME IN NOTHING. So it clears
     * a bit-granularity check by construction -- and asserting the FRAME here,
     * not just the verdict, is what makes that structural rather than lucky. */
    (void)fsd_pipe_observe(&f, 1u, 0x3C2u, HORN_MUX0, 8u, 1000u);
    memset(&r, 0, sizeof(r));
    fsd_pipe_release(FSD_ACT_LIGHT_HORN, 0, 3u, &f, 1050u, &r);
    CHECK(r.stage == FSD_PIPE_OK, "the release goes through, got %s (%s)",
          fsd_pipe_stage_str(r.stage), fsd_pipe_reason_str(r.stage, r.reason));
    CHECK(r.frame.id == 0x3C2u && r.frame.dlc == 8u, "on 0x3C2, 8 bytes");
    CHECK(memcmp(r.frame.data, HORN_MUX0, 8) == 0,
          "and it is the car's frame, byte for byte");

    /* 🔴 The PRESS still faces the axis, and a zeroed input is not a car we
     * are hearing. The release skipping the axis must not mean the gesture
     * skips it -- the first half is where that question is asked. */
    FsdPipeResult press;
    memset(&press, 0, sizeof(press));
    FsdBodyInputs in;
    memset(&in, 0, sizeof(in));
    fsd_pipe_one(FSD_ACT_LIGHT_HORN, 0, 3u, &in, &f, 1050u, &press);
    CHECK(press.stage == FSD_PIPE_BLOCKED_BODY,
          "the press is refused by the axis, got %s", fsd_pipe_stage_str(press.stage));

    /* 🔴 AND THE RELEASE REFUSES WHEN THE CAR SAYS SOMEBODY IS ON THE HORN.
     * This is the one case where a release must NOT go out, and now that the
     * row exists it is reachable end to end rather than only in the emitter. */
    uint8_t held[8];
    memcpy(held, HORN_MUX0, 8);
    held[0] = 0x04u;
    FsdPipeFrames hf;
    fsd_pipe_init(&hf);
    (void)fsd_pipe_observe(&hf, 1u, 0x3C2u, held, 8u, 1000u);
    memset(&r, 0, sizeof(r));
    fsd_pipe_release(FSD_ACT_LIGHT_HORN, 0, 3u, &hf, 1050u, &r);
    CHECK(r.stage == FSD_PIPE_BLOCKED_EMIT, "a held horn refuses the release, got %s",
          fsd_pipe_stage_str(r.stage));
    CHECK(r.reason == (uint8_t)FSD_EMIT_FIELD_IN_USE, "named FIELD_IN_USE, got %s",
          fsd_emit_result_str((FsdEmitResult)r.reason));

    /* An action with no release at all must say so rather than build one. */
    memset(&r, 0, sizeof(r));
    fsd_pipe_release(FSD_ACT_MAP_LIGHT, 0, 0u, &f, 1050u, &r);
    CHECK(r.stage == FSD_PIPE_BLOCKED_EMIT,
          "the map light has no release, got %s", fsd_pipe_stage_str(r.stage));
    CHECK(r.reason == (uint8_t)FSD_EMIT_NO_ENCODING,
          "named as a gap, got %s", fsd_emit_result_str((FsdEmitResult)r.reason));

    /* 🔴 AND THE FRAME IS ZEROED unless the stage is OK, the same contract
     * fsd_pipe_one() keeps -- a caller that forgets to check the stage sends
     * id 0 rather than something plausible. */
    CHECK(r.frame.id == 0u && r.frame.dlc == 0u, "a refused release carries no frame");

    /* Safe with any NULL. */
    fsd_pipe_release(FSD_ACT_LIGHT_HORN, 0, 0u, NULL, 1000u, &r);
    fsd_pipe_release(FSD_ACT_LIGHT_HORN, 0, 0u, &f, 1000u, NULL);
}


/* ════════════════════════════════════════════════════════════════════════
 * fsd_burst — WHEN a decided frame goes out.
 *
 * 🔴 THIS EXISTS BECAUSE OF A FAILURE IN THE CAR, 2026-09-08. The blinker
 * worked and the mirror did not, from the same board, the same session and
 * the same permission state. The capture said why: 0x273 arrived twenty times
 * and every one of them was the car's — not one of ours reached the bus.
 *
 * The scheduler shipped the FIRST frame on the trigger's clock, the instant
 * the switch was pressed, and only the repeats on the car's. For 0x249 that
 * is harmless: at 50 ms the reference is never more than 50 ms old and the
 * 200 ms freshness window always covers it. For 0x273 at 500 ms it almost
 * never does, and the mirror is reps=1 — so that one frame was the whole
 * command.
 *
 * 🔴 THE BLINKER WAS NOT A SUCCESS, IT WAS LUCK. Both actions had the same
 * defect; one of them ran on a bus fast enough to hide it.
 *
 * So every frame waits for the car's. That is not a tuning choice — the
 * emitter COPIES the car's last frame of that id and changes a few bits, so a
 * frame sent between arrivals is built from a template up to a full period
 * old. The chokepoint then refuses it, correctly, as a stale reference. There
 * was never a version of "send now" that could work.
 *
 * The cost is latency: up to one frame period, so 50 ms for the blinker and
 * 500 ms for the mirror. That is the trade this file makes on purpose.
 * ════════════════════════════════════════════════════════════════════════ */

static void test_burst_waits_for_the_cars_frame(void) {
    printf("\n-- burst: nothing goes out until the car's frame does --\n");
    FsdBurst b;
    fsd_burst_reset(&b);

    /* Nothing armed: an arrival produces nothing. */
    FsdBurstDue due;
    CHECK(!fsd_burst_on_frame(&b, 0x249u, 1000u, &due), "idle: no emission");

    /* The mirror. One frame, on 0x273. */
    CHECK(fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 3u, 0x273u, 1u, 1000u), "armed");
    CHECK(fsd_burst_pending(&b) == 1u, "one burst waiting");

    /* 🔴 THE ASSERTION THE CAR PAID FOR. Arming consumes NOTHING: the
     * first arrival still has a frame to emit. The old code shipped here, on
     * the trigger's clock, and that frame is the one the chokepoint refused
     * twenty times out of twenty. If arming emitted, reps=1 would be spent
     * and the arrival below would find an empty slot. */

    /* Another id arriving is not ours. */
    CHECK(!fsd_burst_on_frame(&b, 0x249u, 1010u, &due), "a different id does nothing");
    CHECK(fsd_burst_pending(&b) == 1u, "and the burst is still waiting");

    /* Ours arrives. */
    CHECK(fsd_burst_on_frame(&b, 0x273u, 1400u, &due), "our id fires it");
    CHECK(due.action == FSD_ACT_MIRROR && due.arg == 1 && due.rule_index == 3u,
          "and it carries the decision unchanged");
    CHECK(fsd_burst_pending(&b) == 0u, "reps=1 leaves nothing");
    CHECK(!fsd_burst_on_frame(&b, 0x273u, 1900u, &due), "the next frame is not ours");
}

static void test_burst_counts_four_arrivals_for_the_blinker(void) {
    printf("\n-- burst: four frames, four arrivals --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    CHECK(fsd_burst_arm(&b, FSD_ACT_TURN_SIGNAL, 0, 1u, 0x249u, 4u, 0u), "armed 4");

    FsdBurstDue due;
    for (unsigned i = 0; i < 4u; i++) {
        CHECK(fsd_burst_on_frame(&b, 0x249u, 50u * (i + 1u), &due),
              "arrival %u emits", i + 1u);
        CHECK(due.action == FSD_ACT_TURN_SIGNAL, "arrival %u is the blinker", i + 1u);
    }
    /* 🔴 FOUR, NOT FIVE. The old code sent one immediately and armed reps-1,
     * so switching to reps here without dropping the immediate send would put
     * an extra frame on the bus. */
    CHECK(!fsd_burst_on_frame(&b, 0x249u, 250u, &due), "and then it is done");
    CHECK(fsd_burst_pending(&b) == 0u, "nothing left");
}

static void test_burst_gives_up_when_the_id_never_comes(void) {
    printf("\n-- burst: a deadline, because waiting forever is not waiting --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    CHECK(fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 3u, 0x273u, 1u, 1000u), "armed");

    /* 🔴 THE COST OF WAITING FOR THE CAR IS THAT THE CAR MIGHT NOT SPEAK. A
     * burst with no deadline outlives the press that made it: the operator
     * gives up, walks away, and the frame goes out whenever the bus comes
     * back — a command with no one behind it. */
    CHECK(fsd_burst_tick(&b, 1000u + FSD_BURST_MAX_WAIT_MS - 1u) == 0u,
          "inside the window it keeps waiting");
    CHECK(fsd_burst_pending(&b) == 1u, "still armed");
    CHECK(fsd_burst_tick(&b, 1000u + FSD_BURST_MAX_WAIT_MS) == 1u, "at the window it expires");
    CHECK(fsd_burst_pending(&b) == 0u, "and the slot is free");
    CHECK(b.expired == 1u, "counted, so ruleq can say it happened");

    FsdBurstDue due;
    CHECK(!fsd_burst_on_frame(&b, 0x273u, 9999u, &due),
          "a late frame does not resurrect it");
}

static void test_burst_deadline_is_per_frame_not_per_press(void) {
    printf("\n-- burst: each arrival buys the next one time --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    /* 🔴 A four-frame burst on a bus that stalls halfway must not be judged
     * by when the button was pressed. The question a deadline answers is "has
     * this id gone quiet", and that clock restarts every time it speaks. */
    CHECK(fsd_burst_arm(&b, FSD_ACT_TURN_SIGNAL, 0, 1u, 0x249u, 4u, 0u), "armed 4");
    FsdBurstDue due;
    CHECK(fsd_burst_on_frame(&b, 0x249u, FSD_BURST_MAX_WAIT_MS - 10u, &due),
          "an arrival just inside the window");
    CHECK(fsd_burst_tick(&b, FSD_BURST_MAX_WAIT_MS + 10u) == 0u,
          "does not expire on the ORIGINAL deadline");
    CHECK(fsd_burst_pending(&b) == 1u, "it is still going");
    /* ...but the new one still applies, so a bus that then goes quiet ends it. */
    CHECK(fsd_burst_tick(&b, 2u * FSD_BURST_MAX_WAIT_MS) == 1u,
          "and does expire on the refreshed one");
    CHECK(fsd_burst_pending(&b) == 0u, "gone");
}

static void test_burst_one_frame_per_arrival(void) {
    printf("\n-- burst: two rules on one id do not stack in one millisecond --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    /* The map light and the mirror are both 0x273. If the owner puts both on
     * one switch, two decisions arrive together.
     *
     * 🔴 ONE FRAME PER ARRIVAL. Emitting both on the same arrival would put
     * two of our frames in the same millisecond slot behind one reference —
     * and the second would be built from a template the first has already
     * contradicted. They take turns instead. */
    CHECK(fsd_burst_arm(&b, FSD_ACT_MAP_LIGHT, 0, 0u, 0x273u, 1u, 0u), "map light armed");
    CHECK(fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 3u, 0x273u, 1u, 0u), "mirror armed");
    CHECK(fsd_burst_pending(&b) == 2u, "both waiting");

    FsdBurstDue due;
    CHECK(fsd_burst_on_frame(&b, 0x273u, 500u, &due), "first arrival emits one");
    CHECK(due.action == FSD_ACT_MAP_LIGHT, "and it is the one armed first");
    CHECK(fsd_burst_pending(&b) == 1u, "the other still waits");
    CHECK(fsd_burst_on_frame(&b, 0x273u, 1000u, &due), "second arrival emits the other");
    CHECK(due.action == FSD_ACT_MIRROR, "the mirror, second");
    CHECK(fsd_burst_pending(&b) == 0u, "and now nothing");
}

static void test_burst_slots_are_finite_and_say_so(void) {
    printf("\n-- burst: a full table refuses rather than forgets --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    for (unsigned i = 0; i < FSD_BURST_MAX; i++)
        CHECK(fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, (uint8_t)i, 0x273u, 1u, 0u),
              "slot %u", i);
    /* 🔴 REFUSE, do not overwrite. Silently dropping the oldest would make a
     * command vanish with nothing said — and the counter is what lets ruleq
     * say it happened at all. */
    CHECK(!fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 9u, 0x273u, 1u, 0u), "full: refused");
    CHECK(b.dropped == 1u, "and counted");
    CHECK(fsd_burst_pending(&b) == FSD_BURST_MAX, "nothing was evicted");
}

static void test_burst_reset_stops_everything(void) {
    printf("\n-- burst: stop means stop --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    CHECK(fsd_burst_arm(&b, FSD_ACT_TURN_SIGNAL, 0, 1u, 0x249u, 4u, 0u), "armed");
    /* Locking transmission mid-burst has to drop it here, not rely on a gate
     * further down refusing the rest. Same argument as the release flag. */
    fsd_burst_reset(&b);
    CHECK(fsd_burst_pending(&b) == 0u, "reset clears the table");
    FsdBurstDue due;
    CHECK(!fsd_burst_on_frame(&b, 0x249u, 50u, &due), "and arrivals do nothing");
}

static void test_burst_arming_zero_is_not_a_burst(void) {
    printf("\n-- burst: reps 0 --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    /* fsd_emit_repeat() returns at least 1 for every action, so 0 means a
     * caller got confused. Refuse it: an armed slot that can never emit would
     * hold a slot until the deadline. */
    CHECK(!fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 3u, 0x273u, 0u, 0u), "reps 0 refused");
    CHECK(fsd_burst_pending(&b) == 0u, "nothing armed");
}


/* ════════════════════════════════════════════════════════════════════════
 * min_interval_ms — how often a COMMAND may be issued.
 *
 * 🔴 IT WAS DECORATIVE UNTIL 2026-09-08. FsdBodyInputs.last_act_ms had no
 * producer anywhere in the firmware: body_task.cpp memset the struct and
 * nothing ever wrote that array, so `now - 0` was always enormous and every
 * row's interval passed. Four gates were advertised and three were enforced.
 *
 * It was found while writing the light horn's row — where the failure that
 * matters is not one beep in the wrong place but a stuck rule leaning on the
 * horn — and deliberately left alone, because switching it on naively breaks
 * the indicator: that action sends FOUR frames about 50 ms apart and its own
 * row allows 50 ms, so frames two through four would refuse themselves.
 *
 * 🔴 THE FIX IS NOT A BIGGER NUMBER. It is that the question was misread. The
 * table has always meant commands, not frames — the door's 3000 is "do not
 * open it twice in three seconds" and the horn's 1000 is "no two beeps in one
 * second". A four-frame burst is ONE command. So the stamp goes down when a
 * command is ACCEPTED, once, and the frames it owes are exempt.
 *
 * The record lives here because the burst table already is the record of
 * commands issued, and because rule_task.cpp cannot be compiled on a host —
 * which is how the previous scheduling defect reached the car.
 * ════════════════════════════════════════════════════════════════════════ */

static void test_last_act_is_stamped_once_per_command(void) {
    printf("\n-- rate limit: the stamp is the press, not the frame --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    uint32_t seen[FSD_ACT_COUNT];

    fsd_burst_fill_last_act(&b, seen, FSD_ACT_COUNT);
    CHECK(seen[FSD_ACT_TURN_SIGNAL] == 0u, "never fired reads 0");

    CHECK(fsd_burst_arm(&b, FSD_ACT_TURN_SIGNAL, 0, 1u, 0x249u, 4u, 5000u), "armed");
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_COUNT);
    CHECK(seen[FSD_ACT_TURN_SIGNAL] == 5000u, "stamped at the press");
    CHECK(seen[FSD_ACT_MIRROR] == 0u, "and only that action");

    /* 🔴 THE FRAMES DO NOT RE-STAMP. If they did, a burst would push its own
     * next command out by the whole interval every time it ran. */
    FsdBurstDue due;
    fsd_burst_on_frame(&b, 0x249u, 5050u, &due);
    fsd_burst_on_frame(&b, 0x249u, 5100u, &due);
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_COUNT);
    CHECK(seen[FSD_ACT_TURN_SIGNAL] == 5000u, "still the press, got %u",
          (unsigned)seen[FSD_ACT_TURN_SIGNAL]);
}

static void test_the_frames_of_a_command_are_exempt(void) {
    printf("\n-- rate limit: a burst does not refuse itself --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    CHECK(fsd_burst_arm(&b, FSD_ACT_TURN_SIGNAL, 0, 1u, 0x249u, 4u, 1000u), "armed");

    uint32_t seen[FSD_ACT_COUNT];
    /* 🔴 THE CASE THAT KEPT THE LIMITER SWITCHED OFF. The indicator's row
     * allows 50 ms and its frames arrive about 50 ms apart, so with the press
     * stamped and no exemption the axis would answer TOO_SOON for frames two
     * through four of the command it just accepted — every time, on jitter
     * alone. */
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_TURN_SIGNAL);
    CHECK(seen[FSD_ACT_TURN_SIGNAL] == 0u, "the action being emitted reads 'never'");

    /* ...and nothing else is exempted along with it. A burst on 0x273 must not
     * open a hole for the door. */
    CHECK(fsd_burst_arm(&b, FSD_ACT_DOOR_OPEN, 0, 2u, 0x1F9u, 1u, 1000u), "door armed");
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_TURN_SIGNAL);
    CHECK(seen[FSD_ACT_DOOR_OPEN] == 1000u, "the door keeps its stamp");
}

static void test_a_refused_command_leaves_no_stamp(void) {
    printf("\n-- rate limit: only an accepted command counts --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    /* reps 0 is refused, so nothing was issued and nothing may be recorded --
     * a stamp for a command that never happened would lock out the next real
     * one. (The axis refusals happen before this is reached at all.) */
    CHECK(!fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 3u, 0x273u, 0u, 7000u), "refused");
    uint32_t seen[FSD_ACT_COUNT];
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_COUNT);
    CHECK(seen[FSD_ACT_MIRROR] == 0u, "no stamp");

    /* A full table is the same answer for the same reason. */
    for (unsigned i = 0; i < FSD_BURST_MAX; i++)
        fsd_burst_arm(&b, FSD_ACT_MAP_LIGHT, 0, (uint8_t)i, 0x273u, 1u, 7000u);
    CHECK(!fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 3u, 0x273u, 1u, 8000u), "table full");
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_COUNT);
    CHECK(seen[FSD_ACT_MIRROR] == 0u, "still no stamp for the mirror");
    CHECK(seen[FSD_ACT_MAP_LIGHT] == 7000u, "the map light kept its own");
}

static void test_stamps_outlive_the_burst_that_made_them(void) {
    printf("\n-- rate limit: the interval is measured from the last command --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 3u, 0x273u, 1u, 2000u);
    FsdBurstDue due;
    CHECK(fsd_burst_on_frame(&b, 0x273u, 2400u, &due), "the frame goes");
    CHECK(fsd_burst_pending(&b) == 0u, "burst done");

    uint32_t seen[FSD_ACT_COUNT];
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_COUNT);
    /* 🔴 A slot is reused, so the stamp cannot live in the slot. Losing it when
     * the burst finished would mean the interval only ever gated commands that
     * overlapped -- which is every case it does not need to cover. */
    CHECK(seen[FSD_ACT_MIRROR] == 2000u, "the stamp survives, got %u",
          (unsigned)seen[FSD_ACT_MIRROR]);

    /* And expiring the wait does not erase it either: the command WAS issued;
     * what failed was the car answering. */
    fsd_burst_arm(&b, FSD_ACT_MAP_LIGHT, 0, 0u, 0x273u, 1u, 3000u);
    fsd_burst_tick(&b, 3000u + FSD_BURST_MAX_WAIT_MS);
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_COUNT);
    CHECK(seen[FSD_ACT_MAP_LIGHT] == 3000u, "an expired command still counts");
}

static void test_reset_forgets_the_stamps_too(void) {
    printf("\n-- rate limit: stop clears the record --\n");
    FsdBurst b;
    fsd_burst_reset(&b);
    fsd_burst_arm(&b, FSD_ACT_MIRROR, 1, 3u, 0x273u, 1u, 4000u);
    fsd_burst_reset(&b);
    uint32_t seen[FSD_ACT_COUNT];
    fsd_burst_fill_last_act(&b, seen, FSD_ACT_COUNT);
    /* Session state, like the arm flag. Locking transmission and unlocking it
     * again should not leave a command the operator cannot re-issue. */
    CHECK(seen[FSD_ACT_MIRROR] == 0u, "cleared");
}


/* ════════════════════════════════════════════════════════════════════════
 * WHICH BUS a command goes out on.
 *
 * 🔴 rule_task.h has always promised "a command goes back out the way it came
 * in", and it was written after that promise was broken on the bench: a write
 * went out on can0 while every frame we read arrived on can1. Nothing answers
 * on can0, so the controller saw no ACK, the error counters ran away and the
 * isolator took the bus down.
 *
 * 🔴 THE IMPLEMENTATION KEPT THAT PROMISE ONLY BECAUSE THERE IS ONE BUS. It
 * was a single global set on every received frame -- "whichever channel spoke
 * most recently". With one channel wired that is the same thing. With two it
 * is not: a 0x273 template arrives on Vehicle, a Party frame arrives a
 * millisecond later, and the command built from the Vehicle template goes out
 * on Party. The chokepoint cannot catch it -- it compares bytes, not channels.
 *
 * So the bus belongs to the TEMPLATE, not to the clock. Found while preparing
 * the second channel, which is the only reason it is not still a defect
 * waiting in the car.
 * ════════════════════════════════════════════════════════════════════════ */

static void test_a_command_goes_out_where_its_template_came_in(void) {
    printf("\n-- bus: the template carries it, not the clock --\n");
    FsdPipeFrames f;
    fsd_pipe_init(&f);

    /* Nothing heard yet. */
    FsdBodyInputs in = good_inputs(1000u);
    FsdPipeResult r;
    fsd_pipe_one(FSD_ACT_MAP_LIGHT, 0, 0u, &in, &f, 1000u, &r);
    CHECK(r.stage != FSD_PIPE_OK, "no template -> no frame");
    CHECK(r.bus == FSD_PIPE_BUS_NONE,
          "and no bus either -- a caller that skips the stage must not get a "
          "plausible channel, same rule as the zeroed frame");

    /* The car's 0x273 arrives on can1. */
    CHECK(fsd_pipe_observe(&f, 1u, 0x273u, BODY273, 8u, 1000u) > 0, "0x273 stored");
    fsd_pipe_one(FSD_ACT_MAP_LIGHT, 0, 0u, &in, &f, 1010u, &r);
    CHECK(r.stage == FSD_PIPE_OK, "map light builds");
    CHECK(r.bus == 1u, "and goes out on can1, got %u", (unsigned)r.bus);

    /* 🔴 THE CASE A SECOND CHANNEL CREATES. Something else speaks on can0 in
     * between. The old global would now say 0, and the command built from a
     * can1 template would be written to a channel nothing on it answers. */
    CHECK(fsd_pipe_observe(&f, 0u, 0x249u, LSTALK, 4u, 1015u) > 0,
          "an unrelated id arrives on can0");
    fsd_pipe_one(FSD_ACT_MAP_LIGHT, 0, 0u, &in, &f, 1020u, &r);
    CHECK(r.stage == FSD_PIPE_OK, "still builds");
    CHECK(r.bus == 1u, "STILL can1 -- the other channel's traffic is not ours, got %u",
          (unsigned)r.bus);

    /* ...and the action that DID arrive on can0 goes out there. */
    fsd_pipe_one(FSD_ACT_TURN_SIGNAL, 0, 1u, &in, &f, 1020u, &r);
    CHECK(r.stage == FSD_PIPE_OK, "turn signal builds");
    CHECK(r.bus == 0u, "on can0, where its template came from, got %u", (unsigned)r.bus);
}

static void test_a_template_that_moves_bus_moves_with_it(void) {
    printf("\n-- bus: re-cabling is followed, not remembered --\n");
    FsdPipeFrames f;
    fsd_pipe_init(&f);
    FsdBodyInputs in = good_inputs(2000u);
    FsdPipeResult r;

    CHECK(fsd_pipe_observe(&f, 0u, 0x273u, BODY273, 8u, 2000u) > 0, "heard on can0");
    fsd_pipe_one(FSD_ACT_MAP_LIGHT, 0, 0u, &in, &f, 2005u, &r);
    CHECK(r.bus == 0u, "can0");

    /* The same id now arrives on the other channel -- which is what a re-cable
     * looks like, and also what a gateway forwarding to both looks like. The
     * newest frame is the template, so the newest frame's channel is the
     * answer. */
    CHECK(fsd_pipe_observe(&f, 1u, 0x273u, BODY273, 8u, 2010u) > 0, "now on can1");
    fsd_pipe_one(FSD_ACT_MAP_LIGHT, 0, 0u, &in, &f, 2015u, &r);
    CHECK(r.bus == 1u, "the answer follows the template, got %u", (unsigned)r.bus);
}

static void test_the_release_goes_out_where_the_press_did(void) {
    printf("\n-- bus: both halves of a gesture on one channel --\n");
    FsdPipeFrames f;
    fsd_pipe_init(&f);
    /* The light horn's press and release are one gesture 12 ms apart. Sending
     * the release on the other channel would leave the car holding a button
     * that nothing can let go -- the exact failure the release exists to
     * prevent, with an extra step. */
    CHECK(fsd_pipe_observe(&f, 1u, 0x3C2u, HORN_MUX0, 8u, 3000u) > 0, "mux0 stored");
    FsdPipeResult rel;
    fsd_pipe_release(FSD_ACT_LIGHT_HORN, 0, 5u, &f, 3012u, &rel);
    CHECK(rel.stage == FSD_PIPE_OK, "release builds");
    CHECK(rel.bus == 1u, "on can1, where the press was, got %u", (unsigned)rel.bus);

    /* 🔴 AND A REFUSED RELEASE REPORTS NO CHANNEL. Written because the
     * mutation that let memset's 0 stand here survived: every other test
     * looked at fsd_pipe_one()'s refusals, and nobody looked at this one's.
     * can0 is a real channel, and handing it back for a frame that does not
     * exist is the same defect as handing back a plausible id. */
    FsdPipeFrames empty;
    fsd_pipe_init(&empty);
    FsdPipeResult none;
    fsd_pipe_release(FSD_ACT_LIGHT_HORN, 0, 5u, &empty, 3012u, &none);
    CHECK(none.stage != FSD_PIPE_OK, "no template -> no release");
    CHECK(none.bus == FSD_PIPE_BUS_NONE, "and no channel, got %u", (unsigned)none.bus);

    /* ...including the refusals that come before the template is even looked
     * at -- an action that has no release at all. */
    fsd_pipe_release(FSD_ACT_MAP_LIGHT, 0, 0u, &f, 3012u, &none);
    CHECK(none.stage != FSD_PIPE_OK, "the map light is not a gesture");
    CHECK(none.bus == FSD_PIPE_BUS_NONE, "still no channel, got %u", (unsigned)none.bus);
}

static void test_init_knows_nothing(void) {
    printf("\n-- bus: nothing is assumed before the car speaks --\n");
    FsdPipeFrames f;
    memset(&f, 0xAAu, sizeof(f));
    fsd_pipe_init(&f);
    FsdBodyInputs in = good_inputs(10u);
    for (unsigned a = 0; a < FSD_ACT_COUNT; a++) {
        FsdPipeResult r;
        fsd_pipe_one((FsdBodyAction)a, 0, 0u, &in, &f, 10u, &r);
        CHECK(r.bus == FSD_PIPE_BUS_NONE, "action %u starts with no bus", a);
    }
    /* 🔴 0 is a real channel (can0), so it cannot mean "never heard". The same
     * argument as byte 26's profile sentinel and byte 28's percentage. */
    CHECK(FSD_PIPE_BUS_NONE != 0u && FSD_PIPE_BUS_NONE != 1u,
          "the sentinel is not a channel");
}

int main(void) {
    printf("test_pipeline: rule -> axis -> emitter -> chokepoint\n");
    test_turn_signal_needs_a_burst();
    test_a_repeat_faces_every_gate();
    test_observe();
    test_turn_signal_end_to_end();
    test_each_layer_refuses();
    test_quiet_cases();
    test_names();
    test_release_faces_the_chokepoint();
    test_burst_waits_for_the_cars_frame();
    test_burst_counts_four_arrivals_for_the_blinker();
    test_burst_gives_up_when_the_id_never_comes();
    test_burst_deadline_is_per_frame_not_per_press();
    test_burst_one_frame_per_arrival();
    test_burst_slots_are_finite_and_say_so();
    test_burst_reset_stops_everything();
    test_burst_arming_zero_is_not_a_burst();
    test_last_act_is_stamped_once_per_command();
    test_the_frames_of_a_command_are_exempt();
    test_a_refused_command_leaves_no_stamp();
    test_stamps_outlive_the_burst_that_made_them();
    test_reset_forgets_the_stamps_too();
    test_a_command_goes_out_where_its_template_came_in();
    test_a_template_that_moves_bus_moves_with_it();
    test_the_release_goes_out_where_the_press_did();
    test_init_knows_nothing();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
