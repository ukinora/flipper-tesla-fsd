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

    in.drive_session = true;

    in.driver_seen = true;
    in.driver_present = true;
    in.driver_ms = now_ms;

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
    CHECK(fsd_pipe_observe(&f, 0x249u, LSTALK, 4u, 1000u) == 1u, "0x249 stored once");
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
    CHECK(fsd_pipe_observe(&f, 0x273u, BODY273, 8u, 1000u) == 2u,
          "0x273 feeds the map light AND the mirror");
    CHECK(f.tpl[FSD_ACT_MAP_LIGHT].seen && f.tpl[FSD_ACT_MIRROR].seen,
          "both actions have a template");
    CHECK(f.tpl[FSD_ACT_MAP_LIGHT].data != f.tpl[FSD_ACT_MIRROR].data,
          "and they are two objects, not one shared one");

    /* Wrong length is not the frame the row describes. */
    fsd_pipe_init(&f);
    CHECK(fsd_pipe_observe(&f, 0x249u, LSTALK, 3u, 1000u) == 0u, "dlc 3 refused");
    CHECK(!f.tpl[FSD_ACT_TURN_SIGNAL].seen, "and nothing was written");

    /* The multiplex is checked. 0x3C2 mux 1 is the scroll wheel; mux 0 is the
     * windows, the belt and the seats. Storing one as the other would describe
     * a completely different set of fields. */
    fsd_pipe_init(&f);
    const uint8_t mux29[8] = {0x29, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40};
    const uint8_t mux00[8] = {0x00, 0x55, 0x55, 0x55, 0x00, 0x00, 0x65, 0x85};
    (void)fsd_pipe_observe(&f, 0x3C2u, mux29, 8u, 1000u);
    CHECK(f.tpl[FSD_ACT_CAMERA].seen, "mux 0x29 reached the camera slot");
    CHECK(!f.tpl[FSD_ACT_SEAT_DRIVER].seen, "and not the driver seat slot");
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 0x3C2u, mux00, 8u, 1000u);
    CHECK(f.tpl[FSD_ACT_SEAT_DRIVER].seen, "mux 0 reached the driver seat slot");
    CHECK(!f.tpl[FSD_ACT_CAMERA].seen, "and not the camera slot");

    /* NULL is survivable: this runs on every frame on the bus. */
    CHECK(fsd_pipe_observe(NULL, 0x249u, LSTALK, 4u, 1000u) == 0u, "NULL store");
    CHECK(fsd_pipe_observe(&f, 0x249u, NULL, 4u, 1000u) == 0u, "NULL data");
    CHECK(fsd_pipe_observe(&f, 0x249u, LSTALK, 0u, 1000u) == 0u, "dlc 0");
    CHECK(fsd_pipe_observe(&f, 0x249u, LSTALK, 9u, 1000u) == 0u, "dlc 9");
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
    (void)fsd_pipe_observe(&f, 0x249u, LSTALK, 4u, now);

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
    (void)fsd_pipe_observe(&f, 0x249u, LSTALK, 4u, now);
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
    (void)fsd_pipe_observe(&f, 0x273u, BODY273, 8u, now);
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
    (void)fsd_pipe_observe(&f, 0x249u, LSTALK, 4u, 0u);
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
    (void)fsd_pipe_observe(&f, 0x249u, LSTALK, 4u, now - 500u);
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
    (void)fsd_pipe_observe(&f, 0x3C2u, mux29, 8u, now);
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
    (void)fsd_pipe_observe(&f, 0x249u, LSTALK, 4u, now);
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
    fsd_pipe_observe(&f, 0x249u, LSTALK, sizeof(LSTALK), now);

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

    /* 🔴 THE HALF THAT MATTERS. Take the belt off between frame two and
     * frame three of a burst and the rest must stop -- with a name. A burst
     * that bypassed the axis would be a rule that keeps acting on a car
     * nobody is sitting in. */
    in.driver_present = false;
    in.belt_latched = false;
    dirty(res);
    fsd_pipe_one(FSD_ACT_TURN_SIGNAL, 0, 3, &in, &f, now, &res[0]);
    CHECK(res[0].stage == FSD_PIPE_BLOCKED_BODY,
          "a repeat must face the axis, not skip it");
    CHECK(res[0].reason == (uint8_t)FSD_BODY_NO_DRIVER_PRESENT,
          "and it must say which gate, got %s",
          fsd_pipe_reason_str(res[0].stage, res[0].reason));

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
    (void)fsd_pipe_observe(&f, 0x3C2u, HORN_MUX0, 8u, 1000u);
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
    (void)fsd_pipe_observe(&hf, 0x3C2u, held, 8u, 1000u);
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
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
