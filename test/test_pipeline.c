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

    /* 🔴 An action with no wire row gets no template, and that is what makes
     * the ordering inside fsd_pipe_run() necessary rather than tidy. */
    CHECK(!f.tpl[FSD_ACT_MAP_LIGHT].seen, "map light: no row, so no template");
    CHECK(fsd_pipe_observe(&f, 0x273u, BODY273, 8u, 1000u) == 0u,
          "0x273 arrives and is stored nowhere");

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

    /* 2. The chokepoint's missing row, for an action whose emitter works.
     *
     * 🔴 THIS IS THE ORDERING TEST. The map light's frame arrives every 500 ms
     * and fsd_emit_build() can produce it, so if the row check came after the
     * emitter this would say NO_TEMPLATE and send a person to look at the bus.
     * It has to say NO_ROW, which is a decision nobody has made. */
    one_rule(&rules, FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, FSD_ACT_MAP_LIGHT, 0);
    fsd_pipe_init(&f);
    (void)fsd_pipe_observe(&f, 0x273u, BODY273, 8u, now);
    in = good_inputs(now);
    const FsdTriggerEvent map_ev = ev_of(FSD_SIG_MAP_SW_FL, FSD_TRIG_PRESS, 0, now);
    dirty(out);
    CHECK(fsd_pipe_run(&rules, &map_ev, &in, &f, now, out, FSD_PIPE_MAX_OUT) == 1u, "one result");
    CHECK(out[0].stage == FSD_PIPE_BLOCKED_WIRE, "map light stops at the chokepoint, got %s",
          fsd_pipe_stage_str(out[0].stage));
    CHECK(out[0].reason == (uint8_t)FSD_WIRE_NO_ROW, "named NO_ROW, got %s",
          fsd_pipe_reason_str(out[0].stage, out[0].reason));

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

int main(void) {
    printf("test_pipeline: rule -> axis -> emitter -> chokepoint\n");
    test_observe();
    test_turn_signal_end_to_end();
    test_each_layer_refuses();
    test_quiet_cases();
    test_names();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
