/*
 * rule_task.cpp — see rule_task.h.
 *
 * Threading: every entry point runs on the Arduino loop task, exactly as
 * body_task.cpp and camera_task.cpp do. rule_task_observe() has one caller
 * inside process_frame(), and the tick is called from loop(). Nothing here may
 * be called from a NimBLE callback.
 *
 * 🔴 WHAT IS AND IS NOT COVERED BY A TEST, because this file is Arduino and no
 * host test can reach a line of it. Everything that DECIDES lives elsewhere:
 * the rules (fsd_rules.c), the four gates (fsd_pipeline.c), when a frame goes
 * out (fsd_burst.c), what a frame contains (fsd_body_emit.c). What is left
 * here is glue -- and glue is where this file has been wrong twice:
 *
 *   2026-09-08  run_event() shipped frame one on the trigger's clock. The car
 *               found it: the mirror never reached the bus.
 *   2026-09-08  Removing that send also removed the stage check, so a REFUSED
 *               decision armed a burst -- and stamped the rate limiter, which
 *               would have locked out the next valid press. Reading found it,
 *               an hour after writing it.
 *
 * So: when editing this file, the question is not "do the tests pass". It is
 * "which of these lines is a decision that should not be here at all".
 */

#include "rule_task.h"

#ifdef BLE_SERVER_ENABLED

#include "../../fsd_logic/fsd_autonomy.h"
#include "../../fsd_logic/fsd_burst.h"
#include "../../fsd_logic/fsd_pipeline.h"
#include "body_task.h"    // the permission inputs, already assembled there
#include "rules_store.h"  // the owner's table

#include <Arduino.h>
#include <string.h>

/* How many events one call can carry. fsd_trig_on_frame() writes at most one
 * per signal the frame holds, and 0x3C2 mux 0 holds the most of any frame we
 * read. Overflow is dropped, not queued: a rule engine that acts on a stale
 * backlog is worse than one that misses an edge. */
#define RULE_EVENTS_MAX 8u

static FsdTriggers g_trig;
static FsdPipeFrames g_frames;
static FSDState* g_state = nullptr;
static portMUX_TYPE* g_mux = nullptr;
static RuleTaskSend g_send = nullptr;

/* 🔴 Session-only. Not in NVS, not in prefs.cpp, nowhere that survives a
 * power cut. See rule_task.h. */
static bool g_armed = false;

/* 🔴 THE BUS MOVED INTO THE RESULT (2026-09-08). It used to live here, as a
 * global set on every received frame -- "whichever channel spoke most
 * recently". That kept rule_task.h's promise only because ONE channel is
 * wired: with two, a template from Vehicle and a Party frame a millisecond
 * later would put the command on Party, and the chokepoint cannot catch that
 * because it compares bytes, not channels.
 *
 * FsdPipeResult.bus now carries the channel its TEMPLATE arrived on, decided
 * in fsd_logic where a host test can drive both. Found while preparing the
 * second channel; it was a defect waiting in the car, not one that had
 * happened yet. */

static uint32_t g_sent = 0;
static uint32_t g_refused = 0;
static char g_last_refusal[64] = "none";

/* A gesture's second half: the release, owed a fixed number of milliseconds
 * after a press that actually went out.
 *
 * 🔴 A TIMER, WHICH NOTHING ELSE IN THIS FILE IS. The burst below counts the
 * car's own frame arrivals on purpose -- 0x249 comes every 50 ms and that is
 * the cadence TSL uses. The light horn cannot work that way: its frame's mux 0
 * variant only arrives every 100 ms and the release is owed after 12. So this
 * one carries a due time and the loop delivers it.
 *
 * ⚠️ 12 ms IS WHAT TSL USED, NOT A DEADLINE WE CAN MISS DANGEROUSLY. Late is
 * harmless -- the frame is the car's own bytes -- and the requirement is only
 * that it beat the car's own next mux-0 frame, about 100 ms out. loop() runs
 * with no delay in it, so this lands within a loop pass of when it is due.
 *
 * Session state, like the arm flag and the burst. It dies with the power. */
static struct {
    FsdBodyAction action;
    int32_t arg;
    uint8_t rule_index;
    bool pending;
    uint32_t due_ms;
} g_release = {(FsdBodyAction)0, 0, 0, false, 0u};

void rule_task_init(FSDState* state, portMUX_TYPE* mux, RuleTaskSend send) {
    g_state = state;
    g_mux = mux;
    g_send = send;
    fsd_trig_init(&g_trig);
    fsd_pipe_init(&g_frames);
    g_armed = false;

    g_release.pending = false;
    g_sent = 0;
    g_refused = 0;
    strncpy(g_last_refusal, "none", sizeof(g_last_refusal) - 1);
    g_last_refusal[sizeof(g_last_refusal) - 1] = '\0';
}

/* 🔴 사람이 읽는 한국어는 "매핑" 이다 (차주 지시 2026-09-03). 코드 이름은
 * "rule" 그대로 — 파일명 · 함수 · [RULE] 태그 · rules/rulearm 명령 전부
 * 안 바꾼다. 한쪽만 바꾸면 화면과 소스의 대조가 끊긴다. rules_store.cpp 가
 * 같은 규칙을 따르고, 이 파일은 2026-09-07 에 새로 생기면서 그것을 놓쳐
 * 하루 동안 "규칙" 을 찍고 있었다. */
/* Decisions waiting for the car's next frame of their id.
 *
 * 🔴 THE CAR'S CLOCK, NOT OURS — for EVERY frame, including the first. That
 * last word is the 2026-09-08 fix: this used to ship frame one the instant the
 * switch was pressed and only the repeats on arrivals, which worked on 0x249
 * (50 ms) and could not work on 0x273 (500 ms). The blinker lighting up that
 * day was luck, not a result.
 *
 * The table itself is pure C in fsd_logic/fsd_burst.c, where a host test can
 * drive it. This file owns a bus; that one owns the clock. */
static FsdBurst g_burst;

void rule_task_set_armed(bool armed) {
    if (g_armed == armed) return;
    g_armed = armed;
    /* Disarming stops a burst mid-flight. The axis would refuse the rest
     * anyway (NOT_ENABLED), but "stop" should not depend on a gate further
     * down agreeing with it.
     *
     * 🔴 AND SINCE 2026-09-08 IT DROPS THE WHOLE COMMAND, not just its tail:
     * no frame goes out before the car's next one, so a press that has not
     * seen an arrival yet has written nothing at all. That is the safer
     * direction and worth saying, because it means "lock" now cancels
     * commands that used to be half-sent. */
    fsd_burst_reset(&g_burst);
    /* 🔴 AND IT DROPS A RELEASE THAT WAS OWED, which is the one place where
     * "stop" costs something: the horn stays pressed until the car's own next
     * mux-0 frame, up to 100 ms. That is the right trade -- disarm means stop
     * writing to the bus, and 100 ms of a bit the car will contradict by
     * itself is a smaller thing than a rule that keeps writing after the
     * operator said no. */
    g_release.pending = false;
    /* 🔴 THIS SAID "무장/해제" UNTIL 2026-09-08, when the owner asked why.
     * It was a literal rendering of `armed`. In English that word is neutral
     * safety-engineering vocabulary only because of long usage -- you arm an
     * alarm -- and the usage does not travel. In Korean 무장 is military and
     * nothing else, so the translation kept the letter and dropped the sense.
     *
     * 🔴 Worse, one word was carrying four different jobs: this switch, the
     * capability table's armable_at_runtime (a property of an action, not a
     * state), the blackbox capture window, and the camera policy's ARMED
     * phase -- and the last two can appear on the same phone screen. The
     * visible words now name what each one actually does.
     *
     * 🔴 The identifiers do not change. g_armed, rule_task_set_armed() and
     * the `rulearm` command keep their names, for the reason OpMode keeps
     * "Active": the command is typed, and the field procedure quotes it as
     * an anchor. Renaming the visible half and leaving the anchor is the
     * decision, not an oversight. */
    Serial.printf("[RULE] %s\n", armed
        ? "송신 허용 — 매핑이 실제로 CAN 에 씁니다 (이 세션에만, 전원과 함께 꺼집니다)"
        : "송신 잠금 — 매핑은 판정만 하고 아무것도 보내지 않습니다");
}

bool rule_task_armed(void) {
    return g_armed;
}

/* The permission inputs, with the session switch folded in.
 *
 * body_task.cpp already assembles every field from the shared state and its
 * own detectors, and duplicating that here is how the two would drift. What
 * this adds is the one thing body_task has no opinion about: whether the
 * operator armed the engine. Disarmed, every action_enabled stays false and
 * the axis answers NOT_ENABLED — the same refusal a disabled rule would get,
 * with a name. */
/* `emitting` is the action whose frame is going out right now, or
 * FSD_ACT_COUNT on the press path where nothing is. It exempts that one action
 * from min_interval_ms -- see fsd_burst_fill_last_act(), which owns the reason.
 *
 * 🔴 body_task.cpp memsets last_act_ms and nothing else fills it, so before
 * 2026-09-08 every min_interval_ms in FSD_BODY_CAPS was decorative: four gates
 * advertised, three enforced. The record lives in the burst table because that
 * table already is the list of commands accepted. */
static FsdBodyInputs rule_inputs(uint32_t now_ms, unsigned emitting) {
    FsdBodyInputs in = body_task_permission_inputs(now_ms);
    if (g_armed) {
        for (unsigned a = 0; a < FSD_ACT_COUNT; a++) in.action_enabled[a] = true;
    }
    fsd_burst_fill_last_act(&g_burst, in.last_act_ms, emitting);
    return in;
}

static void note_refusal(const FsdPipeResult* r) {
    g_refused++;
    snprintf(g_last_refusal, sizeof(g_last_refusal), "%s: %s",
             fsd_pipe_stage_str(r->stage), fsd_pipe_reason_str(r->stage, r->reason));
}

/* Push one trigger event through the pipeline and send whatever survives.
 *
 * 🔴 THIS IS THE ONLY PLACE IN THE FIRMWARE THAT PUTS A BODY FRAME ON THE BUS.
 * fsd_pipe_run() decides and builds; it has no way to transmit. The send below
 * is a call back into main.cpp, which still applies its own mode gate and its
 * own ID refusals afterwards. Two more layers after the four. */
/* Ship one pipeline result. Shared by the rule path and the burst path so
 * they cannot drift: same refusal names, same counters, same bus guard. */
static bool ship(const FsdPipeResult* r, uint32_t now_ms, const char* what) {
    if (r->stage != FSD_PIPE_OK) {
        note_refusal(r);
        /* Every refusal is logged. This path is rare by construction -- a
         * trigger the owner wired to an action -- so it cannot flood, and
         * "why did nothing happen" is the question this feature will be
         * asked most often. */
        Serial.printf("[RULE] 매핑 %u %s %s 거부 — %s (%s)\n", (unsigned)r->rule_index,
                      fsd_body_action_str(r->action), what, fsd_pipe_stage_str(r->stage),
                      fsd_pipe_reason_str(r->stage, r->reason));
        return false;
    }

    if (!g_send) return false;
    if (r->bus == FSD_PIPE_BUS_NONE) {
        /* No frame has arrived, so we do not know which bus to answer
         * on. fsd_pipe_run() cannot reach this state -- the emitter
         * needs a template first -- but guessing a bus is exactly the
         * mistake this field exists to prevent. */
        g_refused++;
        Serial.println("[RULE] 어느 버스로 보낼지 모른다 — 프레임을 받은 적이 없다");
        return false;
    }

    const bool ok = g_send(r->bus, r->frame.id, r->frame.data, r->frame.dlc);
    if (ok) {
        g_sent++;
        Serial.printf("[RULE] 매핑 %u %s -> 0x%03X %s\n", (unsigned)r->rule_index,
                      fsd_body_action_str(r->action), (unsigned)r->frame.id, what);
    } else {
        /* main.cpp refused it after we did not. Counted as a refusal
         * because that is what it is, and named so the two are not
         * confused: this one came from below the pipeline. */
        g_refused++;
        snprintf(g_last_refusal, sizeof(g_last_refusal), "bus: 0x%03X 거부됨",
                 (unsigned)r->frame.id);
        Serial.printf("[RULE] 매핑 %u %s -> 0x%03X 버스가 거부\n",
                      (unsigned)r->rule_index, fsd_body_action_str(r->action),
                      (unsigned)r->frame.id);
    }

    /* Tell the trigger layer we just disturbed these signals, so our own
     * write does not come back as an event. A rule whose action changes
     * the state it triggers on would otherwise run forever. */
    FsdSignal touched[FSD_RULE_MAX_AFFECTS];
    const uint8_t m = fsd_rule_affects(r->action, touched, FSD_RULE_MAX_AFFECTS);
    for (uint8_t k = 0; k < m; k++) fsd_trig_disturbed(&g_trig, touched[k], now_ms);
    return ok;
}

/* Arm the release, if this action has one. Called after a press that actually
 * reached the bus, never after a release -- a release that armed another one
 * would beep forever. */
static void arm_release(const FsdPipeResult* r, uint32_t now_ms) {
    const uint16_t ms = fsd_emit_release_ms(r->action);
    if (ms == 0u) return;
    g_release.action = r->action;
    g_release.arg = r->arg;
    g_release.rule_index = r->rule_index;
    g_release.due_ms = now_ms + ms;
    g_release.pending = true;
}

/* Send the release, if one is due.
 *
 * 🔴 THE DECISION IS NOT MADE HERE. fsd_pipe_release() runs the emitter and
 * the bit-granularity chokepoint, exactly as fsd_pipe_one() does for a press,
 * and it is the one that explains why the permission axis is skipped and the
 * chokepoint is not. This function owns a clock and a counter; that is all.
 *
 * ⚠️ AN EARLIER VERSION OF THIS DID DECIDE, and got it wrong in two ways a
 * review caught: it went from the emitter straight to the bus with a
 * hand-written memcmp -- skipping fsd_body_wire_check(), which every other
 * write in this firmware goes through -- and its comment justified skipping
 * the axis by claiming the rate limit would refuse the release, which was not
 * true of that build: last_act_ms had no producer and every min_interval_ms
 * was decorative. Both halves moved into fsd_logic/ where a host test can
 * stand.
 *
 * ⚠️ The limiter IS enforced since 2026-09-08, so the claim is now arguable --
 * and the skip is still not justified by it. The release is skipped because it
 * is the second half of one gesture, not because of what any gate would
 * answer; see fsd_pipe_release().
 *
 * What still stands in front of the frame after this: the arm flag and the bus
 * guard here, then send_on_bus()'s mode gate and id refusals in main.cpp. */
static void release_due(uint32_t now_ms) {
    if (!g_release.pending) return;
    /* Signed, so the millisecond counter wrapping does not make a due release
     * wait another 49 days. */
    if ((int32_t)(now_ms - g_release.due_ms) < 0) return;
    g_release.pending = false;

    /* 🔴 Disarming between the press and the release stops the release. The
     * horn then stays pressed until the car's own next mux-0 frame, up to
     * 100 ms -- the right trade, because "stop" has to mean stop writing. */
    if (!g_armed) return;

    FsdPipeResult r;
    fsd_pipe_release(g_release.action, g_release.arg, g_release.rule_index, &g_frames,
                     now_ms, &r);
    (void)ship(&r, now_ms, "놓음");
}

static void run_event(const FsdTriggerEvent* ev, uint32_t now_ms) {
    const FsdRules* rules = rules_store_table();
    if (!rules) return;

    /* Nothing is being emitted here, so nothing is exempt: this is the one
     * place min_interval_ms is asked, and a press inside the interval is
     * refused before a burst can be armed. */
    const FsdBodyInputs in = rule_inputs(now_ms, FSD_ACT_COUNT);

    FsdPipeResult out[FSD_PIPE_MAX_OUT];
    memset(out, 0, sizeof(out));
    /* 🔴 THE PRESS DECIDES; IT DOES NOT BUILD. Until 2026-09-09 this was
     * fsd_pipe_run(), which ran all four layers here -- and the chokepoint's
     * freshness question made every press on a 500 ms frame a 40 % coin flip.
     * The car measured it: the mirror moved on 7 of 19 presses. Nothing was
     * wrong with the frames that went out; they were simply the ones whose
     * finger happened to land inside the window.
     *
     * fsd_pipe_decide() takes no template store, so the question cannot be
     * asked here even by accident. It is asked below, in burst_on_frame(),
     * where the answer is always yes because that IS the arrival. */
    const uint8_t n = fsd_pipe_decide(rules, ev, &in, now_ms, out, FSD_PIPE_MAX_OUT);

    for (uint8_t i = 0; i < n; i++) {
        /* 🔴 NOTHING IS SENT HERE. This used to ship frame one immediately and
         * arm only the repeats; 2026-09-08 in the car showed why that cannot
         * work — see fsd_burst.h. Every frame now waits for the car's next one
         * of this id, which is both where TSL puts its frames and the only
         * moment the emitter's template is fresh.
         *
         * The decision is still made HERE, at the press: which rules matched
         * is a question about the event. Only the writing moved. */
        /* 🔴🔴 A REFUSAL ARMS NOTHING. Dropping the ship() call that used to
         * stand here removed the only thing that separated an accepted
         * decision from a refused one -- and arming on a refusal is worse
         * than sending on one: fsd_burst_arm() STAMPS last_act_ms, so a
         * command the axis just rejected would lock the next, valid press out
         * for the whole interval. It would also hold a slot until the
         * deadline, and report its reason 500 ms late instead of under the
         * finger that caused it.
         *
         * Found while wiring the rate limiter -- the same commit that made the
         * stamp mean anything is the one that made this dangerous. */
        if (out[i].stage != FSD_PIPE_OK) {
            note_refusal(&out[i]);
            Serial.printf("[RULE] 매핑 %u %s 거부 — %s: %s\n",
                          (unsigned)out[i].rule_index,
                          fsd_body_action_str(out[i].action),
                          fsd_pipe_stage_str(out[i].stage),
                          fsd_pipe_reason_str(out[i].stage, out[i].reason));
            continue;
        }

        const FsdBodyWire* w = fsd_body_wire(out[i].action);
        if (!w) {
            /* Every action in the enum has a row, so this is unreachable
             * today. It stays because the next person to add an action meets
             * it here rather than finding a command that silently never
             * fires. */
            g_refused++;
            snprintf(g_last_refusal, sizeof(g_last_refusal),
                     "chokepoint: 이 동작에 행이 없다");
            continue;
        }
        if (!fsd_burst_arm(&g_burst, out[i].action, out[i].arg, out[i].rule_index,
                           w->can_id, fsd_emit_repeat(out[i].action), now_ms)) {
            g_refused++;
            snprintf(g_last_refusal, sizeof(g_last_refusal),
                     "burst: 대기열이 가득 찼다");
            Serial.printf("[RULE] 매핑 %u %s -> 대기열이 가득 찼다 (동시에 %u개까지)\n",
                          (unsigned)out[i].rule_index,
                          fsd_body_action_str(out[i].action), (unsigned)FSD_BURST_MAX);
        }
    }
}

/* The car's frame just landed. If anything is owed on this id, put ours right
 * behind it -- 0-1 ms after the frame we copied, which is where TSL's are.
 *
 * 🔴 THIS IS NOW THE ONLY PLACE A PRESS REACHES THE BUS. It used to handle
 * frames 2..N while run_event() sent the first one on the trigger's clock. */
static void burst_on_frame(uint32_t can_id, uint32_t now_ms) {
    if (!g_state || !g_mux) return;

    FsdBurstDue due;
    if (!fsd_burst_on_frame(&g_burst, can_id, now_ms, &due)) return;

    /* Exempt this action: the command it belongs to already passed the
     * interval at the press, and the indicator's own row (50 ms) would
     * otherwise refuse frames two through four of its own burst. */
    const FsdBodyInputs in = rule_inputs(now_ms, (unsigned)due.action);
    FsdPipeResult r;
    memset(&r, 0, sizeof(r));
    fsd_pipe_one(due.action, due.arg, due.rule_index, &in, &g_frames, now_ms, &r);
    /* A refusal here stops nothing by itself -- the slot is already spent, so
     * the burst runs out on its own. What it does is name why, which is the
     * whole point of doing it through the pipeline instead of around it. */
    if (ship(&r, now_ms, "보냄")) arm_release(&r, now_ms);
}

void rule_task_observe(uint8_t bus, uint32_t can_id, const uint8_t* data, uint8_t dlc,
                       uint32_t now_ms) {
    if (!data) return;

    /* Store first. A frame that is both a trigger and a template — 0x3C2 is
     * both — should be available to the emitter as of THIS frame, not the
     * previous one. */
    (void)fsd_pipe_observe(&g_frames, bus, can_id, data, dlc, now_ms);

    /* A release owed on a 12 ms clock is checked here as well as in the tick,
     * because a busy bus is exactly when a loop pass gets long -- and this
     * runs once per frame. AFTER the store, so the release echoes the car's
     * most recent statement rather than the one before it. */
    release_due(now_ms);

    /* 🔴 BURST FIRST, THEN TRIGGERS. The template this frame just became is
     * the one our burst frame copies, and putting it out now -- in the same
     * millisecond -- is what reproduces TSL's spacing. Doing triggers first
     * would work too, but it would put a rule match between the car's frame
     * and ours for no reason. */
    burst_on_frame(can_id, now_ms);

    FsdTriggerEvent ev[RULE_EVENTS_MAX];
    const uint8_t n = fsd_trig_on_frame(&g_trig, can_id, data, dlc, now_ms, ev, RULE_EVENTS_MAX);
    for (uint8_t i = 0; i < n; i++) run_event(&ev[i], now_ms);
}

void rule_task_tick(uint32_t now_ms) {
    if (!g_state || !g_mux) return;
    /* The guaranteed path: a quiet bus delivers no frames, so rule_task_observe()
     * would never run and a release would sit pending forever. */
    release_due(now_ms);
    /* 🔴 AND THE SAME ARGUMENT NOW APPLIES TO THE PRESS ITSELF. Since every
     * frame waits for the car, a bus that goes quiet leaves a command armed
     * with nobody behind it. fsd_burst_tick() drops those; without this call
     * the deadline in fsd_burst.h would be a comment rather than a rule. */
    const uint8_t gone = fsd_burst_tick(&g_burst, now_ms);
    if (gone) {
        Serial.printf("[RULE] 대기 중이던 명령 %u개를 버렸다 — 그 프레임이 %u ms 동안 "
                      "안 왔다\n", (unsigned)gone, (unsigned)FSD_BURST_MAX_WAIT_MS);
    }
    FsdTriggerEvent ev[RULE_EVENTS_MAX];
    const uint8_t n = fsd_trig_tick(&g_trig, now_ms, ev, RULE_EVENTS_MAX);
    for (uint8_t i = 0; i < n; i++) run_event(&ev[i], now_ms);
}

uint32_t rule_task_sent(void) {
    return g_sent;
}

uint32_t rule_task_refused(void) {
    return g_refused;
}

const char* rule_task_last_refusal(void) {
    return g_last_refusal;
}

void rule_task_print(void) {
    Serial.printf("[RULE] %s · 보냄 %u · 거부 %u · 마지막 거부: %s\n",
                  g_armed ? "송신 허용" : "송신 잠금", (unsigned)g_sent, (unsigned)g_refused,
                  g_last_refusal);
    /* 🔴 WITHOUT THIS LINE "보냄 0" HAS TWO MEANINGS since 2026-09-08: nothing
     * was decided, or something was decided and is still waiting for the car.
     * At the car those look identical and the second one is not a failure. */
    Serial.printf("[RULE] 차의 프레임을 기다리는 명령 %u개 · 기다리다 버린 것 %u · "
                  "대기열이 가득 차 못 받은 것 %u\n",
                  (unsigned)fsd_burst_pending(&g_burst), (unsigned)g_burst.expired,
                  (unsigned)g_burst.dropped);
    if (!g_armed) {
        Serial.println("[RULE] 허용하려면 'rulearm on'. 이 세션에만 유효하고 "
                       "전원이 끊기면 꺼집니다.");
    }
}

#endif // BLE_SERVER_ENABLED
