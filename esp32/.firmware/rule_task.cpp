/*
 * rule_task.cpp — see rule_task.h.
 *
 * Threading: every entry point runs on the Arduino loop task, exactly as
 * body_task.cpp and camera_task.cpp do. rule_task_observe() has one caller
 * inside process_frame(), and the tick is called from loop(). Nothing here may
 * be called from a NimBLE callback.
 */

#include "rule_task.h"

#ifdef BLE_SERVER_ENABLED

#include "../../fsd_logic/fsd_autonomy.h"
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

/* Which bus the templates came in on. 🔴 A command goes back out the
 * same way: see rule_task.h. 0xFF until the first frame arrives, and a
 * command decided before that is refused rather than guessed at. */
static uint8_t g_bus = 0xFFu;

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
    g_bus = 0xFFu;
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
/* A burst in progress: the same decision, waiting for the car's next frames.
 *
 * 🔴 THE CAR'S CLOCK, NOT OURS. remaining counts DOWN on arrivals of the frame
 * this action writes, so our frame lands 0-1 ms after the car's own -- the
 * spacing TSL uses and the reason there is no timer here. A timer would drift
 * against a 50 ms bus and put us in the middle of a gap, which is where the
 * first car attempt died.
 *
 * Session state, not settings. It dies with the power like the arm flag, and
 * anything that stops the burst early (a refusal, a mode change) just leaves
 * remaining at whatever it was -- the next arrival tries again and gets a named
 * refusal, which is the honest outcome. */
static struct {
    FsdBodyAction action;
    int32_t arg;
    uint8_t rule_index;
    uint8_t remaining;
    uint32_t id;      /* the CAN id whose arrival drives it */
} g_burst = {(FsdBodyAction)0, 0, 0, 0u, 0u};

void rule_task_set_armed(bool armed) {
    if (g_armed == armed) return;
    g_armed = armed;
    /* Disarming stops a burst mid-flight. The axis would refuse the rest
     * anyway (NOT_ENABLED), but "stop" should not depend on a gate further
     * down agreeing with it. */
    g_burst.remaining = 0u;
    /* 🔴 AND IT DROPS A RELEASE THAT WAS OWED, which is the one place where
     * "stop" costs something: the horn stays pressed until the car's own next
     * mux-0 frame, up to 100 ms. That is the right trade -- disarm means stop
     * writing to the bus, and 100 ms of a bit the car will contradict by
     * itself is a smaller thing than a rule that keeps writing after the
     * operator said no. */
    g_release.pending = false;
    Serial.printf("[RULE] %s\n", armed
        ? "무장됨 — 매핑이 실제로 CAN 에 씁니다 (이 세션에만, 전원과 함께 꺼집니다)"
        : "해제됨 — 매핑은 판정만 하고 아무것도 보내지 않습니다");
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
static FsdBodyInputs rule_inputs(uint32_t now_ms) {
    FsdBodyInputs in = body_task_permission_inputs(now_ms);
    if (g_armed) {
        for (unsigned a = 0; a < FSD_ACT_COUNT; a++) in.action_enabled[a] = true;
    }
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
    if (g_bus == 0xFFu) {
        /* No frame has arrived, so we do not know which bus to answer
         * on. fsd_pipe_run() cannot reach this state -- the emitter
         * needs a template first -- but guessing a bus is exactly the
         * mistake this field exists to prevent. */
        g_refused++;
        Serial.println("[RULE] 어느 버스로 보낼지 모른다 — 프레임을 받은 적이 없다");
        return false;
    }

    const bool ok = g_send(g_bus, r->frame.id, r->frame.data, r->frame.dlc);
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
 * the axis by claiming the rate limit would refuse the release, which is not
 * true of this build (last_act_ms has no producer). Both halves moved into
 * fsd_logic/ where a host test can stand.
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

    const FsdBodyInputs in = rule_inputs(now_ms);

    FsdPipeResult out[FSD_PIPE_MAX_OUT];
    memset(out, 0, sizeof(out));
    const uint8_t n = fsd_pipe_run(rules, ev, &in, &g_frames, now_ms, out, FSD_PIPE_MAX_OUT);

    for (uint8_t i = 0; i < n; i++) {
        if (!ship(&out[i], now_ms, "보냄")) continue;

        /* 🔴 AND ONE FRAME IS SOMETIMES ONLY HALF THE COMMAND. A gesture owes
         * a release; everything else owes nothing and this returns at once. */
        arm_release(&out[i], now_ms);

        /* 🔴 ONE FRAME IS NOT ALWAYS THE COMMAND. Arm the rest of the burst;
         * the car's next frames of this id drive it. See fsd_emit_repeat(). */
        const uint8_t reps = fsd_emit_repeat(out[i].action);
        const FsdBodyWire* w = fsd_body_wire(out[i].action);
        if (reps > 1u && w) {
            g_burst.action = out[i].action;
            g_burst.arg = out[i].arg;
            g_burst.rule_index = out[i].rule_index;
            g_burst.remaining = (uint8_t)(reps - 1u);
            g_burst.id = w->can_id;
        }
    }
}

/* The car's frame just landed. If a burst is owed on this id, put ours in
 * front of the next one -- 0-1 ms behind the frame we copied, which is where
 * TSL's are. */
static void burst_on_frame(uint32_t can_id, uint32_t now_ms) {
    if (g_burst.remaining == 0u || can_id != g_burst.id) return;
    if (!g_state || !g_mux) return;

    g_burst.remaining--;

    const FsdBodyInputs in = rule_inputs(now_ms);
    FsdPipeResult r;
    memset(&r, 0, sizeof(r));
    fsd_pipe_one(g_burst.action, g_burst.arg, g_burst.rule_index, &in, &g_frames,
                 now_ms, &r);
    /* A refusal here stops nothing by itself -- remaining is already down, so
     * the burst runs out on its own. What it does is name why, which is the
     * whole point of doing it through the pipeline instead of around it. */
    if (ship(&r, now_ms, "보냄")) arm_release(&r, now_ms);
}

void rule_task_observe(uint8_t bus, uint32_t can_id, const uint8_t* data, uint8_t dlc,
                       uint32_t now_ms) {
    if (!data) return;
    g_bus = bus;

    /* Store first. A frame that is both a trigger and a template — 0x3C2 is
     * both — should be available to the emitter as of THIS frame, not the
     * previous one. */
    (void)fsd_pipe_observe(&g_frames, can_id, data, dlc, now_ms);

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
                  g_armed ? "무장됨" : "해제됨", (unsigned)g_sent, (unsigned)g_refused,
                  g_last_refusal);
    if (!g_armed) {
        Serial.println("[RULE] 무장하려면 'rulearm on'. 이 세션에만 유효하고 "
                       "전원이 끊기면 꺼집니다.");
    }
}

#endif // BLE_SERVER_ENABLED
