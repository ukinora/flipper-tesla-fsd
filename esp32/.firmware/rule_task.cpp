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

static uint32_t g_sent = 0;
static uint32_t g_refused = 0;
static char g_last_refusal[64] = "none";

void rule_task_init(FSDState* state, portMUX_TYPE* mux, RuleTaskSend send) {
    g_state = state;
    g_mux = mux;
    g_send = send;
    fsd_trig_init(&g_trig);
    fsd_pipe_init(&g_frames);
    g_armed = false;
    g_sent = 0;
    g_refused = 0;
    strncpy(g_last_refusal, "none", sizeof(g_last_refusal) - 1);
    g_last_refusal[sizeof(g_last_refusal) - 1] = '\0';
}

void rule_task_set_armed(bool armed) {
    if (g_armed == armed) return;
    g_armed = armed;
    Serial.printf("[RULE] %s\n", armed
        ? "무장됨 — 규칙이 실제로 CAN 에 씁니다 (이 세션에만, 전원과 함께 꺼집니다)"
        : "해제됨 — 규칙은 판정만 하고 아무것도 보내지 않습니다");
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
static void run_event(const FsdTriggerEvent* ev, uint32_t now_ms) {
    const FsdRules* rules = rules_store_table();
    if (!rules) return;

    const FsdBodyInputs in = rule_inputs(now_ms);

    FsdPipeResult out[FSD_PIPE_MAX_OUT];
    memset(out, 0, sizeof(out));
    const uint8_t n = fsd_pipe_run(rules, ev, &in, &g_frames, now_ms, out, FSD_PIPE_MAX_OUT);

    for (uint8_t i = 0; i < n; i++) {
        if (out[i].stage != FSD_PIPE_OK) {
            note_refusal(&out[i]);
            /* Every refusal is logged. This path is rare by construction — a
             * trigger the owner wired to an action — so it cannot flood, and
             * "why did nothing happen" is the question this feature will be
             * asked most often. */
            Serial.printf("[RULE] 규칙 %u %s 거부 — %s (%s)\n", (unsigned)out[i].rule_index,
                          fsd_body_action_str(out[i].action), fsd_pipe_stage_str(out[i].stage),
                          fsd_pipe_reason_str(out[i].stage, out[i].reason));
            continue;
        }

        if (!g_send) continue;
        const bool ok = g_send(out[i].frame.id, out[i].frame.data, out[i].frame.dlc);
        if (ok) {
            g_sent++;
            Serial.printf("[RULE] 규칙 %u %s -> 0x%03X 보냄\n", (unsigned)out[i].rule_index,
                          fsd_body_action_str(out[i].action), (unsigned)out[i].frame.id);
        } else {
            /* main.cpp refused it after we did not. Counted as a refusal
             * because that is what it is, and named so the two are not
             * confused: this one came from below the pipeline. */
            g_refused++;
            snprintf(g_last_refusal, sizeof(g_last_refusal), "bus: 0x%03X 거부됨",
                     (unsigned)out[i].frame.id);
            Serial.printf("[RULE] 규칙 %u %s -> 0x%03X 버스가 거부\n",
                          (unsigned)out[i].rule_index, fsd_body_action_str(out[i].action),
                          (unsigned)out[i].frame.id);
        }

        /* Tell the trigger layer we just disturbed these signals, so our own
         * write does not come back as an event. A rule whose action changes
         * the state it triggers on would otherwise run forever. */
        FsdSignal touched[FSD_RULE_MAX_AFFECTS];
        const uint8_t m = fsd_rule_affects(out[i].action, touched, FSD_RULE_MAX_AFFECTS);
        for (uint8_t k = 0; k < m; k++) fsd_trig_disturbed(&g_trig, touched[k], now_ms);
    }
}

void rule_task_observe(uint32_t can_id, const uint8_t* data, uint8_t dlc, uint32_t now_ms) {
    if (!data) return;

    /* Store first. A frame that is both a trigger and a template — 0x3C2 is
     * both — should be available to the emitter as of THIS frame, not the
     * previous one. */
    (void)fsd_pipe_observe(&g_frames, can_id, data, dlc, now_ms);

    FsdTriggerEvent ev[RULE_EVENTS_MAX];
    const uint8_t n = fsd_trig_on_frame(&g_trig, can_id, data, dlc, now_ms, ev, RULE_EVENTS_MAX);
    for (uint8_t i = 0; i < n; i++) run_event(&ev[i], now_ms);
}

void rule_task_tick(uint32_t now_ms) {
    if (!g_state || !g_mux) return;
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
