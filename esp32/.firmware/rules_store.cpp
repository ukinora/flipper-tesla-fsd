/*
 * rules_store.cpp — see rules_store.h. One FsdRules, one NVS blob, no matcher.
 */

#include "rules_store.h"

/* Outside the guard: the no-op variant still answers on the serial console, and
 * a `Serial` that only exists in one branch is the fifth pattern waiting to
 * happen — it would build here and fail on the other seven boards. */
#include <Arduino.h>

#ifdef BLE_SERVER_ENABLED

#include <Preferences.h>

#include <string.h>

/* Its own namespace, like bleowner and powerlog. prefs.cpp holds the operator's
 * toggles and is rewritten whenever any one of them changes; folding 288 bytes
 * into that blob would mean rewriting the rules every time a checkbox moved,
 * and would make either side harder to reason about when it went wrong. */
static const char* NS = "fsdrules";
static const char* KEY = "tbl";

static Preferences g_nvs;

/* 🔴 No mutex, and that is a contract rather than an omission — see the header.
 * Everything that touches these runs on loop(). */
static FsdRules g_rules;
static uint32_t g_rev = 0;
static bool g_save_pending = false;

/* Blank means blank. An untouched table packs to 288 zero bytes (host-tested),
 * so "nothing configured" and "rule 0 says signal 0 does action 0" are not the
 * same picture on the phone. */
static void bump(void) {
    g_rev++;
    if(g_rev == 0) g_rev = 1; // 0 means "never seen"; do not hand it back
    g_save_pending = true;
}

/* Write the table and CHECK IT LANDED.
 *
 * 🔴 Read back rather than trusting the byte count, exactly as ble_owner.cpp
 * does. A full or worn NVS reports a short write, but a short write is not the
 * only way to end up with the wrong bytes on flash, and the failure has no
 * symptom at all until the next boot comes up with rules the owner thought were
 * saved. Checking costs one 288-byte read per change. */
static bool commit(void) {
    uint8_t blob[FSD_RULES_WIRE_LEN];
    fsd_rules_pack_all(&g_rules, blob);

    if(!g_nvs.begin(NS, /*readOnly=*/false)) return false;

    bool stored = false;
    if(g_nvs.putBytes(KEY, blob, sizeof(blob)) == sizeof(blob)) {
        uint8_t back[FSD_RULES_WIRE_LEN] = {};
        stored = g_nvs.getBytes(KEY, back, sizeof(back)) == sizeof(back) &&
                 memcmp(back, blob, sizeof(blob)) == 0;
    }
    g_nvs.end();
    return stored;
}

void rules_store_init(void) {
    fsd_rules_init(&g_rules);
    g_rev = 1;
    g_save_pending = false;

    uint8_t blob[FSD_RULES_WIRE_LEN] = {};
    bool have = false;
    if(g_nvs.begin(NS, /*readOnly=*/true)) {
        have = g_nvs.getBytes(KEY, blob, sizeof(blob)) == sizeof(blob);
        g_nvs.end();
    }
    if(!have) {
        Serial.println("[RULES] 저장된 매핑이 없다 — 24칸 전부 비어 있다");
        return;
    }

    uint8_t kept = 0, disabled = 0;
    for(uint8_t i = 0; i < FSD_RULE_MAX; i++) {
        FsdRule r;
        if(!fsd_rule_unpack(&blob[(size_t)i * FSD_RULE_WIRE_LEN], &r)) continue;

        /* Through fsd_rules_set(), never straight into the table. A blob written
         * by an older build can hold a signal id that has since come to mean
         * something else, and this is the first of the two places that catches
         * it (fsd_rules_match() is the second). */
        const FsdRuleVerdict v = fsd_rules_set(&g_rules, i, &r);
        if(v == FSD_RULE_OK) {
            if(r.enabled) kept++;
            continue;
        }

        /* 🔴 Keep the owner's fields, switch the rule OFF, and say so.
         *
         * Dropping it would silently delete something the owner built, and they
         * would find out by the rule not working. Storing it as-is would leave a
         * rule that cannot fire looking like one that can. Disabled keeps both
         * halves honest: the app still shows what was configured, and the module
         * still refuses to act on it.
         *
         * NVS is deliberately NOT rewritten here. The bytes on flash are the
         * owner's; a firmware that understands them again should find them
         * intact, and the next deliberate edit is what persists the change. */
        FsdRule off = r;
        off.enabled = false;
        fsd_rules_set(&g_rules, i, &off);
        disabled++;
        Serial.printf("[RULES] 🔴 %u번 매핑은 이 펌웨어에서 성립하지 않는다 (%s) — 꺼 둔 채로 둔다\n",
                      (unsigned)i, fsd_rule_verdict_str(v));
    }

    Serial.printf("[RULES] %u개 켜짐%s (24칸) — 'rules' 로 본다\n", (unsigned)kept,
                  disabled ? ", 성립하지 않아 끈 것 있음" : "");
}

/* How long to wait before trying a failed write again.
 *
 * 🔴 The RETRY is rate-limited, not just the log. An earlier version limited
 * only the message and left `commit()` running on every loop() — a permanently
 * broken NVS would then attempt a 288-byte flash write thousands of times a
 * second. Two things go wrong with that, and the second is the bad one:
 *
 *   1. it hammers a flash part that is already telling us it cannot take it;
 *   2. it stalls loop(), and a stalled loop drops CAN frames. That is measured,
 *      not theoretical — writing one capture cost 22,947 missed frames on can0
 *      and about 25,000 on can1. The one-shot capture before TSL comes out is
 *      exactly when this must not happen.
 *
 * A second is far below what a person would notice on a rule they just saved,
 * and far above loop rate. */
#define RULES_RETRY_MS 1000u

void rules_store_tick(uint32_t now_ms) {
    if(!g_save_pending) return;

    /* First attempt goes through immediately — the common case is a rule the
     * owner just saved, and it should be on flash within this loop. Only a
     * FAILED attempt starts the clock. */
    static uint32_t s_next_ms = 0;
    if(s_next_ms && (int32_t)(now_ms - s_next_ms) < 0) return;

    if(commit()) {
        g_save_pending = false;
        s_next_ms = 0;
        return;
    }

    s_next_ms = now_ms + RULES_RETRY_MS;
    if(!s_next_ms) s_next_ms = 1u; // 0 means "no wait"; do not hand it back

    /* 🔴 The flag stays set: loop() comes back and tries again. Rate-limit the
     * log so a permanently broken NVS does not bury everything else on the one
     * console this board has — but never go quiet, because until the module
     * reboots this failure has no other symptom. */
    static uint32_t s_warn_ms = 0;
    if(!s_warn_ms || (uint32_t)(now_ms - s_warn_ms) >= 5000u) {
        s_warn_ms = now_ms ? now_ms : 1u;
        Serial.println("[RULES] 🔴 NVS 저장 실패 — 계속 재시도한다. "
                       "지금 재부팅하면 방금 만든 매핑이 사라진다");
    }
}

void rules_store_erase_now(void) {
    fsd_rules_init(&g_rules);
    bump();

    if(g_nvs.begin(NS, /*readOnly=*/false)) {
        /* remove() reports false when the key was not there to begin with, and
         * that is success for us — absent is the goal however we get there. */
        g_nvs.remove(KEY);
        const bool gone = !g_nvs.isKey(KEY);
        g_nvs.end();
        if(gone) {
            g_save_pending = false;
            Serial.println("[RULES] 매핑을 전부 지웠다");
            return;
        }
    }
    /* The caller is a factory reset and reboots ~200 ms later, so there is no
     * later loop() to retry in. Report rather than block. Failing to erase is at
     * least the direction that does nothing new — the old rules stay, and none
     * of them can act while there is no emitter. */
    Serial.println("[RULES] 🔴 지우지 못했다 — 이전 매핑이 그대로 남는다");
}

void rules_store_render(uint8_t out[FSD_RULES_WIRE_LEN]) {
    fsd_rules_pack_all(&g_rules, out);
}

uint8_t rules_store_set_packed(uint8_t idx, const uint8_t in[FSD_RULE_WIRE_LEN]) {
    if(idx >= FSD_RULE_MAX) return (uint8_t)FSD_RULE_BAD_INDEX;

    FsdRule r;
    if(!fsd_rule_unpack(in, &r)) return (uint8_t)FSD_RULE_BAD_ARGS;

    const FsdRuleVerdict v = fsd_rules_set(&g_rules, idx, &r);
    if(v != FSD_RULE_OK) {
        Serial.printf("[RULES] %u번 거부: %s\n", (unsigned)idx, fsd_rule_verdict_str(v));
        return (uint8_t)v;
    }

    bump();
    const FsdSignalDef* d = fsd_signal_def(r.signal);
    Serial.printf("[RULES] %u번 저장: %s · %s -> %s (arg %ld)%s\n", (unsigned)idx,
                  d ? d->name : "?", fsd_trig_kind_str(r.kind),
                  fsd_body_action_str(r.action), (long)r.arg,
                  r.enabled ? "" : "  [꺼짐]");
    return (uint8_t)FSD_RULE_OK;
}

uint8_t rules_store_clear(uint8_t idx) {
    if(idx == 0xFFu) {
        fsd_rules_init(&g_rules);
        bump();
        Serial.println("[RULES] 매핑을 전부 지웠다");
        return (uint8_t)FSD_RULE_OK;
    }
    if(idx >= FSD_RULE_MAX) return (uint8_t)FSD_RULE_BAD_INDEX;

    /* One blank slot, built the way fsd_rules_init() builds them, so a cleared
     * slot and a never-touched slot are byte-identical on the wire. */
    FsdRules blank;
    fsd_rules_init(&blank);
    g_rules.rule[idx] = blank.rule[0];
    bump();
    Serial.printf("[RULES] %u번을 비웠다\n", (unsigned)idx);
    return (uint8_t)FSD_RULE_OK;
}

uint32_t rules_store_revision(void) { return g_rev; }

void rules_store_print(void) {
    uint8_t shown = 0;
    Serial.printf("[RULES] 저장된 매핑 (%u칸)\n", (unsigned)FSD_RULE_MAX);

    for(uint8_t i = 0; i < FSD_RULE_MAX; i++) {
        const FsdRule* r = fsd_rules_get(&g_rules, i);
        if(!r) continue;
        /* An empty slot is one nobody has started. A half-built rule -- off but
         * with a trigger chosen -- is shown, because "I made that and it is not
         * running" is exactly the question this command answers. */
        if(!r->enabled && r->kind == FSD_TRIG_NONE) continue;
        shown++;

        const FsdSignalDef* d = fsd_signal_def(r->signal);
        char sig[24];
        if(d) {
            snprintf(sig, sizeof(sig), "%s", d->name);
        } else {
            snprintf(sig, sizeof(sig), "?(%u)", (unsigned)r->signal);
        }

        /* The verdict is printed for every row, not only the refused ones. A
         * rule that is on AND valid says "ok" here, so a row that says anything
         * else is visibly the odd one out. */
        const FsdRuleVerdict v = fsd_rule_valid(r);
        Serial.printf("  %2u %s  %s · %s", (unsigned)i, r->enabled ? "켜짐" : "꺼짐", sig,
                      fsd_trig_kind_str(r->kind));
        if(r->kind == FSD_TRIG_STATE_ENTER || r->kind == FSD_TRIG_STATE_LEAVE ||
           r->kind == FSD_TRIG_DELTA) {
            Serial.printf(" %ld", (long)r->value);
        }
        Serial.printf(" -> %s (arg %ld)", fsd_body_action_str(r->action), (long)r->arg);
        if(v != FSD_RULE_OK) Serial.printf("   🔴 %s", fsd_rule_verdict_str(v));
        Serial.println();
    }

    if(shown == 0) Serial.println("  (비어 있다)");

    /* 🔴 Nothing in this build acts on a rule. Saying it on every print is the
     * cheap way to stop somebody concluding the car is broken because a rule
     * they can see stored did nothing. Delete this line the day an emitter
     * exists, and not before. */
    Serial.println("[RULES] ⚠️ 저장만 한다 — 아직 어떤 매핑도 차를 움직이지 않는다");
    if(g_save_pending) Serial.println("[RULES] 🔴 아직 플래시에 저장되지 않았다");
}

#else // no rule core on this variant

void rules_store_init(void) {}
void rules_store_tick(uint32_t) {}
void rules_store_erase_now(void) {}

/* Answer rather than ignore, the way the `wifi` command does on this build:
 * silence on a serial console reads as a hung port, not as a decision. */
void rules_store_print(void) {
    Serial.println("[RULES] 이 보드에는 매핑 엔진이 없다");
}

#endif // BLE_SERVER_ENABLED
