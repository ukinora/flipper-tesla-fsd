/*
 * fsd_trigger.c — see fsd_trigger.h.
 */

#include "fsd_trigger.h"

/* Switch signals occupy button slots in table order. The mapping is computed
 * rather than written down: a second table would be a second thing to keep in
 * step with fsd_signal.c, and this one cannot drift. */
static int switch_slot(FsdSignal s) {
    int slot = 0;
    for(int i = 0; i < FSD_SIG_COUNT; i++) {
        const FsdSignalDef* d = fsd_signal_def((FsdSignal)i);
        if(!d || d->kind != FSD_SIGK_SWITCH) continue;
        if((FsdSignal)i == s) return slot;
        slot++;
    }
    return -1;
}

static int switch_count(void) {
    int n = 0;
    for(int i = 0; i < FSD_SIG_COUNT; i++) {
        const FsdSignalDef* d = fsd_signal_def((FsdSignal)i);
        if(d && d->kind == FSD_SIGK_SWITCH) n++;
    }
    return n;
}

/* One slot number spread over several banks, because FSD_BTN_MAX cannot hold
 * them all. Kept in one place so no call site has to know the arithmetic. */
static FsdButtons* bank_of(FsdTriggers* t, int slot) {
    if(!t || slot < 0 || slot >= FSD_TRIG_SWITCH_COUNT) return NULL;
    return &t->switches[slot / FSD_BTN_MAX];
}

static uint8_t index_in_bank(int slot) {
    return (uint8_t)(slot % FSD_BTN_MAX);
}

static FsdTriggerKind kind_of(FsdBtnEvent e) {
    switch(e) {
    case FSD_BTN_EV_SHORT: return FSD_TRIG_PRESS;
    case FSD_BTN_EV_LONG: return FSD_TRIG_LONG;
    case FSD_BTN_EV_DOUBLE: return FSD_TRIG_DOUBLE;
    case FSD_BTN_EV_STUCK: return FSD_TRIG_STUCK;
    case FSD_BTN_EV_NONE: break;
    }
    return FSD_TRIG_NONE;
}

static void emit(FsdTriggerEvent* out, uint8_t max_out, uint8_t* n, FsdSignal s,
                 FsdTriggerKind k, int32_t v, uint32_t now_ms) {
    if(!out || *n >= max_out) return;
    out[*n].signal = s;
    out[*n].kind = k;
    out[*n].value = v;
    out[*n].at_ms = now_ms;
    (*n)++;
}

void fsd_trig_init(FsdTriggers* t) {
    if(!t) return;
    for(int i = 0; i < FSD_SIG_COUNT; i++) {
        t->seen[i] = false;
        t->last[i] = 0;
        t->quiet_until[i] = 0;
        t->quiet_armed[i] = false;
    }
    for(int b = 0; b < FSD_TRIG_BANKS; b++) fsd_btn_init(&t->switches[b]);

    /* 🔴 The one place car switches are declared LEVEL. fsd_button.c defaults
     * to EVENT because getting it wrong the other way wedges a button until
     * reboot; here we measured it, so we say so — for all of them, from the
     * table, so none can be forgotten. */
    for(int i = 0; i < FSD_SIG_COUNT; i++) {
        const FsdSignalDef* d = fsd_signal_def((FsdSignal)i);
        if(!d || d->kind != FSD_SIGK_SWITCH) continue;
        const int slot = switch_slot((FsdSignal)i);
        FsdButtons* bank = bank_of(t, slot);
        if(bank) fsd_btn_set_kind(bank, index_in_bank(slot), FSD_BTN_KIND_LEVEL);
    }
}

void fsd_trig_set_double(FsdTriggers* t, FsdSignal s, bool on) {
    if(!t) return;
    const int slot = switch_slot(s);
    FsdButtons* bank = bank_of(t, slot);
    if(!bank) return; // not a switch; nothing to double-press
    fsd_btn_set_double(bank, index_in_bank(slot), on);
}

void fsd_trig_disturbed(FsdTriggers* t, FsdSignal s, uint32_t now_ms) {
    if(!t || s >= FSD_SIG_COUNT) return;
    const FsdSignalDef* d = fsd_signal_def(s);
    /* Switches are exempt. We cannot press one, so a trigger there is a
     * person, and suppressing it would silence the owner rather than us. */
    if(!d || d->kind == FSD_SIGK_SWITCH) return;
    t->quiet_until[s] = now_ms + FSD_TRIG_SUPPRESS_MS;
    t->quiet_armed[s] = true;
}

bool fsd_trig_is_quiet(const FsdTriggers* t, FsdSignal s, uint32_t now_ms) {
    if(!t || s >= FSD_SIG_COUNT) return false;
    if(!t->quiet_armed[s]) return false;
    /* Signed difference so the millisecond counter wrapping does not un-quiet
     * or permanently quiet a signal. */
    return (int32_t)(t->quiet_until[s] - now_ms) > 0;
}

uint8_t fsd_trig_on_frame(FsdTriggers* t, uint32_t can_id, const uint8_t* data, uint8_t dlc,
                          uint32_t now_ms, FsdTriggerEvent* out, uint8_t max_out) {
    uint8_t n = 0;
    if(!t || !data) return 0;

    for(int i = 0; i < FSD_SIG_COUNT; i++) {
        const FsdSignal s = (FsdSignal)i;
        const FsdSignalDef* d = fsd_signal_def(s);
        if(!d || d->can_id != can_id) continue;

        int32_t v = 0;
        /* Anything but OK leaves this signal completely untouched — no value,
         * no `seen`, no event. A wrong-multiplex frame must not be read as
         * "everything it does not carry went to zero". */
        if(fsd_signal_extract(s, can_id, data, dlc, &v) != FSD_SIGV_OK) continue;

        const bool first = !t->seen[s];
        const int32_t prev = t->last[s];
        t->seen[s] = true;
        t->last[s] = v;

        switch(d->kind) {
        case FSD_SIGK_SWITCH: {
            const int slot = switch_slot(s);
            FsdButtons* bank = bank_of(t, slot);
            if(!bank) break;
            /* Any non-zero detent is "down". The first/second detent
             * distinction is real and is deliberately not a trigger yet: no
             * rule asks for it, and inventing an event nobody consumes is how
             * an unreachable feature gets written. */
            const FsdBtnEvent e =
                fsd_btn_report(bank, index_in_bank(slot), v != 0, now_ms);
            const FsdTriggerKind k = kind_of(e);
            if(k != FSD_TRIG_NONE) emit(out, max_out, &n, s, k, 0, now_ms);
            break;
        }

        case FSD_SIGK_STATE:
            /* The first frame establishes the state; it is not a transition.
             * Otherwise every boot would look like the whole car just changed
             * at once, and every rule would fire. */
            if(first || v == prev) break;
            if(fsd_trig_is_quiet(t, s, now_ms)) break;
            emit(out, max_out, &n, s, FSD_TRIG_STATE_LEAVE, prev, now_ms);
            emit(out, max_out, &n, s, FSD_TRIG_STATE_ENTER, v, now_ms);
            break;

        case FSD_SIGK_DELTA:
            /* A count, not a level: zero is "nothing happened", and the value
             * is the whole event. Suppression applies — our own scroll write
             * would otherwise trigger a rule watching the scroll. */
            if(v == 0) break;
            if(fsd_trig_is_quiet(t, s, now_ms)) break;
            emit(out, max_out, &n, s, FSD_TRIG_DELTA, v, now_ms);
            break;
        }
    }
    return n;
}

uint8_t fsd_trig_tick(FsdTriggers* t, uint32_t now_ms, FsdTriggerEvent* out, uint8_t max_out) {
    uint8_t n = 0;
    if(!t) return 0;

    for(int i = 0; i < FSD_SIG_COUNT; i++) {
        const FsdSignal s = (FsdSignal)i;
        const FsdSignalDef* d = fsd_signal_def(s);
        if(!d || d->kind != FSD_SIGK_SWITCH) continue;
        const int slot = switch_slot(s);
        FsdButtons* bank = bank_of(t, slot);
        if(!bank) continue;
        const FsdTriggerKind k = kind_of(fsd_btn_tick(bank, index_in_bank(slot), now_ms));
        if(k != FSD_TRIG_NONE) emit(out, max_out, &n, s, k, 0, now_ms);
    }
    return n;
}

const char* fsd_trig_kind_str(FsdTriggerKind k) {
    switch(k) {
    case FSD_TRIG_NONE: return "none";
    case FSD_TRIG_PRESS: return "press";
    case FSD_TRIG_LONG: return "long press";
    case FSD_TRIG_DOUBLE: return "double press";
    case FSD_TRIG_STUCK: return "stuck";
    case FSD_TRIG_STATE_ENTER: return "became";
    case FSD_TRIG_STATE_LEAVE: return "stopped being";
    case FSD_TRIG_DELTA: return "moved";
    }
    return "?";
}

/* The slot count is derived at runtime from the signal table, so it cannot be
 * asserted at compile time against it. This function exists to be called by the
 * tests, which is where the two numbers are compared — and where adding a
 * switch signal without room for it turns red. */
int fsd_trig_switch_count(void) {
    return switch_count();
}
