#include "fsd_burst.h"

#include <string.h>

/* Signed comparison throughout, so the millisecond counter wrapping does not
 * turn a due deadline into a 49-day wait. Same trick as release_due(). */
static bool reached(uint32_t now_ms, uint32_t at_ms) {
    return (int32_t)(now_ms - at_ms) >= 0;
}

void fsd_burst_reset(FsdBurst* b) {
    if (!b) return;
    memset(b, 0, sizeof(*b));
}

bool fsd_burst_arm(FsdBurst* b, FsdBodyAction action, int32_t arg, uint8_t rule_index,
                   uint32_t can_id, uint8_t reps, uint32_t now_ms) {
    if (!b) return false;
    /* An armed slot that can never emit would hold a slot until the deadline
     * and then report an expiry that never had a chance. */
    if (reps == 0u) return false;

    for (unsigned i = 0; i < FSD_BURST_MAX; i++) {
        FsdBurstSlot* s = &b->slot[i];
        if (s->remaining != 0u) continue;
        s->action = action;
        s->arg = arg;
        s->rule_index = rule_index;
        s->id = can_id;
        s->remaining = reps;
        s->deadline_ms = now_ms + FSD_BURST_MAX_WAIT_MS;
        s->seq = b->next_seq++;
        return true;
    }

    b->dropped++;
    return false;
}

bool fsd_burst_on_frame(FsdBurst* b, uint32_t can_id, uint32_t now_ms, FsdBurstDue* out) {
    if (!b || !out) return false;

    /* Oldest first. `seq` rather than slot order because a freed slot is
     * reused, so position says nothing about age. */
    FsdBurstSlot* pick = NULL;
    for (unsigned i = 0; i < FSD_BURST_MAX; i++) {
        FsdBurstSlot* s = &b->slot[i];
        if (s->remaining == 0u || s->id != can_id) continue;
        if (!pick || (uint16_t)(s->seq - pick->seq) > 0x8000u) pick = s;
    }
    if (!pick) return false;

    out->action = pick->action;
    out->arg = pick->arg;
    out->rule_index = pick->rule_index;

    pick->remaining--;
    /* The id spoke, so this slot is not the one that has gone quiet. */
    pick->deadline_ms = now_ms + FSD_BURST_MAX_WAIT_MS;
    return true;
}

uint8_t fsd_burst_tick(FsdBurst* b, uint32_t now_ms) {
    if (!b) return 0u;
    uint8_t n = 0u;
    for (unsigned i = 0; i < FSD_BURST_MAX; i++) {
        FsdBurstSlot* s = &b->slot[i];
        if (s->remaining == 0u) continue;
        if (!reached(now_ms, s->deadline_ms)) continue;
        s->remaining = 0u;
        b->expired++;
        n++;
    }
    return n;
}

uint8_t fsd_burst_pending(const FsdBurst* b) {
    if (!b) return 0u;
    uint8_t n = 0u;
    for (unsigned i = 0; i < FSD_BURST_MAX; i++)
        if (b->slot[i].remaining != 0u) n++;
    return n;
}
