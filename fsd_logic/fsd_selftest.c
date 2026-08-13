#include "fsd_selftest.h"

FsdSelftestAction fsd_selftest_decide(uint32_t elapsed_ms, uint32_t loops,
                                      bool healthy) {
    /* Both halves of the proof, or it is not proof. Time alone is not evidence
     * -- a loop that has stopped still accumulates it -- and iterations alone
     * are not either, since a healthy loop reaches a thousand of them in a few
     * milliseconds. `healthy` is separate again: it says the subsystems came UP,
     * which is a fact about initialisation and says nothing about whether the
     * loop is still turning. */
    const bool proven = (elapsed_ms >= FSD_SELFTEST_MIN_MS)
                     && (loops >= FSD_SELFTEST_MIN_LOOPS);

    if(proven && healthy) return FSD_SELFTEST_ACCEPT;

    /* 🔴 The deadline is a decision point, NOT an amnesty.
     *
     * This used to read `return healthy ? ACCEPT : ROLLBACK`, which accepted an
     * image on subsystem health alone once the clock ran out -- so an image
     * whose loop had all but stopped, but whose CAN, storage and BLE had come up
     * during setup(), was approved at the five-minute mark with one iteration to
     * its name. That threw away the very evidence the loop counter exists to
     * gather, and it contradicted this file's own header.
     *
     * At the deadline there is either proof or there is not. */
    if(elapsed_ms >= FSD_SELFTEST_DEADLINE_MS) return FSD_SELFTEST_ROLLBACK;

    /* Inside the window and not yet proven: keep running. A controller can still
     * come back on its re-init retry, and rolling back a good image for being
     * slow to start is its own kind of failure. */
    return FSD_SELFTEST_WAIT;
}
