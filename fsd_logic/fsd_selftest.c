#include "fsd_selftest.h"

FsdSelftestAction fsd_selftest_decide(uint32_t elapsed_ms, uint32_t loops,
                                      bool healthy) {
    /* The deadline is checked FIRST and on its own. It used to be checked last,
     * behind the two warm-up gates, which meant a slow loop could keep the
     * decision in WAIT forever: at elapsed = 1,000,000 ms with 999 loops the old
     * order still returned early on the loop-count gate. An image too broken to
     * run its loop is exactly the image the deadline exists for. */
    if(elapsed_ms >= FSD_SELFTEST_DEADLINE_MS) {
        return healthy ? FSD_SELFTEST_ACCEPT : FSD_SELFTEST_ROLLBACK;
    }

    /* Before the deadline, an image must prove it is really running before we
     * throw away its way back. Time alone is not proof -- a hung loop still
     * accumulates it -- and iterations alone are not either, since a fast loop
     * reaches a thousand of them in a few milliseconds. */
    if(elapsed_ms < FSD_SELFTEST_MIN_MS) return FSD_SELFTEST_WAIT;
    if(loops < FSD_SELFTEST_MIN_LOOPS) return FSD_SELFTEST_WAIT;

    /* Healthy and proven. Unhealthy but still inside the window keeps waiting:
     * a controller can still come back on its re-init retry, and rolling back a
     * good image for being slow to start is its own kind of failure. */
    return healthy ? FSD_SELFTEST_ACCEPT : FSD_SELFTEST_WAIT;
}
