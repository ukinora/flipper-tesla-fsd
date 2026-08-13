#pragma once
/*
 * fsd_selftest.h — when to accept a freshly installed image, and when to give up.
 *
 * WHY THIS IS A SEPARATE FILE
 * ---------------------------
 * It started as three `if` statements inside ota_selftest_tick(), and their
 * ORDER was wrong in a way nobody could see by reading them: the warm-up gates
 * came before the deadline, so an image whose loop ran slowly never reached the
 * deadline check at all. A broken image that managed one loop per second would
 * have waited ~1000 s instead of 300, and a slower one indefinitely -- the exact
 * failure the deadline was added to prevent.
 *
 * Three ifs in a shim are untestable on a host. The same three as a pure
 * function are four lines of test, which is where the boundary cases belong.
 *
 * Pure: no clock, no NVS, no esp_ota. main.cpp supplies the numbers and carries
 * out the verdict.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Minimum evidence before an image may be accepted: it has to have been alive
 * for a while AND have gone round enough times that the scheduler is clearly
 * running. Neither alone is enough -- a hung loop still accumulates time, and a
 * fast loop accumulates iterations in milliseconds. */
#define FSD_SELFTEST_MIN_MS     20000u
#define FSD_SELFTEST_MIN_LOOPS  1000u

/* When the image is still unhealthy at this point, stop waiting and roll back.
 * Sized to give a 30 s controller re-init retry about ten attempts, so a device
 * that is merely slow to come up is not mistaken for one that is broken. */
#define FSD_SELFTEST_DEADLINE_MS 300000u

typedef enum {
    FSD_SELFTEST_WAIT = 0,  /* not enough evidence yet, keep running */
    FSD_SELFTEST_ACCEPT,    /* healthy and proven — cancel the rollback */
    FSD_SELFTEST_ROLLBACK,  /* still broken at the deadline — go back */
} FsdSelftestAction;

/**
 * @param elapsed_ms  since the first loop iteration after boot
 * @param loops       loop iterations counted in that time
 * @param healthy     every subsystem the image needs came up
 *
 * 🔴 The deadline outranks the warm-up gates. Whatever else is true, an
 * unhealthy image is rolled back once elapsed_ms reaches the deadline -- the
 * loop counter must never be able to hold it past that point, because a loop
 * that is not counting is itself a symptom.
 */
FsdSelftestAction fsd_selftest_decide(uint32_t elapsed_ms, uint32_t loops,
                                      bool healthy);

#ifdef __cplusplus
}
#endif
