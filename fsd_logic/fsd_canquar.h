#pragma once
/*
 * fsd_canquar.h — refuse to bring up a CAN bus that panicked the board.
 *
 * 🔴 Why this had to move into the boot path (measured 2026-08-17):
 *
 * A TWAI controller that cannot decode its bus raises error interrupts without
 * pause and starves everything else until the interrupt watchdog reboots the
 * board — forever. The first attempt at a cure watched the error counter from
 * the loop (fsd_bushealth.h). It was run against the real fault and LOST:
 *
 *     19.7s  [CAN] can0 TWAI Listen-Only @ 500 kbps
 *            ...not one line of loop output in between...
 *     20.3s  Guru Meditation Error: Interrupt wdt timeout on CPU1
 *
 * The loop ran ZERO times. No threshold tunes around code that never executes.
 *
 * The boot path does execute — it runs before the controller is even enabled.
 * So the decision belongs there: remember that enabling this bus killed us last
 * time, and do not enable it. One reboot, then the module comes up on whatever
 * is left and stays up.
 *
 * ── Why a bus is only quarantined after repeated panics ──────────────────────
 * Quarantining a healthy bus costs exactly what the fault costs: at the car,
 * either one loses the capture. So every uncertain case resolves to "do not
 * quarantine":
 *   - a clean boot clears the counter outright;
 *   - a panic while no bus is unproven is somebody else's panic, not ours;
 *   - one panic is not enough — boards panic for other reasons;
 *   - only the lowest-numbered UNPROVEN bus is taken, one per round, so a
 *     second bus is only lost if killing the first did not help.
 *
 * "Unproven" means: enabled this boot, and the loop has not yet run healthily
 * for long enough to vouch for it. The caller sets that mask before begin() and
 * clears it once the module has survived a while.
 *
 * ⚠️ Quarantine never lifts itself. Coming back automatically would walk
 * straight into the storm again, which is how the reboot loop happens in the
 * first place. It is cleared explicitly (serial `canclear`, or a factory reset)
 * and reported loudly at every boot so it cannot rot unnoticed.
 *
 * Header-only + dependency-free so the host test runs the same code as the
 * firmware.
 */

#include <stdbool.h>
#include <stdint.h>

/* Consecutive panic boots, each with an unproven bus, before one is taken out.
 * Two rather than one because a single unrelated panic must not cost a bus, and
 * two reboots of this failure take about a second. */
#define FSD_CANQUAR_PANICS 2u

typedef struct {
    uint8_t quarantined; /* bit i = do not bring up bus i */
    uint8_t panics;      /* consecutive panic boots with an unproven bus */
} FsdCanQuar;

typedef struct {
    uint8_t newly; /* bit of the bus quarantined by this call, 0 if none */
} FsdCanQuarResult;

static inline void fsd_canquar_reset(FsdCanQuar* q) {
    if (!q) return;
    q->quarantined = 0u;
    q->panics = 0u;
}

/** Operator override: allow everything again. */
static inline void fsd_canquar_clear(FsdCanQuar* q) { fsd_canquar_reset(q); }

/** Call once at boot, before any bus is brought up.
 *
 * @param was_panic  the previous run ended in a panic / watchdog reset, not a
 *                   power cycle or a deliberate reboot.
 * @param unproven   mask of buses that were enabled last run and had not yet
 *                   been vouched for when it died.
 * @param bus_count  how many buses this board actually has; bits at or above
 *                   this are ignored so corrupt NVS cannot invent one.
 */
static inline FsdCanQuarResult fsd_canquar_boot(FsdCanQuar* q, bool was_panic,
                                                uint8_t unproven,
                                                uint8_t bus_count) {
    FsdCanQuarResult r;
    r.newly = 0u;
    if (!q) return r;

    uint8_t valid = (bus_count >= 8u) ? 0xFFu : (uint8_t)((1u << bus_count) - 1u);
    q->quarantined &= valid;
    unproven &= valid;

    /* Not our kind of failure — a clean boot, or a panic that happened while no
     * bus was on trial. Either way the streak is broken. */
    if (!was_panic || unproven == 0u) {
        q->panics = 0u;
        return r;
    }

    if (++q->panics < FSD_CANQUAR_PANICS) return r;

    /* Take the lowest-numbered bus still on trial. One per round: if losing it
     * does not stop the panics, the next round takes the next one. */
    uint8_t candidates = (uint8_t)(unproven & (uint8_t)~q->quarantined);
    for (uint8_t i = 0; i < bus_count && i < 8u; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        if (candidates & bit) {
            q->quarantined |= bit;
            r.newly = bit;
            break;
        }
    }

    /* Start counting again either way — if nothing was left to take, we do not
     * want a stale streak quarantining the next bus the instant one frees up. */
    q->panics = 0u;
    return r;
}
