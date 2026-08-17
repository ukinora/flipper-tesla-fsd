#pragma once
/*
 * fsd_bushealth.h — "has this CAN controller started storming?"
 *
 * 🔴 What this exists to prevent (measured 2026-08-17, T-2CAN):
 *
 * Given a bus its TWAI controller could not decode, can0 raised error
 * interrupts without pause. twai_intr_handler_main preempted the loop
 * mid-SPI-read of the OTHER controller, over and over, until the interrupt
 * watchdog panicked CPU1 and rebooted the board — in a loop. The healthy bus
 * was collateral damage: can1 read 39,999 frames fine on its own and could not
 * be read at all while can0 sat on the same board.
 *
 * So a sick controller is not a lost feature, it is a dead module. That was
 * worked around with a compile-time mask (FSD_DISABLED_BUS_MASK) for the one
 * channel known to be faulty on this unit — but a mask only helps for a fault
 * already known. In the car the same shape can come from a mis-wired pair, a
 * swapped H/L, or the wrong bitrate on one connector, and the symptom would be
 * a module that reboots forever on the single visit that matters.
 *
 * This decides, from the driver's own cumulative error counter, when to give up
 * on one bus so the other keeps working.
 *
 * ⚠️ Best-effort by construction, and MEASURED NOT TO HELP against the one
 * fault we can reproduce. It is polled from the loop, and on 2026-08-17 the
 * reproducer was run against it deliberately:
 *
 *     19.7s  [CAN] can0 TWAI Listen-Only @ 500 kbps
 *            ...not one line of loop output in between...
 *     20.3s  Guru Meditation Error: Interrupt wdt timeout on CPU1
 *
 * The loop ran ZERO times between the controller coming up and the panic. No
 * threshold tunes around that — the window and strike count are irrelevant when
 * the code never executes. A dead transceiver produces exactly this.
 *
 * What it is still for: a bus that is bad but not that bad — a wrong bitrate or
 * a marginal pair, where the loop keeps running and errors merely pile up. That
 * case is plausible and untested; do not assume it works either.
 *
 * 🔴 So this is NOT a reason to drop the compile-time mask for a channel known
 * to be dead, and it is not a safety net you can lean on in the car. The real
 * fix for the starve-the-loop case has to live somewhere the loop is not
 * required — i.e. in the boot path, refusing to bring up a bus that panicked
 * the board last time.
 *
 * Header-only + dependency-free so the host test runs the same code as the
 * firmware.
 */

#include <stdbool.h>
#include <stdint.h>

/* Shortest span worth judging. Below this a sample is noise — a few errors in a
 * few ms is a bus hiccup, not a storm. */
#define FSD_BUS_WINDOW_MS 250u

/* Errors per second above which a window counts as a strike. A healthy Tesla
 * bus produces essentially none and the observed storm produced them
 * continuously, so this does not need to be precise — only far above "normal"
 * and far below "storm". */
#define FSD_BUS_ERR_PER_S 200u

/* Consecutive bad windows before the bus is shut down. More than one because
 * plugging a connector in, or a car waking up, can produce a single ugly
 * window — and shutting a good bus down loses the capture just as surely as
 * letting the board die. */
#define FSD_BUS_STRIKES 4u

typedef enum {
    FSD_BUS_OK = 0, /* nothing to do */
    FSD_BUS_STORM,  /* shut this controller down — it is taking the board with it */
} FsdBusVerdict;

typedef struct {
    bool     primed; /* have a baseline to subtract from */
    uint32_t last_ms;
    uint32_t last_errors;
    uint32_t strikes;
} FsdBusHealth;

static inline void fsd_bus_reset(FsdBusHealth* h) {
    if (!h) return;
    h->primed = false;
    h->last_ms = 0u;
    h->last_errors = 0u;
    h->strikes = 0u;
}

/** Feed the driver's cumulative error count. Call once per loop per bus.
 *
 * Returns FSD_BUS_STORM once per storm — the caller shuts the bus down, and
 * repeating the verdict every window afterwards would spam that path.
 *
 * Every uncertain case resolves to OK, because a false alarm costs a working
 * bus: the first sample only takes a baseline; windows shorter than
 * FSD_BUS_WINDOW_MS accumulate instead of being judged; a counter that moved
 * BACKWARDS means the driver was reinstalled (the count is per-installation),
 * so it re-baselines rather than reading a huge rate; and a clean window clears
 * the strikes so only a sustained storm trips it.
 */
static inline FsdBusVerdict fsd_bus_sample(FsdBusHealth* h, uint32_t errors,
                                           uint32_t now_ms) {
    if (!h) return FSD_BUS_OK;

    if (!h->primed) {
        h->primed = true;
        h->last_ms = now_ms;
        h->last_errors = errors;
        h->strikes = 0u;
        return FSD_BUS_OK;
    }

    /* Driver reinstalled — the counter restarted. Re-baseline; computing an
     * unsigned delta here would invent an enormous rate and shut down the bus
     * we just brought back up. */
    if (errors < h->last_errors) {
        h->last_ms = now_ms;
        h->last_errors = errors;
        h->strikes = 0u;
        return FSD_BUS_OK;
    }

    uint32_t elapsed = now_ms - h->last_ms;
    /* A stamp ahead of now is a clock that moved backwards, not a 49-day
     * window. Re-stamp and wait for a real one. */
    if (elapsed > 0x80000000u) {
        h->last_ms = now_ms;
        h->last_errors = errors;
        return FSD_BUS_OK;
    }
    if (elapsed < FSD_BUS_WINDOW_MS) return FSD_BUS_OK; /* keep accumulating */

    uint32_t delta = errors - h->last_errors;
    h->last_ms = now_ms;
    h->last_errors = errors;

    /* delta/elapsed > limit/1000, cross-multiplied to avoid both division and
     * floating point. 64-bit because delta can be large after a long window. */
    bool over = ((uint64_t)delta * 1000u) >
                ((uint64_t)FSD_BUS_ERR_PER_S * (uint64_t)elapsed);
    if (!over) {
        h->strikes = 0u;
        return FSD_BUS_OK;
    }

    h->strikes++;
    if (h->strikes < FSD_BUS_STRIKES) return FSD_BUS_OK;

    /* Fire once. Dropping the baseline means a caller that ignores the verdict
     * gets a fresh assessment rather than a verdict every window. */
    fsd_bus_reset(h);
    return FSD_BUS_STORM;
}
