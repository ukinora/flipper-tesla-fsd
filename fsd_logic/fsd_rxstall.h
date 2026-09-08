#pragma once
/*
 * fsd_rxstall.h — "this controller was listening, and stopped."
 *
 * 🔴 TWICE IN THE CAR, AND EACH TIME IT COST MORE THAN HALF AN HOUR:
 *
 *   2026-09-01  RX frozen at 701.        Cause: CAN-H/L reversed. Fixing the
 *               wiring was NOT enough — the controller stayed latched until
 *               the USB was pulled.
 *   2026-09-08  RX frozen at 4,187,198.  Cause: NOT that. The wiring was fine,
 *               the car was awake, the gear went in, Err was 0, Mod was 0.
 *               Same fix: pull the USB, put it back.
 *
 * Two different causes, one signature, and no way to tell them apart from the
 * outside. Both times the operator was in the car reading a counter that would
 * not move, working through the wiring — because that is what the first
 * incident taught — while the answer was a power cycle.
 *
 * WHAT THIS WATCHES, AND HOW IT DIFFERS FROM fsd_bushealth.h
 * ----------------------------------------------------------
 * That file watches a controller that raises errors without pause and starves
 * the loop. This one watches the opposite: **the error counter says zero and
 * the controller simply goes deaf.** Nothing complains. On the MCP2515 in
 * Listen-Only the error counter cannot move at all (it counts TX failures, and
 * we never transmit), so silence is the ONLY thing there is to read.
 *
 * WHY A GIVE-UP AND NOT JUST A TIMER
 * ----------------------------------
 * 🔴 A sleeping car reads exactly the same: the bus goes quiet and RX stops.
 * Re-initialising then is pointless, and doing it forever is worse than not
 * trying — the log fills with a recovery that never recovers, and a real stall
 * becomes invisible inside it.
 *
 * The discriminator is not a cleverer threshold. It is that **recovery either
 * works or it does not**: a latched controller comes back on the first
 * re-init; a sleeping car does not come back however many times we ask. So we
 * ask a few times, then stop until traffic returns on its own.
 *
 * WHAT THE CALLER MUST NOT GET WRONG
 * ----------------------------------
 * 🔴 THE CONTROLLER MUST COME BACK UP IN THE MODE IT WAS IN. Bringing an
 * MCP2515 back in normal mode puts a node on the car's bus that ACKs — which
 * is exactly what Listen-Only exists to prevent, and it would happen at the
 * moment nobody is watching. The re-init has to read isListenOnly() first.
 *
 * Header + pure C so the host tests run the same code the board does.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Silence that counts as a stall.
 *
 *  The Vehicle bus runs about 4,000 frames a second while the car is awake, so
 *  three seconds is thousands of missing frames — far outside anything jitter
 *  or a slow id could explain. Short enough that a stall in the car is
 *  recovered while the operator is still holding the switch, which is the
 *  whole point: both incidents were found by a person staring at a number. */
#define FSD_RXSTALL_QUIET_MS 3000u

/** Between attempts.
 *
 *  A re-init needs time to show whether it worked. Asking again a millisecond
 *  later would stack resets on a controller that is still coming up. */
#define FSD_RXSTALL_COOLDOWN_MS 10000u

/** How many times before concluding it is not our controller.
 *
 *  Three, because one re-init fixes a latch and no number fixes a parked car.
 *  The budget is refunded the moment a frame arrives. */
#define FSD_RXSTALL_MAX_TRIES 3u

typedef enum {
    FSD_RXSTALL_OK = 0,  /* nothing to do */
    FSD_RXSTALL_RECOVER, /* bring this controller back up, in the SAME mode */
} FsdRxStallVerdict;

typedef struct {
    uint32_t last_rx;
    uint32_t last_change_ms;
    uint32_t last_try_ms;
    bool seen_traffic; /* has this controller ever received anything? */
    bool gave_up;      /* budget spent; waiting for traffic to return */
    uint8_t tries;
    uint16_t recoveries; /* total asked for, so the console can say it */
} FsdRxStall;

void fsd_rxstall_reset(FsdRxStall* st);

/** One sample of the driver's cumulative RX counter. */
FsdRxStallVerdict fsd_rxstall_sample(FsdRxStall* st, uint32_t rx_count, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
