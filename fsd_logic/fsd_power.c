#include "fsd_power.h"

#include <stddef.h>

uint32_t fsd_pwr_quiet_ms(uint32_t now_ms, uint32_t last_rx_ms, bool seen_any) {
    /* Never received anything. We have not watched the bus fall silent -- we
     * have watched nothing. Reporting a huge quiet time here would make a
     * module that was never wired to the car look exactly like a car that fell
     * asleep, which is the one confusion this whole file exists to prevent. */
    if(!seen_any) return 0u;

    /* Unsigned on purpose: the ms clock wraps after ~49 days and this stays
     * correct across the wrap. A signed comparison would briefly report a bus
     * that had been quiet for weeks. */
    uint32_t gap = now_ms - last_rx_ms;

    /* 🔴 ...but the same arithmetic cannot tell "49 days elapsed" from "the
     * stamp is 1 ms in the future", and the second one really happens: loop()
     * samples now = millis() before draining RX, and process_frame() stamps
     * last_rx_ms during the drain with a later millis(). The wrapped gap then
     * clears FSD_PWR_ALWAYS_ON_MS and fsd_pwr_live_verdict() announces
     * ALWAYS-ON -- a verdict about the car's 12V feed manufactured by a race in
     * our own loop, in the one file whose whole job is not doing that.
     *
     * Anything past half the range is backwards, not elapsed: this never
     * measures 24 days. Seen 21,113 times in a 40 s bench replay (2026-08-17),
     * the first time this board ever received CAN. */
    if(gap > 0x80000000u) return 0u;

    if(gap < FSD_PWR_QUIET_MS) return 0u;
    return gap;
}

FsdPwrVerdict fsd_pwr_verdict(FsdPwrReset reset, const FsdPwrRecord* prev) {
    if(!prev || !prev->valid) return FSD_PWR_UNKNOWN;

    switch(reset) {
    case FSD_PWR_RESET_BROWNOUT:
        /* The supply sagged rather than vanished. Thin tap wire, a cold crimp,
         * an e-fuse near its limit. Calling this "switched" would send the
         * operator looking at the wrong thing entirely. */
        return FSD_PWR_BROWNOUT;

    case FSD_PWR_RESET_SOFTWARE:
    case FSD_PWR_RESET_EXTERNAL:
        /* We fell over, or someone pressed reset. The supply is not implicated
         * and must not be blamed. */
        return FSD_PWR_CRASH;

    case FSD_PWR_RESET_POWERON:
        /* 🔴 The trap one level below the next one. quiet_ms == 0 arrives here
         * from TWO opposite situations, and only one of them says anything
         * about the car:
         *
         *   the bus was busy right to the end  -> the car stayed awake
         *   we never heard a single frame      -> we know nothing about the car
         *
         * fsd_pwr_quiet_ms() collapses the second into 0 deliberately, so that
         * an unwired module cannot look like a sleeping car. But NO_SLEEP is a
         * claim about the CAR, and a session that never heard the bus only ever
         * observed the WIRING. Swap CAN-H and CAN-L at the tap -- which the
         * checklist lists as a real possibility -- and an hour of silence gets
         * reported as "the car never slept", sending the operator to the sentry
         * settings for a wiring fault. */
        if(!prev->seen_any) return FSD_PWR_NO_BUS;

        /* 🔴 The trap. Coming back from a power cut looks like proof of a
         * switched feed, but only if the car actually slept. Sentry left on, a
         * scheduled charge, or someone opening the app all keep the bus alive;
         * then a power cut means something else happened entirely and we have
         * no business calling it. */
        if(prev->quiet_ms == 0u) return FSD_PWR_NO_SLEEP;
        return FSD_PWR_SWITCHED;

    case FSD_PWR_RESET_UNKNOWN:
    default: return FSD_PWR_UNKNOWN;
    }
}

FsdPwrVerdict fsd_pwr_live_verdict(uint32_t quiet_ms) {
    /* The boot path can never reach ALWAYS_ON: a module that kept its power
     * never rebooted to report anything. So it is decided here, while running,
     * and only after the bus has been silent long enough that a switched feed
     * would already have dropped us. */
    if(quiet_ms >= FSD_PWR_ALWAYS_ON_MS) return FSD_PWR_ALWAYS_ON;
    return FSD_PWR_UNKNOWN;
}

const char* fsd_pwr_reset_str(FsdPwrReset r) {
    switch(r) {
    case FSD_PWR_RESET_POWERON: return "power applied";
    case FSD_PWR_RESET_BROWNOUT: return "supply sagged (brownout)";
    case FSD_PWR_RESET_SOFTWARE: return "software reset";
    case FSD_PWR_RESET_EXTERNAL: return "reset pin";
    case FSD_PWR_RESET_UNKNOWN:
    default: return "unknown";
    }
}

const char* fsd_pwr_verdict_str(FsdPwrVerdict v) {
    switch(v) {
    case FSD_PWR_NO_SLEEP: return "inconclusive - the car never slept";
    case FSD_PWR_SWITCHED: return "SWITCHED - the feed dies with the car";
    case FSD_PWR_ALWAYS_ON: return "ALWAYS-ON - the feed outlives the car";
    case FSD_PWR_BROWNOUT: return "BROWNOUT - check the wiring, not the switching";
    case FSD_PWR_CRASH: return "we crashed - says nothing about the supply";
    case FSD_PWR_NO_BUS:
        return "NO CAN HEARD - suspect the CAN pair (H/L swapped?), not the car";
    case FSD_PWR_UNKNOWN:
    default: return "no verdict yet";
    }
}
