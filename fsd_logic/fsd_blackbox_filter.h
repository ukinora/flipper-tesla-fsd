#pragma once
/*
 * fsd_blackbox_filter.h — capture ID filter for the black-box recorder (#124).
 *
 * The recorder's RAM ring is small: 6000 frames on a no-PSRAM S3. A busy Tesla
 * full bus runs ~3300 frames/s (measured on a Legacy Model X), which fills the
 * ring in ~1.8 s — truncating the intended capture window (5 s pre / 1 s post
 * on this fork) and losing the exact steer-jerk / abort lead-up it exists for.
 *
 * Recording only the key diagnostic IDs drops the stored rate ~15x (the key set
 * is ~a few hundred f/s), so 6000 frames covers ~15-30 s even on a busy bus —
 * the whole window survives.
 *
 * Header-only + dependency-free (only <stdint.h>/<stdbool.h>) so the host test
 * can exercise the predicate without the ESP32 backends.
 *
 * Escape hatch: define BLACKBOX_CAPTURE_ALL at build time to keep EVERY frame
 * instead — full fidelity, but on a busy bus the window truncates to ~1.8 s as
 * above. Default off.
 *
 * ⚠️ On this fork's T-2CAN the ring is 40000 frames (PSRAM), so CAPTURE_ALL would
 * hold ~12 s of a busy bus rather than 1.8 s. It is still off by default: for the
 * pre-teardown capture, "long enough to catch a gesture the operator performs
 * three times" is worth more than "every frame for twelve seconds". When an
 * genuinely unfiltered baseline IS the point — the idle A-B baseline in
 * 차량-방문-체크리스트.md B-2.5 §0, where the whole idea is to catch frames we
 * do not know to look for — use the laptop path (tools/can_capture.py), which
 * never filters and is not bounded by a RAM ring.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Curated key diagnostic IDs — the frames the abort / steer-jerk / nag analysis
// actually reads. Edit this list to add or drop a capture ID. IDs mirror the
// CAN_ID_* defines in fsd_handler.h / esp32/.firmware/config.h.
static const uint32_t FSD_BLACKBOX_KEY_IDS[] = {
    0x370u,  // EPAS3P_sysStatus     — nag killer / EPAS
    0x399u,  // DAS_status (HW3/Legacy), also ISA speed on HW4
    0x39Bu,  // DAS_status (HW4)
    0x488u,  // DAS_steeringControl  — DAS steering request/angle
    0x129u,  // SCCM_steeringAngleSensor — steering angle (#108 soft-engage gate)
    0x3EEu,  // DAS_autopilot        — AP legacy
    0x3FDu,  // DAS_autopilotControl — AP control (HW3/HW4 core)
    0x145u,  // ESP_status           — driver brake pedal
    0x238u,  // UI_driverAssistMapData — map speed limit
    0x389u,  // DAS_status2          — ACC speed limit
    0x2B9u,  // DAS_control          — cruise set speed / ACC state
    0x118u,  // DI_systemStatus      — vehicle state

    /* ── tesla-can-mod: the frames THIS fork's capture plan is about ─────────
     *
     * 🔴 Without these the recorder was structurally unable to record the one
     * capture this fork exists to take. The list above is upstream's abort /
     * steer-jerk / nag analysis set; none of it overlaps what a pre-teardown
     * TSL capture needs except 0x3FD. So `bbon` -> operate -> `mark` ->
     * DUMP_START produced a file with the interesting frames filtered OUT, and
     * that would only have become visible after the TSL was removed — i.e.
     * after the one chance to record it was gone.
     *
     * Rate cost is small: these are switch/door/lighting frames, not the
     * high-rate powertrain traffic the filter exists to keep out. */
    0x3C2u,  // VCLEFT_switchStatus  — scroll wheel. THE frame the one-shot
             //   capture is for: S3XY/TSL speed-profile changes ride here
             //   (mux 1 swcRightScrollTicks), and derive_scroll_encoding.py
             //   has nothing to chew on without it.
    0x3C3u,  // VCRIGHT_switchStatus — right-side switches (T2 gesture input)
    0x102u,  // VCLEFT_doorStatus    — latch/handle, T1 input + trigger candidate
    0x103u,  // VCRIGHT_doorStatus   — same, right side
    0x3F5u,  // VCFRONT_lighting     — courtesyLightingRequest, the T1 OUTPUT
             //   candidate. Also on send_on_bus()'s unconditional deny list:
             //   we record it, we never emit it.
    0x3E2u,  // VCLEFT_lightStatus   — map light STATE (+ switch bits). This is
             //   what lets an A-B diff tell "TSL turned the light on" from
             //   "the car did it itself" without relying on someone writing it
             //   down by hand. Unverified bit layout (joshwardell, MIT).

    /* ── Added 2026-08-17, before the teardown visit ─────────────────────────
     *
     * A doc sweep caught that T1's input set was only two thirds recorded:
     * CLAUDE.md defines T1 as "rear door open/close -> cabin light" with
     * inputs 0x102/0x103 latch+handle AND 0x311 anyDoorOpen — and 0x311 was
     * not in this list. The other two cover the T2 output side, which has no
     * confirmed frame at all yet, so leaving them out would mean the capture
     * cannot even be searched for it.
     *
     * The bar for this list before a one-shot capture is not "we know we need
     * it", it is "we would be unable to look". Everything here is a body /
     * switch frame; none of it is powertrain-rate. */
    0x311u,  // UI_warning           — anyDoorOpen (the third T1 input), plus
             //   buckle and blinkers in the same frame. That makes it the
             //   cheapest way to reconstruct "what state was the car in when
             //   TSL acted", which an A-B diff needs to be readable at all.
             //   Also the frame fsd_supervised_drive() reads for the belt.
    0x119u,  // VCSEC_windowRequests — T2 is "close passenger window twice ->
             //   passenger door opens". If TSL emits either half as a request
             //   rather than a switch press, this is where it would ride.
             //   Unproven: we have never seen TSL's T2 output frame.
    0x3E9u,  // DAS_bodyControls     — lights / hazards / turn / wipers. The
             //   other plausible home for a body actuation TSL might emit, and
             //   the second candidate for T1's output alongside 0x3F5.
};

#define FSD_BLACKBOX_KEY_ID_COUNT \
    (sizeof(FSD_BLACKBOX_KEY_IDS) / sizeof(FSD_BLACKBOX_KEY_IDS[0]))

// True when `id` should enter the black-box ring. Runs on every RX frame, so it
// stays cheap: a linear scan of ~12 constants (O(1)-ish, no allocation). Define
// BLACKBOX_CAPTURE_ALL to bypass the filter and keep every frame.
static inline bool fsd_blackbox_should_record(uint32_t id) {
#ifdef BLACKBOX_CAPTURE_ALL
    (void)id;
    return true;
#else
    for (size_t i = 0; i < FSD_BLACKBOX_KEY_ID_COUNT; i++) {
        if (FSD_BLACKBOX_KEY_IDS[i] == id) return true;
    }
    return false;
#endif
}
