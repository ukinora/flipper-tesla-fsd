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
    0x249u,  // SCCM_leftStalk       — high beam / turn signal / wiper-wash
             //   button. Two reasons, and the second is the load-bearing one:
             //   (1) washWipeButtonStatus is a grade-B trigger candidate
             //       (순정입력-트리거-후보.md), and the field procedure there
             //       says to capture it — which was impossible until now.
             //   (2) 키토스 implements "wiper 0.5 단" and "짧은 경적" by
             //       replaying stalk input. If TSL does anything similar this
             //       is where it rides, and TSL's full behaviour — not just
             //       T1/T2/T3 — is what disappears at teardown.
             //   ⚠️ This fork can BUILD 0x249 frames (fsd_handler.h:284-303:
             //   high-beam strobe, turn signal, wiper wash). Recording it is
             //   still read-only — the capture runs Listen-Only — but if an
             //   emitter is ever wired, an A-B diff on this id would show our
             //   own frames as well as TSL's. Note it before concluding.

    /* ── comfort control (2026-08-20) ────────────────────────────────────────
     *
     * 🔴 ADDED AFTER THE PRIORITIES TURNED OUT TO BE WRITTEN DOWN WRONG.
     *
     * Every plan in this repository said the top feature was the speed-camera
     * profile drop, and this set was chosen against that. The owner corrected
     * it: the first three features are comfort control from a Bluetooth
     * button, comfort control from the car's OWN buttons, and automation from
     * vehicle state. The camera work is phase two.
     *
     * Most of what those need was already above, because the T1/T2 work chose
     * ids by "we must not become unable to see it" rather than by "we know we
     * need it". That judgement paid for itself here. These four were missed. */

    0x273u,  // UI_vehicleControl — mirror fold, lock, wiper, horn, seat heat.
             //   The single frame carrying most of what "comfort function"
             //   means, and it was in config.h and in the capability prober
             //   but NOT here.
    0x343u,  // VCRIGHT_status — rear defrost, mirror heat, footwell light
             //   current. More comfort actuation, and the footwell light is
             //   interior lighting, which is feature #1 on the owner's list.
    0x229u,  // SCCM_rightStalk — carries SCCM_parkButtonStatus, a button on
             //   the car that the owner may want as a trigger.
             //   ⚠️ This is the GEAR LEVER frame. Recording it is read-only and
             //   carries no transmit risk: send_on_bus() refuses this id
             //   unconditionally, ahead of every other gate. It is excluded as
             //   a TRIGGER for good reasons (pressing it changes gear); that is
             //   a separate decision from whether we may LOOK at it.
    /* ── tyre pressure (2026-08-21) ──────────────────────────────────────
     *
     * 0x219 VCSEC_TPMSData carries all four wheels, multiplexed by byte 0:
     *   byte 1 pressure (x0.025 bar), byte 2 temperature, byte 3 sensor
     *   battery, byte 4 bits[2:0] LOCATION -- the frame says which corner a
     *   reading belongs to, so the mapping does not have to be guessed.
     *
     * 🔴 Added because it is the ONLY way to settle two unknowns, and both
     * are unanswerable once the car is gone: does this frame reach OUR tap,
     * and what do the location values 0..4 actually mean. Neither is in any
     * document we have.
     *
     * ⚠️ joshwardell's DBC labels this ChassisBus -- but it labels 0x399
     * ChassisBus too, and 0x399 comes from the AP ECU, which Tesla's own
     * network drawing puts on Vehicle CAN. The label is not a reliable
     * predictor for our tap. VCSEC itself IS on Vehicle CAN per that
     * drawing, so this frame is likely reachable; "likely" is exactly what
     * a capture is for.
     *
     * Rate is low (a TPMS sensor reports every few seconds), so this costs
     * the ring almost nothing. */
    0x219u,  // VCSEC_TPMSData — four wheels, muxed

    0x257u,  // DI_speed — road speed. "Automation from vehicle state" has to be
             //   correlated against whether the car was moving, and 0x118 gives
             //   gear only. Also the only proof the car actually moves, which
             //   the GPS freeze detector already depends on.
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
