#pragma once

/*
 * fsd_ota.h — the one place that decides "is the car installing an update".
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * The decision used to live twice: once in fsd_logic/fsd_handler.c (Flipper)
 * and once in esp32/.firmware/fsd_handler.cpp (ESP32). The two copies drifted
 * and nobody noticed, because only the Flipper copy is reachable from the host
 * tests. Upstream commit b3e5eef even says "Flipper OTA gate aligned with
 * ESP32" while moving the Flipper copy to raw == 2 — but the ESP32 copy was
 * raw == 1 and stayed there. The commit message described an alignment that
 * did not happen.
 *
 * On 2026-09-07 that drift bit us on the bench: the ESP32 read raw == 1 as
 * "updating", latched tesla_ota_in_progress, and every gate downstream closed
 * — fsd_supervised_drive_why() answered OTA and fsd_can_transmit() refused.
 * Only a reboot cleared it.
 *
 * So the decision lives here now, in fsd_logic, where both platforms link it
 * and the host tests exercise it. Two copies cannot disagree if there is one.
 *
 * WHAT THE RAW VALUE MEANS
 * ------------------------
 * GTW_carState (0x318) byte 6, bits [1:0] — GTW_updateInProgress:
 *
 *     0  no update      1  update available
 *     2  installing     3  scheduled
 *
 * Only 2 suspends TX. 1 means "there is an update you could install", which is
 * true while the car drives around perfectly normally.
 *
 * 🔴 AND ON THIS CAR THAT FIELD IS NOT A FLAG AT ALL.
 * Across all 31 captures (2026-09-01 .. 2026-09-06, parked and driving alike)
 * 0x318 byte 6 walks 0x21, 0x23, 0x25 ... 0x3F — sixteen values, stepping by
 * two, wrapping. It is a rolling counter whose bit 0 is always 1, so bits [1:0]
 * alternate 1, 3, 1, 3 forever and value 2 never appears. Reading 1 as
 * "installing" turned a counter into an update notice. Same family as the
 * 2-byte 0x311, the 3-byte 0x399 and the 4-byte 0x3D8: the DBC describes a
 * frame this car does not send.
 *
 * The consecutive-frame hysteresis stays. A dropped frame can put two 1s back
 * to back; three in a row is what it took to trip the old code, and requiring
 * a run still costs nothing when the value is real (0x318 arrives every 100 ms,
 * so three frames is 0.3 s).
 */

#include "fsd_state.h"

#include <stdint.h>

/* 🔴 The ESP32 caller is C++ (fsd_handler.cpp) and this file is C. Without the
 * guard the two disagree about the symbol name and the link fails on all eight
 * boards — which is exactly what happened the first time, and which the host
 * tests cannot see because they are C on both sides. */
#ifdef __cplusplus
extern "C" {
#endif

/* GTW_updateInProgress raw values. */
#define FSD_OTA_RAW_NO_UPDATE 0u
#define FSD_OTA_RAW_AVAILABLE 1u
#define FSD_OTA_RAW_INSTALLING 2u
#define FSD_OTA_RAW_SCHEDULED 3u

/* Consecutive samples needed to change the latch, in either direction.
 * Asymmetric on purpose: turning the protection ON is cheap, turning it OFF
 * takes twice as much evidence. Failing towards "do not transmit" is the safe
 * direction for this particular gate. */
#define FSD_OTA_ASSERT_FRAMES 3u
#define FSD_OTA_CLEAR_FRAMES 6u

/** Feed one GTW_updateInProgress sample (only bits [1:0] are read).
 *
 * Updates state->ota_raw_state every call, and state->tesla_ota_in_progress
 * once a run is long enough. Safe to call with a NULL state. */
void fsd_ota_observe_raw(FSDState* state, uint8_t raw);

#ifdef __cplusplus
}
#endif
