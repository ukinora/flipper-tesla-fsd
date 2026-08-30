#pragma once

#include <stdint.h>
#include <stddef.h>
#include "fsd_handler.h"  // for CanFrame

enum CanBusId : uint8_t {
    CAN_BUS_PRIMARY = 0,    // can0
    CAN_BUS_SECONDARY = 1,  // can1
    CAN_BUS_COUNT = 2,
};

static inline const char *can_bus_name(CanBusId bus) {
    return bus == CAN_BUS_SECONDARY ? "can1" : "can0";
}

// ── Abstract CAN driver ───────────────────────────────────────────────────────
// Implemented by TwaiDriver (CAN_DRIVER_TWAI) and Mcp2515Driver (CAN_DRIVER_MCP2515).
// Compile-time selection via platformio.ini build_flags.

class CanDriver {
public:
    /** Initialise hardware and start the CAN bus.
     *  @param listen_only  If true, enter hardware listen-only mode (no ACK, no TX). */
    virtual bool begin(bool listen_only) = 0;

    /** Send one CAN frame.  Returns false when TX is not allowed (listen-only, bus-off, etc.). */
    virtual bool send(const CanFrame &frame) = 0;

    /** Non-blocking receive.  Fills frame and returns true if a frame was available. */
    virtual bool receive(CanFrame &frame) = 0;

    /** Cumulative bus/TX-error counter.
     *  TWAI: rx_missed + bus_errors + tx_failed.
     *  MCP2515: number of sendMessage() failures (typically ALLTXBUSY). */
    virtual uint32_t errorCount() = 0;

    /** Short human-readable breakdown of what errorCount() just counted.
     *  The two drivers count different things, and a bare number invites the
     *  reader to compare them as if they were the same -- can1 in Listen-Only
     *  cannot report anything but 0, which reads as "clean" next to a TWAI
     *  number that includes RX-queue overflow. Writes a suffix (may be empty). */
    virtual void errorDetail(char *out, size_t n) { if (n) out[0] = 0; }

    /** Cumulative count of frames successfully transmitted on the bus. */
    virtual uint32_t txCount() = 0;

    /** Cumulative count of frames received from the bus. */
    virtual uint32_t rxCount() = 0;

    /** Switch between listen-only and normal TX mode at runtime.
     *  Implementations must reinitialise the hardware as needed. */
    virtual void setListenOnly(bool enable) = 0;

    /** Is the controller currently in hardware listen-only mode?
     *
     *  send() refuses outright while it is, and the bit is per-controller
     *  rather than per-frame, so a caller deciding whether an action is even
     *  possible has to be able to ASK — not assume from the operating mode.
     *  Added for the body-control gate (fsd_body.h), which carries it as an
     *  input defaulting to "shut". */
    virtual bool isListenOnly() const = 0;

    /** Is the controller actually installed and running?
     *
     * 🔴 isListenOnly() cannot answer this, and must not try. It reports SHUT for
     * an uninstalled controller because that is the safe answer for a TX gate —
     * a dead controller genuinely cannot transmit. But that makes a FAILED
     * "switch to Listen-Only" indistinguishable from a successful one: both
     * report shut, so mode_apply() returns true, the BLE revoke path clears its
     * retry state, and the periodic re-init loop never picks the bus up. The bus
     * stays deaf AND mute until the next reboot, and nothing says so.
     *
     * Two questions, two predicates: isListenOnly() = "can it transmit?",
     * isOperational() = "is it there at all?".
     *
     * Default true for drivers with no install step. */
    virtual bool isOperational() const { return true; }

    /** Restrict hardware reception to a single CAN id for full-rate single-ID
     *  capture on a busy bus, or restore accept-all.
     *  @param single  true = accept only @p id; false = accept all ids.
     *  @param id      the standard 11-bit id to accept when @p single is true.
     *  Default no-op: drivers that don't implement it keep accept-all and rely
     *  on software filtering, which decimates a single id on a busy bus. */
    virtual void setAcceptanceFilter(bool single, uint32_t id) { (void)single; (void)id; }

    /** Whether the underlying CAN hardware was detected on the bus/SPI.
     *  TWAI lives inside the SoC and is therefore always present.
     *  MCP2515 returns true once the chip has answered an SPI probe. */
    virtual bool hardwarePresent() { return true; }

    /** Consume-on-read edge: true exactly once after serviceHealth() has just
     *  initiated a bus-off recovery. Lets the main loop forward an EVT_BUSOFF to
     *  the event-core (the black-box trigger) without fsd_logic reaching into the
     *  TWAI controller state. Default false for drivers with no bus-off concept. */
    virtual bool busOffEvent() { return false; }

    /** Periodic health service — call once per main-loop iteration.
     *  TWAI: detects a bus-off controller (TX errors exceeded the limit) and
     *  drives the ESP-IDF recovery sequence so RX resumes without a manual
     *  Deactivate/Activate toggle. No-op while the bus is healthy. */
    virtual void serviceHealth() {}

    /** Stop this controller and release it. Not usable again until begin().
     *
     *  Exists for the error-storm guard (fsd_bushealth.h): a TWAI controller
     *  that cannot decode its bus interrupts without pause and takes the whole
     *  board down, healthy bus included, so the only cure is to take it off the
     *  bus entirely — stopping the polling is not enough when the damage comes
     *  from its interrupt handler.
     *
     *  Default no-op, which is the honest behaviour for a polled SPI controller:
     *  there is no interrupt storm to silence, and the caller has already
     *  stopped reading it. */
    virtual void shutdown() {}

    virtual ~CanDriver() = default;
};

/** Factory function — returns the driver selected at compile time.
 *  Caller owns the returned pointer. */
CanDriver *can_driver_create();

/** Factory function for boards with two active CAN controllers.
 *  Caller owns the returned pointer. */
CanDriver *can_driver_create(CanBusId bus);
