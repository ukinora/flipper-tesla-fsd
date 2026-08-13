#pragma once
/*
 * mode_switch.h — the one way to change op_mode.
 *
 * WHY THIS EXISTS
 * ---------------
 * op_mode lived in FSDState and the CAN controller's listen-only register lived
 * in the driver, and nothing kept them together. Each caller was trusted to
 * update both, and they did not:
 *
 *   BLE SET_MODE   set op_mode, acknowledged OK, never touched the driver.
 *                  The app showed Active while the hardware still refused every
 *                  transmit.
 *   BLE revoke     lowered op_mode after the phone left, and left the driver
 *                  open if somebody had opened it with the button. That is the
 *                  dangerous direction: Listen-Only's whole safety claim is that
 *                  it is a hardware register, "physically incapable of TX even
 *                  on bus error frames". A Listen-Only that is only a variable
 *                  is not that.
 *
 * So the pair moves behind one call, and the mapping comes from
 * fsd_mode_opens_tx() in fsd_types.h — the same predicate fsd_can_transmit()
 * uses, so the software gate and the hardware register cannot disagree.
 *
 * 🔴 LOOP TASK ONLY. This touches the CAN driver, and on this board one of the
 * two controllers is an MCP2515 reached over SPI that loop() is polling. A BLE
 * callback must NOT call this — it must ask, and let ble_server_tick() apply it
 * from loop(). That is also what makes it possible to acknowledge only after
 * the driver has actually switched.
 */

#include <stdbool.h>

#include "fsd_handler.h"   // OpMode, FSDState

/**
 * Set op_mode and the controllers together.
 *
 * @return true when every initialised controller reported the new register
 *         state. False means the mode is NOT what was asked for — the caller
 *         must report failure rather than assume it worked.
 */
bool mode_apply(OpMode m);

/** The mode as it stands, read under the state lock. */
OpMode mode_current(void);
