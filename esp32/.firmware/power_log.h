#pragma once
/*
 * power_log.h — records whether the car's 12V feed outlives the car.
 *
 * The rear tap carries CAN, ground and 12V. Whether that 12V is SWITCHED (dies
 * when the car sleeps) or ALWAYS-ON decides the final install: always-on means
 * the module draws current overnight and wakes whenever the car wakes.
 *
 * WHY THE MODULE ANSWERS THIS AND NOT A MULTIMETER
 * ------------------------------------------------
 * Reaching the connector means opening a door, and opening a door wakes the
 * car -- so the meter has to be clipped on and read through the glass, and even
 * then it says only "0 V". esp_reset_reason() separates three causes that a
 * voltage reading collapses into one: the supply going away, the supply
 * sagging, and us crashing. Those want three different responses.
 *
 * 🔴 USB DEFEATS THE TEST. Plugging in the cable to watch the serial port also
 * POWERS the board, so the car's feed can vanish without the module noticing.
 * That is the whole reason this writes to NVS instead of just printing: the
 * record has to survive being unpowered and be read back afterwards.
 *
 * The judgement lives in fsd_logic/fsd_power.h and is host-tested. This file
 * owns only NVS, esp_reset_reason(), the clock and Serial.
 *
 * Listen-Only: nothing here transmits.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../../fsd_logic/fsd_power.h"

/**
 * Read the previous session's record, print the verdict, start a new session.
 *
 * Call once from setup(), after Serial is up.
 */
void power_log_init(void);

/**
 * Feed the clock and the bus. Call from loop().
 *
 * NVS writes happen here and nowhere else -- this file follows the same
 * contract as prefs.cpp, because an NVS commit is a flash erase-write that can
 * stall for tens of milliseconds and must not run on the BLE host task.
 *
 * @param last_rx_ms  FSDState.last_rx_ms
 * @param seen_any    whether any frame has ever arrived (last_rx_ms is 0 until
 *                    the first one, and 0 is also a legal clock value)
 */
void power_log_tick(uint32_t now_ms, uint32_t last_rx_ms, bool seen_any);

/** The verdict as it stands, for the serial banner and future BLE reporting. */
FsdPwrVerdict power_log_verdict(void);

/**
 * Print the whole picture on demand: reset reason, the record that was read at
 * boot, the boot verdict, and where this session currently stands.
 *
 * 🔴 THIS EXISTS BECAUSE THE BOOT BANNER CANNOT BE READ (2026-08-17).
 *
 * The board has native USB. Plugging the cable in starts the boot AND the USB
 * enumeration at the same time; the banner is printed around 400 ms and the
 * host cannot open the port for one to two seconds. Measured twice on the bench
 * with a capture script that reopens the port every 200 ms -- the reboot was
 * confirmed both times (the port dropped, and the 20 s factory-reset window was
 * seen closing afterwards) and the banner was gone both times. It is not a race
 * that can be won.
 *
 * That matters far past the bench: 차량-방문-체크리스트 A-1 is a ONE-HOUR test in
 * the car whose final step is "plug USB back in and read the first line". With
 * only a banner there is nothing to read, and the hour is spent for nothing.
 */
void power_log_print(void);
