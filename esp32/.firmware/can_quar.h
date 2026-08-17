#pragma once
/*
 * can_quar.{h,cpp} — refuse to bring up a CAN bus that panicked the board.
 *
 * The decision logic lives in fsd_logic/fsd_canquar.h (pure C, host-tested);
 * this is the ESP32 half: reset reason, NVS, and the "has it survived long
 * enough to be trusted" timer.
 *
 * 🔴 This exists because the loop-side guard (fsd_bushealth.h) was measured
 * losing to the real fault on 2026-08-17 — the loop ran ZERO times between the
 * controller coming up and the interrupt watchdog rebooting the board. Nothing
 * polled from the loop can help. The boot path always runs, so the decision was
 * moved here.
 *
 * Flow:
 *   setup()  can_quar_boot(n)            -> mask of buses to skip
 *            can_quar_mark_trying(i)     -> before each begin(), persisted
 *   loop()   can_quar_prove(now_ms)      -> after a healthy spell, clears the mark
 */

#include <Arduino.h>

/** Read the record, judge the last boot, and report which buses to skip.
 *
 * Call once in setup(), before bringing any bus up. Logs what it decided and
 * why — a quarantine that nobody can see is a module that mysteriously runs on
 * one bus. */
uint8_t can_quar_boot(uint8_t bus_count);

/** True when bus `index` must not be brought up. */
bool can_quar_blocked(uint8_t index);

/** Record that we are about to enable this bus and it has not proven itself.
 * Persisted immediately — the point is to survive the panic that follows. */
void can_quar_mark_trying(uint8_t index);

/** Clear the unproven mark once the module has run healthily for a while.
 * Call every loop; it does the write once and then costs nothing. */
void can_quar_prove(uint32_t now_ms);

/** Operator override — allow every bus again on the next boot. */
void can_quar_clear();

/** One line of status for the serial console. */
void can_quar_print();
