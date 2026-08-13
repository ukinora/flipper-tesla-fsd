#pragma once
/*
 * ble_owner.h — remembers which phone this module belongs to.
 *
 * The link is encrypted and bonded, but nothing restricts who may bond: this
 * board has no display or keypad, so Just Works is the only pairing available.
 * Without this file, any phone in radio range of a parked car could pair and
 * then send SET_MODE(Active) -- which opens CAN transmit -- or download a
 * capture, which contains the VIN.
 *
 * First phone to bond becomes the owner, with no ceremony, so ordinary setup is
 * unchanged. Enrolling a different phone afterwards needs the physical button,
 * which means being inside the car.
 *
 * The decision itself lives in fsd_logic/fsd_owner.h and is host-tested. This
 * file owns NVS, the clock and the serial log.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../../fsd_logic/fsd_owner.h"

/** How long the button holds the door open for a different phone. */
#define BLE_OWNER_WINDOW_MS 120000u

/** Load the enrolled owner. Call once from setup(). */
void ble_owner_init(void);

/**
 * Persist a pending enrolment and expire the button window. Call from loop().
 *
 * 🔴 NVS writes happen HERE and nowhere else. ble_owner_on_bond() runs on the
 * BLE host task, and committing flash from that task is a contract violation
 * this project has already had to fix once (a red-team finding: an NVS write
 * stalls for tens of milliseconds and the BLE host must not block).
 */
void ble_owner_tick(uint32_t now_ms);

/**
 * A peer finished bonding. Safe to call from the BLE host task.
 *
 * Decides, updates the in-RAM owner, and queues the write for ble_owner_tick().
 */
void ble_owner_on_bond(uint8_t addr_type, const uint8_t* addr);

/**
 * May this peer drive the module?
 *
 * Called per write rather than cached per connection on purpose: the callback
 * order for bonding, identity resolution and the first write is not something
 * this code should have to be right about, and asking at the point of use
 * cannot be stale.
 */
bool ble_owner_allows(uint8_t addr_type, const uint8_t* addr);

/** Button: let a different phone take over for BLE_OWNER_WINDOW_MS. */
void ble_owner_open_window(uint32_t now_ms);

/** True while that window is open. */
bool ble_owner_window_open(void);

/** Forget the owner, so the next phone to bond is enrolled. */
void ble_owner_forget(void);

/** Print the current enrolment for the serial console. */
void ble_owner_print(void);
