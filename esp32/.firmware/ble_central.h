#pragma once
/*
 * ble_central.h — the module as a BLE CLIENT, so a generic button can drive it.
 *
 * ble_server.cpp makes this module a peripheral for the phone. This makes it a
 * central for a button at the same time — NimBLE supports both roles at once,
 * and the cost of that is measured rather than assumed (see the PR).
 *
 * BUILT AS A BRING-UP TOOL FIRST, ON PURPOSE
 * ------------------------------------------
 * No button has been chosen yet, and a BLE button can be any of three things:
 * an HID-over-GATT keyboard or consumer-control, a vendor characteristic that
 * notifies a byte, or a device that changes its ADVERTISEMENT on press and
 * never accepts a connection at all. Which one it is decides most of the code.
 *
 * Writing a parser for a device nobody has is the same mistake as writing an
 * emitter for a frame nobody has captured. So the first job here is to LOOK:
 *
 *    scan   -> print every advertiser: name, address, appearance, service UUIDs
 *    bind   -> remember one address in NVS
 *    connect-> subscribe to every notifying characteristic there is
 *    log    -> print the raw bytes of everything that arrives
 *
 * From that log the report layout is obvious in about a minute, and it goes in
 * the table below. Until then FSD_BTN_MAP.verified is false and no report is
 * interpreted as a press — the same discipline as FsdSpEncoding.
 *
 * PRESSES DO NOTHING YET
 * ----------------------
 * The only action a button is meant to drive is the speed-profile step, and
 * that is locked behind fsd_sp_encoding_ok() and FsdSpInputs.tx_armed. Presses
 * are classified by fsd_button.h, counted, and logged. Nothing transmits.
 *
 * CONNECTION SLOTS ARE A REAL BUDGET
 * ----------------------------------
 * NimBLE allows a small fixed number of simultaneous links, shared between the
 * phone (peripheral role) and any buttons (central role). The reference project
 * this one benchmarks against saw its usable device count drop from 20-30 to
 * about 10 after adding another BLE consumer. So the phone is never displaced:
 * scanning is on demand and stops on its own, and the client gives up rather
 * than retrying forever.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef BLE_SERVER_ENABLED

/** Start the client. Call after ble_server_init() — NimBLEDevice::init() must
 *  already have happened, and this deliberately does not call it again. */
void ble_central_init(void);

/** Pump from loop(): drives reconnects and the button tick. Cheap when idle. */
void ble_central_tick(uint32_t now_ms);

/** How many scan results are kept for the app to read. */
#ifndef BLE_CENTRAL_MAX_FOUND
#define BLE_CENTRAL_MAX_FOUND 8
#endif

/** Default scan length when a caller does not say. */
#ifndef BLE_CENTRAL_SCAN_SECS
#define BLE_CENTRAL_SCAN_SECS 5
#endif

/** Scan for `secs` and print what is found. On demand only — scanning costs
 *  radio time the server shares. Returns false if a scan is already running.
 *
 *  🔴 BLOCKS for `secs`. Never call this from a BLE callback — use
 *  ble_central_request_scan() there. */
bool ble_central_scan(uint8_t secs);

/** Park a scan for loop() to run. Safe from a BLE characteristic callback. */
void ble_central_request_scan(uint8_t secs);

/** True while a parked scan is running. */
bool ble_central_scanning(void);

/** How many devices the last scan kept (capped at BLE_CENTRAL_MAX_FOUND). */
uint8_t ble_central_found_count(void);

/** Read one result. Pointers stay valid until the next scan. */
bool ble_central_found(uint8_t i, const char** addr, const char** name, int8_t* rssi);

/** Bind by position in the last scan — the app has an index, not an address. */
bool ble_central_bind_index(uint8_t i);

/** How many remotes can be bound at once. Slot index IS the logical button
 *  index, so this and FSD_BTN_MAX are the same number.
 *
 *  Eight is the radio's ceiling, not a target: the ESP controller allows nine
 *  simultaneous links and the phone holds one. Bind only what is used — every
 *  extra link is radio time the phone's transfers give up. */
#ifndef BLE_CENTRAL_MAX_BUTTONS
#define BLE_CENTRAL_MAX_BUTTONS 8
#endif

/** Bind an address into the first free slot. Returns the slot, or -1 when all
 *  are taken. Re-binding an address already held returns its existing slot
 *  rather than consuming a second one. Persisted. */
int ble_central_add(const char* addr_str);

/** Bind the Nth device from the last scan. Returns the slot, or -1. */
int ble_central_add_found(uint8_t scan_index);

/** Drop one slot: disconnect, free the client, clear NVS. */
bool ble_central_forget(uint8_t slot);

/** Drop every slot. */
void ble_central_forget_all(void);

/** Address bound to a slot, or "" when the slot is free. */
const char* ble_central_slot_addr(uint8_t slot);

/** Whether that slot's remote is connected right now. */
bool ble_central_slot_connected(uint8_t slot);

/** How many slots hold an address. */
uint8_t ble_central_bound_count(void);

/** True when at least one remote is connected. */
bool ble_central_any_connected(void);

/** Raw-report logging: prints every notification as hex. On by default until a
 *  report layout is confirmed, because that log IS the measurement. */
void ble_central_set_verbose(bool on);

/** Counters for the serial line and a future BLE surface. */
uint16_t ble_central_notify_count(void);
uint16_t ble_central_short_presses(void);
uint16_t ble_central_long_presses(void);
uint16_t ble_central_double_presses(void);

#else // no BLE on this variant

static inline void ble_central_init(void) {}
static inline void ble_central_tick(uint32_t) {}
static inline bool ble_central_scan(uint8_t) { return false; }
static inline void ble_central_request_scan(uint8_t) {}
static inline bool ble_central_scanning(void) { return false; }
static inline uint8_t ble_central_found_count(void) { return 0; }
static inline bool ble_central_found(uint8_t, const char**, const char**, int8_t*) { return false; }
static inline uint16_t ble_central_double_presses(void) { return 0; }
static inline int ble_central_add(const char*) { return -1; }
static inline int ble_central_add_found(uint8_t) { return -1; }
static inline bool ble_central_forget(uint8_t) { return false; }
static inline void ble_central_forget_all(void) {}
static inline const char* ble_central_slot_addr(uint8_t) { return ""; }
static inline bool ble_central_slot_connected(uint8_t) { return false; }
static inline uint8_t ble_central_bound_count(void) { return 0; }
static inline bool ble_central_any_connected(void) { return false; }
static inline void ble_central_set_verbose(bool) {}
static inline uint16_t ble_central_notify_count(void) { return 0; }
static inline uint16_t ble_central_short_presses(void) { return 0; }
static inline uint16_t ble_central_long_presses(void) { return 0; }

#endif
