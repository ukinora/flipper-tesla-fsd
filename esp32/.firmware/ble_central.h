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

/** Scan for `secs` and print what is found. On demand only — scanning costs
 *  radio time the server shares. Returns false if a scan is already running. */
bool ble_central_scan(uint8_t secs);

/** Remember this peer and connect to it. Address is "aa:bb:cc:dd:ee:ff".
 *  Persisted, so a button survives the car sleeping. Empty string forgets. */
bool ble_central_bind(const char* addr_str);

/** The bound address, or "" when none. */
const char* ble_central_bound_addr(void);

/** True while a button is connected. */
bool ble_central_connected(void);

/** Raw-report logging: prints every notification as hex. On by default until a
 *  report layout is confirmed, because that log IS the measurement. */
void ble_central_set_verbose(bool on);

/** Counters for the serial line and a future BLE surface. */
uint16_t ble_central_notify_count(void);
uint16_t ble_central_short_presses(void);
uint16_t ble_central_long_presses(void);

#else // no BLE on this variant

static inline void ble_central_init(void) {}
static inline void ble_central_tick(uint32_t) {}
static inline bool ble_central_scan(uint8_t) { return false; }
static inline bool ble_central_bind(const char*) { return false; }
static inline const char* ble_central_bound_addr(void) { return ""; }
static inline bool ble_central_connected(void) { return false; }
static inline void ble_central_set_verbose(bool) {}
static inline uint16_t ble_central_notify_count(void) { return 0; }
static inline uint16_t ble_central_short_presses(void) { return 0; }
static inline uint16_t ble_central_long_presses(void) { return 0; }

#endif
