#pragma once
/*
 * ble_central.h — the module as a BLE CLIENT, so a generic button can drive it.
 *
 * ble_server.cpp makes this module a peripheral for the phone. This makes it a
 * central for a button at the same time — NimBLE supports both roles at once,
 * and the cost of that is measured rather than assumed (see the PR).
 *
 * BUILT AS A BRING-UP TOOL FIRST, AND THAT PAID OFF
 * -------------------------------------------------
 * This was written before a button existed, so it was built to LOOK rather than
 * to parse:
 *
 *    scan   -> print every advertiser: name, address, appearance, service UUIDs
 *    bind   -> remember one address in NVS
 *    connect-> subscribe to every notifying characteristic there is
 *    log    -> print the raw bytes of everything that arrives
 *
 * A remote has since been chosen — a Yiser J6 (차주 결정, 2026-08-18) — and the
 * looking is what made it usable. It is not a keyboard: a press arrives as a
 * synthesised swipe or tap on a digitiser report, and nine distinguishable
 * buttons come down ONE link. Subscribing to everything at once is how that
 * became visible; a parser written for the device we assumed would have found
 * nothing. The decoder is fsd_logic/fsd_btn_j6.h and the measurement is in
 * 블루투스-버튼-조사.md.
 *
 * 🔴 The first reading of that remote was WRONG, and not because of the device.
 * A swipe is up to nine notifications and this file kept one buffer per slot,
 * so eight were overwritten before the loop looked. From what survived the
 * remote appeared to emit nothing usable. Seeing less is not the same as there
 * being less — hence the ring buffer below.
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

/* ── Sizes, defined for EVERY board ───────────────────────────────────────────
 *
 * 🔴 These live ABOVE the #ifdef on purpose. Do not move them inside it.
 *
 * The stub branch below keeps the whole API callable on boards without BLE, so
 * a caller can loop over the slots without guarding — and capability.cpp and
 * main.cpp both do exactly that. If the count only exists on the BLE side, the
 * call still compiles there and the LOOP BOUND does not, which is how seven of
 * the eight board builds broke at once (2026-08-18).
 *
 * Only lilygo-t2can defines BLE_SERVER_ENABLED, so a local build of our own
 * board proves nothing about this. */

/** How many scan results are kept for the app to read. */
#ifndef BLE_CENTRAL_MAX_FOUND
#define BLE_CENTRAL_MAX_FOUND 8
#endif

/** Default scan length when a caller does not say. */
#ifndef BLE_CENTRAL_SCAN_SECS
#define BLE_CENTRAL_SCAN_SECS 5
#endif

/** How many remotes can be bound at once — a RADIO budget, nothing else.
 *
 *  🔴 THIS USED TO SAY "slot index IS the logical button index, so this and
 *  FSD_BTN_MAX are the same number". That stopped being true when the J6 put
 *  ten logical buttons on ONE link, and a comment asserting a retired invariant
 *  is worse than none: someone reconciling the two would raise this to 10, and
 *  10 + the phone exceeds the prebuilt controller — which boot-loops the board
 *  (PR #57). The static_asserts below are what actually holds them apart.
 *
 *  🔴 FIVE IS MEASURED, NOT CHOSEN. The Arduino core ships a controller built
 *  for six simultaneous links (CONFIG_BT_CTRL_BLE_MAX_ACT=6) and the phone
 *  holds one of them. This was 8 for a day, which boot-looped the board — see
 *  the static_assert below.
 *
 *  Bind only what is used even so: every extra link is radio time the phone's
 *  transfers give up, and the one-shot TSL capture goes over that link. */
#ifndef BLE_CENTRAL_MAX_BUTTONS
#define BLE_CENTRAL_MAX_BUTTONS 5
#endif

/** Events a row can carry: short, double, long.
 *
 * 🔴 ABOVE THE #ifdef, like the counts around it. capability.cpp uses this to
 * size a loop and is compiled for every board, so hiding it behind
 * BLE_SERVER_ENABLED breaks the seven boards that do not define it — which is
 * exactly how this file broke eight builds once already. Only lilygo-t2can
 * defines it, so a local build of our own board proves nothing. */
#ifndef FSD_BTN_EVENTS
#define FSD_BTN_EVENTS 3u
#endif

/* ── The slot count is not free. It is spent out of the radio's link budget ──
 *
 * 🔴 This was a boot loop, not a warning (2026-08-18). Asking NimBLE for more
 * links than the controller was BUILT with gives
 *
 *     E BLE_INIT: Invalid value of ble_max_act
 *     assert failed: npl_freertos_mutex_pend
 *
 * every boot, forever. The controller is a PREBUILT library in the Arduino
 * core: CONFIG_BT_CTRL_BLE_MAX_ACT is fixed at compile time of that library and
 * a -D of ours cannot move it. Nothing warned — it built clean and shipped.
 *
 * So the relationship is made a compile error instead. Note it is NOT the same
 * as "does it boot": a too-large MAX_CONNECTIONS still boots (measured: 7 boots
 * fine on a controller built for 6) and then the extra remote simply never
 * connects, with nothing in the log. Booting proves nothing here.
 *
 * The phone holds one link at all times, hence the +1. */
/* 🔴 PULL THE SDK CONFIG IN OURSELVES, or the guard below is not a guard.
 *
 * CONFIG_BT_CTRL_BLE_MAX_ACT is defined in the framework's sdkconfig.h, which
 * arrives via <Arduino.h>. In ble_central.cpp and capability.cpp this header is
 * the FIRST include, so the macro was not in scope and the #ifdef around the
 * assertion read false — the guard silently vanished in exactly the two files
 * where someone would go to change the number. It survived only because
 * main.cpp and ble_server.cpp happen to include Arduino.h first (red team,
 * 2026-08-19).
 *
 * A protection that depends on the include order of unrelated files is not a
 * protection. */
#if defined(__has_include)
#  if __has_include(<sdkconfig.h>)
#    include <sdkconfig.h>
#  endif
#endif

#if defined(BLE_SERVER_ENABLED) && defined(CONFIG_BT_NIMBLE_MAX_CONNECTIONS)

static_assert(BLE_CENTRAL_MAX_BUTTONS + 1 <= CONFIG_BT_NIMBLE_MAX_CONNECTIONS,
              "BLE_CENTRAL_MAX_BUTTONS + the phone exceeds "
              "CONFIG_BT_NIMBLE_MAX_CONNECTIONS (platformio.ini)");

#ifdef CONFIG_BT_CTRL_BLE_MAX_ACT
static_assert(CONFIG_BT_NIMBLE_MAX_CONNECTIONS <= CONFIG_BT_CTRL_BLE_MAX_ACT,
              /* ASCII only: the toolchain escapes non-ASCII in this message,
                 and this is read at the moment something is already wrong. */
              "CONFIG_BT_NIMBLE_MAX_CONNECTIONS exceeds the PREBUILT controller "
              "(CONFIG_BT_CTRL_BLE_MAX_ACT) - this boot-loops the board");
#else
/* Reaching here means NimBLE is configured but the controller's own limit is
 * not visible, so the check above would be skipped in silence. Say so loudly
 * instead: a vanished guard reads exactly like a passing one. */
#error "CONFIG_BT_CTRL_BLE_MAX_ACT not in scope - the boot-loop guard would vanish. Include <sdkconfig.h> before ble_central.h."
#endif

#endif

#ifdef BLE_SERVER_ENABLED

/** Start the client. Call after ble_server_init() — NimBLEDevice::init() must
 *  already have happened, and this deliberately does not call it again. */
void ble_central_init(void);

/** Pump from loop(): drives reconnects and the button tick. Cheap when idle. */
void ble_central_tick(uint32_t now_ms);

/** Start a scan. **Returns immediately** — results arrive on the host task and
 *  ble_central_scanning() goes false when it finishes.
 *
 *  🔴 It used to block for the whole scan, and that made the phone's "find
 *  buttons" unusable: five seconds without answering GATT reads drops the
 *  link, so the result had nowhere to arrive. Do not put a blocking call back
 *  here — a scan that works but disconnects the app is not a working scan.
 *
 *  Returns false when one is already running. */
bool ble_central_scan(uint8_t secs);

/** How many devices the last scan SAW, before keeping only the first
 *  BLE_CENTRAL_MAX_FOUND. Reporting only what we kept reads as "this is all
 *  there is", and someone whose remote is missing then looks in the wrong
 *  place. */
uint16_t ble_central_found_total(void);

/** Park a scan for loop() to run. Safe from a BLE characteristic callback. */
void ble_central_request_scan(uint8_t secs);

/** True while a parked scan is running. */
bool ble_central_scanning(void);

/** How many devices the last scan kept (capped at BLE_CENTRAL_MAX_FOUND). */
uint8_t ble_central_found_count(void);

/** Read one result. Pointers stay valid until the next scan. */
bool ble_central_found(uint8_t i, const char** addr, const char** name, int8_t* rssi);

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

/** How many presses this slot's remote has produced that we could actually
 *  DECODE into a button.
 *
 *  🔴 Not the same as connected, and the difference is the whole point. A
 *  remote we do not understand connects perfectly and sits at zero forever;
 *  without this the app draws it as working. Zero is also the normal state
 *  right after binding — "not pressed yet" rather than "broken" — so the app
 *  has to say those two differently. */
uint16_t ble_central_slot_decoded(uint8_t slot);

/* ── per logical button ──────────────────────────────────────────────────────
 *
 * These are about BUTTONS, not links. One remote carries nine of them, so a
 * per-slot answer cannot address them (fsd_btn_j6.h). */

/** How many times THIS ROW has fired. Row encoding as above.
 *
 *  🔴 PER ROW, NOT PER BUTTON. It was per button — short, double and long added
 *  together — and the screen then drew that one number on all three of a
 *  button's lines, so pressing 6번 once appeared to move its 1회, 2회 AND 길게
 *  counts at once. The number exists to tell those lines APART; summing them
 *  destroyed exactly the information it was for.
 *
 *  Clamped to 255. This is evidence that a line moved, not a statistic. */
uint8_t ble_central_row_events(uint8_t row);

/* ── what each input is supposed to DO ───────────────────────────────────────
 *
 * A button is not one thing. 6번 alone offers a tap, a double tap and a hold,
 * and each is a separate place to hang an action. So the unit here is a ROW —
 * (logical button, event) — not a button.
 *
 *    row = button * FSD_BTN_EVENTS + event,  event 0 short / 1 double / 2 long
 *
 * Today the only choice is on or off, because no vehicle action has been
 * captured yet. Stored anyway, and stored HERE rather than in the phone: the
 * module is what will act, and a mapping the module does not know is a mapping
 * that stops working the moment the phone is elsewhere. Same reasoning as
 * autonomy_enabled.
 *
 * 🔴 The DOUBLE rows are not just intent. Enabling one turns on the wait for a
 * second tap, which delays that button's single press — so this mask replaced
 * the separate double-press mask rather than sitting beside it. Two masks would
 * be two truths about the same thing, and they would drift. */

/** Turn one row on or off. Persisted. Enabling a DOUBLE row also starts the
 *  double-press wait on that button; disabling it stops it. */
void ble_central_set_action(uint8_t row, bool on);

/** Bitmask of enabled rows, bit `row` = that row. */
uint32_t ble_central_action_mask(void);



/** True when this firmware carries a decoder that was measured against a real
 *  device, rather than a guess waiting to be confirmed.
 *
 *  This used to be hardcoded `false` in the capability document because no
 *  button had been bought. It is a function now so that the answer cannot
 *  freeze again while the code around it changes. */
bool ble_central_decoder_verified(void);

/** How many slots hold an address. */
uint8_t ble_central_bound_count(void);

/** True when at least one remote is connected. */
bool ble_central_any_connected(void);

/** Raw-report logging: prints every notification as hex. On by default until a
 *  report layout is confirmed, because that log IS the measurement. */
void ble_central_set_verbose(bool on);

/* ── bring-up instruments ────────────────────────────────────────────────────
 *
 * A remote that connects and then says nothing has three possible reasons and
 * they need different fixes. These two measure rather than guess. Neither is
 * called on its own and neither changes how a bound button behaves.
 *
 * 🔴 BLOCKS for `secs`, like ble_central_scan(). Serial console only. */

/** Print one address's FULL advertisement, every time it is seen.
 *
 *  The scan list only shows name and RSSI. If the press is carried in the
 *  advertisement — manufacturer data, a counter, a rolling code — this is the
 *  only place it becomes visible. Seeing nothing is a result: some buttons
 *  advertise only while held. */
bool ble_central_raw(const char* addr_str, uint8_t secs);

/** Count ADVERTISEMENT PACKETS from one address, not devices.
 *
 * ble_central_raw() answers "did it advertise", and that is all it can answer:
 * the result set it reads collapses packets with identical content, so a burst
 * of twenty shows up as one. This counts every packet the radio delivers, and
 * groups them by payload so a burst can be told from a second press.
 *
 * The question it exists for is how long the radio has to stay on. A remote
 * that repeats its packet for half a second can be caught by a scanner that
 * listens a tenth of the time; one that sends a single packet cannot be caught
 * by anything less than a continuous scan, and continuous scan is parked-car
 * battery drain. Guessing that wrong is expensive in both directions.
 */
bool ble_central_count(const char* addr_str, uint8_t secs);


/** Ask a connected remote to encrypt the link.
 *
 *  Many peripherals withhold notifications until the link is paired. We never
 *  initiate pairing otherwise, so this is the one way to rule that out. Uses
 *  the security parameters the server role already set — the phone's link is
 *  untouched — but it does spend one bond slot. */
bool ble_central_secure(uint8_t slot);

/** Print the connected remote's whole GATT table with properties, and the
 *  current value of anything readable.
 *
 *  subscribe_all() only ever asked "what notifies". That is the right first
 *  question and the wrong second one: a remote that stays silent may be waiting
 *  to be written to, and a write target is invisible from a notify-only view. */
bool ble_central_chars(uint8_t slot);

/** Try a short list of conventional wake values against every write target.
 *
 *  🔴 Not a search — that space is 2^n. A bounded knock: if none of these
 *  starts it talking, the answer has to come from watching the real app, and
 *  knowing that quickly is the whole value. */
bool ble_central_poke(uint8_t slot);

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
static inline uint16_t ble_central_found_total(void) { return 0; }
static inline bool ble_central_found(uint8_t, const char**, const char**, int8_t*) { return false; }
static inline uint16_t ble_central_double_presses(void) { return 0; }
static inline int ble_central_add(const char*) { return -1; }
static inline int ble_central_add_found(uint8_t) { return -1; }
static inline bool ble_central_forget(uint8_t) { return false; }
static inline void ble_central_forget_all(void) {}
static inline const char* ble_central_slot_addr(uint8_t) { return ""; }
static inline bool ble_central_slot_connected(uint8_t) { return false; }
static inline uint16_t ble_central_slot_decoded(uint8_t) { return 0; }
static inline bool ble_central_decoder_verified(void) { return false; }
static inline uint8_t ble_central_row_events(uint8_t) { return 0; }
static inline void ble_central_set_action(uint8_t, bool) {}
static inline uint32_t ble_central_action_mask(void) { return 0; }
static inline uint8_t ble_central_bound_count(void) { return 0; }
static inline bool ble_central_any_connected(void) { return false; }
static inline void ble_central_set_verbose(bool) {}
static inline bool ble_central_raw(const char*, uint8_t) { return false; }
static inline bool ble_central_count(const char*, uint8_t) { return false; }
static inline bool ble_central_secure(uint8_t) { return false; }
static inline bool ble_central_chars(uint8_t) { return false; }
static inline bool ble_central_poke(uint8_t) { return false; }
static inline uint16_t ble_central_notify_count(void) { return 0; }
static inline uint16_t ble_central_short_presses(void) { return 0; }
static inline uint16_t ble_central_long_presses(void) { return 0; }

#endif
