#pragma once
/*
 * rules_store.h — the owner's rule table: one instance, and NVS.
 *
 * 🔴 IT STORES. IT DOES NOT RUN. (2026-09-02)
 * -------------------------------------------
 * Nothing here calls fsd_rules_match(), and nothing here is fed a CAN frame.
 * The rules are accepted, validated, kept across reboots and handed back — and
 * that is the whole of it.
 *
 * That is deliberate and it is not laziness:
 *
 *   - There is still NO EMITTER. fsd_body.h says so at length: no frame is
 *     constructed anywhere in this firmware, six of seven action rows have
 *     armable_at_runtime = false, and send_on_bus() refuses the body IDs
 *     outright. A matcher wired up today would produce decisions that three
 *     separate gates would then throw away.
 *   - The first write to the car is scheduled to happen IN THE CAR, on purpose.
 *     Wiring the trigger layer to the rule engine here would mean the first time
 *     anybody sees the two run together is also the first time the car is
 *     listening.
 *
 * So the trigger layer is not instantiated here either. When it is, it belongs
 * in this file (loop task, one owner) and the header comment above has to stop
 * being true before any of it is switched on.
 *
 * WHERE IT SITS
 * -------------
 *      phone (BLE RULES characteristic)  --write-->  ble_server.cpp
 *                                                        | parks it
 *      loop()  ---------------------------------------> THIS FILE
 *                                                        | fsd_rules_set()
 *                                                     NVS + the 288-byte view
 *
 * 🔴 ONE TASK OWNS THE TABLE, AND IT IS loop().
 * ---------------------------------------------
 * camera_store.cpp needs a mutex because two tasks genuinely touch it: the
 * upload runs on the NimBLE host and the judgement runs on loop(). body_task.cpp
 * needs none because everything it does happens on loop(), and its header says
 * so outright — "nothing here may be called from a NimBLE callback".
 *
 * This file is the second kind. A BLE write PARKS thirteen bytes in
 * ble_server.cpp and loop() brings them here, exactly the way SET_MODE and the
 * button bind already work. That buys three things at once: no lock on a path
 * that runs per CAN frame later, the NVS commit lands on the task that is
 * allowed to block, and the answer the phone gets is the verdict
 * fsd_rules_set() actually returned rather than a guess made before it ran.
 *
 * 🔴 EVERY FUNCTION BELOW IS LOOP-TASK ONLY. No exceptions, including the
 * read-only ones: rules_store_render() walks the table, and a BLE callback
 * reading it while loop() writes it is the tear this design exists to avoid.
 */

#include <stdbool.h>
#include <stdint.h>

/* Available on every variant, because main.cpp and prefs.cpp call them
 * unconditionally. The rest of the API is behind the same guard the rule core
 * itself is: fsd_rules.c is only in this variant's build_src_filter. */

/** Load the table from NVS. Call once from setup(). */
void rules_store_init(void);

/** Commit a pending change. Call from loop().
 *
 *  🔴 NVS writes happen HERE and nowhere else, for the reason ble_owner.h gives:
 *  a flash erase-write stalls for tens of milliseconds and the BLE host task
 *  must not block. */
void rules_store_tick(uint32_t now_ms);

/** Print the stored rules for a person. The serial `rules` command.
 *
 *  Exists because this repo has been bitten six times by a feature with no
 *  reachable caller, and because there is no PC in the car — when the phone
 *  says something unexpected, the only other window into the module is this. */
void rules_store_print(void);

/** Erase the table synchronously. Factory reset only.
 *
 *  🔴 prefs_clear() erases the "fsd" namespace and nothing else. That already
 *  left the BLE owner and the pairing keys behind once, while the log said
 *  everything was gone; the rules live in their own namespace too, so without
 *  this call a factory reset would hand the next person the previous owner's
 *  mappings. */
void rules_store_erase_now(void);

#ifdef BLE_SERVER_ENABLED

#include "../../fsd_logic/fsd_rules.h"

/** Render the whole table into the 288 bytes the RULES characteristic carries.
 *  Slot order; the position is the rule number. */
void rules_store_render(uint8_t out[FSD_RULES_WIRE_LEN]);

/** Store one rule from its twelve wire bytes. Returns an FsdRuleVerdict.
 *
 *  Refuses a rule that could never fire — fsd_rules_set() does that, and the
 *  refusal is the whole point of routing through it: the app must not be able to
 *  offer the owner a mapping the module would silently ignore. */
uint8_t rules_store_set_packed(uint8_t idx, const uint8_t in[FSD_RULE_WIRE_LEN]);

/** Blank one rule, or every rule when idx is 0xFF. Returns an FsdRuleVerdict. */
uint8_t rules_store_clear(uint8_t idx);

/** Bumped whenever the table changes. The notifier compares it rather than
 *  being called back, so nothing on the BLE side can end up holding a pointer
 *  into a table loop() is rewriting. Starts at 1, so a listener that has seen
 *  "0" has genuinely seen nothing. */
uint32_t rules_store_revision(void);

#endif // BLE_SERVER_ENABLED
