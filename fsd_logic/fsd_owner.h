#pragma once
/*
 * fsd_owner.h — which phone is allowed to drive this module.
 *
 * WHY THIS EXISTS
 * ---------------
 * The BLE link is encrypted and bonded, but nothing restricts WHO may bond.
 * This board has no display and no keypad, so a passkey cannot be shown or
 * typed, and requiring MITM protection just makes the stack refuse every
 * command (it did: see ble_server.cpp's WRITE_ENC note). Just Works pairing is
 * therefore the only pairing available -- which means any phone within radio
 * range of a parked car could pair and then send SET_MODE(Active), opening CAN
 * transmit, or pull a capture that contains the VIN.
 *
 * So the control moves up a layer: the module remembers ONE owner and refuses
 * commands from anyone else. Encryption says "nobody is listening in"; this
 * says "and you are the phone I was set up with".
 *
 * TRUST ON FIRST USE, THEN THE BUTTON
 * -----------------------------------
 * The first phone to bond becomes the owner with no ceremony, so ordinary
 * first-time setup is unchanged. After that, enrolling a different phone needs
 * someone to press the physical button on the module -- which means being
 * inside the car. That is the strongest thing this hardware can actually prove.
 *
 * It is not proof against an attacker who is present at the very moment of
 * first pairing. Nothing this board can do is: with no display, there is no
 * channel to confirm a peer out-of-band.
 *
 * Pure: no NVS, no BLE, no clock. ble_owner.cpp owns those.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FSD_OWNER_ADDR_LEN 6u

/** The enrolled owner, as persisted. `enrolled == false` means nobody yet. */
typedef struct {
    bool    enrolled;
    uint8_t type;                       /* BLE address type, compared too */
    uint8_t addr[FSD_OWNER_ADDR_LEN];   /* identity address, not the RPA */
} FsdOwner;

typedef enum {
    FSD_OWNER_ENROLL = 0, /* take this peer as the owner and persist it */
    FSD_OWNER_MATCH,      /* this is the owner */
    FSD_OWNER_REFUSE,     /* someone else — refuse commands */
    FSD_OWNER_BAD_ADDR,   /* the peer has no usable identity yet */
} FsdOwnerVerdict;

/**
 * Is this a usable identity address?
 *
 * 🔴 All-zero is the trap. A peer whose identity has not been resolved yet
 * reports 00:00:00:00:00:00, and enrolling THAT would both lock out the real
 * owner forever and match every future unresolved peer — the exact opposite of
 * what this file is for. Treated as "no identity", never as an address.
 */
bool fsd_owner_addr_valid(uint8_t type, const uint8_t* addr);

/**
 * Decide what to do about a peer that has just bonded.
 *
 * @param stored       what is in NVS (NULL is treated as not enrolled)
 * @param window_open  the operator pressed the button recently, so a NEW owner
 *                     may replace the stored one
 */
FsdOwnerVerdict fsd_owner_check(const FsdOwner* stored, uint8_t type,
                                const uint8_t* addr, bool window_open);

/** True when the two identities are the same peer. */
bool fsd_owner_same(const FsdOwner* stored, uint8_t type, const uint8_t* addr);

const char* fsd_owner_verdict_str(FsdOwnerVerdict v);

/* ── which LINK the owner is on ──────────────────────────────────────────────
 *
 * Identity answers "may this peer command us". It does not answer "is the
 * session we are serving still alive", and those came apart in a way that only
 * shows up when TWO peers are connected at once.
 *
 * The module serves one session at a time, remembered as a connection handle.
 * Nothing made that handle be the OWNER's handle: whoever connected first got
 * it. So with a stranger connected first and the owner second --
 * ordinary when a squatter holds the slot and the phone reconnects directly to
 * a cached address -- the owner's disconnect was filed as "some other client
 * left, our session is untouched" and every teardown was skipped:
 *
 *   - owner-present stayed true, so the 30 s Active recovery NEVER started;
 *     the car drives away with a live transmitter and nobody to take it back
 *   - a parked SET_MODE(Active) from the phone that just left still applied,
 *     because its guard asked "is anyone connected" and the stranger was
 *   - an interrupted camera.bin upload was never aborted
 *
 * These two predicates are that decision, kept pure so the host tests can hold
 * a two-peer sequence still. BLE_CONN_NONE (0xFFFF) means "nobody".
 */
#define FSD_BLE_CONN_NONE 0xFFFFu

/**
 * Does a disconnect on `who` end the session we are serving?
 *
 * @param served     handle of the session being served, or FSD_BLE_CONN_NONE
 * @param owner      handle the owner authenticated on, or FSD_BLE_CONN_NONE
 * @param who        handle that just went away
 * @param remaining  links still up AFTER this one left
 *
 * True for the served link, for the owner's link even when it is not the served
 * one, and for "nobody is left" -- the last is the safety net that keeps a
 * handle which somehow went stale from wedging the module forever.
 */
bool fsd_ble_session_teardown(uint16_t served, uint16_t owner,
                              uint16_t who, uint32_t remaining);

/**
 * May work the BLE task parked for loop() still be applied?
 *
 * @param owner_present  the owner is authenticated on the link right now
 * @param served         handle currently being served
 * @param parked_on      handle that asked for the work
 *
 * 🔴 "Somebody is connected" is not the question. Granting Active because a
 * STRANGER happens to be connected hands CAN transmit to a phone that already
 * left, and the recovery timer is armed on disconnect -- which already ran.
 * The permission would never be taken back.
 */
bool fsd_ble_session_apply_ok(bool owner_present, uint16_t served,
                              uint16_t parked_on);

#ifdef __cplusplus
}
#endif
