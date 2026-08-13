#include "fsd_owner.h"

#include <stddef.h>

bool fsd_owner_addr_valid(uint8_t type, const uint8_t* addr) {
    (void)type;
    if(!addr) return false;

    /* All-zero means "identity not resolved yet", not "the peer at 00:00:...".
     * Enrolling it would lock out the real owner permanently AND match every
     * later peer whose identity has not resolved either. */
    for(unsigned i = 0; i < FSD_OWNER_ADDR_LEN; i++) {
        if(addr[i] != 0u) return true;
    }
    return false;
}

bool fsd_owner_same(const FsdOwner* stored, uint8_t type, const uint8_t* addr) {
    if(!stored || !stored->enrolled || !addr) return false;

    /* Type is part of the identity. The same six bytes as a public address and
     * as a random static address are two different peers, and comparing only
     * the bytes would let one stand in for the other. */
    if(stored->type != type) return false;

    for(unsigned i = 0; i < FSD_OWNER_ADDR_LEN; i++) {
        if(stored->addr[i] != addr[i]) return false;
    }
    return true;
}

FsdOwnerVerdict fsd_owner_check(const FsdOwner* stored, uint8_t type,
                                const uint8_t* addr, bool window_open) {
    if(!fsd_owner_addr_valid(type, addr)) return FSD_OWNER_BAD_ADDR;

    /* Nobody enrolled: trust on first use. Deliberate -- ordinary first-time
     * setup should not need a ceremony, and a module that refuses everyone
     * until a button is pressed is a module whose first boot looks broken. */
    if(!stored || !stored->enrolled) return FSD_OWNER_ENROLL;

    if(fsd_owner_same(stored, type, addr)) return FSD_OWNER_MATCH;

    /* Someone else. The button is the only way past this, and pressing it means
     * being inside the car.
     *
     * Checked AFTER the match: with the window open, the existing owner
     * reconnecting must stay MATCH rather than be re-enrolled, so that opening
     * the window and then walking away cannot leave the enrollment in a
     * different state than it started in. */
    if(window_open) return FSD_OWNER_ENROLL;

    return FSD_OWNER_REFUSE;
}

const char* fsd_owner_verdict_str(FsdOwnerVerdict v) {
    switch(v) {
    case FSD_OWNER_ENROLL: return "enrolled as the owner";
    case FSD_OWNER_MATCH: return "the owner";
    case FSD_OWNER_REFUSE: return "NOT the owner - commands refused";
    case FSD_OWNER_BAD_ADDR: return "no resolved identity";
    default: return "unknown";
    }
}
