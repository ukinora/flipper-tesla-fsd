/*
 * fsd_autonomy.c — see fsd_autonomy.h.
 */

#include "fsd_autonomy.h"

// DI_gear (0x286 DI_state, party bus)
//   opendbc tesla_model3_party.dbc:  SG_ DI_gear : 21|3@1+ (1,0)
//   little-endian, start bit 21, 3 bits wide -> byte 2, bits [7:5].
#define GEAR_BYTE 2u
#define GEAR_SHIFT 5u
#define GEAR_MASK 0x07u

// buckleStatus (0x311 UI_warning, party bus)
//   opendbc:  SG_ buckleStatus : 13|1@0+ (1,0)   1 = LATCHED
//   big-endian, start bit 13 -> byte 1, bit 5. Same position the Flipper's
//   fsd_handle_ui_warning() already uses.
#define BELT_BYTE 1u
#define BELT_SHIFT 5u

/* A frame shorter than the byte we need is not a short read, it is a different
 * frame: same ID, different layout, or a truncated capture. Parsing it would
 * invent a gear. Return without stamping so freshness keeps reporting the last
 * frame we actually understood. */
void fsd_drive_observe_gear(FSDState* state, const CANFRAME* frame, uint32_t now_ms) {
    if(!state || !frame) return;
    if(frame->data_lenght <= GEAR_BYTE) return;

    state->di_gear = (uint8_t)((frame->buffer[GEAR_BYTE] >> GEAR_SHIFT) & GEAR_MASK);
    state->di_gear_ms = now_ms;
    state->di_gear_seen = true;

    /* DI_cruiseState, bit 12|3 little-endian -> byte 1 bits [6:4].
     *
     * Not needed by the gate. It is here because on the ESP32 this is the ONLY
     * parser for 0x286 — that build does not compile fsd_logic/fsd_handler.c —
     * so di_cruise_state has been going out over BLE as a structural zero.
     * A field that is always zero for a reason nobody can see costs more to
     * debug later than it costs to fill now. */
    if(frame->data_lenght > 1) state->di_cruise_state = (frame->buffer[1] >> 4) & 0x07u;
}

void fsd_drive_observe_belt(FSDState* state, const CANFRAME* frame, uint32_t now_ms) {
    if(!state || !frame) return;
    if(frame->data_lenght <= BELT_BYTE) return;

    state->ui_buckle_status = ((frame->buffer[BELT_BYTE] >> BELT_SHIFT) & 0x01u) != 0u;
    state->belt_seen_ms = now_ms;
    state->belt_seen = true;

    /* Same reasoning as the cruise state above: the blinker bits ride in this
     * frame, this is the ESP32's only parser for it, and the BLE State packet
     * has been reporting both of them as zero regardless of the stalk.
     *   leftBlinkerOn  bit 22|1 big-endian -> byte 2 bit 6
     *   rightBlinkerOn bit 23|1 big-endian -> byte 2 bit 7
     *   anyDoorOpen    bit 28|1 big-endian -> byte 3 bit 4 */
    if(frame->data_lenght > 2) {
        state->ui_left_blinker = ((frame->buffer[2] >> 6) & 0x01u) != 0u;
        state->ui_right_blinker = ((frame->buffer[2] >> 7) & 0x01u) != 0u;
    }
    if(frame->data_lenght > 3)
        state->ui_any_door_open = ((frame->buffer[3] >> 4) & 0x01u) != 0u;
    state->ui_warning_seen = true;
}

/* Unsigned subtraction so the millisecond clock wrapping past 2^32 (~49 days of
 * uptime) reads as a small elapsed time rather than an enormous one. Same
 * pattern as fsd_rx_is_stale(). */
static bool stale(uint32_t stamp, uint32_t now_ms) {
    return (uint32_t)(now_ms - stamp) >= FSD_DRIVE_CTX_FRESH_MS;
}

FsdSupVerdict fsd_supervised_drive_why(const FSDState* state, uint32_t now_ms) {
    if(!state) return FSD_SUP_NO_GEAR;

    /* Checked before the driver-presence signals: OTA and a quiet bus are
     * reasons not to act even with a driver sitting right there, and reporting
     * them first gives the more actionable answer. */
    if(state->tesla_ota_in_progress) return FSD_SUP_OTA;
    if(state->rx_stale) return FSD_SUP_RX_STALE;

    if(!state->di_gear_seen) return FSD_SUP_NO_GEAR;
    if(stale(state->di_gear_ms, now_ms)) return FSD_SUP_GEAR_STALE;
    /* R counts. Reversing is driving and the driver is in the seat; whether a
     * camera response makes sense while reversing is the policy layer's
     * problem, not this gate's. P, N, INVALID and SNA do not count. */
    if(state->di_gear != FSD_GEAR_D && state->di_gear != FSD_GEAR_R)
        return FSD_SUP_GEAR_NOT_DRIVE;

    if(!state->belt_seen) return FSD_SUP_NO_BELT;
    if(stale(state->belt_seen_ms, now_ms)) return FSD_SUP_BELT_STALE;
    if(!state->ui_buckle_status) return FSD_SUP_BELT_UNLATCHED;

    return FSD_SUP_OK;
}

bool fsd_supervised_drive(const FSDState* state, uint32_t now_ms) {
    return fsd_supervised_drive_why(state, now_ms) == FSD_SUP_OK;
}

OpMode fsd_autonomy_floor(const FSDState* state) {
    if(!state || !state->autonomy_enabled) return OpMode_ListenOnly;
    return OpMode_Autonomous;
}

bool fsd_autonomy_allows(const FSDState* state, uint32_t now_ms) {
    if(!state || !state->autonomy_enabled) return false;
    if(state->op_mode != OpMode_Autonomous && state->op_mode != OpMode_Active)
        return false;
    return fsd_supervised_drive(state, now_ms);
}

const char* fsd_sup_verdict_str(FsdSupVerdict v) {
    switch(v) {
    case FSD_SUP_OK: return "supervised";
    case FSD_SUP_NO_GEAR: return "no gear frame";
    case FSD_SUP_GEAR_STALE: return "gear stale";
    case FSD_SUP_GEAR_NOT_DRIVE: return "not in D/R";
    case FSD_SUP_NO_BELT: return "no belt frame";
    case FSD_SUP_BELT_STALE: return "belt stale";
    case FSD_SUP_BELT_UNLATCHED: return "belt unlatched";
    case FSD_SUP_RX_STALE: return "bus quiet";
    case FSD_SUP_OTA: return "tesla OTA";
    default: return "unknown";
    }
}
