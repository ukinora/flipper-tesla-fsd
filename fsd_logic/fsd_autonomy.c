/*
 * fsd_autonomy.c — see fsd_autonomy.h.
 */

#include "fsd_autonomy.h"

// CAN_ID_* live here. Included rather than redefined: the bug this file was
// shipped with was a frame identity error, and a second copy of the numbers is
// how those happen.
#include "fsd_handler.h"

/* DI_gear — 0x118 DI_systemStatus, NOT 0x286 DI_state.
 *
 *   opendbc tesla_model3_party.dbc:82, inside BO_ 280 DI_systemStatus:
 *     SG_ DI_gear : 21|3@1+ (1,0)
 *   little-endian, start bit 21, 3 bits wide -> byte 2, bits [7:5].
 *
 * 🔴 THIS WAS WRONG WHEN SHIPPED AND THE BIT MATH IS NOT WHAT WAS WRONG.
 * The original comment cited this exact DBC line and then attributed it to
 * 0x286, which does not carry DI_gear at all (BO_ 646 DI_state, party.dbc:
 * 223-236). On 0x286 those bits are the top three of
 * DI_digitalSpeed : 15|9@1+ (0.5,0), so the gate computed
 *
 *     di_gear = (speed_raw >> 6) & 7
 *
 * which reports D only while the car is doing 128-159 km/h and P at 32-63.
 * fsd_supervised_drive() therefore refused at every ordinary speed: the
 * camera feature could not have run on a real car. It failed CLOSED, which is
 * why nothing dangerous happened and also why nothing revealed it.
 *
 * The tests did not catch it because they built a frame by hand, set
 * f.canId = CAN_ID_DI_STATE, and asserted the bit extraction — which was
 * always correct. Nothing asserted the observer was looking at the right
 * frame. Hence the ID checks below. */
#define GEAR_BYTE 2u
#define GEAR_SHIFT 5u
#define GEAR_MASK 0x07u

/* DI_cruiseState — 0x286 DI_state, party.dbc:234: SG_ DI_cruiseState : 12|3@1+
 * little-endian, start bit 12 -> byte 1, bits [6:4]. This half was right; it
 * just lived in a function reading a different frame. */
#define CRUISE_BYTE 1u
#define CRUISE_SHIFT 4u
#define CRUISE_MASK 0x07u

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
void fsd_drive_observe_speed(FSDState* state, const CANFRAME* frame, uint32_t now_ms) {
    if(!state || !frame) return;
    /* Same guard as the gear observer, for the same reason: this is an input to
     * a safety gate (fsd_profile.c refuses to act above 0.5 km/h) and the
     * dispatcher has been wrong before. */
    if(frame->id != CAN_ID_DI_SPEED) return;

    float kph;
    if(fsd_decode_di_speed_kph(frame->buffer, frame->data_lenght, &kph)) {
        state->vehicle_speed_kph = kph;
        state->speed_seen = true;
        state->last_speed_tick_ms = now_ms;
    }

    /* Separate decode, separate success: DI_uiSpeed needs one more byte than
     * DI_vehicleSpeed, so a short frame can legitimately yield one and not the
     * other. Writing ui_speed from a frame that did not carry it would put a
     * stale or zero speed on the display we are building this for. */
    uint8_t ui;
    if(fsd_decode_ui_speed(frame->buffer, frame->data_lenght, &ui)) {
        state->ui_speed = ui;
        state->ui_speed_seen = true;
    }
}

void fsd_drive_observe_gear(FSDState* state, const CANFRAME* frame, uint32_t now_ms) {
    if(!state || !frame) return;
    /* The check that would have prevented this function's original defect. A
     * frame we were handed by mistake must produce nothing, not a gear derived
     * from whatever happens to sit in byte 2. This is a safety-gate input; the
     * codebase's usual "trust the dispatcher" convention is not good enough
     * here, and the dispatcher was in fact wrong. */
    if(frame->id != CAN_ID_DI_SYS_STATUS) return;
    if(frame->data_lenght <= GEAR_BYTE) return;

    state->di_gear = (uint8_t)((frame->buffer[GEAR_BYTE] >> GEAR_SHIFT) & GEAR_MASK);
    state->di_gear_ms = now_ms;
    state->di_gear_seen = true;
}

void fsd_drive_observe_cruise(FSDState* state, const CANFRAME* frame) {
    if(!state || !frame) return;
    if(frame->id != CAN_ID_DI_STATE) return;
    if(frame->data_lenght <= CRUISE_BYTE) return;

    /* Not needed by the gate. It is here because on the ESP32 this is the ONLY
     * parser for 0x286 — that build does not compile fsd_logic/fsd_handler.c —
     * so di_cruise_state has been going out over BLE as a structural zero.
     * A field that is always zero for a reason nobody can see costs more to
     * debug later than it costs to fill now. */
    state->di_cruise_state = (frame->buffer[CRUISE_BYTE] >> CRUISE_SHIFT) & CRUISE_MASK;
}

void fsd_drive_observe_belt(FSDState* state, const CANFRAME* frame, uint32_t now_ms) {
    if(!state || !frame) return;
    if(frame->id != CAN_ID_UI_WARNING) return; // same reason as the gear observer
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
