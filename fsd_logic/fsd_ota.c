#include "fsd_ota.h"

void fsd_ota_observe_raw(FSDState* state, uint8_t raw) {
    if(!state) return;

    raw = (uint8_t)(raw & 0x03u);
    state->ota_raw_state = raw;

    if(raw == FSD_OTA_RAW_INSTALLING) {
        if(state->ota_assert_count < 255u) state->ota_assert_count++;
        state->ota_clear_count = 0u;
        if(state->ota_assert_count >= FSD_OTA_ASSERT_FRAMES)
            state->tesla_ota_in_progress = true;
    } else {
        if(state->ota_clear_count < 255u) state->ota_clear_count++;
        state->ota_assert_count = 0u;
        if(state->ota_clear_count >= FSD_OTA_CLEAR_FRAMES)
            state->tesla_ota_in_progress = false;
    }
}
