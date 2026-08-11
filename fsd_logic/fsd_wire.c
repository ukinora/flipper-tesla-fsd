/*
 * fsd_wire.c — see fsd_wire.h. Pure: no platform, no globals, no clock.
 */

#include "fsd_wire.h"

#include <string.h>

static void put_le16(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_le32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t sat16(uint32_t v) {
    return (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v;
}

static uint8_t sat8(uint32_t v) {
    return (v > 0xFFu) ? 0xFFu : (uint8_t)v;
}

void fsd_wire_pack_state(const FsdWireState* in, uint8_t* out) {
    if(!in || !out) return;

    uint8_t flags = 0;
    if(in->rx_seen) flags |= (1u << 0);
    if(in->ota_in_progress) flags |= (1u << 1);
    if(in->blinker_left) flags |= (1u << 2);
    if(in->blinker_right) flags |= (1u << 3);
    /* Bits 4 and 5 (blind spot L/R) are deliberately never set:
     * DAS_sideCollisionWarning is not extracted on the ESP32 path, and its bit
     * position is confirmed only for 0x39B (HW4). Left zero rather than guessed.
     * Bit 7 (profile change in progress) belongs to the SET_PROFILE closed
     * loop, which emits nothing while both of its gates are shut. */
    if(in->brake_applied) flags |= (1u << 6);

    int32_t prof = in->speed_profile;
    if(prof < 0) prof = 0;
    if(prof > FSD_WIRE_PROFILE_MAX) prof = FSD_WIRE_PROFILE_MAX;

    float soc = in->soc_percent;
    if(soc < 0.0f) soc = 0.0f;
    if(soc > 100.0f) soc = 100.0f;

    float kph = in->speed_kph;
    if(kph < 0.0f) kph = 0.0f;
    const uint16_t kph10 = sat16((uint32_t)(kph * 10.0f + 0.5f));

    /* Zero when unseen rather than a stale last value: the app shows a blank
     * limit, not a wrong one. */
    const uint16_t lim =
        in->speed_limit_seen ? sat16((uint32_t)(in->speed_limit_kph + 0.5f)) : 0u;

    memset(out, 0, FSD_WIRE_STATE_LEN);
    out[0] = (uint8_t)FSD_WIRE_STATE_VERSION;
    out[1] = flags;
    out[2] = in->op_mode;
    out[3] = in->hw_version;
    out[4] = (uint8_t)prof;
    out[5] = in->ap_state;
    put_le16(&out[6], kph10);
    out[8] = (uint8_t)(soc + 0.5f);
    /* Gear, as originally specified. This byte carried the cruise state as a
     * placeholder while the ESP32 had no PRND parser; fsd_autonomy.c provides
     * one now, so it went back and the version went to 2. */
    out[9] = in->gear;
    put_le16(&out[10], lim);
    put_le16(&out[12], in->rx_fps);
    put_le16(&out[14], sat16(in->crc_err_count));
    put_le32(&out[16], in->uptime_s);
}

void fsd_wire_pack_camstat(const FsdWireCamStat* in, uint8_t* out) {
    if(!in || !out) return;

    uint8_t flags = 0;
    if(in->autonomy_enabled) flags |= (1u << 0);
    if(in->supervised_ok) flags |= (1u << 1);
    if(in->db_loaded) flags |= (1u << 2);
    if(in->autonomy_allows) flags |= (1u << 3);
    if(in->pol_suspended) flags |= (1u << 4);
    if(in->learning_dirty) flags |= (1u << 5);
    if(in->profile_fresh) flags |= (1u << 6);
    if(in->save_failing) flags |= (1u << 7);

    memset(out, 0, FSD_WIRE_CAMSTAT_LEN);
    out[0] = (uint8_t)FSD_WIRE_CAMSTAT_VERSION;
    out[1] = flags;
    out[2] = in->sup_verdict;
    out[3] = in->op_mode;
    put_le32(&out[4], in->camera_count);
    put_le32(&out[8], in->built_at);

    /* Bytes 0..11 above keep their exact v1 meaning, which is what lets a
     * version-ignoring 12-byte reader stay correct. Everything below is new in
     * v2 and a length-strict v1 client will break on it — see 부록 A-2. */
    out[12] = in->gps_verdict;
    out[13] = (uint8_t)((in->pol_phase & 0x07u) | ((in->pol_action & 0x03u) << 3) |
                        ((in->pol_target & 0x03u) << 5));
    put_le16(&out[14], in->nearest_m);
    out[16] = in->gps_accuracy_raw;
    /* RAW, before camera_task's range check: a bring-up drive has to be able to
     * tell a mis-decode ("it says 5") from an absent frame ("it says 0xFF"). */
    out[17] = in->raw_profile;
    out[18] = sat8(in->learned_count);
    out[19] = sat8(in->scan_full_count);
}

void fsd_wire_pack_result(uint8_t cmd, uint8_t res, uint16_t extra, uint8_t* out) {
    if(!out) return;
    out[0] = cmd;
    out[1] = res;
    put_le16(&out[2], extra);
}
