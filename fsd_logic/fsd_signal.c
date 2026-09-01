/*
 * fsd_signal.c — see fsd_signal.h. A table and one extractor.
 */

#include "fsd_signal.h"

/* Frames, so the bit positions below read against something.
 *
 *   0x3E2 VCLEFT_lightStatus   7 bytes, 5 Hz
 *   0x3C2 VCLEFT_switchStatus  8 bytes, 20 Hz, multiplexed on byte 0
 *   0x102 VCLEFT_doorStatus    8 bytes, 10 Hz
 *   0x103 VCRIGHT_doorStatus   8 bytes, 10 Hz
 *   0x118 DI_systemStatus      8 bytes, 100 Hz
 */
#define ID_LIGHT 0x3E2u
#define ID_SWITCH 0x3C2u
#define ID_DOOR_L 0x102u
#define ID_DOOR_R 0x103u
#define ID_DRIVE 0x118u

/* The two multiplexes of 0x3C2, both verbatim from the capture:
 *   0x00 ... switch pack (windows, belt, occupancy)
 *   0x29 ... scroll wheel and the camera bit
 * Byte 0 also carries other flags, so the mux comparison is masked. */
#define MUX_PACK 0x00u
#define MUX_SCROLL 0x29u
#define MUX_MASK 0x2Fu

#define SW(n, nm, bit)                                                     \
    [n] = {.signal = (n), .name = (nm), .can_id = ID_LIGHT,                \
           .mux_byte = FSD_SIG_NO_MUX, .start_bit = (bit), .bit_len = 1,   \
           .kind = FSD_SIGK_SWITCH}

#define LAMP(n, nm, bit)                                                   \
    [n] = {.signal = (n), .name = (nm), .can_id = ID_LIGHT,                \
           .mux_byte = FSD_SIG_NO_MUX, .start_bit = (bit), .bit_len = 2,   \
           .kind = FSD_SIGK_STATE}

#define WIN(n, nm, bit)                                                    \
    [n] = {.signal = (n), .name = (nm), .can_id = ID_SWITCH,               \
           .mux_byte = 0, .mux_mask = MUX_MASK, .mux_value = MUX_PACK,     \
           .start_bit = (bit), .bit_len = 2, .kind = FSD_SIGK_SWITCH}

static const FsdSignalDef FSD_SIGNALS[] = {
    /* 0x3E2 byte1 bits 6-7 and byte2 bits 0-1. The owner pressed front-left,
     * front-right, rear-left, rear-right one second apart, and the capture
     * shows exactly one of these rising per press, in that order. */
    SW(FSD_SIG_MAP_SW_FL, "map switch FL", 14),
    SW(FSD_SIG_MAP_SW_FR, "map switch FR", 15),
    SW(FSD_SIG_MAP_SW_RL, "map switch RL", 16),
    SW(FSD_SIG_MAP_SW_RR, "map switch RR", 17),

    /* Same walk, the other half of the frame: the lamp that stays on after the
     * finger leaves. 6/8/10/12 is also what the borrowed DBC predicted — one of
     * four independent agreements that day, and the reason we now trust the
     * rest of that file enough to check it rather than ignore it. */
    LAMP(FSD_SIG_MAP_ON_FL, "map light FL", 6),
    LAMP(FSD_SIG_MAP_ON_FR, "map light FR", 8),
    LAMP(FSD_SIG_MAP_ON_RL, "map light RL", 10),
    LAMP(FSD_SIG_MAP_ON_RR, "map light RR", 12),

    /* 0x3C2 mux 0, bytes 4 and 5: left pair then right pair, up before down.
     * Walked in the same one-per-second order, both directions. */
    WIN(FSD_SIG_WIN_UP_FL, "window up FL", 32),
    WIN(FSD_SIG_WIN_DN_FL, "window down FL", 34),
    WIN(FSD_SIG_WIN_UP_RL, "window up RL", 36),
    WIN(FSD_SIG_WIN_DN_RL, "window down RL", 38),
    WIN(FSD_SIG_WIN_UP_FR, "window up FR", 40),
    WIN(FSD_SIG_WIN_DN_FR, "window down FR", 42),
    WIN(FSD_SIG_WIN_UP_RR, "window up RR", 44),
    WIN(FSD_SIG_WIN_DN_RR, "window down RR", 46),

    /* Both seen changing in the gear capture: the belt latched at t=6.362 and
     * TSL asked for D 78 ms later. */
    [FSD_SIG_BELT_FRONT] = {.signal = FSD_SIG_BELT_FRONT,
                            .name = "front belt",
                            .can_id = ID_SWITCH,
                            .mux_byte = 0,
                            .mux_mask = MUX_MASK,
                            .mux_value = MUX_PACK,
                            .start_bit = 48,
                            .bit_len = 2,
                            .kind = FSD_SIGK_STATE},
    [FSD_SIG_DRIVER_PRESENT] = {.signal = FSD_SIG_DRIVER_PRESENT,
                                .name = "driver present",
                                .can_id = ID_SWITCH,
                                .mux_byte = 0,
                                .mux_mask = MUX_MASK,
                                .mux_value = MUX_PACK,
                                .start_bit = 4,
                                .bit_len = 1,
                                .kind = FSD_SIGK_STATE},

    /* 2 = shut, 1 = open. Both values observed, in both directions, on the
     * driver's door opened by hand and the right rear opened by TSL. */
    [FSD_SIG_DOOR_FL_LATCH] = {.signal = FSD_SIG_DOOR_FL_LATCH,
                               .name = "driver door latch",
                               .can_id = ID_DOOR_L,
                               .mux_byte = FSD_SIG_NO_MUX,
                               .start_bit = 0,
                               .bit_len = 2,
                               .kind = FSD_SIGK_STATE},
    [FSD_SIG_DOOR_RR_LATCH] = {.signal = FSD_SIG_DOOR_RR_LATCH,
                               .name = "right rear door latch",
                               .can_id = ID_DOOR_R,
                               .mux_byte = FSD_SIG_NO_MUX,
                               .start_bit = 4,
                               .bit_len = 2,
                               .kind = FSD_SIGK_STATE},

    /* Watched go 1 -> 4 fourteen milliseconds after TSL's stalk frame. */
    [FSD_SIG_GEAR] = {.signal = FSD_SIG_GEAR,
                      .name = "gear",
                      .can_id = ID_DRIVE,
                      .mux_byte = FSD_SIG_NO_MUX,
                      .start_bit = 21,
                      .bit_len = 3,
                      .kind = FSD_SIGK_STATE},

    /* 🔴 The only signed field here, and the only accumulating one: a fast roll
     * put +8 and -9 in a single frame. Treating it as a boolean "moved" would
     * throw away exactly the part that matters. */
    [FSD_SIG_SCROLL_TICKS] = {.signal = FSD_SIG_SCROLL_TICKS,
                              .name = "scroll detents",
                              .can_id = ID_SWITCH,
                              .mux_byte = 0,
                              .mux_mask = MUX_MASK,
                              .mux_value = MUX_SCROLL,
                              .start_bit = 24,
                              .bit_len = 6,
                              .kind = FSD_SIGK_DELTA},
};

_Static_assert(sizeof(FSD_SIGNALS) / sizeof(FSD_SIGNALS[0]) == FSD_SIG_COUNT,
               "every FsdSignal needs a row");

const FsdSignalDef* fsd_signal_def(FsdSignal s) {
    if(s >= FSD_SIG_COUNT) return NULL;
    const FsdSignalDef* d = &FSD_SIGNALS[s];
    /* Catches a transposed row, which the length assertion cannot. */
    if(d->signal != s) return NULL;
    if(d->bit_len == 0u) return NULL;
    return d;
}

FsdSignalVerdict fsd_signal_extract(FsdSignal s, uint32_t can_id, const uint8_t* data,
                                    uint8_t dlc, int32_t* out) {
    if(!data || !out) return FSD_SIGV_BAD_ARGS;

    const FsdSignalDef* d = fsd_signal_def(s);
    if(!d) return FSD_SIGV_UNKNOWN;
    if(can_id != d->can_id) return FSD_SIGV_WRONG_ID;

    if(d->mux_byte != FSD_SIG_NO_MUX) {
        if(d->mux_byte >= dlc) return FSD_SIGV_SHORT;
        if((uint8_t)(data[d->mux_byte] & d->mux_mask) != d->mux_value) {
            return FSD_SIGV_WRONG_MUX;
        }
    }

    const uint16_t last_bit = (uint16_t)d->start_bit + d->bit_len - 1u;
    if(last_bit / 8u >= dlc) return FSD_SIGV_SHORT;

    uint32_t v = 0;
    for(uint8_t i = 0; i < d->bit_len; i++) {
        const uint16_t b = (uint16_t)d->start_bit + i;
        if(data[b / 8u] & (uint8_t)(1u << (b % 8u))) v |= (uint32_t)1u << i;
    }

    if(d->kind == FSD_SIGK_DELTA) {
        /* Sign-extend from bit_len. Written as a shift pair rather than a
         * conditional subtract because the field width is a table value, not a
         * constant, and this form cannot disagree with it. */
        const uint8_t shift = (uint8_t)(32u - d->bit_len);
        *out = (int32_t)((int32_t)(v << shift) >> shift);
    } else {
        *out = (int32_t)v;
    }
    return FSD_SIGV_OK;
}

const char* fsd_signal_verdict_str(FsdSignalVerdict v) {
    switch(v) {
    case FSD_SIGV_OK: return "ok";
    case FSD_SIGV_UNKNOWN: return "unknown signal";
    case FSD_SIGV_WRONG_ID: return "wrong CAN id";
    case FSD_SIGV_WRONG_MUX: return "wrong multiplex";
    case FSD_SIGV_SHORT: return "frame too short";
    case FSD_SIGV_BAD_ARGS: return "bad args";
    }
    return "?";
}
