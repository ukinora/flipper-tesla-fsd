#include "fsd_rxstall.h"

#include <string.h>

/* Unsigned subtraction, so an interval that spans the millisecond counter's
 * 49-day wrap still comes out as the elapsed time rather than as a number
 * large enough to postpone the verdict by another 49 days. */
static bool elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t span_ms) {
    return (uint32_t)(now_ms - since_ms) >= span_ms;
}

void fsd_rxstall_reset(FsdRxStall* st) {
    if (!st) return;
    memset(st, 0, sizeof(*st));
}

FsdRxStallVerdict fsd_rxstall_sample(FsdRxStall* st, uint32_t rx_count, uint32_t now_ms) {
    if (!st) return FSD_RXSTALL_OK;

    if (rx_count > st->last_rx) {
        /* A frame arrived. That answers every question this file asks: the
         * controller is listening, so the budget is whole and any give-up is
         * lifted -- a module that sat through one night has to still guard the
         * drive after it. */
        st->last_rx = rx_count;
        st->last_change_ms = now_ms;
        st->seen_traffic = true;
        st->tries = 0u;
        st->gave_up = false;
        return FSD_RXSTALL_OK;
    }

    if (rx_count < st->last_rx) {
        /* 🔴 SMALLER IS A RESTART, NOT A FRAME. The MCP2515 driver does not
         * zero rx_count_ in begin(), so this cannot happen today -- but
         * "today" is the only thing making it safe, and a driver that started
         * zeroing it would turn every FAILED recovery into a successful-looking
         * one. Re-seed and charge nothing either way. */
        st->last_rx = rx_count;
        return FSD_RXSTALL_OK;
    }

    /* Unchanged. */
    if (!st->seen_traffic) {
        /* A bench board with nothing plugged in reads 0 forever. That is not a
         * controller that went deaf; it is one nobody has spoken to. Keep the
         * clock moving with it so the first real frame starts a fresh window. */
        st->last_change_ms = now_ms;
        return FSD_RXSTALL_OK;
    }
    if (st->gave_up) return FSD_RXSTALL_OK;
    if (!elapsed(now_ms, st->last_change_ms, FSD_RXSTALL_QUIET_MS)) return FSD_RXSTALL_OK;
    if (st->tries > 0u && !elapsed(now_ms, st->last_try_ms, FSD_RXSTALL_COOLDOWN_MS))
        return FSD_RXSTALL_OK;

    st->tries++;
    st->last_try_ms = now_ms;
    if (st->recoveries < 0xFFFFu) st->recoveries++;
    if (st->tries >= FSD_RXSTALL_MAX_TRIES) st->gave_up = true;
    return FSD_RXSTALL_RECOVER;
}
