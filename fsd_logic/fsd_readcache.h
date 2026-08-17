#pragma once
/*
 * fsd_readcache.h — block bookkeeping for the capture read path.
 *
 * Why this exists (measured on the bench, 2026-08-17, lilygo-t2can + LittleFS):
 * one 500-byte blackbox_read_chunk() costs ~17.6 ms, split
 *
 *     open 10.6 ms | seek 6.5 ms | read 0.02 ms | close 0.1 ms
 *
 * The read is free; the file handling is everything. That caps the capture
 * download at ~27 KB/s no matter how fast the radio is, and a 1.2 MB capture is
 * the thing the operator is standing next to the car waiting for.
 *
 * 🔴 The standing explanation was "seek gets slower as the offset grows, because
 * LittleFS walks a CTZ skip list". That was never measured, and it is wrong:
 * seek costs ~6.5 ms at offset 200 KB and ~6.4 ms at 600 KB. A fix built on the
 * guess (hold the File open across chunks) broke downloads outright and was
 * reverted. Hence: read a whole block per open and serve chunks out of RAM. No
 * file handle outlives the call, so nothing can go stale while it is held.
 *
 * This header is only the arithmetic — which bytes a request may take from the
 * block that is currently buffered. It is separated out because getting it
 * wrong does not fail loudly: it returns plausible bytes from the wrong offset,
 * and a corrupt capture is discovered after the TSL is out of the car.
 *
 * Header-only + dependency-free so the host test exercises the same code the
 * firmware runs.
 */

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* Capture names are like "evt_553257_manual" (17 chars); the BLE layer already
 * caps them at 40 (g_bulk_name). A name that does not fit cannot be told apart
 * from another one sharing its first 39 characters, so it is refused rather
 * than truncated — the cost of refusing is "no speed-up", the cost of
 * truncating is serving one capture's bytes for another's. */
#define FSD_RC_NAME_MAX 40

typedef struct {
    char   name[FSD_RC_NAME_MAX];
    bool   json;    /* .json vs .log — same offsets, different file */
    bool   valid;
    size_t off;     /* file offset of the block's first byte */
    size_t len;     /* bytes held in the block */
} FsdReadCache;

/* Drop the block. Call whenever the underlying file may have changed or gone:
 * download finished or aborted, capture deleted, filesystem remounted. */
static inline void fsd_rc_reset(FsdReadCache* c) {
    if (c) memset(c, 0, sizeof(*c));
}

/* Record that `len` bytes starting at file offset `off` are now buffered.
 * A zero-length or unnameable block is no block at all — it resets instead, so
 * a caller that fills from a short read cannot leave a hit behind. */
static inline void fsd_rc_fill(FsdReadCache* c, const char* name, bool json,
                               size_t off, size_t len) {
    if (!c) return;
    if (!name || len == 0 || strlen(name) >= FSD_RC_NAME_MAX) {
        fsd_rc_reset(c);
        return;
    }
    memset(c, 0, sizeof(*c));
    memcpy(c->name, name, strlen(name));
    c->json  = json;
    c->off   = off;
    c->len   = len;
    c->valid = true;
}

/* How many bytes of [off, off+cap) the buffered block can satisfy, starting at
 * *index within it. Returns 0 on any miss — wrong file, wrong half of the pair,
 * offset outside the block, or nothing buffered.
 *
 * Never serves partially from the wrong place: a request that starts before the
 * block is a miss, not a rewind, and one that runs past the end is clamped to
 * what is actually held. The caller refills and asks again. */
static inline size_t fsd_rc_serve(const FsdReadCache* c, const char* name,
                                  bool json, size_t off, size_t cap,
                                  size_t* index) {
    if (!c || !c->valid || c->len == 0) return 0;
    if (!name || cap == 0) return 0;
    if (c->json != json) return 0;
    if (strlen(name) >= FSD_RC_NAME_MAX) return 0;
    if (strcmp(c->name, name) != 0) return 0;
    if (off < c->off) return 0;
    size_t rel = off - c->off;
    if (rel >= c->len) return 0;
    size_t avail = c->len - rel;
    if (index) *index = rel;
    return avail < cap ? avail : cap;
}
