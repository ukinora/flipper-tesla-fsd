#pragma once
/*
 * fsd_bb_save.h -- save a black-box capture a slice at a time (2026-10-14).
 *
 * 🔴 WHY: blackbox.cpp used to write the whole window to LittleFS in one call.
 * The loop that does that is the loop that drains CAN, so for the length of the
 * save (seconds on a busy bus) nothing was received: one save cost 22,947
 * frames on can0 in a bench replay, and a rule switch pressed during a save was
 * only classified after it -- late, and as a SHORT press (red-team 2026-10-04,
 * item 2-5).
 *
 * Now the save writes at most `max_lines` frames per loop pass and returns, so
 * CAN is drained between slices. The ring keeps recording meanwhile, which
 * opens one hazard this header exists to close: with the ring full, a new frame
 * would evict the oldest one -- and the oldest one may be a frame the save has
 * not written yet. So the next unwritten frame is protected: the NEW frame is
 * dropped from the ring instead (CAN processing is unaffected, only the
 * recorder misses it). Frames already written may be evicted as usual.
 *
 * Idea from hypery11 v2.17-beta.1 (#245, blackbox_flush.h); rewritten for this
 * tree's ring and kept Arduino-free so test/test_fsd_core.c runs it.
 *
 * The ring is `cap` slots, oldest at `tail`, next write at `head`, holding at
 * most cap-1 frames (head == tail is empty), frames in arrival order.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t fsd_bb_next(uint32_t cap, uint32_t i) {
    return (i + 1u) % cap;
}

/**
 * Claim the slot for a new frame. Evicts the oldest frame when the ring is
 * full -- unless that oldest frame is `keep`, the next frame a running save
 * still has to write. Then nothing is evicted, the new frame is dropped, and
 * this returns false. Pass keep = cap when no save is running.
 */
static inline bool fsd_bb_push(uint32_t cap, uint32_t* head, uint32_t* tail,
                               uint32_t keep, uint32_t* slot) {
    if(fsd_bb_next(cap, *head) == *tail) {
        if(*tail == keep) return false;
        *tail = fsd_bb_next(cap, *tail);
    }
    *slot = *head;
    *head = fsd_bb_next(cap, *head);
    return true;
}

typedef struct {
    bool active;
    uint32_t cursor;  /* next ring index to look at */
    uint32_t lo, hi;  /* window, inclusive, in the ring's ms clock */
    uint32_t written; /* frames emitted so far */
} FsdBbSave;

/** The slot fsd_bb_push() must not evict: the save cursor, or cap if idle. */
static inline uint32_t fsd_bb_keep(const FsdBbSave* s, uint32_t cap) {
    return (s && s->active) ? s->cursor : cap;
}

static inline void fsd_bb_save_begin(FsdBbSave* s, uint32_t tail, uint32_t lo, uint32_t hi) {
    s->active = true;
    s->cursor = tail;
    s->lo = lo;
    s->hi = hi;
    s->written = 0u;
}

typedef uint32_t (*FsdBbTsAt)(void* ctx, uint32_t idx);
typedef void (*FsdBbEmit)(void* ctx, uint32_t idx);

/**
 * Look at no more than `max_lines` ring entries (skipped ones count too, so a
 * step's work is bounded however the window sits in the ring), emitting the
 * ones inside [lo, hi]. Returns true when the window is complete -- the cursor
 * reached the newest frame or a frame past `hi` -- and clears `active`.
 *
 * The same walk as the old one-shot emit: frames before `lo` are skipped, the
 * first frame after `hi` ends it.
 */
static inline bool fsd_bb_save_step(FsdBbSave* s, uint32_t cap, uint32_t head,
                                    uint32_t max_lines, FsdBbTsAt ts_at, FsdBbEmit emit,
                                    void* ctx) {
    if(!s || !s->active) return true;
    for(uint32_t n = 0; n < max_lines; n++) {
        if(s->cursor == head) { s->active = false; return true; }
        const uint32_t ts = ts_at(ctx, s->cursor);
        if(ts > s->hi) { s->active = false; return true; }
        if(ts >= s->lo) { emit(ctx, s->cursor); s->written++; }
        s->cursor = fsd_bb_next(cap, s->cursor);
    }
    return false;
}

#ifdef __cplusplus
}
#endif
