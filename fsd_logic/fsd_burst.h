#pragma once

/*
 * fsd_burst.h — WHEN a decided frame goes out.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * 🔴 A FAILURE IN THE CAR, 2026-09-08. The blinker worked and the mirror did
 * not, from the same board, the same session, the same permission state and
 * the same press. The capture said why: 0x273 arrived twenty times in that
 * window and every one of them was the car's. Not one of ours reached the bus.
 *
 * The scheduler shipped the FIRST frame on the TRIGGER'S clock — the instant
 * the switch was pressed — and only the repeats on the car's. For 0x249 that
 * is harmless: the id runs at 50 ms, so the reference the emitter copies is
 * never more than 50 ms old and the 200 ms freshness window always covers it.
 * For 0x273 at 500 ms it almost never does, and the mirror is reps=1, so that
 * one badly-timed frame WAS the whole command.
 *
 * 🔴 THE BLINKER WAS NOT A SUCCESS. IT WAS LUCK. Both actions carried the same
 * defect; one of them happened to run on a bus fast enough to hide it. Reading
 * that day as "the blinker works, the mirror needs investigating" would have
 * been reading a coincidence as a result.
 *
 * WHY WAITING IS NOT A TUNING CHOICE
 * ----------------------------------
 * The obvious-looking fix — widen the freshness window — is wrong, and wrong
 * in a way worth writing down because it will occur to the next person too.
 * The emitter does not compose a frame from scratch: it COPIES the car's most
 * recent frame of that id and changes a few bits. A frame sent between
 * arrivals is therefore built from a template up to a full period old, and it
 * lands at an arbitrary offset rather than the 1 ms behind the car that TSL
 * uses. Widening the window lets that frame out; it does not make it right.
 *
 * So every frame waits for the car's. There was never a version of "send now"
 * that could have worked.
 *
 * WHAT IT COSTS
 * -------------
 * Latency, bounded by one frame period: ~50 ms for the blinker, up to ~500 ms
 * for anything on 0x273. That is the trade, made on purpose.
 *
 * WHY IT IS HERE AND NOT IN rule_task.cpp
 * ---------------------------------------
 * Because that file is Arduino and cannot be compiled on a host, so the
 * scheduling had no tests at all — the tenth pattern, and the reason
 * fsd_pipeline.c exists in this directory rather than next to its caller.
 * This file owns clocks and counters and knows nothing about a bus.
 */

#include "fsd_body.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** How many decisions can be waiting for the car at once.
 *
 *  One trigger event yields up to FSD_PIPE_MAX_OUT decisions, and each needs
 *  its own slot now that none of them is sent immediately. Matching that
 *  number is not a coincidence: a smaller table would silently drop part of a
 *  single press. */
#define FSD_BURST_MAX 4u

/** How long a burst waits for its id before giving up.
 *
 *  🔴 WAITING FOR THE CAR MEANS THE CAR MIGHT NOT SPEAK. Without a deadline a
 *  burst outlives the press that made it: the operator gives up, walks away,
 *  and the frame goes out whenever the bus comes back — a command with nobody
 *  behind it.
 *
 *  One second, chosen the same way FSD_SPEED_LIMIT_MAX_AGE_MS was: long
 *  compared with every id we drive (the slowest, 0x273, is 500 ms) so it fires
 *  on a bus that has actually gone quiet rather than on jitter, and short
 *  enough that a command cannot arrive long after the person who asked for it
 *  has stopped expecting it.
 *
 *  ⚠️ The window restarts on every arrival — see fsd_burst_on_frame(). The
 *  question it answers is "has this id gone quiet", which does not depend on
 *  when the button was pressed. */
#define FSD_BURST_MAX_WAIT_MS 1000u

/** One decision, waiting for its id. */
typedef struct {
    FsdBodyAction action;
    int32_t arg;
    uint8_t rule_index;
    uint32_t id;          /* the CAN id whose arrival drives it */
    uint8_t remaining;    /* frames still owed; 0 means the slot is free */
    uint32_t deadline_ms; /* give up at this point if the id stays quiet */
    uint16_t seq;         /* arm order, so a tie on one id is resolved fairly */
} FsdBurstSlot;

/** What one arrival owes. A copy, so the caller cannot edit the table. */
typedef struct {
    FsdBodyAction action;
    int32_t arg;
    uint8_t rule_index;
} FsdBurstDue;

/** The table. Session state: it dies with the power, like the arm flag. */
typedef struct {
    FsdBurstSlot slot[FSD_BURST_MAX];
    /* When each action last STARTED a command, for min_interval_ms. 0 = never.
     *
     * 🔴 NOT IN THE SLOT. A slot is freed and reused, so a stamp living there
     * would vanish the moment the burst finished -- and then the interval
     * would only ever gate commands that OVERLAP, which is the one case it
     * does not need to cover. */
    uint32_t last_act_ms[FSD_ACT_COUNT];
    uint16_t next_seq;
    uint16_t expired; /* bursts abandoned because the id never came back */
    uint16_t dropped; /* arms refused because every slot was busy */
} FsdBurst;

/** Clear everything, counters included.
 *
 *  Also the "stop" path: locking transmission mid-burst has to drop the table
 *  here rather than rely on a gate further down refusing the rest. Same
 *  argument as the pending release. */
void fsd_burst_reset(FsdBurst* b);

/** Register a decision and wait for `can_id`.
 *
 *  Returns false if `reps` is 0 (a caller confused: fsd_emit_repeat() never
 *  returns less than 1) or if every slot is busy — in which case `dropped`
 *  goes up. 🔴 It REFUSES rather than evicting: silently dropping the oldest
 *  would make a command vanish with nothing said. */
bool fsd_burst_arm(FsdBurst* b, FsdBodyAction action, int32_t arg, uint8_t rule_index,
                   uint32_t can_id, uint8_t reps, uint32_t now_ms);

/** The car's frame of `can_id` just landed. Fills `out` and returns true if a
 *  frame is owed on it.
 *
 *  🔴 ONE FRAME PER ARRIVAL, oldest first. Emitting two of ours behind a
 *  single reference would put them in the same millisecond slot, and the
 *  second would be built from a template the first has already contradicted.
 *  Two rules on one id take turns.
 *
 *  Every arrival of a live id also restarts that slot's deadline. */
bool fsd_burst_on_frame(FsdBurst* b, uint32_t can_id, uint32_t now_ms, FsdBurstDue* out);

/** Time passed. Drop anything whose id has gone quiet; returns how many. */
uint8_t fsd_burst_tick(FsdBurst* b, uint32_t now_ms);

/** How many decisions are still waiting. */
uint8_t fsd_burst_pending(const FsdBurst* b);

/** Fill `out` (FSD_ACT_COUNT entries) with when each action last STARTED a
 *  command, for FsdBodyInputs.last_act_ms.
 *
 *  🔴 min_interval_ms WAS DECORATIVE UNTIL 2026-09-08: nothing in the firmware
 *  ever wrote that array, so `now - 0` was always enormous and every row's
 *  interval passed. Four gates advertised, three enforced.
 *
 *  🔴 IT COUNTS COMMANDS, NOT FRAMES, and that is what kept it switched off.
 *  The indicator sends four frames about 50 ms apart and its own row allows
 *  50 ms, so stamping every frame would make a burst refuse itself on jitter
 *  alone. The table has always meant commands -- the door's 3000 is "do not
 *  open it twice in three seconds" -- so the stamp goes down once, when a
 *  command is ACCEPTED, and the frames it owes are exempt.
 *
 *  `emitting` is the action whose frame is going out right now; its entry
 *  comes back 0 ("never"), because that frame belongs to a command which
 *  already answered this question at the press. Pass FSD_ACT_COUNT on the
 *  press path, where nothing is exempt.
 *
 *  A REFUSED arm leaves no stamp: a command that never happened must not lock
 *  out the next real one. */
void fsd_burst_fill_last_act(const FsdBurst* b, uint32_t* out, unsigned emitting);

#ifdef __cplusplus
}
#endif
