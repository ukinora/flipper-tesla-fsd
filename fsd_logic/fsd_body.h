#pragma once
/*
 * fsd_body.h — the authority axis for body control, and nothing else.
 *
 * WHY A THIRD AXIS
 * ----------------
 * The safety design stood on one sentence: the only thing we ever write is a
 * 0x3C2 scroll detent. Body control is a write, so that sentence has to be
 * extended rather than quietly stretched. This header is the extension.
 *
 * 🔴 THE ROWS ARE ACTIONS, NOT FEATURES (rewritten 2026-09-01)
 * ------------------------------------------------------------
 * They used to be T1 ("rear door -> interior light") and T2 ("double window-up
 * -> passenger door") -- a trigger and an action welded together. The owner
 * wants a rule engine: arbitrary trigger -> arbitrary action, overlapping. Once
 * triggers and actions are M:N, a row like T1 cannot exist.
 *
 * Risk does not live on the trigger. It lives on WHAT WE DO TO THE CAR. Index
 * the table by action and a hundred rules still check the same eight rows.
 *
 *      rule engine     trigger -> action      (owner builds these in the app)
 *          |
 *      THIS AXIS       may this action happen right now
 *          |
 *      emitter         builds the frame
 *          |
 *      TX chokepoint   the last denial
 *
 * WHAT IT IS NOT
 * --------------
 * It is not permission to transmit. fsd_can_transmit() still has to say yes
 * first; this then says yes again, with more conditions. Its OpMode allow-set is
 * exactly {Active, Service} -- the same set fsd_can_transmit() already admits --
 * so the axis is structurally incapable of widening anything. It can only
 * subtract. No OpMode value is added, and neither copy of fsd_can_transmit()
 * changes.
 *
 * THE EMITTER EXISTS FOR FOUR ACTIONS (2026-09-06)
 * ------------------------------------------------
 * 🔴 This section said "THERE IS STILL NO EMITTER ANYWHERE" until the map
 * light command was measured. Four now exist in fsd_body_emit.c — map light,
 * door, hazards, turn signal — and it refuses every other action.
 *
 * 🟢 NOTHING TRANSMITS. The emitter returns bytes; no caller puts them on a
 * bus. The first real write is a decision to be made in the car, with something
 * reversible, and it is not made in code.
 *
 * 🔴🔴 THE ARMING FLAG IS GONE (owner's instruction, 2026-09-10). This
 * section used to explain armable_at_runtime -- which rows may be switched on
 * at all, and what evidence flipped each bool. There is no such flag any more,
 * and no fsd_body_allows() to read it: see the box on FsdBodyCaps below.
 *
 * ⚠️ WHAT THE OLD TEXT GOT RIGHT AND IS WORTH KEEPING: a hand-written number
 * in a comment has no test behind it. That sentence used to count rows and the
 * count was wrong the day it was written. Say "every other", never a number.
 *
 * 🟢 One pairing survived the removal and still matters: an action that can be
 * chosen in a rule must have an emitter, or the press produces nothing and the
 * screen has no way to say why. test_body_emit.c still asserts that.
 *
 * See 권한축-재설계.md and 페일세이프-정책.md.
 */

#include "fsd_types.h"

#include <stdbool.h>
#include <stddef.h> // NULL — fsd_body_caps() returns it, and newlib does not
                    // hand it over for free the way glibc happens to. The host
                    // tests compiled fine without this; only the ESP32 build
                    // caught it. Fifth pattern, exactly.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One row per thing we can do TO THE CAR. Direction (seat forward/back, scroll
 * up/down) is a parameter of the emitter, not a row: it does not change the
 * risk, and a row that cannot change the answer is a row that should not exist.
 * Which PART of the car does change it -- the passenger seat needs an occupancy
 * check the driver's seat does not -- so those are separate rows. */
typedef enum {
    FSD_ACT_MAP_LIGHT = 0,  // was the old T1 row
    FSD_ACT_DOOR_OPEN,      // was the old T2 row
    FSD_ACT_CAMERA,         // 0x3C2 byte6 bit3 (toggle) -- measured 2026-09-01
    FSD_ACT_SEAT_DRIVER,    // 0x3C2 byte1 bits 2-3 / 0-1
    FSD_ACT_SEAT_PASSENGER, // 🔴 NOT the same frame: 0x3C3 byte0
                            //   (앞 bits 4-5 / 뒤 bits 2-3), measured
                            //   2026-09-03. Left/right is a different
                            //   FRAME, not different bits — VCLEFT vs
                            //   VCRIGHT. The old comment would have
                            //   sent an emitter to 0x3C2.
    FSD_ACT_SCROLL,         // 0x3C2 byte3, 6-bit signed detents
    FSD_ACT_GEAR_D,         // 0x229 -- P -> D only. See caps row.
    /* 🔴 APPENDED, NOT INSERTED, AND THAT IS NOT A STYLE CHOICE. This
     * number is written into NVS by fsd_rule_pack() (out[7] = rule->action) and
     * goes out on the wire. Inserting anywhere above would silently turn every
     * saved rule into a rule about a different action -- a door rule becoming a
     * seat rule, with nothing on screen to say so. New actions go here. */
    FSD_ACT_HAZARDS,        // 0x3E9 byte0 bit2 -- measured 2026-09-05
    /* Left/right/cancel is the emitter's ARGUMENT, not three rows. Same
     * reasoning as the seat: direction does not change the risk, so a row that
     * cannot change the answer should not exist. Which LAMP it is does not
     * change it either -- unlike the seats, where left and right are two
     * different people. */
    FSD_ACT_TURN_SIGNAL,    // 0x249 byte2 stalk replay -- measured 2026-09-05
    /* Fold and unfold are the ARGUMENT, for the third time and the same
     * reason: which way the mirrors go does not change what the action can
     * cost. 🔴 Shares 0x273 with the map light and writes a different byte --
     * the first pair of actions in this enum to share a frame and both have an
     * emitter, which is why fsd_pipe_observe() keys its templates by ACTION
     * and not by CAN id. */
    FSD_ACT_MIRROR,         // 0x273 byte3 (1 fold, 2 unfold) -- measured 2026-09-06
    /* 🔴 THE FIRST ACTION IN THIS ENUM THAT IS A GESTURE RATHER THAN A
     * SETTING. Every other one is a single frame the car acts on; this is a
     * press AND a release 12 ms later, and the 12 ms is what makes it the
     * LIGHT horn instead of a horn. See fsd_emit_release_ms().
     *
     * 🔴 It is also the first action that writes a bit we READ AS A SWITCH
     * (FSD_SIG_HORN_SW), which is why fsd_trig_disturbed()'s switch exemption
     * had to learn a condition on the same day. */
    FSD_ACT_LIGHT_HORN,     // 0x3C2 mux0 byte0 bit2, 12 ms -- measured 2026-09-06
    FSD_ACT_COUNT,
} FsdBodyAction;

/* 🔴🔴 THIS ROW USED TO CARRY THE GATES, AND THE OWNER REMOVED THEM
 * (2026-09-10): "차의 모든 안전게이트관련사항을 삭제해라. 필요하다면 추후
 * 내가 하나씩 추가하겠다."
 *
 * Gone: may_act_while_moving, may_act_out_of_park, armable_at_runtime,
 * requires_park, requires_passenger_empty, requires_belt, min_interval_ms --
 * together with the whole predicate that read them, the inputs it read, and
 * every named refusal it could return.
 *
 * 🔴 DELETED RATHER THAN DEFAULTED OPEN. A check that always passes still
 * looks like a gate, and this repository has been bitten by that shape more
 * than once; the 2026-09-08 removal of the driver and belt gates set the same
 * rule. If one comes back it comes back as a decision, not as a flag someone
 * flipped.
 *
 * WHAT STILL STANDS IN FRONT OF A FRAME -- and none of it is about the car's
 * situation, all of it is about not writing bytes nobody measured:
 *   - the rule has to exist and be switched on (fsd_rules, the owner's own list)
 *   - the emitter has to know how to build the frame at all
 *   - the bit-granularity chokepoint (fsd_body_wire) -- only measured bits
 *   - the deny-list below -- 0x3F5/0x102/0x103 here, 0x229 at send_on_bus()
 *   - the emitter needs a FRESH template from the car to copy */
typedef struct {
    FsdBodyAction action; // must equal its own index; checked at runtime

    /* 0 = this action may not be held at all (single-shot only). An action that
     * must be re-sent to stay in effect needs an upper bound, or one stuck
     * flag transmits forever.
     *
     * 🔴 KEPT ON PURPOSE while the gates went. This is not a question about
     * the car's state -- it is what stops a single stuck flag from writing to
     * the bus forever, which is the same family as the chokepoint. */
    uint16_t max_hold_ms;
} FsdBodyCaps;

/** The row for an action, or NULL if the index is out of range or the table
 *  drifted. Exposed so the emitter can read max_hold_ms without a second copy
 *  of the table -- two copies is how the deny-lists diverged before. */
const FsdBodyCaps* fsd_body_caps(FsdBodyAction a);


/* (FsdBodyInputs 는 게이트와 함께 사라졌다 — 위 상자 참조.) */

/* (FsdBodyVerdict 도 마찬가지다 — 거부할 것이 없으면 거부 이름도 없다.) */





/** Human-readable action name, for the same reason. */
const char* fsd_body_action_str(FsdBodyAction a);

/** True for every CAN ID this feature could conceivably want to write.
 *
 *  UNCONDITIONAL, and deliberately not derived from the capability table: this
 *  firmware constructs no body frame at all, so the honest answer for every one
 *  of these IDs is "refused", in every state, forever. It exists so that a
 *  future call site cannot forget its gate -- a backstop, never a permission.
 *
 *  🔴 0x3C2 is NOT here, and must not be added: the scroll path already writes
 *  it under its own double gate. That is exactly why step 2 of the redesign
 *  moves this chokepoint from frame granularity to BIT granularity -- camera,
 *  seat and scroll share one frame, so "refuse the ID" and "allow the ID" are
 *  both wrong answers. */
bool fsd_body_tx_id_refused(uint32_t can_id);

#ifdef __cplusplus
}
#endif
