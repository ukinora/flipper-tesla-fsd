/*
 * fsd_body.c — see fsd_body.h. One predicate, no emitters, no wire tables.
 */

#include "fsd_body.h"

#include "fsd_autonomy.h" // FSD_GEAR_*
#include "fsd_state.h"   // FSDState, for fsd_body_inputs_from_state()

/* The capability table. Designated initialisers so a row cannot silently land
 * at the wrong index, plus a self-check inside each row and a length assertion
 * below — three independent ways to catch the same class of mistake, because
 * this project has already shipped a permission table that defaulted open.
 *
 * 🔴 FOUR ROWS ARE ARMABLE (2026-09-06). MAP_LIGHT carries over unchanged
 * from the old T1. DOOR_OPEN and HAZARDS joined it when the third visit
 * produced the thing their comments demanded -- the command frames, measured,
 * not inferred. DOOR_OPEN joined with every other restriction still on;
 * HAZARDS joined with the motion gates open, because a hazard light that may
 * only act in park cannot do what hazard lights are for. TURN_SIGNAL joined
 * on the fourth visit's frames plus two explicit owner decisions, which its
 * own row records -- the capture could not have supplied either of them.
 *
 * Each row names the evidence that flips its bool. That is the discipline this
 * table is for: a row opens when a stated condition is met, and the condition
 * is written down BEFORE anyone wants the row open. */
/* 🔴🔴 아래 행 주석에 `may_act_while_moving` · `may_act_out_of_park` ·
 * `armable_at_runtime` · `requires_park` · `requires_belt` ·
 * `requires_passenger_empty` · `min_interval_ms` 가 계속 나온다. **그 필드들은
 * 2026-09-10 에 사라졌다** (차주 지시).
 *
 * 문장을 지우지 않고 남기는 이유: 그것들은 **무엇을 왜 재서 그렇게 정했는지의
 * 기록**이고, 게이트를 하나씩 되살릴 때 그 근거가 필요하다. 차주가
 * *"필요하다면 추후 내가 하나씩 추가하겠다"* 고 했다.
 *
 * 🔴 **그러나 지금 동작을 말하는 문장이 아니다.** 이 저장소는 2026-09-09
 * 감사에서 정확히 이 모양에 물렸다 — 없는 필드(`may_act_without_driver`)를
 * 근거로 자기를 설명하는 주석 넷이 있었고, 그것만 읽으면 정반대로 안다. */
static const FsdBodyCaps FSD_BODY_CAPS[] = {
    [FSD_ACT_MAP_LIGHT] =
        {
            .action = FSD_ACT_MAP_LIGHT,
            /* An interior light is idempotent and its worst case is a light
             * left on, so none of the motion gates apply. Nothing else does
             * either now: the drive-session gate that used to stop a car which
             * had sat untouched since yesterday, and the occupancy gate this
             * row waived, were both removed on 2026-09-08. What holds this
             * action is the rate limit and the rule the owner switched on. */
            /* TSL holds the light by re-sending; whether we must too is not
             * settled (see 권한축-재설계.md 8-D), so the bound exists before the
             * emitter does rather than after it misbehaves. */
            .max_hold_ms = 30000u,
        },

    /* Opening a door has no undo and no upper bound on consequences.
     *
     * 🔴 THE CONDITION THIS ROW WROTE FOR ITSELF WAS MET (2026-09-05).
     * It said "FLIPS WHEN: the unfiltered capture identifies the actual command
     * frame", and the third visit did: 0x1F9 byte 1 = 0x03, injected in the
     * same millisecond as the car's own frame, 113 ms before the right front
     * door moved. So the row opens -- and nothing else about it relaxes.
     *
     * EVERY restriction stays on. The struct is permissive-when-true, so the
     * four bools left false below are four gates, each backed by a signal we
     * actually receive:
     *
     *      may_act_while_moving         false -> 0x257 DI_speed, standstill
     *      may_act_out_of_park          false -> 0x118 DI_gear, P only
     *
     * 🔴 AND HERE IS WHAT NONE OF THEM COVER. A door swings OUTWARD, and
     * nothing on this bus says what is beside the car -- not a person, not a
     * wall, not a passing cyclist. That hazard is unobservable, it is the main
     * one, and no gate here reduces it. What the gates do is make the car
     * stationary, in park, with someone in the driver's seat when it happens.
     * The residue is why action_enabled[] is session-scoped and dies with the
     * power: this is an action an operator opts into for one sitting, not a
     * setting a car carries around.
     *
     * min_interval_ms is 3000, an order above the light's 500. It is not a
     * debounce -- it is a bound on how bad a stuck rule can get, and a door
     * that reopens twice a second is a different event from a light that does.
     * (A 0 here would also be a second structural lock: fsd_body_caps_verdict()
     * refuses any row whose interval is 0, so the value had to be chosen, not
     * inherited.)
     *
     * max_hold_ms is 1000. TSL sends the command twice 300 ms apart and stops;
     * there is nothing to hold, and the bound exists so a re-sender written
     * later cannot quietly become one. */
    [FSD_ACT_DOOR_OPEN] =
        {
            .action = FSD_ACT_DOOR_OPEN,
            .max_hold_ms = 1000u,
        },

    [FSD_ACT_CAMERA] =
        {
            .action = FSD_ACT_CAMERA,
            /* Owner decision 2026-09-01: both on and off. The risk of "off" is
             * not that it is hard to undo — it is that nobody notices for
             * hours. That is answered by logging and by showing the state on
             * the dashboard, not by a gate. */
        },

    [FSD_ACT_SEAT_DRIVER] =
        {
            .action = FSD_ACT_SEAT_DRIVER,
            /* Owner decision 2026-09-01: moving as well. A Tesla driver can
             * already adjust their own seat under way, so this is not a new
             * capability for the car — but a stuck rule driving a motor is, and
             * that is what max_hold_ms bounds. No occupancy check: the person
             * in this seat is the person asking. */
            .max_hold_ms = 1000u,
        },

    [FSD_ACT_SEAT_PASSENGER] =
        {
            .action = FSD_ACT_SEAT_PASSENGER,
            /* The difference from the driver's seat, and the whole reason these
             * are two rows: nobody asked on behalf of the passenger. */
            .max_hold_ms = 1000u,
        },

    [FSD_ACT_SCROLL] =
        {
            .action = FSD_ACT_SCROLL,
            /* The detent the whole safety story was originally written around.
             * Its own path (fsd_speed_profile.c) keeps a separate double gate;
             * this row does not replace it. */
        },

    [FSD_ACT_GEAR_D] =
        {
            .action = FSD_ACT_GEAR_D,
            /* Owner decision 2026-09-01: adopted, replicating what TSL already
             * does in this car (belt latched -> D). Every gate this axis has,
             * plus two of its own.
             *
             * 🟢 The load-bearing interlock is not ours: Tesla will not leave P
             * without the brake pressed, and our frame is a stalk REQUEST that
             * the drive controller still judges. We cannot bypass it.
             * 🔴 We have also never verified that. All three captures had the
             * brake down because that is what the procedure said. Until the
             * no-brake case is measured, this row stays unarmable — borrowed
             * safety you have not seen is safety you are only assuming.
             *
             * FLIPS WHEN: the no-brake refusal is observed on the car. */
        },

    /* Hazards. Measured 2026-09-05: 0x3E9 byte 0 bit 2, with a counter and a
     * check that make this the one command we REWRITE rather than copy.
     *
     * The motion gates are OPEN here, and deliberately. TSL's own rule turns
     * the hazards on for reverse -- moving, out of park -- and a hazard light
     * that may only act in P is a hazard light that cannot do the thing hazard
     * lights are for. This is the row where "every restriction on" would be the
     * unsafe choice, not the safe one.
     *
     * 🔴 What bounds it instead is that we can only ever turn them ON.
     * The command sets a bit; it has no clear. Ceasing to send returns the
     * lamps to the car, exactly as the map light does. So the failure this row
     * has to survive is a stuck rule HOLDING them, and that is max_hold_ms --
     * not a gate on when it may start.
     *
     * 🔴 THE OCCUPANCY GATE THIS PARAGRAPH ARGUED ABOUT NO LONGER EXISTS
     * (removed 2026-09-08, owner's instruction). Kept as a record of what
     * the row was reasoning about; see fsd_body_caps_verdict() for what
     * stands in its place.
     *
     * It said: the driver gate stays shut, not because an empty car must never
     * flash -- that is arguably what hazards are for -- but because no rule we
     * had wanted it yet. 🔴 THAT IS NOW ALLOWED. Nothing here refuses an
     * unattended car any more.
     *
     * min_interval_ms 500 matches the car's own period (~495 ms): this is a
     * command that is held by re-sending, so the interval is a cadence, not a
     * debounce. */
    [FSD_ACT_HAZARDS] =
        {
            .action = FSD_ACT_HAZARDS,
            .max_hold_ms = 30000u,
        },

    /* Turn signals. Measured 2026-09-05: 0x249 byte 2, a replay of the stalk
     * itself -- not a lamp command. Two owner decisions on 2026-09-06 put this
     * row where it is, and both are recorded because neither follows from the
     * capture.
     *
     * 🔴 DECISION 1 -- MAY WE SEND THE STALK FRAME AT ALL. Yes (owner). It
     * matters because 0x249 is an INPUT frame: everything else this axis emits
     * is a request to a body controller, and this is us pretending a person
     * moved a control. That is the same technique the commercial device uses
     * for the speed profile, and the same one it uses here, so it is not new to
     * this car -- but "not new" is not "already agreed", and the deny-list did
     * not cover this id. Now it is deliberate rather than a default.
     *
     * 🔴 DECISION 2 -- MAY IT ACT WHILE MOVING. Yes (owner). The motion gates
     * are open for the same reason as the hazards, one step further: an
     * indicator that may only act in park cannot indicate anything. A lane
     * change is the whole point, and a lane change happens at speed. This is
     * the second row where "every restriction on" is the unsafe answer.
     *
     * 🔴 AND HERE IS WHAT THAT DOES NOT COVER. Unlike the hazards, this
     * command has a DIRECTION, so a wrong one is not a louder version of the
     * right one -- it tells the traffic behind that we are going the other way.
     * No gate on this bus can catch that; only the rule the owner wrote can be
     * right or wrong about it. What the gates do is the same as everywhere
     * else: refuse when we cannot see the car, and bound how long a stuck rule
     * can go on.
     *
     * 🔴 THE OCCUPANCY GATE THIS PARAGRAPH ARGUED ABOUT NO LONGER EXISTS
     * (removed 2026-09-08, owner's instruction). Kept as a record of what
     * the row was reasoning about; see fsd_body_caps_verdict() for what
     * stands in its place.
     *
     * It said: an empty car has no lane to change into. 🔴 THE INDICATOR CAN
     * NOW FIRE ON AN EMPTY CAR. What bounds it is the 50 ms interval and the
     * fact that a rule has to fire.
     *
     * min_interval_ms is 50, the car's own period for this frame. That number
     * was picked when the column was read as a FRAME spacing: TSL sends three
     * or four back to back at exactly that rate and we do not know which of
     * them the car acts on, so a larger interval would have made the emitter
     * structurally unable to reproduce the only sequence we have seen work.
     *
     * ⚠️ SINCE 2026-09-08 THE COLUMN COUNTS COMMANDS, so 50 no longer bounds
     * the burst at all -- the burst is one command and its frames are exempt.
     * What it now says is "a new indicator command may be issued twenty times
     * a second", which is permissive and deliberately left that way: this row
     * is not where the indicator is protected, and changing the number would
     * be a decision with its own measurement to do.
     * 🔴 That is the loosest interval in this table, and the thing that keeps
     * it from being a burst generator is max_hold_ms, not this.
     *
     * max_hold_ms is 1000. TSL is done in 150-200 ms. Unlike the map light and
     * the hazards this command is NOT held by re-sending -- the car latches the
     * indicator and the cancel is its own command -- so a re-sender is not a
     * feature to be bounded, it is a bug to be caught. */
    [FSD_ACT_TURN_SIGNAL] =
        {
            .action = FSD_ACT_TURN_SIGNAL,
            .max_hold_ms = 1000u,
        },

    /* Mirrors. Measured 2026-09-06: 0x273 byte 3, 1 folds and 2 unfolds, each
     * injected one millisecond behind a car frame that never carries either.
     *
     * 🔴 THE CONDITION THIS TABLE ASKS FOR IS MET, AND NOTHING ELSE IS.
     * The command frame is measured rather than inferred, so the row is
     * armable -- and every motion gate stays SHUT, which is the opposite of
     * what the hazard and indicator rows did. Those two opened the gates
     * because a hazard light that may only act in park cannot do what hazard
     * lights are for. The mirrors are the other case entirely:
     *
     *   🔴 FOLDING A MIRROR AT SPEED REMOVES REARWARD VISION. That is not a
     *   louder version of the right answer, it is a worse car. Every use an
     *   owner would actually write -- fold on walking away, unfold on getting
     *   in -- happens in park, so the restriction costs nothing it protects.
     *
     * 🔴 THE OCCUPANCY GATE THIS PARAGRAPH ARGUED ABOUT NO LONGER EXISTS
     * (removed 2026-09-08, owner's instruction). Kept as a record of what
     * the row was reasoning about; see fsd_body_caps_verdict() for what
     * stands in its place.
     *
     * 🟢 AND IT CHANGES WHAT THIS ROW CAN DO. The paragraph below noted that
     * the gate refused "fold the mirrors after I walk away" -- the most natural
     * rule for this action. That refusal is gone: the rule is now buildable.
     * The row still requires standstill and park, which that rule satisfies.
     *
     * (the paragraph as it stood) "That does refuse 'fold the mirrors after I
     * walk away', which is the most natural rule for this action. Refused on
     * purpose and written down rather than quietly granted; if the owner wants
     * it, this is the line that has to change and this is the paragraph that
     * says what changing it gives up." -- the line did change, and this is
     * what it gave up: nothing now asks whether anyone is in the car.
     *
     * min_interval_ms is 3000. This one drives a MOTOR that takes a second or
     * two to finish, and a rule re-firing inside that is a rule fighting the
     * hardware. Same order as the door, for a different reason: there it
     * bounds how bad a stuck rule gets, here it also bounds how often we ask a
     * mechanism to reverse mid-travel.
     *
     * max_hold_ms is 1000. TSL sends ONE frame per direction -- 20 idle frames
     * either side of it in the capture -- so there is nothing to hold, and the
     * bound is here so a re-sender written later cannot quietly become one. */
    [FSD_ACT_MIRROR] =
        {
            .action = FSD_ACT_MIRROR,
            .max_hold_ms = 1000u,
        },

    /* The light horn. Measured 2026-09-06: 0x3C2 mux 0 byte 0 bit 2, pressed
     * and released 12 ms later. A replay of the horn BUTTON, the same
     * technique as the indicator stalk -- we are pretending a person put a
     * thumb on a control, not asking a body controller for a noise.
     *
     * 🔴 THIS ROW IS ON THE HAZARDS' SIDE OF THE LINE, NOT THE MIRROR'S, and
     * the two are one row apart so the difference has to be said. A horn warns
     * somebody who is about to hit you; that happens at speed, and a horn
     * which may only sound in park cannot do it. Same sentence as the hazard
     * row, and the opposite of the mirror row directly above -- where the
     * restriction costs nothing because nobody folds a mirror at 80 km/h on
     * purpose.
     *
     * 🔴 THE OCCUPANCY GATE THIS PARAGRAPH ARGUED ABOUT NO LONGER EXISTS
     * (removed 2026-09-08, owner's instruction). Kept as a record of what
     * the row was reasoning about; see fsd_body_caps_verdict() for what
     * stands in its place.
     *
     * It said the same sentence a third time. 🔴 The horn can now sound with
     * nobody in the car; the 1000 ms interval is what keeps a stuck rule from
     * leaning on it.
     *
     * 🔴 min_interval_ms IS THE GATE THAT MATTERS HERE, because the failure to
     * survive is not one beep in the wrong place -- it is a stuck rule leaning
     * on the horn. 1000 is an order above the map light's 500 and still leaves
     * the feature usable: nobody wants two beeps in one second.
     *
     * 🟢 AND IT IS ENFORCED SINCE 2026-09-08. It was not before: last_act_ms
     * had no producer anywhere in the firmware -- body_task.cpp memset the
     * struct and nothing ever wrote that array -- so every min_interval_ms in
     * this table was decorative, four gates advertised and three enforced.
     *
     * It was found while wiring this row and left alone on purpose, because
     * switching it on naively breaks the indicator: four frames about 50 ms
     * apart against a row that allows 50 ms. 🔴 The answer turned out not to be
     * a bigger number but a re-reading of the question -- this column counts
     * COMMANDS, which is what every row in this table already assumed (3000
     * here is "do not open the door twice in three seconds"). A burst is one
     * command; it is stamped once, at the press, and its own frames are exempt.
     * See fsd_burst_fill_last_act().
     *
     * max_hold_ms is 1000. There is nothing to hold -- the gesture is 12 ms --
     * so the bound is here for the same reason as the door's: a re-sender
     * written later must not quietly become one. */
    [FSD_ACT_LIGHT_HORN] =
        {
            .action = FSD_ACT_LIGHT_HORN,
            .max_hold_ms = 1000u,
        },
};

_Static_assert(sizeof(FSD_BODY_CAPS) / sizeof(FSD_BODY_CAPS[0]) == FSD_ACT_COUNT,
               "every FsdBodyAction needs an explicit capability row");

const FsdBodyCaps* fsd_body_caps(FsdBodyAction a) {
    if(a >= FSD_ACT_COUNT) return NULL;
    const FsdBodyCaps* c = &FSD_BODY_CAPS[a];
    if(c->action != a) return NULL;
    return c;
}

bool fsd_body_tx_id_refused(uint32_t can_id) {
    /* Hardcoded, not derived from the capability table, and true regardless of
     * any flag. 0x3F5 is the lighting frame, 0x102/0x103 the door status frames.
     *
     * 🔴 0x3F5 WAS OUR MAP-LIGHT CANDIDATE AND IT WAS WRONG (2026-09-03).
     * TSL turns the map lights on with 0x273, not 0x3F5 — six captures agree.
     * The refusal stays anyway: nothing constructs 0x3F5, so refusing it costs
     * nothing and it remains exactly what a mistake would reach for first.
     *
     * 🔴 0x273 IS DELIBERATELY NOT ON THIS LIST. It is the frame the map
     * light emitter builds, so refusing it here would refuse the one thing this
     * axis exists to allow. Its protection is the other three layers — the rule
     * has to match, the axis has to allow, and today nothing calls the emitter
     * at all. If that ever feels thin, the answer is a narrower gate on 0x273
     * (bit 59 only, as fsd_body_wire.c already does for 0x3C2), not a blanket
     * refusal that would make the feature impossible. */
    return can_id == 0x3F5u || can_id == 0x102u || can_id == 0x103u;
}

const char* fsd_body_action_str(FsdBodyAction a) {
    switch(a) {
    case FSD_ACT_MAP_LIGHT: return "map-light";
    case FSD_ACT_DOOR_OPEN: return "door-open";
    case FSD_ACT_CAMERA: return "camera";
    case FSD_ACT_SEAT_DRIVER: return "seat-driver";
    case FSD_ACT_SEAT_PASSENGER: return "seat-passenger";
    case FSD_ACT_SCROLL: return "scroll";
    case FSD_ACT_GEAR_D: return "gear-D";
    case FSD_ACT_HAZARDS: return "hazards";
    /* One word, no spaces -- the log line is "<name> -> <verdict>" and a
     * two-word name makes the arrow the only thing separating them. Same rule
     * as the J6 button names. */
    case FSD_ACT_TURN_SIGNAL: return "turn-signal";
    case FSD_ACT_MIRROR: return "mirror";
    case FSD_ACT_LIGHT_HORN: return "light-horn";
    case FSD_ACT_COUNT: break;
    }
    return "?";
}

/* 🔴🔴 fsd_body_allows() · fsd_body_caps_verdict() · fsd_body_inputs_from_state()
 * · fsd_body_verdict_str() ARE ALL GONE (owner's instruction, 2026-09-10):
 * "차의 모든 안전게이트관련사항을 삭제해라. 필요하다면 추후 내가 하나씩
 * 추가하겠다."
 *
 * Deleted rather than defaulted open, for the reason this repository keeps
 * re-learning: a check that always passes still LOOKS like a gate, and the
 * next person reads it as one. The 2026-09-08 removal of the driver and belt
 * gates set that rule and this follows it -- fields, verdict names, the inputs
 * struct and the producers went with the predicate.
 *
 * WHAT IS LEFT IN THIS FILE is not a gate about the car's situation. It is the
 * deny-list below: three ids that must never leave this module no matter what
 * built them. That, the bit chokepoint in fsd_body_wire.c, and the emitter's
 * need for a fresh template are the whole of what stands in front of a frame
 * now -- plus the owner's own rule list, which is a choice and not a gate. */

