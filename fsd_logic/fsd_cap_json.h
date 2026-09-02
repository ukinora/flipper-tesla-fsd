#pragma once
/*
 * fsd_cap_json.h — the module's two read-only documents, rendered in pure C.
 *
 * 🔴 WHY THIS IS NOT IN capability.cpp ANY MORE.
 *
 * The Capability document outgrew the 512 bytes a GATT attribute may hold THREE
 * times (2026-08-18 twice, 2026-08-19). Every time the same thing happened:
 * NimBLE REFUSED the oversized setValue() rather than truncating it, the
 * characteristic held ZERO bytes, and the phone read an empty document — which
 * a reader cannot tell apart from "the module has not answered yet".
 *
 * Every time, the fix was to shave bytes. Every time it fit again, until the
 * next field. Shaving is not a fix; it moves the overflow into the future and
 * into a place nobody is looking, because the overflow does not appear on the
 * bench. It appears in the CAR: the second bus only enters the document once it
 * has seen one of the capability ids, and the replay files used on the bench do
 * not contain them. So the document that fits on the desk is not the document
 * the car builds.
 *
 * The renderers therefore live here, in pure C with an explicit output cap, so
 * a host test can build the LARGEST document each can ever produce and assert
 * it fits. Add a field, and the test goes red on a desk instead of the
 * characteristic going empty in a car.
 *
 * That is also why fsd_cap_json_worst_*() live in this module rather than in
 * the test: the worst case is a property of the format, so it belongs next to
 * the format. Kept in the test file, the two would drift and the guard would
 * quietly stop guarding the fields added after it.
 *
 * Pure / deterministic: no I/O, no platform calls, no global state. The ESP32
 * wrapper in capability.cpp gathers live values into these structs and wraps
 * the result in a String; it makes no formatting decisions of its own.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Compiled as C for the host tests and linked from C++ on the board
 * (capability.cpp). Without this the ESP32 link fails on every one of the eight
 * board variants with an undefined reference — which is at least loud, unlike
 * most of what this module exists to prevent. */
#ifdef __cplusplus
extern "C" {
#endif

/* The largest value a GATT attribute may hold.
 *
 * 🔴 SPEC, NOT SETTING. BLE_ATT_ATTR_MAX_LEN is 512 in the Bluetooth core
 * specification, so raising the MTU does not raise this. capability.h
 * static-asserts that its CAP_ATTR_MAX equals this one — the host test must not
 * be measuring a different limit than the board enforces. */
#define FSD_CAP_JSON_ATTR_MAX 512u

/* Two CAN controllers on this board; both may appear in one document. */
#define FSD_CAP_JSON_MAX_BUSES 2u

/* Bound remotes. Mirrors BLE_CENTRAL_MAX_BUTTONS; capability.cpp static-asserts
 * it has not grown past this, because a slot the document cannot carry is a
 * slot the app cannot show. */
#define FSD_CAP_JSON_MAX_BOUND 5u

/* Logical buttons x events. Mirrors FSD_J6_COUNT * FSD_BTN_EVENTS. */
#define FSD_CAP_JSON_ROWS 30u

/* "2f:95:e6:54:51:84" plus the terminator. */
#define FSD_CAP_JSON_ADDR_MAX 18u

/* "can0" plus the terminator, with room for a longer name. */
#define FSD_CAP_JSON_BUSNAME_MAX 8u

/* ── the packed verdict word ─────────────────────────────────────────────────
 *
 * 🔴 THE SIX VERDICTS USED TO BE SIX NAMED FIELDS, and together with the hint
 * string they cost about 160 bytes PER BUS — a third of the whole budget spent
 * writing out what is six tri-state enums, one flag and a three-way hint. That
 * is fifteen bits. `ids` was compressed for exactly this reason and the same
 * argument applies here; writing a small enum as prose was the mistake.
 *
 * ⚠️ THE BIT LAYOUT IS A CONTRACT. wire/Capability.kt decodes it back into the
 * same named fields, so moving a field silently renames every field above it.
 * APPEND ONLY, and CapabilityTest pins every position. */
#define FSD_CAP_V_NAG_KILLER_SHIFT      0u
#define FSD_CAP_V_AP_FIRST_SHIFT        2u
#define FSD_CAP_V_FSD_ACTIVATION_SHIFT  4u
#define FSD_CAP_V_SOFT_ENGAGE_SHIFT     6u
#define FSD_CAP_V_BODY_CONTROL_SHIFT    8u
#define FSD_CAP_V_SCROLL_PROFILE_SHIFT 10u
#define FSD_CAP_V_HW_UNCONFIRMED_SHIFT 12u
#define FSD_CAP_V_HINT_SHIFT           13u

/* A verdict is FSDCapVerdict (0..2) and the hint FSDCapBusHint (0..2), so two
 * bits each. The packer masks to these widths: a value that does not fit is a
 * programming error upstream, and silently widening the field here would break
 * the decoder's fixed positions. */
#define FSD_CAP_V_VERDICT_MASK 0x3u
#define FSD_CAP_V_HINT_MASK    0x3u

/* One bus's line. The verdicts are FSDCapVerdict values and the hint an
 * FSDCapBusHint, carried as plain integers so this header stays free of
 * fsd_capability.h — the renderer does not interpret them, it only packs them. */
typedef struct {
    char     name[FSD_CAP_JSON_BUSNAME_MAX];
    uint32_t frames;
    uint32_t ids;            /* CAP_ID_BIT_* bitmask */
    uint8_t  nag_killer;
    uint8_t  ap_first;
    uint8_t  fsd_activation;
    uint8_t  soft_engage;
    uint8_t  body_control;
    uint8_t  scroll_profile;
    bool     hw_unconfirmed;
    uint8_t  hint;
} FsdCapJsonBus;

typedef struct {
    uint8_t  state;
    uint32_t ms_left;
    uint32_t window_ms;
    uint8_t  hw;
    uint8_t  bus_count;
    FsdCapJsonBus bus[FSD_CAP_JSON_MAX_BUSES];

    /* ── the capture disk ────────────────────────────────────────────────────
     *
     * 🔴 The phone could not see ANY of this, and the serial console is the only
     * other place it exists. That is a problem in the car: `bbclear` can now be
     * pressed from the phone (2026-09-02), so somebody can delete captures
     * WITHOUT KNOWING HOW MUCH ROOM THEY HAD -- or whether they needed to.
     *
     * 🔴 `mark_lost` is the heavy one. When a manual capture fails to become a
     * file, the phone sees only "받을 것이 없습니다", and that sentence cannot
     * tell the owner whether to shoot again or to clear the disk. main.cpp has
     * said exactly this in a comment since the flag was added; it just had no
     * way out of the board. Before TSL comes out, guessing wrong costs a capture
     * that cannot be retaken.
     *
     * Carried here rather than in a new command because the app already re-reads
     * this document, and rather than in State because free space changes about
     * once a minute while State goes out five times a second. */
    uint32_t bb_free_kb;  /* room left, KB */
    uint16_t bb_count;    /* captures stored */
    uint8_t  bb_lost;     /* 1 = the last manual mark never became a file */
} FsdCapJsonStatus;

typedef struct {
    uint8_t slot;
    char    addr[FSD_CAP_JSON_ADDR_MAX];
    bool    connected;
    uint8_t decoded;
} FsdCapJsonBound;

typedef struct {
    bool     connected;
    bool     verified;
    uint8_t  slots;
    uint8_t  bound_count;
    FsdCapJsonBound bound[FSD_CAP_JSON_MAX_BOUND];
    uint8_t  rows[FSD_CAP_JSON_ROWS];
    uint32_t act;            /* one bit per row */
} FsdCapJsonButtons;

/** Pack one bus's verdicts into the word the `v` field carries. Exposed so the
 *  test can pin each bit position without going through the renderer. */
uint32_t fsd_cap_json_pack_verdicts(const FsdCapJsonBus* b);

/** Render the Capability document — what CAN this module see.
 *
 * Returns the number of bytes written, not counting the terminator, or 0 if the
 * document did not fit in `cap` — in which case `out` is left holding an empty
 * string rather than a half-written one. A truncated JSON document is worse
 * than none: the parser rejects it and the reason never reaches anyone.
 */
size_t fsd_cap_json_status(char* out, size_t cap, const FsdCapJsonStatus* s);

/** Render the buttons document — which remotes are bound and what they pressed.
 *
 * 🔴 ITS OWN DOCUMENT, not a block inside Capability, and the reason is the
 * same one that moved the scan list out: five bound remotes are about four
 * hundred bytes of addresses and counters, which is most of the budget on its
 * own. Sharing one attribute meant one of the two had to lose, and the loser
 * lost completely — an attribute NimBLE refuses holds nothing at all, so an
 * overflow caused by the buttons also erased the bus verdicts.
 *
 * They are also different questions asked by different screens at different
 * times, so there was never much reason to carry them together. Same contract
 * as above.
 */
size_t fsd_cap_json_buttons(char* out, size_t cap, const FsdCapJsonButtons* b);

/** Fill with the largest values every field can hold.
 *
 * 🔴 KEEP THESE HONEST. They are the only reason the size guard means anything.
 * When you add a field to either document, widen the matching worst case in the
 * same commit — otherwise the test keeps passing while the real document grows
 * past it, which is exactly how this failed the last three times. */
void fsd_cap_json_worst_status(FsdCapJsonStatus* s);
void fsd_cap_json_worst_buttons(FsdCapJsonButtons* b);

#ifdef __cplusplus
}
#endif
