/* fsd_cap_json.c — see fsd_cap_json.h. */

#include "fsd_cap_json.h"

#include <string.h>

/* ── a sink that cannot half-write ───────────────────────────────────────────
 *
 * Every append goes through here. Once `ok` goes false it stays false and
 * nothing more is written, so a document that does not fit produces NOTHING
 * rather than a truncated JSON body. Truncated JSON is the worse failure: the
 * parser rejects it and the reason never reaches anyone.
 */
typedef struct {
    char*  buf;
    size_t cap;   /* bytes usable for text, terminator excluded */
    size_t len;
    bool   ok;
} CapSink;

static void sink_init(CapSink* s, char* out, size_t cap) {
    s->buf = out;
    s->cap = (cap == 0u) ? 0u : (cap - 1u);   /* leave room for the terminator */
    s->len = 0u;
    s->ok  = (out != NULL) && (cap != 0u);
    if (s->ok) out[0] = '\0';
}

static void put(CapSink* s, const char* txt) {
    if (!s->ok) return;
    const size_t n = strlen(txt);
    if (s->len + n > s->cap) { s->ok = false; return; }
    memcpy(s->buf + s->len, txt, n);
    s->len += n;
}

static void put_c(CapSink* s, char c) {
    if (!s->ok) return;
    if (s->len + 1u > s->cap) { s->ok = false; return; }
    s->buf[s->len++] = c;
}

static void put_u(CapSink* s, uint32_t v) {
    char tmp[11];
    uint8_t n = 0;
    if (v == 0u) {
        put_c(s, '0');
        return;
    }
    while (v != 0u && n < (uint8_t)sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (n != 0u) put_c(s, tmp[--n]);
}

static void put_bool(CapSink* s, bool v) { put(s, v ? "true" : "false"); }

/* Close out: either the whole document or an empty string. */
static size_t sink_finish(CapSink* s) {
    if (!s->ok) {
        if (s->buf != NULL) s->buf[0] = '\0';
        return 0u;
    }
    s->buf[s->len] = '\0';
    return s->len;
}

uint32_t fsd_cap_json_pack_verdicts(const FsdCapJsonBus* b) {
    if (b == NULL) return 0u;
    uint32_t v = 0u;
    v |= (uint32_t)(b->nag_killer     & FSD_CAP_V_VERDICT_MASK) << FSD_CAP_V_NAG_KILLER_SHIFT;
    v |= (uint32_t)(b->ap_first       & FSD_CAP_V_VERDICT_MASK) << FSD_CAP_V_AP_FIRST_SHIFT;
    v |= (uint32_t)(b->fsd_activation & FSD_CAP_V_VERDICT_MASK) << FSD_CAP_V_FSD_ACTIVATION_SHIFT;
    v |= (uint32_t)(b->soft_engage    & FSD_CAP_V_VERDICT_MASK) << FSD_CAP_V_SOFT_ENGAGE_SHIFT;
    v |= (uint32_t)(b->body_control   & FSD_CAP_V_VERDICT_MASK) << FSD_CAP_V_BODY_CONTROL_SHIFT;
    v |= (uint32_t)(b->scroll_profile & FSD_CAP_V_VERDICT_MASK) << FSD_CAP_V_SCROLL_PROFILE_SHIFT;
    if (b->hw_unconfirmed) v |= 1u << FSD_CAP_V_HW_UNCONFIRMED_SHIFT;
    v |= (uint32_t)(b->hint & FSD_CAP_V_HINT_MASK) << FSD_CAP_V_HINT_SHIFT;
    return v;
}

size_t fsd_cap_json_status(char* out, size_t cap, const FsdCapJsonStatus* s) {
    CapSink k;
    sink_init(&k, out, cap);
    if (s == NULL) { k.ok = false; return sink_finish(&k); }

    put(&k, "{\"state\":");     put_u(&k, s->state);
    put(&k, ",\"ms_left\":");   put_u(&k, s->ms_left);
    put(&k, ",\"window_ms\":"); put_u(&k, s->window_ms);
    put(&k, ",\"hw\":");        put_u(&k, s->hw);
    /* One object rather than three top-level keys: it keeps the disk together
     * when someone reads the raw document, and costs the same bytes. */
    put(&k, ",\"bb\":{\"free_kb\":"); put_u(&k, s->bb_free_kb);
    put(&k, ",\"n\":");               put_u(&k, s->bb_count);
    put(&k, ",\"lost\":");            put_bool(&k, s->bb_lost != 0u);
    put(&k, "}");

    put(&k, ",\"buses\":[");

    uint8_t n = s->bus_count;
    if (n > FSD_CAP_JSON_MAX_BUSES) n = (uint8_t)FSD_CAP_JSON_MAX_BUSES;
    for (uint8_t i = 0; i < n; i++) {
        const FsdCapJsonBus* b = &s->bus[i];
        if (i != 0u) put_c(&k, ',');
        put(&k, "{\"bus\":\"");    put(&k, b->name);
        put(&k, "\",\"frames\":"); put_u(&k, b->frames);
        put(&k, ",\"ids\":");      put_u(&k, b->ids);
        put(&k, ",\"v\":");        put_u(&k, fsd_cap_json_pack_verdicts(b));
        put_c(&k, '}');
    }
    put(&k, "]}");

    return sink_finish(&k);
}

size_t fsd_cap_json_buttons(char* out, size_t cap, const FsdCapJsonButtons* t) {
    CapSink k;
    sink_init(&k, out, cap);
    if (t == NULL) { k.ok = false; return sink_finish(&k); }

    put(&k, "{\"connected\":");  put_bool(&k, t->connected);
    put(&k, ",\"verified\":");   put_bool(&k, t->verified);
    put(&k, ",\"slots\":");      put_u(&k, t->slots);
    put(&k, ",\"bound\":[");

    /* 🔴 SHORT KEYS, and only here. Five bound remotes are five 17-character
     * addresses plus their state, and spelling the keys out cost 120 bytes of a
     * 512-byte budget for four facts the app reads positionally anyway. The
     * outer keys stay readable because they are written once. */
    uint8_t bn = t->bound_count;
    if (bn > FSD_CAP_JSON_MAX_BOUND) bn = (uint8_t)FSD_CAP_JSON_MAX_BOUND;
    for (uint8_t i = 0; i < bn; i++) {
        const FsdCapJsonBound* e = &t->bound[i];
        if (i != 0u) put_c(&k, ',');
        put(&k, "{\"s\":");   put_u(&k, e->slot);
        put(&k, ",\"a\":\""); put(&k, e->addr);
        put(&k, "\",\"c\":"); put_u(&k, e->connected ? 1u : 0u);
        put(&k, ",\"d\":");   put_u(&k, e->decoded);
        put_c(&k, '}');
    }
    put(&k, "],");

    /* 🔴 HEX STRING, NOT AN ARRAY, and the reason is size. Thirty rows written
     * as decimal numbers grow with the counts — a document that fits today
     * stops fitting after a hundred presses, and NimBLE answers an oversized
     * value by storing NOTHING. Two hex characters per row is 60 bytes no
     * matter what the counts are, so the document cannot outgrow the limit by
     * being used. */
    put(&k, "\"btn\":\"");
    for (uint8_t r = 0; r < FSD_CAP_JSON_ROWS; r++) {
        static const char kHex[] = "0123456789abcdef";
        const uint8_t v = t->rows[r];
        put_c(&k, kHex[(v >> 4) & 0x0Fu]);
        put_c(&k, kHex[v & 0x0Fu]);
    }
    /* Last field, so no trailing comma. The document was invalid JSON for a
     * whole build once because a field was removed from in front of the closing
     * brace and left `"act":N,}` behind. */
    put(&k, "\",\"act\":"); put_u(&k, t->act);
    put_c(&k, '}');

    return sink_finish(&k);
}

/* ── the worst case ──────────────────────────────────────────────────────────
 *
 * Filled from each field's TYPE maximum, not from what the values happen to
 * reach today. `ids` uses eleven bits now, but the field is a uint32 and the
 * renderer prints whatever is in it; a guard built on today's semantics stops
 * guarding the moment a twelfth id is added. Booleans take their LONGER
 * spelling ("false").
 *
 * The result is deliberately pessimistic. A document that passes this cannot
 * overflow for any input, which is the whole point — the last three overflows
 * were all "but it fits with the values we have".
 */
void fsd_cap_json_worst_status(FsdCapJsonStatus* s) {
    if (s == NULL) return;
    memset(s, 0, sizeof(*s));

    s->state     = 0xFFu;
    s->ms_left   = 0xFFFFFFFFu;
    s->window_ms = 0xFFFFFFFFu;
    s->hw        = 0xFFu;
    s->bus_count = (uint8_t)FSD_CAP_JSON_MAX_BUSES;

    /* Widened in the same commit as the fields, as the header demands. `lost`
     * renders as `true`, which is the longer of the two spellings. */
    s->bb_free_kb = 0xFFFFFFFFu;
    s->bb_count   = 0xFFFFu;
    s->bb_lost    = 1u;

    for (uint8_t i = 0; i < FSD_CAP_JSON_MAX_BUSES; i++) {
        FsdCapJsonBus* b = &s->bus[i];
        memset(b->name, 'x', FSD_CAP_JSON_BUSNAME_MAX - 1u);
        b->name[FSD_CAP_JSON_BUSNAME_MAX - 1u] = '\0';
        b->frames         = 0xFFFFFFFFu;
        b->ids            = 0xFFFFFFFFu;
        b->nag_killer     = 0xFFu;
        b->ap_first       = 0xFFu;
        b->fsd_activation = 0xFFu;
        b->soft_engage    = 0xFFu;
        b->body_control   = 0xFFu;
        b->scroll_profile = 0xFFu;
        b->hw_unconfirmed = true;
        b->hint           = 0xFFu;
    }
}

void fsd_cap_json_worst_buttons(FsdCapJsonButtons* t) {
    if (t == NULL) return;
    memset(t, 0, sizeof(*t));

    t->connected   = false;   /* "false" is longer than "true" */
    t->verified    = false;
    t->slots       = 0xFFu;
    t->bound_count = (uint8_t)FSD_CAP_JSON_MAX_BOUND;
    for (uint8_t i = 0; i < FSD_CAP_JSON_MAX_BOUND; i++) {
        FsdCapJsonBound* e = &t->bound[i];
        e->slot = 0xFFu;
        memset(e->addr, 'f', FSD_CAP_JSON_ADDR_MAX - 1u);
        e->addr[FSD_CAP_JSON_ADDR_MAX - 1u] = '\0';
        e->connected = true;
        e->decoded   = 0xFFu;
    }
    for (uint8_t r = 0; r < FSD_CAP_JSON_ROWS; r++) t->rows[r] = 0xFFu;
    t->act = 0xFFFFFFFFu;
}
