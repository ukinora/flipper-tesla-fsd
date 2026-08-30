/*
 * blackbox.cpp — black-box incident recorder (ESP32, #124). See blackbox.h.
 *
 * Layout: a common section (ring buffer, PSRAM detect, trigger/flush logic,
 * .json build) and a storage section split by a compile-time backend, chosen
 * per board exactly like can_dump.cpp's per-board #if blocks:
 *   - LittleFS  : boards with a real data partition (waveshare-s3, lilygo-t2can)
 *   - SD        : LILYGO (reuses the SD volume can_dump.cpp already mounts)
 *   - RAM       : min_spiffs boards — keep the last event in RAM, download-only
 */

#include "blackbox.h"
#include "config.h"
#include "../../fsd_logic/fsd_capture.h"           // tesla_format_candump_line
#include "../../fsd_logic/fsd_events.h"            // fsd_events_inject
#include "../../fsd_logic/fsd_blackbox_summary.h"  // fsd_blackbox_format_json
#include "../../fsd_logic/fsd_blackbox_filter.h"   // fsd_blackbox_should_record
#include "../../fsd_logic/fsd_readcache.h"         // FsdReadCache (download path)
#include <stdlib.h>
#include <string.h>
#if defined(BLACKBOX_BACKEND_LITTLEFS)
#include <Preferences.h>   // fs_ever_mounted() — the format guard, see backend_init()
#endif

// Backend selection (BLACKBOX_BACKEND_*) + the enable-default live in blackbox.h
// so main.cpp / prefs.cpp share one source of truth.

// ── Tunable window ───────────────────────────────────────────────────────────
// 10 s total, split 5 pre / 5 post (was 10/5 = 15 s, then 5/1; post restored
// to 5 s on 2026-08-23 -- see the POST-ROLL note below).
//
// Why shrink: capture bytes are linear in window length, and on this fork both
// costs that matter are bytes. Measured 2026-08-17 on the bench —
//   * a 15 s window filled the whole 40000-frame ring: ~3.1 MB claimed against
//     a 3.5 MB partition, i.e. ONE capture on disk and the next one refused;
//   * BLE download runs 8-14 KB/s, so that capture takes 3.5-6 minutes to pull.
// The car procedure (차량-방문-체크리스트.md B-2.5) is three repeats per action,
// A and B sets, many actions — at 15 s that is hours of downloading, with a
// delete-between-every-capture dance because only one fits.
//
// 🔴 POST-ROLL WENT BACK UP: 1 s -> 5 s (owner's call, 2026-08-23).
//
// The 1 s post-roll was argued from the MANUAL sequence — the operator performs
// the action and THEN reaches for `mark`, so the evidence is all pre-roll and
// the post-roll holds nothing but the operator's own hand. That reasoning is
// sound for that ONE sequence, and it quietly made that sequence mandatory:
// with 1 s of post-roll there is no other way to take a capture.
//
// A 5 s post-roll buys a SECOND procedure, and it is the safer one:
//
//     press `mark` FIRST, then perform the action within 5 s.
//
// That matters because the reaction budget is this capture's single largest
// risk, and it fails SILENTLY — overrun it and the capture still succeeds,
// still downloads, is still the right size, and simply does not contain the
// gesture, which for the one-shot pre-teardown capture is discovered at home
// after the TSL is out. Symmetric 5/5 gives the operator a budget in whichever
// direction they are able to work, and a way to retry an action that went wrong
// without re-arming. It also covers a delayed reaction — TSL answering an input
// a beat later, or the second half of an event (door closes -> light fades).
//
// What it does NOT cost: the ring. At 40000 frames a 10 s window is nowhere
// near capacity on a filtered bus, and the ring caps a capture's worst case at
// ~3.2 MB regardless of window length (bb_would_fit refuses rather than
// overrunning the partition). Disk cost is frames x BB_BYTES_PER_LINE, so it
// scales with the bus's FILTERED rate — which we have never measured in the
// car. Measure it on the first capture rather than guessing here.
//
// Both are build flags if the car says otherwise:
//     build_flags = -D BLACKBOX_PRE_MS=8000 -D BLACKBOX_POST_MS=3000
#ifndef BLACKBOX_PRE_MS
#define BLACKBOX_PRE_MS    5000u   // pre-roll kept before the trigger
#endif
#ifndef BLACKBOX_POST_MS
#define BLACKBOX_POST_MS   5000u   // post-roll recorded after the trigger
#endif
// Floors, not preferences. A pre-roll under ~2 s cannot span a human noticing
// they finished the action and pressing the button, so a manual capture would
// systematically miss what it exists to record. A post-roll under ~500 ms lands
// inside one loop() period, so blackbox_tick() can flush before any post-trigger
// frame arrives — and blackbox_mark()'s "ms until the file exists" return value
// would tell the app to download a file that is not written yet.
static_assert(BLACKBOX_PRE_MS  >= 2000u, "pre-roll too short to hold a human reaction");
static_assert(BLACKBOX_POST_MS >= 500u,  "post-roll too short: flush races the loop");
// Ring capacity in frames @ 19 B/frame (BBFrame carries a 1-byte RX/TX tag).
// PSRAM (runtime-detected) holds the window many times over at a busy bus rate.
// The internal-RAM fallback must cover the window on its own — PSRAM is never
// required and no build enables it:
//   - S3-class (512 KB SRAM): 6000 frames ≈ 114 KB → ≥15 s even at ~400 f/s,
//     so it covers the 10 s window with room to spare.
//     Only paired with the persistent disk backends here, which stream the ring
//     straight to file (no frozen copy), so the ring is the whole footprint.
//   - Classic ESP32: 3000 frames ≈ 57 KB safety cap. These are the volatile
//     RAM-backend boards (default OFF) and a flush also frees+allocs a frozen
//     copy of the in-window frames, so the cap bounds the transient peak.
// All allocation/heap-guard math uses sizeof(BBFrame), so the byte figures track
// the struct automatically — only these comments need the 18→19 B update.
#ifndef BLACKBOX_FRAMES_PSRAM
#define BLACKBOX_FRAMES_PSRAM    40000u   // ~760 KB
#endif
#ifndef BLACKBOX_FRAMES_INTERNAL
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define BLACKBOX_FRAMES_INTERNAL  6000u   // ~114 KB (S3, 512 KB SRAM)
#else
#define BLACKBOX_FRAMES_INTERNAL  3000u   // ~57 KB  (classic ESP32)
#endif
#endif

#define BLACKBOX_DIR        "/blackbox"
#define BLACKBOX_RETAIN      5u      // keep newest N events (disk backends)
#define BLACKBOX_TL_MAX      16      // das_ap_state timeline depth
#define BLACKBOX_TL_EMIT     12      // timeline entries written to the summary

struct __attribute__((packed)) BBFrame {
    uint32_t ts_ms;
    uint32_t id;
    uint8_t  bus;
    uint8_t  dlc;
    uint8_t  is_tx;    // 0 = RX (bus frame), 1 = TX (our injected/echoed frame)
    uint8_t  data[8];
};

// ── Common module state ──────────────────────────────────────────────────────
static FSDState*     g_state = nullptr;
static portMUX_TYPE* g_mux   = nullptr;

static BBFrame*  g_ring = nullptr;
static uint32_t  g_cap  = 0;        // ring capacity (frames); 0 = unavailable
static uint32_t  g_head = 0;        // next write slot
static uint32_t  g_tail = 0;        // oldest frame
static bool      g_ring_psram = false;
static bool      g_want_psram = false;  // size decision from init; alloc is lazy

static bool      g_armed = false;

/* 🔴 "방금 찍은 것이 실제로 저장됐는가" 를 센다 (2026-08-31 레드팀 ①).
 *
 * do_flush() 는 자리가 모자라면 **아무것도 안 쓰고 return** 하고, 그 직후
 * blackbox_tick() 이 g_armed 를 내린다. 그러면 ble_bulk_start() 의 눈에는
 * "무장 안 됨 + 캡처 있음" 이라 **직전 캡처를 성공으로 내준다.**
 *
 * 실물 재현 (2026-08-31):
 *   flushed evt_5249_manual        <- 캡처 A 저장, 남은 공간 1824 KB
 *   저장 거부 evt_25279_manual     <- 두 번째는 거부됐는데
 *   read test evt_5249_manual.log  <- "최신" 은 여전히 A 다
 *
 * 앱에는 전부 성공으로 보이고, 사장님은 **직전 동작의 파일**을 들고 차를
 * 떠난다. 되돌릴 수 없는 캡처에서 이것이 조용히 일어난다.
 *
 * 두 수가 어긋나 있으면 "마지막 수동 mark 가 파일이 되지 못했다" 는 뜻이다.
 * 🔴 **자동 캡처는 세지 않는다** — 차에서 EVT_* 가 실패했다고 사장님의 정상
 * 다운로드를 막으면 안 된다. 사람이 요청한 그 한 건만 지킨다. */
static uint32_t  g_manual_armed_n = 0;   // MANUAL 로 무장한 횟수
static uint32_t  g_manual_saved_n = 0;   // 그중 실제로 파일이 된 횟수
static BBTrigger g_trig = BB_TRIG_ABORT;
static uint32_t  g_trig_ms = 0;
static uint32_t  g_flush_at_ms = 0;
static FSDState  g_snap;            // FSDState at trigger time (toggles/hw/evt_last_*)
static uint32_t  g_captures = 0;    // monotonic count since boot (badge source)

// das_ap_state mini-timeline
static uint32_t  g_tl_ts[BLACKBOX_TL_MAX];
static uint8_t   g_tl_state[BLACKBOX_TL_MAX];
static uint32_t  g_tl_head = 0;
static uint32_t  g_tl_count = 0;
static uint8_t   g_tl_last = 0xFF;

static void bb_enter() { if (g_mux) portENTER_CRITICAL(g_mux); }
static void bb_exit()  { if (g_mux) portEXIT_CRITICAL(g_mux); }

static const char* trig_name(BBTrigger t) {
    switch (t) {
        case BB_TRIG_ABORT:  return "abort";
        case BB_TRIG_BUSOFF: return "busoff";
        case BB_TRIG_MANUAL: return "manual";
        default:             return "evt";
    }
}
static const char* trig_name_uc(BBTrigger t) {
    switch (t) {
        case BB_TRIG_ABORT:  return "ABORT";
        case BB_TRIG_BUSOFF: return "BUSOFF";
        case BB_TRIG_MANUAL: return "MANUAL";
        default:             return "EVT";
    }
}

static inline uint32_t ring_next(uint32_t i) { return (i + 1u) % g_cap; }

// candump interface field for a stored frame. RX keeps the plain can0/can1 so
// existing parsers (CRC cracker, candump tools) read the .log unchanged; TX (our
// injected frames) get a can0TX/can1TX suffix — a greppable, ID-preserving
// direction tag. candump has no standard direction column, so the interface
// field carries it; the ID/data fields stay in their usual positions.
static inline const char* bb_iface_name(const BBFrame& f) {
    if (f.is_tx) return f.bus == CAN_BUS_SECONDARY ? "can1TX" : "can0TX";
    return f.bus == CAN_BUS_SECONDARY ? "can1" : "can0";
}

// Event basenames are device-generated as [A-Za-z0-9_]; reject anything else so
// a crafted ?name= can't escape BLACKBOX_DIR (no '/', '.', etc.).
static bool bb_name_ok(const char* name) {
    if (!name || !name[0]) return false;
    size_t n = strlen(name);
    if (n >= 40) return false;
    for (size_t i = 0; i < n; i++) {
        char c = name[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_')) return false;
    }
    return true;
}

/* 🔴 Declared OUT here, above the backend #if, on purpose: do_flush() names
 * every capture and do_flush() is common to all backends. Inside the disk block
 * it compiled on this board and broke the other five -- the same shape that has
 * caught this repository before (fork PR #56). Only the disk backend has a
 * directory to seed it from; on RAM it stays 0, which is correct because that
 * backend holds a single slot and has no ordering to get wrong. */
/* Added to millis() when naming a capture, so names keep increasing across a
 * reboot.
 *
 * 🔴 Capture names are evt_<ms>_<trigger>, and <ms> is millis(), which restarts
 * at 0 every boot. "Latest" picks the largest <ms> and retention evicts the
 * smallest, so a capture taken 3 s after a reboot looks OLDER than one taken
 * 50 s before it. The BLE download has no "list" and no "fetch by name" -- only
 * "give me the latest" -- so the operator can complete a download, see success,
 * and be holding the previous capture. In the one-shot session before the TSL
 * comes out that is discovered at home, if at all.
 *
 * Seeded at boot from the highest name already on disk, so the very next
 * capture outranks everything stored. No NVS needed: the directory IS the
 * record. With nothing stored the base is 0 and there is nothing to be out of
 * order with. */
static uint32_t g_name_base = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Storage backends
// ─────────────────────────────────────────────────────────────────────────────
#if defined(BLACKBOX_BACKEND_LITTLEFS) || defined(BLACKBOX_BACKEND_SD)

#if defined(BLACKBOX_BACKEND_LITTLEFS)
  #include <LittleFS.h>
  #define BB_FS        LittleFS
  #define BB_OPEN_W(p) BB_FS.open((p), "w")
  #define BB_OPEN_R(p) BB_FS.open((p), "r")
#else
  #include <SD.h>
  #define BB_FS        SD
  #define BB_OPEN_W(p) BB_FS.open((p), FILE_WRITE)
  #define BB_OPEN_R(p) BB_FS.open((p), FILE_READ)
#endif

static bool g_fs_ok = false;
// Cached count of saved .json events. The status/aux poll (~every 2.5 s) reads
// this instead of scanning the directory — a live scan on that hot path was
// stalling the web task and dropping WiFi (#124). Kept current by recomputing
// only inside the save/delete paths, which already touch the filesystem.
static int  g_event_count = 0;

// Read-ahead block for the capture download path — see backend_read_chunk() for
// the measurements that justify it. Declared up here because the write and
// delete paths above invalidate it, and they come first in this file.
//
// 🔴 Off by default on classic ESP32. It is a pure speed optimisation and it
// costs static DRAM, which those boards do not have: adding 4 KB overflowed
// esp32-lilygo's dram0_0_seg by 4,432 bytes, i.e. it had ~176 bytes spare. They
// also do not need it — the fast path exists for the BLE capture download,
// which is the S3 boards' job here; the classic boards download over the web
// dashboard, which streams the file in one open (backend_stream_body).
#ifndef BB_READAHEAD
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define BB_READAHEAD 4096u   /* one LittleFS block; ~8 BLE chunks per open */
#else
#define BB_READAHEAD 0u      /* no DRAM to spare — plain open/seek per chunk */
#endif
#endif

#if BB_READAHEAD > 0
static uint8_t      g_rc_buf[BB_READAHEAD];
static FsdReadCache g_rc;
#define BB_RC_DROP() fsd_rc_reset(&g_rc)
#else
#define BB_RC_DROP() ((void)0)
#endif

// Strip a directory prefix and a trailing extension → bare event basename.
static void bb_basename(const char* path, char* out, size_t n) {
    const char* slash = strrchr(path, '/');
    const char* base = slash ? slash + 1 : path;
    size_t i = 0;
    for (; base[i] && base[i] != '.' && i < n - 1; i++) out[i] = base[i];
    out[i] = '\0';
}

static uint32_t bb_seq_from_name(const char* base) {
    // evt_<ms>_<trigger>  → parse <ms>
    const char* u = strchr(base, '_');
    return u ? (uint32_t)strtoul(u + 1, nullptr, 10) : 0u;
}

/* Is this a capture somebody asked for by hand?
 *
 * 🔴 Manual captures are never evicted. The one the operator takes before the
 * TSL comes out is a manual capture, and it cannot be retaken.
 *
 * Without this, retention deleted it FIRST. Two things stacked up:
 *  - eviction picks the smallest <ms>, and the manual capture is taken before
 *    the teardown, so it is the oldest by construction;
 *  - unplugging the connector during teardown provokes bus-off, which arms
 *    automatic captures. Five of those and the manual one is gone, silently.
 *
 * <ms> is millis(), which also resets to 0 on reboot — so a capture taken after
 * a reboot looks OLDER than one taken before it, and eviction order inverts.
 * Pinning manual captures sidesteps that for the file that matters; the
 * automatic ones are diagnostic and losing the wrong one is survivable.
 */
static bool bb_is_manual(const char* base) {
    const char* u = strrchr(base, '_');
    return u && strcmp(u + 1, "manual") == 0;
}

// Keep only the newest BLACKBOX_RETAIN events (smallest <ms> deleted first).
// Manual captures are counted but never chosen for deletion; when only manual
// ones remain, oldest_base stays empty and the loop exits.
static void bb_enforce_retention() {
    for (;;) {
        File dir = BB_FS.open(BLACKBOX_DIR);
        if (!dir) return;
        uint32_t count = 0, oldest = 0xFFFFFFFFu;
        char oldest_base[40] = {};
        for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
            const char* nm = e.name();
            if (strstr(nm, ".json")) {
                char base[40];
                bb_basename(nm, base, sizeof(base));
                uint32_t s = bb_seq_from_name(base);
                count++;
                if (!bb_is_manual(base) && s < oldest) {
                    oldest = s; strncpy(oldest_base, base, sizeof(oldest_base) - 1);
                }
            }
            e.close();
        }
        dir.close();
        if (count <= BLACKBOX_RETAIN || oldest_base[0] == '\0') return;
        char p[64];
        snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.log", oldest_base);  BB_FS.remove(p);
        snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.json", oldest_base); BB_FS.remove(p);
    }
}

// Live directory scan → number of .json events. Only ever called from the
// save/delete paths (which already touch the FS) and once at init; never from
// the status/aux poll — that reads the g_event_count cache.
/* Highest <ms> among stored captures. See g_name_base. */
static uint32_t bb_scan_max_seq() {
    if (!g_fs_ok) return 0;
    File dir = BB_FS.open(BLACKBOX_DIR);
    if (!dir) return 0;
    uint32_t hi = 0;
    for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
        if (strstr(e.name(), ".json")) {
            char base[40];
            bb_basename(e.name(), base, sizeof(base));
            uint32_t s = bb_seq_from_name(base);
            if (s > hi) hi = s;
        }
        e.close();
    }
    dir.close();
    return hi;
}

static int bb_scan_count() {
    if (!g_fs_ok) return 0;
    File dir = BB_FS.open(BLACKBOX_DIR);
    if (!dir) return 0;
    int n = 0;
    for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
        if (strstr(e.name(), ".json")) n++;
        e.close();
    }
    dir.close();
    return n;
}

#if defined(BLACKBOX_BACKEND_LITTLEFS)
/* "Has this board ever mounted its filesystem?" — and the answer has to live
 * OUTSIDE that filesystem, or a format erases the very fact that says a format
 * would be destructive. Hence NVS.
 *
 * A blank board genuinely needs one format. A board that has mounted before and
 * suddenly cannot is a different situation entirely: the likely causes are a
 * power cut mid-write (this module is on a switched feed that dies with the car)
 * or transient corruption, and the stored data at that moment may include the
 * one-shot capture taken before the TSL came out — which cannot be retaken. */
/* 세 가지 답을 돌려준다. bool 두 값으로는 부족하다는 것이 이 함수의 요점이다.
 *
 * 🔴 처음에는 begin() 실패를 그냥 false("마운트한 적 없다")로 반환했다. 그러면
 * NVS 자체가 이상할 때 "새 보드" 로 읽혀 포맷이 허용된다 — 즉 판단 근거를 잃은
 * 상황에서 가장 파괴적인 쪽을 고른다. 가드의 목적과 정반대다. */
enum class FsHistory { NEVER_MOUNTED, MOUNTED_BEFORE, UNKNOWN };

static FsHistory fs_history(void) {
    Preferences p;
    // 읽기 전용으로 못 열리는 경우가 둘이다: 네임스페이스가 아직 없거나(정말
    // 첫 부팅), NVS 가 상한 것. 쓰기 모드로 한 번 더 물어 그 둘을 가른다 —
    // 쓰기로도 안 열리면 NVS 쪽 문제이므로 아무것도 단정하지 않는다.
    if (!p.begin("fsd", /*readOnly=*/true)) {
        if (!p.begin("fsd", /*readOnly=*/false)) return FsHistory::UNKNOWN;
        bool has = p.isKey("fsmnt");
        bool v   = has && p.getBool("fsmnt", false);
        p.end();
        return v ? FsHistory::MOUNTED_BEFORE : FsHistory::NEVER_MOUNTED;
    }
    bool has = p.isKey("fsmnt");
    bool v   = has && p.getBool("fsmnt", false);
    p.end();
    return v ? FsHistory::MOUNTED_BEFORE : FsHistory::NEVER_MOUNTED;
}

static void fs_mark_mounted(void) {
    Preferences p;
    if (!p.begin("fsd", /*readOnly=*/false)) return;
    if (!p.getBool("fsmnt", false)) p.putBool("fsmnt", true);
    p.end();
}
#endif

static void backend_init() {
#if defined(BLACKBOX_BACKEND_LITTLEFS)
    g_fs_ok = LittleFS.begin(false);
    if (!g_fs_ok) {
        // One retry before concluding anything. A mount can fail transiently and
        // the cost of asking twice is a few milliseconds.
        delay(50);
        g_fs_ok = LittleFS.begin(false);
    }
    if (!g_fs_ok) {
        // 🔴 begin(true) formats, and it used to run on ANY mount failure. That
        // erases the camera database, the learning file and every stored capture.
        // Formatting is now allowed exactly once in a board's life: when it has
        // never successfully mounted, i.e. it really is blank.
        const FsHistory hist = fs_history();
        if (hist != FsHistory::NEVER_MOUNTED) {
            Serial.println("[BB] 🔴 LittleFS mount FAILED — NOT formatting.");
            Serial.println(hist == FsHistory::MOUNTED_BEFORE
                ? "[BB] This board has mounted before; stored captures would be lost."
                : "[BB] NVS is unreadable, so we cannot tell whether data exists. "
                  "Refusing to guess.");
            Serial.println("[BB] Blackbox and the camera database are disabled this boot.");
            Serial.println("[BB] If the data is expendable: erase flash over USB and reflash.");
            return;
        }
        // 🔴 여기까지 와도 "정말 빈 보드" 라고 확신할 수는 없다. fsmnt 키는 이
        // 가드와 함께 생겼으므로, 그 이전 펌웨어에서 올라온 보드는 캡처를 갖고도
        // NEVER_MOUNTED 로 보인다. 다만 그 보드는 **직전 부팅에서 마운트에
        // 성공했을** 것이고, 그러면 아래에서 키가 심어져 다음부터는 보호된다.
        // 지금 이 자리는 "마운트도 실패했고 이력도 없다" 이므로 빈 보드일
        // 가능성이 가장 높다 — 포맷하되, 무엇을 하는지 크게 남긴다.
        Serial.println("[BB] 🔴 LittleFS mount failed and no mount history — treating as a blank board.");
        Serial.println("[BB] FORMATTING. If this board should have had data, stop and reflash "
                       "before taking any capture.");
        g_fs_ok = LittleFS.begin(true);
    }
    if (!g_fs_ok) { Serial.println("[BB] LittleFS mount failed"); return; }
    fs_mark_mounted();   // from here on, a failure is a reason to stop, not format
#else
    g_fs_ok = SD.cardType() != CARD_NONE;  // can_dump_init() already mounted SD
#endif
    if (g_fs_ok && !BB_FS.exists(BLACKBOX_DIR)) BB_FS.mkdir(BLACKBOX_DIR);
    g_event_count = g_fs_ok ? bb_scan_count() : 0;  // one boot-time scan
    /* +1 so the next capture strictly outranks the highest stored one, even if
     * it is taken in the same millisecond of uptime as that one was. */
    g_name_base = g_fs_ok ? bb_scan_max_seq() + 1u : 0u;
    Serial.printf("[BB] backend=%s ok=%d events=%d\n",
                  BLACKBOX_BACKEND_NAME, g_fs_ok, g_event_count);
}

// Persist one event. `frame_count`/per-bus counts already computed by the caller;
// `write` is invoked to stream candump lines into the open .log.
/* Bytes a stored frame costs on disk. MEASURED, not estimated.
 *
 * 2026-08-17: a freshly formatted 3.5 MB partition, one capture of 39,999
 * frames, 536 KB left afterwards -> ~75 bytes per frame once the candump line,
 * the .json summary and LittleFS overhead are all counted. The line itself
 * ("(0.000000) can1 3C2#0102030405060708\n") is only ~45 of that.
 *
 * 80 rounds that UP on purpose: this number only ever gates "do we have room",
 * and refusing one capture too early is much cheaper than corrupting the
 * filesystem. An estimate that is too LOW is the dangerous direction -- it lets
 * a write start that cannot finish. */
#define BB_BYTES_PER_LINE 80u

/* Never fill the last of the partition. LittleFS needs room to manoeuvre and
 * its behaviour at the very edge is what bit us. */
#define BB_STORE_MARGIN   (64u * 1024u)

static uint32_t bb_free_bytes() {
#if defined(BLACKBOX_BACKEND_LITTLEFS)
    size_t total = BB_FS.totalBytes(), used = BB_FS.usedBytes();
    return (total > used) ? (uint32_t)(total - used) : 0u;
#else
    return 0xFFFFFFFFu;   // SD: not our problem to police
#endif
}

/* Would this capture fit? Answering NO is a feature.
 *
 * 🔴 Even a 10 s window on a busy bus is thousands of frames, so a capture is
 * still ~1 MB against a 3.5 MB partition -- a few of them fill it. And
 * retention cannot rescue us: bb_enforce_retention() deliberately never deletes
 * a MANUAL capture, because the one-shot capture taken before the TSL comes out
 * must not be evicted by a later automatic one. Correct, and it means that once
 * manual captures fill the disk, nothing is deletable.
 *
 * Writing anyway is not a soft failure. Running LittleFS out of room while it
 * is mid-write leaves the filesystem inconsistent, and after that EVERY capture
 * fails -- observed 2026-08-17, when the allocator started dividing by zero and
 * panicked the board. Refusing early keeps what is already saved. */
/* Delete evictable captures, oldest first, until `need` bytes are free.
 *
 * 🔴 Retention is a COUNT rule (keep the newest BLACKBOX_RETAIN), so it does
 * nothing about a disk filled by three large automatic captures -- and it only
 * ran AFTER a successful write anyway. The preflight therefore refused new
 * captures while gigabytes of evictable diagnostic data sat there. The capture
 * that gets refused is whichever one comes next, and in the session before the
 * TSL comes out that is the one that cannot be retaken.
 *
 * Manual captures are never touched, for the reason bb_is_manual() gives. When
 * only manual ones are left this returns false and the caller refuses, which is
 * the same answer as before -- just for a real reason. */
static bool bb_make_room(uint32_t need) {
    for (;;) {
        if (bb_free_bytes() >= need) return true;
        File dir = BB_FS.open(BLACKBOX_DIR);
        if (!dir) return false;
        uint32_t oldest = 0xFFFFFFFFu;
        char oldest_base[40] = {};
        for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
            const char* nm = e.name();
            if (strstr(nm, ".json")) {
                char base[40];
                bb_basename(nm, base, sizeof(base));
                uint32_t sq = bb_seq_from_name(base);
                if (!bb_is_manual(base) && sq < oldest) {
                    oldest = sq;
                    strncpy(oldest_base, base, sizeof(oldest_base) - 1);
                }
            }
            e.close();
        }
        dir.close();
        if (oldest_base[0] == '\0') return false;   // nothing left we may delete
        char p[64];
        snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.log", oldest_base);  BB_FS.remove(p);
        snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.json", oldest_base); BB_FS.remove(p);
        Serial.printf("[BB] 자리를 비우려고 %s 를 지웠다 (자동 캡처)\n", oldest_base);
    }
}

static bool bb_store_fits(uint32_t frames, size_t json_len, const char* base) {
    uint32_t need = frames * BB_BYTES_PER_LINE + (uint32_t)json_len + BB_STORE_MARGIN;
    uint32_t have = bb_free_bytes();
    if (have >= need) return true;
    if (bb_make_room(need)) return true;   // evictable data first, refusal second
    Serial.printf("[BB] 🔴 저장 거부 %s — 약 %lu KB 필요한데 %lu KB 남았다\n",
                  base, (unsigned long)(need / 1024u), (unsigned long)(have / 1024u));
    Serial.println("[BB]    캡처를 폰/USB 로 먼저 빼낸 뒤 지우고 다시 시도한다.");
    Serial.println("[BB]    (수동 캡처는 자동으로 지워지지 않는다 — 일부러 그렇게 뒀다)");
    return false;
}

/* 무장하기 **전에** 이 캡처가 들어갈 자리가 있는지 어림한다.
 *
 * bb_store_fits() 는 flush 때, 즉 5초 뒤에야 불린다. 그때 거부하면 앱은 이미
 * "기록했습니다" 를 받은 뒤다 — 그 창이 위 g_manual_* 주석의 사고다. 여기서
 * 먼저 보면 사장님이 **그 자리에서** 알고 지우고 다시 찍을 수 있다.
 *
 * 어림하는 법: pre-roll 창에 이미 들어와 있는 프레임 수를 세고 **두 배**로
 * 본다(post-roll 이 같은 속도로 온다고 가정). 링 크기를 넘지 않는다.
 *
 * ⚠️ 어림이지 보장이 아니다. 그래서 이것만으로 끝내지 않고 g_manual_* 이
 *    뒤를 받친다 — 어림이 빗나가 flush 가 실패해도 엉뚱한 파일은 안 나간다. */
static uint32_t bb_estimated_need(uint32_t now_ms) {
    uint32_t lo = (now_ms >= BLACKBOX_PRE_MS) ? now_ms - BLACKBOX_PRE_MS : 0u;
    uint32_t pre = 0;
    for (uint32_t i = g_tail; i != g_head; i = ring_next(i)) {
        if (g_ring[i].ts_ms >= lo) pre++;
    }
    uint32_t est = (pre > (0xFFFFFFFFu / 2u)) ? g_cap : pre * 2u;
    if (est > g_cap) est = g_cap;
    return est * BB_BYTES_PER_LINE + BB_STORE_MARGIN + 1024u;   /* + .json */
}

/* Size of a stored file, read back from the filesystem. Deliberately reopens:
 * the point is to learn what LANDED, which a write() return value cannot say. */
static bool bb_size_of(const char* path, size_t* out) {
    File f = BB_OPEN_R(path);
    if (!f) return false;
    if (out) *out = f.size();
    f.close();
    return true;
}

/* Returns true only when BOTH halves of the capture are on flash and the
 * right size. `expect_frames` is how many frames the emitter was given, so
 * an empty .log can be told apart from a lost one. */
static bool backend_store(const char* base, const char* json,
                          uint32_t expect_frames,
                          void (*emit)(File&)) {
    if (!g_fs_ok) return false;
    char lp[64], jp[64];
    snprintf(lp, sizeof(lp), BLACKBOX_DIR "/%s.log", base);
    snprintf(jp, sizeof(jp), BLACKBOX_DIR "/%s.json", base);

    bool log_ok = false, json_ok = false;
    const size_t json_len = strlen(json);

    File lf = BB_OPEN_W(lp);
    if (lf) { emit(lf); lf.flush(); lf.close(); log_ok = true; }
    else Serial.printf("[BB] .log open failed: %s\n", lp);

    File jf = BB_OPEN_W(jp);
    if (jf) {
        json_ok = jf.write((const uint8_t*)json, json_len) == json_len;
        jf.flush();
        jf.close();
    } else Serial.printf("[BB] .json open failed: %s\n", jp);

    /* 🔴 Ask the FILESYSTEM what landed, not the write call.
     *
     * Every one of open, write and flush could fail and the capture was
     * still counted and still logged as "flushed". That is the worst shape
     * a failure can take here: the operator reads a success line, pulls the
     * TSL out, and finds the truncated file at home -- and the capture
     * cannot be retaken. Reopening for size costs one directory lookup and
     * catches short writes and lost flushes that the return values do not.
     *
     * An empty .log is only a fault when there were frames to write; a
     * capture of a silent bus is legitimately empty. */
    size_t on_disk = 0;
    if (json_ok) json_ok = bb_size_of(jp, &on_disk) && on_disk == json_len;
    if (log_ok && expect_frames > 0)
        log_ok = bb_size_of(lp, &on_disk) && on_disk > 0;

    if (!json_ok) {
        /* The .json is what "latest" and the event list key on, so a bad one
         * does not just lose this capture -- it hides the good ones behind
         * it. Remove the pair. Captures stored EARLIER are untouched. */
        BB_FS.remove(lp);
        BB_FS.remove(jp);
        Serial.printf("[BB] 🔴 %s 저장 실패 — 이 캡처는 남지 않았다\n", base);
    } else if (!log_ok) {
        /* Keep it: a short .log is still worth downloading, and its .json
         * names it. Say so loudly instead of counting it as a success. */
        Serial.printf("[BB] 🔴 %s 의 .log 가 불완전하다 — 저장 공간을 확인하라\n", base);
    }

    bb_enforce_retention();
    // A new capture was just written and retention may have dropped others, so
    // whatever is buffered describes a filesystem that no longer exists.
    BB_RC_DROP();
    g_event_count = bb_scan_count();  // reflects the new event + any retention drop
    return log_ok && json_ok;
}

// Status/poll path: return the cache, never scan (see g_event_count).
static int backend_count() { return g_fs_ok ? g_event_count : 0; }

static String backend_list_json() {
    String out = "[";
    if (!g_fs_ok) { out += "]"; return out; }
    File dir = BB_FS.open(BLACKBOX_DIR);
    if (!dir) { out += "]"; return out; }
    bool first = true;
    for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
        const char* nm = e.name();
        if (strstr(nm, ".json")) {
            char base[40];
            bb_basename(nm, base, sizeof(base));
            String summary;
            while (e.available()) summary += (char)e.read();
            if (!first) out += ',';
            first = false;
            out += "{\"name\":\""; out += base; out += "\",\"summary\":";
            out += summary.length() ? summary : String("null");
            out += '}';
        }
        e.close();
    }
    dir.close();
    out += "]";
    return out;
}

static bool backend_size(const char* name, bool json, size_t* out) {
    if (!g_fs_ok) return false;
    char p[64];
    snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.%s", name, json ? "json" : "log");
    File f = BB_OPEN_R(p);
    if (!f) return false;
    if (out) *out = f.size();
    f.close();
    return true;
}

#if !defined(FSD_NO_WIFI)
static void backend_stream_body(WiFiClient& client, const char* name, bool json) {
    if (!g_fs_ok) return;
    char p[64];
    snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.%s", name, json ? "json" : "log");
    File f = BB_OPEN_R(p);
    if (!f) return;
    uint8_t buf[512];
    while (f.available()) {
        int n = f.read(buf, sizeof(buf));
        if (n <= 0) break;
        client.write(buf, n);
    }
    f.close();
}
#endif

/* Read one chunk, serving from a buffered block so the file work amortises.
 *
 * ── What this used to be, and what was actually wrong ───────────────────────
 * It opened, seeked, read `cap` bytes, and closed — every chunk. Over BLE a
 * 1,320,604 B capture took 1 min 21 s (13.9 KB/s). The comment here blamed
 * LittleFS seek "walking a CTZ skip-list from the start of the file, so it gets
 * SLOWER the further the transfer goes", and a fix aimed at that (hold the File
 * open across chunks) broke downloads outright and was reverted.
 *
 * 🔴 That explanation was never measured, and it is wrong. Timed on the bench
 * 2026-08-17 (`bbread`), one 500 B chunk of an 811 KB capture:
 *
 *      offset        total     open      seek     read    close
 *      0          16.0 ms   10.6 ms    4.4 ms   791 us   103 us
 *      202,843    17.7 ms   10.6 ms    6.8 ms    20 us   102 us
 *      405,686    17.5 ms   10.7 ms    6.6 ms    19 us    98 us
 *      608,529    17.3 ms   10.6 ms    6.4 ms    21 us   104 us
 *
 * Seek does not grow with offset — 6.4 ms at 600 KB, 6.8 ms at 200 KB. And the
 * read is 20 us: reading bytes was never the cost. It is open (61%) and seek
 * (37%), both paid per call, which capped the whole path at 27 KB/s before the
 * radio got a say.
 *
 * ── The fix ─────────────────────────────────────────────────────────────────
 * Read a whole block per open and hand out chunks from RAM. One open+seek now
 * covers BB_READAHEAD/cap chunks (~8 at the BLE payload size), so the amortised
 * cost drops to ~2 ms and the bottleneck moves to the radio, which is where it
 * belongs.
 *
 * Deliberately NOT a cached file handle. Nothing here outlives the call, so
 * there is no handle to go stale against a delete, a retention sweep, or an
 * unmount — the failure mode that made the previous attempt unshippable. A
 * stale *buffer* is only wrong bytes, and fsd_rc_serve() refuses to serve
 * across an identity change; the invalidation calls below close the rest.
 *
 * This path is the ONLY way a capture leaves the module (the web dashboard that
 * was the other one went in PR #28) and the TSL capture cannot be taken twice,
 * so verify with `bbread` — which exercises this function with no radio in the
 * way — before trusting a change here. */
static size_t backend_read_chunk(const char* name, bool json, size_t offset,
                                 uint8_t* out, size_t cap) {
    if (!g_fs_ok || !out || cap == 0) return 0;

#if BB_READAHEAD > 0
    size_t idx = 0;
    size_t n = fsd_rc_serve(&g_rc, name, json, offset, cap, &idx);
    if (n) { memcpy(out, g_rc_buf + idx, n); return n; }
#endif

    // Miss — refill the block from exactly where the caller asked, so a
    // sequential walk hits on every chunk after the first of each block, and a
    // random seek costs no more than the old code did.
    char p[64];
    snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.%s", name, json ? "json" : "log");
    File f = BB_OPEN_R(p);
    if (!f) { BB_RC_DROP(); return 0; }
    size_t got = 0;
#if BB_READAHEAD > 0
    if (f.seek(offset)) {
        int r = f.read(g_rc_buf, sizeof(g_rc_buf));
        if (r > 0) got = (size_t)r;
    }
    f.close();

    fsd_rc_fill(&g_rc, name, json, offset, got);   // got==0 resets, not fills
    n = fsd_rc_serve(&g_rc, name, json, offset, cap, &idx);
    if (!n) return 0;
    memcpy(out, g_rc_buf + idx, n);
    return n;
#else
    // No read-ahead on this board: straight into the caller's buffer.
    if (f.seek(offset)) {
        int r = f.read(out, cap);
        if (r > 0) got = (size_t)r;
    }
    f.close();
    return got;
#endif
}

bool blackbox_read_phases(const char* name, bool json, size_t offset,
                          uint32_t* open_us, uint32_t* seek_us,
                          uint32_t* read_us, uint32_t* close_us) {
    if (!g_fs_ok || !name) return false;
    // Stack, not static: this is a diagnostic and the DRAM it would cost is the
    // difference between esp32-lilygo linking and not.
    uint8_t scratch[256];
    char p[64];
    snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.%s", name, json ? "json" : "log");

    uint32_t a = micros();
    File f = BB_OPEN_R(p);
    uint32_t t_open = micros() - a;
    if (!f) return false;

    a = micros();
    bool ok = f.seek(offset);
    uint32_t t_seek = micros() - a;

    a = micros();
    if (ok) (void)f.read(scratch, sizeof(scratch));
    uint32_t t_read = micros() - a;

    a = micros();
    f.close();
    uint32_t t_close = micros() - a;

    if (open_us)  *open_us  = t_open;
    if (seek_us)  *seek_us  = t_seek;
    if (read_us)  *read_us  = t_read;
    if (close_us) *close_us = t_close;
    return true;
}

// Newest = last entry the directory hands back. Capture names are timestamped,
// so listing order tracks age closely enough for "give me the one I just took".
/* 🔴 "latest" has to mean newest, not last-listed.
 *
 * This used to keep whatever the directory walk happened to hand over last.
 * LittleFS does not promise creation order, so on a board with more than one
 * stored capture the BLE download — which is the ONLY way to get a capture off
 * this module, and has no "list" or "fetch by name" command — could serve an
 * older one while reporting success. The operator sees a completed download,
 * removes the TSL, and only later finds the file is the wrong capture.
 *
 * Pick the largest <ms> instead. Same caveat as retention: millis() resets on
 * reboot, so "newest" is only ordered within one power cycle. That is the case
 * that matters here (mark, then download in the same session), and the manual
 * capture is pinned against eviction anyway.
 */
static bool backend_latest_name(char* out, size_t cap) {
    if (!g_fs_ok || !out || cap == 0) return false;
    File dir = BB_FS.open(BLACKBOX_DIR);
    if (!dir) return false;
    bool found = false;
    uint32_t newest = 0;
    for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
        const char* nm = e.name();
        if (strstr(nm, ".json")) {
            char base[40];
            bb_basename(nm, base, sizeof(base));
            uint32_t s = bb_seq_from_name(base);
            if (!found || s >= newest) {
                newest = s;
                snprintf(out, cap, "%s", base);
                found = true;
            }
        }
        e.close();
    }
    dir.close();
    return found;
}

static bool backend_delete(const char* name) {
    if (!g_fs_ok) return false;
    char p[64];
    bool ok = false;
    snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.log", name);  ok |= BB_FS.remove(p);
    snprintf(p, sizeof(p), BLACKBOX_DIR "/%s.json", name); ok |= BB_FS.remove(p);
    // The read-ahead block may hold bytes of the file we just removed. Dropping
    // it unconditionally (not only on ok) is the cautious order: the cost is one
    // extra open, the cost of getting it wrong is serving a deleted capture.
    BB_RC_DROP();
    if (ok) g_event_count = bb_scan_count();
    return ok;
}

static void backend_delete_all() {
    if (!g_fs_ok) return;
    for (;;) {
        File dir = BB_FS.open(BLACKBOX_DIR);
        if (!dir) return;
        char victim[40] = {};
        for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
            if (strstr(e.name(), ".json")) { bb_basename(e.name(), victim, sizeof(victim)); e.close(); break; }
            e.close();
        }
        dir.close();
        if (victim[0] == '\0') return;
        backend_delete(victim);
    }
}

static bool backend_is_volatile() { return false; }

#else  // ── RAM backend (volatile, download-only) ──────────────────────────────

static bool      g_slot_used = false;
static char      g_slot_base[40] = {};
static String    g_slot_json;
static BBFrame*  g_slot_frames = nullptr;
static uint32_t  g_slot_n = 0;
static uint32_t  g_slot_window_start = 0;

static void backend_init() {
    Serial.println("[BB] backend=ram (volatile, dashboard download only)");
}

static void backend_store_ram(const char* base, const char* json,
                              const BBFrame* frames, uint32_t n, uint32_t window_start) {
    if (g_slot_frames) { free(g_slot_frames); g_slot_frames = nullptr; }
    g_slot_used = false;
    g_slot_frames = (BBFrame*)malloc((size_t)n * sizeof(BBFrame));
    if (!g_slot_frames) { Serial.println("[BB] RAM event alloc failed"); return; }
    memcpy(g_slot_frames, frames, (size_t)n * sizeof(BBFrame));
    g_slot_n = n;
    g_slot_window_start = window_start;
    g_slot_json = json;
    strncpy(g_slot_base, base, sizeof(g_slot_base) - 1);
    g_slot_used = true;
}

static int backend_count() { return g_slot_used ? 1 : 0; }

static String backend_list_json() {
    String out = "[";
    if (g_slot_used) {
        out += "{\"name\":\""; out += g_slot_base; out += "\",\"summary\":";
        out += g_slot_json.length() ? g_slot_json : String("null");
        out += '}';
    }
    out += "]";
    return out;
}

static bool backend_size(const char* name, bool json, size_t* out) {
    if (!g_slot_used || strcmp(name, g_slot_base) != 0) return false;
    if (json) { if (out) *out = g_slot_json.length(); return true; }
    // .log: sum the formatted candump line lengths (no large RAM text copy).
    size_t total = 0;
    char line[72];
    for (uint32_t i = 0; i < g_slot_n; i++) {
        const BBFrame& f = g_slot_frames[i];
        total += tesla_format_candump_line(line, sizeof(line), f.ts_ms - g_slot_window_start,
                                           bb_iface_name(f),
                                           f.id, f.data, f.dlc);
    }
    if (out) *out = total;
    return true;
}

#if !defined(FSD_NO_WIFI)
static void backend_stream_body(WiFiClient& client, const char* name, bool json) {
    if (!g_slot_used || strcmp(name, g_slot_base) != 0) return;
    if (json) { client.print(g_slot_json); return; }
    char line[72];
    for (uint32_t i = 0; i < g_slot_n; i++) {
        const BBFrame& f = g_slot_frames[i];
        int n = tesla_format_candump_line(line, sizeof(line), f.ts_ms - g_slot_window_start,
                                          bb_iface_name(f),
                                          f.id, f.data, f.dlc);
        client.write((const uint8_t*)line, n);
    }
}
#endif

static size_t backend_read_chunk(const char* name, bool json, size_t offset,
                                 uint8_t* out, size_t cap) {
    if (!g_slot_used || strcmp(name, g_slot_base) != 0 || !out || cap == 0) return 0;
    if (json) {
        size_t len = g_slot_json.length();
        if (offset >= len) return 0;
        size_t n = len - offset;
        if (n > cap) n = cap;
        memcpy(out, g_slot_json.c_str() + offset, n);
        return n;
    }
    // The .log has no stored text — it is regenerated line by line, so seeking
    // means re-formatting from the start and discarding up to `offset`. O(n)
    // per chunk, but the ring is ~100 KB and this only runs during a download.
    char line[72];
    size_t pos = 0, written = 0;
    for (uint32_t i = 0; i < g_slot_n && written < cap; i++) {
        const BBFrame& f = g_slot_frames[i];
        int n = tesla_format_candump_line(line, sizeof(line), f.ts_ms - g_slot_window_start,
                                          bb_iface_name(f),
                                          f.id, f.data, f.dlc);
        if (n <= 0) continue;
        if (pos + (size_t)n > offset) {
            size_t skip = (offset > pos) ? offset - pos : 0;
            size_t take = (size_t)n - skip;
            if (take > cap - written) take = cap - written;
            memcpy(out + written, line + skip, take);
            written += take;
        }
        pos += (size_t)n;
    }
    return written;
}

static bool backend_latest_name(char* out, size_t cap) {
    if (!g_slot_used || !out || cap == 0) return false;
    snprintf(out, cap, "%s", g_slot_base);
    return true;
}

static bool backend_delete(const char* name) {
    if (!g_slot_used || strcmp(name, g_slot_base) != 0) return false;
    if (g_slot_frames) { free(g_slot_frames); g_slot_frames = nullptr; }
    g_slot_used = false;
    g_slot_json = String();
    return true;
}

static void backend_delete_all() { if (g_slot_used) backend_delete(g_slot_base); }

static bool backend_is_volatile() { return true; }

// No file exists on this backend -- the capture is a RAM slot, so there is no
// open/seek/read/close to time. Declared in blackbox.h for every board, so it
// needs a definition here too or the RAM boards fail to link.
bool blackbox_read_phases(const char*, bool, size_t,
                          uint32_t*, uint32_t*, uint32_t*, uint32_t*) {
    return false;
}

#endif  // backend

// ─────────────────────────────────────────────────────────────────────────────
//  Common: ring + trigger + flush
// ─────────────────────────────────────────────────────────────────────────────
// Keep this much heap/PSRAM free after the ring so the WiFi/web stack always
// has a working margin. The ~108 KB S3 ring grabbed at boot could starve WiFi
// → OOM reboot loop (#124); the guard makes an on-demand enable refuse rather
// than brick connectivity.
#define BLACKBOX_HEAP_MARGIN  (60u * 1024u)

// Try to allocate the ring in the requested memory, but only if enough stays
// free afterwards. Returns true and publishes g_ring/g_cap on success.
static bool bb_try_alloc(bool psram, uint32_t frames) {
    size_t bytes = (size_t)frames * sizeof(BBFrame);
    size_t freeb = psram ? ESP.getFreePsram() : ESP.getFreeHeap();
    if (freeb < bytes + BLACKBOX_HEAP_MARGIN) {
        Serial.printf("[BB] ring alloc skipped (%s): free %luB < need %luB + %luB margin\n",
                      psram ? "psram" : "heap", (unsigned long)freeb,
                      (unsigned long)bytes, (unsigned long)BLACKBOX_HEAP_MARGIN);
        return false;
    }
    BBFrame* p = (BBFrame*)(psram ? ps_malloc(bytes) : malloc(bytes));
    if (!p) { Serial.printf("[BB] ring alloc failed (%s)\n", psram ? "psram" : "heap"); return false; }
    g_ring = p;
    g_cap = frames;
    g_ring_psram = psram;
    g_head = g_tail = 0;
    Serial.printf("[BB] ring=%lu frames (%lu KB) %s  window=%us pre/%us post\n",
                  (unsigned long)g_cap, (unsigned long)(bytes / 1024u),
                  g_ring_psram ? "PSRAM" : "internal-RAM",
                  BLACKBOX_PRE_MS / 1000u, BLACKBOX_POST_MS / 1000u);
    return true;
}

// Lazily allocate the ring (on enable). PSRAM first when present, else internal
// RAM — each attempt is heap-guarded. Returns true once g_ring is ready.
static bool blackbox_alloc_ring() {
    if (g_ring) return true;
    if (g_want_psram && bb_try_alloc(true, BLACKBOX_FRAMES_PSRAM)) return true;
    return bb_try_alloc(false, BLACKBOX_FRAMES_INTERNAL);
}

// Free the ring and drop any pre-roll / armed capture (on disable).
static void blackbox_free_ring() {
    if (g_ring) { free(g_ring); g_ring = nullptr; }
    g_cap = 0;
    g_head = g_tail = 0;
    g_armed = false;
}

void blackbox_init(FSDState* state, portMUX_TYPE* state_mux) {
    g_state = state;
    g_mux   = state_mux;

    // Size decision only — the ring is allocated lazily on enable so boot never
    // competes with the WiFi/web stack for heap (#124).
    g_want_psram = ESP.getPsramSize() > 0;
    g_ring = nullptr;
    g_cap = 0;
    g_ring_psram = false;

    backend_init();
    Serial.printf("[BB] init backend=%s — ring allocated on enable "
                  "(%s target %lu frames)\n",
                  BLACKBOX_BACKEND_NAME,
                  g_want_psram ? "PSRAM" : "internal-RAM",
                  (unsigned long)(g_want_psram ? BLACKBOX_FRAMES_PSRAM
                                               : BLACKBOX_FRAMES_INTERNAL));
}

// Common ring-write path shared by the RX and TX entry points. `is_tx` tags the
// stored frame; everything else — id filter, eviction, memcpy — is identical, so
// TX frames pass the SAME key-id filter as RX and can't flood the ring with the
// high-rate nag echo.
static void bb_record(CanBusId bus, const CanFrame& frame, uint32_t now_ms, uint8_t is_tx) {
    if (g_cap == 0 || g_state == nullptr || !g_state->blackbox_enabled) return;
    // Store only the key diagnostic IDs (fsd_blackbox_filter.h). On a busy full
    // bus (~3300 f/s) recording everything fills the ring in ~1.8 s, truncating
    // the 5 s pre / 5 s post window; the filter drops the stored rate ~15x so
    // the whole window survives. Define BLACKBOX_CAPTURE_ALL to keep every frame.
    if (!fsd_blackbox_should_record(frame.id)) return;
    if (ring_next(g_head) == g_tail) g_tail = ring_next(g_tail);  // evict oldest
    BBFrame& s = g_ring[g_head];
    s.ts_ms = now_ms;
    s.id    = frame.id;
    s.bus   = (uint8_t)bus;
    s.dlc   = frame.dlc > 8 ? 8 : frame.dlc;
    s.is_tx = is_tx;
    memcpy(s.data, frame.data, s.dlc);
    if (s.dlc < 8) memset(s.data + s.dlc, 0, 8 - s.dlc);
    g_head = ring_next(g_head);
}

void blackbox_record(CanBusId bus, const CanFrame& frame, uint32_t now_ms) {
    bb_record(bus, frame, now_ms, 0);   // RX (bus) frame
}

void blackbox_record_tx(CanBusId bus, const CanFrame& frame, uint32_t now_ms) {
    bb_record(bus, frame, now_ms, 1);   // TX (our injected/echoed) frame
}

void blackbox_note_ap_state(uint8_t ap_state, uint32_t now_ms) {
    if (ap_state == g_tl_last) return;
    g_tl_last = ap_state;
    g_tl_ts[g_tl_head] = now_ms;
    g_tl_state[g_tl_head] = ap_state;
    g_tl_head = (g_tl_head + 1u) % BLACKBOX_TL_MAX;
    if (g_tl_count < BLACKBOX_TL_MAX) g_tl_count++;
}

void blackbox_arm(BBTrigger trig, const FSDState* snap, uint32_t now_ms) {
    if (g_cap == 0 || g_state == nullptr || !g_state->blackbox_enabled) return;
    if (g_armed) return;  // already capturing; ignore until the post-roll flushes
    g_armed = true;
    if (trig == BB_TRIG_MANUAL) g_manual_armed_n++;   // 짝은 do_flush 에서 맞춘다
    g_trig = trig;
    g_trig_ms = now_ms;
    g_flush_at_ms = now_ms + BLACKBOX_POST_MS;
    if (snap) g_snap = *snap; else g_snap = *g_state;
    // Both halves of the window, not just the post-roll. At the car the pre-roll
    // is the number the operator needs: it is how long they had between finishing
    // the action and pressing `mark`, and nothing later in the flow reveals it.
    Serial.printf("[BB] armed by %s @ %lums — window %ums pre / %ums post\n",
                  trig_name_uc(trig), (unsigned long)now_ms,
                  BLACKBOX_PRE_MS, BLACKBOX_POST_MS);
}

void blackbox_busoff(uint32_t now_ms) {
    if (g_state == nullptr) return;
    FSDState snap;
    bb_enter();
    FSDEventType e = fsd_events_inject(g_state, EVT_BUSOFF, now_ms);
    snap = *g_state;
    bb_exit();
    if (e == EVT_BUSOFF) blackbox_arm(BB_TRIG_BUSOFF, &snap, now_ms);
}

uint32_t blackbox_mark(uint32_t now_ms) {
    if (g_state == nullptr) return 0;
#if defined(BLACKBOX_BACKEND_LITTLEFS) || defined(BLACKBOX_BACKEND_SD)
    /* 🔴 자리가 없으면 **무장하지 않는다** (2026-08-31 레드팀 ①).
     *
     * 예전에는 그냥 무장하고 5초 뒤 flush 에서 거부했다. 그 사이 앱은
     * "기록했습니다" 를 받고 기다렸다가 **직전 캡처**를 성공으로 받아 갔다.
     * 여기서 거부하면 사장님이 그 자리에서 알고 지우고 다시 찍는다. */
    if (g_cap != 0 && g_ring != nullptr) {
        uint32_t need = bb_estimated_need(now_ms);
        uint32_t have = bb_free_bytes();
        if (have < need) {
            Serial.printf("[BB] 🔴 기록하지 않았다 — 약 %lu KB 필요한데 %lu KB 남았다\n",
                          (unsigned long)(need / 1024u), (unsigned long)(have / 1024u));
            Serial.println("[BB]    받아낸 뒤 'bbclear yes' 로 지우고 다시 찍는다.");
            return 0;   /* 0 = 무장되지 않았다. BLE 는 REJECTED 로 답한다 */
        }
    }
#endif
    FSDState snap;
    bb_enter();
    FSDEventType e = fsd_events_inject(g_state, EVT_MANUAL, now_ms);
    snap = *g_state;
    bb_exit();
    if (e == EVT_MANUAL) blackbox_arm(BB_TRIG_MANUAL, &snap, now_ms);
    else Serial.println("[BB] mark suppressed (cooldown)");

    /* 🔴 무엇이 실제로 일어났는지 돌려준다.
     *
     * 예전에는 void 였고, BLE 는 "성공 + (이전 개수 + 1)" 을 무조건 답했다.
     * 둘 다 예측이지 사실이 아니다:
     *   - cooldown 이 삼키면 아무것도 생기지 않는데도 성공이라고 답했다;
     *   - 캡처는 post-roll 이 끝나야 파일이 되므로, 답한 그 순간에는 개수가
     *     아직 이전 값이다. 그 사이 DUMP_START 를 보내면 latest 는 **직전
     *     캡처**를 준다 — 그리고 화면에는 다운로드 성공이라고 뜬다.
     *
     * 반환값 = 파일이 생기기까지 남은 ms (0 이면 아무것도 무장되지 않았다).
     * 호출자는 이것으로 "지금 받으면 엉뚱한 것을 받는다" 를 알 수 있다. */
    if (!g_armed) return 0;
    uint32_t left = (uint32_t)(g_flush_at_ms - now_ms);
    return left ? left : 1u;   // armed 인데 0 을 돌려주지 않는다
}

// Build the .json summary from g_snap + the windowed frame/bus counts + timeline.
static void build_summary(char* out, int out_sz, uint32_t frame_count,
                          uint32_t window_start, uint32_t bus0, uint32_t bus1) {
    uint32_t tl_ts[BLACKBOX_TL_EMIT];
    uint8_t  tl_state[BLACKBOX_TL_EMIT];
    int tlc = 0;
    uint32_t lo = (g_trig_ms >= BLACKBOX_PRE_MS) ? g_trig_ms - BLACKBOX_PRE_MS : 0u;
    uint32_t hi = g_trig_ms + BLACKBOX_POST_MS;
    // Walk the timeline ring oldest→newest, keep entries inside the window.
    uint32_t start = (g_tl_count < BLACKBOX_TL_MAX) ? 0u
                     : g_tl_head;  // wrapped: oldest is at head
    for (uint32_t k = 0; k < g_tl_count && tlc < BLACKBOX_TL_EMIT; k++) {
        uint32_t idx = (start + k) % BLACKBOX_TL_MAX;
        uint32_t ts = g_tl_ts[idx];
        if (ts < lo || ts > hi) continue;
        // window_start is the first frame in-window, which can post-date an
        // entry (or the trigger) on a sparse/idle bus — clamp so the rel-ms
        // never underflows to ~2^32.
        tl_ts[tlc] = (ts >= window_start) ? ts - window_start : 0u;
        tl_state[tlc] = g_tl_state[idx];
        tlc++;
    }

    FSDBlackboxSummary s;
    memset(&s, 0, sizeof(s));
    s.trigger        = trig_name_uc(g_trig);
    s.from_state     = g_snap.evt_last_from;
    s.to_state       = g_snap.evt_last_to;
    // When the window's first frame post-dates the trigger (idle bus, trigger
    // fires before any post-roll frame lands) window_start > g_trig_ms; guard
    // the unsigned subtraction so t= reads 0 instead of underflowing to ~2^32.
    s.trigger_rel_ms = (g_trig_ms >= window_start) ? g_trig_ms - window_start : 0u;
    s.window_pre_ms  = BLACKBOX_PRE_MS;
    s.window_post_ms = BLACKBOX_POST_MS;
    s.frame_count    = frame_count;
    s.hw_version     = (int)g_snap.hw_version;
    s.hw4_das_status_seen = g_snap.das_hw4_status_seen;
#if defined(CAN_DRIVER_T2CAN_DUAL)
    s.dual_can       = true;
#else
    s.dual_can       = false;
#endif
    s.bus0_frames    = bus0;
    s.bus1_frames    = bus1;
    s.nag            = g_snap.nag_killer;
    s.ap_first       = g_snap.ap_first;
    s.abort_guard    = g_snap.abort_guard;
    s.signal_map     = (g_snap.cfg_das_id != 0);
    s.nag_burst      = g_snap.nag_burst;
    s.tl_ts          = tl_ts;
    s.tl_state       = tl_state;
    s.tl_count       = tlc;
    fsd_blackbox_format_json(out, out_sz, &s);
}

#if defined(BLACKBOX_BACKEND_LITTLEFS) || defined(BLACKBOX_BACKEND_SD)
// Disk emit: walk the ring window and stream candump lines into the open file.
static uint32_t g_emit_lo, g_emit_hi, g_emit_start;
static void disk_emit(File& f) {
    char line[72];
    bool started = false;
    for (uint32_t i = g_tail; i != g_head; i = ring_next(i)) {
        const BBFrame& fr = g_ring[i];
        if (fr.ts_ms < g_emit_lo) continue;
        if (fr.ts_ms > g_emit_hi) break;
        if (!started) { g_emit_start = fr.ts_ms; started = true; }
        int n = tesla_format_candump_line(line, sizeof(line), fr.ts_ms - g_emit_start,
                                          bb_iface_name(fr),
                                          fr.id, fr.data, fr.dlc);
        f.write((const uint8_t*)line, n);
    }
}
#endif

static void do_flush() {
    uint32_t lo = (g_trig_ms >= BLACKBOX_PRE_MS) ? g_trig_ms - BLACKBOX_PRE_MS : 0u;
    uint32_t hi = g_trig_ms + BLACKBOX_POST_MS;

    // Pass 1: window start + per-bus counts + injected (TX) frame count.
    uint32_t count = 0, bus0 = 0, bus1 = 0, window_start = 0, txc = 0;
    bool started = false;
    for (uint32_t i = g_tail; i != g_head; i = ring_next(i)) {
        const BBFrame& fr = g_ring[i];
        if (fr.ts_ms < lo) continue;
        if (fr.ts_ms > hi) break;
        if (!started) { window_start = fr.ts_ms; started = true; }
        count++;
        if (fr.is_tx) txc++;
        if (fr.bus == CAN_BUS_SECONDARY) bus1++; else bus0++;
    }
    if (!started) window_start = g_trig_ms;

    char base[40];
    /* g_name_base, not a bare millis(): see the note on g_name_base. Without it
     * a capture taken after a reboot sorts BELOW one taken before it, and
     * "download the latest" hands over the wrong file while reporting success. */
    snprintf(base, sizeof(base), "evt_%lu_%s",
             (unsigned long)(g_name_base + g_trig_ms), trig_name(g_trig));

    char json[640];
    build_summary(json, sizeof(json), count, window_start, bus0, bus1);

    // The shared summary formatter (fsd_logic, pure) is a per-event summary, not
    // a per-frame record list, so direction is reported here as a count: splice a
    // "tx_frames" field in before the closing brace so a decoded summary tells you
    // how many of the frames were ours. Per-frame direction lives in the .log via
    // the can0TX/can1TX interface tag.
    {
        size_t jl = strlen(json);
        if (jl > 0 && json[jl - 1] == '}' && jl + 24 < sizeof(json))
            snprintf(json + jl - 1, sizeof(json) - (jl - 1),
                     ",\"tx_frames\":%lu}", (unsigned long)txc);
    }

#if defined(BLACKBOX_BACKEND_LITTLEFS) || defined(BLACKBOX_BACKEND_SD)
    if (!bb_store_fits(count, strlen(json), base)) return;   // nothing written
    g_emit_lo = lo; g_emit_hi = hi;
    const bool stored = backend_store(base, json, count, disk_emit);
#else
    // RAM: freeze the windowed frames into a contiguous buffer for download.
    BBFrame* frozen = (BBFrame*)malloc((size_t)(count ? count : 1) * sizeof(BBFrame));
    uint32_t fn = 0;
    if (frozen) {
        for (uint32_t i = g_tail; i != g_head && fn < count; i = ring_next(i)) {
            const BBFrame& fr = g_ring[i];
            if (fr.ts_ms < lo) continue;
            if (fr.ts_ms > hi) break;
            frozen[fn++] = fr;
        }
        backend_store_ram(base, json, frozen, fn, window_start);
        free(frozen);
    } else {
        Serial.println("[BB] flush alloc failed");
    }
    const bool stored = (frozen != nullptr);
#endif
    /* 🔴 Count and announce a capture only when it is actually on flash.
     * "flushed" used to print unconditionally -- including after an open that
     * failed -- so a lost capture and a stored one produced the same line. */
    if (!stored) {
        Serial.printf("[BB] 🔴 %s 를 저장하지 못했다 — 다시 캡처해야 한다\n", base);
        return;
    }
    g_captures++;
    /* 파일이 실제로 생겼을 때만 짝을 맞춘다. 위 early return 들은
     * 여기에 닿지 못하므로 어긋난 채로 남는다 — 그게 신호다. */
    if (g_trig == BB_TRIG_MANUAL) g_manual_saved_n = g_manual_armed_n;
    Serial.printf("[BB] flushed %s  frames=%lu (can0=%lu can1=%lu)\n",
                  base, (unsigned long)count, (unsigned long)bus0, (unsigned long)bus1);
#if defined(BLACKBOX_BACKEND_LITTLEFS)
    /* Say how much room is left, every time. Without this the operator only
     * finds out at the moment a capture is refused -- which in the car is the
     * one moment there is no time to deal with it. */
    {
        uint32_t have = bb_free_bytes();
        uint32_t per  = (count ? count : 1u) * BB_BYTES_PER_LINE;
        Serial.printf("[BB] 남은 공간 %lu KB — 이만한 캡처 약 %lu 개분\n",
                      (unsigned long)(have / 1024u),
                      (unsigned long)(per ? (have / per) : 0u));
    }
#endif
}

void blackbox_tick(uint32_t now_ms) {
    if (!g_armed) return;
    if ((int32_t)(now_ms - g_flush_at_ms) < 0) return;  // post-roll still running
    do_flush();
    g_armed = false;
}

void blackbox_set_enabled(bool enabled) {
    if (g_state == nullptr) return;
    if (enabled) {
        // Allocate the ring on demand, heap-guarded. If it can't be had without
        // starving WiFi/web, stay disabled and report it — never brick the link.
        if (!blackbox_alloc_ring()) {
            bb_enter();
            g_state->blackbox_enabled = false;
            bb_exit();
            Serial.println("[BB] enable refused — not enough free heap; staying off");
            return;
        }
        bb_enter();
        g_state->blackbox_enabled = true;
        bb_exit();
        Serial.println("[BB] enabled");
    } else {
        bb_enter();
        g_state->blackbox_enabled = false;
        bb_exit();
        blackbox_free_ring();  // stop + drop pre-roll + release the ring
        Serial.println("[BB] disabled");
    }
}

// Reports true only when the ring is actually live, so a guard-refused enable
// (or a persisted-ON boot before reconcile) shows as off on the dashboard.
uint32_t blackbox_capture_count() { return g_captures; }

bool blackbox_is_enabled() {
    return g_state != nullptr && g_state->blackbox_enabled && g_ring != nullptr;
}

String blackbox_status_json() {
    String j = "{";
    j += "\"enabled\":";  j += blackbox_is_enabled() ? "true" : "false"; j += ',';
    j += "\"backend\":\""; j += BLACKBOX_BACKEND_NAME; j += "\",";
    j += "\"volatile\":"; j += backend_is_volatile() ? "true" : "false"; j += ',';
    j += "\"psram\":";    j += g_ring_psram ? "true" : "false"; j += ',';
    j += "\"cap\":";      j += g_cap; j += ',';
    j += "\"armed\":";    j += g_armed ? "true" : "false"; j += ',';
    j += "\"events\":";   j += backend_count(); j += ',';
    j += "\"captures\":"; j += g_captures;
    j += '}';
    return j;
}

String blackbox_list_json() { return backend_list_json(); }

bool blackbox_file_size(const char* name, bool json, size_t* size_out) {
    if (!bb_name_ok(name)) return false;
    return backend_size(name, json, size_out);
}

#if !defined(FSD_NO_WIFI)
void blackbox_stream_body(WiFiClient& client, const char* name, bool json) {
    if (!bb_name_ok(name)) return;
    backend_stream_body(client, name, json);
}
#endif

size_t blackbox_read_chunk(const char* name, bool json, size_t offset,
                           uint8_t* out, size_t cap) {
    if (!bb_name_ok(name)) return 0;
    return backend_read_chunk(name, json, offset, out, cap);
}

// Spans arm -> post-roll -> flush, because blackbox_tick() clears g_armed only
// after do_flush() returns. That is exactly the window in which "the latest
// capture" is still the previous one.
bool blackbox_capture_pending() { return g_armed; }

/* 마지막 수동 mark 가 파일이 되지 못했나. ble_bulk_start() 가 이걸 보고
 * **직전 캡처를 내주지 않는다** — 그것이 성공으로 보이는 것이 사고다. */
bool blackbox_last_mark_lost() {
    return !g_armed && (g_manual_armed_n != g_manual_saved_n);
}

bool blackbox_latest_name(char* out, size_t cap) {
    return backend_latest_name(out, cap);
}

bool blackbox_delete(const char* name) {
    if (!bb_name_ok(name)) return false;
    return backend_delete(name);
}

void blackbox_delete_all() { backend_delete_all(); }

uint32_t blackbox_free_bytes() {
    // 🔴 bb_free_bytes() only exists inside the LittleFS/SD section. The RAM
    // backend has no filesystem to run out of — the ring IS the storage — so
    // there is nothing to police and 0xFFFFFFFF ("plenty") is the honest answer.
    //
    // Broke five boards in CI when this called bb_free_bytes() unguarded
    // (2026-08-17). The board we use builds LittleFS, so a local build said
    // nothing.
#if defined(BLACKBOX_BACKEND_LITTLEFS) || defined(BLACKBOX_BACKEND_SD)
    return bb_free_bytes();
#else
    return 0xFFFFFFFFu;
#endif
}

int blackbox_event_count() { return backend_count(); }

bool blackbox_storage_ok() {
#if defined(BLACKBOX_BACKEND_LITTLEFS) || defined(BLACKBOX_BACKEND_SD)
    return g_fs_ok;
#else
    // RAM backend: the ring is the storage, so there is nothing that can fail
    // to mount. Reporting false here would make the OTA self-test unpassable.
    return true;
#endif
}
