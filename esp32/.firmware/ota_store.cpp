#include "ota_store.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

#include "../../fsd_logic/fsd_ota_gate.h"
#include "../../fsd_logic/fsd_ota_image.h"
#include "ble_owner.h"
#include "fsd_build_stamp.h"

/*
 * 🔴 받는 동안의 상태는 **정적이 아니라 힙**에 둔다. 그리고 그것을 이 파일이
 * 배운 방식이 기록해 둘 값어치가 있다:
 *
 * 처음에는 평범한 `static` 이었다. 여덟 보드를 구우니 `esp32-lilygo` 하나가
 * **DRAM 을 112 바이트 넘겨** 링크에서 터졌다 — 이 파일을 아무도 안 부르는
 * 보드에서. 링커는 안 쓰는 함수는 버리지만 그 함수들이 쥔 `static` 은 자리에
 * 남겼다. 즉 **BLE 가 없는 일곱 보드가 쓰지도 않을 240 바이트를 내고 있었다.**
 *
 * 조건부 컴파일로 막을 수도 있었지만 그러면 일곱 보드가 이 파일을 **아예 안
 * 컴파일**하게 되고, 이 저장소는 그 자리에서 이미 다쳤다(*"호스트만 컴파일하는
 * 파일은 나중에 더 나쁜 때에 깨진다"*). 힙으로 옮기면 둘 다 얻는다 — 여덟 보드가
 * 전부 컴파일하고, 정적 자리는 8 바이트다.
 *
 * 🔴 `MALLOC_CAP_INTERNAL` 인 것도 이유가 있다. PSRAM 은 플래시 캐시를 통해
 * 닿는데 플래시를 쓰는 동안 그 캐시가 꺼진다. OTA 상태가 거기 있으면 쓰는 도중에
 * 자기 자신을 못 읽는다.
 */
typedef struct {
    esp_ota_handle_t handle;
    const esp_partition_t *slot;
    FsdOtaXfer xfer;
    FsdOtaScan scan;
    uint32_t last_ms;
} OtaSession;

static SemaphoreHandle_t g_mux = nullptr;
static OtaSession *g_s = nullptr;

static inline bool lock(void) {
    if (!g_mux) return false;
    return xSemaphoreTake(g_mux, portMAX_DELAY) == pdTRUE;
}
static inline void unlock(void) {
    if (g_mux) xSemaphoreGive(g_mux);
}

/* 자리를 놓아 준다. 이미 잠근 채로 부른다.
 *
 * 🔴 조용하다 — 성공한 설치도 여기를 지나간다. 여기서 "접었다" 고 찍으면
 * 성공이 화면에서 실패처럼 읽힌다. */
static void free_locked(void) {
    if (!g_s) return;
    if (g_s->handle != 0) esp_ota_abort(g_s->handle);
    heap_caps_free(g_s);
    g_s = nullptr;
}

/* 접는다 — 이유를 말하고 놓아 준다. */
static void drop_locked(const char *why) {
    if (!g_s) return;
    Serial.printf("[OTA] 받던 것을 접었다 — %s\n", why ? why : "이유 없음");
    free_locked();
}

void ota_store_init(void) {
    if (!g_mux) g_mux = xSemaphoreCreateMutex();
}

/*
 * 우리가 지금 돌고 있는 이미지의 칩 번호.
 *
 * 🔴 상수로 적지 않는 이유: 적으면 갈라진다. 여덟 보드 중 일곱이 원래 ESP32(0)
 * 이고 우리만 S3(9) 인데, 그 값을 이 파일에 박아 두면 다른 보드에서 조용히 틀린
 * 답을 낸다. **돌고 있는 판에게 물어보는 것**이 언제나 맞고, 질문 자체가 정확히
 * 알고 싶은 것이다 — *"들어온 이미지가 나와 같은 칩용인가"*.
 */
static uint16_t running_chip_id(void) {
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (!run) return 0xFFFFu; /* 모르면 아무 이미지와도 안 맞는다 — 닫히는 쪽 */
    uint8_t head[FSD_OTA_HEAD_MIN];
    if (esp_partition_read(run, 0, head, sizeof(head)) != ESP_OK) return 0xFFFFu;
    return (uint16_t)((uint16_t)head[FSD_OTA_CHIP_OFFSET] |
                      ((uint16_t)head[FSD_OTA_CHIP_OFFSET + 1u] << 8));
}

uint8_t ota_store_begin(uint32_t total_bytes, bool motion_seen, bool saving_capture) {
    if (!lock()) return OTA_ST_ESP_BEGIN;

    const esp_partition_t *slot = esp_ota_get_next_update_partition(nullptr);

    FsdOtaBeginIn in;
    memset(&in, 0, sizeof(in));
    in.transfer_running = (g_s != nullptr);
    in.saving_capture = saving_capture;
    in.motion_seen = motion_seen;
    in.owner_window_open = ble_owner_window_open();
    in.slot_bytes = slot ? (uint32_t)slot->size : 0u;
    in.declared_bytes = total_bytes;

    const FsdOtaBeginVerdict v = fsd_ota_begin_check(&in);
    if (v != FSD_OTA_BEGIN_OK) {
        Serial.printf("[OTA] 시작 거절 — %s\n", fsd_ota_begin_verdict_str(v));
        unlock();
        return (uint8_t)(OTA_ST_BEGIN_BASE + (uint8_t)v);
    }

    OtaSession *s =
        (OtaSession *)heap_caps_malloc(sizeof(OtaSession), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s) {
        Serial.println("[OTA] 받을 자리를 못 잡았다 (내부 RAM)");
        unlock();
        return OTA_ST_ESP_BEGIN;
    }
    memset(s, 0, sizeof(*s));

    /* 🔴 실제 크기를 준다. `OTA_SIZE_UNKNOWN` 을 주면 ESP-IDF 가 **칸 전체**
     * (6.25 MB)를 지운다 — 필요한 것의 아홉 배이고 그 시간이 전부 블로킹이다. */
    Serial.printf("[OTA] %s 을 지운다 (%u 바이트)…\n", slot->label, (unsigned)total_bytes);
    const uint32_t t0 = millis();
    const esp_err_t err = esp_ota_begin(slot, (size_t)total_bytes, &s->handle);
    if (err != ESP_OK) {
        Serial.printf("[OTA] esp_ota_begin 실패: %s\n", esp_err_to_name(err));
        heap_caps_free(s);
        unlock();
        return OTA_ST_ESP_BEGIN;
    }

    s->slot = slot;
    s->last_ms = millis();
    fsd_ota_xfer_init(&s->xfer, total_bytes);
    fsd_ota_scan_init(&s->scan);
    g_s = s;
    Serial.printf("[OTA] 준비됐다 — %s, 지우는 데 %u ms\n", slot->label,
                  (unsigned)(s->last_ms - t0));
    unlock();
    return OTA_ST_OK;
}

uint8_t ota_store_chunk(uint16_t seq, const uint8_t *data, size_t n) {
    if (!lock()) return OTA_ST_NOT_RUNNING;
    if (!g_s) {
        unlock();
        return OTA_ST_NOT_RUNNING;
    }

    const FsdOtaChunkVerdict cv = fsd_ota_xfer_take(&g_s->xfer, seq, (uint32_t)n);
    if (cv != FSD_OTA_CHUNK_OK) {
        /* 🔴 빈 조각은 접지 않는다. 알맹이 없는 프레임 하나로 13 초짜리 전송을
         * 죽일 이유가 없고, 순번도 안 먹으므로 다음 진짜 조각이 그대로 이어진다.
         * 나머지 둘(순서·넘침)은 이미지에 구멍이 났다는 뜻이라 접는다. */
        if (cv != FSD_OTA_CHUNK_EMPTY) {
            Serial.printf("[OTA] 조각 거절 — %s (seq %u)\n", fsd_ota_chunk_verdict_str(cv),
                          (unsigned)seq);
            drop_locked(fsd_ota_chunk_verdict_str(cv));
        }
        unlock();
        return (uint8_t)(OTA_ST_CHUNK_BASE + (uint8_t)cv);
    }

    const esp_err_t err = esp_ota_write(g_s->handle, data, n);
    if (err != ESP_OK) {
        Serial.printf("[OTA] esp_ota_write 실패: %s\n", esp_err_to_name(err));
        drop_locked("플래시에 못 썼다");
        unlock();
        return OTA_ST_ESP_WRITE;
    }

    /* 표식 찾기는 쓰기와 **같은 바이트**를 본다. 따로 읽지 않는 이유는, 따로
     * 읽으면 "검사한 것" 과 "구운 것" 이 다를 수 있기 때문이다. */
    fsd_ota_scan_feed(&g_s->scan, data, (uint32_t)n);
    g_s->last_ms = millis();

    unlock();
    return OTA_ST_OK;
}

bool ota_store_ready_to_finish(void) {
    if (!lock()) return false;
    const bool done = g_s && fsd_ota_xfer_complete(&g_s->xfer);
    unlock();
    return done;
}

uint8_t ota_store_finish(void) {
    if (!lock()) return OTA_ST_NOT_RUNNING;
    if (!g_s) {
        unlock();
        return OTA_ST_NOT_RUNNING;
    }

    /* 🔴 먼저 esp_ota_end(). 여기서 ESP-IDF 가 이미지의 해시와 구조를 본다 —
     * 우리 표식 검사는 "누가 구웠나" 이지 "안 깨졌나" 가 아니다. 둘 다 필요하고,
     * 깨진 이미지를 우리 판정에 먼저 먹이는 것은 순서가 거꾸로다. */
    const esp_err_t eerr = esp_ota_end(g_s->handle);
    g_s->handle = 0; /* end 가 성공하든 실패하든 핸들은 소비됐다 — abort 금지 */
    if (eerr != ESP_OK) {
        Serial.printf("[OTA] esp_ota_end 실패: %s\n", esp_err_to_name(eerr));
        drop_locked("이미지가 깨졌다");
        unlock();
        return OTA_ST_ESP_END;
    }

    /* 🔴 여기가 1층의 계약이 말하는 자리 — **전부 받은 뒤에** 판정을 믿는다.
     * 보드 이름은 상수가 아니라 **지금 돌고 있는 판 자신의 표식**에서 온다. */
    const FsdOtaImgVerdict iv =
        fsd_ota_scan_verdict(&g_s->scan, FSD_OTA_MARK.board, running_chip_id());
    if (iv != FSD_OTA_IMG_OK) {
        Serial.printf("[OTA] 이미지 거절 — %s\n", fsd_ota_img_verdict_str(iv));
        drop_locked(fsd_ota_img_verdict_str(iv));
        unlock();
        return (uint8_t)(OTA_ST_IMG_BASE + (uint8_t)iv);
    }

    const esp_err_t berr = esp_ota_set_boot_partition(g_s->slot);
    if (berr != ESP_OK) {
        Serial.printf("[OTA] 부팅 칸을 못 바꿨다: %s\n", esp_err_to_name(berr));
        drop_locked("부팅 칸을 못 바꿨다");
        unlock();
        return OTA_ST_ESP_BOOT;
    }

    Serial.printf("[OTA] 심었다 — %s · %s · %s\n", g_s->slot->label, g_s->scan.mark.board,
                  g_s->scan.mark.stamp);
    Serial.println("[OTA] 다시 켜면 이 판으로 뜬다. 자가진단을 통과해야 확정된다.");
    free_locked();
    unlock();
    return OTA_ST_OK;
}

bool ota_store_tick(uint32_t now_ms) {
    if (!lock()) return false;
    bool cut = false;
    if (g_s && (uint32_t)(now_ms - g_s->last_ms) > FSD_OTA_STALL_MS) {
        /* 🔴 시한이 없으면 폰이 그냥 사라졌을 때 칸이 영원히 잡혀 있고, 다음
         * 시도가 BUSY 로 거절된다 — 그 증상은 차 옆에서 "왜 안 되지" 로만 보인다. */
        drop_locked("조각이 15초째 안 온다");
        cut = true;
    }
    unlock();
    return cut;
}

bool ota_store_busy(void) {
    if (!lock()) return false;
    const bool b = (g_s != nullptr);
    unlock();
    return b;
}

uint32_t ota_store_progress(void) {
    if (!lock()) return 0;
    const uint32_t w = g_s ? g_s->xfer.written : 0u;
    unlock();
    return w;
}

void ota_store_abandon(const char *why) {
    if (!lock()) return;
    drop_locked(why);
    unlock();
}
