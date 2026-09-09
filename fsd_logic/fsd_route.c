/* fsd_route.c — 앱이 준 경로. 판단 근거는 fsd_route.h 머리말에 있다. */
#include "fsd_route.h"

#include <string.h>

#include "fsd_camera.h" // fsd_cam_segment_distance_m

void fsd_route_init(FsdRoute* r) {
    if(!r) return;
    memset(r, 0, sizeof(*r));
}

static int32_t le32(const uint8_t* b) {
    return (int32_t)((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) |
                     ((uint32_t)b[3] << 24));
}

bool fsd_route_feed(FsdRoute* r, uint16_t seq, uint16_t total, const uint8_t* pts,
                    size_t n, uint32_t now_ms) {
    if(!r || !pts) return false;
    if(n == 0u || (n % FSD_ROUTE_POINT_BYTES) != 0u) return false;
    if(total == 0u || total > FSD_ROUTE_MAX_POINTS) return false;

    const size_t incoming = n / FSD_ROUTE_POINT_BYTES;

    /* seq 0 은 언제나 새 출발. 이어 붙이면 목적지를 바꿨을 때 옛 길이 그대로
     * 남아 그 위의 카메라가 계속 통과한다. */
    if(seq == 0u) {
        r->count = 0u;
        r->expected = total;
    } else {
        /* 받던 것과 다른 경로의 조각이거나, 자리가 어긋나면 버린다. */
        if(r->expected != total) return false;
        if(seq != r->count) return false;
    }

    if((size_t)r->count + incoming > (size_t)total) return false;
    if((size_t)r->count + incoming > FSD_ROUTE_MAX_POINTS) return false;

    for(size_t i = 0; i < incoming; i++) {
        const uint8_t* p = pts + i * FSD_ROUTE_POINT_BYTES;
        r->lat_e7[r->count] = le32(p);
        r->lon_e7[r->count] = le32(p + 4);
        r->count++;
    }
    r->set_ms = now_ms;
    return true;
}

bool fsd_route_complete(const FsdRoute* r) {
    return r && r->expected != 0u && r->count == r->expected;
}

uint16_t fsd_route_count(const FsdRoute* r) { return r ? r->count : 0u; }

float fsd_route_distance_m(const FsdRoute* r, int32_t lat_e7, int32_t lon_e7) {
    if(!r || r->count == 0u) return -1.0f;

    /* 점이 하나뿐이면 선분이 없다 — 그 점까지의 거리다. 선분 함수에 같은 점을
     * 두 번 주면 0 으로 나누는 구현이 있을 수 있어 갈라 둔다. */
    if(r->count == 1u) {
        return fsd_cam_distance_m(r->lat_e7[0], r->lon_e7[0], lat_e7, lon_e7);
    }

    float best = -1.0f;
    for(uint16_t i = 0; i + 1u < r->count; i++) {
        const float d = fsd_cam_segment_distance_m(r->lat_e7[i], r->lon_e7[i],
                                                   r->lat_e7[i + 1u], r->lon_e7[i + 1u],
                                                   lat_e7, lon_e7);
        if(best < 0.0f || d < best) best = d;
    }
    return best;
}

bool fsd_route_rejects(const FsdRoute* r, int32_t here_lat_e7, int32_t here_lon_e7,
                       int32_t cam_lat_e7, int32_t cam_lon_e7) {
    /* ⑴ 경로가 다 와 있는가. 절반만 온 경로로 거르면 아직 안 온 절반 위의
     *    카메라가 전부 남의 길이 된다. */
    if(!fsd_route_complete(r)) return false;

    /* ⑵ 우리가 그 경로 위에 있는가. 🔴 이것이 없으면 다른 길로 접어든 순간
     *    필터가 정확히 반대로 일한다 — 지금 달리는 길의 카메라를 전부 버린다. */
    const float here = fsd_route_distance_m(r, here_lat_e7, here_lon_e7);
    if(here < 0.0f || here > FSD_ROUTE_ONROUTE_M) return false;

    /* ⑶ 카메라가 회랑 밖인가. 여기까지 와서야 버린다. */
    const float cam = fsd_route_distance_m(r, cam_lat_e7, cam_lon_e7);
    if(cam < 0.0f) return false;
    return cam > FSD_ROUTE_CORRIDOR_M;
}
