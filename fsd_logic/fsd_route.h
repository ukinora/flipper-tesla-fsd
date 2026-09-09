/* fsd_route.h — 앱이 준 경로. "이 카메라가 우리 길 위인가" 를 가른다.
 *
 * WHY THIS EXISTS
 * ---------------
 * 추적기는 지금 위치에서 600 m 안의 카메라를 줍고 방위로 접근을 판단한다.
 * 그것이 못 하는 질문이 하나 있다 — **우리가 그 앞에서 빠지는가.** 나란한
 * 도로의 카메라도, 300 m 뒤에 좌회전으로 벗어날 길의 카메라도 똑같이 "앞에
 * 있다" 로 보인다. 경로를 알면 그 질문에 답할 수 있고, 경로를 아는 것은
 * 앱뿐이다(TMAP).
 *
 * 🔴 카메라 DB 는 여전히 모듈에 있다. 앱은 자기만 아는 것(경로)만 준다 —
 * 앱이 DB 를 따로 읽으면 같은 지식이 두 곳에 생기고, 이 저장소는 사본이
 * 갈라지는 것으로 이미 여러 번 다쳤다(열 번째 패턴).
 *
 * 🔴🔴 이것은 게이트가 아니라 덤이다. 하는 일이 **카메라를 버리는 것**이라,
 * 틀리는 방향이 둘인데 무게가 다르다:
 *
 *     잘못 버린다 → 우리 길 위의 단속카메라를 못 본다      🔴 무겁다
 *     잘못 남긴다 → 남의 길 카메라에 한 번 느려진다        🟡 가볍다
 *
 * 그래서 **확신할 때만 버린다.** 경로가 없거나 · 아직 다 안 왔거나 · 우리가
 * 그 경로 위에 있지 않으면 아무것도 버리지 않고, 그때 동작은 경로가 생기기
 * 전과 정확히 같다.
 *
 * 차주 지시 2026-09-09.
 */
#ifndef FSD_ROUTE_H
#define FSD_ROUTE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 담을 수 있는 점의 수. 8 B × 512 = 4 KB — 이 보드에는 PSRAM 8 MB 가 있다.
 *
 *  앱이 여기 맞춰 솎아서 보낸다. 서울에서 부산까지가 400 km 인데 512 점이면
 *  점 사이 780 m 이고, 회랑이 100 m 라 그 정도 성김으로도 길을 가른다. */
#define FSD_ROUTE_MAX_POINTS 512u

/** 한 점의 바이트 수 — `int32 lat_e7` + `int32 lon_e7`, 리틀엔디언. */
#define FSD_ROUTE_POINT_BYTES 8u

/** 경로에서 이만큼까지는 **같은 길**로 본다.
 *
 *  🔴 넓게 잡는 것이 안전한 쪽이다. 폰 GPS 오차가 흔히 5~15 m 이고 나쁘면
 *  50 m 다. 왕복 8차선은 40 m 쯤 되고, 앱이 솎아 보낸 폴리라인은 곡선을
 *  현으로 자르므로 그 자체로 수십 m 를 벌린다. 좁게 잡으면 맞은편 차선의
 *  카메라를 버린다. */
#define FSD_ROUTE_CORRIDOR_M 100.0f

/** 우리가 경로에서 이만큼 벗어나면 **필터를 끈다.**
 *
 *  🔴 이것이 이 파일에서 가장 중요한 숫자다. 다른 길로 접어들었는데 옛 경로로
 *  계속 거르면, 지금 달리는 길의 카메라가 **전부** 남의 길이 되어 필터가
 *  정확히 반대로 일한다. 회랑보다 넉넉히 크게 둔다 — 잠깐 벗어난 것과 아주
 *  다른 길로 간 것을 가르는 값이다. */
#define FSD_ROUTE_ONROUTE_M 250.0f

typedef struct {
    int32_t lat_e7[FSD_ROUTE_MAX_POINTS];
    int32_t lon_e7[FSD_ROUTE_MAX_POINTS];
    uint16_t count;    // 지금까지 받은 점
    uint16_t expected; // 앱이 말한 전체 점 수. 0 = 받는 중이 아니다
    uint32_t set_ms;   // 마지막 조각이 온 시각
} FsdRoute;

/** 비운다. 이 상태에서는 아무것도 버리지 않는다. */
void fsd_route_init(FsdRoute* r);

/**
 * 조각 하나를 받는다.
 *
 * `seq` 는 **이 조각의 첫 점 번호**다 — 청크 번호가 아니라 점 번호라, 조각
 * 크기가 MTU 에 따라 달라져도 붙는 자리가 흔들리지 않는다. `seq == 0` 은
 * 언제나 **새 경로의 시작**이고, 받던 것을 버린다.
 *
 * 🔴 `seq` 가 지금 자리와 다르면 그 조각을 **버린다.** BLE 는 순서를 지키지만
 * 거기 기대면, 어긋난 조각 하나가 경로를 조용히 엉뚱한 모양으로 만들고 그
 * 경로로 거른 결과는 전부 그럴듯해 보인다.
 *
 * 받아들였으면 true. 길이가 8 의 배수가 아니거나, 총 점 수가 담을 수 있는
 * 것보다 많거나, 자리가 안 맞으면 false 이고 아무것도 안 바뀐다.
 */
bool fsd_route_feed(FsdRoute* r, uint16_t seq, uint16_t total,
                    const uint8_t* pts, size_t n, uint32_t now_ms);

/** 다 왔는가. 미완인 경로로는 아무것도 거부하지 않는다. */
bool fsd_route_complete(const FsdRoute* r);

/** 지금까지 받은 점 수. */
uint16_t fsd_route_count(const FsdRoute* r);

/** 점에서 경로까지의 최단 거리(m). 경로가 없으면 음수. */
float fsd_route_distance_m(const FsdRoute* r, int32_t lat_e7, int32_t lon_e7);

/**
 * 🔴 **이 카메라를 버려도 되는가.**
 *
 * true 는 *"우리 길이 아니라고 확신한다"* 는 뜻이고, 그때만 true 다:
 *
 *   ⑴ 경로가 다 와 있고
 *   ⑵ 우리가 그 경로 위에 있고 (`FSD_ROUTE_ONROUTE_M` 안)
 *   ⑶ 카메라가 회랑 밖이다 (`FSD_ROUTE_CORRIDOR_M` 밖)
 *
 * 셋 중 하나라도 아니면 false — 즉 경로가 없던 때와 똑같이 동작한다.
 */
bool fsd_route_rejects(const FsdRoute* r, int32_t here_lat_e7, int32_t here_lon_e7,
                       int32_t cam_lat_e7, int32_t cam_lon_e7);

#ifdef __cplusplus
}
#endif

#endif // FSD_ROUTE_H
