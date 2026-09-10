#pragma once
/*
 * fsd_build_stamp.h — when this image was built, and from what.
 *
 * The definition lives in fsd_build_stamp.cpp, which esp32/gen_build_stamp.py
 * REGENERATES before every build. That file is not committed; if it is missing
 * the link fails loudly, which is the right way to notice that the pre-build
 * script did not run.
 *
 * Format: "YYYY-MM-DD HH:MM:SS <rev>", where <rev> is the short git revision
 * with "-dirty" appended when the tree had uncommitted changes, or "nogit"
 * when the build did not happen inside a git checkout.
 *
 * 🔴 Do NOT go back to __DATE__ / __TIME__ here. Those freeze when the
 * translation unit that uses them compiles, so a build that touched some other
 * file leaves the banner reporting an older date than the image — measured on
 * 2026-08-23, and it costs a re-flash to discover.
 */

#include "../../fsd_logic/fsd_ota_image.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const char FSD_BUILD_STAMP[];

/* 이미지 안에 심는 표식 — 폰이 보낸 펌웨어가 **이 보드의 것인가**를 받는 쪽이
 * 판정할 수 있게 하는 유일한 근거다.
 *
 * 🔴 왜 esp_app_desc_t 가 아닌가: 우리 이미지의 그 칸은 프레임워크가 미리
 * 구워 둔 값이다 — project_name 이 'arduino-lib-builder' 이고, **모든**
 * Arduino-ESP32 이미지가 같은 값을 낸다. 그것으로 검사하면 검사처럼 보이면서
 * 아무것도 안 거른다 (2026-09-10 실측, fsd_ota_image.h 참조).
 *
 * 🔴 배너가 이 값을 읽는 것이 그냥 친절이 아니다. 아무도 안 읽으면 링커가
 * 이 상수를 버릴 수 있고, 그러면 이미지에 표식이 없어 **자기가 자기를 거부**
 * 한다. 부팅 배너에 보드 이름이 찍히는 것이 그 증거이기도 하다. */
extern const FsdOtaMarkWire FSD_OTA_MARK;

#ifdef __cplusplus
}
#endif
