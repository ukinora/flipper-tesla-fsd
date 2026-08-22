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

#ifdef __cplusplus
extern "C" {
#endif

extern const char FSD_BUILD_STAMP[];

#ifdef __cplusplus
}
#endif
