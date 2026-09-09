# gen_build_stamp.py — PlatformIO pre-build extra_script.
#
# Writes .firmware/fsd_build_stamp.cpp with the wall-clock time and the git
# revision of THIS repository, so the boot banner describes the image that is
# actually running.
#
# ── Why this exists ──────────────────────────────────────────────────────────
#
# The banner used to print __DATE__ " " __TIME__ from main.cpp. Those macros are
# frozen when main.cpp COMPILES, not when the image is built or flashed — so a
# build that only touched another translation unit kept main.cpp's old .o and
# reported a stale date. Measured 2026-08-23: blackbox.cpp changed, the image
# was rebuilt and flashed, and the board booted saying "Build: Aug 21".
#
# 🔴 That is the worst kind of wrong output: the boot banner is the FIRST thing
# read when a board misbehaves, and a stale date sends you to re-flash an image
# that was already correct. This repository has been bitten by the same shape
# before — a banner advertising a compiled-out feature (fork PR #53).
#
# ── Why a generated .cpp and not a -D flag ───────────────────────────────────
#
# A build flag would work, but build_flags are part of EVERY object's signature,
# so a stamp that changes each run would force a full rebuild of all sources on
# every build, on all eight boards. A generated file is a dependency of exactly
# one tiny translation unit: only it recompiles, then the image relinks.
#
# ── Why generated content and not `touch` ────────────────────────────────────
#
# 🔴 Touching the file is NOT enough. SCons (PlatformIO's build engine) decides
# staleness with MD5-timestamp: a newer mtime with identical content can still
# be treated as unchanged, so the rebuild would be skipped and the stamp would
# silently stay stale — the exact failure this script exists to remove. Writing
# genuinely different content makes the rebuild unconditional.
Import("env")
import os
import subprocess
import time

# SCons execs this script rather than importing it, so __file__ is NOT defined
# here -- PlatformIO hands the project root over as PROJECT_DIR instead.
FIRMWARE_DIR = os.path.join(env["PROJECT_DIR"], ".firmware")
OUT = os.path.join(FIRMWARE_DIR, "fsd_build_stamp.cpp")


def _git(*args):
    """Run a git command in this repo. Returns "" when git or the repo is absent.

    Never raises: a missing git must degrade the stamp, not break the build.
    """
    try:
        out = subprocess.check_output(
            ["git"] + list(args),
            cwd=FIRMWARE_DIR,
            stderr=subprocess.STDOUT,
        )
        return out.decode("utf-8", "replace").strip()
    except Exception:
        return ""


def _revision():
    """Short revision, with -dirty when the working tree has uncommitted edits.

    🔴 The dirty marker is the load-bearing half. A bare hash on a modified tree
    names the last COMMIT, not the code in the image — which is precisely the
    situation this banner is consulted in (a change made, built, flashed, and
    not yet committed).
    """
    rev = _git("rev-parse", "--short=8", "HEAD")
    if not rev:
        return "nogit"
    # --porcelain prints one line per changed path; any output means dirty.
    if _git("status", "--porcelain", "--untracked-files=no"):
        rev += "-dirty"
    return rev


stamp = "%s %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), _revision())

# ── OTA 표식 ────────────────────────────────────────────────────────────────
#
# 이미지 안에 우리 것임을 남긴다. 폰이 보낸 펌웨어가 **이 보드의 것인가** 를
# 받는 쪽이 판정할 근거이고, 지금 그 근거가 될 수 있는 유일한 것이다.
#
# 🔴 esp_app_desc_t 로는 안 된다. 우리 이미지의 그 칸은 프레임워크가 미리
# 구워 둔 값이라 project_name 이 'arduino-lib-builder' 이고, 모든
# Arduino-ESP32 이미지가 같은 값을 낸다 (2026-09-10 실측). 그것으로 검사하면
# 검사처럼 보이면서 아무것도 안 거른다.
#
# 🔴 길이는 fsd_ota_image.h 가 정한다. 여기서 잘리면 받는 쪽이 다른 문자열을
# 보게 되므로, 넘치면 조용히 자르지 않고 빌드를 세운다.
MARK_BOARD_LEN = 24
MARK_STAMP_LEN = 40

board = env["PIOENV"]
if len(board) > MARK_BOARD_LEN:
    raise SystemExit(
        "[stamp] 보드 이름 %r 이 %d 자를 넘는다 — fsd_ota_image.h 의 "
        "FSD_OTA_MARK_BOARD_LEN 과 함께 늘려야 한다" % (board, MARK_BOARD_LEN))
if len(stamp) > MARK_STAMP_LEN:
    raise SystemExit(
        "[stamp] 판번호 %r 이 %d 자를 넘는다 — fsd_ota_image.h 의 "
        "FSD_OTA_MARK_STAMP_LEN 과 함께 늘려야 한다" % (stamp, MARK_STAMP_LEN))


def _c_array(text, width):
    """NUL 로 채운 char 배열 리터럴. 문자열 리터럴을 안 쓰는 이유는 길이가
    딱 맞아야 하고(끝의 NUL 이 칸을 먹으면 안 된다), 컴파일러가 초기화자
    개수를 세어 주기 때문이다."""
    b = text.encode("utf-8")
    if len(b) > width:
        raise SystemExit("[stamp] %r 이 %d 바이트를 넘는다" % (text, width))
    b = b + b"\0" * (width - len(b))
    return "{" + ", ".join(str(x) for x in b) + "}"

# 🔴 fsd_ota_image.h 의 FSD_OTA_MARK_MAGIC_* 와 **같은 바이트여야 한다.**
# 둘이 갈라지면 보드가 자기 이미지를 "우리가 구운 것이 아니다" 로 거부한다.
# 한 곳으로 합칠 수 없는 이유는 하나가 파이썬이고 하나가 C 라서다 — 그래서
# 아래 self-check 가 대신 묻는다.
magic_bytes = [ord("F"), ord("S"), ord("D"), ord("-"), ord("O"), ord("T"), ord("A"), 1]

# 🔴 헤더에 적힌 매직과 실제로 같은지 여기서 확인한다. 갈라지면 증상이
# "폰이 보낸 판을 보드가 거부한다" 인데, 차 옆에서 그것을 보면 원인이 매직
# 불일치라고는 아무도 생각 못 한다.
_hdr = open(os.path.join(env["PROJECT_DIR"], "..", "fsd_logic", "fsd_ota_image.h"),
            encoding="utf-8").read()
for _i, _want in enumerate(magic_bytes):
    _tok = "#define FSD_OTA_MARK_MAGIC_%d " % _i
    _at = _hdr.find(_tok)
    if _at < 0:
        raise SystemExit("[stamp] fsd_ota_image.h 에 %s 가 없다" % _tok)
    _val = _hdr[_at + len(_tok):_hdr.index("\n", _at)].strip()
    _got = ord(_val[1]) if _val.startswith("'") else int(_val.rstrip("u"), 0)
    if _got != _want:
        raise SystemExit(
            "[stamp] 매직 바이트 %d 가 헤더(%d)와 다르다(%d) — 갈라지면 보드가 "
            "자기 이미지를 거부한다" % (_i, _got, _want))

# 🔴 `extern` on the definition is required, not decorative. In C++ a const
# array at namespace scope has INTERNAL linkage by default, so plain
# `const char FSD_BUILD_STAMP[] = ...` would be private to this translation
# unit and main.cpp would fail to link against it.

body = (
    "// GENERATED by esp32/gen_build_stamp.py before every build. Do not edit;\n"
    "// do not commit. Regenerating it is what keeps the boot banner honest.\n"
    '#include "fsd_build_stamp.h"\n'
    'extern const char FSD_BUILD_STAMP[];\n'
    'const char FSD_BUILD_STAMP[] = "%s";\n'
    "\n"
    "// OTA 표식. 이미지 안에 그대로 실려 나가고, 받는 쪽이 이것을 찾는다.\n"
    "extern const FsdOtaMarkWire FSD_OTA_MARK;\n"
    "const FsdOtaMarkWire FSD_OTA_MARK = {\n"
    "    {%s},\n"
    "    %s,\n"
    "    %s,\n"
    "};\n" % (
        stamp,
        ", ".join(str(x) for x in magic_bytes),
        _c_array(board, MARK_BOARD_LEN),
        _c_array(stamp, MARK_STAMP_LEN),
    )
)

# Write unconditionally. Identical content on two builds in the same second is
# possible and harmless; SCons then correctly skips the recompile because
# nothing actually changed.
with open(OUT, "w") as f:
    f.write(body)

print("[stamp] %s  board=%s" % (stamp, board))
