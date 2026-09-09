#!/usr/bin/env python3
"""표식이 정말 이미지 안에 있는가 — 빌드마다 CI 가 묻는다.

    python ci_check_ota_mark.py <pioenv>

🔴 왜 검사가 필요한가
─────────────────────────────────────────────────────────────────────────────
표식은 평범한 `const` 구조체다. 아무도 안 읽으면 링커가 버릴 수 있고, 버려진
이미지는 **보드가 자기 자신을 거부한다** — 폰으로 그 판을 보내면
"우리가 구운 파일이 아니다" 가 나온다.

그 증상은 차 옆에서 원인을 짚을 수 없는 종류다. 배선도 아니고 앱도 아니고
링커다. 그래서 여기서 막는다.

지금은 main.cpp 의 부팅 배너가 표식을 읽어 주므로 버려지지 않는다. 그런데
그 한 줄은 **누군가 배너를 정리하다 지울 수 있는 줄**이고, 지운 사람은 그것이
OTA 를 끊는다는 것을 모른다. 이 검사가 그때 빨개진다.

🔴 그리고 **보드 이름이 맞는지**도 본다. 이름이 틀린 표식은 있으나 마나가
아니라 더 나쁘다 — 다른 보드용 이미지를 통과시키거나, 자기 것을 거부한다.
lilygo-t2can 과 waveshare-s3-can 은 **칩 id 가 같아서**(둘 다 ESP32-S3 = 9)
칩 검사로는 안 갈린다. 갈리는 것은 이 이름뿐이다.
"""
import os
import sys

# 🔴 출력 인코딩을 먼저 못 박는다. 이 저장소는 같은 자리에서 이미 한 번
# 물렸다 — ci_check_no_wifi.py 의 줄표 하나가 Windows 콘솔(cp949)에서
# **성공 경로에서만** 터져서, 통과해야 할 검사가 예외로 죽었다. 실패 메시지는
# 사람이 읽어야 하는 것이라 ASCII 로 깎는 대신 출력을 UTF-8 로 고정한다.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# fsd_logic/fsd_ota_image.h 와 같아야 하는 값들. 하나로 합칠 수 없어서(하나는
# 파이썬, 하나는 C) 헤더에서 읽어 온다 — 적어 두면 갈라진다.
HDR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fsd_logic",
                   "fsd_ota_image.h")


def _hdr_int(text, name):
    tok = "#define %s " % name
    at = text.find(tok)
    if at < 0:
        raise SystemExit("[ota-mark] fsd_ota_image.h 에 %s 가 없다" % name)
    val = text[at + len(tok):text.index("\n", at)].strip()
    # "'F'" 또는 "0x01u" 또는 "24u"
    if val.startswith("'"):
        return ord(val[1])
    return int(val.split()[0].rstrip("u"), 0)


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    env = sys.argv[1]

    with open(HDR, encoding="utf-8") as f:
        hdr = f.read()

    magic = bytes(_hdr_int(hdr, "FSD_OTA_MARK_MAGIC_%d" % i) for i in range(8))
    board_len = _hdr_int(hdr, "FSD_OTA_MARK_BOARD_LEN")
    stamp_len = _hdr_int(hdr, "FSD_OTA_MARK_STAMP_LEN")

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".pio", "build", env,
                        "firmware.bin")
    # 🔴 파일이 없으면 통과가 아니라 실패다. 빌드 폴더가 비었는데 초록이면
    #    이 검사는 아무것도 안 지킨다 — ci_check_no_wifi.py 가 같은 이유로
    #    같은 규칙을 쓴다.
    if not os.path.isfile(path):
        print("[ota-mark] 🔴 %s 가 없다 — 먼저 빌드해야 한다" % path)
        return 1

    with open(path, "rb") as f:
        img = f.read()

    n = img.count(magic)
    if n == 0:
        print("[ota-mark] 🔴 %s: 표식이 이미지에 없다.\n"
              "           링커가 FSD_OTA_MARK 를 버린 것이다 — main.cpp 의 배너가\n"
              "           그것을 읽는 줄이 아직 있는지 확인하라. 이대로 나가면\n"
              "           보드가 자기 이미지를 OTA 로 거부한다." % env)
        return 1
    if n != 1:
        print("[ota-mark] 🔴 %s: 표식이 %d 개다. 하나여야 한다." % (env, n))
        return 1

    at = img.index(magic)
    board = img[at + 8:at + 8 + board_len].split(b"\x00")[0].decode("utf-8", "replace")
    stamp = img[at + 8 + board_len:at + 8 + board_len + stamp_len].split(b"\x00")[0].decode(
        "utf-8", "replace")

    if board != env:
        print("[ota-mark] 🔴 %s: 표식의 보드 이름이 %r 이다 — env 와 달라야 할 이유가 없다.\n"
              "           틀린 이름은 없는 것보다 나쁘다: 다른 보드용을 통과시키거나\n"
              "           자기 것을 거부한다." % (env, board))
        return 1

    if not stamp:
        print("[ota-mark] 🔴 %s: 판번호가 비었다." % env)
        return 1

    print("[ota-mark] 🟢 %s: 0x%06X 에 표식 하나 · board=%s · %s" % (env, at, board, stamp))
    return 0


if __name__ == "__main__":
    sys.exit(main())
