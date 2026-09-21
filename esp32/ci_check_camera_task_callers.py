#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera_task 의 것들이 **NimBLE 콜백에서 불리지 않는가.**

    python esp32/ci_check_camera_task_callers.py

WHY THIS EXISTS
---------------
`esp32/.firmware/camera_task.h` 의 THREADING 절이 이렇게 적는다:

    nothing guards the instances in this file, and NOTHING HERE MAY EVER BE
    CALLED FROM A NimBLE CALLBACK.

그 문장 위에 `g_gps`(구조체)와 `g_route`(512점 배열)가 **잠금 없이** 서 있다.
loop() 가 CAN 프레임마다 쓰고 1초 틱에서 읽는데, 다른 태스크가 같은 메모리를
동시에 만지면 반쯤 바뀐 값으로 판정한다.

🔴 **그 문장이 2026-09-12 부터 09-22 까지 거짓이었다.** 폰의 세 입력
(ROUTE_SET · ROUTE_CLEAR · GPS_FIX)이 `CommandCB::onWrite` 에서 곧장
적용되고 있었다 — 그것은 NimBLE 호스트 태스크다. **고의였다**: ble_server.cpp
쪽에 «미루면 지연이 붙는다» 는 근거가 글로 적혀 있었다. 즉 **두 파일이 서로
반대를 말했고**, 둘 다 읽은 사람만 그것을 볼 수 있었다.

🔴 **주석은 아무것도 강제하지 않는다.** 그것이 이 파일이 있는 이유 전부다.
고친 것과 같은 커밋에 이 검사를 넣어, 다음에 누가 «한 줄만 여기서 부르면
편한데» 라고 생각하는 날 **빌드가 먼저 터지게** 한다.

WHAT IT LOOKS AT
----------------
`ble_server.cpp` 안의 NimBLE 콜백 메서드 본문을 중괄호로 잘라 내고, 그 안에
`camera_task_` 라는 글자가 있는지 본다. 문자열과 주석은 먼저 지운다 — 주석
안의 중괄호 하나가 본문 경계를 통째로 어긋내기 때문이다.

🔴 **못 찾으면 «통과» 가 아니라 «오류»(exit 2) 다.** 콜백을 하나도 못 찾았다면
그것은 «위반이 없다» 가 아니라 «검사가 낡았다» 이고, 둘은 화면에서 똑같이
보인다. 이 저장소는 그 구분을 `bc` 와 `jq` 에서 두 번 치렀다.
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = os.path.join(HERE, '.firmware', 'ble_server.cpp')

# NimBLE 2.x 가 부르는 것들. 늘리는 쪽은 안전하다 — 모르는 콜백을 빠뜨리는 것이
# 위험하지 그 반대가 아니다.
CALLBACKS = (
    'onWrite', 'onRead', 'onSubscribe', 'onStatus', 'onNotify',
    'onConnect', 'onDisconnect', 'onMTUChange', 'onAuthenticationComplete',
    'onIdentity', 'onPassKeyDisplay', 'onConfirmPIN',
)
MIN_CALLBACKS = 4   # 지금 파일에 이보다 적으면 스캐너가 낡은 것이다
NEEDLE = 'camera_task_'


def blank_comments_and_strings(src):
    """주석·문자열을 같은 길이의 공백으로 바꾼다. 줄 번호와 오프셋이 보존된다."""
    out = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        two = src[i:i + 2]
        if two == '//':
            j = src.find('\n', i)
            if j < 0:
                j = n
            out.append(' ' * (j - i)); i = j
        elif two == '/*':
            j = src.find('*/', i + 2)
            j = n if j < 0 else j + 2
            out.append(''.join('\n' if ch == '\n' else ' ' for ch in src[i:j])); i = j
        elif c in '"\'':
            q, j = c, i + 1
            while j < n:
                if src[j] == '\\':
                    j += 2; continue
                if src[j] == q:
                    j += 1; break
                j += 1
            out.append(''.join('\n' if ch == '\n' else ' ' for ch in src[i:j])); i = j
        else:
            out.append(c); i += 1
    return ''.join(out)


def callback_bodies(src):
    """(이름, 시작오프셋, 끝오프셋) 목록. 오프셋은 원본 src 기준이다."""
    clean = blank_comments_and_strings(src)
    found = []
    pattern = re.compile(r'\b(' + '|'.join(CALLBACKS) + r')\s*\(')
    for m in pattern.finditer(clean):
        open_brace = clean.find('{', m.end())
        if open_brace < 0:
            continue
        # 선언만 있고 본문이 없는 경우(`;` 가 먼저 오면) 건너뛴다.
        semi = clean.find(';', m.end())
        if 0 <= semi < open_brace:
            continue
        depth, i, n = 0, open_brace, len(clean)
        while i < n:
            if clean[i] == '{':
                depth += 1
            elif clean[i] == '}':
                depth -= 1
                if depth == 0:
                    break
            i += 1
        if depth != 0:
            return None   # 짝이 안 맞는다 — 스캐너를 믿을 수 없다
        found.append((m.group(1), open_brace, i))
    return found


def violations(src):
    """콜백 본문 안의 NEEDLE 을 (줄번호, 이름, 줄내용) 으로 돌려준다."""
    bodies = callback_bodies(src)
    if bodies is None:
        return None, None
    clean = blank_comments_and_strings(src)
    hits = []
    for name, a, b in bodies:
        start = 0
        while True:
            k = clean.find(NEEDLE, a, b)
            if k < 0:
                break
            line_no = src.count('\n', 0, k) + 1
            line = src.splitlines()[line_no - 1].strip()
            hits.append((line_no, name, line))
            a = k + len(NEEDLE)
            start += 1
    return bodies, hits


def die(msg):
    print(u'[X] 검사 자체가 못 돌았다: %s' % msg)
    sys.exit(2)


# ── 자가진단 — 이 검사가 **위반을 실제로 잡는가** ───────────────────────────
FAKE = '''
class CommandCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo &info) override {
        /* 주석 안의 중괄호 { 는 경계를 흔들면 안 된다 */
        const char* s = "여는 괄호 { 가 든 문자열";
        if (x) { camera_task_route_clear(); }
    }
};
void loop_only(void) { camera_task_tick(0); }
'''
_b, _h = violations(FAKE)
if _b is None or len(_h) != 1 or _h[0][1] != 'onWrite':
    die(u'자가진단이 심어 둔 위반을 못 잡았다 — 스캐너가 죽어 있다 (찾은 것: %r)' % (_h,))

# 그리고 콜백 **밖**의 것은 잡으면 안 된다 (loop_only 가 그것이다).
if any('loop_only' in h[2] for h in _h):
    die(u'자가진단: 콜백 밖의 호출까지 잡았다 — 경계가 틀렸다')

# ── 진짜 검사 ───────────────────────────────────────────────────────────────
if not os.path.isfile(TARGET):
    die(u'파일이 없다: %s' % TARGET)

source = io = open(TARGET, encoding='utf-8').read()
bodies, hits = violations(source)
if bodies is None:
    die(u'중괄호 짝이 안 맞는다 — 스캐너를 믿을 수 없다')
if len(bodies) < MIN_CALLBACKS:
    die(u'NimBLE 콜백을 %d 개밖에 못 찾았다 (최소 %d) — 이름이 바뀌었거나 정규식이 낡았다'
        % (len(bodies), MIN_CALLBACKS))

if hits:
    print(u'[X] camera_task 의 것이 NimBLE 콜백 안에서 불린다:')
    for line_no, name, line in hits:
        print(u'   - ble_server.cpp:%d  (%s 안)  %s' % (line_no, name, line))
    print(u'')
    print(u'camera_task.h 의 THREADING 절을 읽을 것. g_gps 와 g_route 는 잠금이')
    print(u'없고 loop() 가 쓴다 — 콜백에서 만지면 반쯤 바뀐 값으로 판정한다.')
    print(u'고치는 법: ble_server.cpp 의 «카메라 판정 자료로 가는 세 명령» 처럼')
    print(u'실어 두었다가 ble_server_tick() 에서 적용하고 그제야 답한다.')
    sys.exit(1)

print(u'[OK] NimBLE 콜백 %d 개 안에 camera_task 호출이 없다' % len(bodies))
