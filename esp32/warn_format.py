"""우리 소스에서 printf 형식 불일치를 **컴파일 에러**로 만든다.

🔴 2026-09-10 에 실제로 물렸다. 송신 허용을 지우면서
`Serial.printf("[RULE] %s · 보냄 %u ...", g_sent, g_refused, g_last_refusal)`
가 남았다 — `%s` 는 넷인데 인자가 셋이다. **보드 여덟 개가 전부 SUCCESS 로
빌드됐고**, 그 줄은 차에서 `ruleq` 를 쳤을 때에만 터진다. 차에는 이것을 고칠
PC 가 없다.

`Print::printf()` 에는 이미 `__attribute__((format(printf, 2, 3)))` 이 붙어
있다. 즉 컴파일러는 **처음부터 알고 있었고 아무도 안 물었다.**

🔴 `projenv` 인 것이 요점이다 — 우리 소스에만 건다. Arduino 코어와 라이브러리는
우리가 고칠 수 없는 코드라, 거기서 나는 경고로 빌드를 세우면 이 검사는 첫날에
꺼진다. 늘 실패하는 검사는 지워진다는 것을 이 저장소는 `12345678` 로 한 번
배웠다.

⚠️ post 스크립트인 이유: `projenv` 는 post 에서만 존재한다. 그래도 컴파일보다
먼저 돈다 (SConscript 시점).
"""
Import("projenv")  # noqa: F821

projenv.Append(CCFLAGS=["-Wformat", "-Werror=format"])
