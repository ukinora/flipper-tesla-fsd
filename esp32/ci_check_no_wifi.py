#!/usr/bin/env python3
"""Assert that a FSD_NO_WIFI build really has no WiFi in it.

Run after `pio run -e lilygo-t2can`:

    python ci_check_no_wifi.py lilygo-t2can

WHY THIS EXISTS
---------------
Removing WiFi from this variant was a security fix (the AP password is public
and the dashboard's WebSocket authenticated nothing, so anyone near a parked car
could open CAN transmit). Nothing about the source makes that removal hard to
undo by accident: adding one `#include <WiFi.h>` to a shared header, or dropping
a `-<...>` line while editing build_src_filter, puts it back with no visible
symptom. This check is the thing that notices.

WHAT IT LOOKS AT
----------------
Three layers, because each one can be clean while another is not:

  objects    -- did the excluded .cpp files get compiled after all?
  libraries  -- did the dependency finder pull the WiFi library onto the link
                line? (It scans #include lines as text, so a guarded include
                still counts unless lib_ldf_mode evaluates conditionals.)
  image      -- is any of it actually in the firmware?

🔴 THE TRAP IN CHECKING FOR "12345678"
--------------------------------------
It is a substring of the C library's hex digit table, "0123456789abcdef", which
is in every image. A naive substring search fails forever, and a check that
always fails gets deleted. So credentials are matched as WHOLE NUL-delimited
tokens, and there is a self-test below that fails if that distinction ever stops
working -- otherwise a future change to the extraction could quietly turn this
check into a no-op that always passes.
"""

import re
import sys
from pathlib import Path

# Compiled sources that must not exist for this variant.
FORBIDDEN_OBJECTS = ("wifi_manager", "web_dashboard", "http_can_stream")

# Archives that must not reach the linker.
FORBIDDEN_LIBS = ("libWiFi.a", "libWebSockets.a", "libWebServer.a", "libESPmDNS.a")

# Exact strings. Matched against whole NUL-delimited tokens, never as substrings
# -- see the note about "12345678" above.
FORBIDDEN_TOKENS = ("Tesla-FSD", "12345678")

# Substrings. Safe to match loosely: none of these occur in library boilerplate.
FORBIDDEN_SUBSTRINGS = (b"softAP", b"WebSocket", b"192.168.4.1", b"/sdformat")

TOKEN_RE = re.compile(rb"[\x20-\x7e]{4,}")


def tokens(blob: bytes) -> set:
    """Printable runs between non-printable bytes -- i.e. what a string literal
    looks like in .rodata. A run is one token; substrings of it are not."""
    return {m.group().decode("ascii") for m in TOKEN_RE.finditer(blob)}


def self_test() -> None:
    """If this fails, the check below cannot be trusted in either direction."""
    hex_table = b"\x000123456789abcdef\x00"
    assert "12345678" not in tokens(hex_table), (
        "extraction is matching substrings -- this check would fail on every "
        "clean build and get deleted"
    )
    assert "12345678" in tokens(b"\x00Tesla-FSD\x0012345678\x00"), (
        "extraction is missing a real credential -- this check would pass on "
        "every build, including a broken one"
    )


def main() -> int:
    self_test()

    env = sys.argv[1] if len(sys.argv) > 1 else "lilygo-t2can"
    build = Path(__file__).parent / ".pio" / "build" / env
    image = build / "firmware.bin"

    # Fail loudly rather than pass vacuously if the layout moves. A check that
    # silently examines nothing is worse than no check: it reports success.
    if not image.is_file():
        print(f"FAIL: {image} not found - build {env} first")
        return 1

    failures = []

    objects = sorted(p.name for p in build.rglob("*.o")
                     if any(f in p.name for f in FORBIDDEN_OBJECTS))
    if objects:
        failures.append(f"compiled objects that should be excluded: {objects}")

    libs = sorted(p.name for p in build.rglob("*.a") if p.name in FORBIDDEN_LIBS)
    if libs:
        failures.append(f"archives on the link line: {libs}")

    blob = image.read_bytes()

    found_tokens = sorted(set(FORBIDDEN_TOKENS) & tokens(blob))
    if found_tokens:
        failures.append(f"credential strings in the image: {found_tokens}")

    found_subs = sorted(s.decode() for s in FORBIDDEN_SUBSTRINGS if s in blob)
    if found_subs:
        failures.append(f"web strings in the image: {found_subs}")

    if failures:
        print(f"FAIL: {env} was built without WiFi, but:")
        for f in failures:
            print(f"  - {f}")
        print("\nIf WiFi is being deliberately restored on this variant, remove")
        print("this check in the same commit and say why in the message.")
        return 1

    print(f"OK: {env} - no WiFi objects, no WiFi archives, "
          f"no credentials, no web strings ({len(blob):,} B image)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
