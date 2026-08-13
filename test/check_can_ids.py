#!/usr/bin/env python3
"""Assert the two CAN ID lists agree.

    python test/check_can_ids.py

WHY THIS EXISTS
---------------
CAN IDs are defined in two places:

    fsd_logic/fsd_handler.h    the Flipper build and the host tests
    esp32/.firmware/config.h   the ESP32 firmware

21 names appear in both. Nothing enforced that they matched -- the compiler
emitted "macro redefined" warnings when one file happened to include both, and
those warnings were ignored, 21 of them on every build.

That is now worse rather than better: camera_task.cpp stopped pulling in the
second header, so the build is warning-clean and the two lists are no longer
compared by anything at all. This script is what replaces the warning, and
unlike the warning it says which name disagrees and what the two values are.

A mismatch is not cosmetic. These numbers decide which frames the firmware
parses and which it refuses to transmit; a firmware reading gear from the wrong
ID has happened here before (PR #18) and it failed silently for weeks.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCES = {
    "fsd_logic/fsd_handler.h": ROOT / "fsd_logic" / "fsd_handler.h",
    "esp32/.firmware/config.h": ROOT / "esp32" / ".firmware" / "config.h",
}

DEFINE_RE = re.compile(r"^\s*#define\s+(CAN_ID_\w+)\s+(0x[0-9A-Fa-f]+|\d+)", re.M)


def read_ids(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    # int(..., 0) handles the 0x form; the trailing u suffix is stripped by the
    # regex, which deliberately does not capture it.
    return {m.group(1): int(m.group(2), 0) for m in DEFINE_RE.finditer(text)}


def main() -> int:
    tables = {}
    for label, path in SOURCES.items():
        if not path.is_file():
            print(f"FAIL: {path} not found")
            return 1
        tables[label] = read_ids(path)
        if not tables[label]:
            # Parsing nothing and reporting success is the failure mode this
            # check has to avoid: it would go green forever after a refactor
            # renamed the macros or moved them to an enum.
            print(f"FAIL: no CAN_ID_* defines found in {label} — has the format changed?")
            return 1

    (label_a, a), (label_b, b) = tables.items()
    common = sorted(set(a) & set(b))

    if not common:
        print(f"FAIL: {label_a} and {label_b} share no names — is one of them the wrong file?")
        return 1

    mismatched = [(k, a[k], b[k]) for k in common if a[k] != b[k]]
    if mismatched:
        print(f"FAIL: {len(mismatched)} CAN ID(s) disagree between the two headers:")
        for name, va, vb in mismatched:
            print(f"  {name}: {label_a}=0x{va:03X}  {label_b}=0x{vb:03X}")
        print("\nThese decide which frames get parsed and transmitted. Fix the")
        print("wrong one rather than the check.")
        return 1

    print(f"OK: {len(common)} CAN IDs defined in both headers agree "
          f"({label_a}={len(a)}, {label_b}={len(b)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
