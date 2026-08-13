#!/usr/bin/env python3
"""Assert the two CAN ID lists agree, and stay agreed.

    python test/check_can_ids.py

WHY THIS EXISTS
---------------
CAN IDs are defined in two places:

    fsd_logic/fsd_handler.h    the Flipper build and the host tests
    esp32/.firmware/config.h   the ESP32 firmware

Several names appear in both, and nothing enforced that they matched. The
compiler emitted "macro redefined" whenever a file happened to include both, and
those warnings were ignored -- 48 of them per build. The headers no longer
collide, so the build is warning-clean and nothing compares the two lists any
more. This script is what replaces the warning, and unlike the warning it says
which name disagrees and what the two values are.

A mismatch is not cosmetic. These numbers decide which frames the firmware
parses and which it refuses to transmit; reading gear from the wrong ID has
happened here before (PR #18) and it failed silently for weeks.

🔴 WHY THERE IS AN EXPLICIT LIST RATHER THAN AN INTERSECTION
------------------------------------------------------------
The first version of this compared `set(a) & set(b)` and nothing else, which
made it blind to exactly the change it was supposed to catch: delete or rename a
shared ID on one side and it simply drops out of the intersection, leaving zero
mismatches and a green build. Renaming CAN_ID_AP_CONTROL in one header would
have passed.

So the shared set is written down. Four things fail:

    value changed          the original job
    deleted from a header  it left the intersection -- previously invisible
    renamed                a deletion plus an unexpected addition
    unexpectedly shared    a new name in both headers that nobody declared,
                           which means a second source of truth was created
                           without anyone deciding to

The last one is what keeps this list honest: it cannot silently fall behind,
because falling behind IS a failure.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCES = {
    "fsd_logic/fsd_handler.h": ROOT / "fsd_logic" / "fsd_handler.h",
    "esp32/.firmware/config.h": ROOT / "esp32" / ".firmware" / "config.h",
}

# Every CAN ID that both headers define, and the value both must carry.
# Adding a row is a deliberate act: it means accepting a second definition of
# something. Prefer defining an ID once and including it.
REQUIRED_SHARED_IDS = {
    "CAN_ID_AP_CONTROL": 0x3FD,
    "CAN_ID_AP_LEGACY": 0x3EE,
    "CAN_ID_BMS_HV_BUS": 0x132,
    "CAN_ID_BMS_SOC": 0x292,
    "CAN_ID_BMS_THERMAL": 0x312,
    "CAN_ID_DAS_AP_CONFIG": 0x331,
    "CAN_ID_DAS_CONTROL": 0x2B9,
    "CAN_ID_DAS_STATUS2": 0x389,
    "CAN_ID_DAS_STATUS_HW3": 0x399,
    "CAN_ID_DI_SPEED": 0x257,
    "CAN_ID_DI_STATE": 0x286,
    "CAN_ID_DI_SYS_STATUS": 0x118,
    "CAN_ID_EPAS_STATUS": 0x370,
    "CAN_ID_ESP_STATUS": 0x145,
    "CAN_ID_FOLLOW_DIST": 0x3F8,
    "CAN_ID_GTW_CAR_CONFIG": 0x398,
    "CAN_ID_GTW_CAR_STATE": 0x318,
    "CAN_ID_ISA_SPEED": 0x399,
    "CAN_ID_SCCM_RSTALK": 0x229,
    "CAN_ID_STEER_ANGLE": 0x129,
    "CAN_ID_STW_ACTN_RQ": 0x045,
    "CAN_ID_TRIP_PLANNING": 0x082,
    "CAN_ID_UI_WARNING": 0x311,
    "CAN_ID_VCFRONT_LIGHT": 0x3F5,
    "CAN_ID_VCLEFT_SWITCH": 0x3C2,
}

DEFINE_RE = re.compile(r"^\s*#define\s+(CAN_ID_\w+)\s+(0x[0-9A-Fa-f]+|\d+)", re.M)


def read_ids(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    # int(..., 0) handles the 0x form; the trailing u suffix is not captured.
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
            # renamed the macros or moved them into an enum.
            print(f"FAIL: no CAN_ID_* defines found in {label} — has the format changed?")
            return 1

    problems = []

    for name, expected in sorted(REQUIRED_SHARED_IDS.items()):
        for label, table in tables.items():
            if name not in table:
                problems.append(
                    f"{name} is missing from {label}. If it moved to a single "
                    f"definition, drop it from REQUIRED_SHARED_IDS in the same commit."
                )
            elif table[name] != expected:
                problems.append(
                    f"{name} is 0x{table[name]:03X} in {label}, expected 0x{expected:03X}."
                )

    # A name in both headers that nobody declared means a second source of truth
    # appeared without a decision. Failing here is what stops the list above from
    # quietly falling behind the code it is supposed to describe.
    (label_a, a), (label_b, b) = tables.items()
    undeclared = sorted((set(a) & set(b)) - set(REQUIRED_SHARED_IDS))
    for name in undeclared:
        problems.append(
            f"{name} is defined in BOTH headers but not listed here "
            f"({label_a}=0x{a[name]:03X}, {label_b}=0x{b[name]:03X}). "
            f"Define it once, or add it to REQUIRED_SHARED_IDS on purpose."
        )

    if problems:
        print(f"FAIL: {len(problems)} problem(s) with the shared CAN ID list:")
        for p in problems:
            print(f"  - {p}")
        print("\nThese numbers decide which frames get parsed and transmitted.")
        print("Fix the header, not the check.")
        return 1

    print(f"OK: {len(REQUIRED_SHARED_IDS)} shared CAN IDs present and equal in both "
          f"headers ({label_a}={len(a)}, {label_b}={len(b)}; no undeclared overlap)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
