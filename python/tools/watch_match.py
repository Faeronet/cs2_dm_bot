#!/usr/bin/env python3
"""Realtime match-detection monitor (for debugging stuck menu/wait)."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.match_calibration import calibrate_match_offsets
from cs2_reader.reader import Reader
from tools.launch_dm_official import _signals


def main() -> int:
    r = Reader.from_live()
    if not r:
        print("CS2 not running")
        return 1
    while True:
        fixes = calibrate_match_offsets(r, min_interval_s=5.0, force=False)
        sig = _signals(r)
        os.system("clear")
        print(
            f"map={sig['map']!r} on_srv={sig['on_server']} alive_fast={sig['alive_fast']} "
            f"hp={sig['health']} pawn=0x{sig['pawn']:x}"
        )
        if fixes:
            print("fixes:", "; ".join(fixes))
        time.sleep(0.5)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
