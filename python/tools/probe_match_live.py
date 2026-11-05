#!/usr/bin/env python3
"""Покадровая диагностика матч-детекта (офсеты + сигналы)."""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.match_calibration import calibrate_match_offsets
from cs2_reader.reader import Reader
from tools.launch_dm_official import _signals


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--once", action="store_true")
    args = p.parse_args()

    r = Reader.from_live()
    if not r:
        print("CS2 не запущен")
        return 1

    while True:
        fixes = calibrate_match_offsets(r, min_interval_s=0.0)
        sig = _signals(r)
        gv = r._global_vars_base()
        print(
            f"map={sig['map']!r} resolved={r.resolved_map_name()!r} "
            f"loaded={sig['map_loaded']} on_srv={sig['on_server']} in_world={sig['in_world']} "
            f"alive_fast={sig['alive_fast']} hp={sig['health']} ls={sig['life_state']} "
            f"pawn=0x{sig['pawn']:x} gv=0x{gv:x}"
        )
        if fixes:
            print("  fixes:", "; ".join(fixes))
        if args.once:
            break
        time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
