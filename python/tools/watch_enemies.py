#!/usr/bin/env python3
"""Real-time enemy list from CS2 memory; auto-calibrates stale offsets."""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.enemy_calibration import calibrate_enemy_offsets, schema_drift_messages
from cs2_reader.reader import Reader


def _fmt_enemy(e) -> str:
    vis = "VIS" if e.spotted else ("RAD" if e.radar_spotted else "   ")
    vuln = "ok" if e.vulnerable else "imm"
    hz = e.head_position.z - e.position.z
    return (
        f"#{e.index:2} {vis} {e.name or '?':16} hp={e.health:3} {vuln} "
        f"dist={e.distance:6.0f} "
        f"pos=({e.position.x:6.0f},{e.position.y:6.0f},{e.position.z:5.1f}) "
        f"head_dz={hz:4.1f} "
        f"{'D' if e.dormant else ' '}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Watch enemies in real time")
    parser.add_argument("--interval", type=float, default=0.25, help="Poll period (seconds)")
    parser.add_argument("--calibrate-every", type=float, default=15.0, help="Offset check period")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    reader = Reader.from_live()
    if not reader or not reader.backend.is_valid():
        print("CS2 not running or memory not accessible", file=sys.stderr)
        return 1

    last_cal = 0.0
    while True:
        now = time.monotonic()
        if now - last_cal >= args.calibrate_every:
            drift = schema_drift_messages(reader.schema)
            fixes = calibrate_enemy_offsets(reader, min_interval_s=0.0)
            last_cal = now
        else:
            drift = []
            fixes = []

        snap = reader.capture()
        if not args.once:
            os.system("clear")

        print(f"map={snap.map_name or '?'}  enemies={len(snap.enemies)}  local=#{snap.local_index}")
        if drift:
            print("schema drift:", "; ".join(drift))
        for line in fixes or reader._enemy_calibrate_log[-3:]:
            print(f"fix: {line}")

        enemies = sorted(snap.enemies, key=lambda e: e.distance)
        for e in enemies[:20]:
            print(_fmt_enemy(e))
        if len(enemies) > 20:
            print(f"… +{len(enemies) - 20} more")

        if args.once:
            break
        time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
