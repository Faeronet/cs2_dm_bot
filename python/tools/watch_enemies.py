#!/usr/bin/env python3
"""Real-time enemy list + active offset audit (writes data/runtime_tweaks.json)."""

from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.enemy_calibration import (
    audit_enemy_reads,
    audit_visibility_reads,
    calibrate_enemy_offsets,
    schema_drift_messages,
)
from cs2_reader.match_calibration import calibrate_match_offsets
from cs2_reader.reader import Reader

LOG_PATH = Path(__file__).resolve().parents[2] / "data" / "enemy_watch.log"


def _log(line: str) -> None:
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%H:%M:%S")
    msg = f"[{ts}] {line}"
    with LOG_PATH.open("a", encoding="utf-8") as fh:
        fh.write(msg + "\n")


def _fmt_enemy(e) -> str:
    if e.spotted:
        vis = "VIS"
    elif getattr(e, "radar_spotted", False):
        vis = "RAD"
    else:
        vis = "   "
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
    parser.add_argument("--interval", type=float, default=0.35, help="Poll period (seconds)")
    parser.add_argument("--calibrate-every", type=float, default=8.0, help="Full offset pass")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    reader = Reader.from_live()
    if not reader or not reader.backend.is_valid():
        print("CS2 not running or memory not accessible", file=sys.stderr)
        return 1

    _log("watch_enemies started")
    last_cal = 0.0
    while True:
        now = time.monotonic()
        issues = audit_enemy_reads(reader)
        if issues or now - last_cal >= args.calibrate_every:
            try:
                drift = schema_drift_messages(reader.schema)
            except OSError as exc:
                drift = [f"schema drift skip: {exc}"]
            mfixes = calibrate_match_offsets(reader, min_interval_s=0.0, allow_dump=False, force=bool(issues))
            efixes = calibrate_enemy_offsets(
                reader, min_interval_s=0.0, allow_dump=True, force=bool(issues),
            )
            last_cal = now
            for line in drift + mfixes + efixes + [f"audit: {x}" for x in issues]:
                _log(line)
        else:
            drift = []
            mfixes = []
            efixes = []

        snap = reader.capture()
        if not args.once:
            os.system("clear")

        vis_n = sum(1 for e in snap.enemies if e.spotted)
        rad_n = sum(1 for e in snap.enemies if e.radar_spotted)
        viss = audit_visibility_reads(reader)
        print(
            f"map={snap.map_name or '?'}  enemies={len(snap.enemies)} "
            f"VIS={vis_n} RAD={rad_n} local=#{snap.local_index}"
        )
        if issues or viss:
            print("WARN:", "; ".join(issues + viss))
        for line in (mfixes + efixes)[-5:]:
            print(f"fix: {line}")

        enemies = sorted(snap.enemies, key=lambda e: e.distance)
        for e in enemies[:18]:
            print(_fmt_enemy(e))
        if len(enemies) > 18:
            print(f"… +{len(enemies) - 18} more")

        if args.once:
            break
        time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
