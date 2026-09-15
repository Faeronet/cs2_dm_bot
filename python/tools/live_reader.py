#!/usr/bin/env python3
"""Live CS2 memory reader — Linux native + Windows."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.reader import Reader


def print_snapshot(snap, json_mode: bool) -> None:
    if json_mode:
        print(json.dumps(snap.to_dict(), indent=2, ensure_ascii=False))
        return

    print("=== CS2 Memory Snapshot ===")
    print(f"Platform: {snap.platform}")
    print(f"In game: {'yes' if snap.in_game else 'no'} | In match: {'yes' if snap.in_match else 'no'}")
    print(f"Map: {snap.map_name or '<none>'}")
    print(f"Build: {snap.build_number}")
    print(
        f"Local: idx={snap.local_index} team={snap.local_team} "
        f"pos=({snap.local_position.x:.1f}, {snap.local_position.y:.1f}, {snap.local_position.z:.1f})"
    )
    eye = snap.local_eye_angles
    aim = snap.local_aim_point
    print(
        f"Aim: pitch={eye.x:.1f} yaw={eye.y:.1f} → "
        f"({aim.x:.0f}, {aim.y:.0f}, {aim.z:.0f})"
    )
    if snap.errors:
        print("Errors:", ", ".join(snap.errors))

    print(f"\nPlayers ({len(snap.players)}):")
    for p in snap.players:
        tag = " [LOCAL]" if p.is_local else ""
        print(
            f"  [{p.index:2}] {p.name or '<unknown>':16} team={p.team} hp={p.health:3} "
            f"dist={p.distance:6.1f} dormant={'Y' if p.dormant else 'N'} "
            f"spotted={'Y' if p.spotted else 'N'} "
            f"pos=({p.position.x:.1f}, {p.position.y:.1f}, {p.position.z:.1f}){tag}"
        )

    print(f"\nVisible enemies ({len(snap.visible_enemies)}):")
    for e in snap.visible_enemies:
        meters = e.distance / 39.37
        print(
            f"  {e.name or '<unknown>':16} hp={e.health} "
            f"pos=({e.position.x:.1f}, {e.position.y:.1f}, {e.position.z:.1f}) ~{meters:.1f}m"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="CS2 live memory reader")
    parser.add_argument("--pid", type=int, help="Process PID")
    parser.add_argument("--once", action="store_true", help="Single snapshot")
    parser.add_argument("--interval", type=float, default=0.5, help="Poll interval seconds")
    parser.add_argument("--json", action="store_true", help="JSON output")
    args = parser.parse_args()

    if args.pid:
        reader = Reader.from_pid(args.pid)
    else:
        reader = Reader.from_live()
        if not reader:
            print("CS2 process not found. Launch the game first.", file=sys.stderr)
            return 1

    if not reader.backend.is_valid():
        print("Memory backend is not valid.", file=sys.stderr)
        return 1

    while True:
        snap = reader.capture()
        if not args.json:
            os.system("clear")
        print_snapshot(snap, args.json)
        if args.once:
            break
        time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
