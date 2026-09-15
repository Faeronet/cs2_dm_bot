#!/usr/bin/env python3
"""Read CS2 state from a Windows minidump (.dmp) file."""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.reader import Reader


def main() -> int:
    parser = argparse.ArgumentParser(description="CS2 minidump reader")
    parser.add_argument("dump", help="Path to .dmp file")
    parser.add_argument("--json", action="store_true", help="JSON output")
    args = parser.parse_args()

    try:
        reader = Reader.from_dump(args.dump)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    if not reader.backend.is_valid():
        print("Dump has no readable memory regions.", file=sys.stderr)
        return 1

    snap = reader.capture()
    if args.json:
        print(json.dumps(snap.to_dict(), indent=2, ensure_ascii=False))
    else:
        print(f"Map: {snap.map_name or '<none>'}")
        print(f"Local pos: ({snap.local_position.x:.1f}, {snap.local_position.y:.1f}, {snap.local_position.z:.1f})")
        eye = snap.local_eye_angles
        aim = snap.local_aim_point
        print(f"Aim pitch/yaw: {eye.x:.1f} / {eye.y:.1f}")
        print(f"Aim point: ({aim.x:.0f}, {aim.y:.0f}, {aim.z:.0f})")
        print(f"Players: {len(snap.players)}, visible enemies: {len(snap.visible_enemies)}")
        for e in snap.visible_enemies:
            print(f"  ENEMY {e.name}: ({e.position.x:.1f}, {e.position.y:.1f}, {e.position.z:.1f})")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
