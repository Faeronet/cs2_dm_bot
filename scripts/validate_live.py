#!/usr/bin/env python3
"""Validate live CS2 memory reading against dumped offsets."""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from cs2_reader.platform import find_cs2_pid
from cs2_reader.reader import Reader


def main() -> int:
    pid = find_cs2_pid()
    if not pid:
        print("ERROR: CS2 process not found. Launch the game first.", file=sys.stderr)
        return 1

    print(f"CS2 PID: {pid}")
    reader = Reader.from_pid(pid)
    if not reader.backend.is_valid():
        print("ERROR: Cannot access process memory.", file=sys.stderr)
        return 1

    mods = reader.debug_modules()
    key_mods = {k: hex(v) for k, v in mods.items() if "client" in k or "engine" in k}
    print(f"Key modules: {json.dumps(key_mods, indent=2)}")

    snap = reader.capture()
    print(json.dumps(snap.to_dict(), indent=2, ensure_ascii=False))

    if not snap.client_base:
        print("\nFAIL: libclient.so not mapped", file=sys.stderr)
        return 2

    if snap.in_match:
        print(f"\nOK: In match on {snap.map_name}, {len(snap.players)} players, {len(snap.visible_enemies)} visible enemies")
    elif snap.in_game:
        print(f"\nWARN: In game ({snap.map_name}) but not in active match (local_pawn=0). Join a match for full validation.")
    else:
        print("\nWARN: In main menu. Join a match (Deathmatch/Casual) for full player/position validation.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
