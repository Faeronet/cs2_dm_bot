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
    wpn = snap.local_weapon
    print(
        f"Aim: pitch={eye.x:.1f} yaw={eye.y:.1f} → "
        f"({aim.x:.0f}, {aim.y:.0f}, {aim.z:.0f})"
    )
    print(f"Weapon: {wpn.name or '<none>'} (id={wpn.id}) clip={wpn.clip}" + (" reload" if wpn.in_reload else ""))
    life = snap.local_life
    vuln = "vuln" if life.vulnerable else "immune"
    print(
        f"Life: {life.status} hp={life.health} pawn_alive={'Y' if life.pawn_alive else 'N'} "
        f"{vuln} moved={'Y' if life.has_moved_since_spawn else 'N'}"
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

    print(f"\nEnemies ({len(snap.enemies)}): visible={sum(1 for e in snap.enemies if e.visible)}")
    for e in snap.enemies:
        tag = "VISIBLE" if e.visible else ("radar" if e.radar_spotted else "hidden")
        meters = e.distance / 39.37
        vuln = "vuln" if e.vulnerable else "immune"
        chest = e.chest_position
        lsh = e.left_shoulder_position
        rsh = e.right_shoulder_position
        print(
            f"  [{tag:7}] {e.name or '<unknown>':16} hp={e.health} {vuln} "
            f"head=({e.head_position.x:.1f},{e.head_position.y:.1f},{e.head_position.z:.1f}) "
            f"chest=({chest.x:.1f},{chest.y:.1f},{chest.z:.1f}) "
            f"L=({lsh.x:.1f},{lsh.y:.1f},{lsh.z:.1f}) "
            f"R=({rsh.x:.1f},{rsh.y:.1f},{rsh.z:.1f}) ~{meters:.1f}m"
        )

    print(f"\nLadders ({len(snap.ladders)}):")
    for lad in snap.ladders[:12]:
        meters = lad.distance / 39.37
        print(
            f"  #{lad.index:4} h={lad.height:.0f} w={lad.width:.0f} d={lad.depth:.0f} "
            f"bot=({lad.bottom.x:.0f},{lad.bottom.y:.0f},{lad.bottom.z:.0f}) "
            f"top=({lad.top.x:.0f},{lad.top.y:.0f},{lad.top.z:.0f}) ~{meters:.0f}m"
        )
    if len(snap.ladders) > 12:
        print(f"  … ещё {len(snap.ladders) - 12}")

    print(f"\nDoors ({len(snap.doors)}): passable={sum(1 for d in snap.doors if d.passable)}")
    for door in snap.doors[:12]:
        meters = door.distance / 39.37
        lock = "locked" if door.locked else "unlocked"
        print(
            f"  #{door.index:4} {door.name:22} {door.state:8} {lock:8} "
            f"pos=({door.position.x:.0f},{door.position.y:.0f},{door.position.z:.0f}) "
            f"w={door.width:.0f} ~{meters:.0f}m pass={'yes' if door.passable else 'no'}"
        )
    if len(snap.doors) > 12:
        print(f"  … ещё {len(snap.doors) - 12}")

    blocking = sum(1 for br in snap.breakables if not br.passable)
    print(f"\nBreakables ({len(snap.breakables)}): blocking={blocking}")
    for br in snap.breakables[:16]:
        meters = br.distance / 39.37
        if br.broken:
            state = "broken"
        elif br.passable and br.model and "/entities/" in br.model.lower():
            state = "visual"
        else:
            state = f"hp={br.health}"
        model = br.model.rsplit("/", 1)[-1] if br.model else ""
        label = br.name if len(br.name) <= 24 else br.name[:21] + "..."
        if model and model != label:
            label = f"{label} ({model[:18]})" if len(label) < 20 else label
        print(
            f"  #{br.index:4} {br.kind:8} {label:32} {state:8} "
            f"pos=({br.position.x:.0f},{br.position.y:.0f},{br.position.z:.0f}) "
            f"w={br.width:.0f} ~{meters:.0f}m pass={'yes' if br.passable else 'no'}"
        )
    if len(snap.breakables) > 16:
        print(f"  … ещё {len(snap.breakables) - 16}")


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
