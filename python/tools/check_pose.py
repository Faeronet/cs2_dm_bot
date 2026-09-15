#!/usr/bin/env python3
"""Проверка: позиция + куда смотрит прицел (live или дамп)."""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.reader import Reader


def print_pose(reader: Reader, target_xy: tuple[float, float] | None) -> None:
    feet = reader.local_position()
    eyes = reader.local_eye_position()
    ang = reader.local_eye_angles()
    aim = reader.local_aim_point()
    horiz = reader.local_aim_forward()

    print(f"Ноги:   ({feet.x:.1f}, {feet.y:.1f}, {feet.z:.1f})")
    print(f"Глаза:  ({eyes.x:.1f}, {eyes.y:.1f}, {eyes.z:.1f})  offset_z={eyes.z - feet.z:.1f}")
    print(f"Углы:   pitch={ang.x:.1f}°  yaw={ang.y:.1f}°  roll={ang.z:.1f}°")
    print(f"Прицел: ({aim.x:.1f}, {aim.y:.1f}, {aim.z:.1f})")
    print(f"Горизонт: ({horiz.x:.3f}, {horiz.y:.3f}, {horiz.z:.3f})")

    if abs(ang.x) > 70:
        print("  → pitch большой: смотришь вниз/вверх, прицел на пересечении с уровнем ног")
    if target_xy:
        chk = reader.local_pose_check(target_xy[0], target_xy[1])
        print(
            f"К точке ({target_xy[0]:.0f},{target_xy[1]:.0f}): "
            f"bearing={chk['bearing_deg']:.1f}° err={chk['yaw_err_to_target']:.1f}°"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dump", help="Файл .dmp")
    parser.add_argument("--watch", type=float, metavar="SEC")
    parser.add_argument("--toward", nargs=2, type=float, metavar=("X", "Y"), help="Сверить yaw с точкой")
    args = parser.parse_args()

    reader = Reader.from_dump(args.dump) if args.dump else Reader.from_live()
    if not reader or not reader.backend.is_valid():
        print("Нет данных памяти", file=sys.stderr)
        return 1

    target = tuple(args.toward) if args.toward else None

    if args.watch:
        end = time.time() + args.watch
        while time.time() < end:
            print("=" * 48)
            print_pose(reader, target)
            time.sleep(0.3)
    else:
        print_pose(reader, target)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
