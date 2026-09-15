#!/usr/bin/env python3
"""Тест: чтение прицела + клавиши/мышь (резко, без навигации)."""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.input import create_input
from cs2_reader.reader import Reader


def _reader(dump: str | None) -> Reader:
    if dump:
        return Reader.from_dump(dump)
    reader = Reader.from_live()
    if not reader:
        raise SystemExit("CS2 не запущен (или укажи --dump file.dmp)")
    return reader


def _print_aim(reader: Reader) -> None:
    pos = reader.local_position()
    eye = reader.local_eye_angles()
    fwd = reader.local_aim_forward()
    aim = reader.local_aim_point()
    print(f"Позиция:     ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
    print(f"Прицел pitch/yaw: {eye.x:.2f} / {eye.y:.2f}")
    print(f"Направление: ({fwd.x:.4f}, {fwd.y:.4f}, {fwd.z:.4f})")
    print(f"Точка прицела: ({aim.x:.0f}, {aim.y:.0f}, {aim.z:.0f})")


def main() -> int:
    parser = argparse.ArgumentParser(description="Чтение прицела и тест ввода")
    parser.add_argument("--dump", help="Путь к .dmp (иначе live)")
    parser.add_argument("--read-aim", action="store_true", help="Показать куда смотрит прицел")
    parser.add_argument("--watch-aim", type=float, metavar="SEC", help="Печатать прицел N секунд")
    parser.add_argument("--key", help="Нажать клавишу: w/a/s/d/space")
    parser.add_argument("--hold", type=float, default=1.0, help="Секунд держать --key")
    parser.add_argument("--mouse", nargs=2, type=int, metavar=("DX", "DY"), help="Резко сдвинуть мышь")
    parser.add_argument(
        "--look",
        nargs=2,
        type=float,
        metavar=("YAW_DEG", "PITCH_DEG"),
        help="Резкий поворот камеры на N градусов",
    )
    args = parser.parse_args()

    if os.geteuid() == 0 and (args.key or args.mouse or args.look):
        print("Ввод: не запускай от root", file=sys.stderr)
        return 1

    reader = _reader(args.dump)

    if args.read_aim or args.watch_aim:
        if not reader.backend.is_valid():
            print("Память недоступна", file=sys.stderr)
            return 1
        if args.read_aim:
            _print_aim(reader)
        if args.watch_aim:
            end = time.time() + args.watch_aim
            while time.time() < end:
                _print_aim(reader)
                print("-" * 40)
                time.sleep(0.25)

    if args.key or args.mouse or args.look:
        inp = create_input()
        inp.prepare()
        try:
            print("Через 3 сек — переключись на CS2")
            time.sleep(3)
            if args.key:
                print(f"KEY {args.key} {args.hold}s")
                inp.key_down(args.key)
                time.sleep(args.hold)
                inp.key_up(args.key)
            if args.mouse:
                dx, dy = args.mouse
                print(f"MOUSE {dx},{dy}")
                inp.mouse_move(dx, dy)
            if args.look:
                yaw, pitch = args.look
                print(f"LOOK yaw={yaw} pitch={pitch}")
                inp.mouse_look_deg(yaw, pitch)
        finally:
            inp.close()

    if not any([args.read_aim, args.watch_aim, args.key, args.mouse, args.look]):
        parser.print_help()
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
