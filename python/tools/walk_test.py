#!/usr/bin/env python3
"""Тест-драйв: крестик → мышь → круг. Читает прицел из памяти, без навигации."""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.input import create_input
from cs2_control.movement import Movement
from cs2_reader.reader import Reader


def main() -> int:
    parser = argparse.ArgumentParser(description="Тест передвижения: крестик, мышь, круг")
    parser.add_argument("--countdown", type=int, default=5, help="Секунд до старта")
    parser.add_argument("--leg", type=float, default=1.2, help="Секунд на каждую сторону крестика")
    parser.add_argument("--circle", type=float, default=6.0, help="Секунд идти кругом")
    parser.add_argument("--cross-only", action="store_true")
    parser.add_argument("--spin-only", action="store_true")
    parser.add_argument("--circle-only", action="store_true")
    parser.add_argument("--deg-per-px", type=float, default=0.08, help="Калибровка мыши (град/px)")
    args = parser.parse_args()

    if os.geteuid() == 0:
        print("Не запускай от root — uinput не дойдёт до CS2", file=sys.stderr)
        return 1

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    for _ in range(30):
        if reader.local_pawn():
            break
        print("Ожидание матча…")
        time.sleep(1)
    else:
        print("Зайди в матч и повтори", file=sys.stderr)
        return 1

    mov = Movement(reader, create_input(), deg_per_px=args.deg_per_px)
    mov.log_aim("ready")

    print()
    print("Тест: крестик (WASD) → поворот мыши → круг (W + mouse)")
    print("Прицел читается из памяти на каждом шаге")
    for sec in range(args.countdown, 0, -1):
        print(f"  Старт через {sec}… (CS2 в фокусе)")
        time.sleep(1)

    try:
        only = args.cross_only or args.spin_only or args.circle_only
        if args.cross_only or not only:
            mov.cross(args.leg)
            time.sleep(0.3)
        if args.spin_only or not only:
            mov.spin_mouse(steps=4, yaw_deg=90.0)
            time.sleep(0.3)
        if args.circle_only or not only:
            mov.circle(seconds=args.circle)
        mov.log_aim("done")
        print("Тест завершён.")
    except KeyboardInterrupt:
        print("\nПрервано.")
    finally:
        mov.stop()
        mov.input.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
