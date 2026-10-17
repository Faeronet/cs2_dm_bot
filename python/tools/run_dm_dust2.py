#!/usr/bin/env python3
"""Walk-nav на de_dust2. Отдельного тренировочного layout под Dust2 нет,
поэтому поддерживаем только консольный путь и --skip-menu.

  python3 python/tools/run_dm_dust2.py                     # map de_dust2 через ~
  python3 python/tools/run_dm_dust2.py --skip-menu         # уже на dust2
  python3 python/tools/run_dm_dust2.py --duration 240      # лимит сек
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.input import create_input
from cs2_menu import MenuController
from cs2_nav.coord_map import CoordinateMap
from cs2_nav.navigator import NavConfig, Navigator
from cs2_reader.reader import Reader

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_NAV = os.path.join(ROOT, "data", "maps", "de_dust2.nav.json")
MAP_ID = "de_dust2"


def countdown(sec: int, msg: str) -> None:
    print(msg, flush=True)
    for i in range(sec, 0, -1):
        print(f"  {i}…", flush=True)
        time.sleep(1)


def wait_pawn(reader: Reader, timeout: float = 120.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if reader.local_pawn():
            m = (reader.map_name() or "").lower()
            if MAP_ID in m:
                return True
        print(
            f"  ждём {MAP_ID}… map={reader.map_name() or '—'} "
            f"pawn={'да' if reader.local_pawn() else 'нет'}",
            flush=True,
        )
        time.sleep(1.0)
    return False


def run_navigation(reader: Reader, inp, nav_path: str, duration: float, hz: float) -> int:
    if not os.path.isfile(nav_path):
        print(f"Нет навмеша: {nav_path}", file=sys.stderr)
        return 1
    print(f"Навмеш: {nav_path}", flush=True)
    cmap = CoordinateMap.from_navmesh(nav_path)
    print(cmap.stats(), flush=True)

    cfg = NavConfig(tick_hz=hz, duration=duration or None, loop=True, verbose_log=True)
    nav = Navigator(reader, inp, cmap, cfg)
    pos = reader.local_position()
    route = nav.plan((pos.x, pos.y, pos.z))
    print(
        f"Маршрут: ({pos.x:.0f},{pos.y:.0f},{pos.z:.0f}) → "
        f"цель id={nav.goal_id}, {route.length:.0f}u",
        flush=True,
    )
    countdown(5, "Навигация — CS2 в фокусе")
    nav.run(duration=cfg.duration)
    if nav.events:
        print("События:", "; ".join(nav.events[-8:]), flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="de_dust2: walk nav (консоль или skip-menu)")
    parser.add_argument("--nav", default=DEFAULT_NAV)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--hz", type=float, default=40.0)
    parser.add_argument("--skip-menu", action="store_true", help="Уже на dust2 — пропустить загрузку карты")
    args = parser.parse_args()

    if os.geteuid() == 0:
        print("Не запускай от root", file=sys.stderr)
        return 1

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    inp = create_input()
    inp.prepare()
    menu = MenuController(reader, game_input=inp)

    try:
        if not args.skip_menu:
            countdown(5, "Старт: CS2 в фокусе (главное меню или консоль откроется сама)")
            print(f"Режим: консоль map {MAP_ID}", flush=True)
            if not menu.load_map_console(MAP_ID):
                return 2
        else:
            print(f"skip-menu, карта={menu.current_map()!r}", flush=True)

        if not wait_pawn(reader):
            return 3

        print(f"OK: {reader.map_name()} pawn={hex(reader.local_pawn() or 0)}", flush=True)
        return run_navigation(reader, inp, args.nav, args.duration, args.hz)
    finally:
        menu.close()
        inp.close()


if __name__ == "__main__":
    raise SystemExit(main())
