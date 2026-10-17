#!/usr/bin/env python3
"""Живая навигация: карта из записей → случайная точка → камера + WASD.

Примеры:
  python3 python/tools/walk_nav.py --duration 30            # вся de_inferno
  python3 python/tools/walk_nav.py -r data/maps/de_dust2_lower_tunel1.json
  python3 python/tools/walk_nav.py -r "data/maps/de_dust2_*.json"   # склеить всё на лету
  python3 python/tools/walk_nav.py --png /tmp/route.png     # сохранить карту+маршрут перед стартом
  python3 python/tools/walk_nav.py --nav data/maps/de_cache.nav.json
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.input import create_input
from cs2_nav.coord_map import CoordinateMap
from cs2_nav.navigator import NavConfig, Navigator
from cs2_reader.reader import Reader

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def resolve_recordings(patterns: list[str]) -> list[str]:
    paths: list[str] = []
    for pat in patterns:
        if not os.path.isabs(pat):
            pat = os.path.join(ROOT, pat)
        found = sorted(glob.glob(pat))
        if not found and os.path.exists(pat):
            found = [pat]
        paths.extend(found)
    return sorted(set(paths))


def main() -> int:
    parser = argparse.ArgumentParser(description="Навигация по записям координат")
    parser.add_argument(
        "--recording", "-r",
        action="append",
        help="JSON записи (glob, можно несколько). По умолчанию вся de_inferno",
    )
    parser.add_argument(
        "--nav",
        default=None,
        help="Разобранный .nav.json из VPK (вместо записи прогулки)",
    )
    parser.add_argument("--force-nav", action="store_true", help="Не сверять имя навмеша с картой в игре")
    parser.add_argument("--duration", type=float, default=120.0, help="Лимит сек (0 = без лимита)")
    parser.add_argument("--countdown", type=int, default=5)
    parser.add_argument("--hz", type=float, default=40.0)
    parser.add_argument("--no-loop", action="store_true", help="Остановиться на первой цели, не брать следующую")
    parser.add_argument("--goal", type=int, default=None, help="id узла-цели (по умолчанию случайная точка)")
    parser.add_argument("--xdotool-mouse", action="store_true", help="Мышь через xdotool (если uinput не доходит до CS2)")
    parser.add_argument("--png", default=None, help="Сохранить карту с маршрутом перед стартом")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--no-engage", action="store_true", help="Не смотреть на мишени (только обход)")
    parser.add_argument("--no-fire", action="store_true", help="Наводиться на мишени, но не стрелять")
    parser.add_argument("--shots", type=int, default=0, help="Лимит выстрелов по мишени (0 = стрелять, пока не пропадёт)")
    parser.add_argument("--quiet", action="store_true", help="Короткий лог")
    args = parser.parse_args()

    if os.geteuid() == 0:
        print("Не запускай от root", file=sys.stderr)
        return 1

    nav_path = None
    if args.nav:
        nav_path = args.nav if os.path.isabs(args.nav) else os.path.join(ROOT, args.nav)
        if not os.path.isfile(nav_path):
            print(f"Нет файла навмеша: {nav_path}", file=sys.stderr)
            return 1
        recs = []
    else:
        recs = resolve_recordings(args.recording or ["data/maps/de_inferno_de_inferno.json"])
        if args.recording and not recs:
            print("Записи не найдены", file=sys.stderr)
            return 1

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    pawn_ok = False
    for i in range(45):
        if reader.local_pawn():
            pawn_ok = True
            break
        map_name = reader.map_name() or "—"
        ctrl = "да" if reader.local_controller() else "нет"
        print(f"Ожидание пешки… карта={map_name} controller={ctrl}")
        if i == 4 and map_name == "—":
            print(
                "Карта не читается — оффсеты, скорее всего, устарели после патча CS2.\n"
                "Обнови: python3 scripts/update_offsets.py  (CS2 должен быть запущен)",
                file=sys.stderr,
            )
        time.sleep(1)
    if not pawn_ok:
        print("Пешка так и не появилась. Зайди в матч или обнови оффсеты.", file=sys.stderr)
        return 1

    live_map = (reader.map_name() or "").lower().replace("maps/", "").removesuffix(".vpk")
    if nav_path is None and not args.recording and live_map:
        # --nav/-r не заданы: берём навмеш текущей карты, если он есть
        auto = os.path.join(ROOT, "data", "maps", f"{live_map}.nav.json")
        if os.path.isfile(auto):
            nav_path = auto
            print(f"Карта в игре: {live_map} → навмеш {os.path.relpath(auto, ROOT)}")
    if nav_path and live_map:
        nav_map = os.path.basename(nav_path).split(".")[0].lower()
        if nav_map != live_map and not args.force_nav:
            print(
                f"Навмеш {nav_map} не совпадает с картой в игре ({live_map}). "
                f"Укажи --nav data/maps/{live_map}.nav.json или --force-nav.",
                file=sys.stderr,
            )
            return 1

    if nav_path:
        print(f"Сборка карты из навмеша {nav_path}…")
        cmap = CoordinateMap.from_navmesh(nav_path)
    elif recs:
        cmap = CoordinateMap.from_recording(recs)
    else:
        print(
            f"Нет навмеша для карты {live_map or '?'} (data/maps/<карта>.nav.json) и нет записей. "
            "Укажи --nav или -r.",
            file=sys.stderr,
        )
        return 1
    print(f"Карта {cmap.map_name}/{cmap.location}: {cmap.stats()}")

    cfg = NavConfig(
        tick_hz=args.hz,
        duration=args.duration or None,
        loop=not args.no_loop,
        seed=args.seed,
        verbose_log=not args.quiet,
        engage_targets=not args.no_engage,
        fire_enabled=not args.no_fire,
        shots_per_target=args.shots,
    )
    inp = create_input(prefer_xdotool_mouse=args.xdotool_mouse)
    inp.prepare()
    nav = Navigator(reader, inp, cmap, cfg)

    pos = reader.local_position()
    route = nav.plan((pos.x, pos.y, pos.z), goal_id=args.goal)
    print(f"Старт ({pos.x:.0f},{pos.y:.0f},{pos.z:.0f}) → цель id={nav.goal_id}, маршрут {route.length:.0f}u, точек {len(route.pts)}")

    if args.png:
        from cs2_nav.render import render_map_png

        render_map_png(cmap, args.png, route=route.pts, goal=(route.pts[-1][0], route.pts[-1][1]))
        print(f"PNG → {args.png}")

    print("Мышь и yaw-offset калибруются на лету по памяти игры; клавиши W/A/S/D + Space.")
    if not cfg.engage_targets:
        mode = "только обход"
    elif cfg.fire_enabled:
        limit = f"{cfg.shots_per_target} выстрела" if cfg.shots_per_target > 0 else "стрелять, пока не пропадёт"
        mode = f"остановиться, флик на мишень, {limit} (ЛКМ), идти дальше + обход как препятствия"
    else:
        mode = "взгляд на видимую мишень + обход как препятствия"
    print("Мишени: " + mode)
    for sec in range(args.countdown, 0, -1):
        print(f"  Старт через {sec}… (CS2 в фокусе)")
        time.sleep(1)

    try:
        nav.run(duration=cfg.duration)
    finally:
        inp.close()
    if nav.events:
        print("События:", "; ".join(nav.events[-10:]))
    if nav.targets_engaged:
        print(f"Мишени: наводок {nav.targets_engaged}, отстреляно {nav.targets_shot}, выстрелов {nav.shots_fired}, "
              f"тиков со взглядом на мишень {nav.engaged_ticks}, запомнено позиций {len(nav.known_targets)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
