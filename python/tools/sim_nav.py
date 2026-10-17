#!/usr/bin/env python3
"""
Оффлайн-симуляция навигации (без CS2): карта → дальняя точка → маршрут → движение.
Печатает метрики и сохраняет PNG (карта, маршрут синим, пройденный путь оранжевым,
направление камеры белыми штрихами каждые 0.5 с).

Примеры:
  python3 python/tools/sim_nav.py
  python3 python/tools/sim_nav.py --nav data/maps/de_vertigo.nav.json --duration 90
  python3 python/tools/sim_nav.py --start -1250,1120,20 --yaw 90
  python3 python/tools/sim_nav.py --random-starts 6 --seed 3
  python3 python/tools/sim_nav.py --invert-mouse --mem-yaw-offset 90   # проверка автокалибровки
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_nav.coord_map import CoordinateMap, MapConfig
from cs2_nav.navigator import NavConfig
from cs2_nav.render import render_map_png
from cs2_nav.simulator import NavSimulator, SimConfig, SimResult

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def resolve_recordings(spec: str) -> list[str]:
    paths: list[str] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        p = part if os.path.isabs(part) else os.path.join(ROOT, part)
        hits = sorted(glob.glob(p))
        paths.extend(hits or [p])
    return paths


def summary(res: SimResult) -> str:
    status = "OK" if res.done else "НЕ ДОШЁЛ"
    eff = res.walked_len / res.route_len if res.route_len else 0.0
    return (
        f"{status}: {res.time:.1f}s, маршрут {res.route_len:.0f}u, пройдено {res.walked_len:.0f}u "
        f"(x{eff:.2f}), до цели {res.final_goal_dist:.0f}u | min clearance {res.min_clearance:.0f}u, "
        f"тиков <12u: {res.low_clearance_ticks}, стоял: {res.stop_ticks} тиков | "
        f"yaw rate max {res.max_yaw_rate:.0f}°/s, в движении p95 {res.moving_yaw_rate_p95:.0f}°/s | "
        f"клавиш: {res.key_events}, застреваний: {res.stucks}, перестроений: {res.replans}, "
        f"столкновений: {res.collisions} | mouse k={res.yaw_per_px_final:+.4f} yaw_offset={res.yaw_offset_detected:+.0f}"
        + (
            f"\n  мишени: взгляд на мишень {res.engaged_ticks * res.time / max(1, res.ticks):.1f}s "
            f"({res.targets_engaged} наводок), наводка за {res.aim_settle_time:.2f}s, "
            f"ошибка прицела p50 {res.aim_err_p50:.1f}°, касаний мишени: {res.target_collisions}"
            + (
                f"\n  стрельба: {res.shots} выстрелов, попаданий {res.hits}, мишеней снято {res.targets_killed}; "
                f"первый выстрел через {res.first_shot_delay:.2f}s после появления, снятие за {res.kill_time:.2f}s; "
                f"пик флика {res.flick_peak_rate:.0f}°/s, скорость при выстреле {res.move_speed_at_shot:.0f}u/s, "
                f"в бою {res.fire_ticks * res.time / max(1, res.ticks):.1f}s"
                if res.shots else ""
            )
            if res.targets else ""
        )
        + f"\n  плавность: ускорение p95 {res.yaw_accel_p95:.0f}°/s², рывок RMS {res.yaw_jerk_rms / 1000:.0f}k°/s³, "
          f"смен направления {res.rate_sign_flips_per_s:.2f}/s"
        + (f", кривизна траектории прицела {res.path_curvature:.1f}°/шаг" if res.targets else "")
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Симуляция навигации по recording JSON")
    parser.add_argument(
        "--recording", "-r",
        default="data/maps/de_inferno_de_inferno.json",
        help="путь(и) через запятую или glob — несколько записей объединяются",
    )
    parser.add_argument(
        "--nav",
        default=None,
        help="Разобранный .nav.json (вместо записи прогулки)",
    )
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--start", default=None, help="x,y,z (по умолчанию — первая точка записи)")
    parser.add_argument("--yaw", type=float, default=0.0, help="начальный yaw камеры")
    parser.add_argument("--random-starts", type=int, default=0, help="N случайных стартов вместо --start")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out", default="data/sim/last_run.png")
    parser.add_argument("--hz", type=float, default=40.0)
    parser.add_argument("--loop", type=int, default=0, help="сколько целей подряд (0 = одна)")
    parser.add_argument("--invert-mouse", action="store_true")
    parser.add_argument("--mem-yaw-offset", type=float, default=0.0, help="смещение yaw в «памяти»")
    parser.add_argument("--game-sens", type=float, default=0.055, help="°/px в «игре»")
    parser.add_argument("--noise", type=float, default=0.0, help="шум позиции, u")
    parser.add_argument("--targets", default=None, help="мишени: 'x,y,z;x,y,z' (ноги)")
    parser.add_argument("--random-targets", type=int, default=0, help="N мишеней в случайных точках карты")
    parser.add_argument("--no-engage", action="store_true", help="не смотреть на мишени (только обход)")
    parser.add_argument("--no-fire", action="store_true", help="смотреть на мишени, но не стрелять")
    parser.add_argument("--shots", type=int, default=0, help="лимит выстрелов по мишени (0 = пока не пропадёт)")
    parser.add_argument("--weapon", default="ak47", help="оружие в руках: имя или id (ak47, usp, awp, ...)")
    parser.add_argument("--clip", type=int, default=None, help="патронов в обойме (по умолчанию полный магазин)")
    parser.add_argument("--hits-to-kill", type=int, default=3, help="попаданий, после которых мишень-макет пропадает (0 = никогда)")
    parser.add_argument("--respawn", type=float, default=6.0, help="с до повторного появления мишени (0 = навсегда)")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    if args.nav:
        nav_path = args.nav if os.path.isabs(args.nav) else os.path.join(ROOT, args.nav)
        print(f"Сборка карты из навмеша {nav_path}…")
        cm = CoordinateMap.from_navmesh(nav_path, MapConfig())
    else:
        recs = resolve_recordings(args.recording)
        cm = CoordinateMap.from_recording(recs if len(recs) > 1 else recs[0], MapConfig())
    print(f"Карта {cm.map_name}/{cm.location}: {json.dumps(cm.stats(), ensure_ascii=False)}")

    rng = random.Random(args.seed)
    starts: list[tuple[tuple[float, float, float], float]] = []
    if args.random_starts > 0:
        good = [n for n in cm.nodes if n.clearance >= 16.0]
        for _ in range(args.random_starts):
            n = rng.choice(good)
            starts.append(((n.x, n.y, n.z), rng.uniform(-180.0, 180.0)))
    elif args.start:
        sx, sy, sz = (float(v) for v in args.start.split(","))
        starts.append(((sx, sy, sz), args.yaw))
    else:
        pool = [n for n in cm.nodes if n.recorded] or list(cm.nodes)
        n = max(pool, key=lambda n: n.clearance)
        starts.append(((n.x, n.y, n.z), args.yaw))

    targets: list[tuple[float, float, float]] = []
    if args.targets:
        for part in args.targets.split(";"):
            if part.strip():
                tx, ty, tz = (float(v) for v in part.split(","))
                targets.append((tx, ty, tz))
    if args.random_targets > 0:
        # мишени на записанных клетках с запасом от стен, не ближе 200u к старту
        trng = random.Random(args.seed + 1000)
        pool = [n for n in cm.nodes if n.recorded and n.clearance >= 20.0]
        s0 = starts[0][0]
        pool = [n for n in pool if math.hypot(n.x - s0[0], n.y - s0[1]) > 200.0]
        while len(targets) < args.random_targets and pool:
            n = trng.choice(pool)
            if all(math.hypot(n.x - t[0], n.y - t[1]) > 120.0 for t in targets):
                targets.append((n.x, n.y, n.z))
            pool.remove(n)
    if targets:
        print("Мишени:", ", ".join(f"({t[0]:.0f},{t[1]:.0f},{t[2]:.0f})" for t in targets))

    ok = 0
    out_base = args.out if os.path.isabs(args.out) else os.path.join(ROOT, args.out)
    for i, (start, yaw0) in enumerate(starts):
        nav_cfg = NavConfig(
            tick_hz=args.hz,
            verbose_log=False,
            seed=args.seed + i,
            loop=args.loop > 0,
            engage_targets=not args.no_engage,
            fire_enabled=not args.no_fire,
            shots_per_target=args.shots,
            habits_enabled=False,
        )
        sim_cfg = SimConfig(
            deg_per_px=args.game_sens,
            invert_mouse=args.invert_mouse,
            yaw_offset=args.mem_yaw_offset,
            pos_noise=args.noise,
            targets=targets,
            target_hits_to_kill=args.hits_to_kill,
            target_respawn=args.respawn,
            weapon=args.weapon,
            weapon_clip=args.clip,
        )
        sim = NavSimulator(cm, start, nav_config=nav_cfg, sim_config=sim_cfg, initial_yaw=yaw0)
        res = sim.run(duration=args.duration, log_every=int(args.hz))
        print(f"\n[{i + 1}/{len(starts)}] старт ({start[0]:.0f},{start[1]:.0f},{start[2]:.0f}) yaw={yaw0:.0f}")
        print("  " + summary(res))
        if not args.quiet:
            for e in res.events:
                print("   •", e)
            for line in res.log[-4:]:
                print("   ", line)
        if res.done:
            ok += 1
        out = out_base if len(starts) == 1 else out_base.replace(".png", f"_{i + 1}.png")
        render_map_png(
            cm,
            out,
            route=res.route,
            walked=res.walked,
            headings=res.headings,
            goal=(res.goal[0], res.goal[1]) if res.goal else None,
            targets=res.targets,
            aim_lines=res.aim_lines,
            shot_lines=res.shot_lines,
            target_radius=sim_cfg.target_radius,
        )
        print(f"  PNG → {out}")

    print(f"\nИтог: {ok}/{len(starts)} дошли")
    return 0 if ok == len(starts) else 1


if __name__ == "__main__":
    raise SystemExit(main())
