#!/usr/bin/env python3
"""Тест антиотдачи / тапов у стены: смотришь в стену, отсчёт, очередь как у бота.

Примеры:
  python3 python/tools/test_recoil.py
  python3 python/tools/test_recoil.py --once --countdown 3
  python3 python/tools/test_recoil.py --shots 15
  python3 python/tools/test_recoil.py --no-rcs          # зажим без ведения линии (для сравнения)
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.input import create_input
from cs2_nav.coord_map import CoordinateMap
from cs2_nav.navigator import NavConfig, Navigator
from cs2_reader.aim import shortest_yaw_err
from cs2_reader.recoil import recoil_pattern, tap_gap_range
from cs2_reader.reader import Reader
from cs2_reader.weapons import is_gun, weapon_is_auto


def _countdown(sec: int) -> None:
    for n in range(sec, 0, -1):
        print(f"  {n}…  (CS2 в фокусе, смотри в стену)", flush=True)
        time.sleep(1)
    print("  огонь", flush=True)


def _weapon_line(w) -> str:
    name = w.name or f"id={w.id}"
    if not is_gun(w.id, w.name):
        mode = "не стреляет"
    elif weapon_is_auto(w.id, w.name):
        mode = "зажим + линия отдачи"
    else:
        lo, hi = tap_gap_range(w.id, w.name)
        mode = f"тап, пауза {lo:.2f}–{hi:.2f}s"
    extra = ""
    if weapon_is_auto(w.id, w.name):
        pat = recoil_pattern(w.id, w.name)
        ys = [p[0] for p in pat]
        extra = f", линия ~{pat[-1][1]:.1f}° вниз, ширина ~{max(ys) - min(ys):.1f}°"
    rel = " перезарядка" if w.in_reload else ""
    return f"{name}  обойма={w.clip}{rel}  {mode}{extra}"


def _ask_next() -> bool:
    try:
        ans = input("Смени оружие, снова смотри в стену, Enter — тест, q — выход: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return ans not in {"q", "quit", "exit"}


def _run_burst(nav: Navigator, reader: Reader, inp, *, shots_limit: int, max_time: float, rcs: bool) -> None:
    w0 = reader.local_weapon()
    if not is_gun(w0.id, w0.name):
        print(f"  пропуск: {w0.name or 'пусто'} — не огнестрел")
        return
    if w0.in_reload or w0.clip <= 0:
        print("  обойма пуста или перезарядка — заряди и повтори")
        return

    auto = weapon_is_auto(w0.id, w0.name)
    want = shots_limit if shots_limit > 0 else w0.clip
    want = max(1, min(want, w0.clip))
    nav.cfg.rcs_enabled = rcs and auto
    nav._rcs_reset()
    nav._yaw_rate = nav._pitch_rate = nav._yaw_acc = nav._pitch_acc = 0.0
    nav._wind[:] = [0.0, 0.0]
    nav._px_carry[:] = [0.0, 0.0]
    nav._shots_on_target = 0
    nav._fire_held = False
    nav._next_fire_at = 0.0

    clip0 = w0.clip
    eye0 = reader.local_eye_angles()
    base_yaw, base_pitch = eye0.y, eye0.x
    hz = nav.cfg.tick_hz
    dt = 1.0 / hz
    t0 = time.monotonic()
    next_t = t0
    fired = 0
    taps = 0
    tap_down_until = -1.0
    idle_deadline = t0 + 0.6
    while reader.local_shots_fired() > 0 and time.monotonic() < idle_deadline:
        time.sleep(0.05)
    mode = "зажим" if auto else "тап"
    rcs_txt = "линия" if (rcs and auto) else ("без RCS" if auto else "сброс отдачи")
    print(f"  {w0.name}: {mode}, {want} выстр., {rcs_txt}")

    try:
        if auto:
            inp.mouse_down("left")
            nav._fire_held = True

        while True:
            now = time.monotonic()
            if now - t0 >= max_time:
                print("  лимит времени")
                break
            w = reader.local_weapon()
            if w.id != w0.id:
                print("  оружие сменили — стоп")
                break
            if w.in_reload or w.clip <= 0:
                fired = max(fired, clip0 - max(0, w.clip), taps)
                break
            fired = taps if not auto else (clip0 - w.clip)
            if fired >= want:
                break

            nav.weapon = w.to_dict()
            nav.weapon["auto"] = auto
            nav.weapon["fire"] = "auto" if auto else "tap"
            nav._mem_shots = nav._read_shots_fired()
            eye = reader.local_eye_angles()
            yaw, pitch = eye.y, eye.x

            if auto:
                ry, rp = nav._rcs_offset(now) if nav.cfg.rcs_enabled else (0.0, 0.0)
                yaw_err = shortest_yaw_err(base_yaw + ry, yaw)
                pitch_err = (base_pitch + rp) - pitch
                d_yaw, d_pitch = nav._rcs_step(yaw_err, pitch_err, dt)
                nav._send_mouse(d_yaw, d_pitch)
            else:
                if tap_down_until > 0 and now >= tap_down_until:
                    inp.mouse_up("left")
                    tap_down_until = -1.0
                    nav._fire_held = False
                    nav._next_fire_at = now + nav._tap_wait()
                elif tap_down_until < 0 and now >= nav._next_fire_at:
                    inp.mouse_down("left")
                    nav._fire_held = True
                    tap_down_until = now + dt * max(1, nav.cfg.fire_hold_ticks)
                    taps += 1
                    print(f"    тап {taps}/{want}  clip={w.clip}", flush=True)

            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()
    finally:
        nav._fire_held = False
        try:
            inp.mouse_up("left")
        except Exception:
            pass
        nav._rcs_reset()

    elapsed = time.monotonic() - t0
    w1 = reader.local_weapon()
    print(
        f"  готово: {fired} выстр. за {elapsed:.2f}s, "
        f"обойма {w0.clip}→{w1.clip}, shots_fired={reader.local_shots_fired()}"
    )
    print("  смотри след на стене (прицел не возвращаю — чтобы было видно линию)")


def main() -> int:
    parser = argparse.ArgumentParser(description="Тест антиотдачи/тапов по стене")
    parser.add_argument("--countdown", type=int, default=5, help="отсчёт секунд до очереди")
    parser.add_argument("--shots", type=int, default=0, help="сколько пуль (0 = вся обойма)")
    parser.add_argument("--max-time", type=float, default=12.0, help="предохранитель, секунд на одну очередь")
    parser.add_argument("--hz", type=float, default=40.0)
    parser.add_argument("--once", action="store_true", help="одно оружие и выход")
    parser.add_argument("--no-rcs", action="store_true", help="зажимать без ведения линии")
    parser.add_argument("--xdotool-mouse", action="store_true")
    parser.add_argument("--yaw-per-px", type=float, default=None, help="°/px мыши (как в игре)")
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    if os.geteuid() == 0:
        print("Не запускай от root", file=sys.stderr)
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
        print("Нет локального игрока", file=sys.stderr)
        return 1

    cfg = NavConfig(
        tick_hz=args.hz,
        seed=args.seed,
        verbose_log=False,
        rcs_enabled=not args.no_rcs,
        habits_enabled=False,
    )
    if args.yaw_per_px is not None:
        cfg.yaw_per_px = args.yaw_per_px
        cfg.pitch_per_px = abs(args.yaw_per_px)

    inp = create_input(prefer_xdotool_mouse=args.xdotool_mouse)
    inp.prepare()
    nav = Navigator(reader, inp, CoordinateMap("wall", "recoil-test"), cfg)

    print("Тест стены: бот стреляет тем, что в руках (зажим ведёт линию, тап ждёт сброс отдачи).")
    print("Встань у стены, прицелься в точку, не двигайся. Между тестами можно менять оружие.")
    first = True
    try:
        while True:
            if not first and not args.once:
                if not _ask_next():
                    break
            first = False
            w = reader.local_weapon()
            print(_weapon_line(w))
            _countdown(max(0, args.countdown))
            _run_burst(
                nav, reader, inp,
                shots_limit=args.shots,
                max_time=args.max_time,
                rcs=not args.no_rcs,
            )
            if args.once:
                break
    except KeyboardInterrupt:
        print("\nстоп")
    finally:
        inp.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
