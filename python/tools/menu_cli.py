#!/usr/bin/env python3
"""CLI для cs2_menu: статус, popups, сценарии."""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu import MenuController
from cs2_menu.popups import save_fingerprint
from cs2_reader.reader import Reader


def main() -> int:
    parser = argparse.ArgumentParser(description="CS2 menu automation")
    parser.add_argument("--dry-run", action="store_true", help="Не кликать, только лог")
    parser.add_argument("--json", action="store_true", help="JSON вывод")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="Текущее состояние UI")
    sub.add_parser("popups", help="Закрыть popups")
    sub.add_parser("scenarios", help="Список сценариев")

    run_p = sub.add_parser("run", help="Запустить сценарий")
    run_p.add_argument("scenario", help="id сценария (deathmatch, main_menu, …)")
    run_p.add_argument("--timeout", type=float, default=120.0)

    dm_p = sub.add_parser("deathmatch", help="Полный цикл start_deathmatch")
    dm_p.add_argument("--timeout", type=float, default=180.0)

    tr_p = sub.add_parser("training", help="Play → Тренировка → Mirage → Соревновательный → Старт")
    tr_p.add_argument("--timeout", type=float, default=120.0)
    tr_p.add_argument("--map", default="de_mirage")

    learn_p = sub.add_parser("learn", help="Запомнить текущий ui_fingerprint")
    learn_p.add_argument("kind", help="main_menu | notification_open | generic_popup | vote_poll | …")

    ls_p = sub.add_parser("learn-screen", help="Запомнить текущий экран меню (region hash)")
    ls_p.add_argument("screen_id", help="play_hub | store | news | mode_deathmatch | …")

    sub.add_parser("where", help="Где сейчас в меню (screen_id + геометрия)")

    sub.add_parser("probe", help="Провести курсором по точкам layout (без кликов)")
с
    args = parser.parse_args()
    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    menu = MenuController(reader, dry_run=args.dry_run)

    if args.cmd == "status":
        info = menu.describe()
        if args.json:
            print(json.dumps(info, indent=2, ensure_ascii=False))
        else:
            st = info["state"]
            print("=== UI State ===")
            for k, v in st.items():
                print(f"  {k}: {v}")
            if st.get("active_popups"):
                print(f"  popups: {st['active_popups']}")
        return 0

    if args.cmd == "scenarios":
        for sid in menu.list_scenarios():
            print(sid)
        return 0

    if args.cmd == "popups":
        n = menu.dismiss_popups()
        print(f"dismiss rounds: {n}")
        return 0

    if args.cmd == "run":
        ok = menu.run(args.scenario, timeout=args.timeout)
        return 0 if ok else 2

    if args.cmd == "deathmatch":
        ok = menu.start_deathmatch(timeout=args.timeout)
        return 0 if ok else 2

    if args.cmd == "training":
        menu.queue_training_mirage()
        ok = menu.wait_in_game(timeout=args.timeout, map_id=args.map)
        print("OK" if ok else "timeout / wrong map")
        return 0 if ok else 2

    if args.cmd == "learn":
        st = menu.read()
        save_fingerprint(args.kind, st.ui_fingerprint)
        print(f"saved {args.kind} fp={st.ui_fingerprint}")
        return 0

    if args.cmd == "learn-screen":
        regions = menu.learn_current_screen(args.screen_id)
        print(f"saved screen {args.screen_id}: {regions}")
        return 0

    if args.cmd == "probe":
        import subprocess

        probe = os.path.join(os.path.dirname(__file__), "menu_probe_layout.py")
        extra = ["--slow"] if "--slow" in sys.argv else []
        return subprocess.call([sys.executable, probe, *extra])

    if args.cmd == "where":
        st = menu.where()
        layout = menu.executor.layout(st)
        if args.json:
            out = st.to_dict()
            if layout:
                out["geometry"] = layout.describe()
            print(json.dumps(out, indent=2, ensure_ascii=False))
        else:
            print("=== Где в меню ===")
            print(f"  screen: {st.screen_id} ({st.screen_confidence:.0%})")
            print(f"  phase: {st.phase.value}")
            print(f"  client: {st.window_memory[0]}x{st.window_memory[1]}")
            if layout:
                g = layout.describe()
                print(f"  outer: {g['outer']}")
                print(f"  client_screen: {g['client_screen']}")
                print(f"  letterbox: {g['letterbox']}")
            if st.region_hashes:
                print(f"  patches: {st.region_hashes}")
            dbg = st.raw.get("screen_debug")
            if dbg:
                print(f"  debug: {dbg}")
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main())
