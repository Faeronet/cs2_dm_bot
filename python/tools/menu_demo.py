#!/usr/bin/env python3
"""Демо: обход меню CS2 (xdotool курсор + uinput клики)."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu import MenuController
from cs2_reader.reader import Reader

TOUR: list[tuple[str, str, str | None, list[str]]] = [
    ("Play", "nav_play", "play_hub", []),
    ("Deathmatch", "mode_deathmatch", "mode_deathmatch", []),
    ("Назад", "back_button", "play_hub", []),
    ("Store", "nav_store", "store", []),
    ("Inventory", "nav_inventory", "inventory", []),
    ("News (TV)", "nav_news", "news", []),
]


def main() -> int:
    if "--probe" in sys.argv:
        import subprocess

        probe = os.path.join(os.path.dirname(__file__), "menu_probe_layout.py")
        extra = ["--slow"] if "--slow" in sys.argv else []
        return subprocess.call([sys.executable, probe, *extra])

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    menu = MenuController(reader, dry_run="--dry-run" in sys.argv, step_delay=0.5)
    menu.executor.refresh_window()

    print("=" * 60)
    print("CS2 Menu Demo")
    print("  Ввод: xdotool (курсор) + evdev uinput (клики/клавиши)")
    print("  Переключись на CS2 — старт через 3 сек")
    print("=" * 60)
    for i in range(3, 0, -1):
        print(f"  {i}…")
        time.sleep(1)

    start = menu.where()
    layout = menu.executor.layout(start)
    print(f"\nСтарт: screen={start.screen_id}")
    print(f"       client {start.window_memory[0]}x{start.window_memory[1]}")
    if layout:
        print(f"       letterbox {layout.letterbox_x}x{layout.letterbox_y}")

    results: list[tuple[str, bool, str, str, str | None]] = []
    try:
        for title, target, expected, prep in TOUR:
            print(f"\n--- {title} ---")
            state = menu.where()
            print(f"  сейчас: {state.screen_id}")

            for p in prep:
                menu.click_target(p, timeout=5.0)

            ok = menu.click_target(target, expected_screen=expected, timeout=6.0)
            after = menu.where()
            print(f"  [{'OK' if ok else 'FAIL'}] {state.screen_id} -> {after.screen_id} (ждали {expected})")
            results.append((title, ok, state.screen_id, after.screen_id, expected))
    finally:
        menu.executor.close()

    print("\n" + "=" * 60)
    ok_n = sum(1 for _, ok, *_ in results if ok)
    print(f"ИТОГ: {ok_n}/{len(results)}")
    for title, ok, fr, to, exp in results:
        print(f"  [{'OK' if ok else 'FAIL'}] {title}: {fr} -> {to} (ждали {exp})")
    print("=" * 60)
    return 0 if ok_n == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
