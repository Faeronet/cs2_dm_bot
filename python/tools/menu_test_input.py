#!/usr/bin/env python3
"""Быстрый тест: uinput-клик по Play. CS2 должен быть на главном меню и в фокусе."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu import MenuController
from cs2_reader.reader import Reader


def main() -> int:
    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    menu = MenuController(reader, step_delay=0.5)
    menu.executor.refresh_window()

    print("=" * 50)
    print("Тест uinput: клик nav_play")
    print("Переключись на CS2 — старт через 3 сек")
    print("=" * 50)
    for i in range(3, 0, -1):
        print(f"  {i}…")
        time.sleep(1)

    before = menu.where()
    print(f"\nДо:  screen={before.screen_id}")

    try:
        ok = menu.click_target("nav_play", expected_screen="play_hub", timeout=6.0)
    finally:
        menu.executor.close()

    after = menu.where()
    print(f"После: screen={after.screen_id}")
    print(f"Результат: {'OK' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
