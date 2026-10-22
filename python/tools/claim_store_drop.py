#!/usr/bin/env python3
"""Магазин Home: прокликать иконки дропа и навестись на «Получить».

Кнопку «Получить» скрипт не нажимает — только ставит на неё курсор.

Нужен уже запущенный CS2 в меню (gamescope). Из корня репозитория:

  python3 python/tools/claim_store_drop.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu import MenuController
from cs2_menu.window_signatures import review_store_drop
from cs2_menu.xwl_pointer import ScreenClicker, ensure_flat_accel
from cs2_reader.reader import Reader


def main() -> int:
    ensure_flat_accel()
    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1
    menu = MenuController(reader)
    env = menu.executor._xdo_env()
    clicker = ScreenClicker(menu.executor.menu_input, env=env)
    try:
        ok = review_store_drop(clicker)
        return 0 if ok else 2
    finally:
        menu.close()


if __name__ == "__main__":
    raise SystemExit(main())
