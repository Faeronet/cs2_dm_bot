#!/usr/bin/env python3
"""Магазин Home: прокликать иконки дропа и навестись на «Получить».

Кнопку «Получить» скрипт не нажимает — только ставит на неё курсор.
CS2 уже должна быть открыта в главном меню, внутри gamescope.

  python3 python/tools/claim_store_drop.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.ei_input import HybridInput
from cs2_control.gamescope_session import ensure_gamescope
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
    try:
        ei_socket = ensure_gamescope(print)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    inp = None
    menu = None
    try:
        inp = HybridInput(ei_socket)
        inp.prepare()
        menu = MenuController(reader, game_input=inp)
        state = menu.driver.read()
        if state.in_game:
            print("Сейчас идёт матч. Магазин Home открывается из главного меню.")
            return 2
        env = menu.executor._xdo_env()
        clicker = ScreenClicker(menu.executor.menu_input, env=env)
        ok = review_store_drop(clicker)
        return 0 if ok else 2
    except (OSError, TimeoutError) as exc:
        print(f"виртуальная мышь gamescope не поднялась: {exc}", file=sys.stderr)
        return 1
    finally:
        if menu is not None:
            menu.close()
        if inp is not None:
            inp.close()


if __name__ == "__main__":
    raise SystemExit(main())
