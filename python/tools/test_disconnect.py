#!/usr/bin/env python3
"""Проверка выхода с сервера тем же путём, что и при повышении ранга.

CS2 уже должна быть в матче, внутри gamescope. Скрипт ждёт 3 секунды,
отпускает клавиши и вызывает leave_server. В очередь заново не встаёт.

  python3 python/tools/test_disconnect.py
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.ei_input import HybridInput
from cs2_control.gamescope_session import ensure_gamescope
from cs2_menu import MenuController
from cs2_menu.xwl_pointer import ensure_flat_accel
from cs2_reader.reader import Reader
from tools.launch_dm_official import _signals, normalize_map
from tools.run_dm import leave_server


def main() -> int:
    ensure_flat_accel()
    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1
    sig = _signals(reader)
    if not sig["map_loaded"]:
        print("Сейчас не на сервере — нечего отключать")
        return 2
    print(f"на сервере: карта {normalize_map(sig['map'])!r}")
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
        menu = MenuController(reader, game_input=inp, log=print)
        for left in (3, 2, 1):
            print(f"disconnect через {left}…")
            time.sleep(1.0)
        leave_server(menu, reader, inp)
        sig = _signals(reader)
        if sig["map_loaded"]:
            print(f"всё ещё на карте {normalize_map(sig['map'])!r}")
            return 2
        print("выход подтверждён: карты нет")
        return 0
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
