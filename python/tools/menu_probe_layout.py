#!/usr/bin/env python3
"""Показать куда бот будет кликать — только двигает курсор, без кликов."""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu import MenuController
from cs2_menu.layout import LAYOUT, TRAINING_MIRAGE_CLICKS
from cs2_reader.reader import Reader

PLAY_HUB_TARGETS = ["nav_play", "mode_practice", "mode_deathmatch", "mode_premier"]
TRAINING_SCREEN_TARGETS = [
    "practice_map_mirage",
    "practice_mode_competitive",
    "practice_start",
    "back_button",
]
TARGETS = list(TRAINING_MIRAGE_CLICKS) + [
    "mode_deathmatch",
    "mode_premier",
    "go_button",
    "back_button",
]


def main() -> int:
    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    menu = MenuController(reader, dry_run=False)
    menu.executor.refresh_window()
    state = menu.where()
    layout = menu.executor.layout(state)
    if not layout:
        print("Не удалось получить геометрию окна", file=sys.stderr)
        return 1

    print("=" * 60)
    print("Probe layout — цепочка тренировки + карточки Play hub")
    print(f"screen={state.screen_id} client={state.window_memory}")
    print(f"geometry={layout.describe()}")
    print("=" * 60)

    pause = 2.5 if "--slow" in sys.argv else 1.5
    if "--training-screen" in sys.argv:
        targets = TRAINING_SCREEN_TARGETS
        print("Режим: экран «Тренировка» уже открыт — калибруем карту/режим/старт\n")
    elif "--play-hub" in sys.argv:
        targets = PLAY_HUB_TARGETS
        print("Режим: главное меню / Play hub\n")
    else:
        targets = TARGETS

    for target in targets:
        pt = LAYOUT[target]
        cx, cy = layout.norm_to_client(pt)
        wx, wy = layout.norm_to_window_rel(pt)
        sx, sy = layout.norm_to_screen_abs(pt)
        in_client = layout.window_rel_in_client(wx, wy)
        print(
            f"\n→ {target}: norm=({pt.x:.3f},{pt.y:.3f}) "
            f"client=({cx},{cy}) window=({wx},{wy}) screen=({sx},{sy}) "
            f"{'OK' if in_client else 'OUT!'}"
        )
        menu.executor.move_target(target, state)
        time.sleep(pause)

    print("\nГотово. Цепочка тренировки — первые 5 точек.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
