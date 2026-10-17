#!/usr/bin/env python3
"""Запуск тренировки «Соревновательный + Mirage» из меню CS2 и ожидание входа в игру.

Поток меню (проверено): PLAY → PRACTICE → COMPETITIVE → Mirage → GO → AUTO SELECT.
Ввод — надёжный XWayland-способ (см. cs2_menu.xwl_pointer): курсор относительными
дельтами uinput при flat-ускорении, скриншот ffmpeg x11grab. Точки клика заданы
долями КОНТЕНТА окна CS2 (без титульной полосы, определяется парсингом окна),
поэтому мапинг точен и не зависит ни от положения окна, ни от разрешения/размера.

Проверка результата — из памяти процесса: map_name()=='de_mirage' и in_game=True.

  python3 python/tools/launch_training_mirage.py            # запустить и дождаться
  python3 python/tools/launch_training_mirage.py --shots    # + скриншоты каждого шага
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu import MenuController
from cs2_menu.driver import UIStateReader
from cs2_menu.modal_vision import clear_modals
from cs2_menu.xwl_pointer import ScreenClicker, ensure_flat_accel, root_shot
from cs2_reader.reader import Reader

# Доли КОНТЕНТА окна CS2 (fx, fy) ∈ [0,1] — отсчёт от игрового контента (без титульной
# полосы), устойчивы к смене разрешения/размера окна.
CLICKS: dict[str, tuple[float, float]] = {
    "play": (0.509, 0.030),         # верхняя навигация: PLAY
    "practice": (0.482, 0.075),     # подвкладка: PRACTICE
    "competitive": (0.330, 0.116),  # режим-правила: COMPETITIVE
    "mirage": (0.515, 0.315),       # плитка карты Mirage (4-я в ряду)
    "go": (0.852, 0.925),           # кнопка GO
    "auto_select": (0.908, 0.935),  # выбор команды: AUTO SELECT
}


def log(msg: str) -> None:
    print(f"[launch] {msg}", flush=True)


def wait_map(reader: Reader, target: str = "de_mirage", timeout: float = 90.0) -> bool:
    """Ждём, пока карта загрузится (map_name()==target)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            if reader.map_name() == target:
                return True
        except Exception:
            pass
        time.sleep(1.0)
    return False


def wait_in_game(driver: UIStateReader, reader: Reader, timeout: float = 30.0) -> bool:
    """Ждём спавна: in_game=True и валидный local_pawn."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = driver.read()
        pawn = reader.local_pawn() or 0
        if st.in_game and pawn > 0x10000:
            return True
        time.sleep(1.0)
    return False


def main() -> int:
    shots = "--shots" in sys.argv

    ensure_flat_accel()

    reader = Reader.from_live()
    if not reader:
        log("CS2 не запущен")
        return 1
    menu = MenuController(reader)
    driver = UIStateReader(reader)
    env = menu.executor._xdo_env()
    clicker = ScreenClicker(menu.executor.menu_input, env=env)

    def step(name: str, *, pre_wait: float = 0.0, settle: float = 1.0) -> None:
        if pre_wait:
            time.sleep(pre_wait)
        fx, fy = CLICKS[name]
        ok = clicker.click_frac(fx, fy, settle=settle)
        log(f"click {name} -> {'ok' if ok else 'FAIL (нет окна CS2)'}")
        if shots and root_shot(env, f"/tmp/launch_{name}.png"):
            log(f"  shot /tmp/launch_{name}.png")

    try:
        # Закрыть залипшие модалки (ошибки/подтверждения), чтобы не мешали
        clear_modals(clicker, env, log=log)

        # На PLAY-экран (сброс к матчмейкингу), затем на PRACTICE
        step("play", settle=1.2)
        step("practice", settle=1.2)
        step("competitive", settle=1.0)
        step("mirage", settle=1.0)
        step("go", settle=1.5)

        # После GO возможна модалка (например «Matchmaking Failed») — закрыть
        clear_modals(clicker, env, log=log)

        log("ждём загрузку карты de_mirage…")
        if not wait_map(reader, "de_mirage", timeout=90):
            log("карта не загрузилась за 90с")
            return 2
        log("карта de_mirage загружена")

        # Экран выбора команды — AUTO SELECT (обычно автозаход, но подстрахуемся)
        step("auto_select", pre_wait=1.0, settle=1.5)

        if wait_in_game(driver, reader, timeout=30):
            st = driver.read()
            log(f"В ИГРЕ: map={reader.map_name()!r} pawn={reader.local_pawn()} phase={st.phase.value}")
            if shots:
                root_shot(env, "/tmp/launch_ingame.png")
            return 0
        log("карта загружена, но спавн не подтверждён (проверь экран команды)")
        return 3
    finally:
        menu.close()


if __name__ == "__main__":
    raise SystemExit(main())
