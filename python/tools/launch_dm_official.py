#!/usr/bin/env python3
"""Запуск ОФИЦИАЛЬНОГО дезматча из любого места меню CS2, обходя попапы/ошибки.

Поток (устойчив к стартовому экрану — верхняя навигация видна всегда):
    [чистим окна] → PLAY → MATCHMAKING → DEATHMATCH → выбор набора карт
    (Defusal Group Alpha: Mirage/Dust2/Inferno/Vertigo/Cache) → GO → [чистим окна] → бой.

На КАЖДОМ шаге сначала закрываем всплывающие окна (ошибки/промо) детектором яркости
(cs2_menu.modal_vision): ESC/Enter их не берут, нужен клик по кнопке. Координаты —
доли КОНТЕНТА окна CS2 (без титульной полосы), найденного парсингом окна, поэтому
мапинг точен и не зависит ни от положения окна, ни от разрешения/размера.

Официальный матчмейкинг требует актуального клиента. Если CS2 устарел, после GO
появится «Matchmaking Failed» — скрипт её закроет и честно сообщит, что матч не
стартовал (нужно обновить игру). Если же очередь пошла и карта загрузилась —
подтвердит вход по памяти/экрану.

  python3 python/tools/launch_dm_official.py [--shots]
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu import MenuController
from cs2_menu.driver import UIStateReader
from cs2_menu.window_signatures import close_window
from cs2_menu.xwl_pointer import ScreenClicker, ensure_flat_accel, root_shot
from cs2_reader.reader import Reader

# Доли КОНТЕНТА окна CS2 (fx, fy) — отсчёт от игрового контента (без титульной полосы),
# поэтому устойчивы к смене разрешения/размера окна. Калиброваны парсингом окна.
CLICKS: dict[str, tuple[float, float]] = {
    "play": (0.509, 0.030),            # верхняя навигация: PLAY
    "matchmaking": (0.423, 0.077),     # подвкладка: MATCHMAKING
    "deathmatch": (0.518, 0.116),      # режим: DEATHMATCH
    "map_group_alpha": (0.290, 0.520), # первый набор карт: Defusal Group Alpha
                                       # (Dust II, Mirage, Inferno, Vertigo, Cache)
    "go": (0.852, 0.925),              # кнопка GO (поиск матча)
    "auto_select": (0.935, 0.930),     # экран выбора команды: AUTO-SELECT (низ справа)
}

# «пустые»/мусорные значения карты в меню
_EMPTY_MAP = {"", "<empty>"}


def log(msg: str) -> None:
    print(f"[dm] {msg}", flush=True)


def _signals(reader: Reader) -> dict:
    """Чёткие признаки состояния матча ПО ПАМЯТИ (а не по видео).

    * map_loaded — загружена валидная карта (а не меню);
    * team       — команда ПО PAWN  (2=T, 3=CT, 0/1 = не назначена/спектатор);
    * ctrl_team  — команда ПО CONTROLLER (назначается раньше pawn-а);
    * alive      — СТРОГИЙ спавн: health>0 И life_state=alive И controller.m_bPawnIsAlive;
    * alive_fast — БЫСТРЫЙ спавн: health>0 И life_state=alive (без флага контроллера,
                   который иногда отстаёт на пару тиков — отсюда «задержка» спавна);
    * life_state — 0=жив, 1=умирает, 2=мёртв/не в игре, 3/4=респавн.
    """
    s = reader.schema

    def _ri32(addr: int, off: int) -> int:
        if not addr:
            return 0
        try:
            return reader.backend.read_i32(addr + off) or 0
        except Exception:  # noqa: BLE001
            return 0

    try:
        mp = reader.map_name()
    except Exception:  # noqa: BLE001
        mp = ""
    pawn = reader.local_pawn() or 0
    ctrl = reader.local_controller() or 0
    team = _ri32(pawn, s.m_i_team_num)
    ctrl_team = _ri32(ctrl, s.m_i_team_num)
    life = reader.read_local_life()
    map_loaded = _valid_map(mp)
    # «быстрый» спавн — не ждём отстающий флаг контроллера, но страхуемся картой
    alive_fast = map_loaded and life.health > 0 and life.life_state == 0
    return {
        "map": mp,
        "map_loaded": map_loaded,
        "pawn": pawn,
        "ctrl": ctrl,
        "team": team,
        "ctrl_team": ctrl_team,
        "alive": bool(life.alive),
        "alive_fast": bool(alive_fast),
        "pawn_alive": bool(life.pawn_alive),
        "health": int(life.health),
        "life_state": int(life.life_state),
    }


def _valid_map(name: str) -> bool:
    if name in _EMPTY_MAP:
        return False
    # имя карты из памяти должно быть печатным ascii вида de_/dm_/ar_/cs_
    return name.isprintable() and all(32 <= ord(c) < 127 for c in name) and len(name) <= 40


def normalize_map(name: str) -> str:
    """de_dust2 из 'maps/de_dust2.vpk' / мусора."""
    name = (name or "").strip().lower().replace("\\", "/")
    if "/" in name:
        name = name.rsplit("/", 1)[-1]
    if name.endswith(".vpk"):
        name = name[: -len(".vpk")]
    return name


def launch_until_spawn(
    reader: Reader,
    menu: MenuController,
    *,
    shots: bool = False,
    log_fn=log,
) -> int:
    """Меню → официальный DM (группа Alpha) → выбор команды → спавн.

    0 — заспавнился, 2 — карта не загрузилась, 3 — команда/спавн не случились.
    Меню не закрывает: вызывающий владеет MenuController и вводом.
    """
    driver = UIStateReader(reader)
    env = menu.executor._xdo_env()
    clicker = ScreenClicker(menu.executor.menu_input, env=env)

    def clear() -> None:
        close_window(clicker, env, reader_state_fn=lambda: driver.read().ui_fingerprint,
                     log=log_fn)

    def step(name: str, *, settle: float = 1.1) -> None:
        fx, fy = CLICKS[name]
        ok = clicker.click_frac(fx, fy, settle=settle)
        log_fn(f"click {name} -> {'ok' if ok else 'FAIL (нет окна CS2)'}")
        if shots:
            root_shot(env, f"/tmp/dmof_{name}.png")

    clear()
    step("play", settle=1.3)
    step("matchmaking", settle=1.1)
    step("deathmatch", settle=1.1)
    step("map_group_alpha", settle=1.0)
    step("go", settle=1.6)

    clear()
    log_fn("фаза 1/3: ждём загрузку карты (очередь → матч)…")
    map_start = time.monotonic()
    map_deadline = map_start + 120
    did_recheck = False
    while time.monotonic() < map_deadline:
        sig = _signals(reader)
        if sig["map_loaded"]:
            log_fn(f"карта загружена: {sig['map']!r}")
            break
        if not did_recheck and time.monotonic() - map_start > 18:
            clear()
            did_recheck = True
        time.sleep(2.0)
    else:
        root_shot(env, "/tmp/dmof_result.png")
        log_fn("карта не загрузилась за отведённое время — СТОП "
               "(ошибка матчмейкинга / нет сети / долгий поиск).")
        return 2

    code = wait_spawn(reader, menu, shots=shots, log_fn=log_fn)
    return code


def wait_spawn(
    reader: Reader,
    menu: MenuController,
    *,
    shots: bool = False,
    log_fn=log,
    timeout: float = 120.0,
) -> int:
    """Карта уже загружена: AUTO-SELECT, пока команда не назначена, затем спавн."""
    env = menu.executor._xdo_env()
    clicker = ScreenClicker(menu.executor.menu_input, env=env)
    log_fn("фаза 2/3: следим за памятью (team/life/spawn)…")
    spawn_deadline = time.monotonic() + timeout
    team_clicks = 0
    last_action = 0.0
    prev_key = None
    shot_team = False
    strict_at = fast_at = 0.0
    while time.monotonic() < spawn_deadline:
        sig = _signals(reader)
        now = time.monotonic()
        if not sig["map_loaded"]:
            log_fn("карта пропала до спавна — вернулись в меню")
            return 3
        if sig["alive_fast"] and not fast_at:
            fast_at = now
        if sig["alive"] and not strict_at:
            strict_at = now
        key = (sig["map_loaded"], sig["team"], sig["ctrl_team"],
               sig["alive"], sig["alive_fast"], sig["life_state"], sig["pawn_alive"])
        if key != prev_key:
            log_fn(f"  mem: map={sig['map']!r} pteam={sig['team']} cteam={sig['ctrl_team']} "
                   f"alive={sig['alive']} fast={sig['alive_fast']} ls={sig['life_state']} "
                   f"pa={sig['pawn_alive']} h={sig['health']}")
            prev_key = key
        if sig["alive_fast"]:
            break
        team_assigned = sig["team"] in (2, 3) or sig["ctrl_team"] in (2, 3)
        need_team = sig["map_loaded"] and not team_assigned
        if need_team:
            if not shot_team and shots:
                root_shot(env, "/tmp/dmof_teamselect.png")
                log_fn("  [shot] экран выбора команды → /tmp/dmof_teamselect.png")
            shot_team = True
            if team_clicks < 6 and now - last_action > 3:
                fx, fy = CLICKS["auto_select"]
                clicker.click_frac(fx, fy, settle=0.8)
                team_clicks += 1
                last_action = now
                log_fn(f"  выбор команды: клик AUTO-SELECT @({fx},{fy}) "
                       f"(попытка {team_clicks}/6)")
        time.sleep(0.25)

    if shots:
        root_shot(env, "/tmp/dmof_result.png")
    sig = _signals(reader)
    if strict_at and fast_at:
        log_fn(f"  (спавн: fast→strict задержка {strict_at - fast_at:.2f}с)")
    if sig["alive"] or sig["alive_fast"]:
        log_fn(f"ЗАСПАВНИЛСЯ ✓ map={sig['map']!r} team={sig['team']} "
               f"health={sig['health']} ls={sig['life_state']} pawn={sig['pawn']}")
        return 0
    log_fn(f"НЕ заспавнился: map={sig['map']!r} team={sig['team']} ctrl_team={sig['ctrl_team']} "
           f"alive={sig['alive']} life_state={sig['life_state']} — СТОП.")
    return 3


def main() -> int:
    shots = "--shots" in sys.argv
    ensure_flat_accel()

    reader = Reader.from_live()
    if not reader:
        log("CS2 не запущен")
        return 1
    menu = MenuController(reader)
    try:
        st0 = menu.driver.read()
        if st0.in_game:
            log("сейчас в игре — выйди в меню (Esc → Exit to main menu); продолжаю попытку")
        return launch_until_spawn(reader, menu, shots=shots, log_fn=log)
    finally:
        menu.close()


if __name__ == "__main__":
    raise SystemExit(main())
