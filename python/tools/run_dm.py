#!/usr/bin/env python3
"""Официальный DM + бот-навигация в одном цикле.

* В меню (или ошибка/кик в меню) — закрыть окна и запустить дезматч
  (PLAY → MATCHMAKING → DEATHMATCH → Defusal Group Alpha → GO → команда).
* После спавна — бот по навмешу ТЕКУЩЕЙ карты (data/maps/<карта>.nav.json).
* Смена карты — выгрузить бота, собрать координатную карту новой карты, идти по ней.
* Смерть на той же карте бота не останавливает.

  python3 python/tools/run_dm.py --match-time 600
  python3 python/tools/run_dm.py --match-time 0    # без лимита
  python3 python/tools/run_dm.py --stop            # остановить уже запущенный бот
  python3 python/tools/run_dm.py --gameoff         # остановить бота и закрыть игру

Лимит считается только пока бот работает в матче (после спавна).
Меню, очередь и поиск матча в это время не входят. Новый матч — новый отсчёт.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import signal
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.ei_input import HybridInput
from cs2_control.gamescope_session import close_game, ensure_gamescope
from cs2_menu import MenuController
from cs2_menu.driver import UIStateReader
from cs2_menu.window_signatures import close_window
from cs2_menu.xwl_pointer import ScreenClicker, ensure_flat_accel
from cs2_nav.coord_map import CoordinateMap
from cs2_nav.navigator import NavConfig, Navigator
from cs2_reader.reader import Reader
from tools.launch_dm_official import (
    _signals,
    launch_until_spawn,
    normalize_map,
    wait_spawn,
)

def _repo_root() -> str:
    """Каталог репозитория: поднимаемся от скрипта, пока не найдём data/maps."""
    cur = os.path.abspath(os.path.dirname(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(cur, "data", "maps")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


ROOT = _repo_root()
MAPS_DIR = os.path.join(ROOT, "data", "maps")
_SCRIPT = os.path.abspath(__file__)


def _pid_path() -> str:
    """Один pid-файл на этот скрипт и этого пользователя."""
    digest = hashlib.sha1(_SCRIPT.encode()).hexdigest()[:12]
    return f"/tmp/cs2-dm-bot-{os.getuid()}-{digest}.pid"


def log(msg: str) -> None:
    print(f"[dm+bot] {msg}", flush=True)


def _proc_args(pid: int) -> list[str]:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            raw = fh.read()
    except OSError:
        return []
    parts = raw.split(b"\x00")
    if parts and parts[-1] == b"":
        parts = parts[:-1]
    return [part.decode("utf-8", "replace") for part in parts]


def _is_this_bot(pid: int) -> bool:
    """Процесс запущен этим же run_dm.py и это не команда --stop."""
    if pid <= 0 or pid == os.getpid():
        return False
    args = _proc_args(pid)
    if not args or "--stop" in args or "--gameoff" in args:
        return False
    try:
        cwd = os.readlink(f"/proc/{pid}/cwd")
    except OSError:
        cwd = "/"
    for arg in args:
        if not arg.endswith("run_dm.py"):
            continue
        candidate = arg if os.path.isabs(arg) else os.path.join(cwd, arg)
        try:
            if os.path.samefile(candidate, _SCRIPT):
                return True
        except OSError:
            continue
    return False


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _read_pidfile() -> int | None:
    try:
        pid = int(open(_pid_path(), encoding="utf-8").read().strip())
    except (OSError, ValueError):
        return None
    if _pid_alive(pid) and _is_this_bot(pid):
        return pid
    return None


def _scan_bot_pids() -> list[int]:
    found: list[int] = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if _is_this_bot(pid):
            found.append(pid)
    return found


def _running_pids() -> list[int]:
    pids: list[int] = []
    recorded = _read_pidfile()
    if recorded:
        pids.append(recorded)
    for pid in _scan_bot_pids():
        if pid not in pids:
            pids.append(pid)
    return pids


def _write_pidfile() -> None:
    path = _pid_path()
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
    try:
        os.write(fd, str(os.getpid()).encode())
    finally:
        os.close(fd)


def _clear_pidfile() -> None:
    path = _pid_path()
    try:
        current = int(open(path, encoding="utf-8").read().strip())
    except (OSError, ValueError):
        return
    if current != os.getpid():
        return
    try:
        os.remove(path)
    except OSError:
        pass


def _install_stop_signal() -> None:
    """SIGTERM от --stop и Ctrl+C выходят из любого места и отпускают клавиши в finally."""
    seen = {"on": False}

    def _handle(signum: int, _frame) -> None:
        if seen["on"]:
            return
        seen["on"] = True
        log("получен стоп")
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _handle)
    signal.signal(signal.SIGINT, _handle)


def stop_running() -> int:
    """Остановить уже запущенный этот же run_dm.py."""
    pids = _running_pids()
    if not pids:
        try:
            os.remove(_pid_path())
        except OSError:
            pass
        log("бот не запущен")
        return 0
    for pid in pids:
        log(f"останавливаю бота pid={pid}")
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not any(_pid_alive(pid) for pid in pids):
            break
        time.sleep(0.1)
    for pid in pids:
        if not _pid_alive(pid):
            continue
        log(f"бот pid={pid} не вышел — принудительно")
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        os.remove(_pid_path())
    except OSError:
        pass
    log("бот остановлен")
    return 0


def nav_path_for(map_name: str) -> str | None:
    """Абсолютный путь data/maps/<карта>.nav.json или None."""
    names: list[str] = []
    raw = (map_name or "").strip()
    if raw:
        names.append(raw.split(".")[0])
    norm = normalize_map(raw)
    if norm and norm not in names:
        names.append(norm)
    if not names or not os.path.isdir(MAPS_DIR):
        return None
    for name in names:
        path = os.path.join(MAPS_DIR, f"{name}.nav.json")
        if os.path.isfile(path):
            return path
    want = {f"{name}.nav.json".lower() for name in names}
    try:
        listed = os.listdir(MAPS_DIR)
    except OSError:
        return None
    for fn in listed:
        if fn.lower() in want:
            return os.path.join(MAPS_DIR, fn)
    return None


def dismiss_errors(menu: MenuController, reader: Reader) -> None:
    """Закрыть ошибку/уведомление, из-за которого выкинуло в меню."""
    driver = UIStateReader(reader)
    env = menu.executor._xdo_env()
    clicker = ScreenClicker(menu.executor.menu_input, env=env)
    close_window(
        clicker, env,
        reader_state_fn=lambda: driver.read().ui_fingerprint,
        log=log,
    )


def ensure_spawned(reader: Reader, menu: MenuController, *, shots: bool) -> str | None:
    """Вернуть имя карты, когда игрок заспавнился. None — не вышло."""
    sig = _signals(reader)
    if sig["map_loaded"] and sig["alive_fast"]:
        log(f"уже в бою: {normalize_map(sig['map'])}")
        return normalize_map(sig["map"])
    if sig["map_loaded"]:
        log(f"карта {normalize_map(sig['map'])} загружена — ждём команду/спавн")
        code = wait_spawn(reader, menu, shots=shots, log_fn=log)
        sig = _signals(reader)
        if code == 0 and sig["map_loaded"]:
            return normalize_map(sig["map"])
        if sig["map_loaded"]:
            log("на карте, но ещё не заспавнился — не трогаю меню, повтор ожидания")
            return None
        log("карта пропала — закрываю ошибку и запускаю DM из меню")
    dismiss_errors(menu, reader)
    code = launch_until_spawn(reader, menu, shots=shots, log_fn=log)
    if code != 0:
        return None
    name = normalize_map(_signals(reader)["map"])
    return name or None


def load_coord_map(map_name: str, cache: dict[str, CoordinateMap]) -> CoordinateMap | None:
    if map_name in cache:
        log(f"координатная карта {map_name} уже собрана")
        return cache[map_name]
    path = nav_path_for(map_name)
    if not path:
        try:
            listed = ", ".join(sorted(os.listdir(MAPS_DIR))) or "пусто"
        except OSError as exc:
            listed = f"каталог не читается ({exc})"
        log(
            f"нет навмеша {os.path.join(MAPS_DIR, map_name + '.nav.json')} "
            f"— в каталоге: {listed}"
        )
        return None
    log(f"собираю координатную карту {map_name} ← {path}")
    cmap = CoordinateMap.from_navmesh(path)
    log(cmap.stats())
    cache[map_name] = cmap
    return cmap


def run_bot_until_change(
    reader: Reader,
    inp,
    cmap: CoordinateMap,
    map_name: str,
    *,
    match_time: float | None,
    hz: float,
    quiet: bool,
) -> str:
    """Крутить бота, пока карта та же и мы не в меню.

    match_time — секунды работы бота именно в этом матче (меню не считается).
    Возвращает 'menu' | 'map:<name>' | 'limit' | 'done' | 'retry'.
    """
    cfg = NavConfig(
        tick_hz=hz,
        enemies_poll_hz=600.0,
        duration=None,
        loop=True,
        verbose_log=not quiet,
        engage_targets=True,
        fire_enabled=True,
    )
    nav = Navigator(reader, inp, cmap, cfg)
    route = None
    plan_deadline = time.monotonic() + 20.0
    while time.monotonic() < plan_deadline:
        sig = _signals(reader)
        live = normalize_map(sig["map"]) if sig["map_loaded"] else ""
        if live and live != map_name:
            break
        pos = reader.local_position()
        try:
            route = nav.plan((pos.x, pos.y, pos.z))
            break
        except Exception as exc:  # noqa: BLE001
            log(f"маршрут на {map_name} ещё не строится ({exc})")
            time.sleep(0.5)
    if route is None:
        log(f"маршрут на {map_name} не построился — повтор")
        nav.release()
        inp.release_all()
        return "retry"
    if match_time:
        log(f"лимит матча: {match_time:.0f}с — только пока бот в матче, меню не считается")
    log(
        f"бот на {map_name}: ({pos.x:.0f},{pos.y:.0f},{pos.z:.0f}) → "
        f"цель id={nav.goal_id}, {route.length:.0f}u"
    )

    empty_since: list[float | None] = [None]
    played = [0.0]
    last_tick = [time.monotonic()]
    hit_limit = [False]
    # (имя, с какого момента оно стабильно отличается от текущей карты)
    other_since: list[tuple[str, float] | None] = [None]

    def should_stop() -> bool:
        now = time.monotonic()
        dt = now - last_tick[0]
        last_tick[0] = now
        sig = _signals(reader)
        live = normalize_map(sig["map"]) if sig["map_loaded"] else ""
        if live == map_name:
            other_since[0] = None
            empty_since[0] = None
            played[0] += max(0.0, dt)
            if match_time and played[0] >= match_time:
                hit_limit[0] = True
                log(f"лимит матча {match_time:.0f}с выработан ({played[0]:.0f}с в бою)")
                return True
            return False
        if live and live != map_name:
            # Один кривой кадр имени карты не останавливает бота.
            prev = other_since[0]
            if prev is None or prev[0] != live:
                other_since[0] = (live, now)
                return False
            return now - prev[1] >= 2.0
        other_since[0] = None
        if empty_since[0] is None:
            empty_since[0] = now
        return now - empty_since[0] >= 3.0

    reason = nav.run(duration=None, should_stop=should_stop)
    if hit_limit[0]:
        return "limit"
    sig = _signals(reader)
    live = normalize_map(sig["map"]) if sig["map_loaded"] else ""
    if reason == "stop" and live and live != map_name:
        log(f"смена карты: {map_name} → {live}")
        return f"map:{live}"
    if reason == "stop" or not live:
        log("выкинуло в меню — закрою ошибку и запущу DM заново")
        return "menu"
    return reason


def main() -> int:
    parser = argparse.ArgumentParser(description="DM: меню → спавн → бот, смена карты и кик")
    parser.add_argument(
        "--match-time",
        type=float,
        default=0.0,
        help="Секунды работы бота в одном матче (меню и очередь не считаются, 0 = без лимита)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="То же, что --match-time (старое имя)",
    )
    parser.add_argument("--hz", type=float, default=40.0, help="Частота тика управления (Гц)")
    parser.add_argument("--shots", action="store_true", help="Скриншоты шагов меню")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Остановить уже запущенный этот же скрипт",
    )
    parser.add_argument(
        "--gameoff",
        action="store_true",
        help="Остановить уже запущенный скрипт и закрыть игру",
    )
    args = parser.parse_args()

    if args.gameoff:
        stop_running()
        close_game(log)
        return 0
    if args.stop:
        return stop_running()

    if os.geteuid() == 0:
        print("Не запускай от root", file=sys.stderr)
        return 1

    already = _running_pids()
    if already:
        log(
            f"бот уже запущен (pid={already[0]}). "
            "Остановить: python3 python/tools/run_dm.py --stop. "
            "Остановить и закрыть игру: python3 python/tools/run_dm.py --gameoff"
        )
        return 1

    _install_stop_signal()
    _write_pidfile()
    inp = None
    menu = None
    try:
        try:
            mesh_names = ", ".join(sorted(os.listdir(MAPS_DIR))) or "пусто"
        except OSError as exc:
            mesh_names = f"не читается ({exc})"
        log(f"навмеши: {MAPS_DIR} → {mesh_names}")

        try:
            ei_socket = ensure_gamescope(log)
        except RuntimeError as exc:
            log(str(exc))
            return 1

        ensure_flat_accel()
        reader = None
        reader_deadline = time.monotonic() + 90
        while time.monotonic() < reader_deadline:
            reader = Reader.from_live()
            if reader:
                break
            time.sleep(1.0)
        if not reader:
            log("CS2 не запущен")
            return 1

        try:
            inp = HybridInput(ei_socket)
            inp.prepare()
        except (OSError, TimeoutError) as exc:
            log(f"виртуальная мышь gamescope не поднялась: {exc}")
            return 1
        menu = MenuController(reader, game_input=inp, log=log)
        cache: dict[str, CoordinateMap] = {}
        match_time = args.duration if args.duration is not None else args.match_time
        match_time = match_time or None

        while True:
            map_name = ensure_spawned(reader, menu, shots=args.shots)
            if not map_name:
                log("не удалось зайти в DM — повтор через 5с")
                time.sleep(5.0)
                continue
            cmap = load_coord_map(map_name, cache)
            if cmap is None:
                log(f"навмеша для {map_name!r} не найден в {MAPS_DIR} — бот на этой карте не стартует, повтор")
                time.sleep(2.0)
                continue
            log(f"карта {map_name} подгружена — запускаю бота")
            outcome = run_bot_until_change(
                reader, inp, cmap, map_name,
                match_time=match_time, hz=args.hz, quiet=args.quiet,
            )
            if outcome == "limit":
                log("лимит времени в матче — стоп")
                return 0
            if outcome == "done":
                log("бот завершил маршрут — стоп")
                return 0
            if outcome == "retry":
                time.sleep(2.0)
            # 'menu' или 'map:…' — цикл сам решит: меню или новый навмеш
    finally:
        _clear_pidfile()
        try:
            if menu is not None:
                menu.close()
        finally:
            if inp is not None:
                inp.close()


if __name__ == "__main__":
    raise SystemExit(main())
