"""CS2 внутри gamescope.

Если игра уже там и у gamescope включена относительная мышь — ничего не трогаем.
Без этого флага вложенный gamescope двигает курсор, а не камеру: перезапускаем.

Иначе gamescope стартует снаружи snap и сам запускает Steam с -noshaders,
чтобы окно «Processing Vulkan shaders» не вставало на пути. Поле Launch Options
для этого не нужно: snap не может выполнить gamescope хоста.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from collections.abc import Callable

GAMESCOPE = "/usr/games/gamescope"
APP_URL = "steam://rungameid/730"
LOG_PATH = "/tmp/cs2-gamescope.log"
WIDTH = 1280
HEIGHT = 800


def _cmdline(pid: int) -> list[str]:
    try:
        raw = open(f"/proc/{pid}/cmdline", "rb").read()
    except OSError:
        return []
    parts = raw.split(b"\x00")
    if parts and parts[-1] == b"":
        parts = parts[:-1]
    return [part.decode("utf-8", "replace") for part in parts]


def _ppid(pid: int) -> int:
    try:
        for line in open(f"/proc/{pid}/status", encoding="utf-8", errors="replace"):
            if line.startswith("PPid:"):
                return int(line.split()[1])
    except (OSError, IndexError, ValueError):
        return 0
    return 0


def _pids() -> list[int]:
    found: list[int] = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return found
    for entry in entries:
        if entry.isdigit():
            found.append(int(entry))
    return found


def _is_cs2(cmd: list[str]) -> bool:
    if not cmd:
        return False
    base = os.path.basename(cmd[0])
    return base == "cs2"


def _is_steam_client(cmd: list[str]) -> bool:
    if not cmd:
        return False
    base = os.path.basename(cmd[0])
    return base == "steam" and "webhelper" not in cmd[0]


def _is_gamescope(cmd: list[str]) -> bool:
    return bool(cmd) and os.path.basename(cmd[0]) == "gamescope"


def cs2_pids() -> list[int]:
    return [pid for pid in _pids() if _is_cs2(_cmdline(pid))]


def steam_pids() -> list[int]:
    return [pid for pid in _pids() if _is_steam_client(_cmdline(pid))]


def gamescope_xdisplay() -> str:
    """Экран Xwayland внутри gamescope.

    Рабочий стол часто уже держит :0, и тогда gamescope берёт :1 или :2.
    """
    for pid in _pids():
        cmd = _cmdline(pid)
        if not cmd or os.path.basename(cmd[0]) != "Xwayland":
            continue
        if gamescope_ancestor(pid) is None:
            continue
        for part in cmd[1:]:
            if len(part) > 1 and part.startswith(":") and part[1:].isdigit():
                return part
    return ":0"


def gamescope_ancestor(pid: int) -> int | None:
    seen: set[int] = set()
    while pid > 1 and pid not in seen:
        seen.add(pid)
        if _is_gamescope(_cmdline(pid)):
            return pid
        pid = _ppid(pid)
    return None


def _ei_socket(gamescope_pid: int) -> str | None:
    try:
        fds = os.listdir(f"/proc/{gamescope_pid}/fd")
    except OSError:
        fds = []
    for fd in fds:
        try:
            target = os.readlink(f"/proc/{gamescope_pid}/fd/{fd}")
        except OSError:
            continue
        if target.endswith("-ei") and "gamescope" in target:
            return target
    runtime = f"/run/user/{os.getuid()}"
    try:
        names = sorted(
            name for name in os.listdir(runtime)
            if name.startswith("gamescope-") and name.endswith("-ei")
        )
    except OSError:
        return None
    if not names:
        return None
    return os.path.join(runtime, names[-1])


def _session_env() -> dict[str, str]:
    env = os.environ.copy()
    if env.get("DISPLAY") and env.get("XDG_RUNTIME_DIR"):
        return env
    for pid in _pids():
        try:
            comm = open(f"/proc/{pid}/comm", encoding="utf-8").read().strip()
        except OSError:
            continue
        if comm != "gnome-shell":
            continue
        try:
            raw = open(f"/proc/{pid}/environ", "rb").read()
        except OSError:
            continue
        for item in raw.split(b"\x00"):
            if b"=" not in item:
                continue
            key, value = item.split(b"=", 1)
            name = key.decode()
            if name in {"DISPLAY", "XAUTHORITY", "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR"}:
                env.setdefault(name, value.decode(errors="replace"))
        break
    env.setdefault("DISPLAY", ":1")
    env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    return env


def _steam_outside() -> list[int]:
    return [pid for pid in steam_pids() if gamescope_ancestor(pid) is None]


def _shutdown_desktop_steam(log: Callable[[str], None]) -> None:
    log("Steam открыт на рабочем столе — закрываю клиент, чтобы игра стартовала внутри gamescope")
    subprocess.run(["snap", "run", "steam", "-shutdown"], check=False, timeout=20)
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline and _steam_outside():
        time.sleep(0.4)


def _launch_gamescope(log: Callable[[str], None]) -> None:
    if not os.path.isfile(GAMESCOPE):
        raise RuntimeError(f"нет {GAMESCOPE}")
    log("запускаю gamescope и CS2, окно шейдеров Vulkan пропускается")
    log_file = open(LOG_PATH, "ab", buffering=0)
    subprocess.Popen(
        [
            GAMESCOPE, "-W", str(WIDTH), "-H", str(HEIGHT),
            "--backend", "sdl", "--force-grab-cursor", "--",
            "snap", "run", "steam", "-noshaders", APP_URL,
        ],
        env=_session_env(),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _launch_app(log: Callable[[str], None]) -> None:
    log("gamescope уже работает — открываю CS2 в нём, шейдеры пропускаются")
    subprocess.Popen(
        ["snap", "run", "steam", "-noshaders", APP_URL],
        env=_session_env(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def _relative_mouse(pid: int) -> bool:
    """Во вложенном gamescope относительная камера есть только с этим флагом."""
    return "--force-grab-cursor" in _cmdline(pid)


def _stop_pid(pid: int) -> None:
    try:
        os.kill(pid, signal.SIGTERM)
    except OSError:
        return
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except OSError:
            return
        time.sleep(0.2)
    try:
        os.kill(pid, signal.SIGKILL)
    except OSError:
        pass


def game_focused() -> bool:
    """Окно gamescope сейчас наверху. Фокус не переключаем."""
    xdo = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        "tools", "xdotool", "usr", "bin", "xdotool",
    )
    if not os.path.isfile(xdo):
        return False
    env = _session_env()
    lib = os.path.join(os.path.dirname(xdo), "..", "lib", "x86_64-linux-gnu")
    env["LD_LIBRARY_PATH"] = lib + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    proc = subprocess.run(
        [xdo, "getactivewindow"],
        env=env, capture_output=True, text=True, check=False,
    )
    active = (proc.stdout or "").strip()
    if not active.isdigit():
        return False
    found = subprocess.run(
        [xdo, "search", "--class", "gamescope"],
        env=env, capture_output=True, text=True, check=False,
    )
    return active in (found.stdout or "").split()


def focus_game() -> None:
    """Отдать кресло окну gamescope: без фокуса uinput уходит на рабочий стол."""
    xdo = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        "tools", "xdotool", "usr", "bin", "xdotool",
    )
    if not os.path.isfile(xdo):
        return
    env = _session_env()
    lib = os.path.join(os.path.dirname(xdo), "..", "lib", "x86_64-linux-gnu")
    env["LD_LIBRARY_PATH"] = lib + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    subprocess.run(
        [xdo, "search", "--class", "gamescope", "windowactivate"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
    )


def ensure_gamescope(log: Callable[[str], None]) -> str:
    """Вернуть сокет libei. Без относительной мыши gamescope перезапускается:
    иначе прицел — это курсор, он упирается в край окна и возвращается назад.
    """
    running = cs2_pids()
    if running:
        scope = gamescope_ancestor(running[0])
        if scope is None:
            raise RuntimeError(
                "CS2 запущена обычным окном, не через gamescope. "
                "Закрой её и запусти скрипт снова: вторую копию не стартую."
            )
        if _relative_mouse(scope):
            sock = _ei_socket(scope)
            if not sock:
                raise RuntimeError("CS2 в gamescope, но сокет виртуального ввода не найден")
            log(f"CS2 уже в gamescope (pid {scope})")
            return sock
        log("gamescope без относительной мыши — перезапускаю, иначе прицел упирается в край окна")
        _stop_pid(scope)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and cs2_pids():
            time.sleep(0.3)

    if _steam_outside():
        _shutdown_desktop_steam(log)
    if any(gamescope_ancestor(pid) for pid in steam_pids()):
        _launch_app(log)
    else:
        _launch_gamescope(log)

    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        found = cs2_pids()
        if found and gamescope_ancestor(found[0]):
            time.sleep(1.0)
            if found[0] in cs2_pids():
                scope = gamescope_ancestor(found[0])
                sock = _ei_socket(scope or found[0])
                if sock:
                    log(f"CS2 поднялась в gamescope (pid {scope})")
                    return sock
        time.sleep(0.5)
    tail = ""
    try:
        data = open(LOG_PATH, "rb").read()[-800:]
        tail = data.decode("utf-8", "replace")
    except OSError:
        pass
    raise RuntimeError("CS2 внутри gamescope не появилась" + (f"\n{tail}" if tail else ""))
