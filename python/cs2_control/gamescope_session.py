"""CS2 внутри gamescope.

Если игра уже там и у gamescope включена относительная мышь — ничего не трогаем.
Без этого флага вложенный gamescope двигает курсор, а не камеру: перезапускаем.

Иначе gamescope стартует снаружи snap и сам запускает Steam с -noshaders,
чтобы окно «Processing Vulkan shaders» не вставало на пути. Поле Launch Options
для этого не нужно: snap не может выполнить gamescope хоста.
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from collections.abc import Callable

GAMESCOPE = "/usr/games/gamescope"
LOG_PATH = "/tmp/cs2-gamescope.log"
WIDTH = 1920
HEIGHT = 1080
# В cs2_video.txt: 0 = 4:3, 1 = 16:9, 2 = 16:10.
ASPECT_16_9 = "1"


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


def _environ(pid: int) -> dict[str, str]:
    try:
        raw = open(f"/proc/{pid}/environ", "rb").read()
    except OSError:
        return {}
    out: dict[str, str] = {}
    for item in raw.split(b"\x00"):
        if b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        out[key.decode(errors="replace")] = value.decode(errors="replace")
    return out


def gamescope_x11() -> tuple[str, str | None]:
    """Экран и ключ Xwayland внутри gamescope.

    Рабочий стол и AnyDesk часто уже держат :0. Gamescope тогда берёт :1 или :2,
    и курсор меню живёт только там. Мышь удалённого рабочего стола в это поле не входит.
    """
    for pid in _pids():
        cmd = _cmdline(pid)
        if not cmd or os.path.basename(cmd[0]) != "Xwayland":
            continue
        if gamescope_ancestor(pid) is None:
            continue
        display = None
        auth = None
        args = cmd[1:]
        for i, part in enumerate(args):
            if len(part) > 1 and part.startswith(":") and part[1:].isdigit():
                display = part
            if part == "-auth" and i + 1 < len(args):
                auth = args[i + 1]
        if display is None:
            continue
        if not auth:
            auth = _environ(pid).get("XAUTHORITY") or None
        return display, auth
    return ":0", None


def gamescope_xdisplay() -> str:
    return gamescope_x11()[0]


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


def _steam_cmd() -> list[str]:
    """CS2 сразу в 16:9. Параметры Steam Launch Options не трогаем."""
    return [
        "snap", "run", "steam", "-noshaders",
        "-applaunch", "730",
        "-w", str(WIDTH), "-h", str(HEIGHT), "-fullscreen",
    ]


def _video_paths() -> list[str]:
    home = os.path.expanduser("~")
    roots = [
        os.path.join(home, "snap", "steam", "common", ".local", "share", "Steam"),
        os.path.join(home, "snap", "steam", "common", ".steam", "steam"),
        os.path.join(home, ".steam", "steam"),
        os.path.join(home, ".steam", "debian-installation"),
        os.path.join(home, ".local", "share", "Steam"),
        os.path.join(home, ".var", "app", "com.valvesoftware.Steam", ".local", "share", "Steam"),
    ]
    found: list[str] = []
    seen: set[str] = set()
    for root in roots:
        userdata = os.path.join(root, "userdata")
        if not os.path.isdir(userdata):
            continue
        try:
            names = os.listdir(userdata)
        except OSError:
            continue
        for name in names:
            account = os.path.join(userdata, name, "730")
            if not os.path.isdir(account):
                continue
            cfg = os.path.join(account, "local", "cfg")
            try:
                os.makedirs(cfg, exist_ok=True)
            except OSError:
                continue
            path = os.path.join(cfg, "cs2_video.txt")
            if path in seen:
                continue
            seen.add(path)
            found.append(path)
    return found


def _upsert_video_key(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf'("{re.escape(key)}")\s+"[^"]*"')
    replacement = rf'\1\t\t"{value}"'
    if pattern.search(text):
        return pattern.sub(replacement, text, count=1)
    line = f'\t"{key}"\t\t"{value}"\n'
    stripped = text.rstrip()
    if stripped.endswith("}"):
        return stripped[: stripped.rfind("}")] + line + "}\n"
    if not stripped:
        return (
            '"video.cfg"\n{\n'
            + line
            + "}\n"
        )
    return text.rstrip() + "\n" + line


def apply_video_mode(log: Callable[[str], None]) -> None:
    """Записать в клиент CS2 16:9 и 1920×1080, не затирая остальные пункты видео."""
    paths = _video_paths()
    if not paths:
        log(f"видео CS2: 16:9 {WIDTH}×{HEIGHT}, файл настроек появится после входа в Steam")
        return
    keys = {
        "setting.defaultres": str(WIDTH),
        "setting.defaultresheight": str(HEIGHT),
        "setting.aspectratiomode": ASPECT_16_9,
        "setting.fullscreen": "1",
        "setting.nowindowborder": "0",
    }
    for path in paths:
        try:
            text = open(path, encoding="utf-8").read() if os.path.isfile(path) else ""
        except OSError:
            text = ""
        for key, value in keys.items():
            text = _upsert_video_key(text, key, value)
        try:
            with open(path, "w", encoding="utf-8") as handle:
                handle.write(text)
        except OSError as exc:
            log(f"не записал {path}: {exc}")
            continue
    log(f"видео CS2: 16:9 {WIDTH}×{HEIGHT}")


def _launch_gamescope(log: Callable[[str], None]) -> None:
    if not os.path.isfile(GAMESCOPE):
        raise RuntimeError(f"нет {GAMESCOPE}")
    log(f"запускаю gamescope {WIDTH}×{HEIGHT} и CS2, окно шейдеров Vulkan пропускается")
    log_file = open(LOG_PATH, "ab", buffering=0)
    subprocess.Popen(
        [
            GAMESCOPE, "-W", str(WIDTH), "-H", str(HEIGHT),
            "--backend", "sdl", "--force-grab-cursor", "--",
            *_steam_cmd(),
        ],
        env=_session_env(),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _launch_app(log: Callable[[str], None]) -> None:
    log(f"gamescope уже работает — открываю CS2 {WIDTH}×{HEIGHT}, шейдеры пропускаются")
    subprocess.Popen(
        _steam_cmd(),
        env=_session_env(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def _relative_mouse(pid: int) -> bool:
    """Во вложенном gamescope относительная камера есть только с этим флагом."""
    return "--force-grab-cursor" in _cmdline(pid)


def _flag_value(cmd: list[str], flag: str) -> str | None:
    for index, item in enumerate(cmd):
        if item == flag and index + 1 < len(cmd):
            return cmd[index + 1]
        if item.startswith(flag + "="):
            return item.split("=", 1)[1]
    return None


def _video_size(pid: int) -> bool:
    cmd = _cmdline(pid)
    return _flag_value(cmd, "-W") == str(WIDTH) and _flag_value(cmd, "-H") == str(HEIGHT)


def _stop_pid(pid: int) -> None:
    _stop_pids([pid])


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _stop_pids(pids: list[int]) -> None:
    pending = []
    for pid in pids:
        if not _alive(pid):
            continue
        pending.append(pid)
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline and any(_alive(pid) for pid in pending):
        time.sleep(0.2)
    for pid in pending:
        if not _alive(pid):
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass


def close_game(log: Callable[[str], None]) -> None:
    """Закрыть CS2 и окно gamescope, в котором она запущена.

    Steam на рабочем столе, вне gamescope, не трогаем.
    """
    scopes: list[int] = []
    game: list[int] = []
    for pid in cs2_pids():
        game.append(pid)
        scope = gamescope_ancestor(pid)
        if scope and scope not in scopes:
            scopes.append(scope)
    for pid in steam_pids():
        scope = gamescope_ancestor(pid)
        if scope is None:
            continue
        if scope not in scopes:
            scopes.append(scope)
        if pid not in game:
            game.append(pid)
    if not game and not scopes:
        log("игра не запущена")
        return
    log("закрываю игру")
    _stop_pids([*game, *scopes])


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
    Окно не 1920×1080 тоже перезапускается: иначе в игре остаётся старое видео.
    """
    apply_video_mode(log)
    running = cs2_pids()
    if running:
        scope = gamescope_ancestor(running[0])
        if scope is None:
            raise RuntimeError(
                "CS2 запущена обычным окном, не через gamescope. "
                "Закрой её и запусти скрипт снова: вторую копию не стартую."
            )
        if _relative_mouse(scope) and _video_size(scope):
            sock = _ei_socket(scope)
            if not sock:
                raise RuntimeError("CS2 в gamescope, но сокет виртуального ввода не найден")
            log(f"CS2 уже в gamescope {WIDTH}×{HEIGHT} (pid {scope})")
            return sock
        if not _relative_mouse(scope):
            log("gamescope без относительной мыши — перезапускаю, иначе прицел упирается в край окна")
        else:
            log(f"gamescope не {WIDTH}×{HEIGHT} — перезапускаю, чтобы видео стало 16:9")
        _stop_pid(scope)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and (cs2_pids() or _alive(scope)):
            time.sleep(0.3)

    for pid in _pids():
        if not _is_gamescope(_cmdline(pid)):
            continue
        if _relative_mouse(pid) and _video_size(pid):
            continue
        log(f"закрываю старый gamescope, дальше окно {WIDTH}×{HEIGHT}")
        _stop_pid(pid)

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
