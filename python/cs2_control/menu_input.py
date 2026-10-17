"""Меню CS2: uinput для движения мыши и кликов; xdotool только для фокуса окна."""

from __future__ import annotations

import glob
import os
import subprocess
import time
from typing import Callable

from cs2_control.input import GameInput, create_input

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
XDOTOOL = os.path.join(ROOT, "tools", "xdotool", "usr", "bin", "xdotool")
XDO_LIB = os.path.join(ROOT, "tools", "xdotool", "usr", "lib", "x86_64-linux-gnu")


class MenuInput:
    """
    CS2 принимает evdev uinput. Курсор и клики — через один uinput-канал,
    иначе игра игнорирует клик (xdotool move + uinput click).
    xdotool — только windowactivate и чтение позиции мыши.
    """

    def __init__(
        self,
        game_input: GameInput | None = None,
        *,
        log: Callable[[str], None] | None = None,
    ):
        self.log = log or (lambda _m: None)
        self._owns_input = game_input is None
        self._inp = game_input or create_input()
        self._window_id: int | None = None
        self._geo: dict[str, int] = {}

    @property
    def game_input(self) -> GameInput:
        return self._inp

    def prepare(self) -> None:
        self._inp.prepare()
        self.refresh_window()

    def close(self) -> None:
        if self._owns_input:
            self._inp.close()

    def refresh_window(self) -> bool:
        wid = self._find_window()
        if not wid:
            self.log("окно CS2 не найдено")
            return False
        self._window_id = wid
        self._geo = self._window_geometry(wid)
        return True

    def focus(self) -> None:
        if not self._window_id and not self.refresh_window():
            return
        self._xdo("windowactivate", "--sync", str(self._window_id))
        time.sleep(0.35)

    def move_window_rel(self, wx: int, wy: int) -> None:
        """Позиционируем курсор относительно окна (xdotool --window корректен)."""
        if not self._window_id and not self.refresh_window():
            return
        self._xdo("mousemove", "--window", str(self._window_id), str(wx), str(wy))

    def click_window_rel(
        self,
        wx: int,
        wy: int,
        *,
        label: str = "",
        hold_ms: int = 70,
        settle_ms: float = 0.12,
        repeat: int = 1,
    ) -> None:
        if label:
            self.log(f"click {label} window=({wx},{wy})")
        self.focus()
        self.move_window_rel(wx, wy)
        time.sleep(settle_ms)
        for i in range(max(1, repeat)):
            self._inp.mouse_click("left", hold_ms=hold_ms)
            if i + 1 < repeat:
                time.sleep(0.08)

    def key_tap(self, key: str, *, repeat: int = 1) -> None:
        self.focus()
        for _ in range(max(1, repeat)):
            self._inp.key_tap(key, hold_ms=45)
            time.sleep(0.05)

    def type_text(self, text: str) -> None:
        self.focus()
        for ch in text:
            if ch == " ":
                self._inp.key_tap("space", hold_ms=35)
            elif ch == "_":
                self._inp.key_down("shift")
                time.sleep(0.02)
                self._inp.key_tap("-", hold_ms=35)
                self._inp.key_up("shift")
            elif ch.isalnum() or ch in ".-":
                self._inp.key_tap(ch.lower(), hold_ms=35)
            time.sleep(0.05)

    def geometry(self) -> dict[str, int]:
        return dict(self._geo)

    def _xdo_env(self) -> dict[str, str]:
        env = {**os.environ, "DISPLAY": os.environ.get("DISPLAY", ":0")}
        if os.path.isdir(XDO_LIB):
            prev = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = f"{XDO_LIB}:{prev}" if prev else XDO_LIB
        uid = os.getuid()
        for pat in (f"/run/user/{uid}/.mutter-Xwaylandauth.*", f"/run/user/{uid}/.Xauthority"):
            hits = glob.glob(pat)
            if hits:
                env["XAUTHORITY"] = hits[0]
                break
        return env

    def _xdo(self, *args: str) -> str:
        if not os.path.isfile(XDOTOOL):
            return ""
        try:
            out = subprocess.run(
                [XDOTOOL, *args],
                env=self._xdo_env(),
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            return (out.stdout or "").strip()
        except (subprocess.TimeoutExpired, OSError):
            return ""

    def _find_window(self) -> int | None:
        if not os.path.isfile(XDOTOOL):
            return None
        env = self._xdo_env()
        for query in (("search", "--name", "Counter-Strike 2"), ("search", "--class", "cs2")):
            try:
                out = subprocess.run(
                    [XDOTOOL, *query],
                    env=env,
                    capture_output=True,
                    text=True,
                    timeout=2.0,
                )
                raw = (out.stdout or "").strip()
                if raw:
                    return int(raw.splitlines()[0].strip())
            except (subprocess.TimeoutExpired, OSError, ValueError):
                continue
        return None

    def _window_geometry(self, wid: int) -> dict[str, int]:
        geo: dict[str, int] = {}
        if not os.path.isfile(XDOTOOL):
            return geo
        try:
            out = subprocess.run(
                [XDOTOOL, "getwindowgeometry", "--shell", str(wid)],
                env=self._xdo_env(),
                capture_output=True,
                text=True,
                timeout=2.0,
            )
            for line in (out.stdout or "").splitlines():
                if "=" not in line:
                    continue
                k, v = line.split("=", 1)
                try:
                    geo[k.lower()] = int(v)
                except ValueError:
                    pass
        except (subprocess.TimeoutExpired, OSError):
            pass
        return geo
