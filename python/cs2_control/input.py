"""Клавиши (evdev uinput) и мышь (evdev / xdotool). Резкие движения, без сглаживания."""

from __future__ import annotations

import glob
import os
import subprocess
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
XDOTOOL = ROOT / "tools" / "xdotool" / "usr" / "bin" / "xdotool"

KEY_MAP = {
    "w": "KEY_W",
    "a": "KEY_A",
    "s": "KEY_S",
    "d": "KEY_D",
    "space": "KEY_SPACE",
    "ctrl": "KEY_LEFTCTRL",
    "shift": "KEY_LEFTSHIFT",
    "r": "KEY_R",
    "f": "KEY_F",
    "1": "KEY_1",
    "2": "KEY_2",
    "3": "KEY_3",
}

MOUSE_BUTTONS = {
    "left": ("BTN_LEFT", 1),     # (evdev-код, номер кнопки xdotool)
    "right": ("BTN_RIGHT", 3),
}


class GameInput:
    """WASD + мышь. Не запускать от root."""

    def prepare(self) -> None:
        pass

    def key_down(self, key: str) -> None:
        raise NotImplementedError

    def key_up(self, key: str) -> None:
        raise NotImplementedError

    def key_tap(self, key: str, hold_ms: int = 40) -> None:
        self.key_down(key)
        time.sleep(max(0, hold_ms) / 1000.0)
        self.key_up(key)

    def mouse_move(self, dx: int, dy: int) -> None:
        raise NotImplementedError

    def mouse_down(self, button: str = "left") -> None:
        raise NotImplementedError

    def mouse_up(self, button: str = "left") -> None:
        raise NotImplementedError

    def mouse_wheel(self, delta: int) -> None:
        """Колесо: +1 вверх (invprev), −1 вниз (invnext)."""
        return

    def mouse_click(self, button: str = "left", hold_ms: int = 30) -> None:
        self.mouse_down(button)
        time.sleep(max(0, hold_ms) / 1000.0)
        self.mouse_up(button)

    def mouse_look_deg(
        self,
        yaw_deg: float,
        pitch_deg: float = 0.0,
        deg_per_px: float = 0.08,
        max_px: int = 35,
    ) -> None:
        """Поворот камеры: градусы → пиксели, с ограничением за тик."""
        if deg_per_px <= 0:
            return
        dx = int(round(yaw_deg / deg_per_px))
        dy = int(round(-pitch_deg / deg_per_px))
        if max_px > 0:
            dx = max(-max_px, min(max_px, dx))
            dy = max(-max_px, min(max_px, dy))
        self.mouse_move(dx, dy)

    def release_all(self) -> None:
        for key in list(KEY_MAP):
            try:
                self.key_up(key)
            except Exception:
                pass
        for btn in list(MOUSE_BUTTONS):
            try:
                self.mouse_up(btn)
            except Exception:
                pass

    def close(self) -> None:
        self.release_all()


class EvdevInput(GameInput):
    """
    Мышь через uinput. Дельта, пришедшая за тик контроллера (40 Гц), не выстреливается одним
    событием, а размазывается фоновым потоком на под-шаги с частотой smooth_hz (как отчёты
    реальной мыши 500–1000 Гц) — иначе камера в игре при 144+ fps идёт «ступеньками» и трясётся.
    """

    def __init__(self, *, quiet: bool = False, smooth_hz: float = 500.0) -> None:
        from evdev import UInput, ecodes as e

        self._e = e
        keys = [getattr(e, name) for name in KEY_MAP.values()]
        keys += [getattr(e, code) for code, _n in MOUSE_BUTTONS.values()]
        cap = {e.EV_KEY: keys, e.EV_REL: [e.REL_X, e.REL_Y, e.REL_WHEEL]}
        self._ui = UInput(cap, name="cs2-control-uinput")
        self._down: set[str] = set()
        self._btn_down: set[str] = set()
        self._write_lock = threading.Lock()
        # сглаживание мыши
        self._smooth_hz = smooth_hz
        self._spread = 0.025                 # с: за сколько «доехать» дельту одного тика
        self._seg: list | None = None        # [dx, dy, t0, t1, emitted_x, emitted_y]
        self._seg_lock = threading.Lock()
        self._running = smooth_hz > 0
        if self._running:
            self._thread = threading.Thread(target=self._mouse_loop, name="mouse-smooth", daemon=True)
            self._thread.start()
        if not quiet:
            print(f"Ввод: evdev uinput (клавиши + мышь {smooth_hz:.0f} Гц + кнопки)")

    def set_mouse_spread(self, seconds: float) -> None:
        """Период тика контроллера: дельта каждого тика распределяется на это время."""
        self._spread = max(0.002, float(seconds))

    def _emit_rel(self, dx: int, dy: int) -> None:
        if not dx and not dy:
            return
        with self._write_lock:
            if dx:
                self._ui.write(self._e.EV_REL, self._e.REL_X, int(dx))
            if dy:
                self._ui.write(self._e.EV_REL, self._e.REL_Y, int(dy))
            self._ui.syn()

    def _mouse_loop(self) -> None:
        period = 1.0 / self._smooth_hz
        while self._running:
            time.sleep(period)
            step_x = step_y = 0
            with self._seg_lock:
                seg = self._seg
                if seg is None:
                    continue
                dx, dy, t0, t1, ex, ey = seg
                frac = min(1.0, (time.monotonic() - t0) / max(1e-6, t1 - t0))
                want_x = int(round(dx * frac))
                want_y = int(round(dy * frac))
                step_x, step_y = want_x - ex, want_y - ey
                seg[4], seg[5] = want_x, want_y
                if frac >= 1.0:
                    self._seg = None
            self._emit_rel(step_x, step_y)

    def mouse_down(self, button: str = "left") -> None:
        if button not in MOUSE_BUTTONS or button in self._btn_down:
            return
        code = getattr(self._e, MOUSE_BUTTONS[button][0])
        self._ui.write(self._e.EV_KEY, code, 1)
        self._ui.syn()
        self._btn_down.add(button)

    def mouse_up(self, button: str = "left") -> None:
        if button not in self._btn_down:
            return
        code = getattr(self._e, MOUSE_BUTTONS[button][0])
        self._ui.write(self._e.EV_KEY, code, 0)
        self._ui.syn()
        self._btn_down.discard(button)

    def mouse_wheel(self, delta: int) -> None:
        steps = int(delta)
        if not steps:
            return
        with self._write_lock:
            self._ui.write(self._e.EV_REL, self._e.REL_WHEEL, steps)
            self._ui.syn()

    def key_down(self, key: str) -> None:
        if key not in KEY_MAP or key in self._down:
            return
        code = getattr(self._e, KEY_MAP[key])
        self._ui.write(self._e.EV_KEY, code, 1)
        self._ui.syn()
        self._down.add(key)

    def key_up(self, key: str) -> None:
        if key not in self._down:
            return
        code = getattr(self._e, KEY_MAP[key])
        self._ui.write(self._e.EV_KEY, code, 0)
        self._ui.syn()
        self._down.discard(key)

    def mouse_move(self, dx: int, dy: int) -> None:
        if not dx and not dy:
            return
        if not self._running:
            self._emit_rel(int(dx), int(dy))
            return
        now = time.monotonic()
        with self._seg_lock:
            rem_x = rem_y = 0
            if self._seg is not None:
                # недоехавший остаток прошлого тика переносим в новый отрезок
                rem_x = self._seg[0] - self._seg[4]
                rem_y = self._seg[1] - self._seg[5]
            self._seg = [int(dx) + rem_x, int(dy) + rem_y, now, now + self._spread, 0, 0]

    def close(self) -> None:
        self._running = False
        seg = None
        with self._seg_lock:
            if self._seg is not None:
                seg = self._seg
                self._seg = None
        if seg is not None:
            self._emit_rel(seg[0] - seg[4], seg[1] - seg[5])
        self.release_all()
        self._ui.close()


class XdotoolInput(GameInput):
    """Fallback: только мышь через xdotool; клавиши — evdev."""

    MOUSE_CHUNK = 32

    def __init__(self) -> None:
        self._keys = EvdevInput(quiet=True)
        if not XDOTOOL.exists():
            import shutil

            x = shutil.which("xdotool")
            if not x:
                raise FileNotFoundError("xdotool не найден")
            self._bin = Path(x)
        else:
            self._bin = XDOTOOL
        self._env = self._xdotool_env()
        print("Ввод: evdev (клавиши) + xdotool (мышь)")

    def _xdotool_env(self) -> dict[str, str]:
        lib = ROOT / "tools" / "xdotool" / "usr" / "lib" / "x86_64-linux-gnu"
        env = {**os.environ, "DISPLAY": os.environ.get("DISPLAY", ":0")}
        if lib.exists():
            prev = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = f"{lib}:{prev}" if prev else str(lib)
        uid = os.getuid()
        for pat in (f"/run/user/{uid}/.mutter-Xwaylandauth.*", f"/run/user/{uid}/.Xauthority"):
            hits = glob.glob(pat)
            if hits:
                env["XAUTHORITY"] = hits[0]
                break
        return env

    def prepare(self) -> None:
        self._keys.prepare()

    def key_down(self, key: str) -> None:
        self._keys.key_down(key)

    def key_up(self, key: str) -> None:
        self._keys.key_up(key)

    def mouse_move(self, dx: int, dy: int) -> None:
        if not dx and not dy:
            return
        dx, dy = int(dx), int(dy)
        chunk = self.MOUSE_CHUNK
        while dx or dy:
            step_x = max(-chunk, min(chunk, dx))
            step_y = max(-chunk, min(chunk, dy))
            dx -= step_x
            dy -= step_y
            try:
                subprocess.run(
                    [str(self._bin), "mousemove_relative", "--", str(step_x), str(step_y)],
                    env=self._env,
                    capture_output=True,
                    timeout=0.4,
                )
            except subprocess.TimeoutExpired:
                return

    def _xdo(self, *args: str) -> None:
        try:
            subprocess.run([str(self._bin), *args], env=self._env, capture_output=True, timeout=0.4)
        except subprocess.TimeoutExpired:
            pass

    def mouse_down(self, button: str = "left") -> None:
        if button in MOUSE_BUTTONS:
            self._xdo("mousedown", str(MOUSE_BUTTONS[button][1]))

    def mouse_up(self, button: str = "left") -> None:
        if button in MOUSE_BUTTONS:
            self._xdo("mouseup", str(MOUSE_BUTTONS[button][1]))

    def mouse_wheel(self, delta: int) -> None:
        steps = int(delta)
        if not steps:
            return
        btn = "4" if steps > 0 else "5"
        for _ in range(abs(steps)):
            self._xdo("click", btn)

    def close(self) -> None:
        self.release_all()
        self._keys.close()


def create_input(*, prefer_xdotool_mouse: bool = False) -> GameInput:
    if os.geteuid() == 0:
        raise RuntimeError("не запускай от root — uinput не дойдёт до CS2")
    if prefer_xdotool_mouse:
        try:
            return XdotoolInput()
        except (OSError, PermissionError, FileNotFoundError):
            pass
    try:
        return EvdevInput()
    except (OSError, PermissionError):
        return XdotoolInput()
