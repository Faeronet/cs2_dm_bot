"""Ввод в CS2 внутри gamescope.

Клавиши и взгляд идут в сокет libei и не трогают курсор рабочего стола.
Это виртуальная мышь самого композита: она крутит камеру и когда окно
игры не наверху.
"""

from __future__ import annotations

import ctypes
import os
import threading
import time

from cs2_control.input import KEY_MAP, MOUSE_BUTTONS, GameInput

# libei 1.2: биты возможностей устройства.
_CAP_POINTER = 1
_CAP_KEYBOARD = 4
_CAP_SCROLL = 16
_CAP_BUTTON = 32

_BTN = {"left": 0x110, "right": 0x111}  # BTN_LEFT, BTN_RIGHT


def _libei():
    lib = ctypes.CDLL("libei.so.1")
    vp = ctypes.c_void_p
    lib.ei_new_sender.argtypes = [vp]
    lib.ei_new_sender.restype = vp
    lib.ei_configure_name.argtypes = [vp, ctypes.c_char_p]
    lib.ei_setup_backend_socket.argtypes = [vp, ctypes.c_char_p]
    lib.ei_setup_backend_socket.restype = ctypes.c_int
    lib.ei_dispatch.argtypes = [vp]
    lib.ei_get_event.argtypes = [vp]
    lib.ei_get_event.restype = vp
    lib.ei_event_get_type.argtypes = [vp]
    lib.ei_event_get_type.restype = ctypes.c_int
    lib.ei_event_type_to_string.argtypes = [ctypes.c_int]
    lib.ei_event_type_to_string.restype = ctypes.c_char_p
    lib.ei_event_get_seat.argtypes = [vp]
    lib.ei_event_get_seat.restype = vp
    lib.ei_event_get_device.argtypes = [vp]
    lib.ei_event_get_device.restype = vp
    lib.ei_event_unref.argtypes = [vp]
    # Сигнатура varargs: перечисление возможностей, конец — 0. Не битовая маска.
    lib.ei_seat_bind_capabilities.argtypes = [vp]
    lib.ei_seat_bind_capabilities.restype = None
    lib.ei_device_has_capability.argtypes = [vp, ctypes.c_int]
    lib.ei_device_has_capability.restype = ctypes.c_bool
    lib.ei_device_ref.argtypes = [vp]
    lib.ei_device_ref.restype = vp
    lib.ei_device_unref.argtypes = [vp]
    lib.ei_device_start_emulating.argtypes = [vp, ctypes.c_uint32]
    lib.ei_device_stop_emulating.argtypes = [vp]
    lib.ei_device_pointer_motion.argtypes = [vp, ctypes.c_double, ctypes.c_double]
    lib.ei_device_pointer_motion_absolute.argtypes = [vp, ctypes.c_double, ctypes.c_double]
    lib.ei_device_get_width.argtypes = [vp]
    lib.ei_device_get_width.restype = ctypes.c_int
    lib.ei_device_get_height.argtypes = [vp]
    lib.ei_device_get_height.restype = ctypes.c_int
    lib.ei_device_keyboard_key.argtypes = [vp, ctypes.c_uint32, ctypes.c_bool]
    lib.ei_device_button_button.argtypes = [vp, ctypes.c_uint32, ctypes.c_bool]
    lib.ei_device_scroll_discrete.argtypes = [vp, ctypes.c_int32, ctypes.c_int32]
    lib.ei_device_frame.argtypes = [vp, ctypes.c_uint64]
    lib.ei_now.argtypes = [vp]
    lib.ei_now.restype = ctypes.c_uint64
    lib.ei_unref.argtypes = [vp]
    return lib


def _event_names(lib) -> dict[str, int]:
    names: dict[str, int] = {}
    for code in range(48):
        raw = lib.ei_event_type_to_string(code)
        if not raw:
            continue
        text = raw.decode()
        if text and text != "unknown":
            names[text] = code
    return names


class EiInput(GameInput):
    """Клавиатура и мышь в сокет gamescope-*-ei."""

    # move_abs вычитает этот угол: указатель ei живёт внутри окна, не на всём экране.
    pointer_local = True

    def __init__(self, socket_path: str, *, smooth_hz: float = 500.0) -> None:
        from evdev import ecodes as e

        self._codes = e
        self._lib = _libei()
        self._names = _event_names(self._lib)
        self._ei = self._lib.ei_new_sender(None)
        if not self._ei:
            raise OSError("libei: не создался отправитель")
        self._lib.ei_configure_name(self._ei, b"cs2-control")
        rc = self._lib.ei_setup_backend_socket(self._ei, socket_path.encode())
        if rc != 0:
            self._lib.ei_unref(self._ei)
            raise OSError(f"libei: сокет {socket_path} не открылся ({rc})")

        self._lock = threading.Lock()
        self._pointer = None
        self._absolute = False
        self._keyboard = None
        self._buttons = None
        self._scroll = None
        self._resumed: set[int] = set()
        self._seq = 1
        self._alive = True
        self._connected = False
        self._down: set[str] = set()
        self._btn_down: set[str] = set()
        self.pointer_origin = (0, 0)

        self._smooth_hz = smooth_hz
        self._spread = 0.025
        self._seg: list | None = None
        self._seg_lock = threading.Lock()
        self._running = False
        self._mouse_started = False
        self._pump = threading.Thread(target=self._pump_loop, name="gamescope-ei", daemon=True)
        self._pump.start()

    def prepare(self) -> None:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            if self._pointer and self._keyboard and self._ready(self._pointer) and self._ready(self._keyboard):
                break
            time.sleep(0.02)
        else:
            raise TimeoutError("gamescope не отдал клавиатуру и мышь по libei")
        if self._mouse_started:
            return
        # Тот же размаз 500 Гц, что у прежней мыши. Край внутреннего курсора
        # взгляд не останавливает: gamescope захватывает относительное движение.
        self._running = True
        threading.Thread(target=self._mouse_loop, name="gamescope-mouse", daemon=True).start()
        self._mouse_started = True
        print("Ввод: gamescope libei (клавиши + мышь, мимо рабочего стола)")

    def _ready(self, dev) -> bool:
        return dev is not None and int(dev) in self._resumed

    def _pump_loop(self) -> None:
        while self._alive:
            with self._lock:
                if not self._ei:
                    return
                self._lib.ei_dispatch(self._ei)
                while True:
                    ev = self._lib.ei_get_event(self._ei)
                    if not ev:
                        break
                    try:
                        self._on_event(ev)
                    finally:
                        self._lib.ei_event_unref(ev)
            time.sleep(0.002)

    def _on_event(self, ev) -> None:
        kind = self._lib.ei_event_get_type(ev)
        connect = self._names.get("EI_EVENT_CONNECT")
        seat = self._names.get("EI_EVENT_SEAT_ADDED")
        added = self._names.get("EI_EVENT_DEVICE_ADDED")
        resumed = self._names.get("EI_EVENT_DEVICE_RESUMED")
        paused = self._names.get("EI_EVENT_DEVICE_PAUSED")
        removed = self._names.get("EI_EVENT_DEVICE_REMOVED")
        disconnect = self._names.get("EI_EVENT_DISCONNECT")
        if kind == connect:
            self._connected = True
            return
        if kind == seat:
            seat_obj = self._lib.ei_event_get_seat(ev)
            self._lib.ei_seat_bind_capabilities(
                seat_obj,
                _CAP_POINTER,
                2,  # абсолютный указатель, если композитор его отдаёт
                _CAP_KEYBOARD,
                _CAP_SCROLL,
                _CAP_BUTTON,
                0,
            )
            return
        if kind == added:
            dev = self._lib.ei_device_ref(self._lib.ei_event_get_device(ev))
            self._claim(dev)
            self._lib.ei_device_start_emulating(dev, self._seq)
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            return
        if kind == resumed:
            dev = self._lib.ei_event_get_device(ev)
            if dev:
                self._resumed.add(int(dev))
            return
        if kind == paused or kind == removed:
            dev = self._lib.ei_event_get_device(ev)
            if dev:
                self._resumed.discard(int(dev))
            return
        if kind == disconnect:
            self._connected = False

    def _claim(self, dev) -> None:
        has = self._lib.ei_device_has_capability
        if has(dev, _CAP_POINTER) and self._pointer is None:
            self._pointer = dev
            self._absolute = bool(has(dev, 2))
        if has(dev, _CAP_KEYBOARD) and self._keyboard is None:
            self._keyboard = dev
        if has(dev, _CAP_BUTTON) and self._buttons is None:
            self._buttons = dev
        if has(dev, _CAP_SCROLL) and self._scroll is None:
            self._scroll = dev

    def _frame(self, dev) -> None:
        self._lib.ei_device_frame(dev, self._lib.ei_now(self._ei))

    def _pixel_region(self) -> bool:
        """Абсолютные координаты годны, только если область — само окно, а не весь int32."""
        if self._pointer is None:
            return False
        lib = self._lib
        vp = ctypes.c_void_p
        if not hasattr(lib, "_region_ready"):
            lib.ei_device_get_region.argtypes = [vp, ctypes.c_size_t]
            lib.ei_device_get_region.restype = vp
            lib.ei_region_get_width.argtypes = [vp]
            lib.ei_region_get_width.restype = ctypes.c_uint32
            lib.ei_region_get_height.argtypes = [vp]
            lib.ei_region_get_height.restype = ctypes.c_uint32
            lib._region_ready = True
        region = lib.ei_device_get_region(self._pointer, 0)
        if not region:
            return False
        width = int(lib.ei_region_get_width(region))
        height = int(lib.ei_region_get_height(region))
        return 320 <= width <= 8192 and 240 <= height <= 8192

    def move_abs_local(self, x: int, y: int) -> bool:
        """Пиксель внутри окна. False — область libei не в пикселях, нужен относительный ход."""
        if not self._absolute or not self._ready(self._pointer) or not self._pixel_region():
            return False
        with self._lock:
            self._lib.ei_device_pointer_motion_absolute(self._pointer, float(x), float(y))
            self._frame(self._pointer)
        return True

    def _emit_rel(self, dx: int, dy: int) -> None:
        if (not dx and not dy) or not self._ready(self._pointer):
            return
        with self._lock:
            self._lib.ei_device_pointer_motion(self._pointer, float(dx), float(dy))
            self._frame(self._pointer)

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

    def set_mouse_spread(self, seconds: float) -> None:
        self._spread = max(0.002, float(seconds))

    def key_down(self, key: str) -> None:
        if key not in KEY_MAP or key in self._down or not self._ready(self._keyboard):
            return
        code = getattr(self._codes, KEY_MAP[key])
        with self._lock:
            self._lib.ei_device_keyboard_key(self._keyboard, int(code), True)
            self._frame(self._keyboard)
        self._down.add(key)

    def key_up(self, key: str) -> None:
        if key not in self._down:
            return
        self._down.discard(key)
        if not self._ready(self._keyboard):
            return
        code = getattr(self._codes, KEY_MAP[key])
        with self._lock:
            self._lib.ei_device_keyboard_key(self._keyboard, int(code), False)
            self._frame(self._keyboard)

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
                rem_x = self._seg[0] - self._seg[4]
                rem_y = self._seg[1] - self._seg[5]
            self._seg = [int(dx) + rem_x, int(dy) + rem_y, now, now + self._spread, 0, 0]

    def mouse_down(self, button: str = "left") -> None:
        if button not in _BTN or button in self._btn_down or not self._ready(self._buttons or self._pointer):
            return
        dev = self._buttons or self._pointer
        with self._lock:
            self._lib.ei_device_button_button(dev, _BTN[button], True)
            self._frame(dev)
        self._btn_down.add(button)

    def mouse_up(self, button: str = "left") -> None:
        if button not in self._btn_down:
            return
        self._btn_down.discard(button)
        dev = self._buttons or self._pointer
        if not self._ready(dev):
            return
        with self._lock:
            self._lib.ei_device_button_button(dev, _BTN[button], False)
            self._frame(dev)

    def mouse_wheel(self, delta: int) -> None:
        steps = int(delta)
        dev = self._scroll or self._pointer
        if not steps or not self._ready(dev):
            return
        with self._lock:
            self._lib.ei_device_scroll_discrete(dev, 0, steps)
            self._frame(dev)

    def close(self) -> None:
        self._running = False
        self._alive = False
        self.release_all()
        time.sleep(0.03)
        with self._lock:
            seen: set[int] = set()
            for dev in (self._pointer, self._keyboard, self._buttons, self._scroll):
                if not dev or int(dev) in seen:
                    continue
                seen.add(int(dev))
                try:
                    self._lib.ei_device_stop_emulating(dev)
                except Exception:
                    pass
            if self._ei:
                self._lib.ei_unref(self._ei)
                self._ei = None


class HybridInput(GameInput):
    """Клавиши и камера — виртуальная мышь gamescope. Курсор рабочего стола не двигается.

    Хост-мышь до композита не доходит, поэтому взгляд шлётся в тот же сокет,
    что и клавиши, и не зависит от того, какое окно сейчас наверху.
    """

    def __init__(self, socket_path: str) -> None:
        self._ei = EiInput(socket_path)
        self._announced = False

    def prepare(self) -> None:
        self._ei.prepare()
        if self._announced:
            return
        self._announced = True
        print("Камера: виртуальная мышь gamescope 500 Гц")

    def key_down(self, key: str) -> None:
        self._ei.key_down(key)

    def key_up(self, key: str) -> None:
        self._ei.key_up(key)

    def mouse_move(self, dx: int, dy: int) -> None:
        self._ei.mouse_move(dx, dy)

    def mouse_down(self, button: str = "left") -> None:
        self._ei.mouse_down(button)

    def mouse_up(self, button: str = "left") -> None:
        self._ei.mouse_up(button)

    def mouse_wheel(self, delta: int) -> None:
        self._ei.mouse_wheel(delta)

    def set_mouse_spread(self, seconds: float) -> None:
        self._ei.set_mouse_spread(seconds)

    @property
    def _running(self) -> bool:
        return bool(self._ei._running)

    @_running.setter
    def _running(self, value: bool) -> None:
        self._ei._running = bool(value)

    def _emit_rel(self, dx: int, dy: int) -> None:
        """Шаг относительной мыши gamescope. Сглаживание к этому моменту уже выключено."""
        self._ei._emit_rel(int(dx), int(dy))

    def _xdo(self, *args: str) -> bool:
        import subprocess

        from cs2_control.gamescope_session import _session_env

        xdo = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            "tools", "xdotool", "usr", "bin", "xdotool",
        )
        if not os.path.isfile(xdo):
            return False
        env = _session_env()
        lib = os.path.join(os.path.dirname(xdo), "..", "lib", "x86_64-linux-gnu")
        env["LD_LIBRARY_PATH"] = lib + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
        subprocess.run(
            [xdo, *args],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        )
        return True

    def move_abs_local(self, x: int, y: int) -> bool:
        """Меню ставится абсолютным курсором. Относительные шаги под --force-grab-cursor кнопку не находят."""
        return self._xdo("mousemove", "--sync", str(int(x)), str(int(y)))

    def click_at_screen(self, x: int, y: int, button: str = "left") -> bool:
        """Клик курсором рабочего стола. Кнопка uinput под захватом gamescope бьёт не в эту точку."""
        code = {"left": "1", "middle": "2", "right": "3"}.get(button, "1")
        return self._xdo("mousemove", "--sync", str(int(x)), str(int(y)), "click", code)

    def _inner_env(self) -> dict[str, str]:
        """Xwayland внутри gamescope. Курсор меню живёт здесь, не на рабочем столе."""
        from cs2_control.gamescope_session import _session_env, gamescope_x11

        env = _session_env()
        display, auth = gamescope_x11()
        env["DISPLAY"] = display
        if auth and os.path.isfile(auth):
            env["XAUTHORITY"] = auth
        else:
            gdm = f"/run/user/{os.getuid()}/gdm/Xauthority"
            if os.path.isfile(gdm):
                env["XAUTHORITY"] = gdm
        if not getattr(self, "_inner_announced", False):
            self._inner_announced = True
            print(f"Курсор меню: экран gamescope {display}", flush=True)
        lib = os.path.join(os.path.dirname(os.path.dirname(self._xdo_bin())), "lib", "x86_64-linux-gnu")
        if os.path.isdir(lib):
            prev = env.get("LD_LIBRARY_PATH", "")
            env["LD_LIBRARY_PATH"] = f"{lib}:{prev}" if prev else lib
        return env

    def _xdo_bin(self) -> str:
        cur = os.path.abspath(os.path.dirname(__file__))
        for _ in range(8):
            cand = os.path.join(cur, "tools", "xdotool", "usr", "bin", "xdotool")
            if os.path.isfile(cand):
                return cand
            nxt = os.path.dirname(cur)
            if nxt == cur:
                break
            cur = nxt
        return ""

    def _inner_run(self, *args: str):
        import subprocess

        xdo = self._xdo_bin()
        if not os.path.isfile(xdo):
            return None
        try:
            proc = subprocess.Popen(
                [xdo, *args],
                env=self._inner_env(),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                start_new_session=True,
            )
        except OSError as exc:
            with open("/tmp/click_trace.txt", "a", encoding="utf-8") as fh:
                fh.write(f"popen {exc}\n")
            return None
        try:
            out, err = proc.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            with open("/tmp/click_trace.txt", "a", encoding="utf-8") as fh:
                fh.write(f"timeout pid={proc.pid} {' '.join(args)}\n")
            proc.kill()
            proc.communicate()
            return None
        proc.stdout = out
        proc.stderr = err
        return proc

    def _inner_xdo(self, *args: str) -> str:
        out = self._inner_run(*args)
        if out is None or out.returncode != 0:
            return ""
        return (out.stdout or "").strip()

    def _inner_window(self) -> tuple[str, int, int, int, int] | None:
        """Окно CS2 внутри gamescope: id и прямоугольник поля, без рамки рабочего стола."""
        raw = self._inner_xdo("search", "--class", "cs2")
        best: tuple[str, int, int, int, int] | None = None
        best_area = 0
        for line in raw.splitlines():
            wid = line.strip()
            if not wid.isdigit():
                continue
            geo: dict[str, int] = {}
            for row in self._inner_xdo("getwindowgeometry", "--shell", wid).splitlines():
                if "=" not in row:
                    continue
                key, value = row.split("=", 1)
                if value.strip().lstrip("-").isdigit():
                    geo[key.strip().lower()] = int(value.strip())
            width, height = geo.get("width", 0), geo.get("height", 0)
            if width < 320 or height < 240:
                continue
            if width * height > best_area:
                best = (wid, geo.get("x", 0), geo.get("y", 0), width, height)
                best_area = width * height
        return best

    def _display_size(self) -> tuple[int, int]:
        """Размер поля gamescope. Окно на рабочем столе для этого не нужно."""
        raw = self._inner_xdo("getdisplaygeometry")
        parts = raw.split()
        if len(parts) >= 2 and parts[0].isdigit() and parts[1].isdigit():
            width, height = int(parts[0]), int(parts[1])
            if width >= 320 and height >= 240:
                return width, height
        found = self._inner_window()
        if found is not None:
            return found[3], found[4]
        return 1280, 800

    def grab_field(self, path: str) -> tuple[int, int] | None:
        """Снимок поля gamescope. Верхняя рамка окна сюда не попадает."""
        import subprocess

        width, height = self._display_size()
        env = self._inner_env()
        try:
            proc = subprocess.run(
                [
                    "ffmpeg", "-y", "-loglevel", "error",
                    "-f", "x11grab", "-video_size", f"{width}x{height}",
                    "-i", f"{env['DISPLAY']}+0,0", "-frames:v", "1", path,
                ],
                env=env,
                capture_output=True,
                timeout=8,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None
        if proc.returncode != 0 or not os.path.isfile(path):
            return None
        return width, height

    def _pointer_pos(self) -> tuple[int, int] | None:
        raw = self._inner_xdo("getmouselocation", "--shell")
        x = y = None
        for row in raw.splitlines():
            if row.startswith("X=") and row[2:].lstrip("-").isdigit():
                x = int(row[2:])
            elif row.startswith("Y=") and row[2:].lstrip("-").isdigit():
                y = int(row[2:])
        if x is None or y is None:
            return None
        return x, y

    def _field_pointer(self, fx: float, fy: float, *, click: bool) -> bool:
        width, height = self._display_size()
        # Курсор композита не выходит за поле и не достаёт до рамки окна.
        fx = min(1.0, max(0.0, float(fx)))
        fy = min(1.0, max(0.0, float(fy)))
        x = min(width - 1, max(0, int(round(fx * (width - 1)))))
        y = min(height - 1, max(0, int(round(fy * (height - 1)))))
        # Один отправленный пиксель по X сдвигается примерно на 1.5.
        # Полная координата уезжала в правую половину, а 2x−ширина — в телевизор.
        x_move = int(round(x * 2 / 3))
        # Ниже верхней полосы клик садится на 18 px ниже центра кнопки.
        # PLAY этой полосы и так попадает — его не поднимаем.
        # Deathmatch после этого ещё на 18 px ниже, GO — ещё на 68.
        lift = 18
        if abs(fx - 0.517) <= 0.06 and abs(fy - 0.118) <= 0.06:
            lift = 36
        elif abs(fx - 0.865) <= 0.05 and abs(fy - 0.956) <= 0.06:
            lift = 86
        y_move = y if y < 40 else max(0, y - lift)
        return self._xtest_from_origin(x_move, y_move, click=click)

    def _xtest_from_origin(self, x: int, y: int, *, click: bool) -> bool:
        """Прижать курсор в левый верх относительными шагами и дойти до (x, y)."""
        import ctypes

        env = self._inner_env()
        x11 = ctypes.CDLL("libX11.so.6")
        xtst = ctypes.CDLL("libXtst.so.6")
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XFlush.argtypes = [ctypes.c_void_p]
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        xtst.XTestFakeRelativeMotionEvent.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_ulong,
        ]
        xtst.XTestFakeRelativeMotionEvent.restype = ctypes.c_int
        xtst.XTestFakeButtonEvent.argtypes = [
            ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong,
        ]
        xtst.XTestFakeButtonEvent.restype = ctypes.c_int

        old_display = os.environ.get("DISPLAY")
        old_auth = os.environ.get("XAUTHORITY")
        os.environ["DISPLAY"] = env["DISPLAY"]
        if env.get("XAUTHORITY"):
            os.environ["XAUTHORITY"] = env["XAUTHORITY"]
        dpy = x11.XOpenDisplay(env["DISPLAY"].encode())
        try:
            if not dpy:
                print(f"[menu-click] нет экрана {env['DISPLAY']}", flush=True)
                return False

            def rel(dx: int, dy: int) -> None:
                step = 40
                left, up = int(dx), int(dy)
                n = 0
                while left or up:
                    sx = max(-step, min(step, left))
                    sy = max(-step, min(step, up))
                    xtst.XTestFakeRelativeMotionEvent(dpy, sx, sy, 0)
                    left -= sx
                    up -= sy
                    n += 1
                    if n % 8 == 0:
                        x11.XFlush(dpy)
                x11.XFlush(dpy)

            rel(-2400, -2400)
            rel(x, y)
            if click:
                xtst.XTestFakeButtonEvent(dpy, 1, 1, 0)
                x11.XFlush(dpy)
                time.sleep(0.03)
                xtst.XTestFakeButtonEvent(dpy, 1, 0, 0)
                x11.XFlush(dpy)
            return True
        finally:
            if dpy:
                x11.XCloseDisplay(dpy)
            if old_display is None:
                os.environ.pop("DISPLAY", None)
            else:
                os.environ["DISPLAY"] = old_display
            if old_auth is None:
                os.environ.pop("XAUTHORITY", None)
            elif old_auth is not None:
                os.environ["XAUTHORITY"] = old_auth

    def hover_field(self, fx: float, fy: float) -> bool:
        return self._field_pointer(fx, fy, click=False)

    def click_field(self, fx: float, fy: float) -> bool:
        """Клик по доле поля 1280×800 внутри gamescope. Курсор рабочего стола не двигается."""
        return self._field_pointer(fx, fy, click=True)

    def close(self) -> None:
        self._ei.close()

