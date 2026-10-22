"""Меню CS2 внутри gamescope.

Курсор композита не выходит за поле игры и не достаёт до верхней рамки окна:
для него игра уже на весь свой экран. Поэтому кадр и клики считаются от поля
gamescope (1280×800), а не от окна на рабочем столе.
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

SCREEN_W, SCREEN_H = 1920, 1200


def ensure_flat_accel() -> None:
    """flat-профиль + speed 0 ⇒ 1 дельта uinput = 1 пиксель курсора композитора."""
    for args in (
        ["org.gnome.desktop.peripherals.mouse", "accel-profile", "flat"],
        ["org.gnome.desktop.peripherals.mouse", "speed", "0.0"],
    ):
        try:
            subprocess.run(["gsettings", "set", *args], capture_output=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            pass


def grab_rect(env: dict, rect: tuple[int, int, int, int], out: str) -> bool:
    """Снимок прямоугольника на том экране, где окно видно глазами."""
    x, y, w, h = rect
    x0, y0 = max(0, x), max(0, y)
    if w < 2 or h < 2:
        return False
    disp = env.get("DISPLAY", ":1")
    try:
        proc = subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "x11grab",
             "-video_size", f"{w}x{h}", "-i", f"{disp}+{x0},{y0}",
             "-frames:v", "1", "-update", "1", out],
            env=env, capture_output=True, timeout=8,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return proc.returncode == 0 and Path(out).exists()


def _xdotool() -> str:
    root = Path(__file__).resolve().parents[2]
    cand = root / "tools" / "xdotool" / "usr" / "bin" / "xdotool"
    return str(cand) if cand.is_file() else "xdotool"


def host_size(env: dict) -> tuple[int, int]:
    """Реальный размер экрана. Жёсткие 1920×1200 на другом мониторе дают пустой кадр."""
    try:
        proc = subprocess.run(
            [_xdotool(), "getdisplaygeometry"],
            env=env, capture_output=True, text=True, timeout=2,
        )
        parts = (proc.stdout or "").split()
        if len(parts) >= 2 and parts[0].isdigit() and parts[1].isdigit():
            width, height = int(parts[0]), int(parts[1])
            if width >= 320 and height >= 240:
                return width, height
    except (OSError, subprocess.TimeoutExpired):
        pass
    return SCREEN_W, SCREEN_H


def root_shot(env: dict, out: str, *, size: tuple[int, int] | None = None) -> bool:
    disp = env.get("DISPLAY", ":1")
    host_w, host_h = host_size(env)
    if size is None:
        w, h = host_w, host_h
    else:
        w, h = min(size[0], host_w), min(size[1], host_h)
    try:
        proc = subprocess.run(
            ["ffmpeg", "-y", "-f", "x11grab", "-video_size", f"{w}x{h}",
             "-i", f"{disp}+0,0", "-frames:v", "1", "-update", "1", out],
            env=env, capture_output=True, timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return proc.returncode == 0 and Path(out).exists()


def detect_content_offset(shot_path: str, win: tuple[int, int, int, int]) -> int | None:
    """Высота декорации заголовка окна (GNOME CSD) в пикселях.

    Окно CS2 — плавающее, с титульной полосой «Counter-Strike 2» сверху. Её высота
    ФИКСИРОВАНА (не масштабируется с окном), поэтому доли, отсчитанные от всего окна,
    ломаются при смене размера/разрешения. Находим нижнюю границу титула — тёмную
    горизонтальную линию-разделитель между титулом и игровым контентом — и возвращаем
    смещение контента. Это и есть «парсинг самого окна» для точного мапинга.
    """
    x, y, w, h = win
    try:
        from PIL import Image
        full = Image.open(shot_path).convert("L")
    except Exception:  # noqa: BLE001
        return None
    crop = full.crop((max(0, x), max(0, y), min(SCREEN_W, x + w), min(SCREEN_H, y + h)))
    cw, chh = crop.size
    if cw < 40 or chh < 60:
        return None
    px = crop.load()

    def rowmean(yy: int) -> float:
        s = n = 0
        for xx in range(0, cw, 4):
            s += px[xx, yy]
            n += 1
        return s / max(1, n)

    def row_dark_frac(yy: int) -> float:
        dark = n = 0
        for xx in range(0, cw, 4):
            n += 1
            if px[xx, yy] < 15:
                dark += 1
        return dark / max(1, n)

    hi = min(chh // 2, 90)
    # 1) тёмная полоса-разделитель во всю ширину (самый надёжный признак)
    for yy in range(24, hi):
        if rowmean(yy) < 6.0 and row_dark_frac(yy) > 0.7:
            return yy + 1
    # 2) запасной: скачок яркости от серого титула к более светлому контенту
    title = sum(rowmean(k) for k in range(2, 9)) / 7
    for yy in range(16, hi):
        below = sum(rowmean(k) for k in range(yy + 2, min(chh, yy + 10))) / 8
        if below > title + 22:
            return yy + 1
    return None


def _blank(im) -> bool:
    """Чёрный кадр Xwayland: картинка игры в нём не лежит, доли с него не читаются."""
    px = im.convert("L").load()
    w, h = im.size
    total = n = 0
    for y in range(0, h, 16):
        for x in range(0, w, 16):
            total += px[x, y]
            n += 1
    return n > 0 and total / n < 8.0


def move_abs(inp, sx: int, sy: int, *, step_delay: float = 0.0003) -> None:
    """Абсолютное позиционирование курсора композитора через относительные дельты."""
    # остановить фоновое сглаживание мыши, если включено
    if getattr(inp, "_running", False):
        inp._running = False
        time.sleep(0.05)
    # Указатель libei живёт в окне gamescope, а sx/sy — пиксели всего экрана.
    origin = getattr(inp, "pointer_origin", (0, 0)) or (0, 0)
    sx = max(0, int(sx) - int(origin[0]))
    sy = max(0, int(sy) - int(origin[1]))
    if hasattr(inp, "move_abs_local") and inp.move_abs_local(sx, sy):
        time.sleep(0.05)
        return
    # прижать в угол (0,0): диагональ экрана с запасом
    for _ in range(60):
        inp._emit_rel(-200, -200)
        time.sleep(0.0004)
    time.sleep(0.12)
    for _ in range(int(sx)):
        inp._emit_rel(1, 0)
        time.sleep(step_delay)
    for _ in range(int(sy)):
        inp._emit_rel(0, 1)
        time.sleep(step_delay)
    time.sleep(0.1)


# Доля окна под титульную декорацию, если распознать её не удалось (≈49px / типовое окно).
_TITLE_FRAC_FALLBACK = 0.083


class ScreenClicker:
    """Клик по долям КОНТЕНТА окна CS2 (без титульной полосы) → пиксель → курсор+клик.

    Доли (fx, fy) ∈ [0,1] отсчитываются от игрового контента окна, а не от всего окна
    с декорацией. Контентный прямоугольник определяется «парсингом» окна: из геометрии
    вычитается высота титульной полосы, найденная по скриншоту. Это делает мапинг точным
    и устойчивым к смене разрешения/размера окна.
    """

    def __init__(self, menu_input, env: dict | None = None) -> None:
        self._mi = menu_input
        self._env = env
        self._content_top: int | None = None  # кэш высоты титульной полосы (px)

    def set_env(self, env: dict) -> None:
        self._env = env

    def recalibrate(self) -> None:
        """Сбросить кэш смещения контента (напр. после смены разрешения)."""
        self._content_top = None

    def window_rect(self) -> tuple[int, int, int, int] | None:
        if not self._mi._window_id and not self._mi.refresh_window():
            return None
        self._mi.refresh_window()
        # Картинка меню рисуется в рамке mutter, а не в прямоугольнике gamescope:
        # тот сдвинут вниз, и клик по доле окна попадает ниже кнопки.
        decor = self._mi.decor_geometry()
        geo = decor if decor.get("width") else self._mi.geometry()
        if not geo.get("width"):
            return None
        return geo.get("x", 0), geo.get("y", 0), geo["width"], geo.get("height", 0)

    def _title_offset(self, win: tuple[int, int, int, int]) -> int:
        """Высота титульной полосы (px), с кэшем и распознаванием по скриншоту."""
        if self._content_top is not None:
            return self._content_top
        w, h = win[2], win[3]
        off = None
        if self._env is not None:
            shot = "/tmp/_content_probe.png"
            if root_shot(self._env, shot):
                off = detect_content_offset(shot, win)
        if off is None:
            off = int(round(h * _TITLE_FRAC_FALLBACK))
        # разумные границы: 0..25% высоты окна
        off = max(0, min(off, int(h * 0.25)))
        self._content_top = off
        return off

    def content_rect(self) -> tuple[int, int, int, int] | None:
        """Прямоугольник игрового контента окна CS2 (без титульной декорации)."""
        rect = self.window_rect()
        if not rect:
            return None
        x, y, w, h = rect
        off = self._title_offset(rect)
        return x, y + off, w, h - off

    def frac_to_screen(self, fx: float, fy: float) -> tuple[int, int] | None:
        rect = self.content_rect()
        if not rect:
            return None
        x, y, w, h = rect
        return int(round(x + fx * w)), int(round(y + fy * h))

    def capture_client(self):
        """Кадр поля gamescope. Рамки окна на этом кадре нет."""
        from PIL import Image

        from cs2_menu.live_ui import Client, Frame, detect_client

        grab = getattr(self._mi.game_input, "grab_field", None)
        shot = "/tmp/_menu_field.png"
        if grab is not None:
            size = grab(shot)
            if not size:
                return None
            im = Image.open(shot).convert("RGB")
            # Чёрный кадр Xwayland не показывает меню. Модалкой его считать нельзя:
            # иначе ESC закроет поиск. Доли кнопок берутся из FALLBACK.
            if _blank(im):
                return None
            w, h = im.size
            return Frame(im, (0, 0), Client(0, 0, w, h))
        # Вне gamescope — прежний кадр окна на рабочем столе, с вырезанием рамки.
        self._mi.focus()
        rect = self.window_rect()
        if not rect or self._env is None:
            return None
        if not root_shot(self._env, shot):
            return None
        x, y, w, h = rect
        full = Image.open(shot).convert("RGB")
        im = full.crop((
            max(0, x), max(0, y),
            min(full.size[0], x + w), min(full.size[1], y + h),
        ))
        return Frame(im, (x, y), detect_client(im))

    def capture_visible(self):
        """Кадр, на котором видно игру.

        Поле внутри gamescope часто чёрное или без меню: картинка лежит в окне
        на рабочем столе. Пока верхней полосы нет, берётся это окно.
        """
        from PIL import Image

        from cs2_menu.live_ui import Client, Frame, menu_bar_visible

        field = self.capture_client()
        if (
            field is not None
            and not _blank(field.image)
            and menu_bar_visible(field.image, field.client)
        ):
            return field
        rect = self.content_rect()
        host = None
        if rect and self._env is not None:
            shot = "/tmp/_menu_visible.png"
            if grab_rect(self._env, rect, shot):
                im = Image.open(shot).convert("RGB")
                if not _blank(im):
                    w, h = im.size
                    host = Frame(im, (rect[0], rect[1]), Client(0, 0, w, h))
        if host is not None:
            return host
        if field is not None and not _blank(field.image):
            return field
        return None

    def act_client(self, fx: float, fy: float, *, hover: bool = False, hold_ms: int = 70, settle: float = 0.9) -> bool:
        """Клик или наведение по доле поля gamescope. Фокус окна не нужен."""
        return self._use_field(fx, fy, hover=hover, settle=settle)

    def click_live(self, name: str, *, hover: bool = False, hold_ms: int = 70, settle: float = 0.9) -> bool:
        """Клик или наведение по имени. Кадр только уточняет долю; без кадра берётся снятая."""
        from cs2_menu.live_ui import FALLBACK, resolve_point

        pt = None
        fb = FALLBACK.get(name)
        fr = self.capture_client()
        if fr is not None and not _blank(fr.image):
            pt = resolve_point(fr.image, fr.client, name)
        # Кадр легко путает PLAY и STORE: соседняя кнопка на полосе.
        # Далеко от снятой доли кликать нельзя — откроется магазин и капсула.
        if fb is not None and (pt is None or abs(pt[0] - fb[0]) > 0.04 or abs(pt[1] - fb[1]) > 0.04):
            pt = fb
        if pt is None:
            print(f"[menu-click] {name}: нет доли", flush=True)
            return False
        return self._use_field(*pt, hover=hover, settle=settle)

    def _use_field(self, fx: float, fy: float, *, hover: bool, settle: float) -> bool:
        """Доля поля. За его край и на рамку окна курсор не ставится."""
        inp = self._mi.game_input
        fn = getattr(inp, "hover_field" if hover else "click_field", None)
        if fn is None or not fn(fx, fy):
            return False
        time.sleep(min(settle, 0.4) if hover else settle)
        return True

    def click_frac(self, fx: float, fy: float, *, hold_ms: int = 70, settle: float = 0.9) -> bool:
        if getattr(self._mi.game_input, "click_field", None) is not None:
            return self._use_field(fx, fy, hover=False, settle=settle)
        if self._use_field(fx, fy, hover=False, settle=settle):
            return True
        pt = self.frac_to_screen(fx, fy)
        if not pt:
            return False
        return self.click_screen(*pt, hold_ms=hold_ms, settle=settle)

    def _sync_pointer_origin(self) -> None:
        inp = self._mi.game_input
        if not getattr(inp, "pointer_local", False):
            return
        rect = self.window_rect()
        if rect:
            inp.pointer_origin = (rect[0], rect[1])

    def click_screen(self, sx: int, sy: int, *, hold_ms: int = 70, settle: float = 0.9) -> bool:
        self._mi.focus()
        inp = self._mi.game_input
        click_at = getattr(inp, "click_at_screen", None)
        if click_at is not None and click_at(sx, sy):
            time.sleep(settle)
            return True
        self._sync_pointer_origin()
        move_abs(inp, sx, sy)
        inp.mouse_click("left", hold_ms=hold_ms)
        time.sleep(settle)
        return True

    def hover_screen(self, sx: int, sy: int, *, settle: float = 0.35) -> bool:
        """Навести курсор в точку БЕЗ клика (для подсветки/подтверждения кнопки)."""
        self._mi.focus()
        self._sync_pointer_origin()
        move_abs(self._mi.game_input, sx, sy)
        time.sleep(settle)
        return True

    def hover_frac(self, fx: float, fy: float, *, settle: float = 0.35) -> bool:
        if getattr(self._mi.game_input, "hover_field", None) is not None:
            return self._use_field(fx, fy, hover=True, settle=settle)
        if self._use_field(fx, fy, hover=True, settle=settle):
            return True
        pt = self.frac_to_screen(fx, fy)
        if not pt:
            return False
        return self.hover_screen(*pt, settle=settle)
