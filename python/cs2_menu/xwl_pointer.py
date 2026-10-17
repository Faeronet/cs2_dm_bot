"""Надёжный ввод мышью в меню CS2 на XWayland (GNOME/mutter).

Почему так:
  * xdotool mousemove двигает X-указатель, который композитор Wayland ИГНОРИРУЕТ —
    клик uinput срабатывает в позиции курсора КОМПОЗИТОРА, а не X.
  * Поэтому позиционируем курсор ОТНОСИТЕЛЬНЫМИ дельтами uinput (ядро → libinput →
    композитор). При flat-профиле ускорения и speed=0 одна дельта = один пиксель.
  * Курсор «прижимаем» в угол (0,0) большими отрицательными дельтами, затем шагаем
    на (x, y) — получается детерминированная абсолютная позиция.
  * Скриншот — ffmpeg x11grab (пооконный xwd даёт мусор, xwd -root — BadColor).

Координаты кликов задаём ДОЛЯМИ прямоугольника окна CS2 (устойчиво к переносу окна).
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


def root_shot(env: dict, out: str, *, size: tuple[int, int] = (SCREEN_W, SCREEN_H)) -> bool:
    disp = env.get("DISPLAY", ":1")
    w, h = size
    try:
        subprocess.run(
            ["ffmpeg", "-y", "-f", "x11grab", "-video_size", f"{w}x{h}",
             "-i", f"{disp}+0,0", "-frames:v", "1", "-update", "1", out],
            env=env, capture_output=True, timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return Path(out).exists()


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


def move_abs(inp, sx: int, sy: int, *, step_delay: float = 0.0003) -> None:
    """Абсолютное позиционирование курсора композитора через относительные дельты."""
    # остановить фоновое сглаживание мыши, если включено
    if getattr(inp, "_running", False):
        inp._running = False
        time.sleep(0.05)
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
        geo = self._mi.geometry()
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

    def click_frac(self, fx: float, fy: float, *, hold_ms: int = 70, settle: float = 0.9) -> bool:
        pt = self.frac_to_screen(fx, fy)
        if not pt:
            return False
        return self.click_screen(*pt, hold_ms=hold_ms, settle=settle)

    def click_screen(self, sx: int, sy: int, *, hold_ms: int = 70, settle: float = 0.9) -> bool:
        self._mi.focus()
        move_abs(self._mi.game_input, sx, sy)
        self._mi.game_input.mouse_click("left", hold_ms=hold_ms)
        time.sleep(settle)
        return True

    def hover_screen(self, sx: int, sy: int, *, settle: float = 0.35) -> bool:
        """Навести курсор в точку БЕЗ клика (для подсветки/подтверждения кнопки)."""
        self._mi.focus()
        move_abs(self._mi.game_input, sx, sy)
        time.sleep(settle)
        return True

    def hover_frac(self, fx: float, fy: float, *, settle: float = 0.35) -> bool:
        pt = self.frac_to_screen(fx, fy)
        if not pt:
            return False
        return self.hover_screen(*pt, settle=settle)
