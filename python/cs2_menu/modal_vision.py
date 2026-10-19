"""Обнаружение и закрытие модальных окон CS2 (ошибки/подтверждения) по видео.

Панорама CS2 показывает модалки так: весь экран затемняется тёмным overlay'ем,
а по центру — более светлая панель с кнопкой (OK/Accept/…). Клавиши ESC/Enter
такие окна (например «Matchmaking Failed — client needs to update») НЕ закрывают —
нужен клик по кнопке. Поэтому:

  * факт модалки определяем по ЯРКОСТИ: при overlay'е средняя яркость рамки окна
    падает (чистое меню ≈56, модалка ≈31);
  * панель находим как яркий прямоугольник в центре на тёмном фоне;
  * кнопку OK кликаем в правом-нижнем углу панели, с запасными точками.

Нужен только PIL (numpy в проекте нет).
"""

from __future__ import annotations

import subprocess
from pathlib import Path

from PIL import Image, ImageStat

SCREEN_W, SCREEN_H = 1920, 1200

# Порог яркости рамки окна: ниже — значит экран затемнён модалкой.
RING_DARK_MAX = 45.0


def grab_window(env: dict, win: tuple[int, int, int, int]) -> Image.Image | None:
    """Снимок ТОЛЬКО области окна CS2 (x,y,w,h) ffmpeg'ом.

    Берём не весь экран, а прямоугольник окна (x11grab +x,y, video_size w×h) — кадр
    меньше ⇒ захват/кодирование заметно быстрее (меньше задержек на каждом наведении).
    """
    disp = env.get("DISPLAY", ":1")
    out = "/tmp/_popup_grab.png"
    x, y, w, h = win
    x0 = max(0, x); y0 = max(0, y)
    w0 = min(SCREEN_W - x0, w); h0 = min(SCREEN_H - y0, h)
    if w0 < 2 or h0 < 2:
        return None
    try:
        subprocess.run(
            ["ffmpeg", "-y", "-f", "x11grab", "-video_size", f"{w0}x{h0}",
             "-i", f"{disp}+{x0},{y0}", "-frames:v", "1", "-update", "1", out],
            env=env, capture_output=True, timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if not Path(out).exists():
        return None
    try:
        return Image.open(out).convert("RGB")
    except Exception:  # noqa: BLE001
        return None


def _ring_mean(gray: Image.Image) -> float:
    w, h = gray.size
    bx, by = max(1, int(w * 0.08)), max(1, int(h * 0.08))
    strips = [
        gray.crop((0, 0, w, by)),
        gray.crop((0, h - by, w, h)),
        gray.crop((0, 0, bx, h)),
        gray.crop((w - bx, 0, w, h)),
    ]
    return sum(ImageStat.Stat(s).mean[0] for s in strips) / 4


def _find_panel(gray: Image.Image, ring_mean: float) -> tuple[int, int, int, int] | None:
    """Bbox светлой центральной панели в координатах окна (x0,y0,x1,y1)."""
    w, h = gray.size
    gw, gh = 131, 87  # ~1/10 масштаб для быстрого скана без numpy
    small = gray.resize((gw, gh))
    px = small.load()
    thr = ring_mean + 15.0
    sx, sy = w / gw, h / gh
    xs: list[int] = []
    ys: list[int] = []
    cx0, cx1 = int(gw * 0.12), int(gw * 0.88)
    cy0, cy1 = int(gh * 0.12), int(gh * 0.88)
    for j in range(cy0, cy1):
        for i in range(cx0, cx1):
            if px[i, j] > thr:
                xs.append(i)
                ys.append(j)
    if len(xs) < 20:
        return None
    xs.sort(); ys.sort()

    def pct(a: list[int], p: float) -> int:
        return a[min(len(a) - 1, max(0, int(len(a) * p)))]

    x0 = int(pct(xs, 0.02) * sx); x1 = int(pct(xs, 0.98) * sx)
    y0 = int(pct(ys, 0.02) * sy); y1 = int(pct(ys, 0.98) * sy)
    pw, ph = x1 - x0, y1 - y0
    # панель-диалог занимает умеренную центральную область; слишком большой bbox —
    # это засвеченный затемнённый арт карты позади, а не диалог → не доверяем
    if pw < w * 0.12 or ph < h * 0.05:
        return None
    if pw > w * 0.65 or ph > h * 0.55:
        return None
    return x0, y0, x1, y1


# Запасные точки закрытия — доли ИГРОВОГО ПОЛЯ (то, что внутри рамки окна).
# Пересчитаны со старых долей контента на снятом окне, без повторного открытия окон.
# Левый нижний угол промо (BUY PRIME) сюда не входит. Зелёную кнопку (CLAIM / GO)
# эти точки не должны накрывать; перед кликом она всё равно отбрасывается.
BUTTON_GRID: list[tuple[float, float]] = [
    (0.666, 0.561),  # OK одиночной ошибки
    (0.500, 0.561),  # центр OK
    (0.581, 0.561),  # YES
    (0.720, 0.933),  # CLOSE промо — правее BUY
]


class ModalInfo:
    def __init__(self, present: bool, ring_mean: float,
                 panel: tuple[int, int, int, int] | None):
        self.present = present
        self.ring_mean = ring_mean
        self.panel = panel  # в координатах окна

    def __repr__(self) -> str:
        return f"ModalInfo(present={self.present}, ring={self.ring_mean:.1f}, panel={self.panel})"


def detect_modal(img: Image.Image) -> ModalInfo:
    gray = img.convert("L")
    ring = _ring_mean(gray)
    present = ring < RING_DARK_MAX
    panel = _find_panel(gray, ring) if present else None
    return ModalInfo(present, ring, panel)


def _not_green(img: Image.Image, x: int, y: int) -> bool:
    """Зелёная кнопка — это GO или «получить». Её при закрытии окна не трогаем."""
    w, h = img.size
    if not (0 <= x < w and 0 <= y < h):
        return False
    r, g, b = img.getpixel((x, y))[:3]
    return not (g > 130 and r < 100 and b < 110 and g > r + 40)


def _panel_dismiss(crop: Image.Image, panel: tuple[int, int, int, int]) -> tuple[int, int] | None:
    """Кнопка закрытия на самой панели: справа, не зелёная, не BUY слева.

    Координаты — пиксели кадра панели (crop). Размер окна сюда не зашит:
    угол панели находится по картинке.
    """
    x0, y0, x1, y1 = panel
    pw, ph = x1 - x0, y1 - y0
    if pw < 40 or ph < 30:
        return None
    px = crop.load()
    xa = x0 + int(pw * 0.42)
    xb = x0 + int(pw * 0.96)
    ya = y0 + int(ph * 0.62)
    yb = y0 + int(ph * 0.94)
    cols: list[int] = []
    ink_y = 0
    ink_n = 0
    for x in range(xa, xb):
        hit = None
        for y in range(ya, yb):
            r, g, b = px[x, y][:3]
            if g > 130 and r < 100 and g > r + 40:
                continue
            if min(r, g, b) > 170 and max(r, g, b) - min(r, g, b) < 50:
                hit = y
                break
        if hit is None:
            continue
        cols.append(x)
        ink_y += hit
        ink_n += 1
    if ink_n < 10:
        return None
    groups: list[tuple[int, int]] = []
    start = prev = cols[0]
    for x in cols[1:]:
        if x - prev > 10:
            if prev - start > 12:
                groups.append((start, prev))
            start = x
        prev = x
    if prev - start > 12:
        groups.append((start, prev))
    if not groups:
        return None
    a, b = groups[-1]
    return (a + b) // 2, ink_y // ink_n


def is_modal_present(clicker, env: dict) -> bool:
    """Быстрая проверка: висит ли сейчас модальное окно (с фокусом окна CS2)."""
    clicker._mi.focus()
    fr = _client_view(clicker)
    if fr is None:
        return False
    return _modal_on_client(fr)[1].present


def _client_view(clicker):
    """Кадр окна и поле игры. None, если снять не удалось."""
    if not hasattr(clicker, "capture_client"):
        return None
    return clicker.capture_client()


def _modal_on_client(fr):
    crop = fr.image.crop((
        fr.client.left, fr.client.top,
        fr.client.left + fr.client.width, fr.client.top + fr.client.height,
    ))
    info = detect_modal(crop)
    return crop, info


def clear_modals(clicker, env: dict, *, max_rounds: int = 2, log=print) -> bool:
    """Закрывать модалки, пока экран не станет «чистым». True — чисто/закрыто.

    Панель и кнопка ищутся на игровом поле текущего кадра, поэтому размер окна
    и полный экран не требуют заново открывать эти окна. Зелёная «получить» / GO
    не нажимается.
    """
    import time as _t

    closed = True
    for rnd in range(max_rounds):
        clicker._mi.focus()
        fr = _client_view(clicker)
        if fr is None:
            log("[popup] окно CS2 не найдено")
            return False
        _crop, info = _modal_on_client(fr)
        if not info.present:
            if rnd:
                log(f"[popup] закрыто (ring={info.ring_mean:.0f})")
            return True
        _t.sleep(0.4)
        fr2 = _client_view(clicker)
        if fr2 is None:
            return False
        crop2, info2 = _modal_on_client(fr2)
        if not info2.present:
            log("[popup] тёмный кадр не подтвердился как окно — пропускаю (вероятно, игра)")
            return True
        log(f"[popup] обнаружена модалка {info2} — закрываю")
        closed = False
        points: list[tuple[float, float]] = []
        if info2.panel:
            local = _panel_dismiss(crop2, info2.panel)
            if local is not None and _not_green(crop2, *local):
                points.append(fr2.client.frac(
                    fr2.client.left + local[0], fr2.client.top + local[1],
                ))
        for fx, fy in BUTTON_GRID:
            if (fx, fy) not in points:
                points.append((fx, fy))
        for fx, fy in points:
            px, py = fr2.client.pixel(fx, fy)
            if not _not_green(fr2.image, px, py):
                log(f"[popup] пропускаю зелёную кнопку @({fx:.3f},{fy:.3f})")
                continue
            clicker.act_client(fx, fy, settle=0.9)
            chk = _client_view(clicker)
            if chk is not None and not _modal_on_client(chk)[1].present:
                log(f"[popup] закрыта кликом @({fx:.3f},{fy:.3f})")
                closed = True
                break
        if not closed:
            log("[popup] кандидаты не помогли, повтор…")
    if not closed:
        log("[popup] не удалось закрыть окно за отведённые попытки")
    return False
