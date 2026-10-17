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


# БЕЗОПАСНЫЕ позиции кнопок закрытия (доли окна CS2). Только мид-скрин OK/YES и
# правый-нижний CLOSE промо-модалки. Сознательно НЕ трогаем левый-нижний угол,
# где у промо-окон кнопка покупки (BUY PRIME) — чтобы случайно её не нажать.
# Порядок: сначала самые частые/безопасные; на ошибке первая же закрывает окно.
# Доли КОНТЕНТА окна CS2 (без титульной полосы) — согласованы с ScreenClicker.content_rect.
BUTTON_GRID: list[tuple[float, float]] = [
    (0.662, 0.549),  # OK одиночной ошибки (Matchmaking Failed/update)
    (0.500, 0.549),  # центр OK
    (0.579, 0.549),  # YES (Leave Match)
    (0.715, 0.912),  # CLOSE промо-модалки (Prime) — правее кнопки BUY
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


def dismiss_candidates(win: tuple[int, int, int, int],
                       panel: tuple[int, int, int, int] | None) -> list[tuple[int, int]]:
    """Экранные точки-кандидаты для закрытия окна.

    Если удалось надёжно найти панель — кнопка OK в её правом-нижнем углу (и центр).
    Иначе (или в дополнение) — типовая сетка позиций кнопок. Безопасно: модалка
    поглощает клики мимо кнопок, поэтому лишние точки ничего не ломают.
    """
    wx, wy, ww, wh = win
    pts: list[tuple[int, int]] = []
    if panel:
        x0, y0, x1, y1 = panel
        pw, ph = x1 - x0, y1 - y0
        pts.append((wx + int(x0 + 0.88 * pw), wy + int(y0 + 0.82 * ph)))  # OK справа-внизу
        pts.append((wx + int(x0 + 0.5 * pw), wy + int(y0 + 0.82 * ph)))   # центр-внизу
    for fx, fy in BUTTON_GRID:
        pt = (wx + int(fx * ww), wy + int(fy * wh))
        if pt not in pts:
            pts.append(pt)
    return pts


def is_modal_present(clicker, env: dict) -> bool:
    """Быстрая проверка: висит ли сейчас модальное окно (с фокусом окна CS2)."""
    clicker._mi.focus()
    rect = clicker.content_rect()
    if not rect:
        return False
    img = grab_window(env, rect)
    if img is None:
        return False
    return detect_modal(img).present


def clear_modals(clicker, env: dict, *, max_rounds: int = 2, log=print) -> bool:
    """Закрывать модалки, пока экран не станет «чистым». True — чисто/закрыто.

    ВАЖНО: перед снимком фокусируем окно CS2 — иначе поверх него может быть
    IDE/другое окно, его тёмная область ложно примется за модалку, и пойдут
    слепые клики. Фокус + анализ яркости рамки даёт честную картину.
    """
    closed = True
    for rnd in range(max_rounds):
        clicker._mi.focus()  # поднять CS2 поверх прочих окон перед снимком
        rect = clicker.content_rect()
        if not rect:
            log("[popup] окно CS2 не найдено")
            return False
        img = grab_window(env, rect)
        if img is None:
            return False
        info = detect_modal(img)
        if not info.present:
            if rnd:
                log(f"[popup] закрыто (ring={info.ring_mean:.0f})")
            return True
        # подтверждение: модалка должна держаться 2 кадра подряд. Иначе это может
        # быть тёмный кадр геймплея (в игре), а не настоящее окно — не кликаем.
        import time as _t
        _t.sleep(0.4)
        img2 = grab_window(env, rect)
        if img2 is None or not detect_modal(img2).present:
            log("[popup] тёмный кадр не подтвердился как окно — пропускаю (вероятно, игра)")
            return True
        log(f"[popup] обнаружена модалка {info} — закрываю")
        closed = False
        for (sx, sy) in dismiss_candidates(rect, info.panel):
            clicker.click_screen(sx, sy, settle=0.9)
            chk = grab_window(env, rect)
            if chk is not None and not detect_modal(chk).present:
                log(f"[popup] закрыта кликом ({sx},{sy})")
                closed = True
                break
        if not closed:
            log("[popup] кандидаты не помогли, повтор…")
    if not closed:
        log("[popup] не удалось закрыть окно за отведённые попытки")
    return False
