"""Меню CS2 по живому кадру.

Доли — от игрового поля внутри окна, не от рамки. Рамка (заголовок и боковые
полосы) каждый раз находится по картинке, поэтому то же поле в окне, без рамки
и на весь экран попадает в те же доли. Вкладки ищутся по порядку ярких слов в
своей полосе, GO — как большая зелёная кнопка, плитки карт — сеткой. Сдвиг
сетки при другой пропорции не оставляет курсор на старом проценте.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from PIL import Image

NAV_ORDER = ("inventory", "loadout", "play", "store", "news")
SUB_ORDER = ("matchmaking", "practice", "workshop")
MODE_ORDER = (
    "premier",
    "competitive",
    "wingman",
    "rush",
    "casual",
    "deathmatch",
    "arms_race",
    "retakes",
    "private",
)
ICON_ORDER = ("home", "watch", "settings", "power")


@dataclass(frozen=True)
class Client:
    """Игровое поле в пикселях снимка окна."""

    left: int
    top: int
    width: int
    height: int

    def frac(self, x: int, y: int) -> tuple[float, float]:
        return (
            (x - self.left) / max(1, self.width - 1),
            (y - self.top) / max(1, self.height - 1),
        )

    def pixel(self, fx: float, fy: float) -> tuple[int, int]:
        return (
            self.left + int(round(fx * max(1, self.width - 1))),
            self.top + int(round(fy * max(1, self.height - 1))),
        )


def detect_client(im: Image.Image) -> Client:
    """Поле игры внутри снимка окна. Без рамки возвращает весь кадр."""
    w, h = im.size
    px = im.load()

    def row_mean(y: int) -> tuple[float, float]:
        s = n = dark = 0
        for x in range(0, w, 4):
            r, g, b = px[x, y][:3]
            lum = (r + g + b) / 3.0
            s += lum
            n += 1
            if lum < 12:
                dark += 1
        return s / max(1, n), dark / max(1, n)

    def col_mean(x: int, y0: int, y1: int) -> float:
        s = n = 0
        for y in range(y0, y1, 6):
            r, g, b = px[x, y][:3]
            s += (r + g + b) / 3.0
            n += 1
        return s / max(1, n)

    # Тонкая чёрная линия под заголовком. У окна без рамки её нет — поле с y=0.
    top = 0
    best_y, best_m = 0, 1e9
    for y in range(16, min(h // 3, 180)):
        mean, dark = row_mean(y)
        if dark > 0.55 and mean < best_m:
            best_m = mean
            best_y = y
    if best_m < 15.0:
        top = best_y + 1

    y0 = min(h - 2, top + 8)
    y1 = min(h - 1, top + max(100, h // 6))
    left = 0
    prev = 0.0
    for x in range(0, w // 4):
        m = col_mean(x, y0, y1)
        if m > 45.0 and prev < 35.0:
            left = x
            break
        prev = m
    right = w
    prev = 0.0
    for x in range(w - 1, (w * 3) // 4, -1):
        m = col_mean(x, y0, y1)
        if m > 45.0 and prev < 35.0:
            right = x + 1
            break
        prev = m
    bottom = h
    for y in range(h - 1, (h * 3) // 4, -1):
        mean, _dark = row_mean(y)
        if mean > 45.0:
            bottom = y + 1
            break
    if right - left < 200 or bottom - top < 200:
        return Client(0, 0, w, h)
    return Client(left, top, right - left, bottom - top)


def _runs_on_row(im: Image.Image, y: int, x0: int, x1: int, pred) -> list[tuple[int, int]]:
    px = im.load()
    runs: list[tuple[int, int]] = []
    start = None
    prev = x0
    for x in range(x0, x1):
        if pred(px[x, y][:3]):
            if start is None:
                start = x
            prev = x
        elif start is not None:
            # буквы внутри слова стоят с зазором в несколько пикселей
            if x - prev > 7:
                if prev - start >= 8:
                    runs.append((start, prev))
                start = None
    if start is not None and prev - start >= 8:
        runs.append((start, prev))
    return runs


def _clusters(
    im: Image.Image,
    y0: int,
    y1: int,
    x0: int,
    x1: int,
    pred,
) -> list[tuple[int, int, int, int]]:
    """Слова на самой плотной строке полосы: (x0, y, x1, y)."""
    w, h = im.size
    y0 = max(0, y0)
    y1 = min(h - 1, y1)
    x0 = max(0, x0)
    x1 = min(w, x1)
    best: list[tuple[int, int]] = []
    best_y = y0
    best_w = 0
    for y in range(y0, y1):
        runs = _runs_on_row(im, y, x0, x1, pred)
        width = sum(b - a for a, b in runs)
        if width > best_w:
            best_w = width
            best = runs
            best_y = y
    return [(a, best_y, b, best_y + 1) for a, b in best]


def _white(c: tuple[int, int, int]) -> bool:
    r, g, b = c
    return r > 168 and g > 168 and b > 168


def _blue(c: tuple[int, int, int]) -> bool:
    r, g, b = c
    return b > 150 and b > r + 25 and g > 80 and r < 170


def _gray_label(c: tuple[int, int, int]) -> bool:
    """Невыбранная подпись: серее фона полосы, но не белая."""
    r, g, b = c
    return 112 < r < 175 and abs(r - g) < 20 and abs(g - b) < 20


def _merge_selected(
    white: list[tuple[int, int, int, int]],
    blue: list[tuple[int, int, int, int]],
) -> list[tuple[int, int, int, int]]:
    """Синяя выбранная вкладка не попадает в белый список — вставляем её по X."""
    out = list(white)
    for box in blue:
        cx = (box[0] + box[2]) / 2
        if any(abs((b[0] + b[2]) / 2 - cx) < 18 for b in out):
            continue
        out.append(box)
    out.sort(key=lambda b: (b[0] + b[2]) / 2)
    return out


def _named_row(
    boxes: list[tuple[int, int, int, int]],
    names: tuple[str, ...],
    client: Client,
) -> dict[str, tuple[float, float]]:
    if len(boxes) < len(names):
        return {}
    # лишние короткие куски (значок NEW, разделители) выкидываем, оставляя самые широкие
    # в левом-правом порядке: берём группы, близкие по Y к самой населённой линии
    ys = sorted((b[1] + b[3]) / 2 for b in boxes)
    mid = ys[len(ys) // 2]
    line = [b for b in boxes if abs((b[1] + b[3]) / 2 - mid) < 14]
    line.sort(key=lambda b: b[0])
    if len(line) > len(names):
        line = sorted(line, key=lambda b: b[2] - b[0], reverse=True)[: len(names)]
        line.sort(key=lambda b: b[0])
    if len(line) != len(names):
        return {}
    found: dict[str, tuple[float, float]] = {}
    for name, (x0, y0, x1, y1) in zip(names, line):
        found[name] = client.frac((x0 + x1) // 2, (y0 + y1) // 2)
    return found


def find_nav(im: Image.Image, client: Client) -> dict[str, tuple[float, float]]:
    y0 = client.top + int(client.height * 0.012)
    y1 = client.top + int(client.height * 0.055)
    x0 = client.left + int(client.width * 0.22)
    x1 = client.left + int(client.width * 0.78)
    boxes = _merge_selected(
        _clusters(im, y0, y1, x0, x1, _white),
        _clusters(im, y0, y1, x0, x1, _blue),
    )
    return _named_row(boxes, NAV_ORDER, client)


def find_icons(im: Image.Image, client: Client) -> dict[str, tuple[float, float]]:
    y0 = client.top + int(client.height * 0.012)
    y1 = client.top + int(client.height * 0.055)
    x1 = client.left + int(client.width * 0.16)
    boxes = _clusters(im, y0, y1, client.left, x1, _white)
    boxes = [b for b in boxes if 8 <= (b[2] - b[0]) <= 28]
    boxes.sort(key=lambda b: b[0])
    # слить куски одной иконки
    merged: list[tuple[int, int, int, int]] = []
    for box in boxes:
        if merged and box[0] - merged[-1][2] < 10:
            p = merged[-1]
            merged[-1] = (p[0], min(p[1], box[1]), box[2], max(p[3], box[3]))
        else:
            merged.append(box)
    if len(merged) < 3:
        return {}
    merged = merged[: len(ICON_ORDER)]
    return {
        name: client.frac((b[0] + b[2]) // 2, (b[1] + b[3]) // 2)
        for name, b in zip(ICON_ORDER, merged)
    }


def _ink_runs(
    im: Image.Image,
    client: Client,
    y_lo: float,
    y_hi: float,
    preds,
) -> list[tuple[float, float]]:
    """Слова в полосе: дырки между буквами склеиваются, слова — нет."""
    px = im.load()
    y0 = client.top + int(client.height * y_lo)
    y1 = client.top + int(client.height * y_hi)
    x0 = client.left + int(client.width * 0.12)
    x1 = client.left + int(client.width * 0.90)
    raw = []
    for x in range(x0, x1):
        hit = False
        for y in range(y0, y1, 1):
            p = px[x, y][:3]
            if any(pred(p) for pred in preds):
                hit = True
                break
        raw.append(hit)
    bridged = raw[:]
    for i in range(len(bridged)):
        if bridged[i]:
            continue
        left = any(i - k >= 0 and raw[i - k] for k in range(1, 6))
        right = any(i + k < len(raw) and raw[i + k] for k in range(1, 6))
        if left and right:
            bridged[i] = True
    runs: list[tuple[int, int]] = []
    start = None
    for i, v in enumerate(bridged):
        if v and start is None:
            start = i
        elif not v and start is not None:
            if i - start >= 12:
                runs.append((start, i))
            start = None
    if start is not None and len(bridged) - start >= 12:
        runs.append((start, len(bridged)))
    found = []
    mid_y = (y0 + y1) // 2
    for a, b in runs:
        found.append(client.frac(x0 + (a + b) // 2, mid_y))
    return found


def find_band(
    im: Image.Image,
    client: Client,
    names: tuple[str, ...],
    y_lo: float,
    y_hi: float,
) -> dict[str, tuple[float, float]]:
    if names == MODE_ORDER:
        pts = _ink_runs(im, client, y_lo, y_hi, (_white, _blue, _gray_label))
        if len(pts) < len(names):
            return {}
        pts = pts[: len(names)]
        return {name: pt for name, pt in zip(names, pts)}
    y0 = client.top + int(client.height * y_lo)
    y1 = client.top + int(client.height * y_hi)
    x0 = client.left + int(client.width * 0.18)
    x1 = client.left + int(client.width * 0.92)
    boxes = _merge_selected(
        _clusters(im, y0, y1, x0, x1, _white),
        _clusters(im, y0, y1, x0, x1, _blue),
    )
    return _named_row(boxes, names, client)


def find_modes(im: Image.Image, client: Client) -> dict[str, tuple[float, float]]:
    return find_band(im, client, MODE_ORDER, 0.108, 0.132)


def _ink_row(im: Image.Image, x0: int, x1: int, y0: int, y1: int) -> int:
    """Строка, где в этом столбце больше всего белой краски."""
    px = im.load()
    y0 = max(0, y0)
    y1 = min(im.size[1], y1)
    best_y = (y0 + y1) // 2
    best = 0
    for y in range(y0, y1):
        n = 0
        for x in range(x0, x1):
            if _white_ink(px[x, y][:3]):
                n += 1
        if n > best:
            best = n
            best_y = y
    return best_y


def _white_ink(rgb: tuple[int, int, int]) -> bool:
    r, g, b = rgb
    return min(r, g, b) > 188 and max(r, g, b) - min(r, g, b) < 45


def _column_groups(
    im: Image.Image,
    x0: int,
    x1: int,
    y0: int,
    y1: int,
    *,
    bridge: int,
    min_width: int,
) -> list[tuple[int, int, int]]:
    """Группы белых столбцов: (x начала, x конца, y середины полосы)."""
    px = im.load()
    y0 = max(0, y0)
    y1 = min(im.size[1], y1)
    x0 = max(0, x0)
    x1 = min(im.size[0], x1)
    if y1 - y0 < 4 or x1 - x0 < 8:
        return []
    hit = []
    for x in range(x0, x1):
        n = 0
        for y in range(y0, y1):
            if _white_ink(px[x, y][:3]):
                n += 1
                if n >= 2:
                    break
        hit.append(n >= 2)
    groups: list[tuple[int, int, int]] = []
    start = None
    last = None
    for i, on in enumerate(hit):
        if on:
            if start is None:
                start = i
            last = i
        elif start is not None and i - last > bridge:
            if last - start >= min_width:
                groups.append((x0 + start, x0 + last, (y0 + y1) // 2))
            start = None
    if start is not None and last is not None and last - start >= min_width:
        groups.append((x0 + start, x0 + last, (y0 + y1) // 2))
    return groups


def find_team_select(im: Image.Image, client: Client) -> dict[str, tuple[float, float]]:
    """Экран выбора стороны: заголовки команд и CANCEL / SPECTATE / AUTO SELECT.

    Кнопки ищутся как белые надписи текущего кадра, поэтому другая ширина окна
    и полный экран не сдвигают точку.
    """
    found: dict[str, tuple[float, float]] = {}
    w, h = im.size
    bridge = max(6, int(w * 0.011))
    min_width = max(18, int(w * 0.025))
    buttons = _column_groups(
        im,
        int(w * 0.55),
        w - 4,
        int(h * 0.86),
        h - 2,
        bridge=bridge,
        min_width=min_width,
    )
    names = ("cancel", "spectate", "auto_select")
    y_band0, y_band1 = int(h * 0.86), h - 2
    if len(buttons) >= 3:
        chosen = list(zip(names, buttons[-3:]))
    elif len(buttons) == 1:
        chosen = [("auto_select", buttons[0])]
    else:
        chosen = []
    for name, (x0, x1, _y) in chosen:
        y = _ink_row(im, x0, x1, y_band0, y_band1)
        found[name] = client.frac((x0 + x1) // 2, y)

    def title(x0f: float, x1f: float) -> tuple[float, float] | None:
        x0 = client.left + int(client.width * x0f)
        x1 = client.left + int(client.width * x1f)
        y0 = client.top + int(client.height * 0.10)
        y1 = client.top + int(client.height * 0.30)
        px = im.load()
        best_y = None
        best_n = 0
        for y in range(y0, y1):
            n = 0
            for x in range(x0, x1, 2):
                if _white_ink(px[x, y][:3]):
                    n += 1
            if n > best_n:
                best_n = n
                best_y = y
        if best_y is None or best_n < 12:
            return None
        groups = _column_groups(
            im, x0, x1, best_y - 8, best_y + 12,
            bridge=max(8, int(client.width * 0.02)),
            min_width=max(24, int(client.width * 0.05)),
        )
        if not groups:
            return None
        x_a, x_b, _y = max(groups, key=lambda g: g[1] - g[0])
        return client.frac((x_a + x_b) // 2, _ink_row(im, x_a, x_b, y0, y1))

    t = title(0.06, 0.48)
    ct = title(0.52, 0.94)
    if t is not None:
        found["team_t"] = t
    if ct is not None:
        found["team_ct"] = ct
    return found


def _is_green(rgb: tuple[int, int, int]) -> bool:
    r, g, b = rgb
    return g > 130 and r < 100 and b < 110 and g > r + 40


def _price_green(rgb: tuple[int, int, int]) -> bool:
    """Зелёная цена «Free» на карточке. Крупная кнопка CLAIM сюда не попадает: она правее."""
    r, g, b = rgb
    return g > 100 and r < 140 and g > r + 25 and g > b + 15


def find_drop_cards(im: Image.Image, client: Client) -> list[tuple[float, float]]:
    """Четыре предмета Weekly Care Package. Точка — картинка карточки, не цена и не i.

    Карточки стоят левее зелёной CLAIM. У каждой слева внизу зелёная цена.
    Четыре таких подписи в одном ряду и есть блок дропа.
    """
    px = im.load()
    claim = find_green(im, client)
    x_lim = client.left + int(client.width * (claim[0] - 0.08 if claim else 0.62))
    y0 = client.top + int(client.height * 0.30)
    y1 = client.top + int(client.height * 0.46)
    x_from = client.left + int(client.width * 0.08)
    cols: list[int] = []
    label_y = 0
    label_n = 0
    for x in range(x_from, x_lim):
        hit_y = None
        for y in range(y0, y1):
            if _price_green(px[x, y][:3]):
                hit_y = y
                break
        if hit_y is None:
            continue
        cols.append(x)
        label_y += hit_y
        label_n += 1
    if label_n < 8:
        return []
    groups: list[tuple[int, int]] = []
    start = prev = cols[0]
    for x in cols[1:]:
        if x - prev > max(14, int(client.width * 0.015)):
            if prev - start >= 6:
                groups.append((start, prev))
            start = x
        prev = x
    if prev - start >= 6:
        groups.append((start, prev))
    if len(groups) != 4:
        return []
    gaps = [groups[i + 1][0] - groups[i][0] for i in range(3)]
    pitch = sorted(gaps)[1]
    if pitch < int(client.width * 0.06):
        return []
    label_y = label_y // label_n
    click_y = max(client.top + 4, label_y - int(client.height * 0.10))
    out = []
    for a, _b in groups:
        cx = a + int(pitch * 0.38)
        out.append(client.frac(cx, click_y))
    return out


def find_inspect_close(im: Image.Image, client: Client) -> tuple[float, float] | None:
    """Белая кнопка CLOSE осмотра предмета. В меню и на выборе стороны её нет."""
    if find_nav(im, client):
        return None
    teams = find_team_select(im, client)
    if "auto_select" in teams or "cancel" in teams:
        return None
    w, h = im.size
    groups = _column_groups(
        im,
        int(w * 0.58),
        w - 4,
        int(h * 0.88),
        h - 2,
        bridge=max(6, int(w * 0.012)),
        min_width=max(18, int(w * 0.03)),
    )
    if not groups:
        return None
    x0, x1, _y = groups[-1]
    y = _ink_row(im, x0, x1, int(h * 0.88), h - 2)
    cx, cy = (x0 + x1) // 2, y
    if 0 <= cx < w and 0 <= cy < h and _is_green(im.getpixel((cx, cy))[:3]):
        return None
    return client.frac(cx, cy)


def find_green(im: Image.Image, client: Client) -> tuple[float, float] | None:
    """Самое большое зелёное пятно в нижней правой части — кнопка GO / Получить."""
    px = im.load()
    x0 = client.left + int(client.width * 0.25)
    y0 = client.top + int(client.height * 0.15)
    x1 = client.left + client.width - 4
    y1 = client.top + client.height - 4
    best: tuple[int, int, int, int] | None = None
    best_n = 0
    seen: set[tuple[int, int]] = set()
    for y in range(y0, y1, 2):
        for x in range(x0, x1, 2):
            if (x, y) in seen:
                continue
            r, g, b = px[x, y][:3]
            if not (g > 140 and r < 110 and b < 150 and g > r + 40 and g > b + 20):
                continue
            stack = [(x, y)]
            seen.add((x, y))
            xa = xb = x
            ya = yb = y
            n = 0
            while stack:
                cx, cy = stack.pop()
                n += 1
                xa, xb = min(xa, cx), max(xb, cx)
                ya, yb = min(ya, cy), max(yb, cy)
                for nx, ny in ((cx + 2, cy), (cx - 2, cy), (cx, cy + 2), (cx, cy - 2)):
                    if nx < x0 or nx >= x1 or ny < y0 or ny >= y1 or (nx, ny) in seen:
                        continue
                    rr, gg, bb = px[nx, ny][:3]
                    if gg > 140 and rr < 110 and bb < 150 and gg > rr + 40 and gg > bb + 20:
                        seen.add((nx, ny))
                        stack.append((nx, ny))
            if n > best_n and (xb - xa) > 40 and (yb - ya) > 12:
                best_n = n
                best = (xa, ya, xb, yb)
    if best is None:
        return None
    return client.frac((best[0] + best[2]) // 2, (best[1] + best[3]) // 2)


class Frame:
    """Снимок окна и поле игры. Доли переводятся в пиксели экрана."""

    def __init__(self, image: Image.Image, origin: tuple[int, int], client: Client):
        self.image = image
        self.origin = origin
        self.client = client

    def screen(self, fx: float, fy: float) -> tuple[int, int]:
        x, y = self.client.pixel(fx, fy)
        return self.origin[0] + x, self.origin[1] + y


# Доли поля 1280×800, снятые с живого меню. Если надпись на кадре не видна,
# берётся эта доля того же поля: рамка окна в неё не входит.
FALLBACK: dict[str, tuple[float, float]] = {
    "play": (0.512, 0.028),
    "inventory": (0.381, 0.028),
    "loadout": (0.449, 0.028),
    "store": (0.581, 0.028),
    "news": (0.631, 0.028),
    "home": (0.013, 0.028),
    "watch": (0.042, 0.028),
    "settings": (0.070, 0.028),
    "matchmaking": (0.422, 0.076),
    "practice": (0.481, 0.076),
    "workshop": (0.544, 0.076),
    "premier": (0.271, 0.118),
    "competitive": (0.326, 0.118),
    "wingman": (0.382, 0.118),
    "rush": (0.424, 0.118),
    "casual": (0.463, 0.118),
    "deathmatch": (0.517, 0.118),
    "arms_race": (0.577, 0.118),
    "retakes": (0.629, 0.118),
    "private": (0.703, 0.118),
    "go": (0.865, 0.956),
    "claim": (0.729, 0.353),
    "map_group_alpha": (0.288, 0.50),
    "map_group_delta": (0.459, 0.50),
    "map_group_community": (0.644, 0.50),
    "map_group_hostage": (0.823, 0.50),
    "mirage": (0.506, 0.316),
    "cache": (0.234, 0.316),
    "anubis": (0.324, 0.316),
    "inferno": (0.416, 0.316),
    "dust2": (0.596, 0.316),
    "nuke": (0.688, 0.316),
    "ancient": (0.779, 0.316),
    "train": (0.870, 0.316),
    "team_t": (0.267, 0.144),
    "team_ct": (0.731, 0.144),
    "cancel": (0.767, 0.971),
    "spectate": (0.840, 0.958),
    "auto_select": (0.919, 0.958),
}

TEAM_NAMES = ("team_t", "team_ct", "cancel", "spectate", "auto_select")

STORE_TABS = ("store_home", "general", "market", "tools", "armory")


def resolve_point(im: Image.Image, client: Client, name: str) -> tuple[float, float] | None:
    """Точка по текущему кадру. None — на этом экране надписи нет, берут FALLBACK."""
    if name in TEAM_NAMES:
        return find_team_select(im, client).get(name)
    if name in ("go", "claim"):
        return find_green(im, client)
    nav = find_nav(im, client)
    if name in nav:
        return nav[name]
    icons = find_icons(im, client)
    if name in icons and name != "power":
        return icons[name]
    modes = find_modes(im, client)
    if name in modes:
        return modes[name]
    sub = find_band(im, client, ("matchmaking", "practice", "workshop"), 0.055, 0.105)
    if name in sub:
        return sub[name]
    if name in STORE_TABS or name == "store_home":
        tabs = find_band(
            im, client, ("home", "general", "market", "tools", "armory"), 0.055, 0.11,
        )
        key = "home" if name == "store_home" else name
        if key in tabs:
            return tabs[key]
    return None


def find_cards(im: Image.Image, client: Client, y_frac: float = 0.42) -> list[tuple[float, float]]:
    """Центры плиток в одном горизонтальном ряду. Ряд задаётся долей высоты поля."""
    y = client.top + int(client.height * y_frac)
    y = max(client.top + 2, min(client.top + client.height - 3, y))
    px = im.load()
    x0 = client.left + int(client.width * 0.12)
    x1 = client.left + client.width - int(client.width * 0.04)
    lum = []
    for x in range(x0, x1):
        r, g, b = px[x, y][:3]
        lum.append((r + g + b) / 3.0)
    if len(lum) < 40:
        return []
    # щели между плитками темнее самих плиток
    smooth = []
    for i in range(len(lum)):
        a = max(0, i - 3)
        b = min(len(lum), i + 4)
        smooth.append(sum(lum[a:b]) / (b - a))
    lo = sorted(smooth)[len(smooth) // 5]
    hi = sorted(smooth)[(len(smooth) * 4) // 5]
    if hi - lo < 12:
        return []
    thr = (lo + hi) / 2
    runs: list[tuple[int, int]] = []
    start = None
    for i, v in enumerate(smooth):
        if v >= thr:
            if start is None:
                start = i
        elif start is not None:
            if i - start > 28:
                runs.append((start, i))
            start = None
    if start is not None and len(smooth) - start > 28:
        runs.append((start, len(smooth)))
    centers = []
    for a, b in runs:
        cx = x0 + (a + b) // 2
        centers.append(client.frac(cx, y))
    return centers
