"""Рисование карты/маршрута/траектории в PNG без внешних зависимостей."""

from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path

from cs2_nav.coord_map import CoordinateMap

Color = tuple[int, int, int]


class Canvas:
    def __init__(self, width: int, height: int, bg: Color = (24, 24, 28)):
        self.w = width
        self.h = height
        self.buf = bytearray(bytes(bg) * (width * height))

    def set(self, x: int, y: int, color: Color) -> None:
        if 0 <= x < self.w and 0 <= y < self.h:
            i = (y * self.w + x) * 3
            self.buf[i : i + 3] = bytes(color)

    def fill_rect(self, x0: int, y0: int, x1: int, y1: int, color: Color) -> None:
        x0, x1 = max(0, min(x0, x1)), min(self.w - 1, max(x0, x1))
        y0, y1 = max(0, min(y0, y1)), min(self.h - 1, max(y0, y1))
        row = bytes(color) * (x1 - x0 + 1)
        for y in range(y0, y1 + 1):
            i = (y * self.w + x0) * 3
            self.buf[i : i + len(row)] = row

    def disc(self, cx: float, cy: float, r: float, color: Color) -> None:
        for y in range(int(cy - r), int(cy + r) + 1):
            for x in range(int(cx - r), int(cx + r) + 1):
                if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                    self.set(x, y, color)

    def ring(self, cx: float, cy: float, r: float, color: Color, width: float = 1.5) -> None:
        for y in range(int(cy - r - width), int(cy + r + width) + 1):
            for x in range(int(cx - r - width), int(cx + r + width) + 1):
                d = math.hypot(x - cx, y - cy)
                if abs(d - r) <= width:
                    self.set(x, y, color)

    def line(self, x0: float, y0: float, x1: float, y1: float, color: Color, width: float = 1.0) -> None:
        dx, dy = x1 - x0, y1 - y0
        n = int(max(abs(dx), abs(dy))) + 1
        r = width * 0.5
        for i in range(n + 1):
            t = i / n
            x, y = x0 + dx * t, y0 + dy * t
            if r <= 0.6:
                self.set(int(round(x)), int(round(y)), color)
            else:
                self.disc(x, y, r, color)

    def save(self, path: Path | str) -> None:
        raw = bytearray()
        stride = self.w * 3
        for y in range(self.h):
            raw.append(0)
            raw += self.buf[y * stride : (y + 1) * stride]

        def chunk(tag: bytes, data: bytes) -> bytes:
            return (
                struct.pack(">I", len(data))
                + tag
                + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
            )

        png = b"\x89PNG\r\n\x1a\n"
        png += chunk(b"IHDR", struct.pack(">IIBBBBB", self.w, self.h, 8, 2, 0, 0, 0))
        png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
        png += chunk(b"IEND", b"")
        Path(path).write_bytes(png)


class MapView:
    """Проекция мир → пиксели (Y вверх, как на радаре)."""

    def __init__(self, cm: CoordinateMap, scale: float = 2.0, margin: float = 24.0):
        min_x, min_y, max_x, max_y = cm.bounds
        self.min_x, self.min_y = min_x - margin, min_y - margin
        self.max_x, self.max_y = max_x + margin, max_y + margin
        self.scale = scale
        self.w = int((self.max_x - self.min_x) * scale) + 1
        self.h = int((self.max_y - self.min_y) * scale) + 1

    def px(self, x: float, y: float) -> tuple[float, float]:
        return (x - self.min_x) * self.scale, (self.max_y - y) * self.scale


def _z_color(t: float, recorded: bool) -> Color:
    """Радарный стиль: пол — песочный, выше — светлее/холоднее."""
    t = max(0.0, min(1.0, t))
    base = (196, 172, 132)
    high = (236, 236, 222)
    col = tuple(int(base[i] + (high[i] - base[i]) * t) for i in range(3))
    if not recorded:
        col = tuple(max(0, c - 14) for c in col)
    return col  # type: ignore[return-value]


def draw_map(
    cm: CoordinateMap,
    canvas: Canvas,
    view: MapView,
    *,
    show_clearance: bool = False,
) -> None:
    z_min = min(n.z for n in cm.nodes)
    z_max = max(n.z for n in cm.nodes)
    span = max(1.0, z_max - z_min)
    cell = cm.cfg.cell
    max_cl = max(1.0, max(n.clearance for n in cm.nodes))
    for (cx, cy), layers in cm.cells.items():
        top = max(layers, key=lambda n: n.z)
        if show_clearance:
            t = top.clearance / max_cl
            color = (int(40 + 60 * t), int(60 + 160 * t), int(90 + 120 * (1 - t)))
        else:
            color = _z_color((top.z - z_min) / span, top.recorded)
        x0, y0 = view.px(cx * cell, (cy + 1) * cell)
        x1, y1 = view.px((cx + 1) * cell, cy * cell)
        canvas.fill_rect(int(x0), int(y0), int(x1) - 1, int(y1) - 1, color)
        if len(layers) > 1:
            # несколько ярусов (лестница над лестницей) — штриховка
            canvas.set(int((x0 + x1) / 2), int((y0 + y1) / 2), (120, 90, 200))

    # контур стен
    wall = (70, 62, 54)
    for (cx, cy), layers in cm.cells.items():
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if (cx + dx, cy + dy) not in cm.cells:
                if dx == 1:
                    x0, y0 = view.px((cx + 1) * cell, (cy + 1) * cell)
                    x1, y1 = view.px((cx + 1) * cell, cy * cell)
                elif dx == -1:
                    x0, y0 = view.px(cx * cell, (cy + 1) * cell)
                    x1, y1 = view.px(cx * cell, cy * cell)
                elif dy == 1:
                    x0, y0 = view.px(cx * cell, (cy + 1) * cell)
                    x1, y1 = view.px((cx + 1) * cell, (cy + 1) * cell)
                else:
                    x0, y0 = view.px(cx * cell, cy * cell)
                    x1, y1 = view.px((cx + 1) * cell, cy * cell)
                canvas.line(x0, y0, x1, y1, wall, 1.0)


def draw_polyline(
    canvas: Canvas,
    view: MapView,
    pts: list[tuple[float, float]],
    color: Color,
    width: float = 2.0,
) -> None:
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        x0, y0 = view.px(ax, ay)
        x1, y1 = view.px(bx, by)
        canvas.line(x0, y0, x1, y1, color, width)


def draw_heading(
    canvas: Canvas,
    view: MapView,
    x: float,
    y: float,
    yaw_deg: float,
    length: float,
    color: Color,
) -> None:
    ex = x + math.cos(math.radians(yaw_deg)) * length
    ey = y + math.sin(math.radians(yaw_deg)) * length
    x0, y0 = view.px(x, y)
    x1, y1 = view.px(ex, ey)
    canvas.line(x0, y0, x1, y1, color, 1.0)


def render_map_png(
    cm: CoordinateMap,
    out: Path | str,
    *,
    scale: float = 2.0,
    route: list[tuple[float, float, float]] | None = None,
    walked: list[tuple[float, float, float]] | None = None,
    headings: list[tuple[float, float, float]] | None = None,   # (x, y, yaw)
    goal: tuple[float, float] | None = None,
    show_clearance: bool = False,
    raw_points: list[tuple[float, float, float]] | None = None,
    targets: list[tuple[float, float, float]] | None = None,             # мишени (ноги)
    aim_lines: list[tuple[float, float, float, float]] | None = None,    # (x, y, tx, ty) — взгляд на мишень
    shot_lines: list[tuple[float, float, float, float, bool]] | None = None,  # (x, y, tx, ty, hit) — выстрелы
    target_radius: float = 16.0,
) -> Path:
    view = MapView(cm, scale=scale)
    canvas = Canvas(view.w, view.h)
    draw_map(cm, canvas, view, show_clearance=show_clearance)

    if raw_points:
        for x, y, _z in raw_points:
            px, py = view.px(x, y)
            canvas.set(int(px), int(py), (90, 70, 40))

    if aim_lines:
        for x, y, tx, ty in aim_lines:
            x0, y0 = view.px(x, y)
            x1, y1 = view.px(tx, ty)
            canvas.line(x0, y0, x1, y1, (255, 70, 70), 1.0)

    if shot_lines:
        for x, y, tx, ty, hit in shot_lines:
            x0, y0 = view.px(x, y)
            x1, y1 = view.px(tx, ty)
            col = (120, 255, 120) if hit else (255, 220, 60)
            canvas.line(x0, y0, x1, y1, col, 1.5)
            canvas.disc(x0, y0, 3.0, col)   # откуда стреляли

    if headings:
        for x, y, yaw in headings:
            draw_heading(canvas, view, x, y, yaw, 26.0, (250, 250, 250))

    if route and len(route) >= 2:
        draw_polyline(canvas, view, [(p[0], p[1]) for p in route], (52, 120, 246), 3.0)
        for p in route:
            px, py = view.px(p[0], p[1])
            canvas.ring(px, py, 3.0, (52, 120, 246), 1.0)

    if walked and len(walked) >= 2:
        draw_polyline(canvas, view, [(p[0], p[1]) for p in walked], (255, 150, 40), 2.0)
        sx, sy = view.px(walked[0][0], walked[0][1])
        canvas.disc(sx, sy, 6, (60, 220, 110))
        ex, ey = view.px(walked[-1][0], walked[-1][1])
        canvas.disc(ex, ey, 5, (255, 230, 90))

    if goal:
        gx, gy = view.px(goal[0], goal[1])
        canvas.ring(gx, gy, 9, (240, 70, 70), 2.0)
        canvas.disc(gx, gy, 3, (240, 70, 70))

    if targets:
        r = max(3.0, target_radius * view.scale)
        for tx, ty, _tz in targets:
            px, py = view.px(tx, ty)
            canvas.disc(px, py, r, (200, 40, 40))
            canvas.ring(px, py, r + 1.5, (255, 200, 200), 1.5)
            # крест-«мишень»
            canvas.line(px - r * 0.6, py, px + r * 0.6, py, (255, 230, 230), 1.0)
            canvas.line(px, py - r * 0.6, px, py + r * 0.6, (255, 230, 230), 1.0)

    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out)
    return out
