"""Маршрут как ломаная: проекция позиции, точка «впереди на L юнитов», уклон."""

from __future__ import annotations

import math

XYZ = tuple[float, float, float]


class Route:
    def __init__(self, points: list[XYZ]):
        if len(points) < 2:
            points = list(points) + [points[0]] if points else [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
        self.pts: list[XYZ] = list(points)
        self.cum: list[float] = [0.0]
        for a, b in zip(self.pts, self.pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.cum[-1]

    @property
    def goal(self) -> XYZ:
        return self.pts[-1]

    def _seg_project(self, i: int, x: float, y: float) -> tuple[float, float]:
        """(t ∈ [0,1], расстояние до отрезка i)."""
        ax, ay, _ = self.pts[i]
        bx, by, _ = self.pts[i + 1]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-9:
            return 0.0, math.hypot(x - ax, y - ay)
        t = ((x - ax) * dx + (y - ay) * dy) / seg2
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        px, py = ax + dx * t, ay + dy * t
        return t, math.hypot(x - px, y - py)

    def project(self, x: float, y: float, s_hint: float | None = None, back_window: float = 60.0) -> tuple[float, float]:
        """
        Продольная координата s ближайшей точки маршрута и расстояние до неё.
        С s_hint ищем только вперёд (минус небольшое окно назад) — маршрут может
        пересекать сам себя (винтовая лестница), прогресс должен быть монотонным.
        """
        s_min = -1.0 if s_hint is None else s_hint - back_window
        best_s, best_d = 0.0, float("inf")
        for i in range(len(self.pts) - 1):
            if self.cum[i + 1] < s_min:
                continue
            t, d = self._seg_project(i, x, y)
            s = self.cum[i] + (self.cum[i + 1] - self.cum[i]) * t
            if s < s_min:
                continue
            # при равных расстояниях предпочитаем более ранний участок (не перескакивать)
            if d < best_d - 0.5:
                best_d, best_s = d, s
        if best_d == float("inf"):
            return self.length, math.hypot(x - self.pts[-1][0], y - self.pts[-1][1])
        return best_s, best_d

    def point_at(self, s: float) -> XYZ:
        if s <= 0.0:
            return self.pts[0]
        if s >= self.length:
            return self.pts[-1]
        # бинарный поиск отрезка
        lo, hi = 0, len(self.cum) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.cum[mid] <= s:
                lo = mid
            else:
                hi = mid
        seg = self.cum[hi] - self.cum[lo]
        t = 0.0 if seg < 1e-9 else (s - self.cum[lo]) / seg
        a, b = self.pts[lo], self.pts[hi]
        return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)

    def next_vertex_index(self, s: float) -> int:
        for i, c in enumerate(self.cum):
            if c > s + 1e-6:
                return i
        return len(self.pts) - 1

    def remaining(self, s: float) -> float:
        return max(0.0, self.length - s)

    def vertical_span_ahead(
        self, s: float, ahead: float, min_dz: float
    ) -> tuple[XYZ, XYZ, bool, int] | None:
        """Ближайший отрезок маршрута с большим Δz (лестница / ярус)."""
        i = self.next_vertex_index(s)
        k = max(0, i - 1)
        end_s = s + ahead
        while k + 1 < len(self.pts) and self.cum[k] <= end_s + 12.0:
            a, b = self.pts[k], self.pts[k + 1]
            if abs(b[2] - a[2]) >= min_dz:
                return a, b, b[2] > a[2], k
            k += 1
        return None

    def slope_ahead(self, s: float, dist: float) -> float:
        """Уклон (градусы, + = вверх) на участке [s, s+dist]."""
        a = self.point_at(s)
        b = self.point_at(min(self.length, s + dist))
        h = math.hypot(b[0] - a[0], b[1] - a[1])
        if h < 1.0:
            return 0.0
        return math.degrees(math.atan2(b[2] - a[2], h))

    def turn_ahead(self, s: float, dist: float) -> float:
        """Суммарный поворот маршрута (градусы) на ближайших dist юнитах — для замедления камеры."""
        i = self.next_vertex_index(s)
        total = 0.0
        while i + 1 < len(self.pts) and self.cum[i] <= s + dist:
            ax, ay, _ = self.pts[i - 1] if i >= 1 else self.point_at(s)
            bx, by, _ = self.pts[i]
            cx, cy, _ = self.pts[i + 1]
            h1 = math.atan2(by - ay, bx - ax)
            h2 = math.atan2(cy - by, cx - bx)
            d = math.degrees(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
            total += abs(d)
            i += 1
        return total
