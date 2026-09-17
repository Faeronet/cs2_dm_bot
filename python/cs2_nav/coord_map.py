"""
Координатная карта локации из записи координат — 2.5D occupancy grid («радар»).

Алгоритм универсален: никаких координат конкретной локации, только физика игрока
(ширина hull 32u, высота ступеньки 18u, прыжок ~48u) и сама запись.

1. Запись = позиции центра игрока. Соседние точки trail соединяются отрезками,
   каждый отрезок растеризуется «капсулой» радиуса R (половина hull) → дилатация.
   Полосы «газонокосилки» (25–40u между проходами) при этом сливаются в сплошной пол,
   а ящики/стены (разрыв ≥ 2R+) остаются дырами.
2. Эрозия на тот же радиус (морфологическое закрытие) возвращает границу на место —
   карта = «где может находиться центр игрока».
3. Внутри клетки z кластеризуется в ярусы (винтовая лестница пересекает сама себя в XY).
4. Для каждой клетки-яруса считается clearance — расстояние до ближайшей стены/дыры.
   Маршрутизация использует его, чтобы не идти впритык к стенам.
"""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RECORDINGS = ROOT / "data" / "recordings"

# (dx, dy, horizontal length multiplier) для 8-связности
_NEIGH8 = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (-1, -1, math.sqrt(2.0)),
)


@dataclass
class MapConfig:
    cell: float = 8.0
    hull_radius: float = 16.0      # половина ширины игрока (hull 32×32)
    step_height: float = 18.0      # ступенька, которую можно пройти без прыжка
    jump_height: float = 48.0      # ступенька с прыжком
    drop_height: float = 120.0     # спрыгнуть вниз без урона
    layer_gap: float = 24.0        # разрыв по z → разные ярусы в одной клетке
    seg_max_len: float = 64.0      # разрыв trail (телепорт / смерть) — не соединять
    seg_max_dz: float = 48.0
    # Радиус закрытия. None → авто по записи: 0.6 × (85-й перцентиль зазора между проходами),
    # в пределах [fill_radius_min, fill_radius_max]. Закрытие заполняет дыры уже 2R.
    # Реальный ящик 64u в «центровом» пространстве = 64 + hull 32 = 96u → при R ≤ 30 не съедается;
    # пропущенная полоса записи (40–55u) при R ≥ 24 заполняется.
    fill_radius: float | None = None
    fill_radius_min: float = 24.0
    fill_radius_max: float = 30.0
    max_clearance: float = 128.0


@dataclass
class Node:
    """Клетка-ярус: одна проходимая позиция центра игрока (cx, cy, z)."""

    id: int
    cx: int
    cy: int
    z: float
    x: float
    y: float
    clearance: float = 0.0
    recorded: bool = False          # в клетке была настоящая записанная точка
    edges: list[tuple[int, float, str]] = field(default_factory=list)  # (nb_id, hdist, kind)

    def dist2d(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)


class CoordinateMap:
    def __init__(self, map_name: str, location: str, cfg: MapConfig | None = None):
        self.map_name = map_name
        self.location = location
        self.cfg = cfg or MapConfig()
        self.nodes: list[Node] = []
        self.cells: dict[tuple[int, int], list[Node]] = {}
        self.fill_radius: float = self.cfg.hull_radius
        self.lane_spacing: float | None = None
        self.n_points = 0
        self.crouch_points: list[tuple[float, float, float]] = []
        self.bounds = (0.0, 0.0, 0.0, 0.0)  # min_x, min_y, max_x, max_y (мировые)

    # ------------------------------------------------------------------ build

    @classmethod
    def from_recording(
        cls,
        path: Path | str | list[Path | str],
        cfg: MapConfig | None = None,
    ) -> CoordinateMap:
        """Одна запись или несколько (объединяются: точки — вместе, trail — с разрывом между файлами)."""
        paths = [Path(p) for p in (path if isinstance(path, (list, tuple)) else [path])]
        pts: list[tuple[float, float, float]] = []
        trail: list[tuple[float, float, float]] = []
        crouch_pts: list[tuple[float, float, float]] = []
        map_name, location = "unknown", paths[0].stem
        for i, p in enumerate(paths):
            raw = json.loads(p.read_text(encoding="utf-8"))
            if i == 0:
                map_name = str(raw.get("map", "unknown"))
                location = str(raw.get("location", p.stem))
            file_pts = [(float(q[0]), float(q[1]), float(q[2])) for q in raw.get("points", [])]
            crouch_block = raw.get("crouch") or {}
            for q in crouch_block.get("points") or []:
                if q is None:
                    continue
                crouch_pts.append((float(q[0]), float(q[1]), float(q[2])))
            # null в path — разрыв между склеенными записями (merge_recordings.py)
            gap = (float("nan"), float("nan"), float("nan"))
            file_trail = [
                gap if q is None else (float(q[0]), float(q[1]), float(q[2]))
                for q in (raw.get("path") or file_pts)
            ]
            pts.extend(file_pts)
            if trail and file_trail:
                # разрыв между файлами: точка далеко «в никуда», чтобы _segments не соединил
                trail.append((float("nan"), float("nan"), float("nan")))
            trail.extend(file_trail)
        if len(paths) > 1:
            location = f"{location}+{len(paths) - 1}"
        cm = cls.build(pts, trail, map_name=map_name, location=location, cfg=cfg)
        cm.crouch_points = _dedupe_xyz(crouch_pts)
        return cm

    @classmethod
    def build(
        cls,
        points: list[tuple[float, float, float]],
        trail: list[tuple[float, float, float]] | None,
        *,
        map_name: str = "unknown",
        location: str = "unknown",
        cfg: MapConfig | None = None,
    ) -> CoordinateMap:
        cm = cls(map_name, location, cfg)
        c = cm.cfg
        if not points:
            raise ValueError("пустая запись координат")
        cm.n_points = len(points)
        trail = trail or points

        segments = cm._segments(trail)
        cm.lane_spacing = cm._estimate_lane_spacing(trail)
        if c.fill_radius is not None:
            R = c.fill_radius
        else:
            R = c.fill_radius_min
            if cm.lane_spacing is not None:
                # полосы «газонокосилки» должны слиться: 2R > типичный зазор между проходами
                R = max(R, cm.lane_spacing * 0.6)
            R = min(R, c.fill_radius_max)
        cm.fill_radius = R

        dilated = cm._rasterize(segments, points, R)
        layers = {key: cm._cluster_z(zs) for key, zs in dilated.items()}
        closed = cm._erode(layers, R)

        # записанные точки всегда остаются на карте (якоря)
        recorded_keys: dict[tuple[int, int], list[float]] = {}
        for x, y, z in points:
            key = cm.cell_of(x, y)
            recorded_keys.setdefault(key, []).append(z)
            if not any(abs(lz - z) <= c.layer_gap for lz in closed.get(key, [])):
                closed.setdefault(key, []).append(z)

        cm._make_nodes(closed, recorded_keys)
        cm._link_edges()
        cm._compute_clearance()
        cm._compute_bounds()
        return cm

    def cell_of(self, x: float, y: float) -> tuple[int, int]:
        c = self.cfg.cell
        return int(math.floor(x / c)), int(math.floor(y / c))

    def cell_center(self, cx: int, cy: int) -> tuple[float, float]:
        c = self.cfg.cell
        return (cx + 0.5) * c, (cy + 0.5) * c

    def _segments(
        self, trail: list[tuple[float, float, float]]
    ) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
        c = self.cfg
        out = []
        for a, b in zip(trail, trail[1:]):
            if math.isnan(a[0]) or math.isnan(b[0]):
                continue
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            if d > c.seg_max_len or abs(b[2] - a[2]) > c.seg_max_dz:
                continue
            out.append((a, b))
        return out

    def _estimate_lane_spacing(self, trail: list[tuple[float, float, float]]) -> float | None:
        """
        Типичный зазор между проходами записи: 85-й перцентиль расстояния от точки
        до ближайшей точки *другого* прохода (медиана занижена пересечениями проходов).
        """
        n = len(trail)
        if n < 60:
            return None
        cell = max(self.cfg.cell * 2.0, 16.0)
        grid: dict[tuple[int, int], list[int]] = {}
        for i, (x, y, _z) in enumerate(trail):
            if math.isnan(x):
                continue
            grid.setdefault((int(x // cell), int(y // cell)), []).append(i)
        gaps: list[float] = []
        step = max(1, n // 400)
        for i in range(0, n, step):
            x, y, z = trail[i]
            if math.isnan(x):
                continue
            kx, ky = int(x // cell), int(y // cell)
            best = float("inf")
            for dx in range(-4, 5):
                for dy in range(-4, 5):
                    for j in grid.get((kx + dx, ky + dy), ()):
                        if abs(j - i) < 40:
                            continue
                        qx, qy, qz = trail[j]
                        if abs(qz - z) > self.cfg.layer_gap:
                            continue
                        d = math.hypot(qx - x, qy - y)
                        if d < best:
                            best = d
            if best < float("inf") and best > 2.0:
                gaps.append(best)
        if len(gaps) < 20:
            return None
        gaps.sort()
        return gaps[min(len(gaps) - 1, int(len(gaps) * 0.85))]

    def _rasterize(
        self,
        segments: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
        points: list[tuple[float, float, float]],
        R: float,
    ) -> dict[tuple[int, int], list[float]]:
        """Капсулы радиуса R вокруг отрезков trail (+ диски вокруг одиночных точек)."""
        cell = self.cfg.cell
        out: dict[tuple[int, int], list[float]] = {}
        r_cells = int(math.ceil(R / cell)) + 1
        R2 = R * R

        def paint(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> None:
            vx, vy = bx - ax, by - ay
            seg2 = vx * vx + vy * vy
            min_cx = int(math.floor(min(ax, bx) / cell)) - r_cells
            max_cx = int(math.floor(max(ax, bx) / cell)) + r_cells
            min_cy = int(math.floor(min(ay, by) / cell)) - r_cells
            max_cy = int(math.floor(max(ay, by) / cell)) + r_cells
            for cx in range(min_cx, max_cx + 1):
                px = (cx + 0.5) * cell
                for cy in range(min_cy, max_cy + 1):
                    py = (cy + 0.5) * cell
                    if seg2 > 1e-6:
                        t = ((px - ax) * vx + (py - ay) * vy) / seg2
                        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
                    else:
                        t = 0.0
                    qx, qy = ax + vx * t, ay + vy * t
                    dx, dy = px - qx, py - qy
                    if dx * dx + dy * dy <= R2:
                        out.setdefault((cx, cy), []).append(az + (bz - az) * t)

        for a, b in segments:
            paint(a[0], a[1], a[2], b[0], b[1], b[2])
        for x, y, z in points:
            paint(x, y, z, x, y, z)
        return out

    def _cluster_z(self, zs: list[float]) -> list[float]:
        zs = sorted(zs)
        gap = self.cfg.layer_gap
        layers: list[float] = []
        start = 0
        for i in range(1, len(zs) + 1):
            if i == len(zs) or zs[i] - zs[i - 1] > gap:
                chunk = zs[start:i]
                layers.append(chunk[len(chunk) // 2])
                start = i
        return layers

    def _erode(
        self, layers: dict[tuple[int, int], list[float]], R: float
    ) -> dict[tuple[int, int], list[float]]:
        """Клетка-ярус остаётся, если весь диск радиуса R вокруг неё покрыт дилатацией."""
        cell = self.cfg.cell
        gap = self.cfg.layer_gap
        r_cells = int(math.floor(R / cell + 1e-6))
        offsets = [
            (dx, dy, math.hypot(dx, dy) * cell)
            for dx in range(-r_cells, r_cells + 1)
            for dy in range(-r_cells, r_cells + 1)
            if math.hypot(dx, dy) * cell <= R + 1e-6
        ]
        out: dict[tuple[int, int], list[float]] = {}
        for (cx, cy), zs in layers.items():
            for z in zs:
                ok = True
                for dx, dy, d in offsets:
                    nb = layers.get((cx + dx, cy + dy))
                    if not nb:
                        ok = False
                        break
                    tol = gap + d  # лестница: допускаем уклон до ~45°
                    if not any(abs(nz - z) <= tol for nz in nb):
                        ok = False
                        break
                if ok:
                    out.setdefault((cx, cy), []).append(z)
        return out

    def _make_nodes(
        self,
        closed: dict[tuple[int, int], list[float]],
        recorded: dict[tuple[int, int], list[float]],
    ) -> None:
        self.nodes = []
        self.cells = {}
        gap = self.cfg.layer_gap
        for (cx, cy), zs in sorted(closed.items()):
            x, y = self.cell_center(cx, cy)
            rec_zs = recorded.get((cx, cy), [])
            for z in sorted(zs):
                node = Node(id=len(self.nodes), cx=cx, cy=cy, z=z, x=x, y=y)
                node.recorded = any(abs(rz - z) <= gap for rz in rec_zs)
                self.nodes.append(node)
                self.cells.setdefault((cx, cy), []).append(node)

    def _passable_layer(self, cx: int, cy: int, z: float, dz_max: float) -> Node | None:
        best: Node | None = None
        for n in self.cells.get((cx, cy), ()):
            dz = abs(n.z - z)
            if dz <= dz_max and (best is None or dz < abs(best.z - z)):
                best = n
        return best

    def _link_edges(self) -> None:
        c = self.cfg
        for node in self.nodes:
            node.edges = []
            for dx, dy, mult in _NEIGH8:
                if dx and dy:
                    # диагональ только если оба ортогональных соседа проходимы (не режем углы стен)
                    if (
                        self._passable_layer(node.cx + dx, node.cy, node.z, c.step_height) is None
                        or self._passable_layer(node.cx, node.cy + dy, node.z, c.step_height) is None
                    ):
                        continue
                hdist = c.cell * mult
                for nb in self.cells.get((node.cx + dx, node.cy + dy), ()):
                    dz = nb.z - node.z
                    if abs(dz) <= c.step_height:
                        kind = "walk"
                    elif 0.0 < dz <= c.jump_height:
                        kind = "jump"
                    elif -c.drop_height <= dz < 0.0:
                        kind = "drop"
                    else:
                        continue
                    node.edges.append((nb.id, hdist, kind))

    def _compute_clearance(self) -> None:
        """Brushfire: расстояние до ближайшей клетки, куда нельзя шагнуть с этого яруса."""
        c = self.cfg
        half = c.cell * 0.5
        dist = [float("inf")] * len(self.nodes)
        heap: list[tuple[float, int]] = []
        for node in self.nodes:
            walk_dirs = {(dx, dy) for dx, dy, _m in _NEIGH8
                         if self._passable_layer(node.cx + dx, node.cy + dy, node.z, c.step_height)}
            if len(walk_dirs) < 8:
                dist[node.id] = half
                heapq.heappush(heap, (half, node.id))
        while heap:
            d, nid = heapq.heappop(heap)
            if d > dist[nid]:
                continue
            if d >= c.max_clearance:
                continue
            for nb_id, hdist, kind in self.nodes[nid].edges:
                if kind != "walk":
                    continue
                nd = d + hdist
                if nd < dist[nb_id]:
                    dist[nb_id] = nd
                    heapq.heappush(heap, (nd, nb_id))
        for node in self.nodes:
            node.clearance = min(c.max_clearance, dist[node.id])

    def _compute_bounds(self) -> None:
        xs = [n.x for n in self.nodes]
        ys = [n.y for n in self.nodes]
        h = self.cfg.cell * 0.5
        self.bounds = (min(xs) - h, min(ys) - h, max(xs) + h, max(ys) + h)

    # ------------------------------------------------------------------ queries

    def layer_at(self, x: float, y: float, z: float, dz_max: float | None = None) -> Node | None:
        """Ярус в клетке (x, y), ближайший по z (в пределах dz_max)."""
        cx, cy = self.cell_of(x, y)
        return self._passable_layer(cx, cy, z, dz_max if dz_max is not None else self.cfg.drop_height)

    def is_walkable(self, x: float, y: float, z: float, dz_max: float | None = None) -> bool:
        return self.layer_at(x, y, z, dz_max if dz_max is not None else self.cfg.step_height) is not None

    def nearest_node(self, x: float, y: float, z: float, max_radius: float = 96.0) -> Node | None:
        """Ближайшая клетка-ярус (игрок может стоять чуть вне карты)."""
        node = self.layer_at(x, y, z)
        if node is not None:
            return node
        cx, cy = self.cell_of(x, y)
        r_cells = int(math.ceil(max_radius / self.cfg.cell))
        best: Node | None = None
        best_score = float("inf")
        for r in range(1, r_cells + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    for n in self.cells.get((cx + dx, cy + dy), ()):
                        dz = abs(n.z - z)
                        if dz > self.cfg.drop_height:
                            continue
                        score = n.dist2d(x, y) + dz * 0.5
                        if score < best_score:
                            best_score, best = score, n
            if best is not None:
                return best
        return None

    def clearance_at(self, x: float, y: float, z: float) -> float:
        n = self.nearest_node(x, y, z, max_radius=32.0)
        return n.clearance if n else 0.0

    def near_crouch(
        self,
        x: float,
        y: float,
        z: float,
        *,
        radius_xy: float = 24.0,
        dz_max: float = 32.0,
    ) -> bool:
        """True, если рядом есть записанная точка прохода с приседанием."""
        r2 = radius_xy * radius_xy
        for cx, cy, cz in self.crouch_points:
            if abs(cz - z) > dz_max:
                continue
            dx, dy = cx - x, cy - y
            if dx * dx + dy * dy <= r2:
                return True
        return False

    def stats(self) -> dict:
        cl = sorted(n.clearance for n in self.nodes)
        return {
            "points": self.n_points,
            "crouch_points": len(self.crouch_points),
            "nodes": len(self.nodes),
            "cells": len(self.cells),
            "multi_layer_cells": sum(1 for v in self.cells.values() if len(v) > 1),
            "fill_radius": round(self.fill_radius, 1),
            "lane_spacing": None if self.lane_spacing is None else round(self.lane_spacing, 1),
            "clearance_median": round(cl[len(cl) // 2], 1) if cl else 0.0,
            "clearance_max": round(cl[-1], 1) if cl else 0.0,
            "jump_edges": sum(1 for n in self.nodes for e in n.edges if e[2] == "jump") // 2,
            "z_min": round(min(n.z for n in self.nodes), 1),
            "z_max": round(max(n.z for n in self.nodes), 1),
        }

    @classmethod
    def find_recording(cls, map_name: str, location: str | None = None) -> Path:
        base = RECORDINGS / map_name
        if location:
            p = base / f"{location}.json"
            if p.exists():
                return p
        if base.is_dir():
            hits = sorted(base.glob("*.json"))
            if hits:
                return hits[0]
        raise FileNotFoundError(f"нет записи для {map_name}/{location or '*'}")


def _dedupe_xyz(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    seen: set[tuple[int, int, int]] = set()
    out: list[tuple[float, float, float]] = []
    for x, y, z in points:
        key = (round(x), round(y), round(z))
        if key in seen:
            continue
        seen.add(key)
        out.append((x, y, z))
    return out
