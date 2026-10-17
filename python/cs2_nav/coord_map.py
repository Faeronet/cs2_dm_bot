"""
Координатная карта локации из записи координат — 2.5D occupancy grid («радар»).

Алгоритм универсален: никаких координат конкретной локации, только физика игрока
(ширина hull 32u, ступенька 18u, прыжок 57u, прыжок с приседом 64u) и сама запись.

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
MAPS = ROOT / "data" / "maps"

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
    step_height: float = 18.0      # sv_stepsize: ступенька без прыжка
    # CS2: sv_gravity=800, sv_jump_impulse=301.993 → h = v²/(2g) ≈ 57u.
    # С приседом в прыжке (hull 54 вместо 72) забираемся на ящик 64u.
    jump_height: float = 57.0
    crouch_jump_height: float = 64.0
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
        self._crouch_grid: dict[tuple[int, int], list[tuple[float, float, float]]] = {}
        self.ladder_links: list[dict] = []
        self._ladder_keys: set[tuple[int, int, int, int, int, int]] = set()
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
            file_crouch: list[tuple[float, float, float]] = []
            for q in crouch_block.get("points") or []:
                if q is None:
                    continue
                file_crouch.append((float(q[0]), float(q[1]), float(q[2])))
            crouch_pts.extend(file_crouch)
            # null в path — разрыв между склеенными записями (merge_recordings.py)
            gap = (float("nan"), float("nan"), float("nan"))
            file_trail = [
                gap if q is None else (float(q[0]), float(q[1]), float(q[2]))
                for q in (raw.get("path") or file_pts)
            ]
            crouch_trail = [
                gap if q is None else (float(q[0]), float(q[1]), float(q[2]))
                for q in (crouch_block.get("path") or file_crouch)
            ]
            pts.extend(file_pts)
            pts.extend(file_crouch)
            if trail and file_trail:
                trail.append(gap)
            trail.extend(file_trail)
            if crouch_trail:
                if trail:
                    trail.append(gap)
                trail.extend(crouch_trail)
        if len(paths) > 1:
            location = f"{location}+{len(paths) - 1}"
        cm = cls.build(pts, trail, map_name=map_name, location=location, cfg=cfg)
        cm.crouch_points = _dedupe_xyz(crouch_pts)
        cm._index_crouch()
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

    def _through_floor(self, node: Node, nb: Node) -> bool:
        """Стек этажей (Вертиго, паласы): нельзя упасть на нижний ярус сквозь пол.

        Если у соседа есть продолжение нашего яруса — идём по нему, а не «сквозь плиту».
        Спрыгнуть с ящика на пол соседа можно: у соседа нашего z нет.
        """
        return self._passable_layer(nb.cx, nb.cy, node.z, self.cfg.step_height) is not None

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
                    elif c.jump_height < dz <= c.crouch_jump_height:
                        kind = "crouch_jump"
                    elif -c.drop_height <= dz < 0.0:
                        if self._through_floor(node, nb):
                            continue
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

    def higher_layer(self, x: float, y: float, z: float, dz_max: float | None = None) -> Node | None:
        """Ближайший ярус СТРОГО выше z в этой клетке (ящик / ступенька)."""
        cap = self.cfg.crouch_jump_height if dz_max is None else dz_max
        cx, cy = self.cell_of(x, y)
        best: Node | None = None
        for n in self.cells.get((cx, cy), ()):
            dz = n.z - z
            if 2.0 < dz <= cap and (best is None or n.z < best.z):
                best = n
        return best

    def is_walkable(self, x: float, y: float, z: float, dz_max: float | None = None) -> bool:
        return self.layer_at(x, y, z, dz_max if dz_max is not None else self.cfg.step_height) is not None

    def nearest_node(self, x: float, y: float, z: float, max_radius: float = 96.0) -> Node | None:
        """Ближайшая клетка-ярус (игрок может стоять чуть вне карты)."""
        node = self.layer_at(x, y, z, dz_max=self.cfg.jump_height)
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
                        score = n.dist2d(x, y) + dz * 1.6
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
        radius_xy: float = 28.0,
        dz_max: float = 40.0,
    ) -> bool:
        """True, если рядом есть записанная точка прохода с приседанием."""
        cell = self.cfg.cell
        r2 = radius_xy * radius_xy
        cx0, cy0 = int(math.floor(x / cell)), int(math.floor(y / cell))
        reach = max(1, int(math.ceil(radius_xy / cell)))
        grid = self._crouch_grid
        if not grid:
            pts = self.crouch_points
            for px, py, pz in pts:
                if abs(pz - z) > dz_max:
                    continue
                dx, dy = px - x, py - y
                if dx * dx + dy * dy <= r2:
                    return True
            return False
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                for px, py, pz in grid.get((cx0 + dx, cy0 + dy), ()):
                    if abs(pz - z) > dz_max:
                        continue
                    if (px - x) * (px - x) + (py - y) * (py - y) <= r2:
                        return True
        return False

    def _index_crouch(self) -> None:
        grid: dict[tuple[int, int], list[tuple[float, float, float]]] = {}
        cell = self.cfg.cell
        for p in self.crouch_points:
            key = (int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell)))
            grid.setdefault(key, []).append(p)
        self._crouch_grid = grid

    def attach_ladders(self, ladders: list) -> int:
        """Связать ярусы карты рёбрами ladder (дно↔верх). Повторный вызов идемпотентен."""
        added = 0
        for raw in ladders or []:
            lad = _ladder_as_dict(raw)
            if not lad or lad.get("disabled") or lad.get("fake"):
                continue
            bot = lad["bottom"]
            top = lad["top"]
            if top[2] < bot[2]:
                bot, top = top, bot
            key = (
                round(bot[0]), round(bot[1]), round(bot[2]),
                round(top[0]), round(top[1]), round(top[2]),
            )
            if key in self._ladder_keys:
                continue
            n_bot = self._node_for_mount(bot[0], bot[1], bot[2])
            n_top = self._node_for_mount(top[0], top[1], top[2])
            if n_bot is None or n_top is None or n_bot.id == n_top.id:
                continue
            if abs(n_top.z - n_bot.z) < self.cfg.crouch_jump_height + 4.0:
                continue
            hdist = math.hypot(n_top.x - n_bot.x, n_top.y - n_bot.y) + abs(n_top.z - n_bot.z)
            if not any(e[0] == n_top.id and e[2] == "ladder" for e in n_bot.edges):
                n_bot.edges.append((n_top.id, hdist, "ladder"))
            if not any(e[0] == n_bot.id and e[2] == "ladder" for e in n_top.edges):
                n_top.edges.append((n_bot.id, hdist, "ladder"))
            self._ladder_keys.add(key)
            self.ladder_links.append(
                {
                    "bottom_id": n_bot.id,
                    "top_id": n_top.id,
                    "bottom": bot,
                    "top": top,
                    "ladder": lad,
                }
            )
            added += 1
        return added

    def _node_for_mount(self, x: float, y: float, z: float):
        n = self.nearest_node(x, y, z, max_radius=72.0)
        if n is not None and n.dist2d(x, y) <= 56.0 and abs(n.z - z) <= 48.0:
            return n
        return self._insert_mount_node(x, y, z)

    def _insert_mount_node(self, x: float, y: float, z: float):
        cx, cy = self.cell_of(x, y)
        node = Node(id=len(self.nodes), cx=cx, cy=cy, z=z, x=x, y=y, clearance=18.0, recorded=True)
        self.nodes.append(node)
        self.cells.setdefault((cx, cy), []).append(node)
        c = self.cfg
        for dx, dy, mult in _NEIGH8:
            for nb in self.cells.get((cx + dx, cy + dy), ()):
                if nb.id == node.id:
                    continue
                if abs(nb.z - z) > c.step_height:
                    continue
                hdist = c.cell * mult
                node.edges.append((nb.id, hdist, "walk"))
                nb.edges.append((node.id, hdist, "walk"))
        return node

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
            "crouch_jump_edges": sum(1 for n in self.nodes for e in n.edges if e[2] == "crouch_jump") // 2,
            "ladder_edges": sum(1 for n in self.nodes for e in n.edges if e[2] == "ladder") // 2,
            "z_min": round(min(n.z for n in self.nodes), 1),
            "z_max": round(max(n.z for n in self.nodes), 1),
        }

    @classmethod
    def from_navmesh(cls, path: Path | str, cfg: MapConfig | None = None) -> CoordinateMap:
        """
        Принять разобранный .nav (полигоны Valve) как запись для текущего сборщика.

        Каждый полигон режется на короткие отрезки короче seg_max_len / seg_max_dz,
        иначе длинное ребро навмеша текущий код считает телепортом и выбрасывает.
        Дальше работает обычный build: капсула, эрозия, ярусы, рёбра по соседним клеткам.
        """
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        cfg = cfg or MapConfig()
        step = max(cfg.cell, 16.0)
        max_len = min(56.0, cfg.seg_max_len - 1.0)
        max_dz = min(40.0, cfg.seg_max_dz - 1.0)
        gap = (float("nan"), float("nan"), float("nan"))
        points: list[tuple[float, float, float]] = []
        trail: list[tuple[float, float, float]] = []
        crouch: list[tuple[float, float, float]] = []
        for area in raw.get("areas") or []:
            corners = [
                (float(p[0]), float(p[1]), float(p[2]))
                for p in (area.get("corners") or [])
                if p is not None and len(p) >= 3
            ]
            if len(corners) < 3:
                continue
            is_crouch = "CrouchHeight" in (area.get("flags") or [])
            samples: list[tuple[float, float, float]] = []
            for a, b in _polygon_segments(corners, step):
                samples.extend(_split_segment(a, b, max_len, max_dz))
            if not samples:
                samples.extend(corners)
            for a, b in zip(samples[0::2], samples[1::2]):
                points.append(a)
                points.append(b)
                trail.append(a)
                trail.append(b)
                trail.append(gap)
                if is_crouch:
                    crouch.append(a)
                    crouch.append(b)
        if not points:
            raise ValueError("в nav-файле нет полигонов")
        source = str(raw.get("source") or path)
        map_name = Path(source).name
        for suffix in (".vpk", ".nav.json", ".json", ".nav"):
            if map_name.endswith(suffix):
                map_name = map_name[: -len(suffix)]
                break
        cm = cls.build(
            points,
            trail,
            map_name=map_name or "nav",
            location="navmesh",
            cfg=cfg,
        )
        cm.crouch_points = _dedupe_xyz(crouch)
        cm._index_crouch()
        cm.attach_ladders(raw.get("ladder_list") or [])
        return cm

    @classmethod
    def find_recording(cls, map_name: str, location: str | None = None) -> Path:
        base = MAPS
        if location:
            p = base / f"{map_name}_{location}.json"
            if p.exists():
                return p
        if base.is_dir():
            hits = sorted(base.glob(f"{map_name}_*.json"))
            hits = [h for h in hits if not h.name.endswith(".nav.json")]
            if hits:
                return hits[0]
        raise FileNotFoundError(f"нет записи для {map_name}/{location or '*'}")


def _split_segment(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    max_len: float,
    max_dz: float,
) -> list[tuple[float, float, float]]:
    """Пары точек (a,b,a,b,...) короче порогов текущего сборщика."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    dist = math.hypot(dx, dy)
    pieces = max(1, math.ceil(dist / max_len), math.ceil(abs(dz) / max_dz))
    out: list[tuple[float, float, float]] = []
    prev = a
    for i in range(1, pieces + 1):
        t = i / pieces
        cur = (a[0] + dx * t, a[1] + dy * t, a[2] + dz * t)
        out.append(prev)
        out.append(cur)
        prev = cur
    return out


def _polygon_segments(
    corners: list[tuple[float, float, float]],
    step: float,
) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """Контур полигона и горизонтальные хорды внутри, шаг step."""
    segs: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    n = len(corners)
    for i in range(n):
        segs.append((corners[i], corners[(i + 1) % n]))
    ys = [c[1] for c in corners]
    y = min(ys) + step * 0.5
    y1 = max(ys)
    while y < y1:
        hits: list[tuple[float, float]] = []
        for i in range(n):
            a = corners[i]
            b = corners[(i + 1) % n]
            ya, yb = a[1], b[1]
            if abs(yb - ya) < 1e-6:
                continue
            if (ya <= y < yb) or (yb <= y < ya):
                t = (y - ya) / (yb - ya)
                hits.append((a[0] + (b[0] - a[0]) * t, a[2] + (b[2] - a[2]) * t))
        hits.sort()
        for i in range(0, len(hits) - 1, 2):
            x0, z0 = hits[i]
            x1, z1 = hits[i + 1]
            if abs(x1 - x0) < 1.0:
                continue
            segs.append(((x0, y, z0), (x1, y, z1)))
        y += step
    return segs


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


def _xyz3(v) -> tuple[float, float, float] | None:
    if v is None:
        return None
    if isinstance(v, dict):
        return float(v.get("x", 0.0)), float(v.get("y", 0.0)), float(v.get("z", 0.0))
    if hasattr(v, "x"):
        return float(v.x), float(v.y), float(v.z)
    if isinstance(v, (tuple, list)) and len(v) >= 3:
        return float(v[0]), float(v[1]), float(v[2])
    return None


def _ladder_as_dict(raw) -> dict | None:
    if raw is None:
        return None
    if hasattr(raw, "to_dict"):
        d = raw.to_dict()
    elif isinstance(raw, dict):
        d = dict(raw)
    else:
        return None
    bot = _xyz3(d.get("bottom"))
    top = _xyz3(d.get("top"))
    if bot is None or top is None:
        return None
    d["bottom"] = bot
    d["top"] = top
    ctr = _xyz3(d.get("center"))
    if ctr is not None:
        d["center"] = ctr
    mins = _xyz3(d.get("bounds_mins"))
    maxs = _xyz3(d.get("bounds_maxs"))
    if mins is not None:
        d["bounds_mins"] = mins
    if maxs is not None:
        d["bounds_maxs"] = maxs
    return d
