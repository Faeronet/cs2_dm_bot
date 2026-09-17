"""
Построение маршрута по координатной карте («как в Яндекс.Картах»).

* farthest_node   — самая дальняя достижимая точка (по длине пути, а не по прямой).
* astar           — кратчайший путь по клеткам с штрафом за близость к стенам:
                    в широком коридоре идём по середине, в узком — как получится.
* smooth_path     — «натягивание нити»: убираем лишние вершины, но каждый прямой
                    отрезок обязан проходить по клеткам с достаточным clearance.
"""

from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass
from typing import Callable

from cs2_nav.coord_map import CoordinateMap, Node

XYZ = tuple[float, float, float]


@dataclass
class RouteConfig:
    preferred_clearance: float = 40.0   # хотим быть не ближе этого к стене (2.5 hull)
    wall_weight: float = 3.0            # сила штрафа за близость к стене
    jump_penalty: float = 80.0
    drop_penalty: float = 24.0
    ladder_penalty: float = 36.0            # лестница дешевле прыжка, дороже пола
    climb_weight: float = 0.6           # за каждый юнит подъёма
    los_min_clearance: float = 12.0     # минимум clearance на прямом отрезке (0.75 hull)
    los_sample: float = 4.0             # шаг проверки отрезка (юниты)
    goal_min_clearance: float = 12.0    # цель — не в углу впритык к стене


def _edge_cost(cm: CoordinateMap, a: Node, b: Node, hdist: float, kind: str, cfg: RouteConfig) -> float:
    cost = hdist
    if kind != "ladder":
        c = min(a.clearance, b.clearance)
        if c < cfg.preferred_clearance:
            t = (cfg.preferred_clearance - c) / cfg.preferred_clearance
            cost *= 1.0 + cfg.wall_weight * t * t
    dz = b.z - a.z
    if dz > 0:
        cost += dz * cfg.climb_weight
    if kind == "jump":
        cost += cfg.jump_penalty
    elif kind == "drop":
        cost += cfg.drop_penalty
    elif kind == "ladder":
        cost += cfg.ladder_penalty + abs(dz) * 0.35
    return cost


Obstacle = tuple[float, float, float, float]   # x, y, z, radius — динамическое препятствие (мишень, игрок)


def blocked_by_obstacles(cm: CoordinateMap, obstacles: list[Obstacle], z_tol: float = 72.0) -> set[int]:
    """Клетки, через которые не пройти из-за препятствий: центр клетки ближе radius+hull к препятствию."""
    blocked: set[int] = set()
    if not obstacles:
        return blocked
    hull = cm.cfg.hull_radius
    cell = cm.cfg.cell
    for ox, oy, oz, r in obstacles:
        # касание тел + одна клетка запаса: маршрут по краю «щели» на практике не проходится
        rr = r + hull + cell
        cx0, cy0 = cm.cell_of(ox - rr, oy - rr)
        cx1, cy1 = cm.cell_of(ox + rr, oy + rr)
        for cx in range(cx0, cx1 + 1):
            for cy in range(cy0, cy1 + 1):
                layers = cm.cells.get((cx, cy))
                if not layers:
                    continue
                x, y = cm.cell_center(cx, cy)
                if math.hypot(x - ox, y - oy) > rr:
                    continue
                for n in layers:
                    if abs(n.z - oz) <= z_tol:
                        blocked.add(n.id)
    return blocked


def dijkstra(
    cm: CoordinateMap,
    start_id: int,
    cfg: RouteConfig | None = None,
    *,
    geometric: bool = False,
    goal_id: int | None = None,
    blocked: set[int] | None = None,
) -> tuple[list[float], list[int]]:
    """Расстояния и предки от start. geometric=True — чистая длина пути без штрафов."""
    cfg = cfg or RouteConfig()
    blocked = blocked or set()
    n = len(cm.nodes)
    dist = [float("inf")] * n
    came = [-1] * n
    dist[start_id] = 0.0
    heap = [(0.0, start_id)]
    while heap:
        d, cur = heapq.heappop(heap)
        if d > dist[cur]:
            continue
        if goal_id is not None and cur == goal_id:
            break
        a = cm.nodes[cur]
        for nb_id, hdist, kind in a.edges:
            if nb_id in blocked:
                continue
            b = cm.nodes[nb_id]
            if geometric:
                step = hdist + abs(b.z - a.z) * 0.5
                if kind == "jump":
                    step += cfg.jump_penalty
                elif kind == "ladder":
                    step += abs(b.z - a.z) * 0.2
            else:
                step = _edge_cost(cm, a, b, hdist, kind, cfg)
            nd = d + step
            if nd < dist[nb_id]:
                dist[nb_id] = nd
                came[nb_id] = cur
                heapq.heappush(heap, (nd, nb_id))
    return dist, came


def farthest_node(
    cm: CoordinateMap, start_id: int, cfg: RouteConfig | None = None, blocked: set[int] | None = None
) -> int:
    """Самая дальняя достижимая клетка по длине пути (не через стены), не впритык к стене."""
    cfg = cfg or RouteConfig()
    blocked = blocked or set()
    dist, _came = dijkstra(cm, start_id, cfg, geometric=True, blocked=blocked)
    max_cl = max(n.clearance for n in cm.nodes)
    min_cl = min(cfg.goal_min_clearance, max_cl * 0.5)
    best, best_d = start_id, -1.0
    for n in cm.nodes:
        d = dist[n.id]
        if d == float("inf") or n.clearance < min_cl or n.id in blocked:
            continue
        # среди почти одинаково дальних — с большим clearance
        score = d + n.clearance * 0.5
        if score > best_d:
            best_d, best = score, n.id
    return best


def reconstruct(came: list[int], goal_id: int) -> list[int]:
    path = [goal_id]
    cur = goal_id
    while came[cur] != -1:
        cur = came[cur]
        path.append(cur)
    path.reverse()
    return path


def astar(
    cm: CoordinateMap, start_id: int, goal_id: int, cfg: RouteConfig | None = None, blocked: set[int] | None = None
) -> list[int]:
    cfg = cfg or RouteConfig()
    blocked = blocked or set()
    if start_id == goal_id:
        return [start_id]
    if goal_id in blocked:
        return []
    goal = cm.nodes[goal_id]
    n = len(cm.nodes)
    g = [float("inf")] * n
    came = [-1] * n
    g[start_id] = 0.0

    def h(node: Node) -> float:
        return node.dist2d(goal.x, goal.y)

    heap = [(h(cm.nodes[start_id]), start_id)]
    closed = [False] * n
    while heap:
        _f, cur = heapq.heappop(heap)
        if closed[cur]:
            continue
        if cur == goal_id:
            return reconstruct(came, goal_id)
        closed[cur] = True
        a = cm.nodes[cur]
        for nb_id, hdist, kind in a.edges:
            if closed[nb_id] or nb_id in blocked:
                continue
            b = cm.nodes[nb_id]
            ng = g[cur] + _edge_cost(cm, a, b, hdist, kind, cfg)
            if ng < g[nb_id]:
                g[nb_id] = ng
                came[nb_id] = cur
                heapq.heappush(heap, (ng + h(b), nb_id))
    return []


def segment_clear(
    cm: CoordinateMap,
    a: XYZ,
    b: XYZ,
    min_clearance: float | Callable[[float], float],
    cfg: RouteConfig | None = None,
    blocked: set[int] | None = None,
) -> bool:
    """
    Прямой отрезок a→b целиком идёт по проходимым клеткам без прыжков,
    и clearance в каждой точке ≥ min_clearance (число или функция от t∈[0,1]).
    """
    cfg = cfg or RouteConfig()
    blocked = blocked or set()
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    steps = max(1, int(math.ceil(length / cfg.los_sample)))
    z = a[2]
    step_h = cm.cfg.step_height
    need_fn = min_clearance if callable(min_clearance) else (lambda _t, _c=min_clearance: _c)
    for i in range(1, steps + 1):
        t = i / steps
        x, y = a[0] + dx * t, a[1] + dy * t
        node = cm.layer_at(x, y, z, dz_max=step_h + 2.0)
        if node is None or node.clearance < need_fn(t) or node.id in blocked:
            return False
        z = node.z
    return abs(z - b[2]) <= step_h + 2.0


def smooth_path(
    cm: CoordinateMap, node_ids: list[int], cfg: RouteConfig | None = None, blocked: set[int] | None = None
) -> list[XYZ]:
    """
    Натягивание нити с учётом clearance. Отрезок i→j допустим, если в каждой его точке
    clearance ≥ min(preferred·0.75, clearance пути A* в этом же месте − ε).
    То есть срезать можно, но не ближе к стене, чем идёт центрированный A* именно здесь:
    в широком коридоре получаем прямые линии, в узком повороте — идём по центру.
    """
    cfg = cfg or RouteConfig()
    pts: list[XYZ] = [(cm.nodes[i].x, cm.nodes[i].y, cm.nodes[i].z) for i in node_ids]
    if len(pts) <= 2:
        return pts
    clear = [cm.nodes[i].clearance for i in node_ids]
    cap = cfg.preferred_clearance * 0.75
    eps = cm.cfg.cell * 0.75
    out: list[XYZ] = [pts[0]]
    i = 0
    n = len(pts)
    while i < n - 1:
        best_j = i + 1
        j = i + 1
        while j < n:
            # прыжок / лестница / большой спуск между соседями нельзя срезать
            if abs(pts[j][2] - pts[j - 1][2]) > cm.cfg.step_height:
                break
            span = j - i

            def need(t: float, _i: int = i, _span: int = span) -> float:
                k = _i + int(round(t * _span))
                k = max(_i, min(_i + _span, k))
                local = min(clear[max(_i, k - 1) : min(n, k + 2)])
                return max(cfg.los_min_clearance, min(cap, local - eps))

            if segment_clear(cm, pts[i], pts[j], need, cfg, blocked):
                best_j = j
                j += 1
            else:
                break
        out.append(pts[best_j])
        i = best_j
    return _drop_collinear(out)


def _drop_collinear(pts: list[XYZ], max_dev: float = 3.0) -> list[XYZ]:
    """Убрать вершины, лежащие почти на прямой между соседями (шум сетки)."""
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    for k in range(1, len(pts) - 1):
        a, b, c = out[-1], pts[k], pts[k + 1]
        ax, ay = c[0] - a[0], c[1] - a[1]
        seg = math.hypot(ax, ay)
        if seg < 1e-6:
            continue
        dev = abs((b[0] - a[0]) * ay - (b[1] - a[1]) * ax) / seg
        if dev > max_dev or abs(b[2] - a[2]) > 4.0:
            out.append(b)
    out.append(pts[-1])
    return out


def plan_route(
    cm: CoordinateMap,
    start: XYZ,
    goal_id: int,
    cfg: RouteConfig | None = None,
    blocked: set[int] | None = None,
) -> list[XYZ]:
    """Полный маршрут от произвольной позиции до клетки goal_id (список точек x, y, z)."""
    cfg = cfg or RouteConfig()
    blocked = blocked or set()
    start_node = cm.nearest_node(start[0], start[1], start[2])
    if start_node is None:
        raise RuntimeError(f"позиция {start} вне карты")
    if start_node.id in blocked:
        # стоим вплотную к препятствию — выходим с ближайшей свободной клетки
        free = _nearest_free(cm, start_node, blocked)
        if free is None:
            raise RuntimeError("вокруг нет свободных клеток")
        start_node = free
    ids = astar(cm, start_node.id, goal_id, cfg, blocked)
    if not ids:
        raise RuntimeError("маршрут не найден (цель недостижима)")
    pts = smooth_path(cm, ids, cfg, blocked)
    # начинаем строго из текущей позиции
    if math.hypot(pts[0][0] - start[0], pts[0][1] - start[1]) > 1.0:
        pts[0] = (start[0], start[1], start[2])
    return pts


def _nearest_free(cm: CoordinateMap, node: Node, blocked: set[int]) -> Node | None:
    """BFS по рёбрам до первой незаблокированной клетки."""
    seen = {node.id}
    frontier = [node]
    for _ in range(12):
        nxt: list[Node] = []
        for a in frontier:
            for nb_id, _h, _k in a.edges:
                if nb_id in seen:
                    continue
                seen.add(nb_id)
                b = cm.nodes[nb_id]
                if nb_id not in blocked:
                    return b
                nxt.append(b)
        frontier = nxt
        if not frontier:
            break
    return None


def pick_farthest_goal(
    cm: CoordinateMap, start: XYZ, cfg: RouteConfig | None = None, blocked: set[int] | None = None
) -> int:
    node = cm.nearest_node(start[0], start[1], start[2])
    if node is None:
        raise RuntimeError(f"позиция {start} вне карты")
    blocked = blocked or set()
    if node.id in blocked:
        free = _nearest_free(cm, node, blocked)
        if free is not None:
            node = free
    return farthest_node(cm, node.id, cfg, blocked)


def pick_random_goal(
    cm: CoordinateMap,
    start: XYZ,
    cfg: RouteConfig | None = None,
    blocked: set[int] | None = None,
    rng: random.Random | None = None,
    exclude: set[int] | None = None,
    min_dist: float = 320.0,
) -> int:
    """Случайная достижимая клетка (не в углу, не вплотную к старту)."""
    cfg = cfg or RouteConfig()
    blocked = blocked or set()
    exclude = exclude or set()
    rng = rng or random.Random()
    node = cm.nearest_node(start[0], start[1], start[2])
    if node is None:
        raise RuntimeError(f"позиция {start} вне карты")
    if node.id in blocked:
        free = _nearest_free(cm, node, blocked)
        if free is not None:
            node = free
    max_cl = max(n.clearance for n in cm.nodes) if cm.nodes else 0.0
    min_cl = min(cfg.goal_min_clearance, max_cl * 0.5) if max_cl else 0.0
    pool = [
        n for n in cm.nodes
        if n.clearance >= min_cl and n.id not in blocked and n.id not in exclude and n.id != node.id
    ]
    if not pool:
        pool = [n for n in cm.nodes if n.id not in blocked and n.id != node.id]
    if not pool:
        return node.id
    n_try = min(10, len(pool))
    for _ in range(n_try):
        cand = pool[rng.randrange(len(pool))]
        if math.hypot(cand.x - start[0], cand.y - start[1]) < min_dist and len(pool) > 8:
            continue
        ids = astar(cm, node.id, cand.id, cfg, blocked)
        if ids:
            return cand.id
    return farthest_node(cm, node.id, cfg, blocked)
