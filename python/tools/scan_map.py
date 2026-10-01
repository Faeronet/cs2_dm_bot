#!/usr/bin/env python3
"""Обход контура вручную, затем плотное сканирование карты ботом.

1. Запускаешь скрипт и сам обходишь карту — пишутся общие контуры.
2. В консоли: start (или старт / ыефке) — бот берёт управление.
3. Из текущей точки идёт змейкой: вперёд до края контура, сдвиг на --step
   юнитов (по умолчанию 5), назад. Разворот камеры — сразу на нужный угол.
   Координаты читаются из памяти и дописываются в тот же JSON.

  python3 python/tools/scan_map.py -m de_dust2 -l de_dust2
  python3 python/tools/scan_map.py -m de_mirage -l de_mirage --step 5
  python3 python/tools/scan_map.py --contour data/recordings/de_cache/de_cache.contour.json
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import record_coords as rc
from cs2_control.input import create_input
from cs2_reader.aim import shortest_yaw_err, yaw_to_target
from cs2_reader.reader import Reader

YAW_PER_PX = -0.0759
PITCH_PER_PX = 0.0759
ARRIVE = 12.0
SNAP_YAW = 35.0
# На прямой не трогаем мышь, пока курс не ушёл сильнее этого.
HEADING_DEADZONE = 14.0
PITCH_DEADZONE = 8.0


def _wait_commands(box: dict, stop: threading.Event) -> None:
    for line in sys.stdin:
        cmd = line.strip().lower()
        if cmd in ("start", "старт", "ыефке"):
            box["start"] = True
            continue
        if cmd in ("stop", "стоп", "q", "quit", "exit"):
            stop.set()
            return


def _convex_hull(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    uniq = sorted(set(pts))
    if len(uniq) <= 2:
        return uniq

    def cross(o, a, b) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in uniq:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(uniq):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _polygon_area(poly: list[tuple[float, float]]) -> float:
    area = 0.0
    for i, (x, y) in enumerate(poly):
        nx, ny = poly[(i + 1) % len(poly)]
        area += x * ny - nx * y
    return abs(area) * 0.5


def _point_in_poly(poly: list[tuple[float, float]], x: float, y: float) -> bool:
    inside = False
    j = len(poly) - 1
    for i, (xi, yi) in enumerate(poly):
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xint = (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            if x < xint:
                inside = not inside
        j = i
    return inside


def _centroid(poly: list[tuple[float, float]]) -> tuple[float, float]:
    return (
        sum(p[0] for p in poly) / len(poly),
        sum(p[1] for p in poly) / len(poly),
    )


def _simplify(loop: list[tuple[float, float]], min_dist: float = 8.0) -> list[tuple[float, float]]:
    if len(loop) < 3:
        return loop
    out = [loop[0]]
    for p in loop[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= min_dist:
            out.append(p)
    if len(out) >= 3 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) < min_dist:
        out.pop()
    return out


def _extract_loops(
    path: list[tuple[float, float]],
    join: float = 32.0,
    min_area: float = 900.0,
) -> list[list[tuple[float, float]]]:
    """Замкнутые обходы: внешний контур и отдельные петли вокруг ящиков и стен."""
    loops: list[list[tuple[float, float]]] = []
    cell = join
    buckets: dict[tuple[int, int], list[int]] = {}
    for i, (x, y) in enumerate(path):
        key = (int(x // cell), int(y // cell))
        best_j: int | None = None
        best_d = join
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                for j in buckets.get((key[0] + ox, key[1] + oy), ()):
                    if i - j < 4:
                        continue
                    d = math.hypot(x - path[j][0], y - path[j][1])
                    if d <= best_d:
                        best_d = d
                        best_j = j
        if best_j is not None:
            loop = _simplify(path[best_j : i + 1])
            if len(loop) >= 4 and _polygon_area(loop) >= min_area:
                loops.append(loop)
        buckets.setdefault(key, []).append(i)
    return loops


def _classify_loops(
    loops: list[list[tuple[float, float]]],
) -> tuple[list[tuple[float, float]], list[list[tuple[float, float]]]]:
    ranked = sorted(loops, key=_polygon_area, reverse=True)
    kept: list[list[tuple[float, float]]] = []
    for loop in ranked:
        area = _polygon_area(loop)
        cx, cy = _centroid(loop)
        same = False
        for prev in kept:
            px, py = _centroid(prev)
            pa = _polygon_area(prev)
            if abs(area - pa) / pa < 0.2 and math.hypot(cx - px, cy - py) < 64.0:
                same = True
                break
        if not same:
            kept.append(loop)
    outer = kept[0]
    outer_area = _polygon_area(outer)
    holes: list[list[tuple[float, float]]] = []
    for loop in kept[1:]:
        cx, cy = _centroid(loop)
        if _polygon_area(loop) >= outer_area * 0.5:
            continue
        if _point_in_poly(outer, cx, cy):
            holes.append(loop)
    return outer, holes


def contours_from_path(
    path: list[tuple[float, float]],
) -> tuple[list[tuple[float, float]], list[list[tuple[float, float]]], bool]:
    """Внешняя граница и дыры (ящики). Третий флаг — контур пришлось заменить оболочкой."""
    loops = _extract_loops(path)
    if loops:
        outer, holes = _classify_loops(loops)
        return outer, holes, False
    if _polygon_area(path) >= 500.0:
        return _simplify(path), [], False
    return _convex_hull(path), [], True


def _segments_at(poly: list[tuple[float, float]], coord: float, *, axis: str) -> list[tuple[float, float]]:
    """Отрезки внутри полигона на горизонтали (axis=y) или вертикали (axis=x)."""
    hits: list[float] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        a1, a2 = (y1, y2) if axis == "y" else (x1, x2)
        b1, b2 = (x1, x2) if axis == "y" else (y1, y2)
        if a1 == a2 or (a1 > coord) == (a2 > coord):
            continue
        t = (coord - a1) / (a2 - a1)
        hits.append(b1 + t * (b2 - b1))
    hits.sort()
    out: list[tuple[float, float]] = []
    for i in range(0, len(hits) - 1, 2):
        if hits[i + 1] - hits[i] >= 8.0:
            out.append((hits[i], hits[i + 1]))
    return out


def _subtract_intervals(
    base: list[tuple[float, float]],
    cuts: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    pieces = list(base)
    for c, d in cuts:
        nxt: list[tuple[float, float]] = []
        for x, y in pieces:
            if d <= x or c >= y:
                nxt.append((x, y))
                continue
            if c - x >= 8.0:
                nxt.append((x, c))
            if y - d >= 8.0:
                nxt.append((d, y))
        pieces = nxt
    return pieces


def _free_cells(
    outer: list[tuple[float, float]],
    holes: list[list[tuple[float, float]]],
    step: float,
) -> tuple[set[tuple[int, int]], float, float]:
    xs = [p[0] for p in outer]
    ys = [p[1] for p in outer]
    ox, oy = min(xs), min(ys)
    free: set[tuple[int, int]] = set()
    coord = math.floor(min(ys))
    last = math.ceil(max(ys))
    while coord <= last:
        segs = _subtract_intervals(
            _segments_at(outer, coord, axis="y"),
            [s for hole in holes for s in _segments_at(hole, coord, axis="y")],
        )
        for a, b in segs:
            x = a
            while x <= b:
                free.add((int(round((x - ox) / step)), int(round((coord - oy) / step))))
                x += step
        coord += step
    return free, ox, oy


def _cell_of(x: float, y: float, ox: float, oy: float, step: float) -> tuple[int, int]:
    return int(round((x - ox) / step)), int(round((y - oy) / step))


def _world_of(ix: int, iy: int, ox: float, oy: float, step: float) -> tuple[float, float]:
    return ox + ix * step, oy + iy * step


def _nearest_free(cell: tuple[int, int], free: set[tuple[int, int]], radius: int = 8) -> tuple[int, int] | None:
    if cell in free:
        return cell
    cx, cy = cell
    best: tuple[int, int] | None = None
    best_d = 10**9
    for r in range(1, radius + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                cand = (cx + dx, cy + dy)
                if cand not in free:
                    continue
                d = dx * dx + dy * dy
                if d < best_d:
                    best_d = d
                    best = cand
        if best is not None:
            return best
    return None


def _grid_clear(
    a: tuple[float, float],
    b: tuple[float, float],
    free: set[tuple[int, int]],
    ox: float,
    oy: float,
    step: float,
) -> bool:
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(dist / max(step * 0.5, 1.0)))
    for i in range(n + 1):
        t = i / n
        cell = _cell_of(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, ox, oy, step)
        if cell not in free:
            return False
    return True


def _astar(
    start: tuple[int, int],
    goal: tuple[int, int],
    free: set[tuple[int, int]],
) -> list[tuple[int, int]] | None:
    start = _nearest_free(start, free) or start
    goal = _nearest_free(goal, free) or goal
    if start not in free or goal not in free:
        return None
    if start == goal:
        return [start]
    open_heap: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start: 0.0}
    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path
        cx, cy = cur
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = (cx + dx, cy + dy)
            if nxt not in free:
                continue
            if dx and dy and ((cx + dx, cy) not in free or (cx, cy + dy) not in free):
                continue
            step_cost = math.sqrt(2.0) if dx and dy else 1.0
            ncost = cost[cur] + step_cost
            if ncost >= cost.get(nxt, 1e18):
                continue
            came[nxt] = cur
            cost[nxt] = ncost
            heur = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(open_heap, (ncost + heur, nxt))
    return None


def _stitch(
    points: list[tuple[float, float]],
    free: set[tuple[int, int]],
    ox: float,
    oy: float,
    step: float,
) -> list[tuple[float, float]]:
    if not points:
        return []
    route = [points[0]]
    for nxt in points[1:]:
        prev = route[-1]
        if _grid_clear(prev, nxt, free, ox, oy, step):
            route.append(nxt)
            continue
        via = _astar(
            _cell_of(prev[0], prev[1], ox, oy, step),
            _cell_of(nxt[0], nxt[1], ox, oy, step),
            free,
        )
        if not via:
            route.append(nxt)
            continue
        for cell in via[1:]:
            route.append(_world_of(cell[0], cell[1], ox, oy, step))
        if math.hypot(route[-1][0] - nxt[0], route[-1][1] - nxt[1]) > step * 0.6:
            route.append(nxt)
    return route


def build_scan_path(
    poly: list[tuple[float, float]],
    origin: tuple[float, float],
    step: float,
    holes: list[list[tuple[float, float]]] | None = None,
) -> list[tuple[float, float]]:
    """Змейка внутри контура. Ящики и стены из петель контура вырезаются, обход — по свободным клеткам."""
    holes = holes or []
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    axis = "y" if span_x >= span_y else "x"
    lo = min(ys) if axis == "y" else min(xs)
    hi = max(ys) if axis == "y" else max(xs)
    origin_along = origin[1] if axis == "y" else origin[0]
    origin_across = origin[0] if axis == "y" else origin[1]

    rows: list[tuple[float, list[tuple[float, float]]]] = []
    coord = math.floor(lo)
    last = math.ceil(hi)
    while coord <= last:
        cuts = [s for hole in holes for s in _segments_at(hole, coord, axis=axis)]
        segs = _subtract_intervals(_segments_at(poly, coord, axis=axis), cuts)
        if segs:
            rows.append((float(coord), segs))
        coord += step
    if not rows:
        return []

    i0 = min(range(len(rows)), key=lambda i: abs(rows[i][0] - origin_along))
    order = list(range(i0, len(rows))) + list(range(i0 - 1, -1, -1))

    waypoints: list[tuple[float, float]] = []
    across = origin_across
    go_positive = True
    for idx, row_i in enumerate(order):
        line, segs = rows[row_i]
        if idx == 0:
            mid = sum((a + b) * 0.5 for a, b in segs) / len(segs)
            go_positive = across <= mid
        segs = sorted(segs, key=lambda s: s[0], reverse=not go_positive)
        for a, b in segs:
            start_c, end_c = (a, b) if go_positive else (b, a)
            start = (start_c, line) if axis == "y" else (line, start_c)
            end = (end_c, line) if axis == "y" else (line, end_c)
            waypoints.append(start)
            waypoints.append(end)
            across = end_c
        go_positive = not go_positive
    if not waypoints:
        return []
    free, ox, oy = _free_cells(poly, holes, step)
    return _simplify_route(_stitch([origin, *waypoints], free, ox, oy, step))


class ScanRecorder:
    def __init__(self) -> None:
        self.seen: set[tuple[int, int, int]] = set()
        self.points: list[list[int]] = []
        self.path: list[list[int] | None] = []
        self.samples = 0

    def add(self, pt: tuple[int, int, int] | None, *, gap: bool = False) -> bool:
        self.samples += 1
        if gap and self.path and self.path[-1] is not None:
            self.path.append(None)
        if pt is None or pt in self.seen:
            return False
        self.seen.add(pt)
        row = [pt[0], pt[1], pt[2]]
        self.points.append(row)
        self.path.append(row)
        return True


def _contour_file(map_name: str, location: str) -> Path:
    return rc.OUT_DIR / map_name / f"{location}.contour.json"


def _scan_file(map_name: str, location: str) -> Path:
    return rc.OUT_DIR / map_name / f"{location}.scan.json"


def _save_contour(
    map_name: str,
    location: str,
    points: list[list[int]],
    path: list[list[int]],
) -> Path:
    out = _contour_file(map_name, location)
    out.parent.mkdir(parents=True, exist_ok=True)
    now = datetime.now(timezone.utc).isoformat()
    payload = {
        "kind": "contour",
        "map": map_name,
        "location": location,
        "created": now,
        "unique_points": len(points),
        "points": points,
        "path": path,
    }
    out.write_text(json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")
    return out


def _save_scan(
    map_name: str,
    location: str,
    scan: ScanRecorder,
    *,
    step: float,
    duration_s: float,
    contour_file: str,
) -> Path:
    out = _scan_file(map_name, location)
    out.parent.mkdir(parents=True, exist_ok=True)
    now = datetime.now(timezone.utc).isoformat()
    prev = rc._load_recording(out) or {}
    payload = {
        "kind": "scan",
        "map": map_name,
        "location": location,
        "created": prev.get("created") or now,
        "updated": now,
        "contour_file": contour_file,
        "step": step,
        "duration_s": round(duration_s, 2),
        "samples": scan.samples,
        "unique_points": len(scan.points),
        "points": scan.points,
        "path": scan.path,
    }
    out.write_text(json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")
    return out


def _aim_error(reader: Reader, tx: float, ty: float, pos) -> tuple[float, float]:
    bearing = yaw_to_target(pos.x, pos.y, tx, ty)
    eye = reader.local_eye_angles()
    return shortest_yaw_err(bearing, eye.y), 0.0 - eye.x


def _snap_turn(reader: Reader, inp, tx: float, ty: float) -> float:
    """Разворот одним движением мыши, без нарезки по тикам."""
    set_spread = getattr(inp, "set_mouse_spread", None)
    if set_spread:
        set_spread(0.001)
    yaw_err = 0.0
    try:
        for _ in range(2):
            pos = reader.local_position()
            yaw_err, pitch_err = _aim_error(reader, tx, ty, pos)
            if abs(yaw_err) < 4.0 and abs(pitch_err) < 3.0:
                return yaw_err
            dx = int(round(yaw_err / YAW_PER_PX))
            dy = int(round(pitch_err / PITCH_PER_PX))
            if dx or dy:
                inp.mouse_move(dx, dy)
            time.sleep(0.02)
    finally:
        if set_spread:
            set_spread(0.025)
    return yaw_err


def _turn(reader: Reader, inp, tx: float, ty: float, pos) -> float:
    """Мелкая ошибка на прямой игнорируется, иначе камера качается влево-вправо."""
    yaw_err, pitch_err = _aim_error(reader, tx, ty, pos)
    if abs(yaw_err) >= SNAP_YAW:
        return _snap_turn(reader, inp, tx, ty)
    if abs(yaw_err) < HEADING_DEADZONE and abs(pitch_err) < PITCH_DEADZONE:
        return yaw_err
    yaw_fix = 0.0 if abs(yaw_err) < HEADING_DEADZONE else yaw_err * 0.45
    pitch_fix = 0.0 if abs(pitch_err) < PITCH_DEADZONE else pitch_err * 0.45
    dx = int(round(yaw_fix / YAW_PER_PX))
    dy = int(round(pitch_fix / PITCH_PER_PX))
    dx = max(-20, min(20, dx))
    dy = max(-12, min(12, dy))
    if dx or dy:
        inp.mouse_move(dx, dy)
    return yaw_err


def _simplify_route(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Прямой участок — одна цель, а не лесенка клеток, из-за которой прицел виляет."""
    if len(points) < 3:
        return points
    out = [points[0]]
    for i in range(1, len(points) - 1):
        a = out[-1]
        b = points[i]
        c = points[i + 1]
        if math.hypot(b[0] - a[0], b[1] - a[1]) < 1.0:
            continue
        b1 = yaw_to_target(a[0], a[1], b[0], b[1])
        b2 = yaw_to_target(b[0], b[1], c[0], c[1])
        if abs(shortest_yaw_err(b2, b1)) > 18.0:
            out.append(b)
    out.append(points[-1])
    return out


def _walk_to(
    reader: Reader,
    pos_reader: rc.FastPositionReader,
    inp,
    tx: float,
    ty: float,
    rec: ScanRecorder,
    stop: threading.Event,
) -> bool:
    """Идти к точке. False — упёрлись в стену или stop."""
    origin = pos_reader.read()
    if origin is None:
        return False
    dist = math.hypot(tx - origin[0], ty - origin[1])
    if dist <= ARRIVE:
        rec.add(origin)
        return True
    timeout = max(3.0, dist / 70.0 + 1.5)
    t0 = time.perf_counter()
    _snap_turn(reader, inp, tx, ty)
    still_since: float | None = None
    still_at: tuple[int, int, int] | None = None
    moving = False
    next_nudge = 0.0
    try:
        while not stop.is_set():
            if time.perf_counter() - t0 > timeout:
                return False
            pt = pos_reader.read()
            if pt is None:
                time.sleep(0.02)
                continue
            rec.add(pt)
            pos = reader.local_position()
            if math.hypot(pos.x - tx, pos.y - ty) <= ARRIVE:
                return True
            now = time.perf_counter()
            if now >= next_nudge:
                yaw_err = _turn(reader, inp, tx, ty, pos)
                if abs(yaw_err) >= HEADING_DEADZONE:
                    next_nudge = now + 0.18
            else:
                yaw_err, _pitch = _aim_error(reader, tx, ty, pos)
            if abs(yaw_err) > 18.0:
                if moving:
                    inp.key_up("w")
                    moving = False
            elif not moving:
                inp.key_down("w")
                moving = True
            if still_at is not None and math.hypot(pt[0] - still_at[0], pt[1] - still_at[1]) < 3.0:
                if still_since is not None and time.perf_counter() - still_since > 0.45 and moving:
                    return False
            else:
                still_at = pt
                still_since = time.perf_counter()
            time.sleep(0.02)
    finally:
        inp.key_up("w")
    return False


def _load_contour(spec: str) -> tuple[str, str, list[list[int]], list[list[int]], Path]:
    """Контур из JSON: блок contour, иначе points/path всей записи."""
    path = Path(spec)
    if not path.is_file():
        alt = rc.ROOT / spec
        if alt.is_file():
            path = alt
    if not path.is_file():
        raise FileNotFoundError(spec)
    data = json.loads(path.read_text(encoding="utf-8"))
    block = data.get("contour") or {}
    raw_path = block.get("path") or data.get("path") or block.get("points") or data.get("points") or []
    raw_points = block.get("points") or data.get("points") or raw_path
    contour_path = [[int(v) for v in p] for p in raw_path if p is not None]
    contour_pts = [[int(v) for v in p] for p in raw_points if p is not None]
    if len(contour_path) < 3:
        raise ValueError(f"в {path} нет контура (нужен блок contour или points/path)")
    map_name = str(data.get("map") or "")
    location = str(data.get("location") or path.stem)
    return map_name, location, contour_pts or contour_path, contour_path, path


def _record_contour(
    pos_reader: rc.FastPositionReader,
    hz: float,
    stop: threading.Event,
    started: dict,
) -> tuple[list[list[int]], list[list[int]]]:
    interval = 1.0 / max(1.0, hz)
    seen: set[tuple[int, int, int]] = set()
    points: list[list[int]] = []
    path: list[list[int]] = []
    last: tuple[int, int, int] | None = None
    next_tick = time.perf_counter()
    print("Обходи карту по контуру. Координаты пишутся ниже.")
    print("Когда контур готов — в этой консоли: start")
    print("Выйти без сканирования: stop")
    print("-" * 50)
    while not stop.is_set() and not started.get("start"):
        now = time.perf_counter()
        if now < next_tick:
            time.sleep(min(0.01, next_tick - now))
            continue
        next_tick += interval
        pt = pos_reader.read()
        if pt is None or pt == last or pt in seen:
            last = pt or last
            continue
        last = pt
        seen.add(pt)
        row = [pt[0], pt[1], pt[2]]
        points.append(row)
        path.append(row)
        print(f"  контур {len(points):5d}  ({pt[0]}, {pt[1]}, {pt[2]})")
    return points, path


def _run_scan(
    args,
    reader: Reader,
    map_name: str,
    location: str,
    contour: tuple[list[list[int]], list[list[int]]] | None = None,
) -> int:
    pos_reader = rc.FastPositionReader(reader)
    stop = threading.Event()
    started: dict = {"start": False}
    threading.Thread(target=_wait_commands, args=(started, stop), daemon=True).start()

    print(f"Карта: {map_name}  |  зона: {location}  |  шаг скана: {args.step:g} u")
    if contour is not None:
        contour_pts, contour_path = contour
        print(f"Контур из файла: {len(contour_path)} точек пути, без повторного обхода.")
        print("Остановка скана: stop или Ctrl+C")
    else:
        contour_pts, contour_path = _record_contour(pos_reader, args.hz, stop, started)
        if stop.is_set() and not started.get("start"):
            if contour_pts:
                out = _save_contour(map_name, location, contour_pts, contour_path)
                print(f"Контур сохранён ({len(contour_pts)} точек): {out}")
            else:
                print("Контур пуст, файл не пишу.")
            return 0
    if len(contour_pts) < 3:
        print("Мало точек контура — обойди границу и снова напиши start.", file=sys.stderr)
        return 1

    contour_out = _save_contour(map_name, location, contour_pts, contour_path)
    if contour is None:
        print(f"Контур: {contour_out}")

    xy = [(float(p[0]), float(p[1])) for p in contour_path]
    outer, holes, used_hull = contours_from_path(xy)
    if len(outer) < 3 or _polygon_area(outer) < 500.0:
        print("Контур не замыкает площадь. Обойди область петлёй и повтори.", file=sys.stderr)
        return 1
    if used_hull:
        print("Контур почти не замкнут — сканирую выпуклую оболочку обхода.")
    elif holes:
        print(f"Контур: внешняя граница и {len(holes)} препятствий (ящики/стены) — маршрут их обойдёт.")

    here = pos_reader.read()
    if here is None:
        print("Нет позиции игрока.", file=sys.stderr)
        return 1
    waypoints = build_scan_path(outer, (here[0], here[1]), args.step, holes)
    if not waypoints:
        print("Внутри контура нет рядов для скана.", file=sys.stderr)
        return 1

    print("-" * 50)
    print(f"Контур: {len(contour_pts)} точек, рядовых целей: {len(waypoints)}")
    print("Через 3 с бот пойдёт из текущей точки. CS2 должен быть в фокусе.")
    print("Остановка: stop или Ctrl+C — координаты сохранятся.")
    for sec in range(3, 0, -1):
        if stop.is_set():
            break
        print(f"  {sec}…")
        time.sleep(1)

    inp = create_input()
    rec = ScanRecorder()
    t0 = time.perf_counter()
    last_save = t0
    blocked = 0
    try:
        for i, (tx, ty) in enumerate(waypoints):
            if stop.is_set():
                break
            ok = _walk_to(reader, pos_reader, inp, tx, ty, rec, stop)
            if not ok:
                blocked += 1
                rec.add(None, gap=True)
            if i % 2 == 1 or not ok:
                print(
                    f"  скан {i + 1}/{len(waypoints)}  точек={len(rec.points)}  "
                    f"стен={blocked}  → ({tx:.0f}, {ty:.0f})"
                )
            now = time.perf_counter()
            if now - last_save > 45.0:
                _save_scan(
                    map_name, location, rec,
                    step=args.step, duration_s=now - t0,
                    contour_file=str(contour_out),
                )
                last_save = now
                print(f"  …скан сохранён ({len(rec.points)} точек)")
    except KeyboardInterrupt:
        print("\nПрервано.")
        stop.set()
    finally:
        inp.release_all()
        inp.close()

    duration = time.perf_counter() - t0
    out = _save_scan(
        map_name, location, rec,
        step=args.step, duration_s=duration,
        contour_file=str(contour_out),
    )
    print("-" * 50)
    print(f"Скан: {len(rec.points)} новых точек за {duration:.0f} с, упоров в стену: {blocked}")
    print(f"Контур: {contour_out}")
    print(f"Скан:   {out}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Контур вручную, затем скан карты змейкой")
    parser.add_argument("--map", "-m", help="Карта (de_dust2 …)")
    parser.add_argument("--location", "-l", help="Имя записи")
    parser.add_argument("--step", type=float, default=5.0, help="Сдвиг между проходами, юниты")
    parser.add_argument("--hz", type=float, default=64.0, help="Частота чтения координат на контуре")
    parser.add_argument(
        "--contour",
        "-c",
        help="JSON с уже записанным контуром: сразу скан, без нового обхода",
    )
    parser.add_argument("--geometry-check", action="store_true", help="Проверить построение змейки и выйти")
    args = parser.parse_args()

    if args.step <= 0:
        print("--step должен быть > 0", file=sys.stderr)
        return 1

    if args.geometry_check:
        outer = [(0.0, 0.0), (200.0, 0.0), (200.0, 80.0), (0.0, 80.0), (0.0, 0.0)]
        box = [(80.0, 20.0), (140.0, 20.0), (140.0, 55.0), (80.0, 55.0), (80.0, 20.0)]
        path = outer + [(40.0, 40.0)] + box
        got, holes, hull = contours_from_path(path)
        assert not hull and len(holes) == 1, (len(holes), hull, round(_polygon_area(got)))
        route = build_scan_path(got, (10.0, 10.0), 10.0, holes)
        assert route, "пустой скан"
        through = [
            p for p in route
            if 80.0 < p[0] < 140.0 and 20.0 < p[1] < 55.0
        ]
        assert not through, through[:6]
        left = any(p[0] < 70.0 and 25.0 < p[1] < 50.0 for p in route)
        right = any(p[0] > 150.0 and 25.0 < p[1] < 50.0 for p in route)
        assert left and right, (left, right, len(route))
        print(f"geometry ok: holes={len(holes)} точек={len(route)}")
        return 0

    if os.geteuid() == 0:
        print("Не запускай от root — ввод не дойдёт до CS2.", file=sys.stderr)
        return 1

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен.", file=sys.stderr)
        return 1
    pawn = None
    for _ in range(20):
        pawn = reader.local_pawn()
        if pawn:
            break
        time.sleep(0.25)
    if not pawn:
        live = reader.map_name() or "<карта не прочиталась>"
        print(
            f"Игрок на карте не читается (map={live}). "
            "Если вы уже в матче, оффсеты устарели: bash scripts/dump_live.sh",
            file=sys.stderr,
        )
        return 1

    loaded: tuple[list[list[int]], list[list[int]]] | None = None
    file_map = ""
    file_location = ""
    if args.contour:
        try:
            file_map, file_location, contour_pts, contour_path, contour_file = _load_contour(args.contour)
        except (OSError, json.JSONDecodeError, ValueError, FileNotFoundError) as exc:
            print(f"Не удалось прочитать контур: {exc}", file=sys.stderr)
            return 1
        loaded = (contour_pts, contour_path)
        print(f"Контур: {contour_file}  ({len(contour_path)} точек)")

    try:
        map_name = rc._normalize_map(args.map) if args.map else (rc._normalize_map(file_map) if file_map else rc._prompt_map())
        location = (
            rc._normalize_location(args.location)
            if args.location
            else (rc._normalize_location(file_location) if file_location else rc._prompt_location())
        )
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    live = reader.map_name().replace("maps/", "").split(".")[0]
    if live and live != map_name:
        print(f"Внимание: в игре сейчас «{live}», запись пойдёт в «{map_name}».", file=sys.stderr)

    try:
        return _run_scan(args, reader, map_name, location, loaded)
    except KeyboardInterrupt:
        print("\nПрервано.")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
