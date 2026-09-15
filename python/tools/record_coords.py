#!/usr/bin/env python3
"""Запись координат локации из памяти CS2 с минимальной задержкой.

Перед стартом обязательно: карта (de_dust2 …) и зона (b_tunnel, short …).
Бегаете по локации — скрипт пишет уникальные целые (x, y, z).
В консоли: stop / стоп / q — сохранить и выйти.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.reader import Reader

ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = ROOT / "data" / "recordings"
MAPS_DIR = ROOT / "data" / "maps"

KNOWN_LOCATIONS = (
    "ct_spawn",
    "t_spawn",
    "mid",
    "short",
    "long",
    "b_site",
    "b_plant",
    "b_tunnel",
    "a_site",
    "cat",
    "pit",
    "outside",
    "tunnels",
    "custom",
)


class FastPositionReader:
    """Только позиция: без capture() и без списка игроков."""

    def __init__(self, reader: Reader, pawn_refresh_s: float = 0.4):
        self.reader = reader
        self.schema = reader.schema
        self._pawn: int | None = None
        self._pawn_refresh_s = pawn_refresh_s
        self._last_pawn_check = 0.0

    def _refresh_pawn(self, force: bool = False) -> int | None:
        now = time.time()
        if force or self._pawn is None or now - self._last_pawn_check >= self._pawn_refresh_s:
            self._pawn = self.reader.local_pawn()
            self._last_pawn_check = now
        return self._pawn

    def read(self) -> tuple[int, int, int] | None:
        pawn = self._refresh_pawn()
        if not pawn:
            self._pawn = None
            return None

        backend = self.reader.backend
        coords = backend.read_vec3(pawn + self.schema.m_v_old_origin)
        if not coords:
            scene = backend.read_u64(pawn + self.schema.m_p_game_scene_node)
            if scene:
                coords = backend.read_vec3(scene + self.schema.m_vec_abs_origin)
        if not coords:
            self._refresh_pawn(force=True)
            return None

        return round(coords[0]), round(coords[1]), round(coords[2])


def _known_maps() -> list[str]:
    names: set[str] = set()
    if MAPS_DIR.is_dir():
        for path in MAPS_DIR.glob("*.geometry.json"):
            names.add(path.name.replace(".geometry.json", ""))
        for path in MAPS_DIR.glob("*.points.json"):
            names.add(path.name.replace(".points.json", ""))
    return sorted(names)


def _normalize_map(raw: str) -> str:
    name = raw.strip().lower().replace(" ", "_").replace("-", "_")
    if name.startswith("maps/"):
        name = name.split("/")[-1]
    if name.endswith(".bsp"):
        name = name[:-4]
    if not name:
        raise ValueError("пустое имя карты")
    for ch in name:
        if not (ch.isalnum() or ch == "_"):
            raise ValueError(f"недопустимый символ в имени карты: {raw!r}")
    return name


def _normalize_location(raw: str) -> str:
    loc = raw.strip().lower().replace(" ", "_").replace("-", "_")
    if not loc:
        raise ValueError("пустое имя локации")
    for ch in loc:
        if not (ch.isalnum() or ch == "_"):
            raise ValueError(f"недопустимый символ в имени: {loc!r}")
    return loc


def _prompt_map() -> str:
    known = _known_maps()
    if known:
        print("Карты в data/maps:", ", ".join(known))
    while True:
        raw = input("Карта (de_dust2 …): ").strip()
        if not raw:
            print("  укажите карту")
            continue
        try:
            return _normalize_map(raw)
        except ValueError as exc:
            print(f"  {exc}")


def _prompt_location() -> str:
    print("Известные зоны:", ", ".join(KNOWN_LOCATIONS))
    while True:
        raw = input("Локация (b_tunnel, short, ct_spawn …): ").strip()
        if not raw:
            print("  укажите имя зоны")
            continue
        try:
            return _normalize_location(raw)
        except ValueError as exc:
            print(f"  {exc}")


def _wait_for_stop(stop: threading.Event) -> None:
    for line in sys.stdin:
        if line.strip().lower() in ("stop", "стоп", "q", "quit", "exit"):
            stop.set()
            return


def _merge_nearby(points: list[list[int]], radius: int) -> list[list[int]]:
    """Убрать точки ближе radius (2D) — на случай мелкого дрожания округления."""
    if radius <= 0:
        return points
    kept: list[list[int]] = []
    for pt in points:
        x, y, z = pt
        too_close = False
        for kx, ky, kz in kept:
            if abs(x - kx) <= radius and abs(y - ky) <= radius and abs(z - kz) <= radius:
                too_close = True
                break
        if not too_close:
            kept.append(pt)
    return kept


def _save(
    map_name: str,
    location: str,
    points: list[list[int]],
    path: list[list[int]],
    poll_hz: float,
    samples: int,
    duration_s: float,
) -> Path:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / map_name / f"{location}.json"
    out.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "map": map_name,
        "location": location,
        "created": datetime.now(timezone.utc).isoformat(),
        "poll_hz": poll_hz,
        "duration_s": round(duration_s, 2),
        "samples": samples,
        "unique_points": len(points),
        "points": points,
        "path": path,
    }
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Запись уникальных целых координат локации из памяти CS2"
    )
    parser.add_argument("--map", "-m", help="Карта (de_dust2 …) — обязательно")
    parser.add_argument("--location", "-l", help="Зона (b_tunnel, short, …) — обязательно")
    parser.add_argument("--hz", type=float, default=128.0, help="Частота опроса позиции (по умолчанию 128)")
    parser.add_argument(
        "--merge-radius",
        type=int,
        default=0,
        help="После stop: убрать точки ближе N единиц (0 = только точные дубликаты)",
    )
    parser.add_argument("--list", action="store_true", help="Показать сохранённые записи и выйти")
    args = parser.parse_args()

    if args.list:
        if not OUT_DIR.exists():
            print("Записей пока нет.")
            return 0
        for path in sorted(OUT_DIR.rglob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                n = data.get("unique_points", len(data.get("points", [])))
                print(f"{path.relative_to(ROOT)}  —  {n} точек")
            except (json.JSONDecodeError, OSError):
                print(path.relative_to(ROOT))
        return 0

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен.", file=sys.stderr)
        return 1

    try:
        map_name = _normalize_map(args.map) if args.map else _prompt_map()
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    try:
        location = _normalize_location(args.location) if args.location else _prompt_location()
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    snap = reader.capture()
    if not snap.in_match:
        print("Зайди в матч на нужной карте.", file=sys.stderr)
        return 1

    live_map = snap.map_name.replace("maps/", "").split(".")[0]
    if live_map and live_map != map_name:
        print(
            f"Внимание: в игре сейчас «{live_map}», запись пойдёт в «{map_name}».",
            file=sys.stderr,
        )

    interval = 1.0 / max(1.0, args.hz)
    pos_reader = FastPositionReader(reader)

    seen: set[tuple[int, int, int]] = set()
    points: list[list[int]] = []
    path: list[list[int]] = []
    last: tuple[int, int, int] | None = None
    stuck_streak = 0
    samples = 0
    stop = threading.Event()

    threading.Thread(target=_wait_for_stop, args=(stop,), daemon=True).start()

    print()
    print(f"Карта: {map_name}  |  зона: {location}  |  опрос: {args.hz:.0f} Hz")
    print("Бегай по локации. Новые точки пишутся ниже.")
    print("Остановка: введите stop (или стоп / q) и Enter")
    print("-" * 50)

    t0 = time.perf_counter()
    next_tick = t0

    while not stop.is_set():
        now = time.perf_counter()
        if now < next_tick:
            time.sleep(min(0.001, next_tick - now))
            continue
        next_tick += interval
        samples += 1

        pt = pos_reader.read()
        if pt is None:
            if samples % int(args.hz * 2) == 0:
                print("[ожидание pawn…]")
            continue

        if pt == last:
            stuck_streak += 1
            continue

        stuck_streak = 0
        last = pt

        if pt in seen:
            continue

        seen.add(pt)
        row = [pt[0], pt[1], pt[2]]
        points.append(row)
        path.append(row)
        print(f"  +{len(points):4d}  ({pt[0]}, {pt[1]}, {pt[2]})")

    duration_s = time.perf_counter() - t0

    if args.merge_radius > 0:
        before = len(points)
        points = _merge_nearby(points, args.merge_radius)
        if len(points) < before:
            print(f"merge-radius {args.merge_radius}: {before} → {len(points)} точек")

    points.sort(key=lambda p: (p[0], p[1], p[2]))

    if not points:
        print("Нет записанных точек.")
        return 1

    out = _save(map_name, location, points, path, args.hz, samples, duration_s)
    print("-" * 50)
    print(f"Готово: {len(points)} уникальных точек за {duration_s:.1f} s")
    print(f"Опросов: {samples} (~{samples / duration_s:.0f} Hz)")
    print(f"Файл: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
