#!/usr/bin/env python3
"""Объединить несколько записей координат в один файл.

Точки из более поздних файлов, попавшие ближе --radius к уже взятым, выбрасываются
(перекрытие на стыке локаций). Trail'ы (path) сохраняются с разрывом (null) между
файлами, чтобы карта не соединяла конец одной записи с началом другой.

  python3 python/tools/merge_recordings.py -l tunels \
      data/recordings/de_dust2/lower_tunel1.json data/recordings/de_dust2/uper_tunel.json
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
REC_DIR = Path(ROOT) / "data" / "recordings"


def resolve(specs: list[str]) -> list[Path]:
    out: list[Path] = []
    for spec in specs:
        p = spec if os.path.isabs(spec) else os.path.join(ROOT, spec)
        hits = sorted(glob.glob(p)) or [p]
        out.extend(Path(h) for h in hits)
    return out


class _Grid:
    """Пространственный индекс уже принятых точек (чтобы не O(n²))."""

    def __init__(self, radius: float):
        self.cell = max(1.0, radius)
        self.radius = radius
        self.cells: dict[tuple[int, int, int], list[list[int]]] = {}

    def _key(self, p: list[int]) -> tuple[int, int, int]:
        return (int(p[0] // self.cell), int(p[1] // self.cell), int(p[2] // self.cell))

    def near(self, p: list[int]) -> bool:
        kx, ky, kz = self._key(p)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for q in self.cells.get((kx + dx, ky + dy, kz + dz), ()):
                        if math.dist(p, q) <= self.radius:
                            return True
        return False

    def add_all(self, pts: list[list[int]]) -> None:
        for p in pts:
            self.cells.setdefault(self._key(p), []).append(p)


def dedup_across(files_points: list[list[list[int]]], radius: float) -> tuple[list[list[int]], list[int]]:
    """Убрать из каждого следующего файла точки ближе radius к точкам ПРЕДЫДУЩИХ файлов.

    Внутри одного файла точки не трогаем (там свои соседи через 1–3u — это норма),
    убираются только перекрытия на стыке локаций и точные дубликаты.
    """
    grid = _Grid(radius)
    kept: list[list[int]] = []
    dropped_per_file: list[int] = []
    seen: set[tuple[int, int, int]] = set()
    for pts in files_points:
        accepted: list[list[int]] = []
        dropped = 0
        for p in pts:
            t = (p[0], p[1], p[2])
            if t in seen or grid.near(p):
                dropped += 1
                continue
            seen.add(t)
            accepted.append(p)
        grid.add_all(accepted)
        kept.extend(accepted)
        dropped_per_file.append(dropped)
    return kept, dropped_per_file


def main() -> int:
    parser = argparse.ArgumentParser(description="Слияние записей координат")
    parser.add_argument("inputs", nargs="+", help="JSON записи (пути или glob), порядок = приоритет")
    parser.add_argument("-l", "--location", required=True, help="имя объединённой локации (имя файла)")
    parser.add_argument("--radius", type=float, default=8.0, help="точки ближе R юнитов считаются дубликатами")
    parser.add_argument("--out", default=None, help="куда сохранить (по умолчанию data/recordings/<map>/<location>.json)")
    args = parser.parse_args()

    files = resolve(args.inputs)
    if len(files) < 2:
        print("Нужно минимум два файла", file=sys.stderr)
        return 1

    map_name: str | None = None
    files_points: list[list[list[int]]] = []
    path: list[list[int] | None] = []
    sources: list[str] = []
    samples = 0
    duration = 0.0
    poll_hz = None
    crouch_points: list[list[int]] = []
    crouch_path: list[list[int] | None] = []
    crouch_duration = 0.0
    crouch_samples = 0
    crouch_created: str | None = None
    for f in files:
        raw = json.loads(f.read_text(encoding="utf-8"))
        m = raw.get("map", "unknown")
        if map_name is None:
            map_name = m
        elif m != map_name:
            print(f"Разные карты: {map_name} и {m} ({f})", file=sys.stderr)
            return 1
        file_pts = [[int(v) for v in p] for p in raw.get("points", [])]
        file_path = [[int(v) for v in p] if p is not None else None for p in (raw.get("path") or file_pts)]
        files_points.append(file_pts)
        if path and file_path:
            path.append(None)
        path.extend(file_path)
        sources.append(str(raw.get("location", f.stem)))
        samples += int(raw.get("samples", 0))
        duration += float(raw.get("duration_s", 0.0))
        poll_hz = poll_hz or raw.get("poll_hz")
        crouch = raw.get("crouch") or {}
        file_crouch_pts = [[int(v) for v in p] for p in crouch.get("points") or []]
        file_crouch_path = [
            [int(v) for v in p] if p is not None else None for p in (crouch.get("path") or file_crouch_pts)
        ]
        if file_crouch_pts:
            crouch_created = crouch_created or crouch.get("created")
            crouch_duration += float(crouch.get("duration_s", 0.0))
            crouch_samples += int(crouch.get("samples", 0))
            crouch_points.extend(file_crouch_pts)
            if crouch_path and file_crouch_path:
                crouch_path.append(None)
            crouch_path.extend(file_crouch_path)

    kept, dropped = dedup_across(files_points, args.radius)
    for f, pts, d in zip(files, files_points, dropped):
        print(f"{f.name}: {len(pts)} точек, убрано дубликатов (в {args.radius:g}u от предыдущих файлов): {d}")
    print(f"Итого {len(kept)} точек")

    out = Path(args.out) if args.out else REC_DIR / str(map_name) / f"{args.location}.json"
    if not out.is_absolute():
        out = Path(ROOT) / out
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "map": map_name,
        "location": args.location,
        "created": datetime.now(timezone.utc).isoformat(),
        "sources": sources,
        "merge_radius": args.radius,
        "poll_hz": poll_hz,
        "duration_s": round(duration, 2),
        "samples": samples,
        "unique_points": len(kept),
        "points": kept,
        "path": path,
    }
    if crouch_points:
        seen: set[tuple[int, int, int]] = set()
        merged_crouch: list[list[int]] = []
        for p in crouch_points:
            t = (p[0], p[1], p[2])
            if t in seen:
                continue
            seen.add(t)
            merged_crouch.append(p)
        payload["crouch"] = {
            "created": crouch_created or datetime.now(timezone.utc).isoformat(),
            "updated": datetime.now(timezone.utc).isoformat(),
            "sources": sources,
            "poll_hz": poll_hz,
            "duration_s": round(crouch_duration, 2),
            "samples": crouch_samples,
            "unique_points": len(merged_crouch),
            "points": merged_crouch,
            "path": crouch_path,
        }
        print(f"Crouch: {len(merged_crouch)} точек")
    out.write_text(json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"→ {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
