#!/usr/bin/env python3
"""Построить координатную карту из записи и сохранить PNG («радар»)."""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_nav.coord_map import CoordinateMap, MapConfig
from cs2_nav.render import render_map_png

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def resolve_recordings(spec: str) -> list[str]:
    paths: list[str] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        p = part if os.path.isabs(part) else os.path.join(ROOT, part)
        hits = sorted(glob.glob(p))
        paths.extend(hits or [p])
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description="Координатная карта из recording JSON → PNG")
    parser.add_argument(
        "--recording", "-r",
        default="data/recordings/de_mirage/de_mirage.json",
        help="путь(и) через запятую или glob — несколько записей объединяются",
    )
    parser.add_argument("--out", "-o", default=None, help="PNG (по умолчанию data/maps/<map>_<loc>.png)")
    parser.add_argument("--scale", type=float, default=2.0, help="пикселей на юнит")
    parser.add_argument("--cell", type=float, default=8.0)
    parser.add_argument("--fill-radius", type=float, default=None, help="радиус закрытия (авто)")
    parser.add_argument("--clearance", action="store_true", help="раскрасить по clearance")
    parser.add_argument("--raw", action="store_true", help="показать записанные точки")
    args = parser.parse_args()

    recs = resolve_recordings(args.recording)
    t0 = time.perf_counter()
    cm = CoordinateMap.from_recording(recs, MapConfig(cell=args.cell, fill_radius=args.fill_radius))
    dt = time.perf_counter() - t0
    print(f"Карта {cm.map_name}/{cm.location}: {json.dumps(cm.stats(), ensure_ascii=False)} за {dt:.2f}s")

    out = args.out or os.path.join(ROOT, "data", "maps", f"{cm.map_name}_{cm.location}.png")
    raw = None
    if args.raw:
        raw = []
        for rec in recs:
            data = json.loads(open(rec, encoding="utf-8").read())
            raw.extend((float(p[0]), float(p[1]), float(p[2])) for p in data.get("points", []))
    path = render_map_png(cm, out, scale=args.scale, show_clearance=args.clearance, raw_points=raw)
    print(f"PNG → {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
