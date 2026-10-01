#!/usr/bin/env python3
"""Достаёт навигационную сетку Source 2 из VPK и печатает её содержимое.

Разбор .nav повторяет ValveResourceFormat.NavMesh.NavMeshFile
(Source 2 Viewer). Блоки KV3 версии 36 пропускаются по заголовку BinaryKV3:
для координат сетки их содержимое не нужно, поток после них читается как
полигоны, зоны, лестницы и параметры генерации.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path

MAGIC_VPK = 0x55AA1234
MAGIC_NAV = 0xFEEDFACE
EMBEDDED_ARCHIVE = 0x7FFF
NO_MOVABLE_MESH = 0xFFFFFFFF

NAV_FLAGS = {
    0x2: "Jump",
    0x8: "NoJump",
    0x10: "Stop",
    0x20: "Run",
    0x40: "Walk",
    0x80: "Avoid",
    0x100: "Transient",
    0x200: "DontHide",
    0x400: "Stand",
    0x800: "NoHostages",
    0x1000: "Stairs",
    0x2000: "NoMerge",
    0x4000: "ObstacleTop",
    0x8000: "NonZUp",
    0x10000: "CrouchHeight",
    0x20000: "NonZUpTransition",
    0x40000: "CrawlHeight",
}

DIRECTIONS = {0: "North", 1: "East", 2: "South", 3: "West"}
COMPRESS = {0: "none", 1: "lz4", 2: "zstd"}


class Reader:
    def __init__(self, data: bytes, pos: int = 0):
        self.data = data
        self.pos = pos

    def remaining(self) -> int:
        return len(self.data) - self.pos

    def u8(self) -> int:
        value = self.data[self.pos]
        self.pos += 1
        return value

    def u16(self) -> int:
        value = struct.unpack_from("<H", self.data, self.pos)[0]
        self.pos += 2
        return value

    def u32(self) -> int:
        value = struct.unpack_from("<I", self.data, self.pos)[0]
        self.pos += 4
        return value

    def i32(self) -> int:
        value = struct.unpack_from("<i", self.data, self.pos)[0]
        self.pos += 4
        return value

    def i64(self) -> int:
        value = struct.unpack_from("<q", self.data, self.pos)[0]
        self.pos += 8
        return value

    def f32(self) -> float:
        value = struct.unpack_from("<f", self.data, self.pos)[0]
        self.pos += 4
        return value

    def vec3(self) -> list[float]:
        return [round(self.f32(), 3), round(self.f32(), 3), round(self.f32(), 3)]

    def cstr(self, encoding: str = "utf-8") -> str:
        end = self.data.index(0, self.pos)
        raw = self.data[self.pos:end]
        self.pos = end + 1
        return raw.decode(encoding, errors="replace")

    def skip(self, count: int) -> None:
        if count < 0 or self.pos + count > len(self.data):
            raise ValueError(f"нельзя пропустить {count} байт с позиции {self.pos}")
        self.pos += count

    def align8(self) -> None:
        self.pos = (self.pos + 7) & ~7


def flag_names(value: int) -> list[str]:
    names = [name for bit, name in NAV_FLAGS.items() if value & bit]
    return names or ["None"]


def read_vpk_tree(blob: bytes) -> tuple[int, list[dict]]:
    reader = Reader(blob)
    magic = reader.u32()
    if magic != MAGIC_VPK:
        raise ValueError(f"это не VPK, magic={magic:#x}")
    version = reader.u32()
    tree_size = reader.u32()
    if version == 2:
        reader.u32()
        reader.u32()
        reader.u32()
        reader.u32()
    elif version != 1:
        raise ValueError(f"неподдерживаемая версия VPK {version}")
    data_start = reader.pos + tree_size
    tree_end = reader.pos + tree_size
    entries: list[dict] = []
    while reader.pos < tree_end:
        ext = reader.cstr("ascii")
        if not ext:
            break
        while True:
            path = reader.cstr("ascii")
            if not path:
                break
            while True:
                name = reader.cstr("ascii")
                if not name:
                    break
                crc = reader.u32()
                preload = reader.u16()
                archive = reader.u16()
                offset = reader.u32()
                length = reader.u32()
                term = reader.u16()
                if term != 0xFFFF:
                    raise ValueError(f"битый каталог VPK, terminator={term:#x}")
                preload_bytes = b""
                if preload:
                    preload_bytes = reader.data[reader.pos:reader.pos + preload]
                    reader.skip(preload)
                rel = f"{path}/{name}.{ext}" if path != " " else f"{name}.{ext}"
                entries.append(
                    {
                        "path": rel.replace("\\", "/"),
                        "crc": crc,
                        "archive": archive,
                        "offset": offset,
                        "length": length,
                        "preload": preload_bytes,
                    }
                )
    return data_start, entries


def extract_entry(vpk: bytes, data_start: int, entry: dict) -> bytes:
    if entry["archive"] != EMBEDDED_ARCHIVE:
        raise ValueError(
            f"{entry['path']} лежит в отдельном архиве _{entry['archive']:03d}.vpk"
        )
    start = data_start + entry["offset"]
    chunk = vpk[start:start + entry["length"]]
    if len(chunk) != entry["length"]:
        raise ValueError(f"{entry['path']}: прочитано {len(chunk)} из {entry['length']}")
    if entry["preload"]:
        chunk = entry["preload"] + chunk
    return chunk


def skip_kv3(reader: Reader) -> dict:
    """Сдвигает поток за бинарный KV3, не разбирая дерево значений."""
    reader.align8()
    start = reader.pos
    magic = reader.u32()
    version = magic & 0xFF
    if (magic & 0xFFFFFF00) != 0x4B563300 or not 1 <= version <= 5:
        raise ValueError(f"ожидался KV3, magic={magic:#x} на {start}")
    reader.skip(16)  # guid формата
    compression = reader.u32()
    if compression not in COMPRESS:
        raise ValueError(f"неизвестное сжатие KV3: {compression}")

    if version == 1:
        reader.u32()
        reader.u32()
        reader.u32()
        size_uncompressed = reader.u32()
        size_compressed = reader.remaining()
        count_blocks = 0
        size_blobs = 0
        size_uncompressed_1 = size_uncompressed
        size_compressed_1 = size_compressed
        size_uncompressed_2 = 0
        size_compressed_2 = 0
    else:
        reader.u16()  # dictionary
        reader.u16()  # frame size
        reader.u32()  # count bytes1
        reader.u32()  # count bytes4
        reader.u32()  # count bytes8
        reader.u32()  # count types
        reader.u16()  # objects
        reader.u16()  # arrays
        reader.u32()  # uncompressed total
        size_compressed_total = reader.u32()
        count_blocks = reader.u32()
        size_blobs = reader.u32()
        if version >= 4:
            reader.u32()  # count bytes2
            reader.u32()  # block compressed sizes
        if version >= 5:
            size_uncompressed_1 = reader.u32()
            size_compressed_1 = reader.u32()
            size_uncompressed_2 = reader.u32()
            size_compressed_2 = reader.u32()
            reader.skip(4 * 8)  # counts buffer2 и два неизвестных поля v5
        else:
            size_uncompressed_1 = 0
            size_compressed_1 = size_compressed_total
            size_uncompressed_2 = 0
            size_compressed_2 = 0

    def payload(uncompressed: int, compressed: int) -> int:
        if compression == 0:
            return uncompressed if version >= 5 else compressed
        return compressed

    reader.skip(payload(size_uncompressed_1, size_compressed_1))
    if version >= 5:
        reader.skip(payload(size_uncompressed_2, size_compressed_2))
    if count_blocks:
        if compression == 0:
            reader.skip(size_blobs)
        elif compression == 2 and version >= 5:
            blobs_compressed = size_compressed_total - size_compressed_1 - size_compressed_2
            reader.skip(blobs_compressed)
        elif compression == 2:
            pass  # до v5 блобы уже внутри первого буфера
        else:
            raise ValueError("LZ4-блобы KV3 в этом файле не ожидались")
        trailer = reader.u32()
        if trailer != 0xFFEEDD00:
            raise ValueError(f"битый хвост KV3 {trailer:#x}")
    return {
        "version": version,
        "compression": COMPRESS[compression],
        "bytes": reader.pos - start,
    }


def read_connections(reader: Reader) -> list[dict]:
    count = reader.u32()
    return [{"area": reader.u32(), "edge": reader.u32()} for _ in range(count)]


def read_hull(reader: Reader, gen_version: int) -> dict:
    hull: dict = {}
    if gen_version >= 9:
        hull["enabled"] = reader.u8() > 0
    hull["radius"] = round(reader.f32(), 4)
    hull["height"] = round(reader.f32(), 4)
    if gen_version >= 9:
        hull["short_height_enabled"] = reader.u8() > 0
        hull["short_height"] = round(reader.f32(), 4)
    if gen_version >= 13:
        hull["crawl_enabled"] = reader.u8() > 0
        hull["crawl_height"] = round(reader.f32(), 4)
    hull["max_climb"] = round(reader.f32(), 4)
    hull["max_slope"] = reader.i32()
    hull["max_jump_down"] = round(reader.f32(), 4)
    hull["max_jump_horiz"] = round(reader.f32(), 4)
    hull["max_jump_up"] = round(reader.f32(), 4)
    if gen_version >= 11:
        hull["border_erosion"] = reader.i32()
    return hull


def parse_nav(blob: bytes) -> dict:
    reader = Reader(blob)
    magic = reader.u32()
    if magic != MAGIC_NAV:
        raise ValueError(f"это не .nav, magic={magic:#x}")
    version = reader.u32()
    if not 30 <= version <= 36:
        raise ValueError(f"версия nav {version} вне 30..36")
    subversion = reader.u32()
    analyzed = (reader.u32() & 1) != 0
    kv3 = []
    if version >= 36:
        kv3.append(skip_kv3(reader))

    polygons = []
    corner_count = 0
    if version >= 31:
        corner_count = reader.u32()
        if corner_count > 2_000_000:
            raise ValueError(f"слишком много углов ({corner_count}), KV3 пропущен неверно")
        corners = [reader.vec3() for _ in range(corner_count)]
        polygon_count = reader.u32()
        for _ in range(polygon_count):
            n = reader.u8()
            poly_corners = []
            for _corner in range(n):
                index = reader.u32()
                poly_corners.append(corners[index])
            movable = NO_MOVABLE_MESH
            if version >= 35:
                movable = reader.u32()
            polygons.append({"corners": poly_corners, "movable_mesh": movable})

    if version >= 32:
        unk2 = reader.u32()
        if unk2 != 0:
            raise ValueError(f"неожиданное поле v32: {unk2}")
    movable_meshes = []
    if version >= 35:
        for _ in range(reader.u32()):
            mesh_id = reader.cstr("ascii")
            reader.skip(48)
            movable_meshes.append(mesh_id)
    if version >= 36:
        kv3.append(skip_kv3(reader))

    areas = []
    area_count = reader.u32()
    for _ in range(area_count):
        area_id = reader.u32()
        flags = reader.i64()
        hull = reader.u8()
        if version >= 31:
            polygon = polygons[reader.u32()]
            area_corners = polygon["corners"]
            movable = polygon["movable_mesh"]
        else:
            area_corners = [reader.vec3() for _ in range(reader.u32())]
            movable = NO_MOVABLE_MESH
        reader.f32()
        connections = [read_connections(reader) for _ in area_corners]
        if reader.u8() != 0 or reader.u32() != 0:
            raise ValueError(f"зона {area_id}: ненулевые legacy-поля")
        above = [reader.u32() for _ in range(reader.u32())]
        below = [reader.u32() for _ in range(reader.u32())]
        areas.append(
            {
                "id": area_id,
                "hull": hull,
                "flags": flag_names(flags),
                "flags_raw": flags,
                "movable_mesh": movable,
                "corners": area_corners,
                "connections": connections,
                "ladders_above": above,
                "ladders_below": below,
            }
        )

    ladders = []
    known = {area["id"] for area in areas}
    for _ in range(reader.u32()):
        ladder_id = reader.u32()
        width = round(reader.f32(), 3)
        top = reader.vec3()
        bottom = reader.vec3()
        length = round(reader.f32(), 3)
        direction = reader.u32()
        links = [reader.u32() for _ in range(5)]
        if version >= 35:
            links.extend(reader.u32() for _ in range(2))
        names = [
            "top_forward",
            "top_left",
            "top_right",
            "top_behind",
            "bottom",
            "bottom_left",
            "bottom_right",
        ]

        def linked(area_id: int) -> int | None:
            return area_id if area_id in known else None

        ladders.append(
            {
                "id": ladder_id,
                "width": width,
                "length": length,
                "top": top,
                "bottom": bottom,
                "direction": DIRECTIONS.get(direction, str(direction)),
                "areas": {name: linked(area_id) for name, area_id in zip(names, links)},
            }
        )

    bounds = []
    for _ in range(reader.i32()):
        mins = reader.vec3()
        maxs = reader.vec3()
        matrix = [[reader.f32() for _ in range(4)] for _ in range(3)]
        bounds.append({"mins": mins, "maxs": maxs, "transform_3x4": matrix})

    gen_version = reader.i32()
    generation = {
        "nav_gen_version": gen_version,
        "use_project_defaults": reader.u32() != 0,
        "tile_size": round(reader.f32(), 4),
        "cell_size": round(reader.f32(), 4),
        "cell_height": round(reader.f32(), 4),
        "min_region_size": reader.i32(),
        "merged_region_size": reader.i32(),
        "mesh_sample_distance": round(reader.f32(), 4),
        "max_sample_error": round(reader.f32(), 4),
        "max_edge_length": reader.i32(),
        "max_edge_error": round(reader.f32(), 4),
        "verts_per_poly": reader.i32(),
    }
    if gen_version >= 7:
        generation["small_area_on_edge_removal"] = round(reader.f32(), 4)
    if gen_version >= 12:
        generation["hull_preset"] = reader.cstr()
        generation["hull_definitions"] = reader.cstr()
    hull_count = reader.i32()
    generation["hulls"] = [read_hull(reader, gen_version) for _ in range(hull_count)]
    if gen_version <= 11:
        for _ in range(hull_count, 3):
            read_hull(reader, gen_version)
    if gen_version >= 12:
        generation["gravity_follows_rotation"] = reader.u8() != 0
    if version >= 36:
        kv3.append(skip_kv3(reader))
    custom = None
    if subversion > 0:
        custom = skip_kv3(reader)
    if reader.pos != len(blob):
        raise ValueError(
            f"файл разобран не до конца: {reader.pos} из {len(blob)} байт"
        )

    xs = [c[0] for area in areas for c in area["corners"]]
    ys = [c[1] for area in areas for c in area["corners"]]
    zs = [c[2] for area in areas for c in area["corners"]]
    hulls: dict[int, int] = {}
    for area in areas:
        hulls[area["hull"]] = hulls.get(area["hull"], 0) + 1
    return {
        "version": version,
        "subversion": subversion,
        "analyzed": analyzed,
        "corners": corner_count,
        "polygons": len(polygons),
        "areas": len(areas),
        "areas_by_hull": {str(k): hulls[k] for k in sorted(hulls)},
        "ladders": len(ladders),
        "bounds": len(bounds),
        "movable_meshes": movable_meshes,
        "bbox": {
            "min": [min(xs), min(ys), min(zs)] if xs else None,
            "max": [max(xs), max(ys), max(zs)] if xs else None,
        },
        "kv3_blocks": kv3,
        "custom_kv3": custom,
        "generation": generation,
        "area_list": areas,
        "ladder_list": ladders,
        "bound_list": bounds,
    }


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Декомпиляция nav-сетки из VPK")
    parser.add_argument(
        "vpk",
        nargs="?",
        default=str(root / "data" / "de_cache.vpk"),
        help="путь к dir-архиву VPK",
    )
    parser.add_argument(
        "--entry",
        default="maps/de_cache.nav",
        help="файл внутри VPK",
    )
    parser.add_argument(
        "--out",
        default=str(Path(__file__).resolve().parent / "out"),
        help="папка для .nav и json",
    )
    args = parser.parse_args()
    vpk_path = Path(args.vpk)
    blob = vpk_path.read_bytes()
    data_start, entries = read_vpk_tree(blob)
    wanted = next((item for item in entries if item["path"] == args.entry), None)
    if wanted is None:
        navs = [item["path"] for item in entries if item["path"].endswith(".nav")]
        raise SystemExit(f"{args.entry} нет в архиве. nav-файлы: {navs}")
    nav = extract_entry(blob, data_start, wanted)
    parsed = parse_nav(nav)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    nav_path = out / Path(args.entry).name
    json_path = out / (Path(args.entry).name + ".json")
    nav_path.write_bytes(nav)
    summary = {key: parsed[key] for key in parsed if not key.endswith("_list")}
    summary["sample_areas"] = parsed["area_list"][:8]
    summary["sample_ladders"] = parsed["ladder_list"][:8]
    json_path.write_text(
        json.dumps(
            {
                "source": str(vpk_path),
                "entry": args.entry,
                "bytes": len(nav),
                **{key: parsed[key] for key in parsed if key != "area_list"},
                "areas": parsed["area_list"],
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"архив: {vpk_path}")
    print(f"файлов в каталоге: {len(entries)}")
    print(f"запись: {args.entry} ({len(nav)} байт) -> {nav_path}")
    print(f"json: {json_path}")
    print(
        f"nav v{parsed['version']}.{parsed['subversion']}, "
        f"разобран: {parsed['analyzed']}, "
        f"зон: {parsed['areas']}, полигонов: {parsed['polygons']}, "
        f"углов: {parsed['corners']}, лестниц: {parsed['ladders']}"
    )
    print(f"зоны по hull: {parsed['areas_by_hull']}")
    print(f"bbox min {parsed['bbox']['min']} max {parsed['bbox']['max']}")
    gen = parsed["generation"]
    print(
        f"генерация v{gen['nav_gen_version']}, клетка {gen['cell_size']}x{gen['cell_height']}, "
        f"hulls {len(gen['hulls'])}, preset {gen.get('hull_preset')}"
    )
    for index, hull in enumerate(gen["hulls"]):
        print(
            f"  hull {index}: radius {hull['radius']} height {hull['height']} "
            f"climb {hull['max_climb']} slope {hull['max_slope']}"
        )
    if parsed["ladder_list"]:
        ladder = parsed["ladder_list"][0]
        print(
            f"лестница #{ladder['id']}: ширина {ladder['width']} длина {ladder['length']} "
            f"верх {ladder['top']} низ {ladder['bottom']} {ladder['direction']}"
        )
    area = parsed["area_list"][0]
    print(
        f"зона #{area['id']} hull {area['hull']} флаги {area['flags']} "
        f"углы {area['corners']}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ошибка: {exc}", file=sys.stderr)
        raise
