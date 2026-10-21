#!/usr/bin/env python3
"""Монитор врагов из памяти CS2 — для отладки драйвера рядом с ботом.

Классы (на каждого игрока):
  VISIBLE   — в списке enemies(), LOS: хотя бы одна кость не за стеной
  WALL      — в списке, координаты свежие, но все кости за стеной (off-screen OK)
  DORMANT   — pawn помечен m_bDormant (координаты могут быть устаревшими)
  UNTRACKED — слот контроллера занят, но read_player/enemies не вернули игрока
  ABSENT    — слот пуст (контроллера нет)

Пишет снимок в /tmp/cs2_enemy_watch.json и лог изменений в /tmp/cs2_enemy_watch.log.
Запуск параллельно с run_dm.py в отдельном терминале.

  python3 python/tools/watch_enemies.py
  python3 python/tools/watch_enemies.py --hz 20 --log /tmp/cs2_enemy_watch.log
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass, field

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_nav.coord_map import CoordinateMap
from cs2_nav.los_visibility import apply_los_visibility
from cs2_reader.life import LIFE_ALIVE
from cs2_reader.offsets import MAX_PLAYERS
from cs2_reader.reader import Reader

SNAPSHOT_PATH = "/tmp/cs2_enemy_watch.json"
DEFAULT_LOG = "/tmp/cs2_enemy_watch.log"

# Статусы для человека и для diff
STATUS_VISIBLE = "VISIBLE"
STATUS_WALL = "WALL"
STATUS_DORMANT = "DORMANT"
STATUS_UNTRACKED = "UNTRACKED"
STATUS_ABSENT = "ABSENT"


@dataclass
class SlotRow:
    index: int
    status: str
    name: str = ""
    team: int = 0
    health: int = 0
    dormant: bool = False
    engine_spotted: bool = False
    radar_spotted: bool = False
    distance_m: float = 0.0
    bones_los: str = ""          # H/C/L/R видимые кости
    ctrl: int = 0
    pawn: int = 0
    life_state: int = -1
    reason: str = ""             # почему UNTRACKED


@dataclass
class WatchSnapshot:
    ts: float
    map_name: str
    local_hp: int
    local_pos: tuple[float, float, float]
    poll_hz: float
    slots: list[SlotRow] = field(default_factory=list)
    summary: dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "ts": self.ts,
            "map": self.map_name,
            "local_hp": self.local_hp,
            "local_pos": self.local_pos,
            "poll_hz": self.poll_hz,
            "summary": self.summary,
            "slots": [asdict(s) for s in self.slots if s.status != STATUS_ABSENT],
            "all_slots": [asdict(s) for s in self.slots],
        }


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def _nav_path(map_name: str) -> str | None:
    path = os.path.join(_repo_root(), "data", "maps", f"{map_name}.nav.json")
    return path if os.path.isfile(path) else None


def _bone_txt(d: dict) -> str:
    parts = []
    for ch, key in (("H", "head_visible"), ("C", "chest_visible"), ("L", "left_shoulder_visible"), ("R", "right_shoulder_visible")):
        if d.get(key):
            parts.append(ch)
    return "".join(parts) or "-"


def _scan_slots(reader: Reader, cmap: CoordinateMap | None) -> list[SlotRow]:
    schema = reader.schema
    b = reader.backend
    local_idx = reader.local_index()
    local_pos = reader.local_position()
    feet_z = local_pos.z
    eye_z = reader.local_view_offset_z() if callable(getattr(reader, "local_view_offset_z", None)) else 64.0
    hull = 16.0
    body = 20.0

    enemy_by_idx = {e.index: e for e in reader.enemies()}
    rows: list[SlotRow] = []

    for i in range(1, MAX_PLAYERS):
        if i == local_idx:
            continue
        ctrl = reader.entity_by_index(i)
        if not ctrl:
            rows.append(SlotRow(index=i, status=STATUS_ABSENT))
            continue

        pawn = reader.pawn_from_controller(ctrl) or 0
        name_ptr = b.read_u64(ctrl + schema.m_s_sanitized_player_name)
        name = b.read_string(name_ptr, 64) if name_ptr else ""

        if not pawn:
            rows.append(SlotRow(
                index=i, status=STATUS_UNTRACKED, name=name or "?",
                ctrl=ctrl, reason="controller без pawn",
            ))
            continue

        health = b.read_i32(pawn + schema.m_i_health) or 0
        life = b.read_u8(pawn + schema.m_life_state)
        life = life if life is not None else -1
        team = b.read_i32(pawn + schema.m_i_team_num) or 0
        scene = b.read_u64(pawn + schema.m_p_game_scene_node)
        dormant = bool(b.read_u8(scene + schema.m_b_dormant)) if scene else False

        ent = enemy_by_idx.get(i)
        if ent is None:
            why = []
            if health <= 0:
                why.append(f"hp={health}")
            if life != LIFE_ALIVE:
                why.append(f"life={life}")
            if team <= 1:
                why.append(f"team={team}")
            rows.append(SlotRow(
                index=i, status=STATUS_UNTRACKED,
                name=name or "?", team=team, health=health,
                dormant=dormant, ctrl=ctrl, pawn=pawn, life_state=life,
                reason="; ".join(why) or "не прошёл фильтр enemies()",
            ))
            continue

        d = {
            "index": ent.index,
            "name": ent.name,
            "dormant": ent.dormant,
            "spotted": ent.spotted,
            "position": (ent.position.x, ent.position.y, ent.position.z),
            "head_position": (ent.head_position.x, ent.head_position.y, ent.head_position.z),
            "chest_position": (ent.chest_position.x, ent.chest_position.y, ent.chest_position.z),
            "left_shoulder_position": (
                ent.left_shoulder_position.x, ent.left_shoulder_position.y, ent.left_shoulder_position.z,
            ),
            "right_shoulder_position": (
                ent.right_shoulder_position.x, ent.right_shoulder_position.y, ent.right_shoulder_position.z,
            ),
            "visible": False,
        }
        if cmap and not ent.dormant:
            apply_los_visibility(
                d, cmap, local_pos.x, local_pos.y, feet_z,
                eye_z=eye_z, hull_radius=hull, target_body_radius=body,
            )

        if ent.dormant:
            status = STATUS_DORMANT
        elif d.get("visible"):
            status = STATUS_VISIBLE
        else:
            status = STATUS_WALL

        rows.append(SlotRow(
            index=i,
            status=status,
            name=ent.name or name or "?",
            team=team,
            health=ent.health,
            dormant=ent.dormant,
            engine_spotted=ent.spotted,
            radar_spotted=ent.radar_spotted,
            distance_m=round(ent.distance / 39.37, 1),
            bones_los=_bone_txt(d),
            ctrl=ctrl,
            pawn=pawn,
            life_state=life,
        ))

    return rows


def _snapshot(reader: Reader, cmap: CoordinateMap | None, poll_hz: float) -> WatchSnapshot:
    life = reader.read_local_life()
    pos = reader.local_position()
    slots = _scan_slots(reader, cmap)
    summary: dict[str, int] = {}
    for s in slots:
        summary[s.status] = summary.get(s.status, 0) + 1
    return WatchSnapshot(
        ts=time.time(),
        map_name=reader.map_name() or "",
        local_hp=life.health,
        local_pos=(round(pos.x, 1), round(pos.y, 1), round(pos.z, 1)),
        poll_hz=poll_hz,
        slots=slots,
        summary=summary,
    )


def _state_key(snap: WatchSnapshot) -> str:
    parts = []
    for row in sorted(snap.slots, key=lambda r: r.index):
        if row.status == STATUS_ABSENT:
            continue
        parts.append(f"{row.index}:{row.status}:{row.bones_los}:{row.health}:{row.dormant}")
    return "|".join(parts)


def _print_snap(snap: WatchSnapshot) -> None:
    vis = snap.summary.get(STATUS_VISIBLE, 0)
    wall = snap.summary.get(STATUS_WALL, 0)
    dorm = snap.summary.get(STATUS_DORMANT, 0)
    unt = snap.summary.get(STATUS_UNTRACKED, 0)
    print(
        f"[watch] map={snap.map_name or '?'} hp={snap.local_hp} "
        f"VIS={vis} WALL={wall} DORM={dorm} UNTRACKED={unt}  pos={snap.local_pos}",
        flush=True,
    )
    for row in sorted(snap.slots, key=lambda r: r.distance_m if r.distance_m else 9999):
        if row.status == STATUS_ABSENT:
            continue
        extra = f" reason={row.reason}" if row.reason else ""
        print(
            f"  [{row.status:9}] #{row.index:2} {row.name:14} "
            f"~{row.distance_m:5.1f}m hp={row.health:3} dorm={row.dormant} "
            f"eng={row.engine_spotted} bones={row.bones_los}{extra}",
            flush=True,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Монитор врагов из памяти (рядом с ботом)")
    parser.add_argument("--hz", type=float, default=20.0, help="Частота вывода/снимка (Гц)")
    parser.add_argument("--log", default=DEFAULT_LOG, help="Лог изменений состояния")
    parser.add_argument("--snapshot", default=SNAPSHOT_PATH, help="JSON-снимок для агента")
    parser.add_argument("--quiet", action="store_true", help="Не печатать в терминал, только файлы")
    args = parser.parse_args()

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    interval = 1.0 / max(1.0, args.hz)
    cmap: CoordinateMap | None = None
    cmap_map = ""
    prev_key = ""
    n = 0

    print(f"[watch] монитор врагов — снимок {args.snapshot}, лог {args.log}", flush=True)
    print(f"[watch] hz={args.hz:.0f}  (запусти бота в другом терминале)", flush=True)

    try:
        while True:
            t0 = time.monotonic()
            map_name = reader.map_name() or ""
            if map_name and map_name != cmap_map:
                nav = _nav_path(map_name)
                if nav:
                    print(f"[watch] навмеш {map_name}…", flush=True)
                    cmap = CoordinateMap.from_navmesh(nav)
                    cmap_map = map_name
                else:
                    print(f"[watch] навмеша нет: {map_name}", flush=True)
                    cmap = None
                    cmap_map = map_name

            snap = _snapshot(reader, cmap, args.hz)
            key = _state_key(snap)
            n += 1

            with open(args.snapshot, "w", encoding="utf-8") as fh:
                json.dump(snap.to_dict(), fh, ensure_ascii=False, indent=2)

            if key != prev_key:
                prev_key = key
                line = json.dumps(snap.to_dict(), ensure_ascii=False)
                with open(args.log, "a", encoding="utf-8") as fh:
                    fh.write(line + "\n")
                if not args.quiet:
                    _print_snap(snap)
            elif not args.quiet and n % max(1, int(args.hz * 5)) == 0:
                vis = snap.summary.get(STATUS_VISIBLE, 0)
                wall = snap.summary.get(STATUS_WALL, 0)
                print(f"[watch] … stable VIS={vis} WALL={wall} hp={snap.local_hp}", flush=True)

            dt = time.monotonic() - t0
            sleep = interval - dt
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\n[watch] стоп", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
