"""Live validation and runtime fixes for enemy-related memory layout."""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

from cs2_reader.aim import BONE_ARRAY_IN_MODEL_STATE, BONE_HEAD, BONE_STRIDE, head_plausible
from cs2_reader.offsets import (
    ENTITY_LIST_CHUNK_OFFSET,
    ENTITY_LIST_CHUNK_OFFSET_WINDOWS,
    ENTITY_LIST_ENTRY_SIZE,
    MAX_PLAYERS,
)
from cs2_reader.platform import Platform, schema_file
from cs2_reader.schema import Schema

if TYPE_CHECKING:
    from cs2_reader.reader import Reader

ROOT = Path(__file__).resolve().parents[2]
BONE_ARRAY_CANDIDATES = tuple(range(0x70, 0xC0, 8))


def schema_drift_messages(schema: Schema) -> list[str]:
    """Compare loaded schema with data/libclient.so.json (enemy-critical fields)."""
    path = schema_file()
    if not path.is_file():
        return [f"schema file missing: {path}"]
    raw = json.loads(path.read_text(encoding="utf-8"))
    key = "libclient.so" if "libclient.so" in raw else "client.dll"
    classes = raw[key]["classes"]

    def dump(cls: str, fld: str) -> int:
        return classes[cls]["fields"][fld]

    checks = [
        ("m_h_player_pawn", "CCSPlayerController", "m_hPlayerPawn", schema.m_h_player_pawn),
        ("m_entity_spotted_state", "C_CSPlayerPawn", "m_entitySpottedState", schema.m_entity_spotted_state),
        ("m_b_spotted_by_mask", "EntitySpottedState_t", "m_bSpottedByMask", schema.m_b_spotted_by_mask),
        ("m_v_old_origin", "C_BasePlayerPawn", "m_vOldOrigin", schema.m_v_old_origin),
        ("m_model_state", "CSkeletonInstance", "m_modelState", schema.m_model_state),
        ("m_i_health", "C_BaseEntity", "m_iHealth", schema.m_i_health),
        ("m_ang_eye_angles", "C_CSPlayerPawn", "m_angEyeAngles", schema.m_ang_eye_angles),
    ]
    out: list[str] = []
    for label, cls, fld, loaded in checks:
        try:
            expected = dump(cls, fld)
        except KeyError:
            continue
        if expected != loaded:
            out.append(f"{label}: schema={loaded} dump={expected}")
    return out


def _count_player_controllers(reader: Reader, chunk_off: int) -> int:
    elist = reader.entity_list()
    if not elist:
        return 0
    backend = reader.backend
    schema = reader.schema
    ok = 0
    for i in range(1, MAX_PLAYERS):
        chunk = (i & 0x7FFF) >> 9
        slot = i & 0x1FF
        list_entry = backend.read_u64(elist + 8 * chunk + chunk_off)
        if not list_entry:
            continue
        controller = backend.read_u64(list_entry + ENTITY_LIST_ENTRY_SIZE * slot)
        if not controller or controller < 0x10000:
            continue
        pawn_h = backend.read_u32(controller + schema.m_h_player_pawn)
        if pawn_h and pawn_h != 0xFFFFFFFF:
            ok += 1
    return ok


def pick_entity_list_chunk(reader: Reader) -> int | None:
    """Return best chunk offset if current one resolves fewer controllers."""
    platform = reader._platform
    current = reader._entity_chunk_off
    if platform == Platform.LINUX_NATIVE:
        candidates = (ENTITY_LIST_CHUNK_OFFSET, ENTITY_LIST_CHUNK_OFFSET_WINDOWS)
    else:
        candidates = (ENTITY_LIST_CHUNK_OFFSET_WINDOWS, ENTITY_LIST_CHUNK_OFFSET)
    scores = {off: _count_player_controllers(reader, off) for off in candidates}
    best = max(scores, key=lambda k: scores[k])
    if scores[best] <= scores.get(current, 0):
        return None
    return best


def pick_bone_array_offset(reader: Reader) -> int | None:
    """Scan CSkeletonInstance.m_modelState for a valid head bone array pointer."""
    pawn = reader.local_pawn()
    if not pawn:
        return None
    backend = reader.backend
    schema = reader.schema
    feet_v = backend.read_vec3(pawn + schema.m_v_old_origin)
    if not feet_v:
        return None
    from cs2_reader.game_state import Vec3

    feet = Vec3(*feet_v)
    scene = backend.read_u64(pawn + schema.m_p_game_scene_node)
    if not scene:
        return None
    current = getattr(reader, "_bone_array_in_model_state", BONE_ARRAY_IN_MODEL_STATE)

    def score(off: int) -> int:
        bone_array = backend.read_u64(scene + schema.m_model_state + off)
        if not bone_array or bone_array < 0x10000:
            return 0
        coords = backend.read_vec3(bone_array + BONE_HEAD * BONE_STRIDE)
        if not coords:
            return 0
        head = Vec3(*coords)
        return 2 if head_plausible(feet, head) else 0

    if score(current) >= 2:
        return None
    best_off = max(BONE_ARRAY_CANDIDATES, key=score)
    return best_off if score(best_off) >= 2 else None


def reload_schema_from_disk() -> Schema:
    return Schema.load()


def run_offset_dump() -> bool:
    script = ROOT / "scripts" / "update_offsets.py"
    if not script.is_file():
        return False
    try:
        subprocess.run(
            [sys.executable, str(script)],
            cwd=str(ROOT),
            check=True,
            capture_output=True,
            text=True,
            timeout=180,
        )
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
        return False


def calibrate_enemy_offsets(
    reader: Reader,
    *,
    allow_dump: bool = True,
    min_interval_s: float = 30.0,
) -> list[str]:
    """
    Validate enemy reads; fix entity-list chunk / bone array at runtime.
    Optionally refresh data/*.json from cs2-dumper when schema drift is detected.
    """
    now = time.monotonic()
    last = getattr(reader, "_enemy_calibrate_at", 0.0)
    if now - last < min_interval_s:
        return []
    reader._enemy_calibrate_at = now

    log: list[str] = []
    drift = schema_drift_messages(reader.schema)
    if drift and allow_dump:
        if run_offset_dump():
            from cs2_reader.profile_rank import ProfileRankReader

            reader.schema = reload_schema_from_disk()
            reader._profile_rank_reader = ProfileRankReader(reader.backend, reader.schema)
            log.append("refreshed data/*.json from live cs2-dumper")
            drift = schema_drift_messages(reader.schema)
    for line in drift:
        log.append(f"schema drift: {line}")

    chunk = pick_entity_list_chunk(reader)
    if chunk is not None:
        reader._entity_chunk_off = chunk
        log.append(f"entity list chunk offset -> 0x{chunk:x}")

    bone_off = pick_bone_array_offset(reader)
    if bone_off is not None:
        reader._bone_array_in_model_state = bone_off
        log.append(f"bone array in modelState -> 0x{bone_off:x}")

    if reader.local_pawn() and not drift and not chunk and not bone_off:
        n = len([p for p in reader.players() if p.health > 0 and p.health <= 100])
        if n == 0 and allow_dump:
            if run_offset_dump():
                reader.schema = reload_schema_from_disk()
                log.append("no players with sane HP — re-dumped offsets")
    return log
