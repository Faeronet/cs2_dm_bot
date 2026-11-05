"""Live validation and runtime fixes for enemy-related memory layout."""

from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

from cs2_reader.aim import BONE_ARRAY_IN_MODEL_STATE, BONE_HEAD, BONE_STRIDE, head_plausible
from cs2_reader.visibility import mask_has_bit, read_radar_spotted, read_spotted_mask, spotted_by_viewer
from cs2_reader.offsets import (
    ENTITY_LIST_CHUNK_OFFSET,
    ENTITY_LIST_CHUNK_OFFSET_WINDOWS,
    ENTITY_LIST_ENTRY_SIZE,
    MAX_PLAYERS,
)
from cs2_reader.platform import Platform, schema_file
from cs2_reader.runtime_tweaks import apply_tweaks, save_tweaks
from cs2_reader.schema import Schema

if TYPE_CHECKING:
    from cs2_reader.reader import Reader

ROOT = Path(__file__).resolve().parents[2]
BONE_ARRAY_CANDIDATES = tuple(range(0x70, 0xC0, 8))
AUDIT_INTERVAL_S = 2.0         # с: как часто проверять чтение врагов (не на каждом players())
ISSUE_RECALIBRATE_S = 5.0      # с: при найденной проблеме — перекалибровка не чаще этого


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


def _visibility_sample(reader: Reader, pawn: int, viewer_idx: int) -> dict:
    s = reader.schema
    state = pawn + s.m_entity_spotted_state
    m0, m1 = read_spotted_mask(reader.backend, state, s.m_b_spotted_by_mask)
    bit = max(0, viewer_idx - 1)
    return {
        "m0": m0,
        "m1": m1,
        "bit": bit,
        "spotted": spotted_by_viewer(
            reader.backend, pawn, s.m_entity_spotted_state, s.m_b_spotted_by_mask, viewer_idx
        ),
        "radar": read_radar_spotted(
            reader.backend, pawn, s.m_entity_spotted_state, s.m_b_spotted
        ),
        "mask_any": m0 != 0 or m1 != 0,
    }


def audit_visibility_reads(reader: Reader) -> list[str]:
    """Состояние spotted/radar и маски для ближних целей."""
    issues: list[str] = []
    li = reader.local_index()
    if li <= 0:
        issues.append("local_index<=0 — бит spottedByMask может быть неверным")
    enemies = [p for p in reader.players(skip_calibrate=True) if not p.is_local and p.alive and p.team > 1]
    if not enemies:
        return issues
    near = sorted(enemies, key=lambda p: p.distance)[:6]
    vis = sum(1 for p in near if p.spotted)
    rad = sum(1 for p in near if reader.is_radar_spotted(p.pawn))
    if len(near) >= 4 and vis == 0 and rad == 0:
        any_mask = False
        checked = 0
        for p in near:
            if p.dormant or p.distance > 1200:
                continue
            checked += 1
            sm = _visibility_sample(reader, p.pawn, li)
            if sm["mask_any"] or sm["radar"]:
                any_mask = True
                break
        if checked >= 3 and not any_mask and li > 0:
            issues.append("ближние (<1200u): spotted/radar/mask=0 — проверь m_entitySpottedState")
    # Альтернативный бит viewer (index без -1) — только диагностика
    if li > 0 and vis == 0:
        alt_hits = 0
        for p in near[:4]:
            if p.dormant:
                continue
            sm = _visibility_sample(reader, p.pawn, li)
            m0, m1 = sm["m0"], sm["m1"]
            if mask_has_bit(m0, m1, li) and not mask_has_bit(m0, m1, li - 1):
                alt_hits += 1
        if alt_hits >= 2:
            issues.append("spotted: возможно бит viewer=index (не index-1)")
    return issues


def audit_enemy_reads(reader: Reader) -> list[str]:
    """Быстрая эвристика: битые HP/кости/пустой entity list."""
    issues: list[str] = []
    if not reader.local_pawn():
        return issues
    players = reader.players(skip_calibrate=True)
    sane = [p for p in players if 0 < p.health <= 100 and p.team > 1]
    if len(players) >= 3 and len(sane) == 0:
        issues.append("нет игроков с нормальным HP при непустом списке")
    bad_hp = sum(1 for p in players if p.health > 100 or p.health < -1000)
    if bad_hp:
        issues.append(f"мусорный HP у {bad_hp} игроков")
    bad_head = 0
    for p in sane[:12]:
        if p.is_local or p.dormant:
            continue
        dz = p.head_position.z - p.position.z
        if dz < 25.0 or dz > 95.0:
            bad_head += 1
    if bad_head >= 3:
        issues.append(f"head_dz вне нормы у {bad_head} целей (кости/modelState?)")
    ctrl_n = _count_player_controllers(reader, reader._entity_chunk_off)
    if ctrl_n == 0 and reader.local_pawn():
        issues.append("entity list: 0 controllers")
    issues.extend(audit_visibility_reads(reader))
    return issues


def calibrate_enemy_offsets(
    reader: Reader,
    *,
    allow_dump: bool = True,
    min_interval_s: float = 30.0,
    force: bool = False,
) -> list[str]:
    """
    Validate enemy reads; fix entity-list chunk / bone array at runtime.
    Optionally refresh data/*.json from cs2-dumper when schema drift is detected.
    """
    now = time.monotonic()
    last = getattr(reader, "_enemy_calibrate_at", 0.0)
    if not force:
        # Аудит сам читает всех игроков дважды. players() зовёт эту функцию на каждом вызове,
        # фоновый скан — до 600 раз в секунду: без паузы аудит съедал процессор,
        # и тик навигатора растягивался с 25 мс до ~0.7 с (камера шла рывками)
        if now - getattr(reader, "_enemy_audit_at", 0.0) < AUDIT_INTERVAL_S:
            return []
        reader._enemy_audit_at = now
    apply_tweaks(reader)
    issues = audit_enemy_reads(reader)
    if not force and now - last < (ISSUE_RECALIBRATE_S if issues else min_interval_s):
        return []
    reader._enemy_calibrate_at = now

    log: list[str] = []
    for msg in issues:
        log.append(f"audit: {msg}")
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
        save_tweaks(entity_chunk_off=chunk)
        log.append(f"entity list chunk offset -> 0x{chunk:x}")

    bone_off = pick_bone_array_offset(reader)
    if bone_off is not None:
        reader._bone_array_in_model_state = bone_off
        save_tweaks(bone_array_in_model_state=bone_off)
        log.append(f"bone array in modelState -> 0x{bone_off:x}")

    if reader.local_pawn() and not drift and not chunk and not bone_off:
        n = len([p for p in reader.players() if p.health > 0 and p.health <= 100])
        if n == 0 and allow_dump:
            if run_offset_dump():
                reader.schema = reload_schema_from_disk()
                log.append("no players with sane HP — re-dumped offsets")
    return log
