"""Live checks for match-detection offsets (pawn, global vars, entity list)."""

from __future__ import annotations

import re
import subprocess
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

from cs2_reader.life import LIFE_ALIVE
from cs2_reader.runtime_tweaks import apply_tweaks, save_tweaks
from cs2_reader.offsets import (
    ENTITY_LIST_CHUNK_OFFSET,
    ENTITY_LIST_CHUNK_OFFSET_WINDOWS,
    ENTITY_LIST_ENTRY_SIZE,
    GLOBAL_VARS_MAP_NAME,
    GLOBAL_VARS_MAP_NAME_WINDOWS,
    MAX_PLAYERS,
)

if TYPE_CHECKING:
    from cs2_reader.reader import Reader

ROOT = Path(__file__).resolve().parents[2]
_MAP_ID = re.compile(r"^(?:de|cs|ar|dz|gd)_[a-z0-9_]+$")
MAX_PTR = 0x00007FFFFFFFFFFF


def _usable_ptr(ptr: int | None) -> bool:
    return bool(ptr and 0x10000 < ptr <= MAX_PTR)


def _sane_health(hp: int | None) -> bool:
    return hp is not None and 0 <= hp <= 100


def _count_controllers(reader: Reader, chunk_off: int) -> int:
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


def _pawn_at_client_off(reader: Reader, off: int) -> int | None:
    client = reader._client_base()
    if not client:
        return None
    pawn = reader.backend.read_u64(client + off)
    if not _usable_ptr(pawn):
        return None
    schema = reader.schema
    hp = reader.backend.read_i32(pawn + schema.m_i_health)
    team = reader.backend.read_i32(pawn + schema.m_i_team_num)
    if not _sane_health(hp) or team not in (2, 3):
        return None
    return pawn


def scan_local_pawn_offset(reader: Reader) -> int | None:
    """cs2-dumper иногда промахивается — ищем указатель на живой pawn."""
    base = reader.offsets.client.dw_local_player_pawn
    for delta in range(-0x3000, 0x3001, 8):
        off = base + delta
        if _pawn_at_client_off(reader, off):
            return off
    return None


def scan_global_vars_offset(reader: Reader) -> tuple[int, int] | None:
    """Вернуть (client_off, map_field_off) если нашли CGlobalVars с de_*."""
    client = reader._client_base()
    if not client:
        return None
    base = reader.offsets.client.dw_global_vars
    map_offs = (
        reader._map_name_off,
        GLOBAL_VARS_MAP_NAME,
        GLOBAL_VARS_MAP_NAME_WINDOWS,
        0x170,
        0x178,
        0x180,
        0x188,
        0x190,
        0x198,
        0x1A0,
        0x1B8,
    )
    for delta in range(-0x400, 0x401, 8):
        off = base + delta
        gv = reader.backend.read_u64(client + off)
        if not _usable_ptr(gv):
            continue
        inner = reader.backend.read_u64(gv)
        bases = [gv]
        if _usable_ptr(inner):
            bases.append(inner)
        for gvb in bases:
            seen: set[int] = set()
            for mo in map_offs:
                if mo in seen:
                    continue
                seen.add(mo)
                raw = reader.backend.read_cstring_ptr(gvb + mo)
                if raw and _MAP_ID.match(raw.strip().lower()):
                    return off, mo
    return None


def scan_local_controller_offset(reader: Reader, pawn: int) -> int | None:
    """Подобрать dwLocalPlayerController, если pawn уже найден."""
    client = reader._client_base()
    if not client or not pawn:
        return None
    schema = reader.schema
    base = reader.offsets.client.dw_local_player_controller
    for delta in range(-0x3000, 0x3001, 8):
        off = base + delta
        ctrl = reader.backend.read_u64(client + off)
        if not _usable_ptr(ctrl):
            continue
        resolved = reader.pawn_from_controller(ctrl)
        if resolved == pawn:
            return off
        h = reader.backend.read_u32(ctrl + schema.m_h_player_pawn)
        if h and h != 0xFFFFFFFF:
            idx = h & 0x7FFF
            ent = reader.entity_by_index(idx)
            if ent == pawn:
                return off
    return None


def calibrate_match_offsets(
    reader: Reader,
    *,
    allow_dump: bool = True,
    min_interval_s: float = 45.0,
    force: bool = False,
) -> list[str]:
    """Fix entity-list chunk; scan pawn/globalvars; optional cs2-dumper."""
    apply_tweaks(reader)
    pawn = reader.local_pawn()
    broken = not _usable_ptr(pawn) or not _sane_health(
        reader.backend.read_i32(pawn + reader.schema.m_i_health) if pawn else None
    )
    now = time.monotonic()
    last = getattr(reader, "_match_calibrate_at", 0.0)
    if not force and not broken and now - last < min_interval_s:
        return []
    reader._match_calibrate_at = now

    log: list[str] = []
    from cs2_reader.platform import Platform

    platform = reader._platform
    if platform == Platform.LINUX_NATIVE:
        candidates = (ENTITY_LIST_CHUNK_OFFSET, ENTITY_LIST_CHUNK_OFFSET_WINDOWS)
    else:
        candidates = (ENTITY_LIST_CHUNK_OFFSET_WINDOWS, ENTITY_LIST_CHUNK_OFFSET)
    scores = {off: _count_controllers(reader, off) for off in candidates}
    best = max(scores, key=lambda k: scores[k])
    if scores[best] > scores.get(reader._entity_chunk_off, 0):
        reader._entity_chunk_off = best
        log.append(f"entity list chunk -> 0x{best:x} ({scores[best]} controllers)")

    pawn = reader.local_pawn()
    if not _usable_ptr(pawn):
        lp_off = scan_local_pawn_offset(reader)
        if lp_off is not None:
            reader._dw_local_player_pawn = lp_off
            save_tweaks(dw_local_player_pawn=lp_off)
            pawn = reader.local_pawn()
            log.append(f"dwLocalPlayerPawn -> client+0x{lp_off:x}")

    gv = reader._global_vars_base()
    if not reader.map_name() or not _usable_ptr(gv) or gv <= 0x1000:
        hit = scan_global_vars_offset(reader)
        if hit is not None:
            off, map_off = hit
            reader._dw_global_vars = off
            reader._map_name_off = map_off
            save_tweaks(dw_global_vars=off, map_name_off=map_off)
            log.append(f"dwGlobalVars -> client+0x{off:x} (map field 0x{map_off:x})")

    pawn = reader.local_pawn()
    ctrl = reader.local_controller()
    if pawn and not _usable_ptr(ctrl):
        co = scan_local_controller_offset(reader, pawn)
        if co is not None:
            reader._dw_local_player_controller = co
            save_tweaks(dw_local_player_controller=co)
            log.append(f"dwLocalPlayerController -> client+0x{co:x}")

    pawn = reader.local_pawn()
    ctrl = reader.local_controller()
    if ctrl and not pawn:
        log.append("controller ok, pawn=0 — проверь m_hPlayerPawn / entity list")

    if pawn:
        hp = reader.backend.read_i32(pawn + reader.schema.m_i_health)
        ls = reader.backend.read_u8(pawn + reader.schema.m_life_state)
        if not _sane_health(hp) and ls != LIFE_ALIVE:
            log.append(f"pawn hp/ls подозрительны (hp={hp} ls={ls})")

    need_dump = allow_dump and broken and not _usable_ptr(reader.local_pawn())
    if need_dump:
        script = ROOT / "scripts" / "update_offsets.py"
        if script.is_file():
            try:
                subprocess.run(
                    [sys.executable, str(script)],
                    cwd=str(ROOT),
                    check=True,
                    capture_output=True,
                    text=True,
                    timeout=180,
                )
                from cs2_reader.offsets import Offsets
                from cs2_reader.schema import Schema

                reader.offsets = Offsets.load()
                reader.schema = Schema.load()
                reader._dw_local_player_pawn = reader.offsets.client.dw_local_player_pawn
                reader._dw_local_player_controller = reader.offsets.client.dw_local_player_controller
                reader._dw_global_vars = reader.offsets.client.dw_global_vars
                log.append("cs2-dumper: data/ обновлены, повторный scan…")
                return log + calibrate_match_offsets(reader, allow_dump=False, force=True)
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as exc:
                log.append(f"cs2-dumper не удался: {exc}")

    return log
