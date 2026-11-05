"""Live checks for match-detection offsets (pawn, global vars, entity list)."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING

from cs2_reader.life import LIFE_ALIVE
from cs2_reader.offsets import (
    ENTITY_LIST_CHUNK_OFFSET,
    ENTITY_LIST_CHUNK_OFFSET_WINDOWS,
    ENTITY_LIST_ENTRY_SIZE,
    MAX_PLAYERS,
)

if TYPE_CHECKING:
    from cs2_reader.reader import Reader

ROOT = Path(__file__).resolve().parents[2]


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


def calibrate_match_offsets(
    reader: Reader,
    *,
    allow_dump: bool = True,
    min_interval_s: float = 45.0,
) -> list[str]:
    """Fix entity-list chunk; re-dump data/ if pawn/health reads look broken."""
    now = time.monotonic()
    last = getattr(reader, "_match_calibrate_at", 0.0)
    if now - last < min_interval_s:
        return []
    reader._match_calibrate_at = now

    log: list[str] = []
    platform = reader._platform
    from cs2_reader.platform import Platform

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
    ctrl = reader.local_controller()
    if ctrl and not pawn:
        log.append("local pawn=0 but controller ok — проверь dwLocalPlayerPawn / m_hPlayerPawn")

    if pawn:
        hp = reader.backend.read_i32(pawn + reader.schema.m_i_health)
        ls = reader.backend.read_u8(pawn + reader.schema.m_life_state)
        if not _sane_health(hp) and ls != LIFE_ALIVE:
            log.append(f"pawn health/life подозрительны (hp={hp} ls={ls})")

    need_dump = allow_dump and (
        (ctrl and not pawn)
        or (pawn and not _sane_health(reader.backend.read_i32(pawn + reader.schema.m_i_health)))
    )
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
                log.append("перезалиты data/offsets.json и libclient.so.json (cs2-dumper)")
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError) as exc:
                log.append(f"cs2-dumper не удался: {exc}")

    return log
