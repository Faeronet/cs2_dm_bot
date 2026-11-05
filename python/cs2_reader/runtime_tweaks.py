"""Cross-process runtime offset tweaks (bot + watch tools share via data/)."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from cs2_reader.reader import Reader

ROOT = Path(__file__).resolve().parents[2]
TWEAKS_PATH = ROOT / "data" / "runtime_tweaks.json"


def _read_raw() -> dict:
    if not TWEAKS_PATH.is_file():
        return {}
    try:
        return json.loads(TWEAKS_PATH.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return {}


def save_tweaks(**fields: int) -> None:
    raw = _read_raw()
    raw.update({k: v for k, v in fields.items() if v is not None})
    raw["updated_at"] = time.time()
    TWEAKS_PATH.parent.mkdir(parents=True, exist_ok=True)
    TWEAKS_PATH.write_text(json.dumps(raw, indent=2), encoding="utf-8")


def apply_tweaks(reader: Reader) -> list[str]:
    # сначала mtime: файл меняется редко, а зовут нас на каждом чтении врагов
    try:
        mtime = TWEAKS_PATH.stat().st_mtime
    except OSError:
        return []
    if mtime <= getattr(reader, "_tweaks_mtime", 0.0):
        return []
    reader._tweaks_mtime = mtime
    raw = _read_raw()
    if not raw:
        return []
    applied: list[str] = []

    mapping = {
        "entity_chunk_off": "_entity_chunk_off",
        "bone_array_in_model_state": "_bone_array_in_model_state",
        "dw_local_player_pawn": "_dw_local_player_pawn",
        "dw_local_player_controller": "_dw_local_player_controller",
        "dw_global_vars": "_dw_global_vars",
        "map_name_off": "_map_name_off",
    }
    for key, attr in mapping.items():
        if key not in raw:
            continue
        val = int(raw[key])
        setattr(reader, attr, val)
        applied.append(f"{key}=0x{val:x}")
    return applied
