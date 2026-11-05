"""После cs2-dumper: подправить dwLocalPlayerPawn / dwGlobalVars по живой памяти."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))

from cs2_reader.match_calibration import (  # noqa: E402
    calibrate_match_offsets,
    scan_global_vars_offset,
    scan_local_pawn_offset,
)
from cs2_reader.reader import Reader
from cs2_reader.runtime_tweaks import save_tweaks


def _patch_offsets_json(pawn_off: int | None, gv_off: int | None) -> list[str]:
    path = ROOT / "data" / "offsets.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    log: list[str] = []
    for item in raw.get("libclient.so", []):
        if item["name"] == "dwLocalPlayerPawn" and pawn_off is not None:
            if item["value"] != pawn_off:
                item["value"] = pawn_off
                log.append(f"offsets.json dwLocalPlayerPawn -> 0x{pawn_off:x}")
        if item["name"] == "dwGlobalVars" and gv_off is not None:
            if item["value"] != gv_off:
                item["value"] = gv_off
                log.append(f"offsets.json dwGlobalVars -> 0x{gv_off:x}")
    if log:
        path.write_text(json.dumps(raw, indent=2) + "\n", encoding="utf-8")
    return log


def main() -> int:
    reader = Reader.from_live()
    if not reader:
        print("CS2 not running — skip live patch")
        return 0
    calibrate_match_offsets(reader, force=True, min_interval_s=0, allow_dump=False)
    pawn = reader._dw_local_player_pawn
    gv = reader._dw_global_vars
    map_off = reader._map_name_off
    save_tweaks(
        dw_local_player_pawn=pawn,
        dw_global_vars=gv,
        map_name_off=map_off,
        entity_chunk_off=reader._entity_chunk_off,
        bone_array_in_model_state=reader._bone_array_in_model_state,
    )
    for line in _patch_offsets_json(pawn, gv):
        print(line)
    pos = reader.local_position()
    ang = reader.local_eye_angles()
    print(f"OK pos=({pos.x:.0f},{pos.y:.0f},{pos.z:.0f}) yaw={ang.y:.1f} map={reader.resolved_map_name()!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
