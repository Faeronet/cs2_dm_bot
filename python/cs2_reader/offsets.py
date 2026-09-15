from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from cs2_reader.platform import client_module_name, detect_platform, engine_module_name, offsets_file


@dataclass(frozen=True)
class ClientOffsets:
    dw_entity_list: int
    dw_global_vars: int
    dw_local_player_controller: int
    dw_local_player_pawn: int
    dw_view_matrix: int
    dw_game_rules: int
    dw_planted_c4: int
    dw_csgo_input: int = 0
    dw_glow_manager: int = 0
    dw_view_angles: int = 0
    dw_view_render: int = 0
    dw_prediction: int = 0
    dw_game_entity_system: int = 0
    dw_game_entity_system_highest_entity_index: int = 0


@dataclass(frozen=True)
class EngineOffsets:
    dw_build_number: int
    dw_window_width: int
    dw_window_height: int


@dataclass(frozen=True)
class Offsets:
    client: ClientOffsets
    engine: EngineOffsets
    platform: str

    @classmethod
    def load(cls, path: Path | None = None) -> "Offsets":
        path = path or offsets_file()
        raw = json.loads(path.read_text(encoding="utf-8"))
        platform = detect_platform()
        client_key = client_module_name(platform)
        engine_key = engine_module_name(platform)

        if client_key not in raw:
            # Fallback: try alternate platform keys
            for alt in ("libclient.so", "client.dll"):
                if alt in raw:
                    client_key = alt
                    break
            else:
                raise KeyError(f"No client offsets in {path}")

        if engine_key not in raw:
            for alt in ("libengine2.so", "engine2.dll"):
                if alt in raw:
                    engine_key = alt
                    break
            else:
                raise KeyError(f"No engine offsets in {path}")

        client = _parse_module_offsets(raw[client_key])
        engine = _parse_module_offsets(raw[engine_key])

        return cls(
            client=ClientOffsets(
                dw_entity_list=client["dwEntityList"],
                dw_global_vars=client["dwGlobalVars"],
                dw_local_player_controller=client["dwLocalPlayerController"],
                dw_local_player_pawn=client["dwLocalPlayerPawn"],
                dw_view_matrix=client["dwViewMatrix"],
                dw_game_rules=client.get("dwGameRules", 0),
                dw_planted_c4=client.get("dwPlantedC4", 0),
                dw_csgo_input=client.get("dwCSGOInput", 0),
                dw_glow_manager=client.get("dwGlowManager", 0),
                dw_view_angles=client.get("dwViewAngles", 0),
                dw_view_render=client.get("dwViewRender", 0),
                dw_prediction=client.get("dwPrediction", 0),
                dw_game_entity_system=client.get("dwGameEntitySystem", 0),
                dw_game_entity_system_highest_entity_index=client.get(
                    "dwGameEntitySystem_highestEntityIndex", 0
                ),
            ),
            engine=EngineOffsets(
                dw_build_number=engine["dwBuildNumber"],
                dw_window_width=engine["dwWindowWidth"],
                dw_window_height=engine["dwWindowHeight"],
            ),
            platform=platform.value,
        )


def _parse_module_offsets(module_data: dict | list) -> dict[str, int]:
    if isinstance(module_data, list):
        return {item["name"]: item["value"] for item in module_data}
    return module_data


ENTITY_LIST_ENTRY_SIZE = 0x70
ENTITY_LIST_CHUNK_OFFSET = 0x0  # Linux native: dwEntityList == GES+0x10
ENTITY_LIST_CHUNK_OFFSET_WINDOWS = 0x10
GLOBAL_VARS_MAP_NAME = 0x198  # Linux native: "de_dust2"
GLOBAL_VARS_MAP_NAME_WINDOWS = 0x180
MAX_PLAYERS = 64
