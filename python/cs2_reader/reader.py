"""High-level CS2 game state reader — Linux native + Windows."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

from cs2_reader.aim import aim_point, eye_position, forward_from_angles, normalize_yaw, yaw_to_target
from cs2_reader.game_state import BombInfo, GameSnapshot, PlayerInfo, Vec3, ViewMatrix
from cs2_reader.memory import MemoryBackend, MinidumpBackend, create_live_backend
from cs2_reader.offsets import (
    ENTITY_LIST_CHUNK_OFFSET,
    ENTITY_LIST_CHUNK_OFFSET_WINDOWS,
    ENTITY_LIST_ENTRY_SIZE,
    GLOBAL_VARS_MAP_NAME,
    GLOBAL_VARS_MAP_NAME_WINDOWS,
    MAX_PLAYERS,
    Offsets,
)
from cs2_reader.platform import Platform, client_module_name, detect_platform, engine_module_name
from cs2_reader.schema import Schema

LIFE_ALIVE = 0


class Reader:
    def __init__(
        self,
        backend: MemoryBackend,
        offsets: Offsets | None = None,
        schema: Schema | None = None,
    ):
        self.backend = backend
        self.offsets = offsets or Offsets.load()
        self.schema = schema or Schema.load()
        self._cached_client_base: int | None = None
        self._cached_engine_base: int | None = None
        self._client_mod = client_module_name()
        self._engine_mod = engine_module_name()
        self._platform = detect_platform()
        self._entity_chunk_off = (
            ENTITY_LIST_CHUNK_OFFSET
            if self._platform == Platform.LINUX_NATIVE
            else ENTITY_LIST_CHUNK_OFFSET_WINDOWS
        )
        self._map_name_off = (
            GLOBAL_VARS_MAP_NAME
            if self._platform == Platform.LINUX_NATIVE
            else GLOBAL_VARS_MAP_NAME_WINDOWS
        )

    @classmethod
    def from_pid(cls, pid: int) -> "Reader":
        if sys.platform == "win32":
            from cs2_reader.memory import WindowsProcessBackend

            return cls(WindowsProcessBackend(pid))
        from cs2_reader.memory import ProcMemoryBackend

        return cls(ProcMemoryBackend(pid))

    @classmethod
    def from_live(cls) -> "Reader | None":
        backend = create_live_backend()
        return cls(backend) if backend else None

    @classmethod
    def from_dump(cls, path: str) -> "Reader":
        from pathlib import Path

        return cls(MinidumpBackend(Path(path)))

    def _client_base(self) -> int | None:
        if self._cached_client_base is None:
            self._cached_client_base = self.backend.module_base(self._client_mod)
        return self._cached_client_base

    def _engine_base(self) -> int | None:
        if self._cached_engine_base is None:
            self._cached_engine_base = self.backend.module_base(self._engine_mod)
        return self._cached_engine_base

    def entity_list(self) -> int | None:
        client = self._client_base()
        if not client:
            return None
        return self.backend.read_u64(client + self.offsets.client.dw_entity_list)

    def local_controller(self) -> int | None:
        client = self._client_base()
        if not client:
            return None
        return self.backend.read_u64(client + self.offsets.client.dw_local_player_controller)

    def local_pawn(self) -> int | None:
        client = self._client_base()
        if not client:
            return None
        pawn = self.backend.read_u64(client + self.offsets.client.dw_local_player_pawn)
        if pawn:
            return pawn
        controller = self.local_controller()
        if controller:
            return self.pawn_from_controller(controller)
        return None

    def entity_by_index(self, index: int) -> int | None:
        elist = self.entity_list()
        if not elist:
            return None
        chunk = (index & 0x7FFF) >> 9
        slot = index & 0x1FF
        list_entry = self.backend.read_u64(elist + 8 * chunk + self._entity_chunk_off)
        if not list_entry:
            return None
        return self.backend.read_u64(list_entry + ENTITY_LIST_ENTRY_SIZE * slot)

    def local_index(self) -> int:
        controller = self.local_controller()
        if not controller:
            return -1
        for i in range(1, MAX_PLAYERS):
            if self.entity_by_index(i) == controller:
                return i
        return -1

    def pawn_from_controller(self, controller: int) -> int | None:
        handle = self.backend.read_u32(controller + self.schema.m_h_player_pawn)
        if not handle or handle == 0xFFFFFFFF:
            return None
        return self.entity_by_index(handle & 0x7FFF)

    def is_visible(self, pawn: int, viewer_index: int) -> bool:
        state = pawn + self.schema.m_entity_spotted_state
        if self.backend.read_u8(state + self.schema.m_b_spotted):
            return True
        if viewer_index < 0 or viewer_index >= 64:
            return False
        data = self.backend.read(state + self.schema.m_b_spotted_by_mask, 8)
        if not data:
            return False
        mask = int.from_bytes(data, "little")
        return bool(mask & (1 << viewer_index))

    def map_name(self) -> str:
        client = self._client_base()
        if not client:
            return ""
        global_vars = self.backend.read_u64(client + self.offsets.client.dw_global_vars)
        if not global_vars:
            return ""

        for off in (self._map_name_off, GLOBAL_VARS_MAP_NAME_WINDOWS, 0x180, 0x188, 0x198):
            name = self.backend.read_cstring_ptr(global_vars + off)
            if not name:
                continue
            if name.startswith("maps/"):
                name = Path(name).stem
            if name and name not in ("<empty>", "") and len(name) >= 2:
                return name
        return ""

    def view_matrix(self) -> ViewMatrix | None:
        client = self._client_base()
        if not client:
            return None
        data = self.backend.read(client + self.offsets.client.dw_view_matrix, 64)
        if not data:
            return None
        import struct

        vals = struct.unpack("<16f", data)
        m = ViewMatrix()
        for row in range(4):
            for col in range(4):
                m.m[row][col] = vals[row * 4 + col]
        return m

    def local_position(self) -> Vec3:
        pawn = self.local_pawn()
        if not pawn:
            return Vec3()
        coords = self.backend.read_vec3(pawn + self.schema.m_v_old_origin)
        if coords:
            return Vec3(*coords)
        scene = self.backend.read_u64(pawn + self.schema.m_p_game_scene_node)
        if scene:
            coords = self.backend.read_vec3(scene + self.schema.m_vec_abs_origin)
            if coords:
                return Vec3(*coords)
        return Vec3()

    def local_velocity(self) -> Vec3:
        pawn = self.local_pawn()
        if not pawn:
            return Vec3()
        coords = self.backend.read_vec3(pawn + self.schema.m_vec_velocity)
        return Vec3(*coords) if coords else Vec3()

    def local_eye_angles(self) -> Vec3:
        """pitch=X, yaw=Y, roll=Z (m_angEyeAngles)."""
        pawn = self.local_pawn()
        if not pawn:
            return Vec3()
        eye = self.backend.read_vec3(pawn + self.schema.m_ang_eye_angles)
        return Vec3(*eye) if eye else Vec3()

    def local_view_offset_z(self) -> float:
        pawn = self.local_pawn()
        if not pawn:
            return 64.0
        off = self.backend.read_vec3(pawn + self.schema.m_vec_view_offset)
        return off[2] if off else 64.0

    def local_eye_position(self) -> Vec3:
        return eye_position(self.local_position(), self.local_view_offset_z())

    def local_aim_forward(self) -> Vec3:
        ang = self.local_eye_angles()
        return forward_from_angles(ang.x, ang.y)

    def local_aim_point(self, distance: float = 8192.0) -> Vec3:
        """Точка прицела: луч из глаз, при взгляде вниз — пересечение с полом."""
        feet = self.local_position()
        ang = self.local_eye_angles()
        return aim_point(
            feet,
            ang.x,
            ang.y,
            distance,
            view_offset_z=self.local_view_offset_z(),
            ground_z=feet.z,
        )

    def local_pose_check(self, tx: float, ty: float) -> dict:
        """Сверка: bearing на (tx,ty) vs yaw прицела."""
        feet = self.local_position()
        ang = self.local_eye_angles()
        bearing = yaw_to_target(feet.x, feet.y, tx, ty)
        err = normalize_yaw(bearing - ang.y)
        aim = self.local_aim_point()
        eyes = self.local_eye_position()
        return {
            "feet": (feet.x, feet.y, feet.z),
            "eyes": (eyes.x, eyes.y, eyes.z),
            "pitch": ang.x,
            "yaw": ang.y,
            "aim_point": (aim.x, aim.y, aim.z),
            "bearing_to": (tx, ty),
            "bearing_deg": bearing,
            "yaw_err_to_target": err,
        }

    def read_player(
        self,
        index: int,
        controller: int,
        local_index: int,
        local_team: int,
        local_pos: Vec3,
    ) -> PlayerInfo | None:
        pawn = self.pawn_from_controller(controller)
        if not pawn:
            return None

        life = self.backend.read_u8(pawn + self.schema.m_life_state)
        health = self.backend.read_i32(pawn + self.schema.m_i_health)
        team = self.backend.read_i32(pawn + self.schema.m_i_team_num)
        if life is None or health is None or team is None:
            return None
        if health <= 0 or life != LIFE_ALIVE:
            return None

        player = PlayerInfo(
            index=index,
            controller=controller,
            pawn=pawn,
            health=health,
            team=team,
            armor=self.backend.read_i32(pawn + self.schema.m_armor_value) or 0,
            alive=True,
            is_local=(index == local_index),
            ping=self.backend.read_i32(controller + self.schema.m_i_ping) or 0,
            scoped=bool(self.backend.read_u8(pawn + self.schema.m_b_is_scoped)),
            shots_fired=self.backend.read_i32(pawn + self.schema.m_i_shots_fired) or 0,
            defusing=bool(self.backend.read_u8(pawn + self.schema.m_b_is_defusing)),
            flags=self.backend.read_i32(pawn + self.schema.m_f_flags) or 0,
        )

        name_ptr = self.backend.read_u64(controller + self.schema.m_s_sanitized_player_name)
        if name_ptr:
            player.name = self.backend.read_string(name_ptr, 64)

        coords = self.backend.read_vec3(pawn + self.schema.m_v_old_origin)
        if coords:
            player.position = Vec3(*coords)
        else:
            scene = self.backend.read_u64(pawn + self.schema.m_p_game_scene_node)
            if scene:
                coords = self.backend.read_vec3(scene + self.schema.m_vec_abs_origin)
                if coords:
                    player.position = Vec3(*coords)

        eye = self.backend.read_vec3(pawn + self.schema.m_ang_eye_angles)
        if eye:
            player.eye_angles = Vec3(*eye)

        scene = self.backend.read_u64(pawn + self.schema.m_p_game_scene_node)
        if scene:
            player.dormant = bool(self.backend.read_u8(scene + self.schema.m_b_dormant))

        player.distance = player.position.distance_to(local_pos)
        player.spotted = self.is_visible(pawn, local_index)
        return player

    def players(self) -> list[PlayerInfo]:
        local_idx = self.local_index()
        local_pawn = self.local_pawn()
        local_team = 0
        if local_pawn:
            local_team = self.backend.read_i32(local_pawn + self.schema.m_i_team_num) or 0
        local_pos = self.local_position()

        result: list[PlayerInfo] = []
        for i in range(1, MAX_PLAYERS):
            controller = self.entity_by_index(i)
            if not controller:
                continue
            player = self.read_player(i, controller, local_idx, local_team, local_pos)
            if player:
                result.append(player)
        return result

    def visible_enemies(self) -> list[PlayerInfo]:
        local_pawn = self.local_pawn()
        local_team = self.backend.read_i32(local_pawn + self.schema.m_i_team_num) if local_pawn else 0
        enemies: list[PlayerInfo] = []
        for p in self.players():
            if p.is_local or p.team == local_team or p.team <= 1:
                continue
            if p.dormant:
                continue
            if p.spotted:
                enemies.append(p)
        return enemies

    def bomb_info(self) -> BombInfo | None:
        client = self._client_base()
        if not client or not self.offsets.client.dw_planted_c4:
            return None
        planted = self.backend.read_u64(client + self.offsets.client.dw_planted_c4)
        if not planted:
            return None
        scene = self.backend.read_u64(planted + self.schema.m_p_game_scene_node)
        if not scene:
            return None
        coords = self.backend.read_vec3(scene + self.schema.m_vec_abs_origin)
        if not coords:
            return None
        return BombInfo(position=Vec3(*coords), planted=True)

    def capture(self) -> GameSnapshot:
        snap = GameSnapshot()
        client = self._client_base()
        engine = self._engine_base()
        if not client or not engine:
            snap.errors.append("client/engine module not found in process memory")
            return snap

        snap.platform = self.offsets.platform
        snap.client_base = client
        snap.engine_base = engine
        snap.map_name = self.map_name()
        snap.in_game = bool(snap.map_name) and snap.map_name not in ("<empty>", "")
        snap.in_match = bool(self.local_pawn())
        snap.build_number = self._read_build_number(engine)
        snap.screen_width = float(self.backend.read_i32(engine + self.offsets.engine.dw_window_width) or 0)
        snap.screen_height = float(self.backend.read_i32(engine + self.offsets.engine.dw_window_height) or 0)
        snap.local_index = self.local_index()
        snap.local_position = self.local_position()
        snap.view_matrix = self.view_matrix()
        snap.local_eye_angles = self.local_eye_angles()
        snap.local_aim_forward = self.local_aim_forward()
        snap.local_aim_point = self.local_aim_point()

        local_pawn = self.local_pawn()
        if local_pawn:
            snap.local_team = self.backend.read_i32(local_pawn + self.schema.m_i_team_num) or 0

        snap.players = self.players()
        snap.visible_enemies = self.visible_enemies()
        snap.bomb = self.bomb_info()
        return snap

    def _read_build_number(self, engine: int) -> int:
        off = self.offsets.engine.dw_build_number
        direct = self.backend.read_u32(engine + off)
        if direct and direct > 1_000_000:
            return direct
        ptr = self.backend.read_u64(engine + off)
        if ptr:
            val = self.backend.read_u32(ptr)
            if val and val > 1_000_000:
                return val
        info_path = Path(__file__).resolve().parents[2] / "data" / "info.json"
        if info_path.exists():
            import json

            return json.loads(info_path.read_text()).get("build_number", 0) or 0
        return direct or 0

    def debug_modules(self) -> dict[str, int]:
        if hasattr(self.backend, "list_modules"):
            return {k: v.base for k, v in self.backend.list_modules().items()}  # type: ignore[attr-defined]
        return {}
