"""High-level CS2 game state reader — Linux native + Windows."""

from __future__ import annotations

import re
import sys
import time
from pathlib import Path
from typing import Optional

from cs2_reader.aim import (
    BONE_ARRAY_IN_MODEL_STATE,
    BONE_CHEST,
    BONE_HEAD,
    BONE_LEFT_SHOULDER,
    BONE_RIGHT_SHOULDER,
    BONE_STRIDE,
    aim_point,
    chest_center_from_view,
    chest_plausible,
    eye_position,
    forward_from_angles,
    head_center_from_view,
    head_plausible,
    normalize_yaw,
    shoulder_plausible,
    yaw_to_target,
)
from cs2_reader.game_state import BombInfo, EnemyInfo, GameSnapshot, PlayerInfo, Vec3, ViewMatrix, WeaponInfo
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
from cs2_reader.breakables import (
    BreakableInfo,
    is_breakable_candidate,
    merge_breakables,
    read_breakable,
)
from cs2_reader.doors import DoorInfo, is_door_entity, read_door
from cs2_reader.ladders import LadderInfo, entity_designer_name, is_ladder_entity, read_ladder
from cs2_reader.life import (
    LIFE_ALIVE,
    LIFE_RESPAWNING,
    LifeTracker,
    LocalLifeInfo,
    LocalLifeUpdate,
    is_playable,
)
from cs2_reader.account import LocalAccountInfo, is_valid_steam_id64, steam_account_id
from cs2_reader.profile_rank import ProfileRankInfo, ProfileRankReader
from cs2_reader.schema import Schema
from cs2_reader.visibility import read_radar_spotted, spotted_by_viewer
from cs2_reader.vulnerability import read_pawn_vulnerability
from cs2_reader.weapons import is_healthshot, weapon_name

MAX_ENTITIES = 8192
_MAP_ID = re.compile(r"^(?:de|cs|ar|dz|gd)_[a-z0-9_]+$")


def _map_id(raw: str) -> str:
    """de_dust2 из строки памяти. Мусор и меню — пустая строка."""
    name = (raw or "").strip().replace("\\", "/")
    if name.startswith("maps/"):
        name = name[5:]
    if "/" in name:
        name = name.rsplit("/", 1)[-1]
    if name.endswith(".vpk"):
        name = name[: -len(".vpk")]
    name = name.lower()
    return name if _MAP_ID.match(name) else ""
LADDER_CACHE_TTL = 5.0
DOOR_CACHE_TTL = 1.0
BREAKABLE_CACHE_TTL = 1.0
BREAKABLE_SCAN_MAX = 4096


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
        self._life_tracker = LifeTracker()
        self._ladder_cache: list[LadderInfo] = []
        self._ladder_cache_at = 0.0
        self._ladder_map = ""
        self._door_cache: list[DoorInfo] = []
        self._door_cache_at = 0.0
        self._door_map = ""
        self._breakable_cache: list[BreakableInfo] = []
        self._breakable_cache_at = 0.0
        self._breakable_map = ""
        self._profile_rank_reader = ProfileRankReader(self.backend, self.schema)

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
        pawn = self._usable_ptr(self.backend.read_u64(client + self.offsets.client.dw_local_player_pawn))
        if pawn:
            return pawn
        controller = self.local_controller()
        if controller:
            return self.pawn_from_controller(controller)
        return None

    @staticmethod
    def _usable_ptr(ptr: int | None) -> int | None:
        """Отсечь 0 и мусор вроде 0xffffffff00000000 (битый dwLocalPlayerPawn)."""
        if not ptr or ptr < 0x10000 or ptr > 0x00007FFFFFFFFFFF:
            return None
        return ptr

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
        """Прямая видимость для локального игрока (m_bSpottedByMask, не m_bSpotted)."""
        return spotted_by_viewer(
            self.backend,
            pawn,
            self.schema.m_entity_spotted_state,
            self.schema.m_b_spotted_by_mask,
            viewer_index,
        )

    def is_radar_spotted(self, pawn: int) -> bool:
        return read_radar_spotted(
            self.backend,
            pawn,
            self.schema.m_entity_spotted_state,
            self.schema.m_b_spotted,
        )

    def map_name(self) -> str:
        client = self._client_base()
        if not client:
            return ""
        global_vars = self.backend.read_u64(client + self.offsets.client.dw_global_vars)
        if not global_vars:
            return ""

        # Несколько оффсетов: первый «непустой» часто мусор (не имя карты).
        # Берём только id вида de_dust2 и предпочитаем оффсет текущей платформы.
        found: list[tuple[int, str]] = []
        seen: set[int] = set()
        for off in (self._map_name_off, GLOBAL_VARS_MAP_NAME, GLOBAL_VARS_MAP_NAME_WINDOWS, 0x180, 0x188, 0x198):
            if off in seen:
                continue
            seen.add(off)
            raw = self.backend.read_cstring_ptr(global_vars + off)
            token = _map_id(raw)
            if token:
                found.append((off, token))
        for off, token in found:
            if off == self._map_name_off:
                return token
        return found[0][1] if found else ""

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

    def local_move_type(self) -> int:
        pawn = self.local_pawn()
        if not pawn or not self.schema.m_n_actual_move_type:
            return 0
        return int(self.backend.read_u8(pawn + self.schema.m_n_actual_move_type) or 0)

    def local_on_ladder(self) -> bool:
        return self.local_move_type() in (8, 9)

    def local_eye_angles(self) -> Vec3:
        """pitch=X, yaw=Y, roll=Z (m_angEyeAngles)."""
        pawn = self.local_pawn()
        if not pawn:
            return Vec3()
        eye = self.backend.read_vec3(pawn + self.schema.m_ang_eye_angles)
        return Vec3(*eye) if eye else Vec3()

    def pawn_view_offset(self, pawn: int) -> Vec3:
        off = self.backend.read_vec3(pawn + self.schema.m_vec_view_offset)
        return Vec3(*off) if off else Vec3(0.0, 0.0, 64.0)

    def pawn_bone_position(self, pawn: int, bone: int = BONE_HEAD) -> Vec3 | None:
        scene = self.backend.read_u64(pawn + self.schema.m_p_game_scene_node)
        if not scene:
            return None
        bone_array = self.backend.read_u64(scene + self.schema.m_model_state + BONE_ARRAY_IN_MODEL_STATE)
        if not bone_array:
            return None
        coords = self.backend.read_vec3(bone_array + bone * BONE_STRIDE)
        if not coords:
            return None
        return Vec3(*coords)

    def pawn_head_position(self, pawn: int, feet: Vec3) -> Vec3:
        """Центр головы: кость 6 (CSkeletonInstance → BoneArray), иначе m_vecViewOffset + 4."""
        head = self.pawn_bone_position(pawn, BONE_HEAD)
        if head and head_plausible(feet, head):
            return head
        return head_center_from_view(feet, self.pawn_view_offset(pawn))

    def pawn_chest_position(self, pawn: int, feet: Vec3, head: Vec3 | None = None) -> Vec3:
        """Центр груди: кость 4 (spine_1), иначе m_vecViewOffset − 8."""
        chest = self.pawn_bone_position(pawn, BONE_CHEST)
        if chest and chest_plausible(feet, chest, head):
            return chest
        view = self.pawn_view_offset(pawn)
        fallback = chest_center_from_view(feet, view)
        if head is not None and chest_plausible(feet, fallback, head):
            return fallback
        if head is None and chest_plausible(feet, fallback):
            return fallback
        return fallback

    def pawn_shoulder_positions(
        self,
        pawn: int,
        feet: Vec3,
        chest: Vec3 | None = None,
        head: Vec3 | None = None,
    ) -> tuple[Vec3, Vec3]:
        """Левое/правое плечо: кости clavicle_L (8) и clavicle_R (13). Без кости — (0,0,0)."""
        left = self.pawn_bone_position(pawn, BONE_LEFT_SHOULDER)
        right = self.pawn_bone_position(pawn, BONE_RIGHT_SHOULDER)
        if not left or not shoulder_plausible(feet, left, chest, head):
            left = Vec3()
        if not right or not shoulder_plausible(feet, right, chest, head):
            right = Vec3()
        return left, right

    def local_head_position(self) -> Vec3:
        pawn = self.local_pawn()
        if not pawn:
            return Vec3()
        feet = self.local_position()
        return self.pawn_head_position(pawn, feet)

    def active_weapon(self, pawn: int) -> int | None:
        ws = self.backend.read_u64(pawn + self.schema.m_p_weapon_services)
        if not ws:
            return None
        handle = self.backend.read_u32(ws + self.schema.m_h_active_weapon)
        if not handle or handle == 0xFFFFFFFF:
            return None
        return self.entity_by_index(handle & 0x7FFF)

    def weapon_from_entity(self, weapon: int) -> WeaponInfo | None:
        if not weapon:
            return None
        s = self.schema
        item = weapon + s.m_attribute_manager + s.m_item
        item_id = self.backend.read_u16(item + s.m_i_item_definition_index)
        if item_id is None:
            return None
        clip = self.backend.read_i32(weapon + s.m_i_clip1)
        reloading = bool(self.backend.read_u8(weapon + s.m_b_in_reload))
        return WeaponInfo(
            id=item_id,
            name=weapon_name(item_id),
            clip=max(0, clip or 0),
            in_reload=reloading,
        )

    def weapon_from_pawn(self, pawn: int) -> WeaponInfo:
        weapon = self.active_weapon(pawn)
        if not weapon:
            return WeaponInfo()
        info = self.weapon_from_entity(weapon)
        return info or WeaponInfo()

    def local_weapon(self) -> WeaponInfo:
        pawn = self.local_pawn()
        if not pawn:
            return WeaponInfo()
        return self.weapon_from_pawn(pawn)

    def pawn_weapon_ids(self, pawn: int) -> list[int]:
        """m_hMyWeapons: id всех предметов в инвентаре pawn."""
        if not pawn:
            return []
        s = self.schema
        if not s.m_p_weapon_services or not s.m_h_my_weapons:
            return []
        ws = self.backend.read_u64(pawn + s.m_p_weapon_services)
        if not ws:
            return []
        vec = ws + s.m_h_my_weapons
        size = self.backend.read_u32(vec)
        ptr = self.backend.read_u64(vec + 8)
        if not size or size > 64 or not ptr:
            return []
        ids: list[int] = []
        for i in range(int(size)):
            handle = self.backend.read_u32(ptr + i * 4)
            if not handle or handle == 0xFFFFFFFF:
                continue
            ent = self.entity_by_index(handle & 0x7FFF)
            if not ent:
                continue
            item = ent + s.m_attribute_manager + s.m_item
            item_id = self.backend.read_u16(item + s.m_i_item_definition_index)
            if item_id:
                ids.append(int(item_id))
        return ids

    def local_has_healthshot(self) -> bool:
        pawn = self.local_pawn()
        if not pawn:
            return False
        w = self.local_weapon()
        if is_healthshot(int(w.id or 0), str(w.name or "")):
            return True
        return any(is_healthshot(i) for i in self.pawn_weapon_ids(pawn))

    def local_shots_fired(self) -> int:
        pawn = self.local_pawn()
        if not pawn or not self.schema.m_i_shots_fired:
            return 0
        return max(0, self.backend.read_i32(pawn + self.schema.m_i_shots_fired) or 0)

    def local_is_scoped(self) -> bool:
        pawn = self.local_pawn()
        if not pawn or not self.schema.m_b_is_scoped:
            return False
        return bool(self.backend.read_u8(pawn + self.schema.m_b_is_scoped))

    def read_local_life(self) -> LocalLifeInfo:
        """
        Жив ли локальный игрок (DM: после смерти False, после респавна True).

        m_iHealth + m_lifeState на pawn, m_bPawnIsAlive на controller.
        """
        controller = self.local_controller()
        pawn_alive_ctrl = False
        if controller:
            flag = self.backend.read_u8(controller + self.schema.m_b_pawn_is_alive)
            pawn_alive_ctrl = bool(flag) if flag is not None else False

        pawn = self.local_pawn()
        if not pawn:
            return LocalLifeInfo(
                alive=False,
                health=0,
                life_state=2,
                pawn_alive=pawn_alive_ctrl,
                pawn=0,
                respawning=False,
            )

        health = self.backend.read_i32(pawn + self.schema.m_i_health) or 0
        life_raw = self.backend.read_u8(pawn + self.schema.m_life_state)
        life_state = life_raw if life_raw is not None else 2
        alive = is_playable(health, life_state, pawn_alive_ctrl)
        vuln = read_pawn_vulnerability(self.backend, self.schema, pawn)
        return LocalLifeInfo(
            alive=alive,
            health=max(0, health),
            life_state=life_state,
            pawn_alive=pawn_alive_ctrl,
            pawn=pawn,
            respawning=life_state == LIFE_RESPAWNING and not alive,
            vulnerable=vuln.vulnerable,
            spawn_immunity=vuln.spawn_immunity,
            has_moved_since_spawn=vuln.has_moved_since_spawn,
        )

    def poll_life(self) -> LocalLifeUpdate:
        """Состояние + импульсы died/respawned с прошлого вызова (одноразовые флаги)."""
        state = self.read_local_life()
        events = self._life_tracker.step(state)
        return LocalLifeUpdate(state=state, events=events)

    def local_view_offset_z(self) -> float:
        pawn = self.local_pawn()
        if not pawn:
            return 64.0
        return self.pawn_view_offset(pawn).z

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

        steam_id64 = self.backend.read_u64(controller + self.schema.m_steam_id)
        if is_valid_steam_id64(steam_id64):
            player.steam_id64 = steam_id64
            player.account_id = steam_account_id(steam_id64)

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

        player.head_position = self.pawn_head_position(pawn, player.position)
        player.chest_position = self.pawn_chest_position(pawn, player.position, player.head_position)
        player.left_shoulder_position, player.right_shoulder_position = self.pawn_shoulder_positions(
            pawn, player.position, player.chest_position, player.head_position,
        )
        player.distance = player.position.distance_to(local_pos)
        player.spotted = self.is_visible(pawn, local_index)
        vel = self.backend.read_vec3(pawn + self.schema.m_vec_velocity)
        if vel:
            player.velocity = Vec3(*vel)
        vuln = read_pawn_vulnerability(self.backend, self.schema, pawn)
        player.vulnerable = vuln.vulnerable
        player.spawn_immunity = vuln.spawn_immunity
        player.has_moved_since_spawn = vuln.has_moved_since_spawn
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

    def enemies(self) -> list[EnemyInfo]:
        """Все живые игроки кроме локального (DM FFA: каждый — цель)."""
        out: list[EnemyInfo] = []
        for p in self.players():
            if not self._is_enemy(p):
                continue
            spotted = p.spotted and not p.dormant
            out.append(
                EnemyInfo(
                    index=p.index,
                    name=p.name,
                    health=p.health,
                    armor=p.armor,
                    position=p.position,
                    head_position=p.head_position,
                    chest_position=p.chest_position,
                    left_shoulder_position=p.left_shoulder_position,
                    right_shoulder_position=p.right_shoulder_position,
                    spotted=spotted,
                    visible=False,  # LOS — заполняет navigator / apply_los_visibility
                    radar_spotted=self.is_radar_spotted(p.pawn),
                    dormant=p.dormant,
                    distance=p.distance,
                    velocity=p.velocity,
                    vulnerable=p.vulnerable,
                    spawn_immunity=p.spawn_immunity,
                    has_moved_since_spawn=p.has_moved_since_spawn,
                )
            )
        return out

    def visible_enemies(self) -> list[PlayerInfo]:
        return [p for p in self.players() if p.spotted and not p.dormant and self._is_enemy(p)]

    def _is_enemy(self, p: PlayerInfo) -> bool:
        """DM: любой живой игрок, кроме себя (без фильтра по команде)."""
        return not p.is_local and p.alive and p.team > 1

    def highest_entity_index(self) -> int:
        client = self._client_base()
        off = self.offsets.client
        if not client or not off.dw_game_entity_system or not off.dw_game_entity_system_highest_entity_index:
            return MAX_ENTITIES
        ges = self.backend.read_u64(client + off.dw_game_entity_system)
        if not ges:
            return MAX_ENTITIES
        hi = self.backend.read_i32(ges + off.dw_game_entity_system_highest_entity_index)
        return min(max(hi or 0, 0), MAX_ENTITIES)

    def ladders(self, force_refresh: bool = False) -> list[LadderInfo]:
        """Все func_ladder на карте: координаты, направление, габариты (кэш ~5 с)."""
        map_name = self.map_name()
        now = time.monotonic()
        if (
            not force_refresh
            and self._ladder_cache
            and now - self._ladder_cache_at < LADDER_CACHE_TTL
            and self._ladder_map == map_name
        ):
            local = self.local_position()
            for lad in self._ladder_cache:
                lad.distance = lad.center.distance_to(local)
            return self._ladder_cache

        local = self.local_position()
        out: list[LadderInfo] = []
        seen: set[tuple[int, int, int]] = set()
        for i in range(1, self.highest_entity_index() + 1):
            ent = self.entity_by_index(i)
            if not ent or ent < 0x10000:
                continue
            cls = entity_designer_name(self.backend, self.schema, ent)
            if not is_ladder_entity(cls):
                continue
            lad = read_ladder(self.backend, self.schema, ent, i, local, name=cls or "func_ladder")
            if not lad or lad.disabled:
                continue
            key = (round(lad.bottom.x), round(lad.bottom.y), round(lad.bottom.z))
            if key in seen:
                continue
            seen.add(key)
            out.append(lad)

        out.sort(key=lambda lad: lad.distance)
        self._ladder_cache = out
        self._ladder_cache_at = now
        self._ladder_map = map_name
        return out

    def doors(self, force_refresh: bool = False) -> list[DoorInfo]:
        """Двери на карте: положение, состояние (open/closed/…), locked (кэш ~1 с)."""
        map_name = self.map_name()
        now = time.monotonic()
        if (
            not force_refresh
            and self._door_cache
            and now - self._door_cache_at < DOOR_CACHE_TTL
            and self._door_map == map_name
        ):
            local = self.local_position()
            for door in self._door_cache:
                door.distance = door.position.distance_to(local)
            return self._door_cache

        local = self.local_position()
        out: list[DoorInfo] = []
        seen: set[tuple[int, int, int]] = set()
        scan_hi = max(self.highest_entity_index(), 512)
        for i in range(1, scan_hi + 1):
            ent = self.entity_by_index(i)
            if not ent or ent < 0x10000:
                continue
            cls = entity_designer_name(self.backend, self.schema, ent)
            if not is_door_entity(cls):
                continue
            door = read_door(self.backend, self.schema, ent, i, local, name=cls or "door")
            if not door:
                continue
            key = (
                round(door.position.x),
                round(door.position.y),
                round(door.position.z),
            )
            if key in seen:
                continue
            seen.add(key)
            out.append(door)

        out.sort(key=lambda door: door.distance)
        self._door_cache = out
        self._door_cache_at = now
        self._door_map = map_name
        return out

    def breakables(self, force_refresh: bool = False) -> list[BreakableInfo]:
        """Ломаемые entity (решётки, окна, func_breakable): позиция и целостность."""
        map_name = self.map_name()
        now = time.monotonic()
        if (
            not force_refresh
            and self._breakable_cache
            and now - self._breakable_cache_at < BREAKABLE_CACHE_TTL
            and self._breakable_map == map_name
        ):
            local = self.local_position()
            for item in self._breakable_cache:
                item.distance = item.position.distance_to(local)
            return self._breakable_cache

        local = self.local_position()
        out: list[BreakableInfo] = []
        scan_hi = max(self.highest_entity_index(), BREAKABLE_SCAN_MAX)
        for i in range(1, scan_hi + 1):
            ent = self.entity_by_index(i)
            if not ent or ent < 0x10000:
                continue
            cls = entity_designer_name(self.backend, self.schema, ent)
            if not is_breakable_candidate(self.backend, self.schema, ent, name=cls):
                continue
            br = read_breakable(self.backend, self.schema, ent, i, local, name=cls)
            if not br:
                continue
            out.append(br)

        out = merge_breakables(out)
        out.sort(key=lambda item: item.distance)
        self._breakable_cache = out
        self._breakable_cache_at = now
        self._breakable_map = map_name
        return out

    def profile_rank(self) -> ProfileRankInfo:
        """Profile Rank 1–40 и XP прогресс (weekly drop rank)."""
        return self._profile_rank_reader.read(self.local_controller())

    def local_account(self) -> LocalAccountInfo:
        """Steam ID64 и 32-bit account ID локального игрока."""
        controller = self.local_controller()
        if not controller:
            return LocalAccountInfo()
        steam_id64 = self.backend.read_u64(controller + self.schema.m_steam_id)
        if not is_valid_steam_id64(steam_id64):
            return LocalAccountInfo()
        return LocalAccountInfo(
            steam_id64=steam_id64,
            account_id=steam_account_id(steam_id64),
            valid=True,
        )

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
        snap.local_weapon = self.local_weapon()
        snap.local_life = self.read_local_life()

        local_pawn = self.local_pawn()
        if local_pawn:
            snap.local_team = self.backend.read_i32(local_pawn + self.schema.m_i_team_num) or 0

        snap.players = self.players()
        snap.enemies = self.enemies()
        snap.visible_enemies = [
            p for p in snap.players if self._is_enemy(p) and p.spotted and not p.dormant
        ]
        snap.bomb = self.bomb_info()
        snap.ladders = self.ladders()
        snap.doors = self.doors()
        snap.breakables = self.breakables()
        snap.profile_rank = self.profile_rank()
        snap.local_account = self.local_account()
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
