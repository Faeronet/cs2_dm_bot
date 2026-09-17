from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from cs2_reader.platform import detect_platform, schema_file


@dataclass(frozen=True)
class Schema:
    m_i_health: int
    m_i_team_num: int
    m_life_state: int
    m_p_game_scene_node: int
    m_vec_abs_origin: int
    m_b_dormant: int
    m_v_old_origin: int
    m_h_player_pawn: int
    m_b_pawn_is_alive: int
    m_s_sanitized_player_name: int
    m_armor_value: int
    m_ang_eye_angles: int
    m_vec_view_offset: int
    m_model_state: int
    m_entity_spotted_state: int
    m_b_spotted: int
    m_b_spotted_by_mask: int
    m_b_is_scoped: int = 0
    m_i_shots_fired: int = 0
    m_b_is_defusing: int = 0
    m_i_ping: int = 0
    m_f_flags: int = 0
    m_vec_velocity: int = 0
    m_p_weapon_services: int = 0
    m_h_active_weapon: int = 0
    m_i_clip1: int = 0
    m_attribute_manager: int = 0
    m_item: int = 0
    m_i_item_definition_index: int = 0
    m_b_in_reload: int = 0
    m_b_gun_game_immunity: int = 0
    m_b_has_moved_since_spawn: int = 0
    m_h_my_weapons: int = 0
    m_p_entity: int = 0
    m_designer_name: int = 0
    m_name: int = 0
    m_p_collision: int = 0
    m_vec_mins: int = 0
    m_vec_maxs: int = 0
    m_vec_surrounding_mins: int = 0
    m_vec_surrounding_maxs: int = 0
    m_vec_ladder_dir: int = 0
    m_vec_local_top: int = 0
    m_vec_player_mount_top: int = 0
    m_vec_player_mount_bottom: int = 0
    m_fl_auto_ride_speed: int = 0
    m_b_ladder_disabled: int = 0
    m_b_fake_ladder: int = 0
    m_b_ladder_has_slack: int = 0

    @classmethod
    def load(cls, path: Path | None = None) -> "Schema":
        path = path or schema_file()
        raw = json.loads(path.read_text(encoding="utf-8"))

        if "client.dll" in raw:
            classes = raw["client.dll"]["classes"]
        elif "libclient.so" in raw:
            classes = raw["libclient.so"]["classes"]
        else:
            raise KeyError(f"Unknown schema format in {path}")

        def field(class_name: str, field_name: str) -> int:
            return classes[class_name]["fields"][field_name]

        return cls(
            m_i_health=field("C_BaseEntity", "m_iHealth"),
            m_i_team_num=field("C_BaseEntity", "m_iTeamNum"),
            m_life_state=field("C_BaseEntity", "m_lifeState"),
            m_p_game_scene_node=field("C_BaseEntity", "m_pGameSceneNode"),
            m_vec_abs_origin=field("CGameSceneNode", "m_vecAbsOrigin"),
            m_b_dormant=field("CGameSceneNode", "m_bDormant"),
            m_v_old_origin=field("C_BasePlayerPawn", "m_vOldOrigin"),
            m_h_player_pawn=field("CCSPlayerController", "m_hPlayerPawn"),
            m_b_pawn_is_alive=field("CCSPlayerController", "m_bPawnIsAlive"),
            m_s_sanitized_player_name=field("CCSPlayerController", "m_sSanitizedPlayerName"),
            m_armor_value=field("C_CSPlayerPawn", "m_ArmorValue"),
            m_ang_eye_angles=field("C_CSPlayerPawn", "m_angEyeAngles"),
            m_vec_view_offset=field("C_BaseModelEntity", "m_vecViewOffset"),
            m_model_state=field("CSkeletonInstance", "m_modelState"),
            m_entity_spotted_state=field("C_CSPlayerPawn", "m_entitySpottedState"),
            m_b_spotted=field("EntitySpottedState_t", "m_bSpotted"),
            m_b_spotted_by_mask=field("EntitySpottedState_t", "m_bSpottedByMask"),
            m_b_is_scoped=field("C_CSPlayerPawn", "m_bIsScoped"),
            m_i_shots_fired=field("C_CSPlayerPawn", "m_iShotsFired"),
            m_b_is_defusing=field("C_CSPlayerPawn", "m_bIsDefusing"),
            m_i_ping=field("CCSPlayerController", "m_iPing"),
            m_f_flags=field("C_BaseEntity", "m_fFlags"),
            m_vec_velocity=field("C_BaseEntity", "m_vecVelocity"),
            m_p_weapon_services=field("C_BasePlayerPawn", "m_pWeaponServices"),
            m_h_active_weapon=field("CPlayer_WeaponServices", "m_hActiveWeapon"),
            m_i_clip1=field("C_BasePlayerWeapon", "m_iClip1"),
            m_attribute_manager=field("C_EconEntity", "m_AttributeManager"),
            m_item=field("C_AttributeContainer", "m_Item"),
            m_i_item_definition_index=field("C_EconItemView", "m_iItemDefinitionIndex"),
            m_b_in_reload=field("C_CSWeaponBase", "m_bInReload"),
            m_b_gun_game_immunity=field("C_CSPlayerPawn", "m_bGunGameImmunity"),
            m_b_has_moved_since_spawn=field("C_CSPlayerPawnBase", "m_bHasMovedSinceSpawn"),
            m_h_my_weapons=field("CPlayer_WeaponServices", "m_hMyWeapons"),
            m_p_entity=field("CEntityInstance", "m_pEntity"),
            m_designer_name=field("CEntityIdentity", "m_designerName"),
            m_name=field("CEntityIdentity", "m_name"),
            m_p_collision=field("C_BaseEntity", "m_pCollision"),
            m_vec_mins=field("CCollisionProperty", "m_vecMins"),
            m_vec_maxs=field("CCollisionProperty", "m_vecMaxs"),
            m_vec_surrounding_mins=field("CCollisionProperty", "m_vecSurroundingMins"),
            m_vec_surrounding_maxs=field("CCollisionProperty", "m_vecSurroundingMaxs"),
            m_vec_ladder_dir=field("C_FuncLadder", "m_vecLadderDir"),
            m_vec_local_top=field("C_FuncLadder", "m_vecLocalTop"),
            m_vec_player_mount_top=field("C_FuncLadder", "m_vecPlayerMountPositionTop"),
            m_vec_player_mount_bottom=field("C_FuncLadder", "m_vecPlayerMountPositionBottom"),
            m_fl_auto_ride_speed=field("C_FuncLadder", "m_flAutoRideSpeed"),
            m_b_ladder_disabled=field("C_FuncLadder", "m_bDisabled"),
            m_b_fake_ladder=field("C_FuncLadder", "m_bFakeLadder"),
            m_b_ladder_has_slack=field("C_FuncLadder", "m_bHasSlack"),
        )
