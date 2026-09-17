"""Лестницы на карте (C_FuncLadder) из entity system."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from cs2_reader.game_state import Vec3

LADDER_CLASS_NAMES = frozenset({"func_ladder", "c_funcladder"})


@dataclass
class LadderInfo:
    """func_ladder: точки крепления, направление и габариты из collision AABB."""

    index: int = 0
    name: str = ""
    bottom: Vec3 = field(default_factory=Vec3)       # m_vecPlayerMountPositionBottom
    top: Vec3 = field(default_factory=Vec3)          # m_vecPlayerMountPositionTop
    center: Vec3 = field(default_factory=Vec3)
    direction: Vec3 = field(default_factory=Vec3)    # m_vecLadderDir (нормализован)
    local_top: Vec3 = field(default_factory=Vec3)  # m_vecLocalTop
    height: float = 0.0
    width: float = 0.0
    depth: float = 0.0
    bounds_mins: Vec3 = field(default_factory=Vec3)
    bounds_maxs: Vec3 = field(default_factory=Vec3)
    disabled: bool = False
    fake: bool = False
    has_slack: bool = False
    auto_ride_speed: float = 0.0
    distance: float = 0.0

    def to_dict(self) -> dict:
        def vec(v: Vec3) -> dict:
            return {"x": v.x, "y": v.y, "z": v.z}

        return {
            "index": self.index,
            "name": self.name,
            "bottom": vec(self.bottom),
            "top": vec(self.top),
            "center": vec(self.center),
            "direction": vec(self.direction),
            "local_top": vec(self.local_top),
            "height": round(self.height, 2),
            "width": round(self.width, 2),
            "depth": round(self.depth, 2),
            "bounds_mins": vec(self.bounds_mins),
            "bounds_maxs": vec(self.bounds_maxs),
            "disabled": self.disabled,
            "fake": self.fake,
            "has_slack": self.has_slack,
            "auto_ride_speed": round(self.auto_ride_speed, 2),
            "distance": round(self.distance, 1),
        }


def _norm3(x: float, y: float, z: float) -> Vec3:
    ln = math.sqrt(x * x + y * y + z * z)
    if ln < 1e-6:
        return Vec3(0.0, 0.0, 1.0)
    return Vec3(x / ln, y / ln, z / ln)


def _plausible_ladder(bottom: Vec3, top: Vec3, direction: Vec3) -> bool:
    h = bottom.distance_to(top)
    if h < 24.0 or h > 4096.0:
        return False
    if max(abs(bottom.x), abs(bottom.y), abs(top.x), abs(top.y)) > 65536.0:
        return False
    if abs(bottom.z) > 65536.0 or abs(top.z) > 65536.0:
        return False
    dz = top.z - bottom.z
    if abs(dz) < 8.0:
        return False
    # направление должно быть близко к top-bottom
    dot = direction.x * (top.x - bottom.x) + direction.y * (top.y - bottom.y) + direction.z * (top.z - bottom.z)
    if dot < h * 0.35:
        return False
    return True


def entity_designer_name(backend, schema, entity: int) -> str:
    if not entity or not schema.m_p_entity:
        return ""
    identity = backend.read_u64(entity + schema.m_p_entity)
    if not identity:
        return ""
    for off in (schema.m_designer_name, schema.m_name):
        if not off:
            continue
        ptr = backend.read_u64(identity + off)
        if ptr and ptr > 0x10000:
            name = backend.read_string(ptr, 64).strip()
            if name:
                return name.lower()
        inline = backend.read_string(identity + off, 64).strip()
        if inline and inline.isprintable() and " " not in inline:
            return inline.lower()
    return ""


def _collision_bounds(backend, schema, entity: int) -> tuple[Vec3, Vec3] | None:
    if not schema.m_p_collision:
        return None
    col = backend.read_u64(entity + schema.m_p_collision)
    if not col:
        return None
    if schema.m_vec_surrounding_mins and schema.m_vec_surrounding_maxs:
        mins = backend.read_vec3(col + schema.m_vec_surrounding_mins)
        maxs = backend.read_vec3(col + schema.m_vec_surrounding_maxs)
        if mins and maxs:
            return Vec3(*mins), Vec3(*maxs)
    if schema.m_vec_mins and schema.m_vec_maxs:
        mins = backend.read_vec3(col + schema.m_vec_mins)
        maxs = backend.read_vec3(col + schema.m_vec_maxs)
        if mins and maxs:
            origin = Vec3()
            scene = backend.read_u64(entity + schema.m_p_game_scene_node) if schema.m_p_game_scene_node else 0
            if scene:
                org = backend.read_vec3(scene + schema.m_vec_abs_origin)
                if org:
                    origin = Vec3(*org)
            return (
                Vec3(origin.x + mins[0], origin.y + mins[1], origin.z + mins[2]),
                Vec3(origin.x + maxs[0], origin.y + maxs[1], origin.z + maxs[2]),
            )
    return None


def _size_from_bounds(mins: Vec3, maxs: Vec3) -> tuple[float, float, float]:
    sx = abs(maxs.x - mins.x)
    sy = abs(maxs.y - mins.y)
    sz = abs(maxs.z - mins.z)
    horiz = sorted((sx, sy))
    width = horiz[1]
    depth = horiz[0]
    height = sz
    return width, depth, height


def read_ladder(
    backend,
    schema,
    entity: int,
    index: int,
    local_pos: Vec3,
    *,
    name: str = "",
) -> LadderInfo | None:
    if not schema.m_vec_ladder_dir:
        return None
    bottom_raw = backend.read_vec3(entity + schema.m_vec_player_mount_bottom)
    top_raw = backend.read_vec3(entity + schema.m_vec_player_mount_top)
    dir_raw = backend.read_vec3(entity + schema.m_vec_ladder_dir)
    if not bottom_raw or not top_raw or not dir_raw:
        return None

    bottom = Vec3(*bottom_raw)
    top = Vec3(*top_raw)
    direction = _norm3(*dir_raw)
    if not _plausible_ladder(bottom, top, direction):
        return None

    local_top = Vec3()
    if schema.m_vec_local_top:
        lt = backend.read_vec3(entity + schema.m_vec_local_top)
        if lt:
            local_top = Vec3(*lt)

    disabled = bool(backend.read_u8(entity + schema.m_b_ladder_disabled)) if schema.m_b_ladder_disabled else False
    fake = bool(backend.read_u8(entity + schema.m_b_fake_ladder)) if schema.m_b_fake_ladder else False
    has_slack = bool(backend.read_u8(entity + schema.m_b_ladder_has_slack)) if schema.m_b_ladder_has_slack else False
    auto_ride = (
        float(backend.read_f32(entity + schema.m_fl_auto_ride_speed) or 0.0)
        if schema.m_fl_auto_ride_speed
        else 0.0
    )

    bounds = _collision_bounds(backend, schema, entity)
    if bounds:
        bounds_mins, bounds_maxs = bounds
        width, depth, bound_h = _size_from_bounds(bounds_mins, bounds_maxs)
    else:
        bounds_mins = Vec3(
            min(bottom.x, top.x) - 16.0,
            min(bottom.y, top.y) - 16.0,
            min(bottom.z, top.z),
        )
        bounds_maxs = Vec3(
            max(bottom.x, top.x) + 16.0,
            max(bottom.y, top.y) + 16.0,
            max(bottom.z, top.z),
        )
        width, depth, bound_h = _size_from_bounds(bounds_mins, bounds_maxs)

    height = bottom.distance_to(top)
    center = Vec3(
        (bottom.x + top.x) * 0.5,
        (bottom.y + top.y) * 0.5,
        (bottom.z + top.z) * 0.5,
    )

    return LadderInfo(
        index=index,
        name=name or "func_ladder",
        bottom=bottom,
        top=top,
        center=center,
        direction=direction,
        local_top=local_top,
        height=height,
        width=max(width, 8.0),
        depth=max(depth, 8.0) if depth > 1.0 else max(bound_h, 8.0),
        bounds_mins=bounds_mins,
        bounds_maxs=bounds_maxs,
        disabled=disabled,
        fake=fake,
        has_slack=has_slack,
        auto_ride_speed=auto_ride,
        distance=center.distance_to(local_pos),
    )


def is_ladder_entity(class_name: str) -> bool:
    if not class_name:
        return False
    c = class_name.lower()
    if c in LADDER_CLASS_NAMES:
        return True
    return "func_ladder" in c or c.endswith("funcladder")
