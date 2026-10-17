"""Двери на карте (prop_door_rotating, func_door) из entity system."""

from __future__ import annotations

from dataclasses import dataclass, field

from cs2_reader.game_state import Vec3
from cs2_reader.ladders import _collision_bounds, entity_designer_name

DOOR_STATE_NAMES = {
    0: "closed",
    1: "opening",
    2: "open",
    3: "closing",
    4: "ajar",
}

DOOR_DESIGNER_HINTS = (
    "prop_door_rotating",
    "prop_door",
    "func_door",
)


@dataclass
class DoorInfo:
    """Дверь: положение, габариты, состояние и можно ли пройти."""

    index: int = 0
    name: str = ""
    position: Vec3 = field(default_factory=Vec3)
    closed_position: Vec3 = field(default_factory=Vec3)
    angles: Vec3 = field(default_factory=Vec3)          # m_angAbsRotation (pitch, yaw, roll)
    closed_angles: Vec3 = field(default_factory=Vec3)
    state: str = "closed"
    state_id: int = 0
    locked: bool = False
    usable: bool = True
    open: bool = False
    passable: bool = False
    width: float = 0.0
    height: float = 0.0
    depth: float = 0.0
    bounds_mins: Vec3 = field(default_factory=Vec3)
    bounds_maxs: Vec3 = field(default_factory=Vec3)
    distance: float = 0.0

    def to_dict(self) -> dict:
        def vec(v: Vec3) -> dict:
            return {"x": v.x, "y": v.y, "z": v.z}

        return {
            "index": self.index,
            "name": self.name,
            "position": vec(self.position),
            "closed_position": vec(self.closed_position),
            "angles": vec(self.angles),
            "closed_angles": vec(self.closed_angles),
            "state": self.state,
            "state_id": self.state_id,
            "locked": self.locked,
            "usable": self.usable,
            "open": self.open,
            "passable": self.passable,
            "width": round(self.width, 2),
            "height": round(self.height, 2),
            "depth": round(self.depth, 2),
            "bounds_mins": vec(self.bounds_mins),
            "bounds_maxs": vec(self.bounds_maxs),
            "distance": round(self.distance, 1),
        }


def _size_from_bounds(mins: Vec3, maxs: Vec3) -> tuple[float, float, float]:
    sx = abs(maxs.x - mins.x)
    sy = abs(maxs.y - mins.y)
    sz = abs(maxs.z - mins.z)
    horiz = sorted((sx, sy))
    width = horiz[1]
    depth = horiz[0]
    height = sz
    return width, depth, height


def _valid_bounds(mins: Vec3, maxs: Vec3) -> bool:
    w, d, h = _size_from_bounds(mins, maxs)
    if w < 1.0 and d < 1.0 and h < 1.0:
        return False
    if w > 512.0 or d > 512.0 or h > 512.0:
        return False
    return True


def _plausible_door(position: Vec3, bounds: tuple[Vec3, Vec3] | None) -> bool:
    if max(abs(position.x), abs(position.y)) > 65536.0 or abs(position.z) > 65536.0:
        return False
    if bounds:
        mins, maxs = bounds
        if _valid_bounds(mins, maxs):
            w, d, _h = _size_from_bounds(mins, maxs)
            if w < 4.0 and d < 4.0:
                return False
    return True


def door_passable(state_id: int, locked: bool, usable: bool = True) -> bool:
    if locked or not usable:
        return False
    return state_id in (2, 4)  # open, ajar


def is_door_entity(class_name: str) -> bool:
    if not class_name:
        return False
    c = class_name.lower()
    if "door" not in c or "indoor" in c:
        return False
    if any(h in c for h in DOOR_DESIGNER_HINTS) or c.endswith("_door"):
        return True
    # Hammer-имя на de_cache и др.: «…_a door»
    return c.endswith(" door") or c.endswith("_door")


def read_door(
    backend,
    schema,
    entity: int,
    index: int,
    local_pos: Vec3,
    *,
    name: str = "",
) -> DoorInfo | None:
    if not schema.m_p_game_scene_node:
        return None
    scene = backend.read_u64(entity + schema.m_p_game_scene_node)
    if not scene:
        return None
    origin_raw = backend.read_vec3(scene + schema.m_vec_abs_origin)
    if not origin_raw:
        return None
    position = Vec3(*origin_raw)

    angles = Vec3()
    if schema.m_ang_abs_rotation:
        ang_raw = backend.read_vec3(scene + schema.m_ang_abs_rotation)
        if ang_raw:
            angles = Vec3(*ang_raw)

    state_id = 0
    has_prop_state = bool(schema.m_e_door_state)
    if has_prop_state:
        state_id = int(backend.read_u32(entity + schema.m_e_door_state) or 0)
    state = DOOR_STATE_NAMES.get(state_id, f"unknown_{state_id}")

    locked = bool(backend.read_u8(entity + schema.m_b_door_locked)) if schema.m_b_door_locked else False
    usable = True
    # m_bIsUsable — поле C_BaseDoor; у prop_door по тому же offset мусор
    if not has_prop_state and schema.m_b_door_usable:
        usable = bool(backend.read_u8(entity + schema.m_b_door_usable))

    closed_position = Vec3()
    if schema.m_closed_position:
        cp = backend.read_vec3(entity + schema.m_closed_position)
        if cp:
            closed_position = Vec3(*cp)

    closed_angles = Vec3()
    if schema.m_closed_angles:
        ca = backend.read_vec3(entity + schema.m_closed_angles)
        if ca:
            closed_angles = Vec3(*ca)

    bounds = _collision_bounds(backend, schema, entity)
    if bounds and not _valid_bounds(bounds[0], bounds[1]):
        bounds = None
    if not _plausible_door(position, bounds):
        return None

    if bounds:
        bounds_mins, bounds_maxs = bounds
        width, depth, height = _size_from_bounds(bounds_mins, bounds_maxs)
    else:
        bounds_mins = Vec3(position.x - 20.0, position.y - 4.0, position.z)
        bounds_maxs = Vec3(position.x + 20.0, position.y + 4.0, position.z + 96.0)
        width, depth, height = _size_from_bounds(bounds_mins, bounds_maxs)

    open_ = state_id in (2, 4)
    passable = door_passable(state_id, locked, usable)

    return DoorInfo(
        index=index,
        name=name or "door",
        position=position,
        closed_position=closed_position,
        angles=angles,
        closed_angles=closed_angles,
        state=state,
        state_id=state_id,
        locked=locked,
        usable=usable,
        open=open_,
        passable=passable,
        width=max(width, 8.0),
        depth=max(depth, 4.0),
        height=max(height, 8.0),
        bounds_mins=bounds_mins,
        bounds_maxs=bounds_maxs,
        distance=position.distance_to(local_pos),
    )
