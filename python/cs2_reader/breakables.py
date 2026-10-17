"""Ломаемые entity на карте (func_breakable, решётки, окна) из entity system."""

from __future__ import annotations

from dataclasses import dataclass, field

from cs2_reader.game_state import Vec3
from cs2_reader.ladders import _collision_bounds, entity_designer_name
from cs2_reader.life import LIFE_ALIVE, life_state_name

_MAX_PTR = 0x00007FFFFFFFFFFF
_SOLID_VPHYSICS = 6

BREAKABLE_NAME_HINTS = (
    "func_breakable",
    "func_breakable_surf",
    "breakable",
    "grate",
    "grating",
    "cagedoor",
    "cage_door",
    "glass",
    "window",
    "fence",
)

BREAKABLE_KIND_HINTS = {
    "grate": ("grate", "grating", "fence", "cage", "shutter"),
    "window": ("window", "glass"),
    "breakable": ("func_breakable", "breakable"),
}

BREAKABLE_MODEL_HINTS = (
    "breakable",
    "grate",
    "grating",
    "window",
    "shutter",
    "cage",
    "fence",
    "glass",
    "chainlink",
)

MAP_ENTITY_SKIP = (
    "func_bomb_target",
    "post_processing",
    "/a site_",
    "/b site_",
    "bomb_target",
)

BREAKABLE_DESIGNER_SKIP = (
    "func_bomb_target",
    "post_processing_volume",
    "weapon_",
    "cs_player",
    "point_",
    "info_",
    "env_",
    "team_",
    "vote_",
    "sky_",
    "precache",
    "camera",
    "lineup",
)


@dataclass
class BreakableInfo:
    """Ломаемый prop: где стоит, цел ли, можно ли пройти после разрушения."""

    index: int = 0
    name: str = ""
    kind: str = "breakable"
    model: str = ""
    position: Vec3 = field(default_factory=Vec3)
    angles: Vec3 = field(default_factory=Vec3)
    health: int = 0
    max_health: int = 0
    life_state: int = 0
    life: str = "alive"
    broken: bool = False
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
            "kind": self.kind,
            "model": self.model,
            "position": vec(self.position),
            "angles": vec(self.angles),
            "health": self.health,
            "max_health": self.max_health,
            "life_state": self.life_state,
            "life": self.life,
            "broken": self.broken,
            "passable": self.passable,
            "width": round(self.width, 2),
            "height": round(self.height, 2),
            "depth": round(self.depth, 2),
            "bounds_mins": vec(self.bounds_mins),
            "bounds_maxs": vec(self.bounds_maxs),
            "distance": round(self.distance, 1),
        }


def _usable_ptr(ptr: int | None) -> int | None:
    if not ptr or ptr < 0x10000 or ptr > _MAX_PTR:
        return None
    return ptr


def _size_from_bounds(mins: Vec3, maxs: Vec3) -> tuple[float, float, float]:
    sx = abs(maxs.x - mins.x)
    sy = abs(maxs.y - mins.y)
    sz = abs(maxs.z - mins.z)
    horiz = sorted((sx, sy))
    return horiz[1], horiz[0], sz


def _valid_bounds(mins: Vec3, maxs: Vec3) -> bool:
    w, d, h = _size_from_bounds(mins, maxs)
    if w < 1.0 and d < 1.0 and h < 1.0:
        return False
    return w <= 512.0 and d <= 512.0 and h <= 512.0


def _kind_from_text(*parts: str) -> str:
    text = " ".join(p for p in parts if p).lower()
    for kind, hints in BREAKABLE_KIND_HINTS.items():
        if any(h in text for h in hints):
            return kind
    return "breakable"


def entity_model_path(backend, schema, entity: int) -> str:
    if not schema.m_p_game_scene_node or not schema.m_model_state or not schema.m_model_name:
        return ""
    scene = _usable_ptr(backend.read_u64(entity + schema.m_p_game_scene_node))
    if not scene:
        return ""
    name_ptr = _usable_ptr(backend.read_u64(scene + schema.m_model_state + schema.m_model_name))
    if name_ptr:
        model = backend.read_string(name_ptr, 160)
        if model:
            return model.strip()
    inline = backend.read_string(scene + schema.m_model_state + schema.m_model_name, 160)
    if inline and inline.isprintable() and "vmdl" in inline.lower():
        return inline.strip()
    return ""


def is_map_entity_model(model: str) -> bool:
    path = model.lower().replace("\\", "/")
    if not path.startswith("maps/") or "/entities/" not in path:
        return False
    return not any(skip in path for skip in MAP_ENTITY_SKIP)


def is_breakable_model(model: str) -> bool:
    if not model:
        return False
    path = model.lower().replace("\\", "/")
    if is_map_entity_model(model):
        return True
    return any(h in path for h in BREAKABLE_MODEL_HINTS)


def _designer_skipped(name: str) -> bool:
    if not name:
        return False
    c = name.lower()
    return any(c.startswith(s) or s in c for s in BREAKABLE_DESIGNER_SKIP)


def is_breakable_entity(class_name: str) -> bool:
    if not class_name:
        return False
    c = class_name.lower()
    if _designer_skipped(c):
        return False
    if any(x in c for x in ("unbreakable", "point_", "info_", "weapon_", "cs_player")):
        return False
    if any(h in c for h in BREAKABLE_NAME_HINTS):
        return True
    if c.startswith("prop_dynamic") and any(h in c for h in ("break", "grate", "glass", "window", "cage")):
        return True
    return False


def entity_has_break_pieces(backend, schema, entity: int) -> bool:
    if not schema.m_b_has_break_pieces:
        return False
    flag = backend.read_u8(entity + schema.m_b_has_break_pieces)
    if flag != 1:
        return False
    health = backend.read_i32(entity + schema.m_i_health)
    max_health = backend.read_i32(entity + schema.m_i_max_health) if schema.m_i_max_health else 0
    return health is not None and health > 0 and (max_health or 0) > 0


def entity_blocks_movement(backend, schema, entity: int, bounds: tuple[Vec3, Vec3] | None) -> bool:
    if not schema.m_p_collision or not schema.m_n_solid_type:
        return bounds is not None
    col = _usable_ptr(backend.read_u64(entity + schema.m_p_collision))
    if not col:
        return False
    solid = backend.read_u8(col + schema.m_n_solid_type)
    if solid != _SOLID_VPHYSICS:
        return False
    return bounds is not None and _valid_bounds(bounds[0], bounds[1])


def is_breakable_candidate(backend, schema, entity: int, *, name: str = "") -> bool:
    designer = name or entity_designer_name(backend, schema, entity)
    if _designer_skipped(designer):
        return False
    if is_breakable_entity(designer):
        return True
    model = entity_model_path(backend, schema, entity)
    if is_breakable_model(model):
        return True
    if entity_has_break_pieces(backend, schema, entity):
        return True
    return False


def read_breakable(
    backend,
    schema,
    entity: int,
    index: int,
    local_pos: Vec3,
    *,
    name: str = "",
) -> BreakableInfo | None:
    if not schema.m_p_game_scene_node:
        return None
    scene = backend.read_u64(entity + schema.m_p_game_scene_node)
    if not scene:
        return None
    origin_raw = backend.read_vec3(scene + schema.m_vec_abs_origin)
    if not origin_raw:
        return None
    position = Vec3(*origin_raw)
    if max(abs(position.x), abs(position.y)) > 65536.0 or abs(position.z) > 65536.0:
        return None

    angles = Vec3()
    if schema.m_ang_abs_rotation:
        ang_raw = backend.read_vec3(scene + schema.m_ang_abs_rotation)
        if ang_raw:
            angles = Vec3(*ang_raw)

    designer = name or entity_designer_name(backend, schema, entity)
    model = entity_model_path(backend, schema, entity)
    display_name = designer or (model.rsplit("/", 1)[-1] if model else "breakable")

    health = int(backend.read_i32(entity + schema.m_i_health) or 0)
    max_health = int(backend.read_i32(entity + schema.m_i_max_health) or 0) if schema.m_i_max_health else 0
    life_state = int(backend.read_u8(entity + schema.m_life_state) or 0)

    bounds = _collision_bounds(backend, schema, entity)
    if bounds and not _valid_bounds(bounds[0], bounds[1]):
        bounds = None

    blocks = entity_blocks_movement(backend, schema, entity, bounds)

    if bounds:
        bounds_mins, bounds_maxs = bounds
        width, depth, height = _size_from_bounds(bounds_mins, bounds_maxs)
    else:
        bounds_mins = Vec3(position.x - 16.0, position.y - 16.0, position.z)
        bounds_maxs = Vec3(position.x + 16.0, position.y + 16.0, position.z + 64.0)
        width, depth, height = _size_from_bounds(bounds_mins, bounds_maxs)

    if max_health > 0:
        broken = health <= 0 or life_state != LIFE_ALIVE
    elif blocks:
        broken = False
    elif model and is_map_entity_model(model):
        # de_mirage: визуальная часть решётки без solid=6 — не «сломана», просто без коллизии
        broken = False
    else:
        broken = health <= 0 or life_state != LIFE_ALIVE

    return BreakableInfo(
        index=index,
        name=display_name,
        kind=_kind_from_text(designer, model),
        model=model,
        position=position,
        angles=angles,
        health=health,
        max_health=max_health,
        life_state=life_state,
        life=life_state_name(life_state),
        broken=broken,
        passable=broken or not blocks,
        width=max(width, 4.0),
        depth=max(depth, 4.0),
        height=max(height, 4.0),
        bounds_mins=bounds_mins,
        bounds_maxs=bounds_maxs,
        distance=position.distance_to(local_pos),
    )


def merge_breakables(items: list[BreakableInfo]) -> list[BreakableInfo]:
    """Слить дубликаты (визуал + коллизия) по позиции; блокировка важнее."""
    by_pos: dict[tuple[int, int, int], BreakableInfo] = {}
    for item in items:
        key = (
            round(item.position.x),
            round(item.position.y),
            round(item.position.z),
        )
        prev = by_pos.get(key)
        if prev is None:
            by_pos[key] = item
            continue
        if not item.passable:
            prev.passable = False
            prev.broken = False
        if item.model and not prev.model:
            prev.model = item.model
        if len(item.name) > len(prev.name):
            prev.name = item.name
        if item.width * item.height > prev.width * prev.height:
            prev.width, prev.depth, prev.height = item.width, item.depth, item.height
            prev.bounds_mins = item.bounds_mins
            prev.bounds_maxs = item.bounds_maxs
    return list(by_pos.values())
