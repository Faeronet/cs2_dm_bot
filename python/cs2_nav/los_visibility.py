"""LOS-видимость врагов: «видим» = кость не за стеной (off-screen OK), «скрыт» = стена."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from cs2_nav.coord_map import CoordinateMap

# Ключи частей тела → поля в dict мишени (как в navigator).
AIM_PART_KEYS: dict[str, str] = {
    "head": "head_position",
    "chest": "chest_position",
    "left_shoulder": "left_shoulder_position",
    "right_shoulder": "right_shoulder_position",
}
AIM_PARTS_ORDER: tuple[str, ...] = ("head", "chest", "left_shoulder", "right_shoulder")


def los_clear(
    cmap: "CoordinateMap",
    x: float,
    y: float,
    feet_z: float,
    tx: float,
    ty: float,
    tz: float,
    *,
    eye_z: float,
    hull_radius: float,
    target_body_radius: float,
) -> bool:
    """Прямая до кости не пересекает стену на карте (2D по навмешу + проверка по Z).

    * Горизонталь — трасс по проходимым клеткам на высоте пола (не на высоте глаз:
      иначе layer_at не находит ярус и всё кажется «за стеной»).
    * Вертикаль — глаза и кость в одном пролёте (|tz − eyes| ≤ 168), лестница не прячет голову.
    * Одна пустая клетка навмеша — не стена. Off-screen не учитывается.
    """
    eyes_z = feet_z + eye_z
    if abs(tz - eyes_z) > 168.0:
        return False

    local_node = cmap.layer_at(x, y, feet_z)
    if local_node is None:
        local_node = cmap.nearest_node(x, y, feet_z, max_radius=48.0)
    if local_node is None:
        return False

    dx, dy = tx - x, ty - y
    d = math.hypot(dx, dy)
    stop = d - (hull_radius + target_body_radius + 8.0)
    if stop <= 0.0:
        return True

    cz = local_node.z
    steps = max(1, int(stop / 8.0))
    gap = 0
    for i in range(1, steps + 1):
        k = (i / steps) * (stop / d)
        node = cmap.layer_at(x + dx * k, y + dy * k, cz, dz_max=cmap.cfg.jump_height)
        if node is None:
            gap += 1
            if gap > 1:
                return False
            continue
        gap = 0
        cz = node.z
    return True


def visible_bone_parts(
    enemy: dict,
    cmap: "CoordinateMap",
    x: float,
    y: float,
    feet_z: float,
    *,
    eye_z: float,
    hull_radius: float,
    target_body_radius: float,
) -> dict[str, bool]:
    """Какие кости видны (LOS), ключи — head/chest/left_shoulder/right_shoulder."""
    if enemy.get("dormant"):
        return {part: False for part in AIM_PARTS_ORDER}
    out: dict[str, bool] = {}
    for part, key in AIM_PART_KEYS.items():
        pos = enemy.get(key)
        if pos is None:
            out[part] = False
            continue
        out[part] = los_clear(
            cmap, x, y, feet_z, pos[0], pos[1], pos[2],
            eye_z=eye_z, hull_radius=hull_radius, target_body_radius=target_body_radius,
        )
    return out


def apply_los_visibility(
    enemy: dict,
    cmap: "CoordinateMap",
    x: float,
    y: float,
    feet_z: float,
    *,
    eye_z: float,
    hull_radius: float,
    target_body_radius: float,
) -> None:
    """Заполнить *_visible и visible в dict мишени (in-place)."""
    parts = visible_bone_parts(
        enemy, cmap, x, y, feet_z,
        eye_z=eye_z, hull_radius=hull_radius, target_body_radius=target_body_radius,
    )
    for part, vis in parts.items():
        enemy[f"{part}_visible"] = vis

    any_bone = any(parts.values())
    if any_bone:
        enemy["visible"] = True
        return

    feet = enemy.get("feet", enemy.get("position"))
    if feet is None:
        enemy["visible"] = False
        return
    fx, fy, fz = feet[0], feet[1], feet[2]
    torso_z = fz + 50.0
    head = enemy.get("head_position")
    if head is not None:
        torso_z = head[2] - 8.0
    clear = los_clear(
        cmap, x, y, feet_z, fx, fy, torso_z,
        eye_z=eye_z, hull_radius=hull_radius, target_body_radius=target_body_radius,
    )
    if clear and enemy.get("chest_position") is not None:
        enemy["chest_visible"] = True
    enemy["visible"] = bool(clear)
