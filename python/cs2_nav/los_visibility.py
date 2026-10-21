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


def _layer_at_ref(
    cmap: "CoordinateMap",
    x: float,
    y: float,
    ref_z: float,
) -> object | None:
    """Ярус навмеша около ref_z (ступеньки, ящики, второй этаж)."""
    node = cmap.layer_at(x, y, ref_z, dz_max=cmap.cfg.drop_height)
    if node is not None:
        return node
    return cmap.nearest_node(x, y, ref_z, max_radius=max(48.0, cmap.cfg.cell * 2))


def _trace_horizontal(
    cmap: "CoordinateMap",
    x: float,
    y: float,
    feet_z: float,
    tx: float,
    ty: float,
    target_z: float,
    *,
    hull_radius: float,
    target_body_radius: float,
) -> bool:
    """2.5D-трасс: ref_z интерполируется от стоп локального до цели (не «ползёт» по нижнему этажу)."""
    dx, dy = tx - x, ty - y
    d = math.hypot(dx, dy)
    stop = d - (hull_radius + target_body_radius + 8.0)
    if stop <= 0.0:
        return True

    steps = max(1, int(stop / 8.0))
    max_gaps = max(3, steps // 5)
    gap = 0
    dz_step = cmap.cfg.drop_height

    for i in range(1, steps + 1):
        k = (i / steps) * (stop / d)
        px = x + dx * k
        py = y + dy * k
        ref_z = feet_z + (target_z - feet_z) * k
        node = cmap.layer_at(px, py, ref_z, dz_max=dz_step)
        if node is None:
            node = cmap.nearest_node(px, py, ref_z, max_radius=cmap.cfg.cell * 2.5)
        if node is None:
            gap += 1
            if gap > max_gaps:
                return False
            continue
        gap = 0
    return True


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

    * Горизонталь — ref_z вдоль луча от стоп стрелка к цели (мультиэтажные карты).
    * Вертикаль — глаза и кость в одном пролёте (|tz − eyes| ≤ 168).
    * Дырки навмеша (1–3 клетки) не считаются стеной. Off-screen OK.
    """
    eyes_z = feet_z + eye_z
    if abs(tz - eyes_z) > 168.0:
        return False

    if _layer_at_ref(cmap, x, y, feet_z) is None and _layer_at_ref(cmap, x, y, feet_z + eye_z * 0.5) is None:
        return False

    # Цель: z стоп противника ≈ под костью; для головы — чуть ниже
    target_feet_z = tz - min(eye_z, 72.0)

    if _trace_horizontal(
        cmap, x, y, feet_z, tx, ty, target_feet_z,
        hull_radius=hull_radius, target_body_radius=target_body_radius,
    ):
        return True

    # Запасной проход: фиксированный ярус стоп стрелка (узкие мосты между ярусами)
    if abs(target_feet_z - feet_z) > cmap.cfg.step_height:
        return _trace_horizontal(
            cmap, x, y, feet_z, tx, ty, feet_z,
            hull_radius=hull_radius, target_body_radius=target_body_radius,
        )
    return False


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
