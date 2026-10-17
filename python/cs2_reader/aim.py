"""Направление прицела: yaw/pitch → луч и точка в мире."""

from __future__ import annotations

import math

from cs2_reader.game_state import Vec3

BONE_HEAD = 6
BONE_CHEST = 4          # spine_1 / верхняя грудь
BONE_LEFT_SHOULDER = 8  # clavicle_L
BONE_RIGHT_SHOULDER = 13  # clavicle_R
BONE_STRIDE = 32
BONE_ARRAY_IN_MODEL_STATE = 0x80
HEAD_ABOVE_EYES = 4.0  # fallback: центр головы чуть выше m_vecViewOffset (глаза)
CHEST_BELOW_EYES = 8.0  # fallback: грудь чуть ниже m_vecViewOffset (глаза)


def normalize_yaw(yaw: float) -> float:
    while yaw > 180.0:
        yaw -= 360.0
    while yaw < -180.0:
        yaw += 360.0
    return yaw


def shortest_yaw_err(bearing: float, camera_yaw: float) -> float:
    """Разница yaw по кратчайшей дуге (важно у ±180°)."""
    delta = math.radians(normalize_yaw(bearing - camera_yaw))
    return math.degrees(
        math.atan2(math.sin(delta), math.cos(delta)),
    )


def clamp_pitch(pitch: float) -> float:
    return max(-89.0, min(89.0, pitch))


def yaw_to_target(x: float, y: float, tx: float, ty: float) -> float:
    """
    Source bearing (градусы): куда смотреть по горизонтали на (tx, ty).

    Стандарт Source/CS2: yaw=0 → +X, yaw=90 → +Y, yaw=±180 → -X, yaw=-90 → -Y.
    Проверено по логам: при зажатом W и yaw≈-178° игрок шёл строго в -X.
    """
    return normalize_yaw(math.degrees(math.atan2(ty - y, tx - x)))


def forward_from_angles(pitch: float, yaw: float) -> Vec3:
    """Единичный вектор взгляда. pitch=X (вниз +), yaw=Y (стандарт Source)."""
    yr = math.radians(yaw)
    pr = math.radians(pitch)
    cp = math.cos(pr)
    sp = math.sin(pr)
    return Vec3(math.cos(yr) * cp, math.sin(yr) * cp, -sp)


def eye_position(feet: Vec3, view_offset_z: float = 64.0) -> Vec3:
    return Vec3(feet.x, feet.y, feet.z + view_offset_z)


def head_center_from_view(feet: Vec3, view_offset: Vec3) -> Vec3:
    """Оценка центра головы, если кость недоступна: m_vecViewOffset + небольшой подъём."""
    return Vec3(
        feet.x + view_offset.x,
        feet.y + view_offset.y,
        feet.z + view_offset.z + HEAD_ABOVE_EYES,
    )


def head_plausible(feet: Vec3, head: Vec3) -> bool:
    dz = head.z - feet.z
    if dz < 50.0 or dz > 92.0:
        return False
    if math.hypot(head.x - feet.x, head.y - feet.y) > 48.0:
        return False
    return True


def chest_center_from_view(feet: Vec3, view_offset: Vec3) -> Vec3:
    """Оценка центра груди, если кость недоступна."""
    return Vec3(
        feet.x + view_offset.x,
        feet.y + view_offset.y,
        feet.z + view_offset.z - CHEST_BELOW_EYES,
    )


def chest_plausible(feet: Vec3, chest: Vec3, head: Vec3 | None = None) -> bool:
    dz = chest.z - feet.z
    if dz < 28.0 or dz > 58.0:
        return False
    if math.hypot(chest.x - feet.x, chest.y - feet.y) > 48.0:
        return False
    if head is not None and chest.z >= head.z - 4.0:
        return False
    return True


def shoulder_plausible(
    feet: Vec3,
    shoulder: Vec3,
    chest: Vec3 | None = None,
    head: Vec3 | None = None,
) -> bool:
    """Кость плеча (clavicle): между грудью и головой, чуть шире по XY."""
    dz = shoulder.z - feet.z
    if dz < 38.0 or dz > 82.0:
        return False
    if math.hypot(shoulder.x - feet.x, shoulder.y - feet.y) > 58.0:
        return False
    if chest is not None and shoulder.z < chest.z - 6.0:
        return False
    if head is not None and shoulder.z > head.z + 4.0:
        return False
    return True


def aim_point(
    feet: Vec3,
    pitch: float,
    yaw: float,
    distance: float = 8192.0,
    view_offset_z: float = 64.0,
    ground_z: float | None = None,
) -> Vec3:
    """Точка в мире, куда смотрит прицел (луч из глаз)."""
    origin = eye_position(feet, view_offset_z)
    fwd = forward_from_angles(pitch, yaw)

    if ground_z is not None and fwd.z < -0.05:
        t = (ground_z - origin.z) / fwd.z
        if t > 0:
            return Vec3(
                origin.x + fwd.x * t,
                origin.y + fwd.y * t,
                ground_z,
            )

    return Vec3(
        origin.x + fwd.x * distance,
        origin.y + fwd.y * distance,
        origin.z + fwd.z * distance,
    )
