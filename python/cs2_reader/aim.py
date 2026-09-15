"""Направление прицела: yaw/pitch → луч и точка в мире."""

from __future__ import annotations

import math

from cs2_reader.game_state import Vec3


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
