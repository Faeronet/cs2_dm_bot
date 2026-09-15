from __future__ import annotations

from dataclasses import dataclass, field
import math


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def distance_to(self, other: "Vec3") -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)


@dataclass
class ViewMatrix:
    m: list[list[float]] = field(default_factory=lambda: [[0.0] * 4 for _ in range(4)])

    def world_to_screen(self, world: Vec3, screen_w: float, screen_h: float) -> tuple[float, float] | None:
        w = (
            self.m[3][0] * world.x
            + self.m[3][1] * world.y
            + self.m[3][2] * world.z
            + self.m[3][3]
        )
        if w < 0.001:
            return None
        inv_w = 1.0 / w
        x = (self.m[0][0] * world.x + self.m[0][1] * world.y + self.m[0][2] * world.z + self.m[0][3]) * inv_w
        y = (self.m[1][0] * world.x + self.m[1][1] * world.y + self.m[1][2] * world.z + self.m[1][3]) * inv_w
        return (screen_w * 0.5 + x * screen_w * 0.5, screen_h * 0.5 - y * screen_h * 0.5)


@dataclass
class PlayerInfo:
    index: int = 0
    controller: int = 0
    pawn: int = 0
    name: str = ""
    team: int = 0
    health: int = 0
    armor: int = 0
    alive: bool = False
    dormant: bool = False
    spotted: bool = False
    is_local: bool = False
    ping: int = 0
    scoped: bool = False
    defusing: bool = False
    shots_fired: int = 0
    flags: int = 0
    position: Vec3 = field(default_factory=Vec3)
    eye_angles: Vec3 = field(default_factory=Vec3)
    distance: float = 0.0

    def aim_position(self, z_offset: float = 64.0) -> Vec3:
        """Позиция для прицела: ноги из памяти + смещение по Z (уровень глаз)."""
        return Vec3(self.position.x, self.position.y, self.position.z + z_offset)


@dataclass
class BombInfo:
    planted: bool = False
    position: Vec3 = field(default_factory=Vec3)


@dataclass
class GameSnapshot:
    platform: str = ""
    in_game: bool = False
    in_match: bool = False
    map_name: str = ""
    build_number: int = 0
    client_base: int = 0
    engine_base: int = 0
    local_index: int = -1
    local_team: int = 0
    local_position: Vec3 = field(default_factory=Vec3)
    local_eye_angles: Vec3 = field(default_factory=Vec3)
    local_aim_forward: Vec3 = field(default_factory=Vec3)
    local_aim_point: Vec3 = field(default_factory=Vec3)
    screen_width: float = 0.0
    screen_height: float = 0.0
    view_matrix: ViewMatrix | None = None
    players: list[PlayerInfo] = field(default_factory=list)
    visible_enemies: list[PlayerInfo] = field(default_factory=list)
    bomb: BombInfo | None = None
    errors: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        def vec(v: Vec3) -> dict:
            return {"x": v.x, "y": v.y, "z": v.z}

        def player_dict(p: PlayerInfo) -> dict:
            return {
                "index": p.index,
                "name": p.name,
                "team": p.team,
                "health": p.health,
                "armor": p.armor,
                "alive": p.alive,
                "dormant": p.dormant,
                "spotted": p.spotted,
                "is_local": p.is_local,
                "ping": p.ping,
                "scoped": p.scoped,
                "defusing": p.defusing,
                "shots_fired": p.shots_fired,
                "position": vec(p.position),
                "eye_angles": vec(p.eye_angles),
                "distance": p.distance,
            }

        return {
            "platform": self.platform,
            "in_game": self.in_game,
            "in_match": self.in_match,
            "map_name": self.map_name,
            "build_number": self.build_number,
            "client_base": hex(self.client_base),
            "engine_base": hex(self.engine_base),
            "local_index": self.local_index,
            "local_team": self.local_team,
            "local_position": vec(self.local_position),
            "local_eye_angles": vec(self.local_eye_angles),
            "local_aim_forward": vec(self.local_aim_forward),
            "local_aim_point": vec(self.local_aim_point),
            "screen": {"width": self.screen_width, "height": self.screen_height},
            "players": [player_dict(p) for p in self.players],
            "visible_enemies": [player_dict(p) for p in self.visible_enemies],
            "bomb": {"planted": self.bomb.planted, "position": vec(self.bomb.position)} if self.bomb else None,
            "errors": self.errors,
        }
