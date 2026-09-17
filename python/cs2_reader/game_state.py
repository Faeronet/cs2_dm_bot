from __future__ import annotations

from dataclasses import dataclass, field
import math

from cs2_reader.life import LocalLifeInfo


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
    head_position: Vec3 = field(default_factory=Vec3)  # центр головы (кость 6)
    chest_position: Vec3 = field(default_factory=Vec3)  # центр груди (кость 4)
    eye_angles: Vec3 = field(default_factory=Vec3)
    velocity: Vec3 = field(default_factory=Vec3)
    distance: float = 0.0
    vulnerable: bool = True                        # not m_bGunGameImmunity
    spawn_immunity: bool = False                   # m_bGunGameImmunity (DM после респавна)
    has_moved_since_spawn: bool = True             # m_bHasMovedSinceSpawn

    def aim_position(self) -> Vec3:
        """Центр головы для прицела (из кости, не ноги + константа)."""
        return self.head_position


@dataclass
class EnemyInfo:
    """Игрок-цель (DM FFA): координаты всегда читаются из памяти, видимость — отдельный флаг."""

    index: int = 0
    name: str = ""
    health: int = 0
    armor: int = 0
    position: Vec3 = field(default_factory=Vec3)   # ноги (m_vOldOrigin)
    head_position: Vec3 = field(default_factory=Vec3)  # центр головы (кость 6)
    chest_position: Vec3 = field(default_factory=Vec3)  # центр груди (кость 4)
    visible: bool = False                          # m_bSpottedByMask[local] и не dormant
    radar_spotted: bool = False                    # m_bSpotted (радар / кем-то замечен)
    dormant: bool = False                          # если True — position может быть устаревшей
    distance: float = 0.0
    velocity: Vec3 = field(default_factory=Vec3)
    vulnerable: bool = True
    spawn_immunity: bool = False
    has_moved_since_spawn: bool = True

    def aim_position(self) -> Vec3:
        return self.head_position


@dataclass
class WeaponInfo:
    """Активное оружие локального игрока."""

    id: int = 0
    name: str = ""
    clip: int = 0              # патроны в обойме (m_iClip1)
    in_reload: bool = False

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "clip": self.clip,
            "in_reload": self.in_reload,
        }


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
    local_weapon: WeaponInfo = field(default_factory=WeaponInfo)
    local_life: LocalLifeInfo = field(default_factory=LocalLifeInfo)
    screen_width: float = 0.0
    screen_height: float = 0.0
    view_matrix: ViewMatrix | None = None
    players: list[PlayerInfo] = field(default_factory=list)
    enemies: list[EnemyInfo] = field(default_factory=list)
    visible_enemies: list[PlayerInfo] = field(default_factory=list)
    bomb: BombInfo | None = None
    ladders: list = field(default_factory=list)  # list[LadderInfo]
    errors: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        def vec(v: Vec3) -> dict:
            return {"x": v.x, "y": v.y, "z": v.z}

        def enemy_dict(e: EnemyInfo) -> dict:
            return {
                "index": e.index,
                "name": e.name,
                "health": e.health,
                "armor": e.armor,
                "visible": e.visible,
                "radar_spotted": e.radar_spotted,
                "dormant": e.dormant,
                "position": vec(e.position),
                "head_position": vec(e.head_position),
                "chest_position": vec(e.chest_position),
                "aim_position": vec(e.aim_position()),
                "distance": e.distance,
                "velocity": vec(e.velocity),
                "vulnerable": e.vulnerable,
                "spawn_immunity": e.spawn_immunity,
                "has_moved_since_spawn": e.has_moved_since_spawn,
            }

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
                "head_position": vec(p.head_position),
                "chest_position": vec(p.chest_position),
                "eye_angles": vec(p.eye_angles),
                "distance": p.distance,
                "velocity": vec(p.velocity),
                "vulnerable": p.vulnerable,
                "spawn_immunity": p.spawn_immunity,
                "has_moved_since_spawn": p.has_moved_since_spawn,
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
            "local_weapon": self.local_weapon.to_dict(),
            "local_life": self.local_life.to_dict(),
            "screen": {"width": self.screen_width, "height": self.screen_height},
            "players": [player_dict(p) for p in self.players],
            "enemies": [enemy_dict(e) for e in self.enemies],
            "visible_enemies": [player_dict(p) for p in self.visible_enemies],
            "bomb": {"planted": self.bomb.planted, "position": vec(self.bomb.position)} if self.bomb else None,
            "ladders": [lad.to_dict() for lad in self.ladders],
            "errors": self.errors,
        }
