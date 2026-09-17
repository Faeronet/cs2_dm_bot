"""Состояние жизни локального игрока (DM: смерть / респawn)."""

from __future__ import annotations

from dataclasses import dataclass

LIFE_ALIVE = 0
LIFE_DYING = 1
LIFE_DEAD = 2
LIFE_RESPAWNABLE = 3
LIFE_RESPAWNING = 4

LIFE_NAMES: dict[int, str] = {
    LIFE_ALIVE: "alive",
    LIFE_DYING: "dying",
    LIFE_DEAD: "dead",
    LIFE_RESPAWNABLE: "respawnable",
    LIFE_RESPAWNING: "respawning",
}


def life_state_name(state: int) -> str:
    return LIFE_NAMES.get(state, f"life_{state}")


def is_playable(health: int, life_state: int, pawn_alive: bool) -> bool:
    """Можно управлять pawn: в DM после респавна снова True."""
    return health > 0 and life_state == LIFE_ALIVE and pawn_alive


@dataclass
class LocalLifeInfo:
    """Текущее состояние локального игрока (без истории)."""

    alive: bool = False
    health: int = 0
    life_state: int = LIFE_DEAD
    pawn_alive: bool = False          # CCSPlayerController.m_bPawnIsAlive
    pawn: int = 0                     # адрес pawn (0 если нет)
    respawning: bool = False          # m_lifeState == LIFE_RESPAWNING
    vulnerable: bool = True           # not m_bGunGameImmunity
    spawn_immunity: bool = False      # DM: защита после респавна
    has_moved_since_spawn: bool = True

    @property
    def status(self) -> str:
        if self.alive:
            return "alive"
        if self.respawning:
            return "respawning"
        return "dead"

    def to_dict(self) -> dict:
        return {
            "alive": self.alive,
            "health": self.health,
            "life_state": self.life_state,
            "life_state_name": life_state_name(self.life_state),
            "pawn_alive": self.pawn_alive,
            "pawn": hex(self.pawn) if self.pawn else None,
            "respawning": self.respawning,
            "status": self.status,
            "vulnerable": self.vulnerable,
            "spawn_immunity": self.spawn_immunity,
            "has_moved_since_spawn": self.has_moved_since_spawn,
        }


@dataclass
class LifeEvents:
    """Импульсы с прошлого poll_life() — одноразовые флаги на тик опроса."""

    died: bool = False
    respawned: bool = False

    def to_dict(self) -> dict:
        return {"died": self.died, "respawned": self.respawned}


@dataclass
class LocalLifeUpdate:
    """Состояние + события для бота (вызывать каждый тик)."""

    state: LocalLifeInfo
    events: LifeEvents

    @property
    def alive(self) -> bool:
        return self.state.alive

    @property
    def died(self) -> bool:
        return self.events.died

    @property
    def respawned(self) -> bool:
        return self.events.respawned

    def to_dict(self) -> dict:
        d = self.state.to_dict()
        d["events"] = self.events.to_dict()
        return d


class LifeTracker:
    """Сравнивает alive между вызовами poll_life()."""

    def __init__(self) -> None:
        self._init = False
        self._was_alive = False

    def step(self, state: LocalLifeInfo) -> LifeEvents:
        ev = LifeEvents()
        alive = state.alive
        if self._init:
            if self._was_alive and not alive:
                ev.died = True
            if not self._was_alive and alive:
                ev.respawned = True
        else:
            self._init = True
        self._was_alive = alive
        return ev
