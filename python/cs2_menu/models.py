"""Модели состояния UI и действий меню CS2."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable


class MenuPhase(str, Enum):
    UNKNOWN = "unknown"
    MAIN_MENU = "main_menu"
    PLAY_HUB = "play_hub"
    MODE_DEATHMATCH = "mode_deathmatch"
    MODE_CASUAL = "mode_casual"
    MODE_PREMIER = "mode_premier"
    MATCHMAKING = "matchmaking"
    ACCEPT_MATCH = "accept_match"
    LOADING = "loading"
    TEAM_SELECT = "team_select"
    IN_GAME = "in_game"
    PAUSE_MENU = "pause_menu"
    POPUP = "popup"


class ActionKind(str, Enum):
    CLICK = "click"
    KEY = "key"
    WAIT = "wait"
    TYPE_TEXT = "type_text"


@dataclass(frozen=True)
class NormPoint:
    """Нормализованная точка в client area окна (0..1)."""

    x: float
    y: float


@dataclass
class UIAction:
    kind: ActionKind
    target: str = ""
    hold_ms: int = 0
    repeat: int = 1

    @staticmethod
    def click(target: str, *, repeat: int = 1) -> UIAction:
        return UIAction(ActionKind.CLICK, target=target, repeat=repeat)

    @staticmethod
    def key(name: str, *, repeat: int = 1, hold_ms: int = 0) -> UIAction:
        return UIAction(ActionKind.KEY, target=name, repeat=repeat, hold_ms=hold_ms)

    @staticmethod
    def wait(ms: int) -> UIAction:
        return UIAction(ActionKind.WAIT, hold_ms=ms)

    @staticmethod
    def type_text(text: str) -> UIAction:
        return UIAction(ActionKind.TYPE_TEXT, target=text)


@dataclass
class PopupSignature:
    """Сигнатура всплывающего окна / уведомления."""

    id: str
    description: str = ""
    # confidence 0..1
    match: Callable[["UIState"], float] = lambda _s: 0.0
    dismiss: list[UIAction] = field(default_factory=list)
    priority: int = 0


@dataclass
class UIState:
    phase: MenuPhase = MenuPhase.UNKNOWN
    in_game: bool = False
    in_match: bool = False
    map_name: str = ""
    matchmaking_map: str = ""
    local_team: int = 0
    local_alive: bool = False
    local_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    player_count: int = 0
    entity_counts: dict[str, int] = field(default_factory=dict)
    menu_context: int = 0
    ui_fingerprint: str = ""
    window_memory: tuple[int, int] = (0, 0)
    window_screen: dict[str, int] = field(default_factory=dict)
    screen_id: str = "unknown"
    screen_confidence: float = 0.0
    region_hashes: dict[str, str] = field(default_factory=dict)
    active_popups: list[str] = field(default_factory=list)
    raw: dict = field(default_factory=dict)

    def at_main_menu(self) -> bool:
        return self.phase in (MenuPhase.MAIN_MENU, MenuPhase.PLAY_HUB) and not self.in_game

    def in_map(self) -> bool:
        return self.in_game and bool(self.map_name) and self.map_name not in ("<empty>", "")

    def to_dict(self) -> dict:
        return {
            "phase": self.phase.value,
            "in_game": self.in_game,
            "in_match": self.in_match,
            "map_name": self.map_name,
            "matchmaking_map": self.matchmaking_map,
            "local_team": self.local_team,
            "local_alive": self.local_alive,
            "local_pos": self.local_pos,
            "player_count": self.player_count,
            "entity_counts": self.entity_counts,
            "menu_context": hex(self.menu_context),
            "ui_fingerprint": self.ui_fingerprint,
            "window_memory": self.window_memory,
            "window_screen": self.window_screen,
            "screen_id": self.screen_id,
            "screen_confidence": round(self.screen_confidence, 2),
            "region_hashes": self.region_hashes,
            "active_popups": self.active_popups,
        }
