"""Автоматизация меню CS2: память → состояние → клики/сценарии."""

from cs2_menu.controller import MenuController
from cs2_menu.models import MenuPhase, UIAction, UIState
from cs2_menu.scenarios import SCENARIOS

__all__ = [
    "MenuController",
    "MenuPhase",
    "UIAction",
    "UIState",
    "SCENARIOS",
]
