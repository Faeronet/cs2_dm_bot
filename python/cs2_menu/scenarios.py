"""Сценарии навигации по меню CS2."""

from __future__ import annotations

from dataclasses import dataclass, field

from cs2_menu.models import MenuPhase, UIAction, UIState


@dataclass
class Scenario:
    id: str
    description: str
    steps: list[UIAction] = field(default_factory=list)
    success_phase: set[MenuPhase] = field(default_factory=set)
    success_in_map: bool = False
    success_matchmaking: bool = False


def _nav_deathmatch_steps() -> list[UIAction]:
    return [
        UIAction.click("nav_play"),
        UIAction.wait(450),
        UIAction.click("mode_deathmatch"),
        UIAction.wait(450),
        UIAction.click("go_button"),
        UIAction.wait(800),
    ]


def _nav_training_mirage_steps() -> list[UIAction]:
    """Play → Тренировка → Mirage → Соревновательный → Старт."""
    return [
        UIAction.click("nav_play"),
        UIAction.wait(500),
        UIAction.click("mode_practice"),
        UIAction.wait(600),
        UIAction.click("practice_map_mirage"),
        UIAction.wait(450),
        UIAction.click("practice_mode_competitive"),
        UIAction.wait(400),
        UIAction.click("practice_start"),
        UIAction.wait(800),
    ]


SCENARIOS: dict[str, Scenario] = {
    "main_menu": Scenario(
        id="main_menu",
        description="Вернуться на главный экран меню",
        steps=[
            UIAction.key("Escape", repeat=5),
            UIAction.wait(300),
            UIAction.click("back_button"),
            UIAction.wait(200),
        ],
        success_phase={MenuPhase.MAIN_MENU, MenuPhase.PLAY_HUB},
    ),
    "dismiss_popups": Scenario(
        id="dismiss_popups",
        description="Закрыть popups / уведомления",
        steps=[],  # выполняется через PopupSignature в controller
        success_phase=set(MenuPhase),
    ),
    "open_play": Scenario(
        id="open_play",
        description="Открыть раздел Play",
        steps=[
            UIAction.click("nav_play"),
            UIAction.wait(400),
        ],
        success_phase={MenuPhase.PLAY_HUB, MenuPhase.MAIN_MENU},
    ),
    "deathmatch": Scenario(
        id="deathmatch",
        description="Запустить Deathmatch (поиск матча)",
        steps=_nav_deathmatch_steps(),
        success_phase={MenuPhase.MATCHMAKING, MenuPhase.LOADING, MenuPhase.IN_GAME, MenuPhase.TEAM_SELECT},
        success_matchmaking=True,
    ),
    "training_mirage": Scenario(
        id="training_mirage",
        description="Тренировка → соревновательный de_mirage → Старт",
        steps=_nav_training_mirage_steps(),
        success_in_map=True,
    ),
    "deathmatch_in_game": Scenario(
        id="deathmatch_in_game",
        description="Deathmatch до загрузки карты",
        steps=_nav_deathmatch_steps(),
        success_in_map=True,
    ),
    "disconnect": Scenario(
        id="disconnect",
        description="Выйти из матча в главное меню",
        steps=[
            UIAction.key("Escape"),
            UIAction.wait(350),
            UIAction.click("pause_leave"),
            UIAction.wait(300),
            UIAction.key("Return"),
            UIAction.wait(300),
            # fallback: консоль
            UIAction.key("grave"),
            UIAction.wait(150),
            UIAction.type_text("disconnect"),
            UIAction.key("Return"),
            UIAction.wait(800),
        ],
        success_phase={MenuPhase.MAIN_MENU, MenuPhase.PLAY_HUB},
    ),
    "accept_match": Scenario(
        id="accept_match",
        description="Принять найденный матч",
        steps=[
            UIAction.click("accept_match"),
            UIAction.wait(300),
            UIAction.key("Return"),
        ],
        success_phase={MenuPhase.LOADING, MenuPhase.IN_GAME, MenuPhase.TEAM_SELECT},
    ),
    "pick_team_ct": Scenario(
        id="pick_team_ct",
        description="Выбрать CT на team select",
        steps=[
            UIAction.click("team_ct"),
            UIAction.wait(400),
        ],
        success_phase={MenuPhase.IN_GAME, MenuPhase.LOADING},
    ),
    "pick_team_t": Scenario(
        id="pick_team_t",
        description="Выбрать T на team select",
        steps=[
            UIAction.click("team_t"),
            UIAction.wait(400),
        ],
        success_phase={MenuPhase.IN_GAME, MenuPhase.LOADING},
    ),
    "load_mirage": Scenario(
        id="load_mirage",
        description="Консоль: map de_mirage (офлайн/DM тренировка)",
        steps=[
            UIAction.key("grave"),
            UIAction.wait(250),
            UIAction.type_text("map de_mirage"),
            UIAction.key("Return"),
            UIAction.wait(800),
        ],
        success_in_map=True,
    ),
    "cancel_search": Scenario(
        id="cancel_search",
        description="Отменить поиск матча",
        steps=[
            UIAction.click("cancel_search"),
            UIAction.wait(300),
            UIAction.key("Escape", repeat=2),
        ],
        success_phase={MenuPhase.PLAY_HUB, MenuPhase.MAIN_MENU, MenuPhase.MODE_DEATHMATCH},
    ),
}


def scenario_success(scenario: Scenario, state: UIState) -> bool:
    if scenario.success_in_map and state.in_map():
        return True
    if scenario.success_matchmaking and state.matchmaking_map not in ("", "<empty>"):
        return True
    if scenario.success_phase and state.phase in scenario.success_phase:
        return True
    if scenario.id == "deathmatch" and state.phase in (MenuPhase.LOADING, MenuPhase.IN_GAME):
        return True
    return False
