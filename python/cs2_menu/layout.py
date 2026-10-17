"""Нормализованные координаты меню CS2 (client area 1280×800).

Play hub — сетка 3×2:
  [Deathmatch] [Casual]   [Premier]
  [Wingman]    [Practice/Тренировка]

Экран «Тренировка»: слева/центр — карты, справа — режим, внизу — Старт.
"""

from __future__ import annotations

from cs2_menu.models import NormPoint

# --- левая колонка ---
NAV_X = 0.048
NAV_PLAY = NormPoint(NAV_X, 0.21)
NAV_STORE = NormPoint(NAV_X, 0.36)
NAV_INVENTORY = NormPoint(NAV_X, 0.51)
NAV_NEWS = NormPoint(NAV_X, 0.62)

# --- Play hub: сетка смещена вправо (центр экрана пустой) ---
MODE_DEATHMATCH = NormPoint(0.19, 0.35)
MODE_CASUAL = NormPoint(0.55, 0.38)
MODE_PREMIER = NormPoint(0.81, 0.35)
MODE_WINGMAN = NormPoint(0.19, 0.55)
MODE_PRACTICE = NormPoint(0.55, 0.58)  # «Тренировка»

# --- экран «Тренировка» ---
PRACTICE_MAP_MIRAGE = NormPoint(0.22, 0.34)
PRACTICE_MODE_COMPETITIVE = NormPoint(0.73, 0.32)
PRACTICE_START = NormPoint(0.80, 0.80)

GO_BUTTON = NormPoint(0.50, 0.845)
BACK_BUTTON = NormPoint(0.05, 0.06)

ACCEPT_MATCH = NormPoint(0.55, 0.72)
CANCEL_SEARCH = NormPoint(0.45, 0.845)

TEAM_CT = NormPoint(0.35, 0.55)
TEAM_T = NormPoint(0.65, 0.55)

POPUP_OK_CENTER = NormPoint(0.50, 0.64)
POPUP_CONTINUE = NormPoint(0.50, 0.68)
POPUP_CLOSE_X = NormPoint(0.92, 0.06)
NOTIFICATION_CLOSE = NormPoint(0.90, 0.10)
NOTIFICATION_BODY = NormPoint(0.50, 0.18)

PAUSE_LEAVE = NormPoint(0.50, 0.72)
PAUSE_RESUME = NormPoint(0.50, 0.55)

LAYOUT: dict[str, NormPoint] = {
    "nav_play": NAV_PLAY,
    "nav_store": NAV_STORE,
    "nav_inventory": NAV_INVENTORY,
    "nav_news": NAV_NEWS,
    "mode_deathmatch": MODE_DEATHMATCH,
    "mode_casual": MODE_CASUAL,
    "mode_premier": MODE_PREMIER,
    "mode_wingman": MODE_WINGMAN,
    "mode_practice": MODE_PRACTICE,
    "mode_training": MODE_PRACTICE,
    "practice_map_mirage": PRACTICE_MAP_MIRAGE,
    "practice_mode_competitive": PRACTICE_MODE_COMPETITIVE,
    "practice_start": PRACTICE_START,
    "go_button": GO_BUTTON,
    "back_button": BACK_BUTTON,
    "accept_match": ACCEPT_MATCH,
    "cancel_search": CANCEL_SEARCH,
    "team_ct": TEAM_CT,
    "team_t": TEAM_T,
    "popup_ok_center": POPUP_OK_CENTER,
    "popup_continue": POPUP_CONTINUE,
    "popup_close_x": POPUP_CLOSE_X,
    "notification_close": NOTIFICATION_CLOSE,
    "notification_body": NOTIFICATION_BODY,
    "pause_leave": PAUSE_LEAVE,
    "pause_resume": PAUSE_RESUME,
}

# Цепочка «тренировка → соревновательный mirage → старт»
TRAINING_MIRAGE_CLICKS: list[str] = [
    "nav_play",
    "mode_practice",
    "practice_map_mirage",
    "practice_mode_competitive",
    "practice_start",
]
