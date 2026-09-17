"""Навигация по записанным координатам: карта → маршрут → управление.

coord_map   — 2.5D-сетка проходимости из записей (универсально для любой локации)
pathfinder  — случайная / дальняя цель, A* с учётом clearance, сглаживание
route       — полилиния маршрута (проекция, lookahead, уклон)
navigator   — контроллер «как игрок»: камера + WASD, онлайн-калибровка мыши
simulator   — офлайн-симуляция физики для тестов без CS2
render      — PNG-радар карты и маршрута
"""

from cs2_nav.coord_map import CoordinateMap, MapConfig, Node
from cs2_nav.navigator import NavConfig, Navigator
from cs2_nav.pathfinder import RouteConfig, pick_farthest_goal, pick_random_goal, plan_route
from cs2_nav.route import Route

__all__ = [
    "CoordinateMap",
    "MapConfig",
    "Node",
    "NavConfig",
    "Navigator",
    "RouteConfig",
    "Route",
    "pick_farthest_goal",
    "pick_random_goal",
    "plan_route",
]
