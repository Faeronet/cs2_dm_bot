"""
Контроллер движения по маршруту — «как обычный игрок в DM».

Идея (pure pursuit):
* Камера смотрит на точку маршрута впереди на 120–260u (зависит от скорости) —
  не на конечную цель и не на ближайшую вершину, а «туда, куда сейчас бегу».
  Поворот плавный: ограничение скорости и ускорения поворота + лёгкий шум мыши.
* Ноги идут к ближней «морковке» (≈60u впереди по маршруту) — WASD выбирается по
  углу между направлением к морковке и текущим yaw камеры (8 направлений, гистерезис).
  На прямых — только W; на поворотах камера ведёт, тело слегка подстраивается A/D.
* Все данные (позиция, скорость, углы) читаются из памяти каждый тик:
  - позиция проецируется на маршрут (монотонный прогресс, лестница может пересекать себя);
  - по Δyaw/px калибруется чувствительность мыши (и знак) на лету;
  - по направлению скорости при зажатых клавишах проверяется соглашение yaw
    (если память отдаёт yaw со смещением — бот сам это обнаружит и учтёт).
* Застревание: по скорости. Прыжок → страйф → шаг назад → перестроить маршрут.
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from cs2_control.input import GameInput
from cs2_nav.coord_map import CoordinateMap, _ladder_as_dict
from cs2_nav.los_visibility import apply_los_visibility, los_clear, visible_bone_parts
from cs2_nav.pathfinder import (
    BoxObstacle,
    Obstacle,
    RouteConfig,
    blocked_by_box_obstacles,
    blocked_by_obstacles,
    pick_farthest_goal,
    pick_random_goal,
    plan_route,
    segment_clear,
)
from cs2_nav.route import Route
from cs2_reader.aim import (
    chest_plausible,
    forward_from_angles,
    head_plausible,
    shoulder_plausible,
    shortest_yaw_err,
    yaw_to_target,
)
from cs2_reader.game_state import Vec3
from cs2_reader.weapons import (
    default_clip,
    is_gun,
    is_healthshot,
    weapon_cycle_s,
    weapon_fire_mode,
    weapon_is_auto,
    weapon_is_pistol,
    weapon_needs_scope,
)
from cs2_reader.recoil import recoil_offset, tap_gap_range

XYZ = tuple[float, float, float]

# Относительный угол движения для комбинации клавиш (Source: +yaw = влево/против часовой)
_KEY_DIRS: list[tuple[float, frozenset[str]]] = [
    (0.0, frozenset({"w"})),
    (45.0, frozenset({"w", "a"})),
    (90.0, frozenset({"a"})),
    (135.0, frozenset({"s", "a"})),
    (180.0, frozenset({"s"})),
    (-135.0, frozenset({"s", "d"})),
    (-90.0, frozenset({"d"})),
    (-45.0, frozenset({"w", "d"})),
]
_DIR_OF_KEYS: dict[frozenset[str], float] = {k: a for a, k in _KEY_DIRS}

class _PendingAxis:
    """Градусы, отправленные мышью, но ещё не отразившиеся в памяти игры (задержка ввода 1–3 тика).

    Без учёта этого замкнутая петля дожима «не видит» своих последних движений, досылает ещё и
    перелетает точку — прицел ёрзает влево-вправо около мишени. Сопоставляем пришедшее Δугла
    с самыми старыми отправками того же знака (FIFO); остаток = «в полёте». Старые записи
    истекают, поэтому ошибка калибровки °/px не копится."""

    __slots__ = ("q", "expire")

    def __init__(self, expire_ticks: int = 5) -> None:
        self.q: list[list[float]] = []      # [tick, remaining_deg]
        self.expire = expire_ticks

    def sent(self, tick: int, deg: float) -> None:
        if abs(deg) > 1e-9:
            self.q.append([float(tick), deg])

    def observed(self, tick: int, deg: float) -> None:
        rem = deg
        for item in self.q:
            if abs(rem) < 1e-9:
                break
            if item[1] * rem > 0:
                take = item[1] if abs(item[1]) <= abs(rem) else rem
                item[1] -= take
                rem -= take
        self.q = [it for it in self.q if abs(it[1]) > 1e-9 and tick - it[0] < self.expire]

    def value(self) -> float:
        return sum(it[1] for it in self.q)

    def clear(self) -> None:
        self.q.clear()


# Какие кости бот умеет брать на мушку. Порядок = приоритет fallback'а,
# если выбранная часть вдруг не видна или отсутствует в кадре.
_AIM_PART_KEYS: dict[str, str] = {
    "head": "head_position",
    "chest": "chest_position",
    "left_shoulder": "left_shoulder_position",
    "right_shoulder": "right_shoulder_position",
}
_AIM_PARTS_ORDER: tuple[str, ...] = ("head", "chest", "left_shoulder", "right_shoulder")
_AIM_PART_RU: dict[str, str] = {
    "head": "голова",
    "chest": "грудь",
    "left_shoulder": "лев. плечо",
    "right_shoulder": "прав. плечо",
}

# xxHash32, урезанный под (зерно, счётчик) — как в разборе генераторов на Хабре:
# https://habr.com/ru/companies/vk/articles/574414/
_XXH_P2 = 2246822519
_XXH_P3 = 3266489917
_XXH_P4 = 668265263
_XXH_P5 = 374761393


def _rotl32(x: int, r: int) -> int:
    x &= 0xFFFFFFFF
    return ((x << r) | (x >> (32 - r))) & 0xFFFFFFFF


def _xxhash32_int(buf: int, seed: int) -> int:
    """Одно 32-битное значение: состояние `buf` и зерно `seed` (без массива байт)."""
    h32 = (int(seed) + _XXH_P5) & 0xFFFFFFFF
    h32 = (h32 + 4) & 0xFFFFFFFF
    h32 = (h32 + (int(buf) & 0xFFFFFFFF) * _XXH_P3) & 0xFFFFFFFF
    h32 = (_rotl32(h32, 17) * _XXH_P4) & 0xFFFFFFFF
    h32 ^= h32 >> 15
    h32 = (h32 * _XXH_P2) & 0xFFFFFFFF
    h32 ^= h32 >> 13
    h32 = (h32 * _XXH_P3) & 0xFFFFFFFF
    h32 ^= h32 >> 16
    return h32 & 0xFFFFFFFF


@dataclass
class NavConfig:
    tick_hz: float = 40.0
    # --- камера
    look_ahead_min: float = 120.0
    look_ahead_max: float = 260.0
    look_ahead_speed_gain: float = 0.55      # + скорость × gain
    yaw_gain: float = 7.9                    # ×1.75: скорость поворота = gain × ошибка
    yaw_rate_max: float = 455.0              # ×1.75 °/с стоя (с cam_pace ±15% это 1.5–2× прежних 260)
    yaw_rate_max_moving: float = 333.0       # ×1.75 °/с на бегу
    yaw_accel_max: float = 2450.0            # ×1.75 °/с²: до нового потолка за то же время
    yaw_jerk_max: float = 24500.0            # ×1.75 °/с³: S-кривая, без рывка в потолок
    yaw_deadband: float = 0.6                # мягкая зона: скорость ∝ err·|err|/(|err|+deadband)
    yaw_wander: float = 2.5                  # ° медленное блуждание прицела
    cam_warm_s: float = 0.70                 # с: после старта взгляд разгоняется как на маршруте, без рывка
    cam_pace_sigma: float = 0.028            # AR(1): насколько «темп» поворота камеры на маршруте гуляет
    cam_pace_range: float = 0.15             # ±15 % к потолку и усилению поворота (не одна скорость)
    # --- «рука»: плавность и кривизна (ветер как в WindMouse: инерционный случайный вектор скорости)
    hand_wind_sigma: float = 3.0             # °/с: σ ветра при панорамировании (стационарное)
    hand_wind_decay: float = 0.88            # за тик: инерция ветра (WindMouse: W/√3 за шаг)
    tremor_sigma: float = 0.0                # °/тик: σ тремора при удержании (0 = прицел стоит; ≥0.01 — видно ±1 px)
    flick_curve_frac: float = 0.012          # слабее дуга — меньше увод с центра в конце маха
    flick_wind_frac: float = 0.006
    pitch_gain: float = 4.4                  # ×1.75
    pitch_rate_max: float = 88.0             # ×1.75
    pitch_base: float = 3.0                  # чуть ниже горизонта, как у игрока
    pitch_slope_gain: float = 0.5            # смотреть вниз на спуске / вверх на подъёме
    # --- движение
    carrot_dist: float = 60.0
    max_speed: float = 250.0                 # u/s для расчёта steering
    steer_damping: float = 0.8               # насколько гасим текущую скорость (0 = чистый pursuit)
    steer_smooth: float = 0.55               # EMA вектора руления (0 = без сглаживания)
    key_hysteresis_deg: float = 9.0
    key_min_hold: float = 0.12               # с: не дёргать клавиши чаще
    forward_only_deg: float = 32.0           # |rel| в конусе — только W, без лёгкого A/D
    strafe_only_deg: float = 70.0            # цель сильно сбоку → только A/D + поворот
    turn_in_place_deg: float = 105.0         # цель сзади → стоим, крутим камеру
    arrive_radius: float = 24.0
    brake_time: float = 0.19                 # с: тормозной путь ≈ v/sv_friction(5.2) — отпустить W заранее
    # --- надёжность
    off_route_replan: float = 80.0
    stuck_speed: float = 30.0
    stuck_time: float = 0.45
    stuck_replan_after: int = 4              # эскалаций до перестроения маршрута
    # --- калибровка (обновляется на лету)
    yaw_per_px: float = -0.0759              # °/px по yaw (Source: dx>0 → yaw уменьшается)
    pitch_per_px: float = 0.0759             # °/px по pitch (dy>0 → смотрим вниз)
    calibrate_online: bool = True
    pitch_follows_yaw: bool = True           # |pitch_per_px| = |yaw_per_px| (одна сенса на обе оси), знак свой
    detect_yaw_offset: bool = True
    max_px_per_tick: int = 160
    precal_ticks: int = 0                    # 0 = без стартового увода мыши; калибровка на поворотах маршрута
    precal_px: int = 16                      # px за тик, если precal_ticks > 0
    # --- мишени вокруг (LOS по карте + spotted, не конус камеры)
    engage_targets: bool = True              # смотреть на видимую вокруг мишень, продолжая идти по маршруту
    enemies_poll_hz: float = 20.0            # как часто читать список врагов из памяти (перебор 64 сущностей)
    ladders_poll_hz: float = 0.5             # лестницы меняются редко — полный перебор entity list
    doors_poll_hz: float = 2.0               # двери: состояние open/closed меняется чаще
    door_block_margin: float = 20.0          # запас вокруг запертой двери при обходе
    door_use_dist: float = 96.0              # u: +use достаёт, жмём на ходу
    door_use_approach: float = 140.0         # u: дверь впереди по маршруту
    door_use_hold: tuple[float, float] = (0.07, 0.12)
    door_use_retry: float = 0.55             # с: повтор, если всё ещё закрыта
    breakables_poll_hz: float = 1.0          # решётки/окна: hp меняется реже дверей
    breakable_block_margin: float = 16.0     # запас вокруг целой решётки, если её нельзя прострелить
    breakable_engage_dist: float = 340.0     # u: решётка на маршруте впереди — начинаем наводку
    breakable_aim_deg: tuple[float, float] = (1.6, 3.4)  # °: точка рядом с прицелом на старте, xxHash
    target_memory: float = 45.0              # с: помним, где стояла мишень, когда её уже не видно (для обхода)
    standoff_dist: float = 110.0             # u: если мишень перекрыла проход — подойти на столько и стрелять с места
    standoff_wait: float = 4.0               # с: стоим на дистанции без мишени в прицеле → бросаем, другая цель
    hunt_hidden: bool = True                 # знаем невидимых: заходим в точку с LOS, не целимся сквозь стену
    hunt_max_dist: float = 900.0             # дальше скрытую не преследуем
    hunt_peek_min: float = 72.0              # u: ближе к мишени пик не ставим
    hunt_peek_max: float = 210.0             # u: дальше — слабая позиция
    hunt_max_detour: float = 280.0           # u: лишний путь ради пика; больше — идём своим маршрутом
    hunt_preaim_dist: float = 150.0          # u: слепой выход — когда начинать смотреть в проём
    hunt_check_s: float = 0.7                # с: как часто пересчитывать пик
    peek_blind_chance: float = 0.40          # xxHash: выйти как обычно, но среагировать быстрее
    peek_preaim_dist: float = 260.0          # u: пик — держать прицел за угол с этого расстояния
    peek_commit_dist: float = 125.0          # u: ближе — выход боком / по дуге
    peek_overshoot: tuple[float, float] = (34.0, 64.0)  # u: прицел чуть дальше угла, в проём
    target_forget_polls: int = 5             # опросов подряд «место видно, мишени нет» → забываем (пропала/убита)
    target_max_dist: float = 1400.0          # дальше — игнорируем
    target_reaction: tuple[float, float] = (0.0, 0.0)     # с: наводимся в тот же тик, как увидели
    target_fast_reaction: tuple[float, float] = (0.0, 0.0)
    target_hold: float = 0.85                # с: spotted в CS2 мигает — не бросаем наводку на каждый кадр
    target_yaw_rate_max: float = 330.0       # °/с: (режим без стрельбы) наводка резче панорамы
    target_yaw_gain: float = 9.0             # 1/с: (режим без стрельбы)
    target_aim_jitter: float = 0.0           # °: на голове макета дрейф сбивает центр — держим прицел
    target_switch_margin: float = 80.0       # угроза: новая должна быть заметно ближе/опаснее (не «кто в прицеле»)
    target_engage_max: tuple[float, float] = (6.0, 8.0)   # с: предохранитель — мишень не пропадает / не навелись → идём дальше
    target_cooldown: float = 6.0             # с: после этого мишень снова «интересна» (пропала / бросили)
    target_done_cooldown: float = 40.0       # с: мишень, которую расстреляли, а она не пропала — не трогаем долго
    target_behind_deg: float = 125.0         # мишень позади по ходу движения → не бежим спиной, отпускаем
    # --- наводка «как рука»: баллистический флик (minimum-jerk) + короткая коррекция по зрению
    aim_flick_deg: float = 30.0              # ≥ столько ° по любой оси — флик как сейчас; меньше — как навигация
    aim_handoff_s: float = 0.03              # ×1.5 к 0.02: разворот чуть спокойнее
    aim_close_dist: float = 140.0            # u: ближе — мышь чуть быстрее панорамы маршрута
    aim_far_dist: float = 750.0
    aim_close_boost: float = 5.68            # ÷1.5 к 8.52
    aim_far_boost: float = 4.32              # ÷1.5 к 6.48
    aim_speed_jitter: float = 0.12           # ± доля: конечная скорость фазы (xxHash, не одно число)
    aim_accel_span: tuple[float, float] = (0.84, 1.18)  # xxHash: ускорение к этой скорости, не одна цифра
    flick_ramp: float = 0.12                 # с: набор к профилю флика (короче — быстрее подхват)
    flick_min_deg: float = 4.0               # меньше — без флика, сразу коррекция
    flick_base_time: float = 0.012           # ×1.5 к 0.008
    flick_per_deg: float = 0.0003            # ×1.5: 90° ≈ 39 мс, 180° ≈ 66 мс
    flick_rate_max: float = 4800.0           # ÷1.5 к 7200
    flick_undershoot: float = 0.0            # без недовода: овал головы маленький
    flick_error_frac: float = 0.004
    flick_error_min: float = 0.05            # °
    aim_correct_delay: tuple[float, float] = (0.0, 0.0)  # без паузы между фликом и выстрелом
    aim_lock_gain: float = 88.0              # ÷1.5 к 132
    aim_lock_rate: float = 1360.0            # ÷1.5 к 2040
    aim_correct_gain: float = 88.0           # ÷1.5
    aim_correct_rate: float = 1360.0         # ÷1.5
    aim_max_px_per_tick: int = 1600          # ÷1.5 к 2400: потолок мыши на развороте
    aim_chase_speed: tuple[float, float] = (3.0, 4.5)  # xxHash: ведение ходящей мишени в 2–3 раза быстрее
    aim_chase_move: float = 70.0             # u/s: ходьба тоже считается движением, не только бег
    aim_chase_strafe: float = 40.0           # u/s поперёк луча
    aim_chase_rise: tuple[float, float] = (0.045, 0.10)  # с: набор к скорости ведения, xxHash, от текущей
    aim_chase_fall: float = 0.22             # с: возврат, когда остановилась
    aim_track_pace: tuple[float, float] = (0.90, 1.12)  # xxHash: то чуть быстрее, то чуть медленнее
    aim_overshoot_u: tuple[float, float] = (0.4, 1.2)   # u: микро-упреждение по стрейфу (было (2.2,5.5))
    aim_overshoot_s: tuple[float, float] = (0.08, 0.16)  # с: вдвое короче окно перевода
    aim_miss_chance: float = 0.015           # xxHash: редкий промах (было 10%, теперь 1.5%)
    aim_miss_u: tuple[float, float] = (0.8, 2.2)   # u: если и мимо — то едва мимо овала
    aim_chest_chance: float = 0.20           # xxHash: грудь вместо головы (если грудь видна)
    aim_shoulder_chance: float = 0.08        # xxHash: плечо вместо головы (если видно только плечо — всегда)
    aim_head_off_xy: tuple[float, float] = (0.5, 2.2)   # u: точка на голове, не всегда центр
    aim_head_off_z: tuple[float, float] = (-1.5, 1.6)
    aim_chest_off_xy: tuple[float, float] = (0.8, 4.0)
    aim_chest_off_z: tuple[float, float] = (-3.2, 3.0)
    aim_shoulder_off_xy: tuple[float, float] = (0.4, 1.8)  # u: плечо мелкое — разброс поменьше
    aim_shoulder_off_z: tuple[float, float] = (-1.2, 1.2)
    aim_bone_off_along: tuple[float, float] = (-0.7, 0.7)  # u: чуть ближе/дальше по лучу
    unscope_click_chance: float = 0.20       # xxHash: выход из скоупа 2×ПКМ, иначе нож→оружие
    unscope_click_gap: tuple[float, float] = (0.14, 0.34)  # с между кликами ПКМ
    # --- стрельба
    fire_enabled: bool = True
    shots_per_target: int = 0                # выстрелов по мишени; 0 = стреляем, пока мишень не пропадёт
    fire_tolerance_deg: float = 0.10
    fire_tolerance_pitch_deg: float = 0.10
    fire_target_half_width: float = 4.5      # u: полуось овала по XY
    fire_target_up_margin: float = 5.5       # u: полуось овала по Z
    fire_oval_frac: float = 0.45             # шире окно попадания (было 0.28 — слишком узкое, выстрел задерживался)
    fire_confirm_s: float = 0.0              # точка в овале — стреляем в этот же тик
    fire_interval: tuple[float, float] = (0.14, 0.22)   # запасной интервал тапа, если нет таблицы
    fire_first_delay: tuple[float, float] = (0.0, 0.0)
    fire_hold_ticks: int = 1                 # тиков держим ЛКМ на тапе (≥1 тик игры)
    fire_stop_moving: bool = True            # стоим на месте во время стрельбы (точность как у игрока)
    vuln_fire_delay: tuple[float, float] = (0.86, 1.18)  # с: после снятия spawn-защиты, xxHash около 1 с
    combat_strafe_chance: float = 0.25       # xxHash: A/D шаги МЕЖДУ выстрелами (первый — всегда с места)
    combat_strafe_dist: tuple[float, float] = (18.0, 44.0)  # u: короткий шаг, каждый раз новый
    # --- скоуп снайперок без хипфайр-прицела (AWP/SSG/автоснайпер)
    scope_noscope_chance: float = 0.30       # xxHash: автоснайперы
    scope_noscope_awp: float = 0.02          # xxHash: AWP, отдельно от SSG
    scope_noscope_ssg: float = 0.15          # xxHash: половина прежних 30%
    scope_noscope_close: float = 450.0       # u: ближе — AWP и SSG без скоупа
    scope_enter_deg: tuple[float, float] = (1.8, 4.6)   # °: радиус прицела, чтобы войти в скоуп
    scope_enter_u: tuple[float, float] = (14.0, 34.0)   # u: мировой промах около мишени (не тот же, что °)
    scope_settle: tuple[float, float] = (0.07, 0.18)    # с: после входа в скоуп ещё доводим
    # --- антиотдача: ведём линию мышью, не телепортируем прицел на точку пули
    rcs_enabled: bool = True
    rcs_rate: float = 28.0                   # °/с база ведения линии (потом плавает)
    rcs_rate_jitter: float = 0.030           # ± доля скорости xxHash (меньше дрейфа — точнее линия)
    rcs_gain: float = 10.5                   # 1/с: догон линии, не lock 340°/с
    rcs_wind_sigma: float = 0.20             # °/с ветер WindMouse на линии (меньше бокового разноса)
    rcs_return_mul: tuple[float, float] = (0.92, 1.32)  # xxHash: скорость возврата от скорости RCS
    rcs_return_done: float = 0.65            # °: считаем, что вернулись
    rcs_break_deg: float = 5.5               # °: сошли с линии слишком далеко — отпускаем зажим
    rcs_path_frac: float = 0.55              # допуск «на линии» vs овал первого выстрела
    # --- перезарядка / осмотр / нож / дальние тапы
    habits_enabled: bool = True
    habit_clear_dist: float = 380.0          # u: «рядом нет мишеней»
    long_tap_dist: float = 1100.0            # u: дальше автомат тапает (кроме 20% зажима)
    long_spray_chance: float = 0.20          # шанс зажима на дистанции
    combat_reload_chance: float = 0.045      # редко перезарядиться, даже если мишень рядом
    healthshot_hp: float = 70.0              # колоть шприц при HP ≤ этого
    healthshot_early_chance: float = 0.20    # xxHash: порог чуть выше 70%
    healthshot_early_hp: tuple[float, float] = (73.0, 82.0)
    healthshot_inject: tuple[float, float] = (1.02, 1.28)  # с: анимация укола
    eye_height: float = 64.0                 # z глаз над ногами (стоя)
    target_body_radius: float = 16.0         # мишень — цилиндр, как игрок
    target_avoid_dist: float = 72.0          # начинаем обходить мишень на теле маршрута с этого расстояния
    # --- цель
    loop: bool = True                        # достигли — сразу к следующей случайной точке
    random_goals: bool = True                # случайная цель, не самая дальняя
    prefetch_dist: float = 520.0             # u: за сколько до цели готовить следующий маршрут
    pause_at_goal: tuple[float, float] = (0.0, 0.0)
    route: RouteConfig = field(default_factory=RouteConfig)
    seed: int | None = None
    verbose_log: bool = True
    duration: float | None = None


class _PoseSource:
    """Минимальный интерфейс чтения (Reader или FakeReader симуляции)."""

    def local_position(self): ...
    def local_velocity(self): ...
    def local_eye_angles(self): ...
    def visible_enemies(self): ...
    def enemies(self): ...
    def local_weapon(self): ...
    def local_shots_fired(self): ...
    def local_is_scoped(self): ...
    def poll_life(self): ...
    def ladders(self): ...
    def local_on_ladder(self): ...
    def doors(self): ...


class Navigator:
    def __init__(
        self,
        reader: _PoseSource,
        inp: GameInput,
        coord_map: CoordinateMap,
        config: NavConfig | None = None,
        clock: Callable[[], float] | None = None,
    ):
        self.reader = reader
        self.input = inp
        self.map = coord_map
        self.cfg = config or NavConfig()
        # дельта мыши каждого тика размазывается бэкендом ввода на период тика (без «ступенек»)
        spread = getattr(inp, "set_mouse_spread", None)
        if callable(spread):
            spread(1.0 / self.cfg.tick_hz)
        self.rng = random.Random(self.cfg.seed)
        self._clock = clock or time.monotonic
        self._aim_xx_seed = (int(self.cfg.seed) & 0xFFFFFFFF) if self.cfg.seed is not None else 0xA24BAED5
        self._aim_xx_n = 0
        self._aim_speed_var = 0.0             # текущий сдвиг скорости наводки в ±aim_speed_jitter
        self._aim_speed_mul = 1.0
        self._aim_accel_mul = 1.0             # xxHash на вход в фазу: насколько быстро добираем скорость
        self._chase_rise = 0.08

        self.route: Route | None = None
        self.goal_id: int | None = None
        self.s = 0.0
        self.done = False
        self.status = "init"

        self._keys: set[str] = set()
        self._keys_since = 0.0
        self._last_t: float | None = None
        self._yaw_rate = 0.0
        self._pitch_rate = 0.0
        self._yaw_acc = 0.0
        self._pitch_acc = 0.0
        self._wander = 0.0
        self._wander_vel = 0.0
        self._cam_pace = 0.0                 # текущий множитель темпа камеры на маршруте (AR(1))
        self._sent_px = (0, 0)
        # «рука»: ветер (инерционная случайная скорость), тремор, дрейф точки прицеливания,
        # дробный остаток пикселей (без него мелкие повороты дребезжат ±1 px)
        self._wind = [0.0, 0.0]
        self._tremor = [0.0, 0.0]
        self._aim_drift = [0.0, 0.0]
        self._px_carry = [0.0, 0.0]
        self._chase_boost = 1.0               # 1 = обычная наводка, >1 = догон бегущей
        self._chase_mul = 1.12                # цель множителя на этот эпизод (xxHash)
        self._chase_on = False
        self._tgt_xy: dict[int, tuple[float, float, float]] = {}
        self._lat_sign = 0
        self._tgt_lat = 0.0
        self._tgt_aim_dist = 400.0
        self._track_aim = [0.0, 0.0]
        self._track_want = (0.0, 0.0)
        self._track_phase = "center"
        self._track_until = 0.0
        self._track_pace = 1.0
        self._aim_part = "head"               # head/chest/left_shoulder/right_shoulder
        self._aim_chest = False               # back-compat alias (_aim_part == "chest")
        self._aim_bone_delta = (0.0, 0.0, 0.0)  # кость − голова на момент выбора (если кость пропала из чтения)
        self._aim_spot = (0.0, 0.0, 0.0)      # lat, along, up — точка на кости (xxHash, на мишень)
        self._shot_miss: bool | None = None
        self._prev_mem = None  # (yaw, pitch)
        self._pend_yaw = _PendingAxis()       # мышь «в полёте»: отправлено, в памяти ещё нет
        self._pend_pitch = _PendingAxis()
        self._cal_win: list[tuple[int, int, float, float]] = []
        self._mouse_dead_ticks = 0
        self._sign_flips_yaw = 0
        self._sign_flips_pitch = 0
        self.yaw_offset = 0.0
        self._offset_samples: list[float] = []
        self._same_keys_ticks = 0
        self._stuck_since: float | None = None
        self._last_jump_at = -10.0
        self._stuck_level = 0
        self._stuck_plan: list[str] = []      # манёвры текущего эпизода застревания
        self._face_turn_since: float | None = None
        self._progress_s = 0.0
        self._progress_at = 0.0
        self._noprog_replans = 0
        self._ladder_cooldown_until = 0.0
        self._override_until = 0.0
        self._override_keys: set[str] = set()
        self._pause_until = 0.0
        self._precal_left = self.cfg.precal_ticks
        self.replans = 0
        self.stucks = 0
        self.events: list[str] = []
        self._brake = False
        self.visible_enemies: list[dict] = []
        self.all_enemies: list[dict] = []        # видимые + скрытые (координаты из памяти)
        self.ladders: list[dict] = [
            dict(lk["ladder"])
            for lk in getattr(coord_map, "ladder_links", [])
            if lk.get("ladder")
        ]
        self.doors: list[dict] = []              # prop_door / func_door на карте
        self.breakables: list[dict] = []       # func_breakable, решётки, окна
        self._break_aim: dict | None = None    # точка на решётке, выбранная в начале наводки
        self._door_use_at: dict[int, float] = {}
        self._door_ahead: dict | None = None
        self._ladders_linked = False
        self._ladder: dict | None = None         # подъём/спуск по лестнице
        self._crouch_until = 0.0
        self.known_targets: list[dict] = []      # живые враги, которые сейчас в памяти
        self._known: dict[int, dict] = {}
        self._hunt: dict | None = None           # скрытая мишень: куда выйти, чтобы увидеть
        self._hunt_routed: int | None = None     # peek_id, к которому уже построен заход
        self._hunt_via_since = 0.0
        self._hunt_check_at = 0.0
        self._hunt_look_hold: tuple[float, float] | None = None
        self._hunt_look_until = 0.0
        self._aim_fast_peek = False              # короткий handoff после слепого/уже наведённого пика
        self._peek_handoff = 0.18
        self._look_yaw_f: float | None = None     # сглаженный yaw точки взгляда (не камеры)
        self._look_mode = "route"
        self._cam_warm_t0: float | None = None    # первый тик: не рвать камеру на look-ahead
        self._recent_aim_index: int | None = None
        self._recent_aim_drop = 0.0
        self._cal_frozen = False
        # состояние наводки на мишень
        self._target: dict | None = None         # текущая мишень вокруг (LOS / spotted)
        self._target_seen_at = 0.0               # когда впервые увидели текущую мишень
        self._target_last_seen = 0.0             # когда видели последний раз
        self._target_reaction = 0.0              # индивидуальная задержка реакции на эту мишень
        self.engaged = False                     # камера сейчас на мишени
        self.engaged_ticks = 0
        self.targets_engaged = 0
        self._route_blockers: frozenset = frozenset()   # какие мишени учтены в текущем маршруте
        self._standoff: int | None = None        # мишень перекрыла проход: стоим на дистанции выстрела, ждём
        self._standoff_since: float | None = None
        self._intended_goal_id: int | None = None  # куда шли до обхода/ожидания — после убийства возвращаемся сюда
        self._given_up: dict[int, float] = {}    # мишени, по которым отстрелялись, а они не ушли (до момента)
        self._block_check_at = 0.0
        self._target_bumps = 0                   # сколько раз подряд упирались в мишень
        self._target_contact_since: float | None = None
        self._enemies_next_poll = 0.0
        self._ladders_next_poll = 0.0
        self._doors_next_poll = 0.0
        self._doors_blocked = 0
        self._breakables_next_poll = 0.0
        self._breakables_blocked = 0
        self._breakables_logged = False
        # соглашение yaw подтверждено движением (до этого не стреляем — прицел может быть смещён)
        self.yaw_verified = not self.cfg.detect_yaw_offset
        # стрельба
        self._shots_on_target = 0                # выстрелов по текущей мишени
        self._next_fire_at = 0.0                 # раньше этого не стреляем
        self._aimed_since: float | None = None   # с какого момента прицел на мишени
        self._fire_release_at_tick = -1          # на каком тике отпустить ЛКМ (тап)
        self._fire_held = False                  # ЛКМ сейчас зажата
        self._scope_plan: dict | None = None     # noscope / deg / radius_u / settle на эту мишень
        self._scope_committed = False            # уже начали вход в скоуп
        self._scope_held = False                 # ПКМ сейчас зажата
        self._scope_click_until_tick = -1
        self._scope_next_click = 0.0
        self._scoped_now = False
        self._unscope_left = 0                   # после боя: 20% два клика ПКМ, иначе нож→оружие
        self._unscope_next = 0.0
        self._combat_strafe: dict | None = None  # 60%: шаги A/D, огонь на контрстрейфе
        self._strafe_fire_ok = True
        self._immune_seen: set[int] = set()      # видели spawn-защиту
        self._vuln_ready_at: dict[int, float] = {}  # index → когда можно стрелять после уязвимости
        self._tick_no = 0
        self._target_visible_now = False         # мишень в списке видимых на этом тике (стреляем только тогда)
        self._eye_z = self.cfg.eye_height        # высота глаз над ногами (обновляется из памяти)
        self._mem_shots = 0                      # m_iShotsFired
        self._rcs_shots_seen = 0
        self._rcs_shot_t = 0.0
        self._rcs_scale_x = 1.0
        self._rcs_scale_y = 1.0
        self._rcs_rot = 0.0
        self._rcs_lag = 0.10
        self._rcs_under = 0.97
        self._rcs_arc = 0.0
        self._rcs_rate_var = 0.0
        self._rcs_sampled = False
        self._rcs_return = False
        self._rcs_return_rate = 28.0
        self._rcs_return_until = 0.0
        self._rcs_return_seeded = True
        # флик: None | dict(t0, T, dyaw, dpitch, s_prev); _flick_pending — новый захват, флик ещё не спланирован
        self._flick: dict | None = None
        self._flick_pending = False
        self._aim_handoff_t0: float | None = None
        self._aim_used_flick = False
        self._subflicks = 0                      # корректирующих подвижений после основного маха
        self._correct_from = 0.0                 # после флика: с какого момента идёт зрительная коррекция
        self.firing = False                      # сейчас в режиме стрельбы (стоим, стреляем)
        self.shots_fired = 0
        self.targets_shot = 0
        self._blocked: set[int] = set()
        self._steer_vec = (0.0, 0.0)
        self._target_engage_until = 0.0          # до какого момента держим взгляд на текущей мишени
        self._target_cooldown: dict[int, float] = {}   # index → когда снова можно смотреть
        self._target_behind_since: float | None = None
        self.weapon: dict = {"id": 0, "name": "", "clip": 0, "in_reload": False, "auto": False, "fire": "none"}
        self.life: dict = {
            "alive": True,
            "status": "alive",
            "health": 100,
            "events": {"died": False, "respawned": False},
        }
        self._long_spray: bool | None = None     # дальняя: None = ещё не кинули 20%
        self._reload_frac = 0.20                 # доля обоймы, при которой жмём R
        self._reload_wid = -1
        self._reload_skip = False                # рядом была мишень — в этот магазин боевой R не катили
        self._reload_clip_seen = 0
        self._habit: dict | None = None
        self._habit_next = 0.0
        self._hpshot_thr: float | None = None
        self._extra_key_until: dict[str, float] = {}
        self._dead = False
        self._route_gen = 0
        self._next_route: Route | None = None
        self._next_goal_id: int | None = None
        self._prefetch_busy = False
        self._prefetch_lock = threading.Lock()

    # ------------------------------------------------------------ helpers

    def _now(self) -> float:
        return self._clock()

    def _enemy_feet(self, e: dict) -> tuple[float, float, float]:
        return e.get("feet", e["position"])

    def _apply_los_visibility(self, e: dict, x: float, y: float, z: float) -> None:
        """Видимость по LOS (не за стеной). Off-screen OK. Заполняет visible и *_visible."""
        apply_los_visibility(
            e, self.map, x, y, z,
            eye_z=self._eye_z,
            hull_radius=self.map.cfg.hull_radius,
            target_body_radius=self.cfg.target_body_radius,
        )

    def _around_visible(self, e: dict, x: float, y: float, z: float) -> bool:
        """Прямая видимость вокруг бота (LOS по карте), не конус камеры."""
        if e.get("dormant"):
            return False
        if "visible" not in e or e.get("_vis_pos") != (round(x), round(y), round(z)):
            self._apply_los_visibility(e, x, y, z)
            e["_vis_pos"] = (round(x), round(y), round(z))
        return bool(e.get("visible"))

    def _visible_parts(self, e: dict, x: float, y: float, z: float) -> dict[str, tuple[float, float, float] | None]:
        """Какие кости видны из глаз бота (LOS по 2.5D-карте).
        Ключи — 'head' / 'chest' / 'left_shoulder' / 'right_shoulder'."""
        parts_vis = visible_bone_parts(
            e, self.map, x, y, z,
            eye_z=self._eye_z,
            hull_radius=self.map.cfg.hull_radius,
            target_body_radius=self.cfg.target_body_radius,
        )
        out: dict[str, tuple[float, float, float] | None] = {}
        for part, key in _AIM_PART_KEYS.items():
            pos = e.get(key)
            out[part] = pos if parts_vis.get(part) and pos is not None else None
        return out

    def _reset_aim_bone(self) -> None:
        self._aim_part = "head"
        self._aim_chest = False
        self._aim_spot = (0.0, 0.0, 0.0)
        self._aim_bone_delta = (0.0, 0.0, 0.0)

    def _enemy_from_reader(self, p) -> dict:
        """Мишень: position = центр овала (голова / макет). Ноги — если кость правдоподобна."""
        feet = p.position if hasattr(p, "position") else Vec3(getattr(p, "x", 0), getattr(p, "y", 0), getattr(p, "z", 0))
        head = getattr(p, "head_position", None)
        chest = getattr(p, "chest_position", None)
        left_sh = getattr(p, "left_shoulder_position", None)
        right_sh = getattr(p, "right_shoulder_position", None)
        if head is not None and head_plausible(feet, head):
            aim, ground = head, feet
        else:
            # координаты из памяти — уже центр объёма (макет-голова), не ноги + 64
            aim = head if head is not None else feet
            ground = feet
            if abs(aim.z - feet.z) < 24.0:
                aim = feet
        spotted = bool(getattr(p, "spotted", False)) and not bool(getattr(p, "dormant", False))
        if not spotted and hasattr(p, "visible"):
            # совместимость со старым EnemyInfo.visible = spotted
            spotted = bool(p.visible) and not bool(getattr(p, "dormant", False))
        spawn_immunity = bool(getattr(p, "spawn_immunity", False))
        vel = getattr(p, "velocity", None)
        if vel is not None and hasattr(vel, "x"):
            vel_xy = (float(vel.x), float(vel.y), float(getattr(vel, "z", 0.0)))
        elif isinstance(vel, (tuple, list)) and len(vel) >= 2:
            vel_xy = (float(vel[0]), float(vel[1]), float(vel[2]) if len(vel) > 2 else 0.0)
        else:
            vel_xy = (0.0, 0.0, 0.0)
        chest_xyz = None
        if chest is not None and hasattr(chest, "x") and chest_plausible(feet, chest, head):
            chest_xyz = (chest.x, chest.y, chest.z)

        def _shoulder_xyz(sh) -> tuple[float, float, float] | None:
            if sh is None or not hasattr(sh, "x"):
                return None
            if not shoulder_plausible(feet, sh, chest, head):
                return None
            return (sh.x, sh.y, sh.z)

        left_sh_xyz = _shoulder_xyz(left_sh)
        right_sh_xyz = _shoulder_xyz(right_sh)
        return {
            "index": p.index,
            "name": getattr(p, "name", ""),
            "health": getattr(p, "health", 100),
            "position": (aim.x, aim.y, aim.z),
            "head_position": (aim.x, aim.y, aim.z),
            "chest_position": chest_xyz,
            "left_shoulder_position": left_sh_xyz,
            "right_shoulder_position": right_sh_xyz,
            "feet": (ground.x, ground.y, ground.z),
            "spotted": spotted,
            "visible": False,
            "head_visible": False,
            "chest_visible": False,
            "left_shoulder_visible": False,
            "right_shoulder_visible": False,
            "radar_spotted": bool(getattr(p, "radar_spotted", False)),
            "dormant": bool(getattr(p, "dormant", False)),
            "vulnerable": bool(getattr(p, "vulnerable", not spawn_immunity)),
            "spawn_immunity": spawn_immunity,
            "has_moved_since_spawn": bool(getattr(p, "has_moved_since_spawn", True)),
            "velocity": vel_xy,
        }

    def _note_vulnerability(self, e: dict, now: float) -> None:
        """Пока spawn-защита — не стреляем; после снятия ждём ~1 с (xxHash, не ровно секунда)."""
        idx = e["index"]
        if not e.get("vulnerable", True):
            self._immune_seen.add(idx)
            self._vuln_ready_at.pop(idx, None)
            return
        if idx not in self._immune_seen or idx in self._vuln_ready_at:
            return
        lo, hi = self.cfg.vuln_fire_delay
        wait = lo + self._xx_u01() * (hi - lo)
        self._vuln_ready_at[idx] = now + wait
        self.log(f"Мишень {e.get('name') or idx} уязвима — огонь через {wait:.2f}s")

    def _target_can_fire(self, now: float, target: dict) -> bool:
        if not target.get("vulnerable", True):
            return False
        ready = self._vuln_ready_at.get(target["index"])
        return ready is None or now >= ready

    def _read_enemies(self) -> list[dict]:
        """Все живые враги из памяти (видимые и нет). Fallback — только spotted."""
        fn = getattr(self.reader, "enemies", None)
        if not callable(fn):
            fn = getattr(self.reader, "visible_enemies", None)
        if not callable(fn):
            return []
        out: list[dict] = []
        try:
            for p in fn():
                out.append(self._enemy_from_reader(p))
        except Exception as exc:
            self.log(f"enemies: ошибка чтения ({exc}); мишени пропущены")
            return self.all_enemies
        return out

    def _read_visible_enemies(self) -> list[dict]:
        return [e for e in self._read_enemies() if e.get("visible")]

    def _read_ladders(self) -> list[dict]:
        fn = getattr(self.reader, "ladders", None)
        if not callable(fn):
            return []
        out: list[dict] = []
        try:
            for lad in fn():
                if hasattr(lad, "to_dict"):
                    out.append(lad.to_dict())
                elif isinstance(lad, dict):
                    out.append(lad)
        except Exception as exc:
            self.log(f"ladders: ошибка чтения ({exc})")
            return self.ladders
        return out

    def _read_doors(self) -> list[dict]:
        fn = getattr(self.reader, "doors", None)
        if not callable(fn):
            return []
        out: list[dict] = []
        try:
            for door in fn():
                if hasattr(door, "to_dict"):
                    out.append(door.to_dict())
                elif isinstance(door, dict):
                    out.append(door)
        except Exception as exc:
            self.log(f"doors: ошибка чтения ({exc})")
            return self.doors
        return out

    def _read_breakables(self) -> list[dict]:
        fn = getattr(self.reader, "breakables", None)
        if not callable(fn):
            return []
        out: list[dict] = []
        try:
            for br in fn():
                if hasattr(br, "to_dict"):
                    out.append(br.to_dict())
                elif isinstance(br, dict):
                    out.append(br)
        except Exception as exc:
            self.log(f"breakables: ошибка чтения ({exc})")
            return self.breakables
        return out

    def _read_on_ladder(self) -> bool:
        fn = getattr(self.reader, "local_on_ladder", None)
        if not callable(fn):
            return False
        try:
            return bool(fn())
        except Exception:
            return False

    def _sync_ladders(self, now: float, pos: XYZ, combat: bool) -> None:
        cfg = self.cfg
        if now < self._ladders_next_poll:
            return
        self._ladders_next_poll = now + 1.0 / max(0.05, cfg.ladders_poll_hz)
        live = self._read_ladders()
        if live:
            self.ladders = live
        elif not self.ladders:
            return
        n = self.map.attach_ladders(self.ladders)
        if n and not self._ladders_linked:
            self._ladders_linked = True
            self.log(f"Лестницы: {len(self.ladders)} шт., {n} связей на карте")
            if self.route is not None and not combat:
                self._replan(pos, "лестницы подключены")

    def _sync_doors(self, now: float, pos: XYZ, combat: bool) -> None:
        cfg = self.cfg
        if now < self._doors_next_poll:
            return
        self._doors_next_poll = now + 1.0 / max(0.05, cfg.doors_poll_hz)
        prev_blocked = self._doors_blocked
        self.doors = self._read_doors()
        blocked = sum(1 for d in self.doors if self._door_blocks_route(d))
        self._doors_blocked = blocked
        if self.doors and blocked != prev_blocked and self.route is not None and not combat:
            self._replan(pos, "запертая дверь")

    def _sync_breakables(self, now: float, pos: XYZ, combat: bool) -> None:
        cfg = self.cfg
        if now < self._breakables_next_poll:
            return
        self._breakables_next_poll = now + 1.0 / max(0.05, cfg.breakables_poll_hz)
        self.breakables = self._read_breakables()
        self._breakables_blocked = sum(
            1 for br in self.breakables if not br.get("passable") and not br.get("broken")
        )
        if self.cfg.verbose_log and self.breakables and not self._breakables_logged:
            self._breakables_logged = True
            blocking = [br for br in self.breakables if not br.get("passable") and not br.get("broken")]
            self.log(f"Решётки/окна: {len(self.breakables)} шт., {len(blocking)} на прострел")
            for br in blocking[:8]:
                self.log(self._breakable_summary(br))
            if len(blocking) > 8:
                self.log(f"  … ещё {len(blocking) - 8}")

    @staticmethod
    def _vec3_dict(raw) -> tuple[float, float, float] | None:
        if raw is None:
            return None
        if isinstance(raw, dict):
            try:
                return float(raw["x"]), float(raw["y"]), float(raw["z"])
            except (KeyError, TypeError, ValueError):
                return None
        if hasattr(raw, "x"):
            return float(raw.x), float(raw.y), float(raw.z)
        if isinstance(raw, (tuple, list)) and len(raw) >= 3:
            return float(raw[0]), float(raw[1]), float(raw[2])
        return None

    @classmethod
    def _object_aabb(cls, obj: dict, margin: float = 0.0) -> BoxObstacle | None:
        mins = cls._vec3_dict(obj.get("bounds_mins"))
        maxs = cls._vec3_dict(obj.get("bounds_maxs"))
        if mins and maxs:
            span = max(abs(maxs[0] - mins[0]), abs(maxs[1] - mins[1]), abs(maxs[2] - mins[2]))
            if span >= 1.0:
                m = margin
                return (
                    min(mins[0], maxs[0]) - m,
                    min(mins[1], maxs[1]) - m,
                    min(mins[2], maxs[2]),
                    max(mins[0], maxs[0]) + m,
                    max(mins[1], maxs[1]) + m,
                    max(mins[2], maxs[2]),
                )
        pos = cls._vec3_dict(obj.get("position"))
        if not pos:
            return None
        try:
            width = max(float(obj.get("width") or 32.0), 4.0)
            depth = max(float(obj.get("depth") or width), 4.0)
            height = max(float(obj.get("height") or 64.0), 4.0)
        except (TypeError, ValueError):
            return None
        m = margin
        return (
            pos[0] - width * 0.5 - m,
            pos[1] - depth * 0.5 - m,
            pos[2],
            pos[0] + width * 0.5 + m,
            pos[1] + depth * 0.5 + m,
            pos[2] + height,
        )

    @classmethod
    def _breakable_summary(cls, br: dict) -> str:
        pos = cls._vec3_dict(br.get("position")) or (0.0, 0.0, 0.0)
        mins = cls._vec3_dict(br.get("bounds_mins"))
        maxs = cls._vec3_dict(br.get("bounds_maxs"))
        name = str(br.get("name") or br.get("model") or "?")[:28]
        w = float(br.get("width") or 0.0)
        d = float(br.get("depth") or 0.0)
        h = float(br.get("height") or 0.0)
        if mins and maxs:
            bounds = (
                f"bbox=({mins[0]:.0f},{mins[1]:.0f},{mins[2]:.0f})"
                f"→({maxs[0]:.0f},{maxs[1]:.0f},{maxs[2]:.0f})"
            )
        else:
            bounds = "bbox=?"
        return (
            f"  #{int(br.get('index') or 0):4} {name:28} "
            f"pos=({pos[0]:.0f},{pos[1]:.0f},{pos[2]:.0f}) "
            f"{w:.0f}x{d:.0f}x{h:.0f} {bounds}"
        )

    def _known_ladders(self) -> list[dict]:
        seen: set[tuple[int, int, int, int, int, int]] = set()
        out: list[dict] = []
        extras = [lk.get("ladder") for lk in getattr(self.map, "ladder_links", [])]
        for raw in list(self.ladders) + extras:
            lad = _ladder_as_dict(raw)
            if not lad:
                continue
            bot, top = lad["bottom"], lad["top"]
            key = (
                round(bot[0]), round(bot[1]), round(bot[2]),
                round(top[0]), round(top[1]), round(top[2]),
            )
            if key in seen:
                continue
            seen.add(key)
            out.append(lad)
        return out

    def _ladder_facing(self, lad: dict) -> tuple[float, float]:
        dirn = str(lad.get("direction") or "").strip().lower()
        nav_face = {
            "north": (0.0, 1.0),
            "south": (0.0, -1.0),
            "east": (1.0, 0.0),
            "west": (-1.0, 0.0),
        }
        if dirn in nav_face:
            return nav_face[dirn]
        bot = lad["bottom"]
        mins = lad.get("bounds_mins")
        maxs = lad.get("bounds_maxs")
        if isinstance(mins, (tuple, list)) and isinstance(maxs, (tuple, list)):
            cx = (mins[0] + maxs[0]) * 0.5
            cy = (mins[1] + maxs[1]) * 0.5
            fx, fy = bot[0] - cx, bot[1] - cy
            ln = math.hypot(fx, fy)
            if ln >= 4.0:
                return fx / ln, fy / ln
            sx = abs(maxs[0] - mins[0])
            sy = abs(maxs[1] - mins[1])
            if sx <= sy:
                return (1.0 if bot[0] >= cx else -1.0), 0.0
            return 0.0, (1.0 if bot[1] >= cy else -1.0)
        top = lad["top"]
        fx, fy = bot[0] - top[0], bot[1] - top[1]
        ln = math.hypot(fx, fy)
        if ln < 1.0:
            return 1.0, 0.0
        return fx / ln, fy / ln

    def _match_ladder(self, a: XYZ, b: XYZ) -> dict | None:
        lo_z, hi_z = (a[2], b[2]) if a[2] <= b[2] else (b[2], a[2])
        best = None
        best_d = 110.0
        for lad in self._known_ladders():
            bot, top = lad["bottom"], lad["top"]
            if top[2] < bot[2]:
                bot, top = top, bot
            d = min(
                math.hypot(a[0] - bot[0], a[1] - bot[1]) + abs(a[2] - bot[2]) * 0.4,
                math.hypot(a[0] - top[0], a[1] - top[1]) + abs(a[2] - top[2]) * 0.4,
                math.hypot(b[0] - bot[0], b[1] - bot[1]) + abs(b[2] - bot[2]) * 0.4,
                math.hypot(b[0] - top[0], b[1] - top[1]) + abs(b[2] - top[2]) * 0.4,
            )
            if abs(bot[2] - lo_z) > 96.0 and abs(top[2] - hi_z) > 96.0:
                continue
            if d < best_d:
                best_d, best = d, lad
        return best

    def _nearest_ladder(self, x: float, y: float, z: float, max_d: float = 72.0) -> dict | None:
        best = None
        best_d = max_d
        for lad in self._known_ladders():
            bot, top = lad["bottom"], lad["top"]
            if top[2] < bot[2]:
                bot, top = top, bot
            lo_z, hi_z = bot[2] - 24.0, top[2] + 24.0
            if z < lo_z or z > hi_z:
                continue
            d = min(
                math.hypot(x - bot[0], y - bot[1]),
                math.hypot(x - top[0], y - top[1]),
            )
            if d < best_d:
                best_d, best = d, lad
        return best

    def _route_uses_ladder(self, route: Route, lad: dict, s: float, ahead: float = 200.0) -> bool:
        bot, top = lad["bottom"], lad["top"]
        if top[2] < bot[2]:
            bot, top = top, bot
        i = max(0, route.next_vertex_index(s) - 1)
        end_s = s + ahead
        near_bot = near_top = False
        while i < len(route.pts) and route.cum[i] <= end_s + 16.0:
            p = route.pts[i]
            if math.hypot(p[0] - bot[0], p[1] - bot[1]) < 72.0 and abs(p[2] - bot[2]) < 48.0:
                near_bot = True
            if math.hypot(p[0] - top[0], p[1] - top[1]) < 72.0 and abs(p[2] - top[2]) < 48.0:
                near_top = True
            i += 1
        return near_bot and near_top

    def _ladder_ends(self, lad: dict) -> tuple[tuple, tuple]:
        bot, top = lad["bottom"], lad["top"]
        if top[2] < bot[2]:
            return top, bot
        return bot, top

    def _ladder_exit_side(self, lad: dict, dest: tuple[float, float], going_up: bool) -> str:
        """Куда уходит маршрут после лестницы: left / right / forward / back."""
        bot, top = self._ladder_ends(lad)
        exit_pt = top if going_up else bot
        fx, fy = self._ladder_facing(lad)
        dx, dy = dest[0] - exit_pt[0], dest[1] - exit_pt[1]
        fwd = dx * fx + dy * fy
        right = dx * (-fy) + dy * fx
        if fwd < -20.0 and abs(fwd) >= abs(right) * 0.85:
            return "back"
        if abs(right) > 16.0 and abs(right) > abs(fwd) * 0.55:
            return "right" if right > 0.0 else "left"
        return "forward"

    def _begin_ladder(
        self,
        now: float,
        route: Route,
        lad: dict,
        going_up: bool,
        idx: int,
        on: bool,
        x: float,
        y: float,
    ) -> None:
        fx, fy = self._ladder_facing(lad)
        bot, top = self._ladder_ends(lad)
        mount = bot if going_up else top
        mx, my = mount[0] + fx * 22.0, mount[1] + fy * 22.0
        exit_z = top[2] if going_up else bot[2]
        k = max(idx, route.next_vertex_index(self.s))
        while k + 1 < len(route.pts) and abs(route.pts[k][2] - exit_z) > 28.0:
            k += 1
        dest = route.pts[min(len(route.pts) - 1, k + 1)]
        side = self._ladder_exit_side(lad, (dest[0], dest[1]), going_up)
        exit_pt = top if going_up else bot
        dx, dy = dest[0] - exit_pt[0], dest[1] - exit_pt[1]
        right = dx * (-fy) + dy * fx
        near_mount = math.hypot(mx - x, my - y) < 28.0
        self._ladder = {
            "lad": lad,
            "up": going_up,
            "phase": "mount" if (on or near_mount) else "approach",
            "mx": mx,
            "my": my,
            "side": side,
            "strafe": "d" if right >= 0.0 else "a",
            "t0": now,
            "seen": on,
        }
        self.log(f"Лестница ({'вверх' if going_up else 'вниз'}, выход {side})")

    def _update_ladder(self, now: float, route: Route, x: float, y: float, z: float) -> None:
        on = self._read_on_ladder()
        # cooldown: не хвататься за лестницу сразу после "сошли/достигли конца"
        if self._ladder is None and now < getattr(self, "_ladder_cooldown_until", 0.0):
            return
        if self._ladder is not None:
            st = self._ladder
            lad = st["lad"]
            going_up = bool(st["up"])
            bot, top = self._ladder_ends(lad)
            exit_z = top[2] if going_up else bot[2]
            done_z = (z >= exit_z - 14.0) if going_up else (z <= exit_z + 18.0)
            if st["phase"] == "exit":
                if math.hypot(st["mx"] - x, st["my"] - y) < 28.0 or now - st.get("exit_t0", st["t0"]) > 1.8:
                    self._ladder = None
                    self.log("Лестница: обошли дыру")
                return
            if now - st["t0"] > 14.0:
                self._ladder = None
                return
            if on:
                st["phase"] = "climb"
                st["seen"] = True
                # если достигли выхода — сойти с лестницы и не хвататься за неё повторно
                reached_end = (z >= exit_z - 4.0) if going_up else (z <= exit_z + 12.0)
                if reached_end:
                    self._ladder = None
                    self._ladder_cooldown_until = now + 2.0
                    self._stuck_level = 0
                    self._stuck_since = None
                    self._override_until = 0.0
                    self._override_keys = set()
                    self.log("Лестница: достигли конца")
                    return
                return
            if st.get("seen") and done_z:
                if st["side"] == "back":
                    fx, fy = self._ladder_facing(lad)
                    sx, sy = (-fy, fx) if st.get("strafe") == "d" else (fy, -fx)
                    exit_pt = top if going_up else bot
                    st["phase"] = "exit"
                    st["mx"] = exit_pt[0] + sx * 56.0 + fx * 14.0
                    st["my"] = exit_pt[1] + sy * 56.0 + fy * 14.0
                    st["exit_t0"] = now
                    self.log("Лестница: обход дыры")
                    return
                self._ladder = None
                self._ladder_cooldown_until = now + 2.0
                self._stuck_level = 0
                self._stuck_since = None
                self._override_until = 0.0
                self._override_keys = set()
                self.log("Лестница: сошли")
                return
            if st["phase"] in ("approach", "mount") and done_z and math.hypot(st["mx"] - x, st["my"] - y) > 48.0:
                # уже на том этаже (ящики / пандус) — лестница не нужна
                self._ladder = None
                return
            if st["phase"] == "approach" and math.hypot(st["mx"] - x, st["my"] - y) < 26.0:
                st["phase"] = "mount"
            return
        if on:
            lad = self._nearest_ladder(x, y, z, max_d=96.0)
            if lad is not None:
                top_z = max(lad["top"][2], lad["bottom"][2])
                bot_z = min(lad["top"][2], lad["bottom"][2])
                # куда идти — выбираем по целевому z дальше по маршруту
                ahead_s = min(route.length, self.s + max(90.0, (top_z - bot_z) + 40.0))
                ahead_z = route.point_at(ahead_s)[2]
                # маршрут не требует подниматься/спускаться — не лезем на лестницу
                if abs(ahead_z - z) < 24.0:
                    return
                going_up = ahead_z >= z + 24.0
                idx = route.next_vertex_index(self.s)
                self._begin_ladder(now, route, lad, going_up, idx, True, x, y)
            return
        span = route.vertical_span_ahead(self.s, 160.0, self.map.cfg.jump_height + 10.0)
        lad = None
        going_up = True
        idx = route.next_vertex_index(self.s)
        if span is not None:
            a, b, going_up, idx = span
            lad = self._match_ladder(a, b)
        if lad is None:
            for cand in self._known_ladders():
                if self._route_uses_ladder(route, cand, self.s):
                    bot, top = cand["bottom"], cand["top"]
                    if top[2] < bot[2]:
                        bot, top = top, bot
                    going_up = abs(z - bot[2]) <= abs(z - top[2]) + 8.0
                    lad = cand
                    break
        if lad is None:
            return
        self._begin_ladder(now, route, lad, going_up, idx, on, x, y)

    def _ladder_keys(self, x: float, y: float, z: float, yaw: float, speed: float) -> set[str]:
        st = self._ladder
        assert st is not None
        lad = st["lad"]
        going_up = bool(st["up"])
        phase = st["phase"]
        on = self._read_on_ladder()
        if on:
            st["phase"] = "climb"
            st["seen"] = True
            bot, top = self._ladder_ends(lad)
            exit_z = top[2] if going_up else bot[2]
            near_exit = (exit_z - z < 36.0) if going_up else (z - exit_z < 32.0)
            # на лестнице W + корректный pitch делает подъём/спуск; S даёт обратное.
            if near_exit:
                side = st["side"]
                if side == "left":
                    return {"w", "a"}
                if side == "right":
                    return {"w", "d"}
                if side == "back":
                    return {"w", str(st.get("strafe") or "d")}
            return {"w"}
        mx, my = st["mx"], st["my"]
        bearing = yaw_to_target(x, y, mx, my)
        rel = shortest_yaw_err(bearing, yaw)
        if phase == "exit" or (phase == "approach" and math.hypot(mx - x, my - y) > 26.0):
            return self._choose_keys(rel, rel, speed, engaged=False)
        # у входа — только вперёд (W), подъём/спуск определяется pitch'ом
        if abs(rel) > 55.0:
            return {"a"} if rel > 0 else {"d"}
        return {"w"}

    def _ladder_look(self, x: float, y: float, z: float) -> tuple[float, float]:
        st = self._ladder
        assert st is not None
        lad = st["lad"]
        going_up = bool(st["up"])
        fx, fy = self._ladder_facing(lad)
        bot, top = self._ladder_ends(lad)
        phase = st["phase"]
        pitch_lim = 50.0
        if phase in ("approach", "exit"):
            tx, ty, tz = st["mx"], st["my"], z + 8.0
        elif going_up and phase == "mount":
            tx, ty = bot[0] - fx * 6.0, bot[1] - fy * 6.0
            tz = bot[2] + 18.0
        elif going_up:
            tx, ty, tz = top[0] - fx * 8.0, top[1] - fy * 8.0, min(top[2], z + 70.0)
        else:
            # спуск: смотреть вниз. Горизонтальный взгляд, как на подъёме, почти не двигает W.
            near_floor = z - bot[2] < 42.0
            if near_floor:
                tx, ty, tz = bot[0] - fx * 8.0, bot[1] - fy * 8.0, bot[2] + 12.0
            else:
                pitch_lim = 74.0
                tx, ty = x - fx * 16.0, y - fy * 16.0
                tz = z + self._eye_z - 120.0
        yaw = yaw_to_target(x, y, tx, ty)
        horiz = math.hypot(tx - x, ty - y)
        pitch = -math.degrees(math.atan2(tz - (z + self._eye_z), max(1.0, horiz)))
        return yaw, max(-55.0, min(pitch_lim, pitch))

    def _want_crouch(self, x: float, y: float, z: float, route: Route, cx: float, cy: float, cz: float) -> bool:
        if self._ladder is not None:
            return False
        r = 28.0
        if self.map.near_crouch(x, y, z, radius_xy=r):
            return True
        if self.map.near_crouch(cx, cy, cz, radius_xy=r):
            return True
        ax, ay, az = route.point_at(min(route.length, self.s + 36.0))
        return self.map.near_crouch(ax, ay, az, radius_xy=r)

    def _read_weapon(self) -> dict:
        fn = getattr(self.reader, "local_weapon", None)
        if not callable(fn):
            return self.weapon
        try:
            w = fn()
            if hasattr(w, "to_dict"):
                d = w.to_dict()
            else:
                d = {
                    "id": int(getattr(w, "id", 0) or 0),
                    "name": str(getattr(w, "name", "") or ""),
                    "clip": int(getattr(w, "clip", 0) or 0),
                    "in_reload": bool(getattr(w, "in_reload", False)),
                }
            item_id = int(d.get("id") or 0)
            name = str(d.get("name") or "")
            d["clip"] = max(0, int(d.get("clip") or 0))
            d["fire"] = weapon_fire_mode(item_id, name)
            d["auto"] = weapon_is_auto(item_id, name)
            return d
        except Exception as exc:
            self.log(f"weapon: ошибка чтения ({exc})")
            return self.weapon

    def _read_life(self) -> dict:
        """Смерть/респawn из драйвера (poll_life — импульсы died/respawned на тик)."""
        fn = getattr(self.reader, "poll_life", None)
        if not callable(fn):
            return self.life
        try:
            upd = fn()
            if hasattr(upd, "to_dict"):
                return upd.to_dict()
            return {
                "alive": bool(getattr(upd, "alive", True)),
                "status": getattr(getattr(upd, "state", None), "status", "alive"),
                "health": int(getattr(getattr(upd, "state", None), "health", 0) or 0),
                "events": {
                    "died": bool(getattr(getattr(upd, "events", None), "died", False)),
                    "respawned": bool(getattr(getattr(upd, "events", None), "respawned", False)),
                },
            }
        except Exception as exc:
            self.log(f"life: ошибка чтения ({exc})")
            return self.life

    def _in_fight(self) -> bool:
        return bool(self.engaged or self._target is not None or self._fire_held)

    def _weapon_ready(self) -> bool:
        w = self.weapon
        if w.get("fire") == "none":
            return False
        clip = int(w.get("clip") or 0)
        # в перестрелке выстрел с остатком обоймы прерывает перезарядку
        if w.get("in_reload") and not (self._in_fight() and clip > 0):
            return False
        h = self._habit
        if h and h.get("kind") in {"reload", "knife", "unscope_knife", "healthshot"}:
            if not (h.get("kind") == "reload" and self._in_fight() and clip > 0):
                return False
        return clip > 0

    def _read_shots_fired(self) -> int:
        fn = getattr(self.reader, "local_shots_fired", None)
        if callable(fn):
            try:
                return max(0, int(fn() or 0))
            except Exception:
                pass
        return max(0, self._shots_on_target)

    def _read_scoped(self) -> bool:
        fn = getattr(self.reader, "local_is_scoped", None)
        if not callable(fn):
            return self._scope_held or self._scoped_now
        try:
            return bool(fn())
        except Exception:
            return self._scoped_now

    def _rcs_resample(self) -> None:
        """Новый спрей: лёгкий хаос xxHash. Разброс сжат, under всегда <1 (как игрок:
        чуть недо-компенсируем, не перекидываем камеру за траекторию отдачи)."""
        self._rcs_scale_x = 0.955 + self._xx_u01() * 0.040   # 0.955..0.995: не overshoot по yaw
        self._rcs_scale_y = 0.960 + self._xx_u01() * 0.035   # 0.960..0.995: не overshoot по pitch
        self._rcs_rot = (self._xx_u01() - 0.5) * 0.9          # ±0.45° поворот паттерна
        self._rcs_lag = 0.020 + self._xx_u01() * 0.020       # 0.020..0.040 с реакция руки
        self._rcs_under = 0.955 + self._xx_u01() * 0.035     # 0.955..0.990 всегда недо-коррекция
        self._rcs_arc = (self._xx_u01() - 0.5) * 0.08         # ±0.04 — мягкий изгиб линии
        self._rcs_rate_var = (self._xx_u01() - 0.5) * self.cfg.rcs_rate_jitter
        self._rcs_sampled = True

    def _rcs_reset(self) -> None:
        self._rcs_sampled = False
        self._rcs_shots_seen = 0
        self._rcs_shot_t = 0.0

    def _rcs_active(self) -> bool:
        if not self.cfg.rcs_enabled or not self.weapon.get("auto"):
            return False
        if self._long_spray is False:
            return False
        return bool(self._fire_held or self._mem_shots > 0)

    def _tap_wait(self) -> float:
        lo, hi = tap_gap_range(int(self.weapon.get("id") or 0), str(self.weapon.get("name") or ""))
        return lo + self._xx_u01() * (hi - lo)

    def _auto_tap_wait(self) -> float:
        """Пауза между тапами на автомате с дальней дистанции: цикл + 0.14…0.32 с."""
        cycle = weapon_cycle_s(int(self.weapon.get("id") or 0), str(self.weapon.get("name") or ""))
        return cycle + 0.14 + self._xx_u01() * 0.18

    def _should_spray(self, dist: float) -> bool:
        """Зажим только вблизи; на long_tap_dist автомат тапает, кроме ~20% зажима с RCS."""
        if not self.weapon.get("auto"):
            return False
        if dist < self.cfg.long_tap_dist:
            if self._long_spray is False:
                self._long_spray = None
            return True
        if self._long_spray is None:
            self._long_spray = self._xx_u01() < self.cfg.long_spray_chance
            self.log("Дальний зажим с антиотдачей" if self._long_spray else "Дальний тап")
        return self._long_spray

    def _pulse_key(self, key: str, now: float, hold: float | None = None) -> None:
        if hold is None:
            hold = 0.036 + self._xx_u01() * 0.022
        self._extra_key_until[key] = now + hold

    def _key_gap(self) -> float:
        """~четверть секунды ±8 мс, xxHash."""
        return 0.250 + (self._xx_u01() * 2.0 - 1.0) * 0.008

    def _merge_extra_keys(self, keys: set[str], now: float) -> set[str]:
        expired = [k for k, t in self._extra_key_until.items() if now >= t]
        for k in expired:
            del self._extra_key_until[k]
        if not self._extra_key_until:
            return keys
        return set(keys) | set(self._extra_key_until)

    def _nearby_targets(self, x: float, y: float, z: float) -> bool:
        dmax = self.cfg.habit_clear_dist
        for e in self.known_targets:
            p = e.get("position")
            if not p:
                continue
            if math.hypot(p[0] - x, p[1] - y) <= dmax:
                return True
        return False

    def _gun_slot_key(self) -> str:
        wid = int(self.weapon.get("id") or 0)
        name = str(self.weapon.get("name") or "")
        return "2" if weapon_is_pistol(wid, name) else "1"

    def _reload_frac_for_weapon(self) -> float:
        """Порог оставшейся обоймы: ~20% (±диапазон), редко ~60%, малый магазин 5–10% ±1%."""
        wid = int(self.weapon.get("id") or 0)
        name = str(self.weapon.get("name") or "")
        mag = max(1, default_clip(wid, name))
        if mag <= 13:
            frac = 0.05 + self._xx_u01() * 0.05
            frac += (self._xx_u01() * 2.0 - 1.0) * 0.01
            return max(0.03, min(0.12, frac))
        if self._xx_u01() < 0.022:
            return 0.59 + self._xx_u01() * 0.02
        frac = 0.17 + self._xx_u01() * 0.06
        if self._xx_u01() < 0.11:
            frac += 0.01 if self._xx_u01() < 0.5 else -0.01
        return max(0.12, min(0.28, frac))

    def _sync_reload_threshold(self) -> None:
        w = self.weapon
        wid = int(w.get("id") or 0)
        clip = int(w.get("clip") or 0)
        if wid != self._reload_wid:
            self._reload_wid = wid
            self._reload_frac = self._reload_frac_for_weapon()
            self._reload_skip = False
        elif clip > self._reload_clip_seen:
            self._reload_frac = self._reload_frac_for_weapon()
            self._reload_skip = False
        self._reload_clip_seen = clip

    def _reload_threshold_clip(self) -> int:
        wid = int(self.weapon.get("id") or 0)
        name = str(self.weapon.get("name") or "")
        mag = max(1, default_clip(wid, name), int(self.weapon.get("clip") or 0))
        return max(1, int(round(mag * self._reload_frac)))

    def _habit_abort(self, now: float, back_to_gun: bool = False) -> None:
        h = self._habit
        self._habit = None
        if back_to_gun and h and h.get("kind") in {"knife", "unscope_knife", "healthshot"} and h.get("phase") != "done":
            self._pulse_key(str(h.get("back") or self._gun_slot_key()), now)
        self._habit_next = now + 8.0 + self._xx_u01() * 8.0

    def _start_reload(self, now: float) -> None:
        self._release_fire()
        self._pulse_key("r", now)
        mag = max(1, default_clip(int(self.weapon.get("id") or 0), str(self.weapon.get("name") or "")))
        self.log(
            f"Перезарядка (R), обойма {int(self.weapon.get('clip') or 0)}/{mag}, "
            f"порог {self._reload_frac * 100:.0f}%"
        )
        self._habit = {"kind": "reload", "t0": now, "until": now + 3.6}

    def _start_inspect(self, now: float) -> None:
        self._pulse_key("f", now)
        hold = 2.4 + self._xx_u01() * 1.2
        self.log("Осмотр оружия")
        self._habit = {"kind": "inspect", "t0": now, "until": now + hold}

    def _start_knife(self, now: float) -> None:
        back = self._gun_slot_key()
        self._pulse_key("3", now)
        self.log(f"Нож → слот {back}")
        self._habit = {
            "kind": "knife",
            "phase": "to_knife",
            "back": back,
            "t0": now,
            "next": now + self._key_gap(),
        }

    def _habit_advance(self, now: float, *, nearby: bool, busy: bool) -> None:
        h = self._habit
        if h is None:
            return
        kind = h.get("kind")
        if kind == "reload":
            clip = int(self.weapon.get("clip") or 0)
            if busy and clip > 0:
                self._habit = None
                self.log("Перезарядка отменена: перестрелка, патроны ещё есть")
                return
            if now >= float(h.get("until") or 0.0):
                self._habit = None
                return
            if self.weapon.get("in_reload"):
                h["saw_reload"] = True
            elif h.get("saw_reload"):
                self._habit = None
            return
        if kind == "inspect":
            if nearby or busy or now >= float(h.get("until") or 0.0):
                self._habit_abort(now)
            return
        if kind == "healthshot":
            self._healthshot_advance(now, busy=busy)
            return
        if kind != "knife" and kind != "unscope_knife":
            self._habit = None
            return
        if kind == "knife" and (nearby or busy):
            self._habit_abort(now, back_to_gun=True)
            return
        phase = h.get("phase")
        if phase == "to_knife" and now >= float(h.get("next") or 0.0):
            self._pulse_key(str(h.get("back") or "1"), now)
            h["phase"] = "to_gun"
            h["next"] = now + self._key_gap()
            return
        if phase == "to_gun" and now >= float(h.get("next") or 0.0):
            self._habit = None
            if kind == "unscope_knife":
                self.log("Скоуп сброшен (нож)")
            self._habit_next = now + 14.0 + self._xx_u01() * 10.0

    def _maybe_reload(self, now: float, nearby: bool, *, fighting: bool = False) -> bool:
        w = self.weapon
        if not is_gun(int(w.get("id") or 0), str(w.get("name") or "")):
            return False
        if w.get("in_reload") and not (fighting and int(w.get("clip") or 0) > 0):
            return False
        if self._flick is not None or self._flick_pending:
            return False
        clip = int(w.get("clip") or 0)
        mag = max(1, default_clip(int(w.get("id") or 0), str(w.get("name") or "")))
        if clip >= mag:
            return False
        # перестрелка: порог обоймы игнорируем, R только на пустом магазине
        if fighting and clip > 0:
            return False
        if clip > self._reload_threshold_clip():
            return False
        if nearby:
            if self._reload_skip:
                return False
            if self._xx_u01() >= self.cfg.combat_reload_chance:
                self._reload_skip = True
                return False
        self._start_reload(now)
        return True

    def _sample_healthshot_thr(self) -> float:
        cfg = self.cfg
        if self._xx_u01() < cfg.healthshot_early_chance:
            lo, hi = cfg.healthshot_early_hp
            return lo + self._xx_u01() * (hi - lo)
        return cfg.healthshot_hp

    def _healthshot_thr(self) -> float:
        if self._hpshot_thr is None:
            self._hpshot_thr = self._sample_healthshot_thr()
        return self._hpshot_thr

    def _has_healthshot(self) -> bool:
        if is_healthshot(int(self.weapon.get("id") or 0), str(self.weapon.get("name") or "")):
            return True
        fn = getattr(self.reader, "local_has_healthshot", None)
        if not callable(fn):
            return False
        try:
            return bool(fn())
        except Exception:
            return False

    def _maybe_healthshot(self, now: float, *, busy: bool) -> bool:
        if busy or self._habit is not None:
            return False
        if not self.life.get("alive", True):
            return False
        hp = int(self.life.get("health") or 0)
        if hp <= 0:
            return False
        if hp > self._healthshot_thr():
            return False
        if not self._has_healthshot():
            return False
        self._start_healthshot(now)
        return True

    def _start_healthshot(self, now: float) -> None:
        self._release_fire()
        back = self._gun_slot_key()
        already = is_healthshot(int(self.weapon.get("id") or 0), str(self.weapon.get("name") or ""))
        self._habit = {
            "kind": "healthshot",
            "phase": "inject" if already else "pick",
            "back": back,
            "t0": now,
            "next": now,
            "tries": 0,
            "seen": set(),
        }
        thr = self._healthshot_thr()
        self.log(f"Медшприц (HP {int(self.life.get('health') or 0):.0f} ≤ {thr:.0f})")
        if already:
            self._healthshot_click(now)

    def _injecting(self) -> bool:
        h = self._habit
        return bool(h and h.get("kind") == "healthshot" and h.get("phase") in {"inject", "wait"})

    def _healthshot_click(self, now: float) -> None:
        h = self._habit
        if h is None:
            return
        self.input.mouse_down("left")
        self._fire_held = True
        # держим ЛКМ всю анимацию укола; короткий клик игра не засчитывает и слот сразу бросается
        self._fire_release_at_tick = -1
        lo, hi = self.cfg.healthshot_inject
        h["phase"] = "wait"
        h["next"] = now + lo + self._xx_u01() * (hi - lo)

    def _healthshot_advance(self, now: float, *, busy: bool) -> None:
        h = self._habit
        if h is None or h.get("kind") != "healthshot":
            return
        phase = h.get("phase")
        if phase == "pick":
            wid = int(self.weapon.get("id") or 0)
            name = str(self.weapon.get("name") or "")
            if is_healthshot(wid, name):
                self._healthshot_click(now)
                return
            if busy:
                self._habit_abort(now, back_to_gun=True)
                return
            if now < float(h.get("next") or 0.0):
                return
            if wid:
                h.setdefault("seen", set()).add(wid)
            tries = int(h.get("tries") or 0) + 1
            h["tries"] = tries
            if tries > 16 or len(h.get("seen") or ()) >= 12:
                self.log("Медшприц: не нашли в слотах")
                self._habit_abort(now, back_to_gun=True)
                return
            fn = getattr(self.input, "mouse_wheel", None)
            if callable(fn):
                fn(-1)
            h["next"] = now + 0.08 + self._xx_u01() * 0.07
            return
        if phase == "inject":
            self._healthshot_click(now)
            return
        if phase == "wait":
            if now < float(h.get("next") or 0.0):
                return
            if self._fire_held:
                self.input.mouse_up("left")
                self._fire_held = False
            self._pulse_key(str(h.get("back") or "1"), now)
            h["phase"] = "to_gun"
            h["next"] = now + self._key_gap()
            return
        if phase == "to_gun" and now >= float(h.get("next") or 0.0):
            self._habit = None
            self._hpshot_thr = self._sample_healthshot_thr()
            self.log("Медшприц использован")

    def _habit_tick(self, now: float, x: float, y: float, z: float) -> None:
        h = self._habit
        if not self.cfg.habits_enabled:
            if h and h.get("kind") in {"unscope_knife", "healthshot"}:
                self._habit_advance(now, nearby=False, busy=False)
                return
            if h:
                self._habit_abort(now, back_to_gun=True)
            if self._maybe_healthshot(now, busy=False):
                return
            return
        self._sync_reload_threshold()
        nearby = self._nearby_targets(x, y, z)
        busy = (
            self.engaged
            or self._target is not None
            or self._fire_held
            or self._flick is not None
            or self._flick_pending
        )
        if self._habit:
            self._habit_advance(now, nearby=nearby, busy=busy)
            return
        if self._maybe_healthshot(now, busy=busy):
            return
        if self._maybe_reload(now, nearby, fighting=busy):
            return
        if busy or nearby:
            return
        if self._habit_next <= 0.0:
            self._habit_next = now + 11.0 + self._xx_u01() * 8.0
            return
        if now < self._habit_next:
            return
        w = self.weapon
        if w.get("in_reload") or not is_gun(int(w.get("id") or 0), str(w.get("name") or "")):
            self._habit_next = now + 4.0 + self._xx_u01() * 5.0
            return
        knife_p = 0.10 + self._xx_u01() * 0.05
        inspect_p = 0.40 + self._xx_u01() * 0.10
        roll = self._xx_u01()
        if roll < knife_p:
            self._start_knife(now)
            return
        if roll < knife_p + inspect_p:
            self._start_inspect(now)
            return
        self._habit_next = now + 7.0 + self._xx_u01() * 9.0

    def _rcs_offset(self, now: float) -> tuple[float, float]:
        """Куда сместить взгляд относительно головы: линия спрея с лагом и кривой."""
        if not self._rcs_active() or self._rcs_return or not self._fire_held:
            return 0.0, 0.0
        wid = int(self.weapon.get("id") or 0)
        wname = str(self.weapon.get("name") or "")
        n = self._mem_shots
        if n != self._rcs_shots_seen:
            if self._rcs_shots_seen == 0 and n > 0:
                self._rcs_resample()
            self._rcs_shots_seen = n
            self._rcs_shot_t = now
        if n <= 0:
            return 0.0, 0.0
        cycle = weapon_cycle_s(wid, wname)
        t = now - self._rcs_lag
        dt_shot = max(0.0, t - (self._rcs_shot_t - self._rcs_lag))
        frac = min(1.0, dt_shot / max(0.05, cycle)) if t >= self._rcs_shot_t - self._rcs_lag else 0.0
        shots_lag = n
        if t < self._rcs_shot_t:
            shots_lag = max(0, n - 1)
            frac = 1.0
        y, p = recoil_offset(wid, wname, shots_lag, frac)
        rad = math.radians(self._rcs_rot)
        c, s = math.cos(rad), math.sin(rad)
        y, p = y * c - p * s, y * s + p * c
        y *= self._rcs_scale_x * self._rcs_under
        p *= self._rcs_scale_y * self._rcs_under
        mag = math.hypot(y, p)
        if mag > 0.15:
            prog = min(1.0, n / 12.0)
            bend = self._rcs_arc * math.sin(math.pi * prog)
            ny, np_ = -p / mag, y / mag
            y += ny * bend
            p += np_ * bend * 0.35
        return y, p

    def _rcs_step(self, yaw_err: float, pitch_err: float, dt: float) -> tuple[float, float]:
        """Ведем линию как рукой: S-кривая, плавающая скорость, ветер WindMouse."""
        cfg = self.cfg
        j = cfg.rcs_rate_jitter
        decay = 0.92
        kick = math.sqrt(max(0.0, 1.0 - decay * decay)) * j
        self._rcs_rate_var = self._rcs_rate_var * decay + (self._xx_u01() * 2.0 - 1.0) * kick
        self._rcs_rate_var = max(-j, min(j, self._rcs_rate_var))
        rate = cfg.rcs_rate * (1.0 + self._rcs_rate_var)
        old_sigma = cfg.hand_wind_sigma
        cfg.hand_wind_sigma = cfg.rcs_wind_sigma
        try:
            return self._camera_step(
                yaw_err, pitch_err, dt, 0.0,
                rate_max=rate, gain=cfg.rcs_gain,
                pitch_gain=cfg.rcs_gain, pitch_rate_max=rate,
                deadband=0.04, wind=True,
            )
        finally:
            cfg.hand_wind_sigma = old_sigma

    def _begin_rcs_return(self) -> None:
        """После зажима: вернуть прицел быстро и плавно, скорость как у RCS, путь WindMouse как на маршруте."""
        if self._rcs_return:
            return
        if not self._rcs_sampled or self._rcs_shots_seen < 1:
            return
        if self._long_spray is False:
            return
        lo, hi = self.cfg.rcs_return_mul
        base = self.cfg.rcs_rate * (1.0 + self._rcs_rate_var)
        self._rcs_return_rate = base * (lo + self._xx_u01() * (hi - lo))
        self._rcs_return = True
        self._rcs_return_until = self._now() + 1.45
        self._rcs_return_seeded = False
        self._flick = None
        self._flick_pending = False
        self._correct_from = 0.0

    def _rcs_return_going(self, now: float, yaw_err: float, pitch_err: float) -> bool:
        if not self._rcs_return:
            return False
        if self._fire_held and self._rcs_active():
            self._rcs_return = False
            return False
        if math.hypot(yaw_err, pitch_err) < self.cfg.rcs_return_done or now > self._rcs_return_until:
            self._rcs_return = False
            return False
        return True

    def _rcs_return_step(self, yaw_err: float, pitch_err: float, dt: float) -> tuple[float, float]:
        """Возврат: стартовая скорость как у RCS, ветер — тот же, что панорама маршрута."""
        if not self._rcs_return_seeded:
            mag = math.hypot(yaw_err, pitch_err)
            if mag > 0.05:
                v = self._rcs_return_rate
                self._yaw_rate = v * yaw_err / mag
                self._pitch_rate = v * pitch_err / mag
                self._yaw_acc = 0.0
                self._pitch_acc = 0.0
            self._rcs_return_seeded = True
        rate = self._rcs_return_rate
        return self._camera_step(
            yaw_err, pitch_err, dt, 0.0,
            rate_max=rate, gain=self.cfg.rcs_gain,
            pitch_gain=self.cfg.rcs_gain, pitch_rate_max=rate,
            deadband=0.08, wind=True,
        )

    # ------------------------------------------------------------ targets

    def _update_target(
        self, now: float, x: float, y: float, z: float, yaw: float, move_bearing: float | None, speed: float
    ) -> dict | None:
        """
        Какую мишень вести камерой (или None — идём по маршруту).

        Берём всех, кого видно вокруг (LOS по карте или spotted), не только узкий конус
        камеры. Ближние сбоку/сзади важнее дальних в прицеле. Пока текущая видна — держим
        её; на новую переключаемся, только если она заметно опаснее. После исчезновения
        ещё target_hold секунд смотрим в ту же точку. Смотрим на одну мишень не дольше
        target_engage_max, потом возвращаемся к маршруту (cooldown). Дальнюю позади на
        бегу отпускаем; ближнюю вокруг — поворачиваемся.
        """
        cfg = self.cfg
        if not cfg.engage_targets or (cfg.fire_enabled and not self.yaw_verified and self._target is None):
            # без подтверждённого соглашения yaw прицел может быть развёрнут — сначала пройдёмся
            self._target = None
            return None

        def ang_err(e: dict) -> float:
            ex, ey, _ = e["position"]
            return abs(shortest_yaw_err(yaw_to_target(x, y, ex, ey), yaw))

        def dist_xy(e: dict) -> float:
            ex, ey, _ = e["position"]
            return math.hypot(ex - x, ey - y)

        def threat(e: dict) -> float:
            d = dist_xy(e)
            close = 0.55 if d < 380.0 else 1.0
            return d * close + ang_err(e) * 0.7

        def behind(e: dict) -> bool:
            if move_bearing is None:
                return False
            ex, ey, _ = e["position"]
            return abs(shortest_yaw_err(yaw_to_target(x, y, ex, ey), move_bearing)) > cfg.target_behind_deg

        def start(e: dict, fast: bool) -> None:
            self._target = dict(e)
            self._target_seen_at = now
            self._target_last_seen = now
            h = self._hunt
            peeking = h is not None and h.get("index") == e["index"]
            style = h.get("style") if peeking else None
            lo, hi = cfg.target_reaction
            if style == "blind":
                lo, hi = cfg.target_fast_reaction
                self._target_reaction = lo + self._xx_u01() * (hi - lo)
                self._aim_fast_peek = True
            elif style == "preaim":
                self._target_reaction = 0.0
                self._aim_fast_peek = True
            else:
                self._target_reaction = self.rng.uniform(lo, hi) * (0.4 if fast else 1.0)
                self._aim_fast_peek = False
            self._target_engage_until = now + self._target_reaction + self.rng.uniform(*cfg.target_engage_max)
            self._target_behind_since = None
            self._shots_on_target = 0
            self._aimed_since = None
            self._shot_miss = None
            near = ang_err(e) < (22.0 if style == "preaim" else 14.0)
            resume = (
                self._recent_aim_index == e["index"]
                and now - self._recent_aim_drop < 1.2
                and near
            )
            if resume:
                # spotted моргнул: камера уже у мишени — не начинать флик заново (иначе виляние)
                self._target_reaction = 0.0
                self._flick = None
                self._flick_pending = False
                self._aim_handoff_t0 = None
                self._aim_used_flick = False
                self._subflicks = 0
                self._correct_from = now
            elif style == "preaim" and near:
                # уже держим угол — только микродоводка, без флика с маршрута
                self._flick = None
                self._flick_pending = False
                self._aim_handoff_t0 = None
                self._aim_used_flick = False
                self._subflicks = 0
                self._correct_from = now
                self._aim_speed_resample()
            else:
                self._flick = None
                self._flick_pending = True
                self._aim_handoff_t0 = None
                self._aim_used_flick = False
                self._subflicks = 0
                self._aim_speed_resample()
            self._release_fire()
            self._long_spray = None
            if self._habit and self._habit.get("kind") == "unscope_knife":
                self._habit_abort(now, back_to_gun=True)
            self._unscope_left = 0
            if resume and self._scope_plan and self._scope_plan.get("index") == e["index"]:
                pass
            else:
                ex, ey, _ = e["position"]
                self._sample_scope(e["index"], math.hypot(ex - x, ey - y))
            if resume and self._combat_strafe and self._combat_strafe.get("index") == e["index"]:
                pass
            else:
                self._sample_combat_strafe(e["index"])
            if not resume:
                self._sample_aim_bone(e, x, y, z)

        def drop(reason: str, cooldown: float | None = None, *, unscope_knife: bool = True) -> None:
            if self._target is not None:
                self._recent_aim_index = self._target["index"]
                self._recent_aim_drop = now
                self._target_cooldown[self._target["index"]] = now + (cooldown or cfg.target_cooldown)
                if cooldown is not None and cooldown >= cfg.target_done_cooldown:
                    self._given_up[self._target["index"]] = now + cooldown
                if self._shots_on_target:
                    self.targets_shot += 1
                    reason += f", {self._shots_on_target} выстр."
                self.log(f"Мишень отпущена ({reason})")
            self._target = None
            self._target_behind_since = None
            self._aimed_since = None
            self._shot_miss = None
            self._flick = None
            self._flick_pending = False
            self._aim_handoff_t0 = None
            self._aim_used_flick = False
            self._release_fire()
            self._begin_unscope(use_knife=unscope_knife)
            self._scope_plan = None
            self._scope_committed = False
            self._combat_strafe = None
            self._strafe_fire_ok = True
            self._rcs_reset()
            self._long_spray = None
            self._aim_fast_peek = False

        cand = [
            e for e in self.all_enemies
            if dist_xy(e) <= cfg.target_max_dist
            and self._target_cooldown.get(e["index"], 0.0) <= now
            and self._around_visible(e, x, y, z)
        ]
        cur = self._target
        same = next((e for e in cand if e["index"] == cur["index"]), None) if cur is not None else None
        self._target_visible_now = same is not None

        if same is not None:
            best = min(cand, key=threat)
            if best["index"] != same["index"] and threat(best) + cfg.target_switch_margin < threat(same):
                start(best, fast=True)
                self._target_visible_now = True
            else:
                self._target = dict(same)
                self._target_last_seen = now
        elif cand:
            fast = cur is not None and now - self._target_last_seen <= cfg.target_hold
            if cur is not None:
                drop("пропала, есть следующая", unscope_knife=False)
            start(min(cand, key=threat), fast)
            self._target_visible_now = True
        elif cur is not None and now - self._target_last_seen > cfg.target_hold:
            drop("пропала из виду")

        t = self._target
        if t is None:
            return None
        # задержка реакции: мишень уже «в памяти», но взгляд ещё не перенесён
        if now - self._target_seen_at < self._target_reaction:
            return None
        # лимит выстрелов (если задан) — обратно к маршруту
        if (
            cfg.fire_enabled and cfg.shots_per_target > 0
            and self._shots_on_target >= cfg.shots_per_target and now >= self._next_fire_at
        ):
            drop("лимит выстрелов", cfg.target_done_cooldown)
            return None
        # предохранитель: слишком долго на одной мишени (не навелись / она не пропадает)
        if now >= self._target_engage_until:
            if not cfg.fire_enabled:
                drop("достаточно")
            elif self._shots_on_target:
                drop("не пропадает, идём дальше", cfg.target_done_cooldown)
            else:
                drop("не навелись, идём дальше")
            return None
        # дальняя позади на бегу — не бежим спиной; ближнюю вокруг себя разворачиваемся
        if speed > 100.0 and behind(t) and not self.firing and dist_xy(t) > 400.0:
            if self._target_behind_since is None:
                self._target_behind_since = now
            elif now - self._target_behind_since > 0.6:
                drop("осталась позади")
                return None
        else:
            self._target_behind_since = None
        return t

    def _aim_angles(self, x: float, y: float, z: float, target: dict) -> tuple[float, float]:
        """yaw/pitch из глаз на выбранную точку кости (голова или грудь)."""
        tx, ty, tz = self._target_aim_xyz(target, x, y)
        yaw = yaw_to_target(x, y, tx, ty)
        horiz = math.hypot(tx - x, ty - y)
        pitch = -math.degrees(math.atan2(tz - (z + self._eye_z), max(1.0, horiz)))
        return yaw, pitch

    def _sample_aim_bone(self, target: dict, x: float, y: float, z: float) -> None:
        """На захват мишени: выбираем часть тела под прицел с учётом видимости.
        База: голова (с вероятностью (1-chest-shoulder)), грудь, плечо (lev/prav).
        Если движок говорит spotted, но LOS в выбранную кость закрыт — пробуем
        другие части по приоритету head → chest → плечо. Точку на кости
        псевдо-рандомим через xxHash (lat/along/up в своём диапазоне)."""
        cfg = self.cfg
        parts = self._visible_parts(target, x, y, z)
        # Только LOS по кости (не за стеной); off-screen OK. Частичный peek = одно плечо.
        allow: dict[str, bool] = {}
        for part, key in _AIM_PART_KEYS.items():
            allow[part] = bool(target.get(f"{part}_visible")) or parts.get(part) is not None

        base = {
            "head": max(0.0, 1.0 - cfg.aim_chest_chance - cfg.aim_shoulder_chance),
            "chest": cfg.aim_chest_chance,
            "left_shoulder": cfg.aim_shoulder_chance * 0.5,
            "right_shoulder": cfg.aim_shoulder_chance * 0.5,
        }
        weights = {k: (base[k] if allow.get(k) else 0.0) for k in _AIM_PARTS_ORDER}
        total = sum(weights.values())

        if total <= 0.0:
            # ни одна часть не видна и не спотится — целимся в голову вслепую
            pick = "head"
        else:
            r = self._xx_u01() * total
            acc = 0.0
            pick = "head"
            for part in _AIM_PARTS_ORDER:
                w = weights[part]
                if w <= 0.0:
                    continue
                acc += w
                if r <= acc:
                    pick = part
                    break

        self._aim_part = pick
        self._aim_chest = (pick == "chest")
        head = target.get("head_position") or target["position"]
        bone = target.get(_AIM_PART_KEYS[pick])
        self._aim_bone_delta = (
            (bone[0] - head[0], bone[1] - head[1], bone[2] - head[2]) if bone is not None else (0.0, 0.0, 0.0)
        )

        if pick == "chest":
            rlo, rhi = cfg.aim_chest_off_xy
            zlo, zhi = cfg.aim_chest_off_z
        elif pick in ("left_shoulder", "right_shoulder"):
            rlo, rhi = cfg.aim_shoulder_off_xy
            zlo, zhi = cfg.aim_shoulder_off_z
        else:
            rlo, rhi = cfg.aim_head_off_xy
            zlo, zhi = cfg.aim_head_off_z

        lat = (rlo + self._xx_u01() * (rhi - rlo)) * (1.0 if self._xx_u01() < 0.5 else -1.0)
        alo, ahi = cfg.aim_bone_off_along
        along = alo + self._xx_u01() * (ahi - alo)
        up = zlo + self._xx_u01() * (zhi - zlo)
        self._aim_spot = (lat, along, up)
        self.log(f"Точка прицела: {_AIM_PART_RU.get(pick, pick)} "
                 f"({lat:+.1f}/{along:+.1f}/{up:+.1f}u)")

    def _target_aim_xyz(self, target: dict, x: float, y: float) -> tuple[float, float, float]:
        """Мировая точка на выбранной кости + смещение, зафиксированное на мишень.
        Если выбранная кость временно пропала из данных — спускаемся по приоритету
        head → chest → лев./прав. плечо → position (ноги)."""
        key = _AIM_PART_KEYS.get(self._aim_part, "head_position")
        bone = target.get(key)
        if bone is None:
            # кость на этот тик не прочиталась — держим ту же точку через смещение от головы,
            # а не прыгаем на другую часть тела (иначе прицел дёргается между костями)
            head = target.get("head_position") or target["position"]
            d = self._aim_bone_delta
            bone = (head[0] + d[0], head[1] + d[1], head[2] + d[2])
        bx, by, bz = bone
        lat, along, up = self._aim_spot
        dx, dy = bx - x, by - y
        dist = math.hypot(dx, dy) or 1.0
        fx, fy = dx / dist, dy / dist
        lx, ly = -fy, fx
        return bx + lx * lat + fx * along, by + ly * lat + fy * along, bz + up

    def _yaw_to_look_point(self, x: float, y: float, lx: float, ly: float, fallback: float) -> float:
        """Точка взгляда слишком близко → угол скачет от шума позиции; держим прошлый yaw."""
        if math.hypot(lx - x, ly - y) < 36.0:
            return fallback
        return yaw_to_target(x, y, lx, ly)

    def _filter_look_yaw(self, want: float, mode: str, facing: float, now: float) -> float:
        """Точка взгляда догоняет цель со скоростью камеры. На старте — от текущего yaw, без телепорта."""
        cfg = self.cfg
        dt = 1.0 / max(1.0, cfg.tick_hz)
        if self._look_yaw_f is None:
            self._look_mode = mode
            self._look_yaw_f = facing
        elif mode != self._look_mode:
            self._look_mode = mode
            if mode == "aim":
                self._look_yaw_f = want
                return want
        err = shortest_yaw_err(want, self._look_yaw_f)
        if mode == "aim" and abs(err) > 20.0:
            # смена мишени вокруг нас — иначе бот не поворачивает дальше узкого конуса
            self._look_yaw_f = want
            return want
        # как yaw_rate_max навигации, не 16°/тик (это 640°/с — рывок);
        # в прицеливании точка взгляда не должна отставать от руки — иначе прицел «догоняет»
        # уезжающую цель и ёрзает около мишени
        cap = (cfg.aim_lock_rate if mode == "aim" else cfg.yaw_rate_max_moving) * dt
        if self._cam_warm_t0 is not None:
            warm = (now - self._cam_warm_t0) / max(0.05, cfg.cam_warm_s)
            if warm < 1.0:
                cap *= 0.35 + 0.65 * (warm * warm * (3.0 - 2.0 * warm))
        step = max(-cap, min(cap, err))
        self._look_yaw_f = shortest_yaw_err(self._look_yaw_f + step, 0.0)
        return self._look_yaw_f

    def _brake_look_rate(self, speed: float) -> None:
        """После мишени не нести 300°/с навигации — иначе перелёт и виляние вокруг маршрута."""
        cap = self._nav_yaw_cap(speed)
        self._yaw_rate = max(-cap, min(cap, self._yaw_rate))
        self._pitch_rate = max(-cap * 0.4, min(cap * 0.4, self._pitch_rate))
        self._wind[0] = 0.0
        self._wind[1] = 0.0

    def _avoid_targets(self, x: float, y: float, des_x: float, des_y: float) -> tuple[float, float]:
        """
        Мишень — твёрдый цилиндр на полу. Если она рядом и по ходу движения, добавляем к
        желаемой скорости боковой «отталкивающий» вектор, чтобы обойти, а не упереться.
        """
        cfg = self.cfg
        if not self.known_targets:
            return des_x, des_y
        vmax = cfg.max_speed
        ax, ay = des_x, des_y
        hull = self.map.cfg.hull_radius
        contact = hull + cfg.target_body_radius            # дистанция касания тел
        for e in self.known_targets:
            if not e.get("visible"):
                continue
            ex, ey, _ = e["position"]
            dx, dy = ex - x, ey - y
            d = math.hypot(dx, dy)
            if d < 1e-6 or d > cfg.target_avoid_dist + contact:
                continue
            # мишень «спереди» относительно желаемого движения?
            dm = math.hypot(des_x, des_y)
            ahead = (dx * des_x + dy * des_y) / (d * dm) if dm > 1e-6 else 1.0
            if ahead < -0.2:
                continue
            # сила: 0 на границе зоны → полная у касания
            k = 1.0 - max(0.0, d - contact) / cfg.target_avoid_dist
            # обходим по касательной (вбок от мишени), чуть отталкиваясь — но не тормозим «в лоб»
            nx, ny = -dx / d, -dy / d
            side = 1.0 if (des_x * dy - des_y * dx) < 0 else -1.0   # с какой стороны маршрут проходит мишень
            tx_, ty_ = -ny * side, nx * side
            w = vmax * k
            ax += w * (0.6 * nx + 0.8 * tx_)
            ay += w * (0.6 * ny + 0.8 * ty_)
        m = math.hypot(ax, ay)
        if m > vmax:
            ax, ay = ax / m * vmax, ay / m * vmax
        return ax, ay

    def log(self, msg: str) -> None:
        self.events.append(msg)
        if self.cfg.verbose_log:
            print(msg)

    def _set_keys(self, keys: set[str]) -> None:
        for k in list(self._keys - keys):
            self.input.key_up(k)
            self._keys.discard(k)
        for k in keys - self._keys:
            self.input.key_down(k)
            self._keys.add(k)

    def release(self) -> None:
        self._release_fire()
        self._unscope_left = 0
        self._release_scope()
        now = self._now()
        if self._habit:
            self._habit_abort(now, back_to_gun=True)
        self._set_keys(self._merge_extra_keys(set(), now))

    def _jump(self) -> None:
        self._last_jump_at = self._now()
        self.input.key_down("space")
        self.input.key_up("space")

    def _do_climb(self, kind: str, now: float) -> None:
        """Прыжок или прыжок+присед. Выше crouch_jump не прыгаем — обход."""
        if kind == "crouch_jump":
            self._crouch_until = max(self._crouch_until, now + 0.48)
        self._jump()

    def _climb_kind(self, dz: float) -> str | None:
        step = self.map.cfg.step_height
        jmp = self.map.cfg.jump_height
        cjmp = self.map.cfg.crouch_jump_height
        if dz <= step + 2.0:
            return None
        if dz <= jmp + 1.0:
            return "jump"
        if dz <= cjmp + 2.0:
            return "crouch_jump"
        return None

    def _climb_needed(
        self, x: float, y: float, z: float, cx: float, cy: float, cz: float | None = None
    ) -> str | None:
        """
        Что нужно, чтобы забраться на клетку морковки:
        jump — обычный прыжок (≤57u), crouch_jump — с приседом (≤64u),
        None — пандус / стена выше прыжка (тогда обходим).
        """
        step = self.map.cfg.step_height
        cjmp = self.map.cfg.crouch_jump_height
        cell = max(4.0, self.map.cfg.cell)
        here = self.map.layer_at(x, y, z, cjmp)
        if here is None:
            return None
        dx, dy = cx - x, cy - y
        dist = math.hypot(dx, dy)
        if cz is not None and dist < 48.0:
            kind = self._climb_kind(cz - here.z)
            if kind is not None:
                return kind
        above = self.map.higher_layer(x, y, here.z, cjmp + 2.0)
        if above is not None:
            kind = self._climb_kind(above.z - here.z)
            if kind is not None:
                return kind
        if dist < 8.0:
            land = self.map.higher_layer(cx, cy, here.z, cjmp + 2.0)
            if land is None:
                return None
            return self._climb_kind(land.z - here.z)
        steps = max(1, int(dist / cell))
        zcur = here.z
        need: str | None = None
        for i in range(1, steps + 1):
            t = i / steps
            px, py = x + dx * t, y + dy * t
            walk = self.map.layer_at(px, py, zcur, step + 2.0)
            if walk is not None:
                zcur = walk.z
                continue
            land = self.map.layer_at(px, py, zcur, cjmp + 2.0)
            if land is None:
                return None
            kind = self._climb_kind(land.z - zcur)
            if kind == "crouch_jump":
                need = kind
            elif kind == "jump" and need is None:
                need = kind
            zcur = land.z
        return need

    # ------------------------------------------------------------ planning

    def _door_xyz(self, door: dict) -> tuple[float, float, float] | None:
        pos = door.get("position")
        if not isinstance(pos, dict):
            return None
        try:
            return float(pos["x"]), float(pos["y"]), float(pos["z"])
        except (KeyError, TypeError, ValueError):
            return None

    def _door_blocks_route(self, door: dict) -> bool:
        """Запертую или неиспользуемую дверь обходим. Закрытую, но открываемую — проходим."""
        if door.get("locked") or door.get("usable") is False:
            return True
        return False

    def _door_needs_open(self, door: dict) -> bool:
        if self._door_blocks_route(door) or door.get("passable"):
            return False
        state = str(door.get("state") or "")
        return state not in ("open", "ajar", "opening")

    def _closed_door_ahead(self, route: Route, x: float, y: float, z: float) -> dict | None:
        """Закрытая дверь на маршруте впереди, ближайшая к боту."""
        cfg = self.cfg
        best = None
        best_d = cfg.door_use_approach + 1.0
        end = min(route.length, self.s + cfg.door_use_approach + 24.0)
        for door in self.doors:
            if not self._door_needs_open(door):
                continue
            pt = self._door_xyz(door)
            if pt is None or abs(pt[2] - z) > 96.0:
                continue
            rad = max(float(door.get("width") or 40.0) * 0.5, 28.0) + 16.0
            hit = False
            d = self.s
            while d <= end:
                px, py, pz = route.point_at(d)
                if abs(pz - pt[2]) <= 80.0 and math.hypot(px - pt[0], py - pt[1]) <= rad:
                    hit = True
                    break
                d += 10.0
            if not hit:
                continue
            dist = math.hypot(pt[0] - x, pt[1] - y)
            if dist < best_d:
                best, best_d = door, dist
        return best

    def _door_look(self, x: float, y: float, z: float) -> tuple[float, float] | None:
        door = self._door_ahead
        if door is None or self.engaged:
            return None
        pt = self._door_xyz(door)
        if pt is None:
            return None
        yaw = yaw_to_target(x, y, pt[0], pt[1])
        horiz = math.hypot(pt[0] - x, pt[1] - y)
        pitch = -math.degrees(math.atan2((pt[2] + 36.0) - (z + self._eye_z), max(1.0, horiz)))
        return yaw, max(-25.0, min(30.0, pitch))

    def _door_use(self, now: float, x: float, y: float) -> None:
        """Короткое E на ходу. Клавиши движения не трогаем."""
        door = self._door_ahead
        if door is None or self.firing:
            return
        pt = self._door_xyz(door)
        if pt is None:
            return
        if math.hypot(pt[0] - x, pt[1] - y) > self.cfg.door_use_dist:
            return
        idx = int(door.get("index") or 0)
        if now < self._door_use_at.get(idx, 0.0):
            return
        lo, hi = self.cfg.door_use_hold
        self._pulse_key("e", now, lo + self._xx_u01() * (hi - lo))
        self._door_use_at[idx] = now + self.cfg.door_use_retry
        self.log(f"Дверь закрыта — открываю на ходу ({door.get('state') or 'closed'})")

    def _obstacle_circles(self) -> list[Obstacle]:
        """Круглые препятствия: видимые мишени."""
        r = self.cfg.target_body_radius + 16.0 * self._target_bumps
        return [
            (*self._enemy_feet(e), r)
            for e in self.known_targets
            if e.get("visible")
        ]

    def _obstacle_boxes(self) -> list[BoxObstacle]:
        """Прямоугольные препятствия: только запертые двери. Решётку на пути простреливаем."""
        boxes: list[BoxObstacle] = []
        for door in self.doors:
            if not self._door_blocks_route(door):
                continue
            box = self._object_aabb(door, self.cfg.door_block_margin)
            if box:
                boxes.append(box)
        return boxes

    def _breakable_shootable(self, br: dict) -> bool:
        return not br.get("passable") and not br.get("broken")

    def _breakable_ahead(self, route: Route, x: float, y: float, z: float) -> dict | None:
        """Целая решётка, через которую проходит маршрут впереди."""
        cfg = self.cfg
        best = None
        best_d = cfg.breakable_engage_dist + 1.0
        end = min(route.length, self.s + cfg.breakable_engage_dist)
        for br in self.breakables:
            if not self._breakable_shootable(br):
                continue
            box = self._object_aabb(br, 8.0)
            if box is None:
                continue
            hit = False
            d = self.s
            while d <= end:
                px, py, pz = route.point_at(d)
                if (
                    box[0] <= px <= box[3]
                    and box[1] <= py <= box[4]
                    and box[2] - 48.0 <= pz <= box[5] + 48.0
                ):
                    hit = True
                    break
                d += 12.0
            if not hit:
                continue
            cx = (box[0] + box[3]) * 0.5
            cy = (box[1] + box[4]) * 0.5
            dist = math.hypot(cx - x, cy - y)
            if dist < best_d:
                best, best_d = br, dist
        return best

    def _ray_on_box(
        self, origin: XYZ, direction: tuple[float, float, float], box: BoxObstacle
    ) -> XYZ:
        ox, oy, oz = origin
        dx, dy, dz = direction
        tmin, tmax = 0.0, 900.0
        hit = True
        for o, d, lo, hi in (
            (ox, dx, box[0], box[3]),
            (oy, dy, box[1], box[4]),
            (oz, dz, box[2], box[5]),
        ):
            if abs(d) < 1e-8:
                if o < lo or o > hi:
                    hit = False
                    break
                continue
            t1, t2 = (lo - o) / d, (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                hit = False
                break
        if hit and tmin >= 0.0:
            return ox + dx * tmin, oy + dy * tmin, oz + dz * tmin
        cx = (box[0] + box[3]) * 0.5
        cy = (box[1] + box[4]) * 0.5
        cz = (box[2] + box[5]) * 0.5
        t = max(0.0, (cx - ox) * dx + (cy - oy) * dy + (cz - oz) * dz)
        px, py, pz = ox + dx * t, oy + dy * t, oz + dz * t
        return (
            min(box[3], max(box[0], px)),
            min(box[4], max(box[1], py)),
            min(box[5], max(box[2], pz)),
        )

    def _sample_break_point(
        self, box: BoxObstacle, x: float, y: float, z: float, yaw: float, pitch: float
    ) -> XYZ:
        """Точка на модели рядом с текущим прицелом. Смещение — xxHash, внутри габарита."""
        eye = (x, y, z + self._eye_z)
        fwd = forward_from_angles(pitch, yaw)
        aim = self._ray_on_box(eye, (fwd.x, fwd.y, fwd.z), box)
        dist = math.hypot(aim[0] - eye[0], aim[1] - eye[1], aim[2] - eye[2])
        lo, hi = self.cfg.breakable_aim_deg
        ang = lo + self._xx_u01() * (hi - lo)
        cap = max(2.0, dist * math.tan(math.radians(ang)))
        off = [(self._xx_u01() * 2.0 - 1.0) * cap for _ in range(3)]
        ln = math.hypot(*off)
        if ln > cap and ln > 1e-6:
            off = [v * cap / ln for v in off]
        inset = 3.0
        spans = (box[3] - box[0], box[4] - box[1], box[5] - box[2])
        pads = tuple(min(inset, s * 0.2) if s > inset * 2.0 else 0.0 for s in spans)
        return (
            min(box[3] - pads[0], max(box[0] + pads[0], aim[0] + off[0])),
            min(box[4] - pads[1], max(box[1] + pads[1], aim[1] + off[1])),
            min(box[5] - pads[2], max(box[2] + pads[2], aim[2] + off[2])),
        )

    def _angles_to(self, x: float, y: float, z: float, tx: float, ty: float, tz: float) -> tuple[float, float]:
        yaw = yaw_to_target(x, y, tx, ty)
        horiz = math.hypot(tx - x, ty - y)
        pitch = -math.degrees(math.atan2(tz - (z + self._eye_z), max(1.0, horiz)))
        return yaw, pitch

    def _update_break_aim(
        self, route: Route, x: float, y: float, z: float, yaw: float, pitch: float, now: float
    ) -> dict | None:
        """Решётка на пути: одна точка на старте наводки, дальше та же рука, что на мишень."""
        br = self._breakable_ahead(route, x, y, z)
        if br is None:
            self._break_aim = None
            return None
        idx = int(br.get("index") or 0)
        box = self._object_aabb(br, 0.0)
        if box is None:
            self._break_aim = None
            return None
        if self._break_aim is None or self._break_aim.get("index") != idx:
            point = self._sample_break_point(box, x, y, z, yaw, pitch)
            self._flick = None
            self._flick_pending = True
            self._aim_handoff_t0 = None
            self._aim_used_flick = False
            self._aim_fast_peek = False
            self._subflicks = 0
            self._aim_speed_resample()
            self._reset_aim_bone()
            self._shots_on_target = 0
            self._aimed_since = None
            self._shot_miss = False
            self._long_spray = None
            self._combat_strafe = None
            self._strafe_fire_ok = True
            self._release_fire()
            self._sample_scope(idx)
            self._break_aim = {"index": idx, "point": point, "t0": now}
            name = str(br.get("name") or br.get("kind") or "решётка")
            self.log(
                f"Решётка на пути — наводка ({point[0]:.0f},{point[1]:.0f},{point[2]:.0f}) {name}"
            )
        pt = self._break_aim["point"]
        return {
            "index": idx,
            "name": str(br.get("name") or "решётка"),
            "position": pt,
            "head_position": pt,
            "vulnerable": True,
            "visible": True,
            "velocity": (0.0, 0.0, 0.0),
        }

    def _blocked_nodes(self) -> set[int]:
        blocked = blocked_by_obstacles(self.map, self._obstacle_circles())
        blocked |= blocked_by_box_obstacles(self.map, self._obstacle_boxes())
        return blocked

    def _blockers_key(self) -> frozenset:
        return frozenset(
            (e["index"], e["position"])
            for e in self.known_targets
            if e.get("visible")
        )

    def plan(self, pos: XYZ, goal_id: int | None = None) -> Route:
        blocked = self._blocked_nodes()
        picking_new = goal_id is None
        if picking_new:
            given_up_now = {e["index"] for e in self.known_targets if self._given_up.get(e["index"], 0.0) > self._now()}
            use_blocked = blocked if given_up_now else set()
            if self.cfg.random_goals:
                excl = {self.goal_id} if self.goal_id is not None else set()
                goal_id = pick_random_goal(self.map, pos, self.cfg.route, use_blocked, self.rng, excl)
            else:
                goal_id = pick_farthest_goal(self.map, pos, self.cfg.route, use_blocked)
            self._intended_goal_id = goal_id
        else:
            if self._intended_goal_id is None:
                self._intended_goal_id = goal_id
            goal_id = self._intended_goal_id
        self._standoff = None
        self._standoff_since = None

        def try_route(blk: set[int]) -> list[XYZ] | None:
            try:
                return plan_route(self.map, pos, goal_id, self.cfg.route, blk)
            except RuntimeError:
                return None

        pts_clear = try_route(set())
        pts_blocked = try_route(blocked) if blocked else pts_clear
        standoff = self._standoff_route(pos, goal_id) if blocked else None

        extra = ""
        if blocked and self._prefer_standoff(pos, pts_clear, pts_blocked, standoff):
            pts, blocker = standoff  # type: ignore[misc]
            self._standoff = blocker["index"]
            extra = f" → стоим перед {blocker.get('name') or blocker['index']}"
            self.log(
                f"Проход перекрыт мишенью {blocker.get('name') or blocker['index']} "
                f"→ подходим на дистанцию выстрела, цель та же"
            )
        elif pts_blocked is not None:
            if (
                pts_clear is not None
                and blocked
                and self._hidden_blocks_reverse(pos, pts_clear, pts_blocked)
            ):
                pts = pts_clear
                extra = " → к скрытой по прямому пути"
            else:
                pts = pts_blocked
        elif standoff is not None:
            pts, blocker = standoff
            self._standoff = blocker["index"]
            extra = f" → стоим перед {blocker.get('name') or blocker['index']}"
            self.log(
                f"Проход перекрыт мишенью {blocker.get('name') or blocker['index']} "
                f"→ подходим на дистанцию выстрела, цель та же"
            )
        elif pts_clear is not None:
            pts = pts_clear
        else:
            alt = (
                pick_random_goal(self.map, pos, self.cfg.route, blocked, self.rng)
                if self.cfg.random_goals
                else pick_farthest_goal(self.map, pos, self.cfg.route, blocked)
            )
            pts = plan_route(self.map, pos, alt, self.cfg.route, blocked)
            self.log("Цель недостижима → другая точка")
            self._intended_goal_id = alt
            goal_id = alt
        self.goal_id = goal_id
        self.route = Route(pts)
        self.s = 0.0
        self._stuck_level = 0
        self._progress_s = 0.0
        self._progress_at = self._now()
        self._face_turn_since = None
        self._route_blockers = self._blockers_key()
        self._blocked = blocked
        g = self.map.nodes[goal_id]
        self.log(
            f"Маршрут: {len(pts)} вершин, {self.route.length:.0f}u → цель "
            f"({g.x:.0f},{g.y:.0f},{g.z:.0f}) clearance={g.clearance:.0f}"
            + (f", обход мишеней: {len(blocked)} клеток" if blocked else "")
            +             extra
        )
        if self.cfg.loop:
            self._kick_prefetch()
        return self.route

    def _path_bearing(self, pos: XYZ, pts: list[XYZ], ahead: float = 80.0) -> float | None:
        route = Route(pts)
        if route.length < 8.0:
            return None
        px, py, _ = route.point_at(min(route.length, ahead))
        if math.hypot(px - pos[0], py - pos[1]) < 4.0:
            return None
        return yaw_to_target(pos[0], pos[1], px, py)

    def _prefer_standoff(
        self,
        pos: XYZ,
        pts_clear: list[XYZ] | None,
        pts_blocked: list[XYZ] | None,
        standoff: tuple[list[XYZ], dict] | None,
    ) -> bool:
        """Обход, который разворачивает назад / сильно длиннее прямой — не берём: стреляем с места."""
        if standoff is None:
            return False
        if pts_blocked is None or pts_clear is None:
            return True
        clear_len = Route(pts_clear).length
        block_len = Route(pts_blocked).length
        if block_len > clear_len * 1.25 + 80.0:
            return True
        bc = self._path_bearing(pos, pts_clear)
        bb = self._path_bearing(pos, pts_blocked)
        if bc is not None and bb is not None and abs(shortest_yaw_err(bb, bc)) > 90.0:
            return True
        return False

    def _hidden_blocks_reverse(
        self, pos: XYZ, pts_clear: list[XYZ], pts_blocked: list[XYZ]
    ) -> bool:
        """Скрытая мишень на прямом пути, а обход разворачивает назад — идём к углу, не к старту."""
        bc = self._path_bearing(pos, pts_clear)
        bb = self._path_bearing(pos, pts_blocked)
        reverse = (
            bc is not None
            and bb is not None
            and abs(shortest_yaw_err(bb, bc)) > 90.0
        ) or Route(pts_blocked).length > Route(pts_clear).length * 1.25 + 80.0
        if not reverse:
            return False
        now = self._now()
        route = Route(pts_clear)
        for e in self._hidden_enemies(now):
            fx, fy, _ = self._enemy_feet(e)
            d = 0.0
            while d <= min(route.length, 480.0):
                px, py, _ = route.point_at(d)
                if math.hypot(fx - px, fy - py) < 90.0:
                    return True
                d += 16.0
        return False

    def _standoff_route(self, pos: XYZ, goal_id: int) -> tuple[list[XYZ], dict] | None:
        """Маршрут к цели без учёта мишеней, обрезанный за standoff_dist до первой мишени на нём.
        None — если стрельба выключена, по мишени уже отстрелялись (не уходит) или обрезать нечего."""
        cfg = self.cfg
        if not cfg.fire_enabled or not self.known_targets:
            return None
        now = self._now()
        live = [
            e for e in self.known_targets
            if self._given_up.get(e["index"], 0.0) <= now and e.get("visible")
        ]
        if not live:
            return None
        try:
            pts = plan_route(self.map, pos, goal_id, cfg.route, set())
        except RuntimeError:
            return None
        route = Route(pts)
        # первая точка маршрута, где до какой-то мишени ближе standoff_dist
        d = 0.0
        while d <= route.length:
            px, py, _ = route.point_at(d)
            for e in live:
                if math.hypot(e["position"][0] - px, e["position"][1] - py) <= cfg.standoff_dist:
                    if d < 24.0:
                        # уже на дистанции — короткий маршрут «на месте», сразу переходим в ожидание
                        return [pts[0], route.point_at(min(route.length, 16.0))], e
                    cut = route.point_at(d)
                    out: list[XYZ] = [pts[0]]
                    acc = 0.0
                    for a, b in zip(pts, pts[1:]):
                        seg = math.hypot(b[0] - a[0], b[1] - a[1])
                        if acc + seg >= d:
                            break
                        acc += seg
                        out.append(b)
                    out.append(cut)
                    return out, e
            d += 8.0
        return None

    def _replan(self, pos: XYZ, reason: str, new_goal: bool = False) -> None:
        self.replans += 1
        self.log(f"Перестроение маршрута ({reason})")
        try:
            if new_goal:
                self._intended_goal_id = None
            self.plan(pos, None if new_goal else (self._intended_goal_id or self.goal_id))
        except RuntimeError as exc:
            self.log(f"  не удалось: {exc}")

    def _life_events(self) -> tuple[bool, bool]:
        ev = self.life.get("events") if isinstance(self.life, dict) else None
        if not isinstance(ev, dict):
            ev = {}
        return bool(ev.get("died")), bool(ev.get("respawned"))

    def _on_death(self) -> None:
        self._dead = True
        self._route_gen += 1
        self._next_route = None
        self._next_goal_id = None
        self._prefetch_busy = False
        self.route = None
        self.goal_id = None
        self._intended_goal_id = None
        self._hunt = None
        self._hunt_routed = None
        self._standoff = None
        self._standoff_since = None
        self._target = None
        self.engaged = False
        self.firing = False
        self._immune_seen.clear()
        self._vuln_ready_at.clear()
        self._flick = None
        self._flick_pending = False
        self._habit = None
        self._brake = False
        self._pause_until = 0.0
        self._ladder = None
        self._crouch_until = 0.0
        self._door_use_at.clear()
        self._door_ahead = None
        self._break_aim = None
        self._rcs_return = False
        self.done = False
        self.release()
        self.log("Умер — клавиши сброшены, ждём респавн")

    def _on_respawn(self, pos: XYZ) -> None:
        """Карта та же; новый случайный маршрут от точки появления."""
        self._dead = False
        self._route_gen += 1
        self._next_route = None
        self._next_goal_id = None
        self._prefetch_busy = False
        self.done = False
        self._pause_until = 0.0
        self._cam_warm_t0 = None
        self._look_yaw_f = None
        self._brake = False
        self._stuck_level = 0
        self._target_bumps = 0
        self._hunt = None
        self._hunt_routed = None
        self._standoff = None
        self._target = None
        self.engaged = False
        self._hpshot_thr = None
        self._ladder = None
        self._crouch_until = 0.0
        try:
            self.plan(pos)
            self.log("Респавн — новый случайный маршрут")
        except RuntimeError as exc:
            self.log(f"Респавн: маршрут не построен ({exc})")

    def _kick_prefetch(self) -> None:
        """Пока идём — в фоне A* до следующей случайной точки (карта не пересобирается)."""
        cfg = self.cfg
        if not cfg.loop or self._dead or self._prefetch_busy or self._next_route is not None:
            return
        gid = self._intended_goal_id if self._intended_goal_id is not None else self.goal_id
        if gid is None or gid >= len(self.map.nodes):
            return
        g = self.map.nodes[gid]
        from_pos = (g.x, g.y, g.z)
        gen = self._route_gen
        seed = int(self._xx_u01() * 4294967296.0) & 0xFFFFFFFF
        exclude = {gid}
        cmap = self.map
        rcfg = self.cfg.route
        random_goals = cfg.random_goals
        self._prefetch_busy = True

        def work() -> None:
            try:
                rng = random.Random(seed)
                blocked: set[int] = set()
                if random_goals:
                    nxt = pick_random_goal(cmap, from_pos, rcfg, blocked, rng, exclude)
                else:
                    nxt = pick_farthest_goal(cmap, from_pos, rcfg, blocked)
                pts = plan_route(cmap, from_pos, nxt, rcfg, blocked)
                with self._prefetch_lock:
                    if gen != self._route_gen:
                        return
                    self._next_goal_id = nxt
                    self._next_route = Route(pts)
            except Exception:
                pass
            finally:
                with self._prefetch_lock:
                    if gen == self._route_gen:
                        self._prefetch_busy = False

        threading.Thread(target=work, name="nav-prefetch", daemon=True).start()

    def _apply_next_route(self, pos: XYZ) -> bool:
        """Подставить заранее собранный маршрут; если нет — построить сейчас."""
        nxt = None
        nid = None
        with self._prefetch_lock:
            nxt, nid = self._next_route, self._next_goal_id
            self._next_route = None
            self._next_goal_id = None
        if nxt is not None and nid is not None:
            self.route = nxt
            self.goal_id = nid
            self._intended_goal_id = nid
            self.s = 0.0
            self._stuck_level = 0
            self._brake = False
            self._route_blockers = self._blockers_key()
            self.s, _off = nxt.project(pos[0], pos[1], None, z=pos[2])
            g = self.map.nodes[nid]
            self.log(
                f"Следующая точка ({g.x:.0f},{g.y:.0f},{g.z:.0f}), "
                f"маршрут {nxt.length:.0f}u уже готов"
            )
            self._kick_prefetch()
            return True
        try:
            self.plan(pos)
            return self.route is not None
        except RuntimeError as exc:
            self.log(f"Следующий маршрут: {exc}")
            return False

    def _target_on_route(self, route: Route, ahead: float = 320.0) -> dict | None:
        """Мишень, стоящая на маршруте впереди (ближе hull+radius к линии маршрута)."""
        if not self.known_targets:
            return None
        reach = self.map.cfg.hull_radius + self.cfg.target_body_radius + 4.0
        end = min(route.length, self.s + ahead)
        d = self.s
        while d <= end:
            px, py, pz = route.point_at(d)
            for e in self.known_targets:
                ex, ey, ez = self._enemy_feet(e)
                if math.hypot(ex - px, ey - py) <= reach and abs(ez - pz) <= 72.0:
                    return e
            d += 8.0
        return None

    def _near_target(self, x: float, y: float, margin: float = 12.0) -> bool:
        reach = self.map.cfg.hull_radius + self.cfg.target_body_radius + margin
        return any(math.hypot(e["position"][0] - x, e["position"][1] - y) <= reach for e in self.known_targets)

    def _los_clear(self, x: float, y: float, z: float, tx: float, ty: float, tz: float) -> bool:
        """Прямая до цели не через стену (2D навмеш + Z). z — ноги локального."""
        return los_clear(
            self.map, x, y, z, tx, ty, tz,
            eye_z=self._eye_z,
            hull_radius=self.map.cfg.hull_radius,
            target_body_radius=self.cfg.target_body_radius,
        )

    def _hidden_enemies(self, now: float) -> list[dict]:
        """Были видимы, спрятались — не тех, кого никогда не видели (за стеной с рождения)."""
        return [
            e for e in self.known_targets
            if not e.get("visible")
            and not e.get("dormant")
            and e.get("last_visible", 0.0) > 0.0
            and self._given_up.get(e["index"], 0.0) <= now
        ]

    def _reveal_along_route(self, route: Route, enemy: dict) -> float | None:
        """Первая точка маршрута впереди, откуда мишень видна по карте (выход из-за угла)."""
        fx, fy, fz = self._enemy_feet(enemy)
        cfg = self.cfg
        d = max(0.0, self.s)
        end = min(route.length, d + 900.0)
        while d <= end:
            px, py, pz = route.point_at(d)
            dist = math.hypot(fx - px, fy - py)
            if dist <= cfg.hunt_peek_max * 1.4 and self._los_clear(px, py, pz, fx, fy, fz):
                return d
            d += 16.0
        return None

    def _find_peek_node(self, pos: XYZ, enemy: dict):
        """Ближайшая удобная клетка с LOS на скрытую мишень (кольцо вокруг неё)."""
        fx, fy, fz = self._enemy_feet(enemy)
        cfg = self.cfg
        best = None
        best_score = float("inf")
        gx = gy = None
        if self._intended_goal_id is not None:
            g = self.map.nodes[self._intended_goal_id]
            gx, gy = g.x, g.y
        to_goal = yaw_to_target(pos[0], pos[1], gx, gy) if gx is not None else None
        for dist in (90.0, 120.0, 160.0, 200.0):
            if dist < cfg.hunt_peek_min or dist > cfg.hunt_peek_max:
                continue
            for i in range(16):
                ang = math.radians(i * 22.5)
                x = fx + dist * math.cos(ang)
                y = fy + dist * math.sin(ang)
                node = self.map.layer_at(x, y, fz, dz_max=self.map.cfg.jump_height)
                if node is None or node.clearance < 14.0:
                    continue
                if not self._los_clear(node.x, node.y, node.z, fx, fy, fz):
                    continue
                go = math.hypot(node.x - pos[0], node.y - pos[1])
                extra = 0.0
                if gx is not None:
                    extra = go + math.hypot(gx - node.x, gy - node.y) - math.hypot(gx - pos[0], gy - pos[1])
                if extra > cfg.hunt_max_detour:
                    continue
                if to_goal is not None and go > 48.0:
                    bear = yaw_to_target(pos[0], pos[1], node.x, node.y)
                    if abs(shortest_yaw_err(bear, to_goal)) > 110.0:
                        continue
                # удобный пик: выходим не в лоб, а под углом ~40°, чтобы сразу видеть мишень
                approach = yaw_to_target(pos[0], pos[1], node.x, node.y)
                face = yaw_to_target(node.x, node.y, fx, fy)
                swing = abs(shortest_yaw_err(face, approach))
                score = go + max(0.0, extra) * 0.5 + abs(swing - 40.0) * 0.9
                if score < best_score:
                    best_score, best = score, node
        return best

    def _sample_peek_style(self) -> dict:
        """60% пик с прицелом за угол; 40% выход как сейчас, но реакция короче."""
        if self._xx_u01() < self.cfg.peek_blind_chance:
            return {"style": "blind", "move": "route"}
        move = "arc" if self._xx_u01() < 0.45 else "side"
        lo, hi = self.cfg.peek_overshoot
        return {"style": "preaim", "move": move, "overshoot": lo + self._xx_u01() * (hi - lo)}

    def _carry_peek_style(self, prev: dict | None, index: int) -> dict:
        if prev and prev.get("index") == index and prev.get("style"):
            return {
                "style": prev["style"],
                "move": prev.get("move", "side"),
                "overshoot": float(prev.get("overshoot") or 48.0),
            }
        return self._sample_peek_style()

    def _peek_look_angles(
        self, x: float, y: float, z: float, enemy: dict, peek: tuple
    ) -> tuple[float, float]:
        """Прицел чуть дальше угла, в проём — не в стену и не сквозь неё в голову."""
        fx, fy, fz = self._enemy_feet(enemy)
        px, py = peek[0], peek[1]
        dx, dy = fx - px, fy - py
        n = math.hypot(dx, dy)
        over = 48.0
        if self._hunt:
            over = float(self._hunt.get("overshoot") or over)
        if n < 1.0:
            return self._aim_angles(x, y, z, enemy)
        lx = px + dx / n * over
        ly = py + dy / n * over
        yaw = yaw_to_target(x, y, lx, ly)
        _, ep = self._aim_angles(x, y, z, enemy)
        pitch = 0.55 * self.cfg.pitch_base + 0.45 * ep
        return yaw, max(-25.0, min(30.0, pitch))

    def _peek_move_keys(self, x: float, y: float, look_yaw: float) -> set[str] | None:
        """Угол: страйф боком или широкая дуга W+A/D, пока не пошли в прицел."""
        h = self._hunt
        if h is None or h.get("style") != "preaim":
            return None
        if self._target is not None and (self._now() - self._target_seen_at) >= self._target_reaction:
            return None
        peek = h.get("peek")
        if not peek:
            return None
        d = math.hypot(peek[0] - x, peek[1] - y)
        if d > self.cfg.peek_commit_dist:
            return None
        yr = math.radians(look_yaw)
        lx, ly = -math.sin(yr), math.cos(yr)
        to_x, to_y = peek[0] - x, peek[1] - y
        if math.hypot(to_x, to_y) < 18.0:
            e = next((t for t in self.known_targets if t["index"] == h["index"]), None)
            if e is not None:
                fx, fy, _ = self._enemy_feet(e)
                to_x, to_y = fx - x, fy - y
        side = {"a"} if (lx * to_x + ly * to_y) > 0.0 else {"d"}
        if h.get("move") == "arc":
            return side | {"w"}
        return side

    def _update_hunt(self, pos: XYZ, route: Route, now: float) -> None:
        """Куда выйти, чтобы увидеть скрытую мишень, не целясь в стену и не бросая маршрут."""
        cfg = self.cfg
        if not cfg.hunt_hidden:
            self._hunt = None
            return
        prev = self._hunt
        hidden = []
        for e in self._hidden_enemies(now):
            fx, fy, _ = self._enemy_feet(e)
            d = math.hypot(fx - pos[0], fy - pos[1])
            if d <= cfg.hunt_max_dist:
                hidden.append((d, e))
        if not hidden:
            self._hunt = None
            self._hunt_routed = None
            return
        hidden.sort(key=lambda t: t[0])
        e = hidden[0][1]
        style = self._carry_peek_style(prev, e["index"])
        reveal_s = self._reveal_along_route(route, e)
        if reveal_s is not None:
            self._hunt = {
                "index": e["index"],
                "reveal_s": reveal_s,
                "peek": route.point_at(reveal_s),
                "via": False,
                **style,
            }
            if prev is None or prev.get("index") != e["index"] or prev.get("via"):
                how = "пик, прицел за угол" if style["style"] == "preaim" else "выход вслепую"
                self.log(
                    f"Мишень {e.get('name') or e['index']} за углом — "
                    f"{how}, маршрут тот же"
                )
            return
        if (
            prev
            and prev.get("via")
            and prev.get("index") == e["index"]
            and self._hunt_routed is not None
        ):
            self._hunt = prev
            return
        peek = self._find_peek_node(pos, e)
        if peek is None:
            self._hunt = None
            return
        self._hunt = {
            "index": e["index"],
            "peek": (peek.x, peek.y, peek.z),
            "peek_id": peek.id,
            "via": True,
            **style,
        }
        if prev is None or prev.get("index") != e["index"] or not prev.get("via"):
            how = "пик, прицел за угол" if style["style"] == "preaim" else "выход вслепую"
            self.log(f"Мишень {e.get('name') or e['index']} за углом — {how}, заход сбоку")

    def _hunt_look(
        self, x: float, y: float, z: float, route: Route
    ) -> tuple[float, float] | None:
        """Куда смотреть, подходя к скрытой мишени: в проём, а сквозь стену — нет."""
        hunt = self._hunt
        if hunt is None or hunt.get("style") == "blind":
            return None
        e = next((t for t in self.known_targets if t["index"] == hunt["index"]), None)
        if e is None:
            return None
        reacting = (
            e.get("visible")
            and self._target is not None
            and self._target.get("index") == hunt["index"]
            and (self._now() - self._target_seen_at) < max(self._target_reaction, 0.04)
        )
        if e.get("visible") and not reacting:
            return None
        fx, fy, fz = self._enemy_feet(e)
        if self._los_clear(x, y, z, fx, fy, fz):
            return self._aim_angles(x, y, z, e)
        if reacting:
            return self._aim_angles(x, y, z, e)
        peek = hunt.get("peek")
        if peek is None:
            return None
        pre_d = self.cfg.peek_preaim_dist if hunt.get("style") == "preaim" else self.cfg.hunt_preaim_dist
        if hunt.get("via"):
            if math.hypot(peek[0] - x, peek[1] - y) > pre_d:
                return None
        else:
            reveal_s = hunt.get("reveal_s")
            if reveal_s is None or self.s + pre_d < reveal_s:
                return None
        return self._peek_look_angles(x, y, z, e, peek)

    # ------------------------------------------------------------ calibration

    def _calibrate_mouse(self, mem_yaw: float, mem_pitch: float) -> None:
        """
        Δугол из памяти vs отправленные пиксели за окно последних тиков.
        Окно (а не один тик) делает оценку устойчивой к задержке ввода на тик.
        """
        if not self.cfg.calibrate_online or self._prev_mem is None:
            return
        dx, dy = self._sent_px
        d_yaw = shortest_yaw_err(mem_yaw, self._prev_mem[0])
        d_pitch = mem_pitch - self._prev_mem[1]
        self._cal_win.append((dx, dy, d_yaw, d_pitch))
        if len(self._cal_win) > 10:
            self._cal_win.pop(0)
        if len(self._cal_win) < 6:
            return
        sum_dx = sum(w[0] for w in self._cal_win)
        sum_dy = sum(w[1] for w in self._cal_win)
        sum_yaw = sum(w[2] for w in self._cal_win)
        sum_pitch = sum(w[3] for w in self._cal_win)

        def steady(vals: list[int]) -> bool:
            """px за тик примерно постоянны и одного знака → задержка ввода на оценку не влияет.
            Во время флика (px быстро растут/падают) оценка смещена — пропускаем."""
            a = [abs(v) for v in vals]
            if min(vals) < 0 < max(vals):
                return False
            mean = sum(a) / len(a)
            return mean >= 4 and (max(a) - min(a)) <= 0.35 * mean + 2

        if abs(sum_dx) >= 60 and not steady([w[0] for w in self._cal_win]):
            sum_dx = 0
        if abs(sum_dy) >= 60 and not steady([w[1] for w in self._cal_win]):
            sum_dy = 0

        if abs(sum_dx) >= 60:
            est = sum_yaw / sum_dx
            if abs(sum_yaw) < 0.3:
                self._mouse_dead_ticks += 1
                if self._mouse_dead_ticks == 12:
                    self.log("ВНИМАНИЕ: мышь не доходит до игры (yaw не меняется)")
            else:
                self._mouse_dead_ticks = 0
            if 0.004 <= abs(est) <= 0.6:
                if est * self.cfg.yaw_per_px < 0:
                    self._sign_flips_yaw += 1
                    if self._sign_flips_yaw >= 4:
                        # знак меняем, величину оставляем (оценка в момент смены знака шумная)
                        self.cfg.yaw_per_px = -self.cfg.yaw_per_px
                        self._sign_flips_yaw = 0
                        self._cal_win.clear()
                        self.log(f"Мышь: yaw инвертирован → {self.cfg.yaw_per_px:+.4f}°/px")
                else:
                    self._sign_flips_yaw = 0
                    self.cfg.yaw_per_px = 0.8 * self.cfg.yaw_per_px + 0.2 * est
                    if self.cfg.pitch_follows_yaw:
                        # в игре одна чувствительность на обе оси; вертикальных ровных движений мало,
                        # поэтому модуль pitch берём от yaw (знак — свой, см. ниже)
                        self.cfg.pitch_per_px = math.copysign(abs(self.cfg.yaw_per_px), self.cfg.pitch_per_px)

        if abs(sum_dy) >= 60:
            est = sum_pitch / sum_dy
            if 0.004 <= abs(est) <= 0.6:
                if est * self.cfg.pitch_per_px < 0:
                    self._sign_flips_pitch += 1
                    if self._sign_flips_pitch >= 4:
                        self.cfg.pitch_per_px = -self.cfg.pitch_per_px
                        self._sign_flips_pitch = 0
                        self.log(f"Мышь: pitch инвертирован → {self.cfg.pitch_per_px:+.4f}°/px")
                else:
                    self._sign_flips_pitch = 0
                    if not self.cfg.pitch_follows_yaw:
                        self.cfg.pitch_per_px = 0.8 * self.cfg.pitch_per_px + 0.2 * est

    def _detect_yaw_offset(self, mem_yaw: float, vx: float, vy: float, speed: float) -> None:
        """Направление скорости vs ожидаемое по клавишам → смещение соглашения yaw."""
        if not self.cfg.detect_yaw_offset:
            return
        keys = frozenset(self._keys)
        if keys not in _DIR_OF_KEYS or self._same_keys_ticks < 4 or speed < 90.0:
            return
        if abs(self._yaw_rate) > 60.0:
            return
        expected = mem_yaw + self.yaw_offset + _DIR_OF_KEYS[keys]
        actual = math.degrees(math.atan2(vy, vx))
        sample = shortest_yaw_err(actual, expected)
        self._offset_samples.append(sample)
        if len(self._offset_samples) > 30:
            self._offset_samples.pop(0)
        if len(self._offset_samples) < 16:
            return
        sx = sum(math.cos(math.radians(a)) for a in self._offset_samples)
        sy = sum(math.sin(math.radians(a)) for a in self._offset_samples)
        n = len(self._offset_samples)
        r = math.hypot(sx, sy) / n
        med = math.degrees(math.atan2(sy, sx))
        if r > 0.9 and abs(med) > 20.0:
            snapped = round(med / 90.0) * 90.0
            if abs(med - snapped) >= 15.0 or snapped == 0.0:
                # не кратно 90° — это не соглашение yaw, а скольжение вдоль стены/мишени; игнорируем
                self._offset_samples.clear()
                return
            self.yaw_offset = shortest_yaw_err(self.yaw_offset + snapped, 0.0)
            self._offset_samples.clear()
            self.log(f"ВНИМАНИЕ: yaw из памяти смещён, поправка {self.yaw_offset:+.0f}°")
        elif r > 0.9:
            self.yaw_verified = True   # направление скорости совпадает с камерой — соглашение верное

    # ------------------------------------------------------------ control pieces

    def _carrot(self, pos: XYZ, route: Route) -> XYZ:
        """
        Точка, куда идут ноги: впереди по маршруту на carrot_dist, но не дальше, чем
        видно по прямой с запасом от стены — иначе на крутом повороте тело срезает угол
        и упирается в стену (pure pursuit cuts corners).
        """
        cfg = self.cfg
        remaining = route.remaining(self.s)
        if remaining <= cfg.carrot_dist:
            return route.goal
        # не срезать косяк двери: запас ≈ 0.85 hull (раньше 0.5 — бот влетал в раму)
        need = max(cfg.route.los_min_clearance, self.map.cfg.hull_radius * 0.85)
        d = cfg.carrot_dist
        cl = self.map.clearance_at(pos[0], pos[1], pos[2])
        if cl < 22.0:
            d = min(d, 32.0)
        # рядом с мишенью идём точнее по линии маршрута (он уже построен в обход)
        if self.known_targets and self._near_target(pos[0], pos[1], margin=cfg.target_avoid_dist):
            d = min(d, 36.0)
        # не перескакивать вертикальный пролёт (лестница / этаж) — ноги к входу, не к XY верха
        span = route.vertical_span_ahead(self.s, max(d, 96.0), self.map.cfg.crouch_jump_height + 8.0)
        if span is not None:
            climb_s = route.cum[span[3]]
            if climb_s > self.s + 10.0:
                d = min(d, climb_s - self.s)
        max_dz = self.map.cfg.crouch_jump_height + 8.0
        while d > 16.0:
            c = route.point_at(self.s + d)
            if abs(c[2] - pos[2]) > max_dz:
                d -= 8.0
                continue
            if segment_clear(self.map, pos, c, need, cfg.route, self._blocked):
                return c
            d -= 8.0
        return route.point_at(self.s + 16.0)

    def _steer_bearing(self, x: float, y: float, vx: float, vy: float, tx: float, ty: float) -> float:
        """
        Куда толкать тело: желаемая скорость (к морковке, полная) минус текущая.
        Так момент гасится заранее (counter-strafe), тело не выносит в стену на повороте.
        На прямой damping не крутит вектор вбок — иначе W+A/W+D на ровном участке.
        """
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return math.degrees(math.atan2(vy, vx)) if (vx or vy) else 0.0
        vmax = self.cfg.max_speed
        des_x, des_y = dx / d * vmax, dy / d * vmax
        des_x, des_y = self._avoid_targets(x, y, des_x, des_y)
        speed = math.hypot(vx, vy)
        heading_err = 0.0
        if speed > 20.0:
            heading_err = abs(
                shortest_yaw_err(
                    math.degrees(math.atan2(vy, vx)),
                    math.degrees(math.atan2(des_y, des_x)),
                )
            )
        if heading_err < 22.0:
            sx, sy = des_x, des_y
        else:
            sx, sy = des_x - vx * self.cfg.steer_damping, des_y - vy * self.cfg.steer_damping
            if math.hypot(sx, sy) < vmax * 0.15:
                sx, sy = des_x, des_y
        # намерение «куда толкать» у человека не меняется каждые 25 мс — сглаживаем вектор
        a = self.cfg.steer_smooth
        px, py = self._steer_vec
        if px == 0.0 and py == 0.0:
            px, py = sx, sy
        sx, sy = px * a + sx * (1.0 - a), py * a + sy * (1.0 - a)
        self._steer_vec = (sx, sy)
        return math.degrees(math.atan2(sy, sx))

    def _choose_keys(self, rel: float, yaw_err: float, speed: float, engaged: bool = False) -> set[str]:
        """WASD по углу между направлением толчка и камерой."""
        arel = abs(rel)
        if engaged:
            # камера на мишени, тело идёт по маршруту: любое из 8 направлений,
            # в т.ч. назад/назад-вбок — так игрок отходит, не отрывая прицел
            cur = frozenset(self._keys)
            _, best_keys = min(_KEY_DIRS, key=lambda kd: abs(shortest_yaw_err(kd[0], rel)))
            if cur in _DIR_OF_KEYS and cur != best_keys:
                cur_err = abs(shortest_yaw_err(_DIR_OF_KEYS[cur], rel))
                if cur_err <= 22.5 + self.cfg.key_hysteresis_deg:
                    return set(cur)
                if self._now() - self._keys_since < self.cfg.key_min_hold and cur_err < 80.0:
                    return set(cur)
                # во время резкого флика человек не перебирает клавиши — держит и поправляет после
                if abs(self._yaw_rate) > 220.0 and cur_err < 110.0:
                    return set(cur)
            return set(best_keys)
        if arel > self.cfg.turn_in_place_deg:
            # цель сзади: как человек — остановиться и развернуться
            if speed > 120.0:
                return {"s"} if arel > 157.5 else ({"s", "a"} if rel > 0 else {"s", "d"})
            # на малой скорости сначала ждём, пока камера довернёт (yaw_err большой).
            # если yaw_err мал > wait_turn — значит look_target впереди по маршруту (U-turn),
            # идём назад по rel, иначе так и будем стоять.
            if abs(yaw_err) > 10.0:
                self._face_turn_since = None
                return set()
            if self._face_turn_since is None:
                self._face_turn_since = self._now()
            if self._now() - self._face_turn_since < 0.35:
                return set()
            _, back_keys = min(_KEY_DIRS, key=lambda kd: abs(shortest_yaw_err(kd[0], rel)))
            return set(back_keys)
        else:
            self._face_turn_since = None
        if arel > self.cfg.strafe_only_deg:
            return {"a"} if rel > 0 else {"d"}
        # прямо по маршруту — только W, даже если морковка/damping чуть сбоку
        if arel <= self.cfg.forward_only_deg:
            return {"w"}

        cur = frozenset(self._keys)
        best_angle, best_keys = min(_KEY_DIRS, key=lambda kd: abs(shortest_yaw_err(kd[0], rel)))
        if cur in _DIR_OF_KEYS and cur != best_keys:
            cur_err = abs(shortest_yaw_err(_DIR_OF_KEYS[cur], rel))
            # на прямой легко отпускаем A/D; на повороте держим комбинацию
            hold_cone = 12.0 if ("a" in cur or "d" in cur) and "w" in cur else (22.5 + self.cfg.key_hysteresis_deg)
            if cur_err <= hold_cone:
                return set(cur)
            # минимальное удержание: человек не перебирает клавиши каждые 25 мс
            if self._now() - self._keys_since < self.cfg.key_min_hold and cur_err < 80.0:
                return set(cur)
        keys = set(best_keys)
        # назад не ходим — только W-варианты и чистый страйф
        if "s" in keys:
            keys = {"a"} if rel > 0 else {"d"}
        return keys

    def _camera_step(
        self,
        yaw_err: float,
        pitch_err: float,
        dt: float,
        speed: float = 0.0,
        rate_max: float | None = None,
        gain: float | None = None,
        pitch_gain: float | None = None,
        pitch_rate_max: float | None = None,
        deadband: float | None = None,
        wind: bool = True,
    ) -> tuple[float, float]:
        cfg = self.cfg
        pitch_gain = cfg.pitch_gain if pitch_gain is None else pitch_gain
        pitch_rate_max = cfg.pitch_rate_max if pitch_rate_max is None else pitch_rate_max
        db = cfg.yaw_deadband if deadband is None else deadband
        if rate_max is None:
            # на бегу крутим камеру спокойнее, стоя — можно резче (как игрок разворачивается)
            rate_max = cfg.yaw_rate_max - (cfg.yaw_rate_max - cfg.yaw_rate_max_moving) * min(1.0, speed / 150.0)
        gain = cfg.yaw_gain if gain is None else gain
        # темп поворота на маршруте слегка плавает (±cam_pace_range) — рука не держит одну скорость
        self._cam_pace = self._cam_pace * 0.97 + self.rng.gauss(0.0, cfg.cam_pace_sigma)
        self._cam_pace = max(-cfg.cam_pace_range, min(cfg.cam_pace_range, self._cam_pace))
        rate_max *= 1.0 + self._cam_pace
        gain *= 1.0 + 0.55 * self._cam_pace
        # желаемая угловая скорость: пропорциональна ошибке, ограничена.
        # Мягкая мёртвая зона (а не порог): у порога скорость не включается/выключается рывком.
        soft = abs(yaw_err) / (abs(yaw_err) + db)
        desired = max(-rate_max, min(rate_max, gain * yaw_err * soft))
        # S-образный разгон/торможение: скорость догоняет желаемую через ускорение,
        # а само ускорение меняется не быстрее yaw_jerk_max (без «углов» на графике скорости)
        self._yaw_rate, self._yaw_acc = self._track_rate(
            self._yaw_rate, self._yaw_acc, desired, dt, cfg.yaw_accel_max, cfg.yaw_jerk_max
        )
        d_yaw = self._yaw_rate * dt

        soft_p = abs(pitch_err) / (abs(pitch_err) + (0.5 if deadband is None else max(0.08, db)))
        p_des = max(-pitch_rate_max, min(pitch_rate_max, pitch_gain * pitch_err * soft_p))
        self._pitch_rate, self._pitch_acc = self._track_rate(
            self._pitch_rate, self._pitch_acc, p_des, dt, cfg.yaw_accel_max * 0.5, cfg.yaw_jerk_max * 0.5
        )
        d_pitch = self._pitch_rate * dt

        # ветер до ограничения по ошибке: у цели гасим, иначе камера уезжает и виляет
        if wind:
            activity = min(1.0, (abs(self._yaw_rate) + abs(self._pitch_rate)) / 60.0)
            activity *= min(1.0, math.hypot(yaw_err, pitch_err) / 3.5)
            wy, wp = self._wind_step(cfg.hand_wind_sigma, cfg.hand_wind_decay)
            d_yaw += wy * activity * dt
            d_pitch += wp * activity * 0.5 * dt

        # не перелетать точку взгляда; скорость обнуляем (не err/dt — иначе bang-bang влево-вправо)
        if abs(d_yaw) > abs(yaw_err):
            d_yaw = yaw_err
            self._yaw_rate = 0.0
            self._yaw_acc = 0.0
        if abs(d_pitch) > abs(pitch_err):
            d_pitch = pitch_err
            self._pitch_rate = 0.0
            self._pitch_acc = 0.0
        return d_yaw, d_pitch

    # ------------------------------------------------------------ aiming at a target

    @staticmethod
    def _min_jerk(tau: float) -> float:
        tau = max(0.0, min(1.0, tau))
        return tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau)

    @staticmethod
    def _min_jerk_vel(tau: float) -> float:
        """Производная min-jerk (форма колокола, пик 1.875 при τ=0.5)."""
        tau = max(0.0, min(1.0, tau))
        return 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau)

    @staticmethod
    def _track_rate(
        rate: float, acc: float, desired: float, dt: float, acc_max: float, jerk_max: float
    ) -> tuple[float, float]:
        """
        Скорость → к желаемой через ускорение с ограничением рывка (S-кривая).
        Ускорение выбирается так, чтобы успеть затормозить у желаемой скорости: |a| ≤ √(2·j·|Δv|).
        """
        dv = desired - rate
        # желаемое ускорение: пропорционально Δv, но не больше того, что можно «погасить» рывком
        a_stop = math.sqrt(2.0 * jerk_max * abs(dv)) if dv else 0.0
        a_des = max(-acc_max, min(acc_max, dv / dt))
        a_des = max(-a_stop, min(a_stop, a_des))
        max_da = jerk_max * dt
        acc += max(-max_da, min(max_da, a_des - acc))
        new_rate = rate + acc * dt
        # не проскакивать желаемую скорость
        if (dv > 0 and new_rate > desired) or (dv < 0 and new_rate < desired):
            new_rate = desired
            acc = 0.0
        return new_rate, acc

    # --- «рука»: ветер и тремор как в WindMouse — инерционные случайные процессы, а не белый шум

    def _wind_step(self, sigma: float, decay: float) -> tuple[float, float]:
        """AR(1)-ветер (°/с) со стационарным σ: W ← W·decay + N(0, σ·√(1−decay²))."""
        k = sigma * math.sqrt(max(0.0, 1.0 - decay * decay))
        self._wind[0] = self._wind[0] * decay + self.rng.gauss(0.0, k)
        self._wind[1] = self._wind[1] * decay + self.rng.gauss(0.0, k)
        return self._wind[0], self._wind[1]

    def _tremor_step(self) -> tuple[float, float]:
        """Тремор удержания (°/тик): медленно меняющийся, без «зернистости» тик-в-тик."""
        decay = 0.85
        k = self.cfg.tremor_sigma * math.sqrt(1.0 - decay * decay)
        self._tremor[0] = self._tremor[0] * decay + self.rng.gauss(0.0, k)
        self._tremor[1] = self._tremor[1] * decay + self.rng.gauss(0.0, k)
        return self._tremor[0], self._tremor[1]

    def _aim_drift_step(self, sigma: float) -> tuple[float, float]:
        """Дрейф точки прицеливания на мишени (°): AR(1) вместо нового случайного смещения каждый тик."""
        decay = 0.92
        k = sigma * math.sqrt(1.0 - decay * decay)
        self._aim_drift[0] = self._aim_drift[0] * decay + self.rng.gauss(0.0, k)
        self._aim_drift[1] = self._aim_drift[1] * decay + self.rng.gauss(0.0, k)
        return self._aim_drift[0], self._aim_drift[1]

    def _xx_u01(self) -> float:
        """Следующее псевдослучайное [0, 1) из xxHash32(счётчик, зерно)."""
        self._aim_xx_n = (self._aim_xx_n + 1) & 0xFFFFFFFF
        return _xxhash32_int(self._aim_xx_n, self._aim_xx_seed) * (1.0 / 4294967296.0)

    def _xx_range(self, span: tuple[float, float]) -> float:
        """Число внутри диапазона: границы и точка между ними — xxHash, не одна константа."""
        lo, hi = span
        return lo + self._xx_u01() * (hi - lo)

    def _sample_aim_accel(self) -> float:
        """Ускорение к скорости фазы. Скорость камеры при этом не обнуляется."""
        self._aim_accel_mul = self._xx_range(self.cfg.aim_accel_span)
        return self._aim_accel_mul

    def _aim_speed_resample(self) -> None:
        """Новая точка в диапазоне на каждое захватывание мишени (одинаковая дистанция ≠ одна скорость)."""
        j = self.cfg.aim_speed_jitter
        self._aim_speed_var = (self._xx_u01() * 2.0 - 1.0) * j
        self._aim_speed_mul = 1.0 + self._aim_speed_var
        self._sample_aim_accel()

    def _aim_speed_step(self) -> float:
        """AR(1)-плавание множителя в ±jitter; шум — xxHash, не gauss из random."""
        cfg = self.cfg
        j = max(0.0, cfg.aim_speed_jitter)
        decay = 0.93
        kick = math.sqrt(max(0.0, 1.0 - decay * decay)) * j
        self._aim_speed_var = self._aim_speed_var * decay + (self._xx_u01() * 2.0 - 1.0) * kick
        self._aim_speed_var = max(-j, min(j, self._aim_speed_var))
        self._aim_speed_mul = 1.0 + self._aim_speed_var
        return self._aim_speed_mul

    def _nav_yaw_cap(self, speed: float) -> float:
        cfg = self.cfg
        return cfg.yaw_rate_max - (cfg.yaw_rate_max - cfg.yaw_rate_max_moving) * min(1.0, speed / 150.0)

    def _aim_dist_boost(self, dist: float) -> float:
        """Ближе — чуть быстрее панорамы маршрута; далеко — почти как навигация."""
        cfg = self.cfg
        span = max(1.0, cfg.aim_far_dist - cfg.aim_close_dist)
        t = max(0.0, min(1.0, (dist - cfg.aim_close_dist) / span))
        return cfg.aim_close_boost + (cfg.aim_far_boost - cfg.aim_close_boost) * t

    def _target_vel_xy(self, target: dict, now: float) -> tuple[float, float]:
        vel = target.get("velocity")
        mem = (0.0, 0.0)
        if isinstance(vel, (tuple, list)) and len(vel) >= 2:
            mem = (float(vel[0]), float(vel[1]))
        idx = target["index"]
        tx, ty, _ = target["position"]
        prev = self._tgt_xy.get(idx)
        self._tgt_xy[idx] = (tx, ty, now)
        est = mem
        if prev is not None:
            dt = now - prev[2]
            if dt >= 0.03:
                est = ((tx - prev[0]) / dt, (ty - prev[1]) / dt)
        if math.hypot(*mem) >= 12.0:
            return mem
        return est

    def _update_aim_chase(self, target: dict | None, x: float, y: float, dt: float, now: float) -> None:
        """Бежит / стрейфит — ускоряем наводку с текущей скорости, множитель xxHash, S-кривая."""
        cfg = self.cfg
        want = 1.0
        moving = False
        if target is not None:
            vx, vy = self._target_vel_xy(target, now)
            spd = math.hypot(vx, vy)
            tx, ty, _ = target["position"]
            lx, ly = tx - x, ty - y
            llen = math.hypot(lx, ly) or 1.0
            lat = vx * (-ly / llen) + vy * (lx / llen)
            sign = 0 if abs(lat) < 18.0 else (1 if lat > 0 else -1)
            adad = sign != 0 and self._lat_sign != 0 and sign != self._lat_sign
            self._lat_sign = sign if sign else self._lat_sign
            moving = spd >= cfg.aim_chase_move or abs(lat) >= cfg.aim_chase_strafe or adad
            self._tgt_lat = lat
            self._tgt_aim_dist = llen
            if moving:
                if not self._chase_on:
                    self._chase_mul = self._xx_range(cfg.aim_chase_speed)
                    self._chase_rise = self._xx_range(cfg.aim_chase_rise)
                    self._sample_aim_accel()
                    self._chase_on = True
                    self._begin_track_over(now)
                want = self._chase_mul * self._track_pace
                self._advance_track_phase(now)
            else:
                self._chase_on = False
                if not self._shot_miss:
                    self._track_phase = "center"
                    self._track_want = (0.0, 0.0)
        else:
            self._chase_on = False
            self._lat_sign = 0
            if not self._shot_miss:
                self._track_phase = "center"
                self._track_want = (0.0, 0.0)
        tau = self._chase_rise if want > self._chase_boost else cfg.aim_chase_fall
        k = 1.0 - math.exp(-dt / max(0.03, tau))
        # резкий набор, без скачка скорости камеры: меняется только потолок, _yaw_rate как был
        self._chase_boost += (want - self._chase_boost) * min(1.0, k * (1.85 if want > 1.05 else 1.0))
        self._slew_track_aim(dt)

    def _begin_track_over(self, now: float) -> None:
        """Одно упреждение по стрейфу: вывести прицел по направлению движения мишени
        на доли градуса и вернуть в центр. НЕ зацикленно (чтобы прицел не вилял
        влево-вправо на малых скоростях). Если стрейфа почти нет — ничего не делаем."""
        cfg = self.cfg
        # Нет внятного поперечного движения — не трогаем прицел, просто ведём центр.
        if abs(self._tgt_lat) < 22.0:
            self._track_phase = "center"
            self._track_want = (0.0, 0.0)
            self._track_until = now
            return
        lo, hi = cfg.aim_overshoot_u
        lead_u = lo + self._xx_u01() * (hi - lo)
        lead = math.degrees(math.atan2(lead_u, max(80.0, self._tgt_aim_dist)))
        cap = 0.28 if self._scoped_now else 0.65
        lead = min(lead, cap)
        sign = 1.0 if self._tgt_lat >= 0.0 else -1.0
        plo, phi = cfg.aim_track_pace
        self._track_pace = plo + self._xx_u01() * (phi - plo)
        slo, shi = cfg.aim_overshoot_s
        # pitch-компонента нулевая: по вертикали не виляем вообще
        self._track_want = (sign * lead, 0.0)
        self._track_phase = "over"
        self._track_until = now + slo + self._xx_u01() * (shi - slo)

    def _advance_track_phase(self, now: float) -> None:
        if now < self._track_until:
            return
        cfg = self.cfg
        slo, shi = cfg.aim_overshoot_s
        if self._track_phase == "over":
            # После упреждения — возврат в центр и всё, цикл не продолжаем.
            self._track_phase = "back"
            self._track_want = (0.0, 0.0)
            self._track_until = now + 0.10 + self._xx_u01() * (shi - slo) * 0.7
            return
        if self._track_phase == "back":
            # Остановились на мишени. Повторный «over» — только если стрейф поменяет знак
            # (это проверяется в _update_aim_chase через adad), а здесь — тишина.
            self._track_phase = "center"
            self._track_want = (0.0, 0.0)

    def _slew_track_aim(self, dt: float) -> None:
        wy, wp = self._track_want
        rate = 4.5 + 3.5 * self._track_pace
        for i, w in enumerate((wy, wp)):
            err = w - self._track_aim[i]
            step = max(-rate * dt, min(rate * dt, err))
            self._track_aim[i] += step

    def _aim_track(
        self, yaw_err: float, pitch_err: float, dt: float, speed: float, dist: float
    ) -> tuple[float, float]:
        """Наводка без флика: та же S-кривая, что на маршруте, начальная скорость не сбрасывается."""
        cfg = self.cfg
        boost = self._aim_dist_boost(dist) * self._aim_speed_mul * max(1.0, self._chase_boost)
        rate_max = self._nav_yaw_cap(speed) * boost
        gain = cfg.yaw_gain * (1.0 + 0.4 * (boost - 1.0) / max(0.01, cfg.aim_close_boost - 1.0))
        pitch_max = max(cfg.pitch_rate_max * boost, 90.0)
        pitch_gain = cfg.pitch_gain * (1.0 + 0.45 * (boost - 1.0) / max(0.01, cfg.aim_close_boost - 1.0))
        return self._camera_step(
            yaw_err, pitch_err, dt, speed, rate_max, gain, pitch_gain, pitch_max, deadband=0.12
        )

    def _aim_lock(self, yaw_err: float, pitch_err: float, dt: float) -> tuple[float, float]:
        """Дожим в центр: скорость с текущей, ветер — тот же WindMouse, что на маршруте."""
        cfg = self.cfg
        boost = max(1.0, self._chase_boost)
        # ведение ходящей мишени быстрее дожима стоячей; потолок и ускорение — из xxHash, не константа
        follow = boost if self._chase_on else 1.0
        g = cfg.aim_lock_gain * follow
        r = cfg.aim_lock_rate * boost * self._aim_speed_mul
        acc = 6400.0 * self._aim_accel_mul
        jerk = cfg.yaw_jerk_max * 3.0 * self._aim_accel_mul
        lag = dt
        ey = yaw_err - self._yaw_rate * lag
        ep = pitch_err - self._pitch_rate * lag
        # меньше полупикселя — не дёргаем мышь туда-сюда (иначе ±1 px дрожание у точки)
        if abs(ey) < 0.55 * abs(cfg.yaw_per_px):
            ey = 0.0
            self._yaw_rate, self._yaw_acc = 0.0, 0.0
        if abs(ep) < 0.55 * abs(cfg.pitch_per_px):
            ep = 0.0
            self._pitch_rate, self._pitch_acc = 0.0, 0.0
        vmax_y = math.sqrt(2.0 * 0.7 * acc * abs(ey)) if ey else 0.0
        vmax_p = math.sqrt(2.0 * 0.7 * acc * abs(ep)) if ep else 0.0
        des_y = max(-min(r, vmax_y), min(min(r, vmax_y), g * ey))
        des_p = max(-min(r, vmax_p), min(min(r, vmax_p), g * ep))
        self._yaw_rate, self._yaw_acc = self._track_rate(self._yaw_rate, self._yaw_acc, des_y, dt, acc, jerk)
        self._pitch_rate, self._pitch_acc = self._track_rate(self._pitch_rate, self._pitch_acc, des_p, dt, acc, jerk)
        d_yaw = self._yaw_rate * dt
        d_pitch = self._pitch_rate * dt
        activity = min(1.0, (abs(self._yaw_rate) + abs(self._pitch_rate)) / 60.0)
        activity *= min(1.0, abs(yaw_err) / 3.5)
        wy, wp = self._wind_step(cfg.hand_wind_sigma, cfg.hand_wind_decay)
        d_yaw += wy * activity * dt
        d_pitch += wp * activity * 0.5 * dt
        # к точке ведения (голова + лёгкий перелёт), без уезда ветром в сторону
        if abs(d_yaw) > abs(yaw_err):
            d_yaw = yaw_err
            self._yaw_rate, self._yaw_acc = 0.0, 0.0
        if abs(d_pitch) > abs(pitch_err):
            d_pitch = pitch_err
            self._pitch_rate, self._pitch_acc = 0.0, 0.0
        return d_yaw, d_pitch

    def _plan_flick(self, now: float, yaw_err: float, pitch_err: float, corrective: bool = False) -> None:
        """
        Баллистический мах к мишени: длительность по Фиттсу, скорость — колокол minimum-jerk,
        траектория — дуга (боковой изгиб sin(πτ)+sin(2πτ), как реальный мах кистью/рукой),
        конец — чуть недоведён и с разбросом. corrective — второе, корректирующее подвижение:
        точнее, без недовода, изгиб меньше.
        Старт с текущей скорости камеры (навигация): v0 затухает к концу маха.
        """
        cfg = self.cfg
        dist = math.hypot(yaw_err, pitch_err)
        self._flick_pending = False
        if dist < cfg.flick_min_deg:
            self._flick = None
            self._correct_from = now            # сразу коррекция
            return
        acc_mul = self._sample_aim_accel()
        T = (cfg.flick_base_time + cfg.flick_per_deg * dist) / acc_mul
        rate_cap = cfg.flick_rate_max * self._aim_speed_mul
        T = max(T, 1.875 * dist / max(120.0, rate_cap))
        ux, uy = yaw_err / dist, pitch_err / dist
        prec = 0.5 if corrective else 1.0
        under = 0.0 if corrective else cfg.flick_undershoot
        along = dist * (1.0 - under) + self.rng.gauss(0.0, prec * (cfg.flick_error_frac * dist + cfg.flick_error_min))
        along = max(dist * 0.5, along)
        # поперёк промахиваются гораздо меньше, чем по дальности (направление рука знает точно):
        # σ ≈ четверть продольной, и не больше 0.6° — иначе в конце появляется «крючок» на коррекции
        across = self.rng.gauss(0.0, prec * 0.2 * (cfg.flick_error_frac * dist + cfg.flick_error_min))
        across = max(-0.12, min(0.12, across))
        self._flick = {
            "t0": now,
            "T": T,
            "u": (ux, uy),                      # вдоль
            "n": (-uy, ux),                     # поперёк
            "along": along,
            "across": across,
            "c1": self.rng.gauss(0.0, prec * cfg.flick_curve_frac * dist),        # изгиб дуги
            "c2": self.rng.gauss(0.0, prec * 0.3 * cfg.flick_curve_frac * dist),  # асимметрия дуги
            "wind_sigma": min(1.5, prec * cfg.flick_wind_frac * dist),           # ° амплитуда волны, ∝ длине маха
            "v0": (self._yaw_rate, self._pitch_rate),  # скорость в момент старта (= навигация)
            "prev": (0.0, 0.0),
        }
        self._aim_used_flick = True
        self._wind = [0.0, 0.0]
        delay = self.rng.uniform(*cfg.aim_correct_delay) * (0.5 if corrective else 1.0)
        self._correct_from = now + T + delay
        self._flick["ramp"] = min(cfg.flick_ramp * (0.55 if corrective else 1.0), 0.45 * T)

    def _flick_pos(self, f: dict, tau: float) -> tuple[float, float]:
        s = self._min_jerk(tau)
        a = f["along"] * s
        c = f["across"] * s + f["c1"] * math.sin(math.pi * tau) + f["c2"] * math.sin(2.0 * math.pi * tau)
        ux, uy = f["u"]
        nx, ny = f["n"]
        px, py = ux * a + nx * c, uy * a + ny * c
        # с текущей скорости маршрута: v0·t·(1−τ)² → 0 к концу, в цель приходим как по профилю
        v0 = f.get("v0", (0.0, 0.0))
        t = max(0.0, tau) * f["T"]
        fade = (1.0 - max(0.0, min(1.0, tau))) ** 2
        px += v0[0] * t * fade
        py += v0[1] * t * fade
        return px, py

    def _aim_step(
        self,
        now: float,
        dt: float,
        yaw_err: float,
        pitch_err: float,
        aim_yaw: float,
        aim_pitch: float,
        speed: float,
        dist: float,
    ) -> tuple[float, float, str]:
        """
        Наводка: сначала продолжаем поворот как на маршруте (та же скорость), затем
        при угле ≥ aim_flick_deg — флик; иначе так и ведём камеру, чуть быстрее на ближней.

        Ошибку считаем с учётом движений мыши «в полёте» (отправлены, в памяти ещё нет):
        иначе петля досылает уже посланное, перелетает и ёрзает около точки.
        """
        self._aim_speed_step()
        return self._aim_step_inner(
            now, dt,
            yaw_err - self._pend_yaw.value(),
            pitch_err - self._pend_pitch.value(),
            aim_yaw, aim_pitch, speed, dist,
        )

    def _aim_step_inner(
        self,
        now: float,
        dt: float,
        yaw_err: float,
        pitch_err: float,
        aim_yaw: float,
        aim_pitch: float,
        speed: float,
        dist: float,
    ) -> tuple[float, float, str]:
        cfg = self.cfg
        axis = max(abs(yaw_err), abs(pitch_err))
        if self._flick_pending:
            if self._aim_handoff_t0 is None:
                self._aim_handoff_t0 = now
                if self._aim_fast_peek:
                    self._peek_handoff = 0.035 + self._xx_u01() * 0.040
                else:
                    self._peek_handoff = cfg.aim_handoff_s
            handoff = self._peek_handoff
            if axis > 75.0:
                handoff *= 0.62
            if now - self._aim_handoff_t0 < handoff:
                oy, op = self._track_aim
                d_yaw, d_pitch = self._aim_track(yaw_err - oy, pitch_err - op, dt, speed, dist)
                return d_yaw, d_pitch, "handoff"
            if axis >= cfg.aim_flick_deg:
                self._plan_flick(now, yaw_err, pitch_err)
                if self._flick is not None:
                    self._flick["aim_prev"] = (aim_yaw, aim_pitch)
            else:
                self._flick_pending = False
                self._flick = None
                self._correct_from = now
        f = self._flick
        if f is not None:
            tau = (now + dt - f["t0"]) / f["T"]
            px, py = self._flick_pos(f, tau)
            d_yaw = px - f["prev"][0]
            d_pitch = py - f["prev"][1]
            f["prev"] = (px, py)
            ap = f.get("aim_prev", (aim_yaw, aim_pitch))
            d_yaw += shortest_yaw_err(aim_yaw, ap[0])
            d_pitch += aim_pitch - ap[1]
            f["aim_prev"] = (aim_yaw, aim_pitch)
            wy, wp = self._wind_step(f["wind_sigma"] / max(0.1, 0.5 * f["T"]), 0.85)
            wpos = f.setdefault("wind_pos", [0.0, 0.0])
            wpos[0] += wy * dt
            wpos[1] += wp * dt
            env = math.sin(math.pi * min(1.0, tau))
            wnow = (wpos[0] * env, wpos[1] * env)
            wprev = f.get("wind_prev", (0.0, 0.0))
            d_yaw += wnow[0] - wprev[0]
            d_pitch += wnow[1] - wprev[1]
            f["wind_prev"] = wnow
            if tau >= 1.0:
                self._flick = None
                self._yaw_rate = d_yaw / dt
                self._pitch_rate = d_pitch / dt
                self._sample_aim_accel()
                return d_yaw, d_pitch, "flick"
            self._yaw_rate = d_yaw / dt
            self._pitch_rate = d_pitch / dt
            return d_yaw, d_pitch, "flick"
        if now < self._correct_from:
            self._yaw_rate, self._yaw_acc = self._track_rate(
                self._yaw_rate, self._yaw_acc, 0.0, dt, cfg.yaw_accel_max, cfg.yaw_jerk_max
            )
            self._pitch_rate, self._pitch_acc = self._track_rate(
                self._pitch_rate, self._pitch_acc, 0.0, dt, cfg.yaw_accel_max, cfg.yaw_jerk_max
            )
            return self._yaw_rate * dt, self._pitch_rate * dt, "settle"
        if self._rcs_return_going(now, yaw_err, pitch_err):
            d_yaw, d_pitch = self._rcs_return_step(yaw_err, pitch_err, dt)
            return d_yaw, d_pitch, "rcs_return"
        if not (self._rcs_active() and self._fire_held):
            if math.hypot(yaw_err, pitch_err) >= cfg.flick_min_deg * 1.25 and self._subflicks < 1:
                self._subflicks += 1
                self._plan_flick(now, yaw_err, pitch_err, corrective=True)
                if self._flick is not None:
                    self._flick["aim_prev"] = (aim_yaw, aim_pitch)
                    return self._aim_step_inner(now, dt, yaw_err, pitch_err, aim_yaw, aim_pitch, speed, dist)
        if self._rcs_active() and self._fire_held:
            d_yaw, d_pitch = self._rcs_step(yaw_err, pitch_err, dt)
            return d_yaw, d_pitch, "rcs"
        d_yaw, d_pitch = self._aim_lock(yaw_err - self._track_aim[0], pitch_err - self._track_aim[1], dt)
        return d_yaw, d_pitch, "lock"

    def _send_mouse(self, d_yaw: float, d_pitch: float, max_px: int | None = None) -> tuple[int, int]:
        """Градусы → пиксели с переносом дробного остатка: мелкие плавные повороты не дребезжат."""
        cfg = self.cfg
        m = cfg.max_px_per_tick if max_px is None else max_px
        out = [0, 0]
        for i, (d, k) in enumerate(((d_yaw, cfg.yaw_per_px), (d_pitch, cfg.pitch_per_px))):
            if not k:
                continue
            if self._scoped_now:
                k = k * (40.0 / 90.0)
            want = d / k + self._px_carry[i]
            px = int(round(want))
            if abs(px) > m:
                px = m if px > 0 else -m
                self._px_carry[i] = 0.0          # обрезали — остаток не копим (иначе «догонит» рывком)
            else:
                self._px_carry[i] = want - px
            out[i] = px
            (self._pend_yaw if i == 0 else self._pend_pitch).sent(self._tick_no, px * k)
        dx, dy = out
        if dx or dy:
            self.input.mouse_move(dx, dy)
        self._sent_px = (dx, dy)
        return dx, dy

    def _bump_target(self, now: float, pos: XYZ) -> set[str]:
        """Упёрлись в мишень: шаг назад и обход с увеличенным радиусом; на 3-й раз — другая цель."""
        self._target_bumps += 1
        self.stucks += 1
        self._override_keys = {"s"}
        self._override_until = now + 0.3
        self._target_contact_since = None
        if self._target_bumps >= 3:
            for e in self.known_targets:
                if math.hypot(e["position"][0] - pos[0], e["position"][1] - pos[1]) <= (
                    self.map.cfg.hull_radius + self.cfg.target_body_radius + 24.0
                ):
                    self._given_up[e["index"]] = now + self.cfg.target_done_cooldown
            self._replan(pos, f"упёрлись в мишень (#{self._target_bumps}) → другая цель", new_goal=True)
        else:
            self._replan(pos, f"упёрлись в мишень (#{self._target_bumps})")
        return set(self._override_keys)

    def _handle_stuck(self, now: float, pos: XYZ, speed: float, keys: set[str], rel: float) -> set[str] | None:
        """Возвращает переопределённые клавиши на время манёвра или None."""
        if now < self._override_until:
            return set(self._override_keys)
        # контакт с мишенью дольше полсекунды — застряли, даже если «скользим» вдоль неё
        if self.known_targets and self._near_target(pos[0], pos[1], margin=4.0):
            if self._target_contact_since is None:
                self._target_contact_since = now
            elif now - self._target_contact_since > 0.5:
                return self._bump_target(now, pos)
        else:
            self._target_contact_since = None
        moving_keys = keys & {"w", "a", "s", "d"}
        if not moving_keys or speed >= self.cfg.stuck_speed:
            self._stuck_since = None
            return None
        if self._stuck_since is None:
            self._stuck_since = now
            return None
        if now - self._stuck_since < self.cfg.stuck_time:
            return None

        self._stuck_since = None
        self._stuck_level += 1
        self.stucks += 1
        lvl = self._stuck_level
        if self._near_target(pos[0], pos[1]):
            # упёрлись в мишень — прыгать/страйфить бессмысленно
            self.stucks -= 1
            return self._bump_target(now, pos)
        if lvl == 1:
            # план манёвров на этот эпизод: сначала перепрыгнуть, потом обойти
            cx, cy = pos[0], pos[1]
            cz = None
            if self.route is not None:
                c = self._carrot(pos, self.route)
                cx, cy, cz = c[0], c[1], c[2]
            kind = self._climb_needed(pos[0], pos[1], pos[2], cx, cy, cz)
            if kind is None:
                # карта не знает о препятствии (ящик, пропс, бортик), а мы упёрлись в него
                # с зажатым W — прыгаем вслепую: сначала обычный, потом с приседом
                self._stuck_plan = ["jump", "crouch_jump", "strafe", "back"]
            else:
                self._stuck_plan = [kind, "strafe", "back"]
        step = self._stuck_plan.pop(0) if self._stuck_plan else None
        if step in ("jump", "crouch_jump"):
            if now - self._last_jump_at >= 0.55:
                self.log("Застрял → " + ("прыжок+присед" if step == "crouch_jump" else "прыжок"))
                self._do_climb(step, now)
                # держим ход вперёд, чтобы залететь на препятствие, а не подпрыгнуть на месте
                self._override_keys = ((moving_keys or {"w"}) - {"s"}) or {"w"}
                self._override_until = now + (0.5 if step == "crouch_jump" else 0.45)
                return set(self._override_keys)
            return None
        if step == "strafe":
            side = {"d"} if rel < 0 else {"a"}
            self._override_keys = side | {"w"}
            self._override_until = now + 0.35
            self.log(f"Застрял → страйф {''.join(sorted(side)).upper()}")
            return set(self._override_keys)
        if step == "back":
            self._override_keys = {"s"}
            self._override_until = now + 0.3
            self.log("Застрял → шаг назад")
            return set(self._override_keys)
        self._replan(pos, "застревание", new_goal=lvl >= self.cfg.stuck_replan_after + 2)
        return None

    # ------------------------------------------------------------ main tick

    # ------------------------------------------------------------ firing

    def _sniper_kind(self) -> str:
        wid = int(self.weapon.get("id") or 0)
        name = str(self.weapon.get("name") or "").lower()
        if wid == 9 or "awp" in name:
            return "awp"
        if wid == 40 or "ssg" in name or "scout" in name:
            return "ssg"
        return "auto"

    def _noscope_now(self, dist: float) -> bool:
        """Жребий зафиксирован на мишень; ближняя дистанция проверяется каждый тик."""
        plan = self._scope_plan
        if plan is None:
            return False
        if plan.get("noscope"):
            return True
        return plan.get("kind") in {"awp", "ssg"} and dist <= self.cfg.scope_noscope_close

    def _sample_scope(self, index: int, dist: float | None = None) -> None:
        """На захват: ноускоуп по своему шансу оружия (xxHash) или вблизи у AWP/SSG."""
        self._scope_committed = False
        wid = int(self.weapon.get("id") or 0)
        name = str(self.weapon.get("name") or "")
        if not weapon_needs_scope(wid, name):
            self._scope_plan = None
            self._begin_unscope(use_knife=False)
            return
        cfg = self.cfg
        kind = self._sniper_kind()
        if kind == "awp":
            chance = cfg.scope_noscope_awp
        elif kind == "ssg":
            chance = cfg.scope_noscope_ssg
        else:
            chance = cfg.scope_noscope_chance
        close = (
            dist is not None
            and kind in {"awp", "ssg"}
            and dist <= cfg.scope_noscope_close
        )
        noscope = close or self._xx_u01() < chance
        lo, hi = cfg.scope_enter_deg
        deg = lo + self._xx_u01() * (hi - lo)
        lo, hi = cfg.scope_enter_u
        radius_u = lo + self._xx_u01() * (hi - lo)
        lo, hi = cfg.scope_settle
        settle = lo + self._xx_u01() * (hi - lo)
        self._scope_plan = {
            "index": index,
            "kind": kind,
            "noscope": noscope,
            "deg": deg,
            "radius_u": radius_u,
            "settle": settle,
            "in_at": None,
        }
        if noscope:
            self._begin_unscope(use_knife=False)
            why = "вблизи" if close else f"{chance * 100:.0f}%"
            self.log(f"Снайперка ({kind}): ноускоуп ({why})")
        else:
            self._unscope_left = 0
            self.log(f"Снайперка: скоуп с {deg:.1f}° / {radius_u:.0f}u")

    def _in_scope_zone(self, yaw_err: float, pitch_err: float, dist: float) -> bool:
        plan = self._scope_plan
        if not plan or self._noscope_now(dist):
            return False
        ang = math.hypot(yaw_err, pitch_err)
        world = max(0.0, dist) * math.tan(math.radians(min(89.0, ang)))
        return ang <= plan["deg"] and world <= plan["radius_u"]

    def _press_scope(self) -> None:
        if self._scope_held:
            return
        self.input.mouse_down("right")
        self._scope_held = True
        self._scope_click_until_tick = -1

    def _release_scope(self) -> None:
        if self._scope_held:
            self.input.mouse_up("right")
        self._scope_held = False
        self._scope_click_until_tick = -1

    def _release_scope_if_due(self) -> None:
        if self._scope_click_until_tick >= 0 and self._tick_no >= self._scope_click_until_tick:
            if self._scope_held:
                self.input.mouse_up("right")
            self._scope_held = False
            self._scope_click_until_tick = -1

    def _unscoping(self) -> bool:
        h = self._habit
        return self._unscope_left > 0 or bool(h and h.get("kind") == "unscope_knife")

    def _start_unscope_knife(self, now: float) -> None:
        """Выход из скоупа как привычка ножа: 3, пауза _key_gap, слот оружия."""
        if self._habit and self._habit.get("kind") == "unscope_knife":
            return
        if self._habit:
            self._habit_abort(now, back_to_gun=False)
        self._release_scope()
        self._scope_committed = False
        back = self._gun_slot_key()
        self._pulse_key("3", now)
        self._habit = {
            "kind": "unscope_knife",
            "phase": "to_knife",
            "back": back,
            "t0": now,
            "next": now + self._key_gap(),
        }
        self.log(f"Выход из скоупа (нож → слот {back})")

    def _begin_unscope(self, now: float | None = None, *, use_knife: bool = True) -> None:
        """После боя: 80% нож→оружие (как привычка), 20% два клика ПКМ с паузой (xxHash)."""
        now = self._now() if now is None else now
        if self._unscoping():
            return
        self._release_scope()
        self._scope_committed = False
        if not self._scoped_now:
            return
        if use_knife and self._xx_u01() >= self.cfg.unscope_click_chance:
            self._start_unscope_knife(now)
            return
        self._unscope_left = 2
        self._unscope_next = now + 0.04 + self._xx_u01() * 0.10
        self.log("Выход из скоупа (2×ПКМ)")

    def _unscope_tick(self, now: float) -> None:
        if self._unscope_left <= 0:
            return
        if self._scope_held:
            return
        if now < self._unscope_next:
            return
        if not self._scoped_now:
            self._unscope_left = 0
            return
        self._press_scope()
        hold = 1 + (1 if self._xx_u01() < 0.45 else 0)
        self._scope_click_until_tick = self._tick_no + hold
        self._unscope_left -= 1
        lo, hi = self.cfg.unscope_click_gap
        self._unscope_next = now + lo + self._xx_u01() * (hi - lo)
        if self._unscope_left <= 0:
            self.log("Скоуп сброшен")

    def _scope_ready(self, now: float) -> bool:
        plan = self._scope_plan
        if plan is None or plan.get("noscope"):
            return True
        if not self._scoped_now:
            return False
        t0 = plan.get("in_at")
        if t0 is None:
            return False
        return now - t0 >= plan["settle"]

    def _scope_blocks_fire(self, now: float, yaw_err: float, pitch_err: float, dist: float) -> bool:
        """Снайперка: не стрелять, пока не вошли в скоуп и не довели (кроме ноускопа)."""
        plan = self._scope_plan
        wid = int(self.weapon.get("id") or 0)
        name = str(self.weapon.get("name") or "")
        if self._unscoping():
            return True
        if not weapon_needs_scope(wid, name) or plan is None:
            return False
        if self._noscope_now(dist):
            return bool(self._scoped_now)
        if not (self._scope_committed or self._in_scope_zone(yaw_err, pitch_err, dist)):
            return False
        return not self._scope_ready(now)

    def _scope_step(self, now: float, yaw_err: float, pitch_err: float, dist: float) -> None:
        """Около мишени — ПКМ в скоуп, дальше доводка; после выстрела AWP жмём снова."""
        wid = int(self.weapon.get("id") or 0)
        name = str(self.weapon.get("name") or "")
        plan = self._scope_plan
        if self._unscoping():
            return
        if not weapon_needs_scope(wid, name) or plan is None or self._noscope_now(dist):
            if (
                plan is not None
                and not plan.get("noscope")
                and plan.get("kind") in {"awp", "ssg"}
                and dist <= self.cfg.scope_noscope_close
                and not plan.get("close_logged")
            ):
                plan["close_logged"] = True
                self.log(f"Снайперка ({plan['kind']}): ноускоуп (вблизи, {dist:.0f}u)")
            if self._scoped_now or self._scope_held:
                self._begin_unscope(now, use_knife=False)
            else:
                self._release_scope()
            self._scope_committed = False
            if plan is not None:
                plan["in_at"] = None
            return
        if not self._target_visible_now:
            # убита / пропала: не зумимся в труп, после выстрела нож (редко 2×ПКМ)
            if self._shots_on_target > 0 and (self._scoped_now or self._scope_held):
                self._begin_unscope(now)
            return
        if self._flick is not None or self._flick_pending:
            return
        if self._in_scope_zone(yaw_err, pitch_err, dist):
            self._scope_committed = True
        if not self._scope_committed:
            return
        if self._scoped_now:
            if plan["in_at"] is None:
                plan["in_at"] = now
            if not self._scope_held:
                self._press_scope()
            return
        plan["in_at"] = None
        if self._scope_held:
            # AWP после выстрела сам выходит — отпускаем и жмём снова
            if now >= self._scope_next_click:
                self._release_scope()
                self._scope_next_click = now + 0.04 + self._xx_u01() * 0.05
            return
        if now < self._scope_next_click:
            return
        self._press_scope()
        self._scope_next_click = now + 0.30 + self._xx_u01() * 0.08

    def _fire_on_oval(self, yaw_err: float, pitch_err: float, dist: float) -> bool:
        """Прицел внутри эллипса (не прямоугольника yaw×pitch) — ближе к центру, чем к краю."""
        cfg = self.cfg
        d = max(1.0, dist)
        frac = cfg.fire_oval_frac
        half_w = max(cfg.fire_tolerance_deg, math.degrees(math.atan2(cfg.fire_target_half_width, d)) * frac)
        half_h = max(cfg.fire_tolerance_pitch_deg, math.degrees(math.atan2(cfg.fire_target_up_margin, d)) * frac)
        ny = yaw_err / half_w
        np_ = pitch_err / half_h
        return ny * ny + np_ * np_ <= 1.0

    def _fire_step(
        self, now: float, target: dict, x: float, y: float, z: float, yaw_err: float, pitch_err: float
    ) -> bool:
        """Решение о выстреле: автомат держит ЛКМ, одиночный — тапает. True = новый выстрел."""
        cfg = self.cfg
        auto = bool(self.weapon.get("auto"))
        tx, ty, tz = self._target_aim_xyz(target, x, y)
        dist = math.hypot(tx - x, ty - y, tz - (z + self._eye_z))
        abort = (
            not cfg.fire_enabled
            or (cfg.shots_per_target > 0 and self._shots_on_target >= cfg.shots_per_target)
            or not self._weapon_ready()
            or not self._target_visible_now
            or self._flick is not None
            or self._flick_pending
            or now < self._correct_from
            or self._scope_blocks_fire(now, yaw_err, pitch_err, dist)
            or not self._target_can_fire(now, target)
            or not self._strafe_fire_ok
        )
        if abort:
            self._aimed_since = None
            self._release_fire()
            return False
        spray = self._should_spray(dist)
        # где прицел окажется, когда долетит уже отправленная мышь (память отстаёт на 1–3 тика):
        # для движущейся мишени «по памяти» точка кости всегда чуть в стороне и выстрел не наступает
        on_oval = self._fire_on_oval(
            yaw_err - self._pend_yaw.value(), pitch_err - self._pend_pitch.value(), dist
        )
        if self._shot_miss is None and not (spray and self._fire_held):
            self._shot_miss = self._xx_u01() < cfg.aim_miss_chance
        if self._shot_miss and not (spray and self._fire_held):
            lo, hi = cfg.aim_miss_u
            if abs(self._track_want[0]) + abs(self._track_want[1]) < 0.05:
                sign = 1.0 if self._xx_u01() < 0.5 else -1.0
                miss_u = lo + self._xx_u01() * (hi - lo)
                lead = math.degrees(math.atan2(miss_u, max(80.0, dist)))
                cap = 0.22 if self._scoped_now else 0.50
                lead = min(lead, cap)
                self._track_want = (sign * lead, (self._xx_u01() - 0.5) * 0.12)
            world = dist * math.tan(math.radians(min(89.0, math.hypot(yaw_err, pitch_err))))
            if on_oval or world < lo:
                self._aimed_since = None
                self._release_fire()
                return False
            if world > hi:
                self._aimed_since = None
                self._release_fire()
                return False
        elif not on_oval:
            if spray and self._fire_held:
                if max(abs(yaw_err), abs(pitch_err)) > cfg.rcs_break_deg:
                    self._aimed_since = None
                    self._release_fire()
                    return False
            else:
                self._aimed_since = None
                self._release_fire()
                return False
        if self._aimed_since is None:
            self._aimed_since = now
            wait = max(cfg.fire_confirm_s, self.rng.uniform(*cfg.fire_first_delay))
            self._next_fire_at = max(self._next_fire_at, now + wait)
            if now < self._next_fire_at:
                return False
        if auto and self._fire_held and not spray:
            self._release_fire()
            return False
        if spray:
            cycle = weapon_cycle_s(int(self.weapon.get("id") or 0), str(self.weapon.get("name") or ""))
            if not self._fire_held:
                if now < self._next_fire_at:
                    return False
                self.input.mouse_down("left")
                self._fire_held = True
                self._fire_release_at_tick = -1
                self._shots_on_target += 1
                self.shots_fired += 1
                self._next_fire_at = now + cycle
                self._on_combat_strafe_shot()
                return True
            if now >= self._next_fire_at:
                self._shots_on_target += 1
                self.shots_fired += 1
                self._next_fire_at = now + cycle
                self._on_combat_strafe_shot()
                return True
            return False
        if now < self._next_fire_at or self._fire_held:
            return False
        self.input.mouse_down("left")
        self._fire_held = True
        self._fire_release_at_tick = self._tick_no + cfg.fire_hold_ticks
        self._shots_on_target += 1
        self.shots_fired += 1
        self._next_fire_at = now + (self._auto_tap_wait() if auto else self._tap_wait())
        self._on_combat_strafe_shot()
        return True

    def _release_fire(self) -> None:
        if self._injecting():
            return
        was = self._fire_held
        if was:
            self.input.mouse_up("left")
        self._fire_held = False
        self._fire_release_at_tick = -1
        if was:
            self._begin_rcs_return()

    def _release_fire_if_due(self) -> None:
        if self._injecting():
            return
        if self._fire_release_at_tick >= 0 and self._tick_no >= self._fire_release_at_tick:
            self.input.mouse_up("left")
            self._fire_held = False
            self._fire_release_at_tick = -1

    def _sample_strafe_dist(self) -> float:
        lo, hi = self.cfg.combat_strafe_dist
        return lo + self._xx_u01() * (hi - lo)

    def _sample_combat_strafe(self, index: int) -> None:
        self._strafe_fire_ok = True
        if self._xx_u01() >= self.cfg.combat_strafe_chance:
            self._combat_strafe = None
            return
        self._combat_strafe = {
            "index": index,
            "side": -1 if self._xx_u01() < 0.5 else 1,
            "phase": "wait",
            "dist": self._sample_strafe_dist(),
            "ox": 0.0,
            "oy": 0.0,
            "t0": 0.0,
            "fired": False,
            "next_at": 0.0,
            "stand": False,
        }
        self.log(f"Стрейф A/D при огне ({self._combat_strafe['dist']:.0f}u)")

    def _on_combat_strafe_shot(self) -> None:
        self._shot_miss = None
        st = self._combat_strafe
        if st is None:
            return
        st["fired"] = True
        if bool(self.weapon.get("auto")) and self._long_spray is not False:
            st["stand"] = True
        st["next_at"] = self._now() + 0.05 + self._xx_u01() * 0.09

    def _combat_strafe_begin_go(self, st: dict, now: float, x: float, y: float) -> None:
        st["phase"] = "go"
        st["ox"], st["oy"] = x, y
        st["t0"] = now
        st["fired"] = False
        st["dist"] = self._sample_strafe_dist()

    def _combat_strafe_update(self, now: float, x: float, y: float, speed: float) -> None:
        """Шаг влево/вправо; стреляем только на контрстрейфе, когда скорость уже падает."""
        st = self._combat_strafe
        if st is None or self._unscoping():
            self._strafe_fire_ok = True
            return
        if self._flick is not None or self._flick_pending:
            self._strafe_fire_ok = False
            return
        if st["phase"] == "wait":
            # Сначала выстрел с места (как только встали), шаг в сторону — уже после него.
            # Иначе бот секунду-две ёрзает прицелом за собственным стрейфом до первого выстрела.
            self._strafe_fire_ok = speed < 80.0
            if st["fired"] and now >= st["next_at"]:
                self._combat_strafe_begin_go(st, now, x, y)
                self._strafe_fire_ok = False
            return
        if st.get("stand"):
            self._strafe_fire_ok = speed < 80.0
            return
        if st["phase"] == "go":
            traveled = math.hypot(x - st["ox"], y - st["oy"])
            stuck = speed < 38.0 and now - st["t0"] > 0.18
            timeout = now - st["t0"] > st["dist"] / 90.0 + 0.22
            if traveled >= st["dist"] or stuck or timeout:
                st["phase"] = "stop"
                st["t0"] = now
                st["fired"] = False
            self._strafe_fire_ok = False
            return
        # stop: контрстрейф, огонь когда почти остановились
        self._strafe_fire_ok = speed < 70.0
        if st["fired"] and now >= st["next_at"] and speed < 80.0:
            st["side"] = -st["side"]
            self._combat_strafe_begin_go(st, now, x, y)
            self._strafe_fire_ok = False

    def _combat_strafe_keys(self, vx: float, vy: float, speed: float, yaw_next: float) -> set[str] | None:
        st = self._combat_strafe
        if st is None or self._unscoping():
            return None
        if self._flick is not None or self._flick_pending:
            return self._counter_strafe_keys(vx, vy, speed, yaw_next)
        if st.get("stand") or st["phase"] == "stop":
            return self._counter_strafe_keys(vx, vy, speed, yaw_next)
        if st["phase"] == "go":
            return {"a"} if st["side"] < 0 else {"d"}
        return self._counter_strafe_keys(vx, vy, speed, yaw_next)

    def _counter_strafe_keys(self, vx: float, vy: float, speed: float, yaw_next: float) -> set[str]:
        """Погасить скорость: клавиша против направления движения (как counter-strafe игрока)."""
        if speed < 60.0:
            return set()
        vel_bearing = math.degrees(math.atan2(vy, vx))
        rel = shortest_yaw_err(vel_bearing + 180.0, yaw_next)
        _, keys = min(_KEY_DIRS, key=lambda kd: abs(shortest_yaw_err(kd[0], rel)))
        return set(keys)

    # ------------------------------------------------------------ main tick

    def tick(self) -> dict:
        cfg = self.cfg
        now = self._now()
        dt = 1.0 / cfg.tick_hz if self._last_t is None else max(0.005, min(0.1, now - self._last_t))
        self._last_t = now
        self._tick_no += 1
        self._release_fire_if_due()
        self._release_scope_if_due()

        p = self.reader.local_position()
        v = self.reader.local_velocity()
        ang = self.reader.local_eye_angles()
        # реальная высота глаз из памяти (стоя 64, присев 46, приземление меньше) — если ридер умеет
        vo = getattr(self.reader, "local_view_offset_z", None)
        if callable(vo):
            try:
                self._eye_z = float(vo()) or cfg.eye_height
            except Exception:
                self._eye_z = cfg.eye_height
        x, y, z = p.x, p.y, p.z
        speed = math.hypot(v.x, v.y)
        mem_yaw, mem_pitch = ang.y, ang.x
        self.weapon = self._read_weapon()
        self.life = self._read_life()
        self._mem_shots = self._read_shots_fired()
        self._scoped_now = self._read_scoped()
        if now >= self._enemies_next_poll:
            self._enemies_next_poll = now + 1.0 / max(1.0, cfg.enemies_poll_hz)
            self.all_enemies = self._read_enemies()
            for e in self.all_enemies:
                self._apply_los_visibility(e, x, y, z)
                e["_vis_pos"] = (round(x), round(y), round(z))
            self.visible_enemies = [e for e in self.all_enemies if e.get("visible")]
            alive_ids = {e["index"] for e in self.all_enemies}
            for e in self.all_enemies:
                idx = e["index"]
                self._note_vulnerability(e, now)
                around = bool(e.get("visible"))
                if around:
                    self._known[idx] = {**e, "seen": now, "last_visible": now}
                elif idx in self._known:
                    prev = self._known[idx]
                    self._known[idx] = {
                        **e,
                        "seen": prev.get("seen", now),
                        "last_visible": prev.get("last_visible", 0.0),
                    }
            forgotten: list[dict] = []
            for idx in list(self._known):
                if idx in alive_ids:
                    continue
                e = self._known[idx]
                forgotten.append(self._known.pop(idx))
                self._immune_seen.discard(idx)
                self._vuln_ready_at.pop(idx, None)
            if forgotten:
                self.known_targets = list(self._known.values())
                for e in forgotten:
                    self.log(f"Мишень {e.get('name') or e['index']} пропала — забыли, продолжаем исходную цель")
                if self._hunt and self._hunt["index"] in {e["index"] for e in forgotten}:
                    self._hunt = None
                    self._hunt_routed = None
                if self.route is not None and (
                    self._blockers_key() != self._route_blockers or self._standoff is not None
                ):
                    self._replan((x, y, z), "мишень пропала, продолжаем маршрут")
            self.known_targets = list(self._known.values())

        combat = (
            self.engaged
            or self._flick is not None
            or self._flick_pending
            or (self._target is not None)
        )
        self._sync_ladders(now, (x, y, z), combat)
        self._sync_doors(now, (x, y, z), combat)
        self._sync_breakables(now, (x, y, z), combat)
        if combat:
            if not self._cal_frozen:
                self._cal_win.clear()
                self._sign_flips_yaw = 0
                self._sign_flips_pitch = 0
                self._cal_frozen = True
        else:
            self._cal_frozen = False
            self._calibrate_mouse(mem_yaw, mem_pitch)
        self._detect_yaw_offset(mem_yaw, v.x, v.y, speed)
        if self._prev_mem is not None:
            # что из отправленного мышью уже дошло до игры (углы меняет только наша мышь)
            self._pend_yaw.observed(self._tick_no, shortest_yaw_err(mem_yaw, self._prev_mem[0]))
            self._pend_pitch.observed(self._tick_no, mem_pitch - self._prev_mem[1])
        self._prev_mem = (mem_yaw, mem_pitch)
        yaw = shortest_yaw_err(mem_yaw + self.yaw_offset, 0.0)

        info: dict = {
            "pos": (round(x), round(y), round(z)),
            "speed": round(speed),
            "yaw": round(yaw, 1),
            "pitch": round(mem_pitch, 1),
            "visible_enemies": self.visible_enemies,
            "all_enemies": self.all_enemies,
            "ladders": self.ladders,
            "doors": self.doors,
            "breakables": self.breakables,
            "weapon": self.weapon,
            "life": self.life,
        }

        died, respawned = self._life_events()
        if died:
            self._on_death()
        if not self.life.get("alive", True) and not respawned:
            if not self._dead:
                self._on_death()
            self.release()
            self._sent_px = (0, 0)
            info["action"] = "dead"
            return info
        if self._dead or respawned:
            self._on_respawn((x, y, z))

        if self.done:
            self.release()
            self._sent_px = (0, 0)
            info["action"] = "done"
            return info

        if now < self._pause_until:
            self.release()
            self._send_mouse(*self._camera_step(0.0, cfg.pitch_base - mem_pitch, dt, 0.0))
            info["action"] = "pause"
            return info

        # мягкое «покачивание» мышью (синус вправо-влево, суммарно ноль): узнаём реальную
        # чувствительность и знак до того, как начнём крутить камеру всерьёз — без рывка на старте
        if self._precal_left > 0 and cfg.calibrate_online:
            i = cfg.precal_ticks - self._precal_left
            self._precal_left -= 1
            # трапеция: 2 тика разгон, ровное удержание (калибровка берёт только ровные px), 2 тика спад
            ramp = 2
            if i < ramp:
                k = (i + 1) / (ramp + 1)
            elif i >= cfg.precal_ticks - ramp:
                k = (cfg.precal_ticks - i) / (ramp + 1)
            else:
                k = 1.0
            px = int(round(cfg.precal_px * k))
            self.release()
            if px:
                self.input.mouse_move(px, 0)
            self._sent_px = (px, 0)
            info["action"] = "precal"
            return info

        if self.route is None:
            self.plan((x, y, z))
        route = self.route
        assert route is not None

        # --- прогресс по маршруту
        self.s, off = route.project(x, y, self.s, z=z)
        gx, gy, gz = route.goal
        goal_d = math.hypot(gx - x, gy - y)
        if off > cfg.off_route_replan and self._ladder is None:
            self._replan((x, y, z), f"ушли с маршрута на {off:.0f}u")
            route = self.route
            assert route is not None
            self.s, off = route.project(x, y, None, z=z)

        # детектор отсутствия прогресса: s не растёт больше N секунд (бот стоит в стороне от маршрута)
        if self.s - self._progress_s >= 32.0:
            self._progress_s = self.s
            self._progress_at = now
            self._noprog_replans = 0
        elif (
            self._ladder is None
            and now - self._progress_at > 2.5
            and goal_d > cfg.arrive_radius + 24.0
        ):
            self._progress_at = now
            self._noprog_replans += 1
            new_goal = self._noprog_replans >= 2  # повторный тупик — другая цель
            self._replan((x, y, z), f"нет прогресса по маршруту (#{self._noprog_replans})", new_goal=new_goal)
            route = self.route
            assert route is not None
            self.s, off = route.project(x, y, None, z=z)
            self._progress_s = self.s
            if new_goal:
                self._noprog_replans = 0

        # мишень встала на маршруте впереди (и маршрут строился без неё) → обойти заранее
        if now >= self._block_check_at:
            self._block_check_at = now + 0.25
            blocker = self._target_on_route(route)
            if (
                blocker is not None
                and blocker.get("visible")
                and self._blockers_key() != self._route_blockers
                and self._ladder is None
            ):
                self._replan((x, y, z), f"мишень {blocker.get('name') or blocker['index']} на пути")
                route = self.route
                assert route is not None
                self.s, off = route.project(x, y, None, z=z)

        # скрытая мишень: заход в точку с LOS (пик), исходную дальнюю не бросаем
        if now >= self._hunt_check_at and self._ladder is None:
            self._hunt_check_at = now + cfg.hunt_check_s
            prev_via = (
                None
                if self._hunt is None
                else (self._hunt.get("via"), self._hunt.get("peek_id"), self._hunt.get("index"))
            )
            self._update_hunt((x, y, z), route, now)
            hunt = self._hunt
            if (
                hunt
                and hunt.get("via")
                and hunt.get("peek_id") is not None
                and hunt.get("peek_id") != self._hunt_routed
            ):
                try:
                    pts = plan_route(
                        self.map, (x, y, z), hunt["peek_id"], cfg.route, self._blocked
                    )
                    self.route = Route(pts)
                    self.s = 0.0
                    self._hunt_routed = hunt["peek_id"]
                    route = self.route
                    self.s, off = route.project(x, y, None, z=z)
                    pk = hunt["peek"]
                    self.log(
                        f"Заход на позицию для выстрела @ ({pk[0]:.0f},{pk[1]:.0f}) "
                        f"по {hunt['index']}, цель маршрута та же"
                    )
                except RuntimeError:
                    self._hunt_routed = None
            elif prev_via and prev_via[0] and (hunt is None or not hunt.get("via")):
                self._hunt_routed = None
                still = next((t for t in self.known_targets if t["index"] == prev_via[2]), None)
                if still is not None and still.get("visible"):
                    pass  # на пике, мишень видна — стреляем, маршрут не трогаем
                elif hunt is not None and not hunt.get("via"):
                    pass  # выход на LOS уже на текущем пути
                else:
                    self._replan((x, y, z), "пик больше не нужен, продолжаем маршрут")
                    route = self.route
                    assert route is not None
                    self.s, off = route.project(x, y, None, z=z)

        hold = False
        if self._standoff is not None and goal_d <= cfg.arrive_radius + 16.0:
            # дошли до дистанции выстрела: стоим и ждём, пока мишень пропадёт. Не считаем это финишем.
            if self._standoff_since is None:
                self._standoff_since = now
            vanished = self._standoff not in self._known
            idle_too_long = self._target is None and now - self._standoff_since > cfg.standoff_wait
            if idle_too_long:
                self._given_up[self._standoff] = now + cfg.target_done_cooldown
            gave_up = self._given_up.get(self._standoff, 0.0) > now
            if vanished:
                self._replan((x, y, z), "мишень пропала, продолжаем маршрут")
                route = self.route
                assert route is not None
                self.s, off = route.project(x, y, None, z=z)
            elif gave_up:
                self._replan((x, y, z), "мишень не уходит → другая цель", new_goal=True)
                route = self.route
                assert route is not None
                self.s, off = route.project(x, y, None, z=z)
            else:
                hold = True
        elif self._standoff is not None and self._given_up.get(self._standoff, 0.0) > now:
            self._replan((x, y, z), "мишень не уходит → другая цель", new_goal=True)
            route = self.route
            assert route is not None
            self.s, off = route.project(x, y, None, z=z)

        dest_id = self._intended_goal_id if self._intended_goal_id is not None else self.goal_id
        dest = self.map.nodes[dest_id] if dest_id is not None else None
        dest_d = math.hypot(dest.x - x, dest.y - y) if dest is not None else goal_d
        dest_z = dest.z if dest is not None else gz
        if (
            cfg.loop
            and not (self._hunt and self._hunt.get("via"))
            and self.route is not None
            and self.route.remaining(self.s) <= cfg.prefetch_dist
        ):
            self._kick_prefetch()
        hunt = self._hunt
        if (
            hunt
            and hunt.get("via")
            and goal_d <= cfg.arrive_radius + 24.0
            and dest_d > cfg.arrive_radius + 40.0
        ):
            # дошли до пика, исходная цель ещё далеко: ждём появления, не «финиш»
            if not self._hunt_via_since:
                self._hunt_via_since = now
            e = next((t for t in self.known_targets if t["index"] == hunt["index"]), None)
            fx, fy, fz = self._enemy_feet(e) if e else (0.0, 0.0, 0.0)
            if e is None:
                self._hunt = None
                self._hunt_routed = None
                self._replan((x, y, z), "пик: мишени нет, продолжаем маршрут")
                route = self.route
                assert route is not None
                self.s, off = route.project(x, y, None, z=z)
            elif not e.get("visible") and not self._los_clear(x, y, z, fx, fy, fz):
                if now - self._hunt_via_since > 1.6:
                    self._hunt = None
                    self._hunt_routed = None
                    self._replan((x, y, z), "пик без обзора, продолжаем маршрут")
                    route = self.route
                    assert route is not None
                    self.s, off = route.project(x, y, None, z=z)
                else:
                    hold = True
            else:
                hold = True
        else:
            self._hunt_via_since = 0.0

        if (
            not hold
            and self._standoff is None
            and self._ladder is None
            and dest_d <= cfg.arrive_radius
            and abs(dest_z - z) <= 10.0
            and (
                (here := self.map.layer_at(x, y, z, self.map.cfg.step_height + 2.0)) is not None
                and abs(here.z - dest_z) <= 8.0
            )
        ):
            self.log(f"Цель достигнута ({x:.0f},{y:.0f},{z:.0f})")
            if cfg.loop:
                if self._apply_next_route((x, y, z)):
                    route = self.route
                    assert route is not None
                    self.s, off = route.project(x, y, None, z=z)
                    gx, gy, gz = route.goal
                    goal_d = math.hypot(gx - x, gy - y)
                    dest_id = self._intended_goal_id if self._intended_goal_id is not None else self.goal_id
                    dest = self.map.nodes[dest_id] if dest_id is not None else None
                    dest_d = math.hypot(dest.x - x, dest.y - y) if dest is not None else goal_d
                    dest_z = dest.z if dest is not None else gz
                else:
                    self.release()
                    info["action"] = "arrived"
                    self._sent_px = (0, 0)
                    return info
            else:
                self.release()
                self.done = True
                info["action"] = "done"
                self._sent_px = (0, 0)
                return info

        # --- цели камеры и ног
        if self._cam_warm_t0 is None:
            self._cam_warm_t0 = now
            self._look_yaw_f = yaw
        look_d = max(cfg.look_ahead_min, min(cfg.look_ahead_max, cfg.look_ahead_min + speed * cfg.look_ahead_speed_gain))
        look_d *= 1.0 + 0.35 * self._cam_pace   # тот же «темп»: чуть ближе/дальше точка взгляда
        look_d = min(look_d, route.remaining(self.s) + 40.0)   # у финиша смотрим на цель
        warm_k = 1.0
        if cfg.cam_warm_s > 0:
            wt = (now - self._cam_warm_t0) / cfg.cam_warm_s
            if wt < 1.0:
                warm_k = wt * wt * (3.0 - 2.0 * wt)
                # сначала смотрим ближе по маршруту — точка взгляда не прыгает на 120u вбок
                look_d *= warm_k
        lx, ly, lz = route.point_at(self.s + look_d)
        cx_, cy_, cz_ = self._carrot((x, y, z), route)
        self._door_ahead = self._closed_door_ahead(route, x, y, z)
        if self._ladder is not None:
            ph = self._ladder.get("phase")
            if self.firing and ph in ("approach", "mount") and not self._read_on_ladder():
                self._ladder = None
            else:
                self._update_ladder(now, route, x, y, z)
        elif not self.firing:
            self._update_ladder(now, route, x, y, z)

        # медленное блуждание прицела ±wander
        # блуждание взгляда: плавный дрейф (случайная скорость с инерцией), а не белый шум на точку
        self._wander_vel = self._wander_vel * 0.96 + self.rng.gauss(0.0, 0.35)
        self._wander += self._wander_vel * dt - self._wander * 0.02
        self._wander = max(-cfg.yaw_wander, min(cfg.yaw_wander, self._wander))

        # --- мишень? камера уходит на неё, ноги продолжают маршрут
        route_bearing = yaw_to_target(x, y, cx_, cy_) if math.hypot(cx_ - x, cy_ - y) > 4.0 else None
        target = self._update_target(now, x, y, z, yaw, route_bearing, speed)
        was_engaged = self.engaged
        self.engaged = target is not None
        if self.engaged and not was_engaged:
            self.targets_engaged += 1
            self.log(f"Мишень {target.get('name') or target['index']} @ {target['position']} — взгляд на неё")
        elif was_engaged and not self.engaged:
            self.log("Мишень потеряна — взгляд обратно на маршрут")
            self._brake_look_rate(speed)
            self._hunt_look_hold = None
            self._hunt_look_until = 0.0

        break_target = None
        if target is not None or self._ladder is not None:
            self._break_aim = None
        else:
            break_target = self._update_break_aim(route, x, y, z, yaw, mem_pitch, now)

        hunt_look = None
        rcs_y = rcs_p = 0.0
        if target is not None:
            self.engaged_ticks += 1
            aim_yaw, aim_pitch = self._aim_angles(x, y, z, target)
            if self._rcs_active():
                rcs_y, rcs_p = self._rcs_offset(now)
            look_yaw = aim_yaw + rcs_y
            pitch_target = max(-60.0, min(60.0, aim_pitch + rcs_p))
            lx, ly = self._target_aim_xyz(target, x, y)[:2]
            look_mode = "aim"
        elif break_target is not None:
            self._reset_aim_bone()
            aim_yaw, aim_pitch = self._aim_angles(x, y, z, break_target)
            if self._rcs_active():
                rcs_y, rcs_p = self._rcs_offset(now)
            look_yaw = aim_yaw + rcs_y
            pitch_target = max(-60.0, min(60.0, aim_pitch + rcs_p))
            look_mode = "aim"
            rate_cap = None
            gain = None
        else:
            fb = self._look_yaw_f if self._look_yaw_f is not None else yaw
            look_yaw = self._yaw_to_look_point(x, y, lx, ly, fb) + self._wander
            slope = route.slope_ahead(self.s, 140.0)
            pitch_target = cfg.pitch_base - slope * cfg.pitch_slope_gain
            pitch_target = max(-25.0, min(30.0, pitch_target))
            hunt_look = self._hunt_look(x, y, z, route)
            if hunt_look is None and now < self._hunt_look_until and self._hunt_look_hold is not None:
                hunt_look = self._hunt_look_hold
            if self._ladder is not None:
                look_yaw, pitch_target = self._ladder_look(x, y, z)
                look_mode = "ladder"
                self._hunt_look_hold = None
            else:
                door_look = self._door_look(x, y, z)
                if door_look is not None:
                    look_yaw, pitch_target = door_look
                    look_mode = "door"
                    self._hunt_look_hold = None
                elif hunt_look is not None:
                    self._hunt_look_hold = hunt_look
                    self._hunt_look_until = now + 0.3
                    look_yaw, pitch_target = hunt_look
                    pitch_target = max(-25.0, min(30.0, pitch_target))
                    look_mode = "hunt"
                else:
                    self._hunt_look_hold = None
                    look_mode = "route"
            rate_cap = None
            gain = None
        look_yaw = self._filter_look_yaw(look_yaw, look_mode, yaw, now)
        yaw_err = shortest_yaw_err(look_yaw, yaw)
        pitch_err = pitch_target - mem_pitch
        if warm_k < 1.0 and look_mode != "aim":
            pitch_err *= warm_k

        # выстрел решаем по прицелу «как есть» до сдвига мыши на этом тике
        # (прицел без дрожания: yaw_err выше содержит jitter, для решения о выстреле берём чистую ошибку)
        self._habit_tick(now, x, y, z)
        shot = False
        aim_phase = ""
        if target is not None:
            if self._fire_held and self.weapon.get("auto"):
                fire_yaw_err, fire_pitch_err = yaw_err, pitch_err
            else:
                fire_yaw_err = shortest_yaw_err(aim_yaw, yaw)
                fire_pitch_err = aim_pitch - mem_pitch
            tx, ty, tz = self._target_aim_xyz(target, x, y)
            aim_dist = math.hypot(tx - x, ty - y, tz - (z + self._eye_z))
            self._combat_strafe_update(now, x, y, speed)
            if not self._unscoping():
                self._scope_step(now, fire_yaw_err, fire_pitch_err, aim_dist)
            shot = self._fire_step(now, target, x, y, z, fire_yaw_err, fire_pitch_err)
            if shot and self._shots_on_target == 1:
                spray = self._long_spray is not False and bool(self.weapon.get("auto"))
                mode = "зажим" if spray else "тап"
                wname = self.weapon.get("name") or "?"
                scope_txt = ""
                plan = self._scope_plan
                if plan is not None:
                    scope_txt = " ноускоуп" if plan.get("noscope") else " со скоупа"
                self.log(
                    f"Огонь ({mode}, {wname}{scope_txt}) по {target.get('name') or target['index']} через "
                    f"{now - self._target_seen_at:.2f}s (ошибка {fire_yaw_err:+.1f}°/{fire_pitch_err:+.1f}°)"
                )
            # режим стрельбы: заметили мишень → останавливаемся, разворачиваемся, стреляем, идём дальше
            self.firing = cfg.fire_enabled
            tx, ty, _ = target["position"]
            aim_dist = math.hypot(tx - x, ty - y)
            self._update_aim_chase(target, x, y, dt, now)
            if self._rcs_return:
                look_yaw = aim_yaw
                pitch_target = max(-60.0, min(60.0, aim_pitch))
                yaw_err = shortest_yaw_err(look_yaw, yaw)
                pitch_err = pitch_target - mem_pitch
            # рука: сначала как на маршруте (та же скорость), затем флик если угол ≥ 30°
            d_yaw, d_pitch, aim_phase = self._aim_step(
                now, dt, yaw_err, pitch_err, aim_yaw, aim_pitch, speed, aim_dist
            )
            if aim_phase == "flick":
                max_px = cfg.aim_max_px_per_tick
            elif self._chase_on:
                max_px = int(cfg.aim_max_px_per_tick * 0.75)
            else:
                max_px = cfg.max_px_per_tick * 2
            dx, dy = self._send_mouse(d_yaw, d_pitch, max_px)
        elif break_target is not None:
            self._target_visible_now = True
            self._reset_aim_bone()
            if self._fire_held and self.weapon.get("auto"):
                fire_yaw_err, fire_pitch_err = yaw_err, pitch_err
            else:
                fire_yaw_err = shortest_yaw_err(aim_yaw, yaw)
                fire_pitch_err = aim_pitch - mem_pitch
            tx, ty, tz = self._target_aim_xyz(break_target, x, y)
            aim_dist = math.hypot(tx - x, ty - y, tz - (z + self._eye_z))
            if not self._unscoping():
                self._scope_step(now, fire_yaw_err, fire_pitch_err, aim_dist)
            shot = self._fire_step(now, break_target, x, y, z, fire_yaw_err, fire_pitch_err)
            if shot and self._shots_on_target == 1:
                self.log(f"Огонь по решётке ({self.weapon.get('name') or '?'})")
            self.firing = cfg.fire_enabled
            self._update_aim_chase(break_target, x, y, dt, now)
            d_yaw, d_pitch, aim_phase = self._aim_step(
                now, dt, yaw_err, pitch_err, aim_yaw, aim_pitch, speed, aim_dist
            )
            if aim_phase == "flick":
                max_px = cfg.aim_max_px_per_tick
            elif self._chase_on:
                max_px = int(cfg.aim_max_px_per_tick * 0.75)
            else:
                max_px = cfg.max_px_per_tick * 2
            dx, dy = self._send_mouse(d_yaw, d_pitch, max_px)
        else:
            self.firing = False
            self._release_fire()
            self._combat_strafe = None
            self._strafe_fire_ok = True
            self._update_aim_chase(None, x, y, dt, now)
            self._scope_committed = False
            if not self._unscoping() and (self._scoped_now or self._scope_held):
                self._begin_unscope(now)
            rcs_back = self._rcs_return_going(now, yaw_err, pitch_err)
            if rcs_back:
                d_yaw, d_pitch = self._rcs_return_step(yaw_err, pitch_err, dt)
            else:
                pitch_rate = None
                if (
                    self._ladder is not None
                    and not self._ladder.get("up")
                    and self._ladder.get("phase") not in ("approach", "exit")
                ):
                    pitch_rate = 360.0
                d_yaw, d_pitch = self._camera_step(
                    yaw_err, pitch_err, dt, speed, rate_cap, gain, pitch_rate_max=pitch_rate
                )
            # только маршрут: иначе потолок 160 px режет новый yaw_rate_max
            nav_px = None if rcs_back else int(cfg.max_px_per_tick * 1.75)
            dx, dy = self._send_mouse(d_yaw, d_pitch, nav_px)

        self._unscope_tick(now)

        # клавиши считаем относительно yaw, каким он будет после этого шага мыши
        yaw_next = yaw + d_yaw
        move_bearing = self._steer_bearing(x, y, v.x, v.y, cx_, cy_)
        rel = shortest_yaw_err(move_bearing, yaw_next)
        climbing = self._ladder is not None and (
            self._read_on_ladder() or self._ladder.get("phase") in ("climb", "mount", "exit")
        )
        if self._ladder is not None and (not self.firing or climbing):
            keys = self._ladder_keys(x, y, z, yaw_next, speed)
        elif self.firing:
            strafe_keys = self._combat_strafe_keys(v.x, v.y, speed, yaw_next)
            if strafe_keys is not None:
                keys = strafe_keys
            elif cfg.fire_stop_moving:
                keys = self._counter_strafe_keys(v.x, v.y, speed, yaw_next)
            else:
                peek_keys = self._peek_move_keys(x, y, look_yaw)
                if peek_keys is not None:
                    keys = peek_keys
                else:
                    keys = self._choose_keys(rel, yaw_err, speed, engaged=self.engaged)
        else:
            peek_keys = self._peek_move_keys(x, y, look_yaw)
            if peek_keys is not None:
                keys = peek_keys
            else:
                keys = self._choose_keys(rel, yaw_err, speed, engaged=self.engaged)

        # торможение у цели: отпустить клавиши чуть раньше, трение остановит внутри arrive_radius
        brake_d = cfg.arrive_radius * 0.5 + speed * cfg.brake_time
        if self._brake and speed < 40.0:
            self._brake = False            # остановились, но не дошли — дошагиваем
        if goal_d < brake_d and speed > 60.0 and not cfg.loop:
            self._brake = True
        if (self._brake or hold) and self._ladder is None:
            keys = set()

        # прыжок по маршруту: 57u обычный, 64u с приседом; выше — не прыгаем, обходим
        climb = None
        if (
            self._ladder is None
            and speed > 28.0
            and now - self._last_jump_at >= 0.55
            and math.hypot(cx_ - x, cy_ - y) < 64.0
        ):
            climb = self._climb_needed(x, y, z, cx_, cy_, cz_)
            if climb:
                self._do_climb(climb, now)

        if self._ladder is None:
            override = self._handle_stuck(now, (x, y, z), speed, keys, rel)
            if override is not None:
                keys = override

        if self._want_crouch(x, y, z, route, cx_, cy_, cz_):
            self._crouch_until = now + 0.18
        if now < self._crouch_until and self._ladder is None:
            keys.add("ctrl")
        self._door_use(now, x, y)

        if frozenset(keys) == frozenset(self._keys):
            self._same_keys_ticks += 1
        else:
            self._same_keys_ticks = 0
            self._keys_since = now
        self._set_keys(self._merge_extra_keys(keys, now))

        if (self._stuck_level or self._target_bumps) and speed > 120.0 and now >= self._override_until:
            self._stuck_level = 0
            self._stuck_plan = []
            if not self._near_target(x, y, margin=cfg.target_avoid_dist):
                self._target_bumps = 0

        info.update(
            {
                "action": "fire" if self.firing else ("move" if keys else ("turn" if abs(yaw_err) > 5 else "idle")),
                "keys": "".join(sorted(k.upper() for k in keys)) or "-",
                "s": round(self.s),
                "route_len": round(route.length),
                "off_route": round(off, 1),
                "goal_d": round(goal_d),
                "look": (round(lx), round(ly)),
                "carrot": (round(cx_), round(cy_)),
                "yaw_err": round(yaw_err, 1),
                "rel": round(rel, 1),
                "yaw_rate": round(self._yaw_rate),
                "mouse": (dx, dy),
                "pitch_t": round(pitch_target, 1),
                "yaw_per_px": round(cfg.yaw_per_px, 4),
                "yaw_offset": round(self.yaw_offset),
                "target": target["position"] if target is not None else None,
                "firing": self.firing,
                "aim_phase": aim_phase,
                "shot": shot,
                "shots_on_target": self._shots_on_target if target is not None else 0,
                "fire_held": self._fire_held,
            }
        )
        return info

    def run(
        self,
        duration: float | None = None,
        *,
        should_stop: Callable[[], bool] | None = None,
    ) -> str:
        """Крутить тики, пока не кончится время / маршрут, или should_stop() не станет True.

        Возвращает 'limit' | 'done' | 'stop'.
        """
        duration = duration if duration is not None else self.cfg.duration
        interval = 1.0 / self.cfg.tick_hz
        t0 = time.monotonic()
        n = 0
        reason = "done"
        try:
            while True:
                if should_stop is not None and should_stop():
                    reason = "stop"
                    break
                if duration and time.monotonic() - t0 > duration:
                    print(f"Лимит {duration:.0f}s")
                    reason = "limit"
                    break
                info = self.tick()
                if info.get("action") == "done":
                    reason = "done"
                    break
                n += 1
                if self.cfg.verbose_log and n % int(self.cfg.tick_hz / 4) == 0:
                    enemies = info.get("visible_enemies") or []
                    enemy_txt = ""
                    if enemies:
                        enemy_txt = " enemies=" + ",".join(
                            f"{e.get('name') or e['index']}@{e['position']}" for e in enemies
                        )
                    w = info.get("weapon") or {}
                    fire_ru = {"auto": "зажим", "tap": "тап", "none": "-"}.get(w.get("fire"), "-")
                    wpn_txt = (
                        f" wpn={w.get('name') or '?'}({w.get('clip', 0)},{fire_ru})"
                        if w.get("name") else ""
                    )
                    print(
                        f"[{info.get('action')}] keys={info.get('keys')} spd={info.get('speed')} "
                        f"pos={info.get('pos')} s={info.get('s')}/{info.get('route_len')} "
                        f"off={info.get('off_route')} goal_d={info.get('goal_d')} "
                        f"yaw={info.get('yaw')} err={info.get('yaw_err')} rel={info.get('rel')} "
                        f"mouse={info.get('mouse')} k={info.get('yaw_per_px')} off_yaw={info.get('yaw_offset')}"
                        f"{wpn_txt}{enemy_txt}"
                    )
                # держим ровный такт
                next_t = t0 + (n + 1) * interval
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
        finally:
            self.release()
            self.input.release_all()
        return reason
