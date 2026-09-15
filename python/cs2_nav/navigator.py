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
import time
from dataclasses import dataclass, field
from typing import Callable

from cs2_control.input import GameInput
from cs2_nav.coord_map import CoordinateMap
from cs2_nav.pathfinder import (
    Obstacle,
    RouteConfig,
    blocked_by_obstacles,
    pick_farthest_goal,
    plan_route,
    segment_clear,
)
from cs2_nav.route import Route
from cs2_reader.aim import shortest_yaw_err, yaw_to_target

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


@dataclass
class NavConfig:
    tick_hz: float = 40.0
    # --- камера
    look_ahead_min: float = 120.0
    look_ahead_max: float = 260.0
    look_ahead_speed_gain: float = 0.55      # + скорость × gain
    yaw_gain: float = 4.5                    # 1/с: скорость поворота = gain × ошибка
    yaw_rate_max: float = 260.0              # °/с стоя
    yaw_rate_max_moving: float = 190.0       # °/с на бегу
    yaw_accel_max: float = 1400.0            # °/с²
    yaw_jerk_max: float = 14000.0            # °/с³: ускорение набирается за ~0.1 с — S-кривая, без рывков
    yaw_deadband: float = 0.6                # мягкая зона: скорость ∝ err·|err|/(|err|+deadband)
    yaw_wander: float = 2.5                  # ° медленное блуждание прицела
    cam_pace_sigma: float = 0.028            # AR(1): насколько «темп» поворота камеры на маршруте гуляет
    cam_pace_range: float = 0.15             # ±15 % к потолку и усилению поворота (не одна скорость)
    # --- «рука»: плавность и кривизна (ветер как в WindMouse: инерционный случайный вектор скорости)
    hand_wind_sigma: float = 3.0             # °/с: σ ветра при панорамировании (стационарное)
    hand_wind_decay: float = 0.88            # за тик: инерция ветра (WindMouse: W/√3 за шаг)
    tremor_sigma: float = 0.0                # °/тик: σ тремора при удержании (0 = прицел стоит; ≥0.01 — видно ±1 px)
    flick_curve_frac: float = 0.025          # доля угла: боковой изгиб траектории флика (лёгкая дуга, ~2%)
    flick_wind_frac: float = 0.015           # доля длины маха: амплитуда волны положения посреди флика (≤1.5°)
    pitch_gain: float = 2.5
    pitch_rate_max: float = 50.0
    pitch_base: float = 3.0                  # чуть ниже горизонта, как у игрока
    pitch_slope_gain: float = 0.5            # смотреть вниз на спуске / вверх на подъёме
    # --- движение
    carrot_dist: float = 60.0
    max_speed: float = 250.0                 # u/s для расчёта steering
    steer_damping: float = 0.8               # насколько гасим текущую скорость (0 = чистый pursuit)
    steer_smooth: float = 0.55               # EMA вектора руления (0 = без сглаживания)
    key_hysteresis_deg: float = 9.0
    key_min_hold: float = 0.12               # с: не дёргать клавиши чаще
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
    precal_ticks: int = 18                   # плавный поворот мышью в начале (трапеция, 0.45 с) — стартовая калибровка
    precal_px: int = 16                      # px за тик на удержании (≈ 10° при 0.038°/px)
    # --- видимые мишени (nav.visible_enemies)
    engage_targets: bool = True              # смотреть на видимую мишень, продолжая идти по маршруту
    enemies_poll_hz: float = 20.0            # как часто читать список врагов из памяти (перебор 64 сущностей)
    target_memory: float = 45.0              # с: помним, где стояла мишень, когда её уже не видно (для обхода)
    standoff_dist: float = 110.0             # u: если мишень перекрыла проход — подойти на столько и стрелять с места
    standoff_wait: float = 4.0               # с: стоим на дистанции без мишени в прицеле → бросаем, другая цель
    target_forget_polls: int = 5             # опросов подряд «место видно, мишени нет» → забываем (пропала/убита)
    target_max_dist: float = 1400.0          # дальше — игнорируем
    target_reaction: tuple[float, float] = (0.12, 0.25)   # с: реакция человека до переноса взгляда
    target_hold: float = 0.35                # с: после потери мишени ещё смотрим туда, где она была
    target_yaw_rate_max: float = 330.0       # °/с: (режим без стрельбы) наводка резче панорамы
    target_yaw_gain: float = 9.0             # 1/с: (режим без стрельбы)
    target_aim_jitter: float = 0.15          # ° дрожание прицела на мишени
    target_switch_margin: float = 15.0       # °: новая мишень должна быть заметно ближе к прицелу, чтобы переключиться
    target_engage_max: tuple[float, float] = (6.0, 8.0)   # с: предохранитель — мишень не пропадает / не навелись → идём дальше
    target_cooldown: float = 6.0             # с: после этого мишень снова «интересна» (пропала / бросили)
    target_done_cooldown: float = 40.0       # с: мишень, которую расстреляли, а она не пропала — не трогаем долго
    target_behind_deg: float = 125.0         # мишень позади по ходу движения → не бежим спиной, отпускаем
    # --- наводка «как рука»: баллистический флик (minimum-jerk) + короткая коррекция по зрению
    flick_ramp: float = 0.11                 # с: в начале флика скорость набирается (smoothstep), не скачком
    flick_min_deg: float = 4.0               # меньше — без флика, сразу коррекция
    flick_base_time: float = 0.09            # с: длительность флика = base + per_deg × угол (закон Фиттса)
    flick_per_deg: float = 0.0021            # с/°: 30° ≈ 0.15 с, 90° ≈ 0.28 с, 180° ≈ 0.47 с
    flick_rate_max: float = 720.0            # °/с: пик скорости флика (min-jerk: 1.875 × угол / T)
    flick_undershoot: float = 0.03           # доля угла: рука обычно чуть не доводит
    flick_error_frac: float = 0.025          # доля угла: разброс конца флика (σ), + flick_error_min
    flick_error_min: float = 0.35            # °
    aim_correct_delay: tuple[float, float] = (0.04, 0.09)  # с: зрительная задержка перед коррекцией
    aim_correct_gain: float = 18.0           # 1/с: коррекция — экспоненциальное «дожатие» (τ ≈ 55 мс)
    aim_correct_rate: float = 260.0          # °/с: предел скорости коррекции
    aim_max_px_per_tick: int = 480           # px: флик — реальный быстрый мах мышью
    # --- стрельба
    fire_enabled: bool = True
    shots_per_target: int = 0                # выстрелов по мишени; 0 = стреляем, пока мишень не пропадёт
    fire_tolerance_deg: float = 1.0          # минимальный допуск прицела по yaw (°)
    fire_tolerance_pitch_deg: float = 0.7    # минимальный допуск по pitch (°) — по вертикали запас меньше
    fire_target_half_width: float = 14.0     # u: полуширина мишени → допуск по yaw растёт для близких
    fire_target_up_margin: float = 12.0      # u: от точки прицеливания до верха головы → допуск по pitch
    target_aim_z: float = 56.0               # u над ногами мишени: верх груди/шея (голова 64–72), а не глаза —
                                             # ошибка вверх остаётся в голове, вниз — в теле
    fire_interval: tuple[float, float] = (0.14, 0.22)   # с между выстрелами (tap-fire)
    fire_first_delay: tuple[float, float] = (0.05, 0.12) # с после наводки до первого выстрела
    fire_hold_ticks: int = 1                 # тиков держим ЛКМ (≥1 тик игры)
    fire_stop_moving: bool = True            # стоим на месте во время стрельбы (точность как у игрока)
    eye_height: float = 64.0                 # z глаз над ногами (стоя)
    target_body_radius: float = 16.0         # мишень — цилиндр, как игрок
    target_avoid_dist: float = 72.0          # начинаем обходить мишень на теле маршрута с этого расстояния
    # --- цель
    loop: bool = False                       # достигли — выбрать новую дальнюю и идти
    pause_at_goal: tuple[float, float] = (0.7, 1.8)
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
        self._prev_mem = None  # (yaw, pitch)
        self._cal_win: list[tuple[int, int, float, float]] = []
        self._mouse_dead_ticks = 0
        self._sign_flips_yaw = 0
        self._sign_flips_pitch = 0
        self.yaw_offset = 0.0
        self._offset_samples: list[float] = []
        self._same_keys_ticks = 0
        self._stuck_since: float | None = None
        self._stuck_level = 0
        self._override_until = 0.0
        self._override_keys: set[str] = set()
        self._pause_until = 0.0
        self._precal_left = self.cfg.precal_ticks
        self.replans = 0
        self.stucks = 0
        self.events: list[str] = []
        self._brake = False
        self.visible_enemies: list[dict] = []
        self.known_targets: list[dict] = []      # видимые + недавно виденные (статичные макеты не уходят)
        self._known: dict[int, dict] = {}
        # состояние наводки на мишень
        self._target: dict | None = None         # текущая мишень (запись из visible_enemies)
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
        # соглашение yaw подтверждено движением (до этого не стреляем — прицел может быть смещён)
        self.yaw_verified = not self.cfg.detect_yaw_offset
        # стрельба
        self._shots_on_target = 0                # выстрелов по текущей мишени
        self._next_fire_at = 0.0                 # раньше этого не стреляем
        self._aimed_since: float | None = None   # с какого момента прицел на мишени
        self._fire_release_at_tick = -1          # на каком тике отпустить ЛКМ
        self._tick_no = 0
        self._target_visible_now = False         # мишень в списке видимых на этом тике (стреляем только тогда)
        self._eye_z = self.cfg.eye_height        # высота глаз над ногами (обновляется из памяти)
        # флик: None | dict(t0, T, dyaw, dpitch, s_prev); _flick_pending — новый захват, флик ещё не спланирован
        self._flick: dict | None = None
        self._flick_pending = False
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

    # ------------------------------------------------------------ helpers

    def _now(self) -> float:
        return self._clock()

    def _read_visible_enemies(self) -> list[dict]:
        """Видимые враги: ноги из памяти + target_aim_z по Z (точка прицеливания)."""
        fn = getattr(self.reader, "visible_enemies", None)
        if not callable(fn):
            return []
        out: list[dict] = []
        try:
            for p in fn():
                aim = p.aim_position(self.cfg.target_aim_z)
                out.append(
                    {
                        "index": p.index,
                        "name": p.name,
                        "health": p.health,
                        "position": (round(aim.x), round(aim.y), round(aim.z)),
                    }
                )
        except Exception as exc:  # чтение памяти не должно ломать движение
            self.log(f"visible_enemies: ошибка чтения ({exc}); мишени пропущены")
            return self.visible_enemies
        return out

    # ------------------------------------------------------------ targets

    def _update_target(
        self, now: float, x: float, y: float, yaw: float, move_bearing: float | None, speed: float
    ) -> dict | None:
        """
        Какую мишень вести камерой (или None — идём по маршруту).

        Как человек: мишень замечается с задержкой реакции; пока текущая видна — держим её
        (не скачем между двумя); на новую переключаемся, только если она заметно ближе к
        прицелу; после исчезновения ещё target_hold секунд смотрим в ту же точку.
        Смотрим на одну мишень не дольше target_engage_max, потом возвращаемся к маршруту
        (cooldown), и не бежим долго спиной вперёд, если мишень осталась позади.
        """
        cfg = self.cfg
        if not cfg.engage_targets or (cfg.fire_enabled and not self.yaw_verified and self._target is None):
            # без подтверждённого соглашения yaw прицел может быть развёрнут — сначала пройдёмся
            self._target = None
            return None

        def ang_err(e: dict) -> float:
            ex, ey, _ = e["position"]
            return abs(shortest_yaw_err(yaw_to_target(x, y, ex, ey), yaw))

        def behind(e: dict) -> bool:
            if move_bearing is None:
                return False
            ex, ey, _ = e["position"]
            return abs(shortest_yaw_err(yaw_to_target(x, y, ex, ey), move_bearing)) > cfg.target_behind_deg

        def start(e: dict, fast: bool) -> None:
            self._target = dict(e)
            self._target_seen_at = now
            self._target_last_seen = now
            self._target_reaction = self.rng.uniform(*cfg.target_reaction) * (0.4 if fast else 1.0)
            self._target_engage_until = now + self._target_reaction + self.rng.uniform(*cfg.target_engage_max)
            self._target_behind_since = None
            self._shots_on_target = 0
            self._aimed_since = None
            self._flick = None
            self._flick_pending = True
            self._subflicks = 0

        def drop(reason: str, cooldown: float | None = None) -> None:
            if self._target is not None:
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
            self._flick = None
            self._flick_pending = False

        cand = [
            e for e in self.visible_enemies
            if math.hypot(e["position"][0] - x, e["position"][1] - y) <= cfg.target_max_dist
            and self._target_cooldown.get(e["index"], 0.0) <= now
        ]
        cur = self._target
        same = next((e for e in cand if e["index"] == cur["index"]), None) if cur is not None else None
        self._target_visible_now = same is not None

        if same is not None:
            best = min(cand, key=ang_err)
            if best["index"] != same["index"] and ang_err(best) + cfg.target_switch_margin < ang_err(same):
                start(best, fast=True)
                self._target_visible_now = True
            else:
                self._target = dict(same)
                self._target_last_seen = now
        elif cand:
            fast = cur is not None and now - self._target_last_seen <= cfg.target_hold
            if cur is not None:
                drop("пропала, есть следующая")
            start(min(cand, key=ang_err), fast)
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
        # мишень позади по ходу, а мы бежим — человек не бегает спиной дольше полсекунды
        # (во время стрельбы стоим, так что это правило работает только на бегу)
        if speed > 100.0 and behind(t) and not self.firing:
            if self._target_behind_since is None:
                self._target_behind_since = now
            elif now - self._target_behind_since > 0.6:
                drop("осталась позади")
                return None
        else:
            self._target_behind_since = None
        return t

    def _aim_angles(self, x: float, y: float, z: float, target: dict) -> tuple[float, float]:
        """yaw/pitch на точку прицеливания мишени (position — ноги + target_aim_z), из своих глаз."""
        tx, ty, tz = target["position"]
        yaw = yaw_to_target(x, y, tx, ty)
        d = math.hypot(tx - x, ty - y)
        pitch = -math.degrees(math.atan2(tz - (z + self._eye_z), max(1.0, d)))
        return yaw, pitch

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
        self._set_keys(set())

    def _jump(self) -> None:
        self.input.key_down("space")
        self.input.key_up("space")

    # ------------------------------------------------------------ planning

    def _obstacles(self) -> list[Obstacle]:
        """Видимые мишени как твёрдые препятствия на полу (position — точка прицеливания → ноги).

        После каждого упирания радиус растёт: узкий проход с мишенью честно становится
        непроходимым, и планировщик берёт другую цель, а не пытается «протиснуться» снова.
        """
        r = self.cfg.target_body_radius + 16.0 * self._target_bumps
        return [
            (e["position"][0], e["position"][1], e["position"][2] - self.cfg.target_aim_z, r)
            for e in self.known_targets
        ]

    def _blockers_key(self) -> frozenset:
        return frozenset((e["index"], e["position"]) for e in self.known_targets)

    def plan(self, pos: XYZ, goal_id: int | None = None) -> Route:
        blocked = blocked_by_obstacles(self.map, self._obstacles())
        picking_new = goal_id is None
        if picking_new:
            # новую дальнюю считаем по карте без мишеней: иначе при перекрытом проходе
            # «самая дальняя достижимая» оказывается стартовой стороной — бот разворачивается
            given_up_now = {e["index"] for e in self.known_targets if self._given_up.get(e["index"], 0.0) > self._now()}
            use_blocked = blocked if given_up_now else set()
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
            alt = pick_farthest_goal(self.map, pos, self.cfg.route, blocked)
            pts = plan_route(self.map, pos, alt, self.cfg.route, blocked)
            self.log("Цель недостижима → новая дальняя цель")
            self._intended_goal_id = alt
            goal_id = alt
        self.goal_id = goal_id
        self.route = Route(pts)
        self.s = 0.0
        self._stuck_level = 0
        self._route_blockers = self._blockers_key()
        self._blocked = blocked
        g = self.map.nodes[goal_id]
        self.log(
            f"Маршрут: {len(pts)} вершин, {self.route.length:.0f}u → цель "
            f"({g.x:.0f},{g.y:.0f},{g.z:.0f}) clearance={g.clearance:.0f}"
            + (f", обход мишеней: {len(blocked)} клеток" if blocked else "")
            + extra
        )
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

    def _standoff_route(self, pos: XYZ, goal_id: int) -> tuple[list[XYZ], dict] | None:
        """Маршрут к цели без учёта мишеней, обрезанный за standoff_dist до первой мишени на нём.
        None — если стрельба выключена, по мишени уже отстрелялись (не уходит) или обрезать нечего."""
        cfg = self.cfg
        if not cfg.fire_enabled or not self.known_targets:
            return None
        now = self._now()
        live = [e for e in self.known_targets if self._given_up.get(e["index"], 0.0) <= now]
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
                ex, ey, ez = e["position"]
                if math.hypot(ex - px, ey - py) <= reach and abs((ez - self.cfg.target_aim_z) - pz) <= 72.0:
                    return e
            d += 8.0
        return None

    def _near_target(self, x: float, y: float, margin: float = 12.0) -> bool:
        reach = self.map.cfg.hull_radius + self.cfg.target_body_radius + margin
        return any(math.hypot(e["position"][0] - x, e["position"][1] - y) <= reach for e in self.known_targets)

    def _los_clear(self, x: float, y: float, z: float, tx: float, ty: float, tz: float) -> bool:
        """Прямая от нас до места мишени идёт по проходимым клеткам карты (не через стену).
        Последние hull+radius юнитов не проверяем: сама мишень может стоять чуть вне записанной полосы."""
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        stop = d - (self.map.cfg.hull_radius + self.cfg.target_body_radius + 8.0)
        if stop <= 0.0:
            return True
        if abs(tz - z) > 96.0:
            return False           # другой этаж: по 2D-карте не судим
        steps = max(1, int(stop / 8.0))
        cz = z
        for i in range(1, steps + 1):
            k = (i / steps) * (stop / d)
            node = self.map.layer_at(x + dx * k, y + dy * k, cz, dz_max=self.map.cfg.jump_height)
            if node is None:
                return False
            cz = node.z
        return True

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
        need = self.map.cfg.hull_radius * 0.5
        d = cfg.carrot_dist
        # рядом с мишенью идём точнее по линии маршрута (он уже построен в обход)
        if self.known_targets and self._near_target(pos[0], pos[1], margin=cfg.target_avoid_dist):
            d = min(d, 36.0)
        while d > 16.0:
            c = route.point_at(self.s + d)
            if segment_clear(self.map, pos, c, need, cfg.route, self._blocked):
                return c
            d -= 8.0
        return route.point_at(self.s + 16.0)

    def _steer_bearing(self, x: float, y: float, vx: float, vy: float, tx: float, ty: float) -> float:
        """
        Куда толкать тело: желаемая скорость (к морковке, полная) минус текущая.
        Так момент гасится заранее (counter-strafe), тело не выносит в стену на повороте.
        """
        dx, dy = tx - x, ty - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return math.degrees(math.atan2(vy, vx)) if (vx or vy) else 0.0
        vmax = self.cfg.max_speed
        des_x, des_y = dx / d * vmax, dy / d * vmax
        des_x, des_y = self._avoid_targets(x, y, des_x, des_y)
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
            # (но если уже бежим — можно погасить момент клавишей назад)
            if speed > 120.0:
                return {"s"} if arel > 157.5 else ({"s", "a"} if rel > 0 else {"s", "d"})
            if abs(yaw_err) > 10.0:
                return set()       # камера ещё доворачивается — ждём
            # камера уже смотрит куда надо (вдоль маршрута), а шагнуть нужно назад/вбок
            # (например, отойти от мишени) — игрок в таком случае просто пятится, не крутясь
            _, back_keys = min(_KEY_DIRS, key=lambda kd: abs(shortest_yaw_err(kd[0], rel)))
            return set(back_keys)
        if arel > self.cfg.strafe_only_deg:
            return {"a"} if rel > 0 else {"d"}

        cur = frozenset(self._keys)
        best_angle, best_keys = min(_KEY_DIRS, key=lambda kd: abs(shortest_yaw_err(kd[0], rel)))
        if cur in _DIR_OF_KEYS and cur != best_keys:
            cur_err = abs(shortest_yaw_err(_DIR_OF_KEYS[cur], rel))
            # гистерезис: не отпускаем текущую комбинацию, пока она «примерно туда»
            if cur_err <= 22.5 + self.cfg.key_hysteresis_deg:
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
    ) -> tuple[float, float]:
        cfg = self.cfg
        pitch_gain = cfg.pitch_gain if pitch_gain is None else pitch_gain
        pitch_rate_max = cfg.pitch_rate_max if pitch_rate_max is None else pitch_rate_max
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
        soft = abs(yaw_err) / (abs(yaw_err) + cfg.yaw_deadband)
        desired = max(-rate_max, min(rate_max, gain * yaw_err * soft))
        # S-образный разгон/торможение: скорость догоняет желаемую через ускорение,
        # а само ускорение меняется не быстрее yaw_jerk_max (без «углов» на графике скорости)
        self._yaw_rate, self._yaw_acc = self._track_rate(
            self._yaw_rate, self._yaw_acc, desired, dt, cfg.yaw_accel_max, cfg.yaw_jerk_max
        )
        d_yaw = self._yaw_rate * dt
        # не перелетать
        if abs(d_yaw) > abs(yaw_err) + 0.5:
            d_yaw = yaw_err
            self._yaw_rate = yaw_err / dt
            self._yaw_acc = 0.0

        soft_p = abs(pitch_err) / (abs(pitch_err) + 0.5)
        p_des = max(-pitch_rate_max, min(pitch_rate_max, pitch_gain * pitch_err * soft_p))
        self._pitch_rate, self._pitch_acc = self._track_rate(
            self._pitch_rate, self._pitch_acc, p_des, dt, cfg.yaw_accel_max * 0.5, cfg.yaw_jerk_max * 0.5
        )
        d_pitch = self._pitch_rate * dt
        if abs(d_pitch) > abs(pitch_err) + 0.2:
            d_pitch = pitch_err
            self._pitch_acc = 0.0

        # ветер (WindMouse): инерционная случайная скорость, сильнее — когда рука движется,
        # затухает у цели; вместо белого шума на каждый тик
        activity = min(1.0, (abs(self._yaw_rate) + abs(self._pitch_rate)) / 60.0)
        wy, wp = self._wind_step(cfg.hand_wind_sigma, cfg.hand_wind_decay)
        d_yaw += wy * activity * dt
        d_pitch += wp * activity * 0.5 * dt
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

    def _plan_flick(self, now: float, yaw_err: float, pitch_err: float, corrective: bool = False) -> None:
        """
        Баллистический мах к мишени: длительность по Фиттсу, скорость — колокол minimum-jerk,
        траектория — дуга (боковой изгиб sin(πτ)+sin(2πτ), как реальный мах кистью/рукой),
        конец — чуть недоведён и с разбросом. corrective — второе, корректирующее подвижение:
        точнее, без недовода, изгиб меньше.
        """
        cfg = self.cfg
        dist = math.hypot(yaw_err, pitch_err)
        self._flick_pending = False
        if dist < cfg.flick_min_deg:
            self._flick = None
            self._correct_from = now            # сразу коррекция
            return
        T = cfg.flick_base_time + cfg.flick_per_deg * dist
        T = max(T, 1.875 * dist / cfg.flick_rate_max)     # пик min-jerk = 1.875·dist/T
        ux, uy = yaw_err / dist, pitch_err / dist
        prec = 0.5 if corrective else 1.0
        under = 0.0 if corrective else cfg.flick_undershoot
        along = dist * (1.0 - under) + self.rng.gauss(0.0, prec * (cfg.flick_error_frac * dist + cfg.flick_error_min))
        # поперёк промахиваются гораздо меньше, чем по дальности (направление рука знает точно):
        # σ ≈ четверть продольной, и не больше 0.6° — иначе в конце появляется «крючок» на коррекции
        across = self.rng.gauss(0.0, prec * 0.25 * (cfg.flick_error_frac * dist + cfg.flick_error_min))
        across = max(-0.6, min(0.6, across))
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
            "prev": (0.0, 0.0),                 # положение на траектории на прошлом тике
        }
        self._wind = [0.0, 0.0]
        # после корректирующего подвижения глазу нужно меньше времени — цель уже почти под прицелом
        delay = self.rng.uniform(*cfg.aim_correct_delay) * (0.5 if corrective else 1.0)
        self._correct_from = now + T + delay
        # скорость не обнуляем: флик подхватывает текущий поворот (на маршруте уже крутили камеру)
        # набор — доля маха, иначе короткий флик весь уходит в «разгон» и гаснет
        self._flick["ramp"] = min(cfg.flick_ramp * (0.55 if corrective else 1.0), 0.32 * T)

    def _flick_pos(self, f: dict, tau: float) -> tuple[float, float]:
        s = self._min_jerk(tau)
        a = f["along"] * s
        c = f["across"] * s + f["c1"] * math.sin(math.pi * tau) + f["c2"] * math.sin(2.0 * math.pi * tau)
        ux, uy = f["u"]
        nx, ny = f["n"]
        return ux * a + nx * c, uy * a + ny * c

    def _aim_step(
        self, now: float, dt: float, yaw_err: float, pitch_err: float, aim_yaw: float, aim_pitch: float
    ) -> tuple[float, float, str]:
        """
        Наводка на мишень как у человека: флик (открытый контур, профиль minimum-jerk по дуге
        с ветром), короткая зрительная задержка, затем корректирующее подвижение / плавная
        коррекция по ошибке из памяти. Возвращает (d_yaw, d_pitch, фаза).
        """
        cfg = self.cfg
        if self._flick_pending:
            self._plan_flick(now, yaw_err, pitch_err)
            if self._flick is not None:
                self._flick["aim_prev"] = (aim_yaw, aim_pitch)
        f = self._flick
        if f is not None:
            tau = (now + dt - f["t0"]) / f["T"]
            px, py = self._flick_pos(f, tau)
            d_yaw = px - f["prev"][0]
            d_pitch = py - f["prev"][1]
            f["prev"] = (px, py)
            # мишень «уезжает», пока мы тормозим — рука ведёт её (feed-forward по смещению цели,
            # зависит только от нашей позиции, не от камеры → без двойного учёта задержки ввода)
            ap = f.get("aim_prev", (aim_yaw, aim_pitch))
            d_yaw += shortest_yaw_err(aim_yaw, ap[0])
            d_pitch += aim_pitch - ap[1]
            f["aim_prev"] = (aim_yaw, aim_pitch)
            # ветер: инерционное смещение *положения* (°) с огибающей sin(πτ) — волнистость посреди маха,
            # у концов → 0, так что в цель приходим без накопленной ошибки (если добавлять к скорости,
            # интеграл ветра оседает в промахе на несколько градусов)
            # скорость ветра — AR(1), положение — её интеграл (гладкая волна, а не зигзаг по тикам)
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
            # набор скорости в начале маха: от текущей (маршрут) к профилю флика, без скачка
            ramp = max(1e-4, f.get("ramp", cfg.flick_ramp))
            u = min(1.0, max(0.0, (now - f["t0"]) / ramp))
            k = u * u * (3.0 - 2.0 * u)
            d_yaw = (1.0 - k) * self._yaw_rate * dt + k * d_yaw
            d_pitch = (1.0 - k) * self._pitch_rate * dt + k * d_pitch
            self._yaw_rate = d_yaw / dt
            self._pitch_rate = d_pitch / dt
            return d_yaw, d_pitch, "flick"
        ty, tp = self._tremor_step()
        if now < self._correct_from:
            # глаз ещё «догоняет» результат флика — рука отдыхает, лёгкий тремор
            self._yaw_rate = self._pitch_rate = 0.0
            self._yaw_acc = self._pitch_acc = 0.0
            return ty, tp, "settle"
        # большой остаток после маха → корректирующее подвижение (тоже баллистическое, точнее),
        # как у человека: primary submovement + corrective submovement
        if math.hypot(yaw_err, pitch_err) >= cfg.flick_min_deg * 1.25 and self._subflicks < 1:
            self._subflicks += 1
            self._plan_flick(now, yaw_err, pitch_err, corrective=True)
            if self._flick is not None:
                self._flick["aim_prev"] = (aim_yaw, aim_pitch)
                return self._aim_step(now, dt, yaw_err, pitch_err, aim_yaw, aim_pitch)
        # мелкая коррекция: экспоненциальное дожатие, скорость набирается S-кривой (без скачка)
        g, r = cfg.aim_correct_gain, cfg.aim_correct_rate
        acc_y, acc_p = 4000.0, 3000.0
        # ошибка из памяти отстаёт на ~1–2 тика: вычитаем то, что уже «в пути»
        lag = 1.5 * dt
        ey = yaw_err - self._yaw_rate * lag
        ep = pitch_err - self._pitch_rate * lag
        # тормозим и по остатку пути: |v| ≤ √(2·a·|e|), иначе на подлёте к цели перелетаем и качаемся
        vmax_y = math.sqrt(2.0 * 0.6 * acc_y * abs(ey))
        vmax_p = math.sqrt(2.0 * 0.6 * acc_p * abs(ep))
        des_y = max(-min(r, vmax_y), min(min(r, vmax_y), g * ey))
        des_p = max(-min(r * 0.7, vmax_p), min(min(r * 0.7, vmax_p), g * ep))
        self._yaw_rate, self._yaw_acc = self._track_rate(self._yaw_rate, self._yaw_acc, des_y, dt, acc_y, 60000.0)
        self._pitch_rate, self._pitch_acc = self._track_rate(self._pitch_rate, self._pitch_acc, des_p, dt, acc_p, 45000.0)
        d_yaw = self._yaw_rate * dt
        d_pitch = self._pitch_rate * dt
        if abs(d_yaw) > abs(yaw_err):
            d_yaw = yaw_err
            self._yaw_rate, self._yaw_acc = yaw_err / dt, 0.0
        if abs(d_pitch) > abs(pitch_err):
            d_pitch = pitch_err
            self._pitch_rate, self._pitch_acc = pitch_err / dt, 0.0
        return d_yaw + ty, d_pitch + tp, "correct"

    def _send_mouse(self, d_yaw: float, d_pitch: float, max_px: int | None = None) -> tuple[int, int]:
        """Градусы → пиксели с переносом дробного остатка: мелкие плавные повороты не дребезжат."""
        cfg = self.cfg
        m = cfg.max_px_per_tick if max_px is None else max_px
        out = [0, 0]
        for i, (d, k) in enumerate(((d_yaw, cfg.yaw_per_px), (d_pitch, cfg.pitch_per_px))):
            if not k:
                continue
            want = d / k + self._px_carry[i]
            px = int(round(want))
            if abs(px) > m:
                px = m if px > 0 else -m
                self._px_carry[i] = 0.0          # обрезали — остаток не копим (иначе «догонит» рывком)
            else:
                self._px_carry[i] = want - px
            out[i] = px
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
            self.log("Застрял → прыжок")
            self._jump()
            return None
        if lvl == 2:
            side = {"d"} if rel < 0 else {"a"}
            self._override_keys = side | {"w"}
            self._override_until = now + 0.35
            self.log(f"Застрял → страйф {''.join(sorted(side)).upper()}")
            return set(self._override_keys)
        if lvl == 3:
            self._override_keys = {"s"}
            self._override_until = now + 0.3
            self.log("Застрял → шаг назад")
            return set(self._override_keys)
        self._replan(pos, "застревание", new_goal=lvl >= self.cfg.stuck_replan_after + 2)
        return None

    # ------------------------------------------------------------ main tick

    # ------------------------------------------------------------ firing

    def _fire_tolerance(self, dist: float) -> tuple[float, float]:
        """Допуск прицела (yaw, pitch): ближе мишень — шире угол, но не меньше минимума.
        По pitch — от запаса вверх до верха головы (вниз тело длиннее, но стреляем по симметрии)."""
        d = max(1.0, dist)
        half_w = math.degrees(math.atan2(self.cfg.fire_target_half_width, d))
        half_h = math.degrees(math.atan2(self.cfg.fire_target_up_margin, d))
        return max(self.cfg.fire_tolerance_deg, half_w * 0.8), max(self.cfg.fire_tolerance_pitch_deg, half_h * 0.8)

    def _fire_step(self, now: float, target: dict, x: float, y: float, yaw_err: float, pitch_err: float) -> bool:
        """Решение о выстреле на этом тике. Возвращает True, если нажали ЛКМ."""
        cfg = self.cfg
        if not cfg.fire_enabled or (cfg.shots_per_target > 0 and self._shots_on_target >= cfg.shots_per_target):
            return False
        if not self._target_visible_now or self._flick is not None or now < self._correct_from:
            # мишень не видна прямо сейчас / рука ещё в махе — не стреляем в пустоту
            self._aimed_since = None
            return False
        tx, ty, _ = target["position"]
        tol_y, tol_p = self._fire_tolerance(math.hypot(tx - x, ty - y))
        on_target = abs(yaw_err) <= tol_y and abs(pitch_err) <= tol_p
        if not on_target:
            self._aimed_since = None
            return False
        if self._aimed_since is None:
            self._aimed_since = now
            self._next_fire_at = max(self._next_fire_at, now + self.rng.uniform(*cfg.fire_first_delay))
            return False
        if now < self._next_fire_at:
            return False
        self.input.mouse_down("left")
        self._fire_release_at_tick = self._tick_no + cfg.fire_hold_ticks
        self._shots_on_target += 1
        self.shots_fired += 1
        self._next_fire_at = now + self.rng.uniform(*cfg.fire_interval)
        return True

    def _release_fire_if_due(self) -> None:
        if self._fire_release_at_tick >= 0 and self._tick_no >= self._fire_release_at_tick:
            self.input.mouse_up("left")
            self._fire_release_at_tick = -1

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
        if now >= self._enemies_next_poll:
            self._enemies_next_poll = now + 1.0 / max(1.0, cfg.enemies_poll_hz)
            self.visible_enemies = self._read_visible_enemies()
            for e in self.visible_enemies:
                self._known[e["index"]] = {**e, "seen": now, "unseen_los": 0}
            forgotten: list[dict] = []
            visible_ids = {e["index"] for e in self.visible_enemies}
            engaged_id = self._target["index"] if self._target is not None else None
            for idx, e in list(self._known.items()):
                if idx in visible_ids:
                    continue
                if now - e["seen"] > cfg.target_memory:
                    forgotten.append(self._known.pop(idx))
                    continue
                # по мишени уже стреляли и она пропала из видимых — забываем сразу,
                # не ждём 5 опросов (иначе ещё идём по объездному маршруту назад)
                if idx == engaged_id and self._shots_on_target:
                    forgotten.append(self._known.pop(idx))
                    continue
                # место, где стояла мишень, видно по прямой, а мишени в списке нет → она пропала
                # (убита/убрана), а не спряталась за углом — забываем, иначе обходим «невидимый столб»
                ex, ey, ez = e["position"]
                if self._los_clear(x, y, z, ex, ey, ez - cfg.target_aim_z):
                    e["unseen_los"] = e.get("unseen_los", 0) + 1
                    if e["unseen_los"] >= cfg.target_forget_polls:
                        forgotten.append(self._known.pop(idx))
                else:
                    e["unseen_los"] = 0
            if forgotten:
                self.known_targets = list(self._known.values())
                for e in forgotten:
                    self.log(f"Мишень {e.get('name') or e['index']} пропала — забыли, продолжаем исходную цель")
                if self.route is not None and (
                    self._blockers_key() != self._route_blockers or self._standoff is not None
                ):
                    # маршрут строился в обход / мы ждали на дистанции — идём дальше туда же, не «новую дальнюю»
                    self._replan((x, y, z), "мишень пропала, продолжаем маршрут")
            self.known_targets = list(self._known.values())

        self._calibrate_mouse(mem_yaw, mem_pitch)
        self._detect_yaw_offset(mem_yaw, v.x, v.y, speed)
        self._prev_mem = (mem_yaw, mem_pitch)
        yaw = shortest_yaw_err(mem_yaw + self.yaw_offset, 0.0)

        info: dict = {
            "pos": (round(x), round(y), round(z)),
            "speed": round(speed),
            "yaw": round(yaw, 1),
            "pitch": round(mem_pitch, 1),
            "visible_enemies": self.visible_enemies,
        }

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
        self.s, off = route.project(x, y, self.s)
        gx, gy, gz = route.goal
        goal_d = math.hypot(gx - x, gy - y)
        if off > cfg.off_route_replan:
            self._replan((x, y, z), f"ушли с маршрута на {off:.0f}u")
            route = self.route
            assert route is not None
            self.s, off = route.project(x, y, None)

        # мишень встала на маршруте впереди (и маршрут строился без неё) → обойти заранее
        if now >= self._block_check_at:
            self._block_check_at = now + 0.25
            blocker = self._target_on_route(route)
            if blocker is not None and self._blockers_key() != self._route_blockers:
                self._replan((x, y, z), f"мишень {blocker.get('name') or blocker['index']} на пути")
                route = self.route
                assert route is not None
                self.s, off = route.project(x, y, None)

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
                self.s, off = route.project(x, y, None)
            elif gave_up:
                self._replan((x, y, z), "мишень не уходит → другая цель", new_goal=True)
                route = self.route
                assert route is not None
                self.s, off = route.project(x, y, None)
            else:
                hold = True
        elif self._standoff is not None and self._given_up.get(self._standoff, 0.0) > now:
            self._replan((x, y, z), "мишень не уходит → другая цель", new_goal=True)
            route = self.route
            assert route is not None
            self.s, off = route.project(x, y, None)

        dest_id = self._intended_goal_id if self._intended_goal_id is not None else self.goal_id
        dest = self.map.nodes[dest_id] if dest_id is not None else None
        dest_d = math.hypot(dest.x - x, dest.y - y) if dest is not None else goal_d
        dest_z = dest.z if dest is not None else gz
        if not hold and self._standoff is None and dest_d <= cfg.arrive_radius and abs(dest_z - z) <= 48.0:
            self.release()
            self.log(f"Цель достигнута ({x:.0f},{y:.0f},{z:.0f})")
            if cfg.loop:
                self._pause_until = now + self.rng.uniform(*cfg.pause_at_goal)
                self.route = None
                info["action"] = "arrived"
            else:
                self.done = True
                info["action"] = "done"
            self._sent_px = (0, 0)
            return info

        # --- цели камеры и ног
        look_d = max(cfg.look_ahead_min, min(cfg.look_ahead_max, cfg.look_ahead_min + speed * cfg.look_ahead_speed_gain))
        look_d *= 1.0 + 0.35 * self._cam_pace   # тот же «темп»: чуть ближе/дальше точка взгляда
        look_d = min(look_d, route.remaining(self.s) + 40.0)   # у финиша смотрим на цель
        lx, ly, lz = route.point_at(self.s + look_d)
        cx_, cy_, cz_ = self._carrot((x, y, z), route)

        # медленное блуждание прицела ±wander
        # блуждание взгляда: плавный дрейф (случайная скорость с инерцией), а не белый шум на точку
        self._wander_vel = self._wander_vel * 0.96 + self.rng.gauss(0.0, 0.35)
        self._wander += self._wander_vel * dt - self._wander * 0.02
        self._wander = max(-cfg.yaw_wander, min(cfg.yaw_wander, self._wander))

        # --- мишень? камера уходит на неё, ноги продолжают маршрут
        route_bearing = yaw_to_target(x, y, cx_, cy_) if math.hypot(cx_ - x, cy_ - y) > 4.0 else None
        target = self._update_target(now, x, y, yaw, route_bearing, speed)
        was_engaged = self.engaged
        self.engaged = target is not None
        if self.engaged and not was_engaged:
            self.targets_engaged += 1
            self.log(f"Мишень {target.get('name') or target['index']} @ {target['position']} — взгляд на неё")
        elif was_engaged and not self.engaged:
            self.log("Мишень потеряна — взгляд обратно на маршрут")

        if target is not None:
            self.engaged_ticks += 1
            aim_yaw, aim_pitch = self._aim_angles(x, y, z, target)
            dy_drift, dp_drift = self._aim_drift_step(cfg.target_aim_jitter)
            look_yaw = aim_yaw + dy_drift
            pitch_target = max(-60.0, min(60.0, aim_pitch + dp_drift))
            rate_cap: float | None = cfg.target_yaw_rate_max
            gain: float | None = cfg.target_yaw_gain
            lx, ly = target["position"][0], target["position"][1]
        else:
            look_yaw = yaw_to_target(x, y, lx, ly) + self._wander
            slope = route.slope_ahead(self.s, 140.0)
            pitch_target = cfg.pitch_base - slope * cfg.pitch_slope_gain
            pitch_target = max(-25.0, min(30.0, pitch_target))
            rate_cap = None
            gain = None
        yaw_err = shortest_yaw_err(look_yaw, yaw)
        pitch_err = pitch_target - mem_pitch

        # выстрел решаем по прицелу «как есть» до сдвига мыши на этом тике
        # (прицел без дрожания: yaw_err выше содержит jitter, для решения о выстреле берём чистую ошибку)
        shot = False
        aim_phase = ""
        if target is not None:
            aim_yaw_err = shortest_yaw_err(aim_yaw, yaw)
            aim_pitch_err = aim_pitch - mem_pitch
            shot = self._fire_step(now, target, x, y, aim_yaw_err, aim_pitch_err)
            if shot and self._shots_on_target == 1:
                self.log(
                    f"Огонь по {target.get('name') or target['index']} через "
                    f"{now - self._target_seen_at:.2f}s (ошибка {aim_yaw_err:+.1f}°/{aim_pitch_err:+.1f}°)"
                )
            # режим стрельбы: заметили мишень → останавливаемся, разворачиваемся, стреляем, идём дальше
            self.firing = cfg.fire_enabled
            if cfg.fire_enabled:
                # рука: флик → пауза → коррекция (ошибка с лёгким дрожанием точки прицеливания)
                d_yaw, d_pitch, aim_phase = self._aim_step(now, dt, yaw_err, pitch_err, aim_yaw, aim_pitch)
            else:
                d_yaw, d_pitch = self._camera_step(yaw_err, pitch_err, dt, speed, rate_cap, gain)
            dx, dy = self._send_mouse(d_yaw, d_pitch, cfg.aim_max_px_per_tick)
        else:
            self.firing = False
            d_yaw, d_pitch = self._camera_step(yaw_err, pitch_err, dt, speed, rate_cap, gain)
            dx, dy = self._send_mouse(d_yaw, d_pitch)

        # клавиши считаем относительно yaw, каким он будет после этого шага мыши
        yaw_next = yaw + d_yaw
        move_bearing = self._steer_bearing(x, y, v.x, v.y, cx_, cy_)
        rel = shortest_yaw_err(move_bearing, yaw_next)
        if self.firing and cfg.fire_stop_moving:
            keys = self._counter_strafe_keys(v.x, v.y, speed, yaw_next)
        else:
            keys = self._choose_keys(rel, yaw_err, speed, engaged=self.engaged)

        # торможение у цели: отпустить клавиши чуть раньше, трение остановит внутри arrive_radius
        brake_d = cfg.arrive_radius * 0.5 + speed * cfg.brake_time
        if self._brake and speed < 40.0:
            self._brake = False            # остановились, но не дошли — дошагиваем
        if goal_d < brake_d and speed > 60.0:
            self._brake = True
        if self._brake or hold:
            keys = set()

        # прыжок по маршруту (ступенька выше step_height впереди)
        if cz_ - z > self.map.cfg.step_height and math.hypot(cx_ - x, cy_ - y) < 48.0 and speed > 40.0:
            self._jump()

        override = self._handle_stuck(now, (x, y, z), speed, keys, rel)
        if override is not None:
            keys = override

        if frozenset(keys) == frozenset(self._keys):
            self._same_keys_ticks += 1
        else:
            self._same_keys_ticks = 0
            self._keys_since = now
        self._set_keys(keys)

        if (self._stuck_level or self._target_bumps) and speed > 120.0 and now >= self._override_until:
            self._stuck_level = 0
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
            }
        )
        return info

    def run(self, duration: float | None = None) -> None:
        duration = duration if duration is not None else self.cfg.duration
        interval = 1.0 / self.cfg.tick_hz
        t0 = time.monotonic()
        n = 0
        try:
            while True:
                if duration and time.monotonic() - t0 > duration:
                    print(f"Лимит {duration:.0f}s")
                    break
                info = self.tick()
                if info.get("action") == "done":
                    break
                n += 1
                if self.cfg.verbose_log and n % int(self.cfg.tick_hz / 4) == 0:
                    enemies = info.get("visible_enemies") or []
                    enemy_txt = ""
                    if enemies:
                        enemy_txt = " enemies=" + ",".join(
                            f"{e.get('name') or e['index']}@{e['position']}" for e in enemies
                        )
                    print(
                        f"[{info.get('action')}] keys={info.get('keys')} spd={info.get('speed')} "
                        f"pos={info.get('pos')} s={info.get('s')}/{info.get('route_len')} "
                        f"off={info.get('off_route')} goal_d={info.get('goal_d')} "
                        f"yaw={info.get('yaw')} err={info.get('yaw_err')} rel={info.get('rel')} "
                        f"mouse={info.get('mouse')} k={info.get('yaw_per_px')} off_yaw={info.get('yaw_offset')}"
                        f"{enemy_txt}"
                    )
                # держим ровный такт
                next_t = t0 + (n + 1) * interval
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
        finally:
            self.release()
            self.input.release_all()
