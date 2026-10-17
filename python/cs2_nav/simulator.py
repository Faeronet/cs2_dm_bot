"""
Оффлайн-симуляция навигации без CS2: физика движения Source (разгон, трение,
скольжение вдоль стен), мышь → углы, память → позиция/скорость/углы.

Позволяет проверить карту, маршрут и контроллер и сразу увидеть картинку.
Намеренно «неудобная» для бота: чувствительность мыши отличается от ожидаемой,
можно инвертировать мышь и сместить yaw в «памяти» — контроллер обязан адаптироваться.
"""

from __future__ import annotations

import math
import random
from collections.abc import Callable
from dataclasses import dataclass, field

from cs2_control.input import GameInput
from cs2_nav.coord_map import CoordinateMap
from cs2_nav.navigator import NavConfig, Navigator
from cs2_reader.aim import forward_from_angles, shortest_yaw_err
from cs2_reader.game_state import Vec3, WeaponInfo
from cs2_reader.weapons import default_clip, resolve_weapon, weapon_cycle_s, weapon_is_auto, weapon_needs_scope
from cs2_reader.recoil import recoil_offset, spray_recover_s

# CS2: sv_gravity 800, sv_jump_impulse 301.993 → высота стоя 57u; с приседом ящик 64u.
_CS2_GRAVITY = 800.0
_CS2_JUMP_IMPULSE = 301.993
_CS2_LADDER_SPEED = 200.0


class FakeInput(GameInput):
    def __init__(self) -> None:
        self.mouse_dx = 0
        self.mouse_dy = 0
        self.keys: set[str] = set()
        self.jump_requested = False
        self.key_events = 0
        self.buttons: set[str] = set()
        self.shot_requested = False
        self.left_presses = 0
        self.left_hold_ticks = 0

    def key_down(self, key: str) -> None:
        k = key.lower()
        if k == "space":
            self.jump_requested = True
            return
        if k not in self.keys:
            self.keys.add(k)
            self.key_events += 1

    def key_up(self, key: str) -> None:
        k = key.lower()
        if k in self.keys:
            self.keys.discard(k)
            self.key_events += 1

    def mouse_move(self, dx: int, dy: int) -> None:
        self.mouse_dx += dx
        self.mouse_dy += dy

    def mouse_down(self, button: str = "left") -> None:
        if button not in self.buttons:
            self.buttons.add(button)
            if button == "left":
                self.shot_requested = True     # фронт нажатия = первый выстрел
                self.left_presses += 1

    def mouse_up(self, button: str = "left") -> None:
        self.buttons.discard(button)

    def release_all(self) -> None:
        self.keys.clear()
        self.buttons.clear()


@dataclass
class SimPlayer:
    x: float
    y: float
    z: float
    yaw: float            # истинный yaw (Source)
    pitch: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    airborne: float = 0.0  # >0 в прыжке
    ground_z: float = 0.0
    crouched: bool = False


@dataclass
class SimTarget:
    """Макет-голова: (x,y,z) — центр объёмного овала, не ноги."""

    index: int
    x: float
    y: float
    z: float
    name: str = ""
    health: int = 100
    radius: float = 8.0             # горизонтальный «зазор» для коллизии тела
    rx: float = 4.5                 # полуось овала XY (голова)
    rz: float = 5.5                 # полуось овала Z
    hits_to_kill: int = 3
    hits: int = 0
    alive: bool = True
    visible: bool = False
    dormant: bool = False
    vulnerable: bool = True
    spawn_immunity: bool = False
    has_moved_since_spawn: bool = True
    immune_until: float = 0.0
    respawn_at: float = float("inf")

    @property
    def position(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)

    @property
    def head_position(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)

    @property
    def chest_position(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)

    @property
    def left_shoulder_position(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)

    @property
    def right_shoulder_position(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)

    def aim_position(self, z_offset: float | None = None) -> Vec3:
        return self.head_position

    @property
    def velocity(self) -> Vec3:
        return Vec3(0.0, 0.0, 0.0)


class FakeReader:
    """Отдаёт «память» с задержкой в один тик и (опционально) смещённым yaw."""

    def __init__(
        self,
        player: SimPlayer,
        yaw_offset: float = 0.0,
        noise: float = 0.0,
        targets: list[SimTarget] | None = None,
        visible_fn: Callable[[SimTarget], bool] | None = None,
        weapon: WeaponInfo | None = None,
    ):
        self.p = player
        self.yaw_offset = yaw_offset
        self.noise = noise
        self.rng = random.Random(1)
        self.targets = targets or []
        self._visible_fn = visible_fn or (lambda _t: True)
        self._visible: list[SimTarget] = []
        self._weapon = weapon or WeaponInfo(id=7, name="ak47", clip=30)
        self._shots_fired = 0
        self._shots_idle_at: float | None = None
        self._scoped = False
        self._ladders: list[dict] = []
        self._snap = self._take()

    def _take(self) -> tuple[Vec3, Vec3, Vec3]:
        p = self.p
        n = self.noise
        pos = Vec3(p.x + self.rng.uniform(-n, n), p.y + self.rng.uniform(-n, n), p.z)
        vel = Vec3(p.vx, p.vy, 0.0)
        ang = Vec3(p.pitch, shortest_yaw_err(p.yaw + self.yaw_offset, 0.0), 0.0)
        return pos, vel, ang

    def latch(self) -> None:
        self._snap = self._take()
        # как m_bSpotted в игре: видимость определяется движком, не FOV камеры
        for t in self.targets:
            t.visible = bool(t.alive and self._visible_fn(t))
        self._visible = [t for t in self.targets if t.alive and t.visible]

    def local_position(self) -> Vec3:
        return self._snap[0]

    def local_velocity(self) -> Vec3:
        return self._snap[1]

    def local_eye_angles(self) -> Vec3:
        return self._snap[2]

    def local_view_offset_z(self) -> float:
        return 46.0 if self.p.crouched else 64.0

    def visible_enemies(self) -> list[SimTarget]:
        return list(self._visible)

    def enemies(self) -> list[SimTarget]:
        """Все живые мишени (и spotted, и за стеной) — как Reader.enemies()."""
        return [t for t in self.targets if t.alive]

    def local_weapon(self) -> WeaponInfo:
        return self._weapon

    def local_shots_fired(self) -> int:
        return self._shots_fired

    def local_is_scoped(self) -> bool:
        return self._scoped

    def local_has_healthshot(self) -> bool:
        return False

    def ladders(self, force_refresh: bool = False):
        return list(self._ladders)

    def doors(self, force_refresh: bool = False):
        return []

    def breakables(self, force_refresh: bool = False):
        return []

    def local_on_ladder(self) -> bool:
        return _player_on_ladder(self.p.x, self.p.y, self.p.z, self._ladders)

    def poll_life(self):
        from cs2_reader.life import LocalLifeInfo, LocalLifeUpdate, LifeEvents

        return LocalLifeUpdate(
            state=LocalLifeInfo(alive=True, health=100, life_state=0, pawn_alive=True, pawn=1),
            events=LifeEvents(),
        )


@dataclass
class SimConfig:
    max_speed: float = 250.0        # u/s (нож/пистолет)
    accelerate: float = 5.5         # sv_accelerate
    friction: float = 5.2           # sv_friction
    stop_speed: float = 80.0        # sv_stopspeed
    deg_per_px: float = 0.055       # чувствительность в «игре» (бот думает 0.0759)
    invert_mouse: bool = False
    yaw_offset: float = 0.0         # смещение yaw в «памяти»
    pos_noise: float = 0.0
    mouse_delay_ticks: int = 1
    jump_impulse: float = _CS2_JUMP_IMPULSE
    gravity: float = _CS2_GRAVITY
    # мишени-макеты: (x, y, z); z у пола → овал на высоте головы, иначе z = центр
    targets: list[tuple[float, float, float]] = field(default_factory=list)
    target_radius: float = 8.0
    target_rx: float = 4.5          # полуось головы по XY
    target_rz: float = 5.5          # полуось головы по Z
    target_vis_dist: float = 1400.0
    target_hits_to_kill: int = 3    # попаданий → мишень пропадает (0 = не пропадает)
    target_respawn: float = 6.0     # с до повторного появления (0 = навсегда)
    spawn_protect: float = 1.5      # с неуязвимости после респавна (как DM)
    weapon: str | int = "ak47"      # id или имя активного оружия
    weapon_clip: int | None = None  # None = полный магазин этого оружия


@dataclass
class SimResult:
    done: bool = False
    ticks: int = 0
    time: float = 0.0
    final_goal_dist: float = float("inf")
    walked: list[tuple[float, float, float]] = field(default_factory=list)
    headings: list[tuple[float, float, float]] = field(default_factory=list)
    route: list[tuple[float, float, float]] = field(default_factory=list)
    goal: tuple[float, float, float] | None = None
    route_len: float = 0.0
    walked_len: float = 0.0
    min_clearance: float = float("inf")
    low_clearance_ticks: int = 0
    stop_ticks: int = 0
    max_yaw_rate: float = 0.0
    yaw_rate_p95: float = 0.0
    moving_yaw_rate_p95: float = 0.0
    key_events: int = 0
    stucks: int = 0
    replans: int = 0
    collisions: int = 0
    yaw_offset_detected: float = 0.0
    yaw_per_px_final: float = 0.0
    events: list[str] = field(default_factory=list)
    log: list[str] = field(default_factory=list)
    # мишени
    targets: list[tuple[float, float, float]] = field(default_factory=list)
    engaged_ticks: int = 0
    targets_engaged: int = 0
    aim_lines: list[tuple[float, float, float, float]] = field(default_factory=list)   # (x, y, tx, ty)
    aim_err_p50: float = 0.0          # ° ошибка прицела по yaw, когда камера на мишени (после наводки)
    aim_settle_time: float = 0.0      # с: медиана времени от «увидел» до ошибки < 3°
    target_collisions: int = 0        # тиков касания тела мишени
    # стрельба
    shots: int = 0
    hits: int = 0
    shot_lines: list[tuple[float, float, float, float, bool]] = field(default_factory=list)  # (x, y, tx, ty, hit)
    fire_ticks: int = 0               # тиков в режиме стрельбы (стоим)
    first_shot_delay: float = 0.0     # с: медиана «увидел → первый выстрел»
    move_speed_at_shot: float = 0.0   # u/s: медиана скорости в момент выстрела (у игрока ≈ 0)
    targets_killed: int = 0
    kill_time: float = 0.0            # с: медиана «увидел → мишень пропала»
    flick_peak_rate: float = 0.0      # °/с: максимум скорости камеры во время наводки
    # плавность камеры
    yaw_accel_p95: float = 0.0        # °/с²: p95 |Δскорости| за тик (рывки)
    yaw_jerk_rms: float = 0.0         # °/с³: RMS изменения ускорения (дёрганость)
    rate_sign_flips_per_s: float = 0.0  # смен направления вращения камеры в секунду (дрожание)
    path_curvature: float = 0.0       # средняя кривизна траектории прицела при наводке (0 = прямая)


def _closest_ladder_dict(x: float, y: float, z: float, ladders: list[dict]) -> dict | None:
    best = None
    best_d = 64.0
    for lad in ladders:
        bot = lad.get("bottom"); top = lad.get("top")
        if not bot or not top:
            continue
        zb = min(bot[2], top[2]); zt = max(bot[2], top[2])
        if not (zb - 24.0 <= z <= zt + 24.0):
            continue
        bx, by = bot[0], bot[1]; tx, ty = top[0], top[1]
        dx, dy = tx - bx, ty - by
        seg = dx * dx + dy * dy
        if seg <= 1.0:
            d = math.hypot(x - bx, y - by)
        else:
            t = max(0.0, min(1.0, ((x - bx) * dx + (y - by) * dy) / seg))
            d = math.hypot(x - (bx + dx * t), y - (by + dy * t))
        if d < best_d:
            best_d = d; best = lad
    return best


def _player_on_ladder(x: float, y: float, z: float, ladders: list[dict]) -> bool:
    for lad in ladders:
        bot = lad.get("bottom")
        top = lad.get("top")
        if not bot or not top:
            continue
        if top[2] < bot[2]:
            bot, top = top, bot
        if not (bot[2] - 4.0 <= z <= top[2] + 16.0):
            continue
        meta = lad.get("ladder") or {}
        width = float(meta.get("width") or 32.0)
        radius = max(32.0, width * 0.5 + 20.0)
        # дистанция до отрезка bot_xy..top_xy (так покрываем диагональные/наклонные лестницы)
        bx, by = bot[0], bot[1]
        tx, ty = top[0], top[1]
        dx, dy = tx - bx, ty - by
        seg = dx * dx + dy * dy
        if seg <= 1.0:
            d = math.hypot(x - bx, y - by)
        else:
            t = max(0.0, min(1.0, ((x - bx) * dx + (y - by) * dy) / seg))
            px, py = bx + dx * t, by + dy * t
            d = math.hypot(x - px, y - py)
        if d <= radius:
            return True
    return False


class NavSimulator:
    def __init__(
        self,
        cm: CoordinateMap,
        start: tuple[float, float, float],
        *,
        nav_config: NavConfig | None = None,
        sim_config: SimConfig | None = None,
        initial_yaw: float = 0.0,
        initial_pitch: float = 0.0,
    ):
        self.cm = cm
        self.sim = sim_config or SimConfig()
        node = cm.nearest_node(*start)
        if node is None:
            raise ValueError(f"старт {start} вне карты")
        self.player = SimPlayer(
            x=start[0], y=start[1], z=node.z, yaw=initial_yaw, pitch=initial_pitch, ground_z=node.z
        )
        self.inp = FakeInput()
        self.targets: list[SimTarget] = []
        for i, (tx, ty, tz) in enumerate(self.sim.targets):
            tn = cm.nearest_node(tx, ty, tz)
            floor_z = tn.z if tn else tz
            # координата у пола → центр овала на высоте глаз; иначе z уже центр
            cz = floor_z + 64.0 if abs(tz - floor_z) <= 16.0 else tz
            self.targets.append(
                SimTarget(
                    index=100 + i, x=tx, y=ty, z=cz, name=f"T{i + 1}",
                    radius=self.sim.target_radius,
                    rx=self.sim.target_rx, rz=self.sim.target_rz,
                    hits_to_kill=self.sim.target_hits_to_kill,
                )
            )
        self.reader = FakeReader(
            self.player,
            yaw_offset=self.sim.yaw_offset,
            noise=self.sim.pos_noise,
            targets=self.targets,
            visible_fn=self._target_visible,
            weapon=self._make_weapon(),
        )
        cfg = nav_config or NavConfig(verbose_log=False, habits_enabled=False)
        self.dt = 1.0 / cfg.tick_hz
        self.time = 0.0
        self.nav = Navigator(self.reader, self.inp, cm, cfg, clock=lambda: self.time)
        self.reader._ladders = self._sim_ladders()
        self._mouse_queue: list[tuple[int, int]] = []
        self.collisions = 0
        self.target_collisions = 0
        self._auto_next = 0.0
        self._scope_was_right = False
        self._zoom_level = 0

    def _sim_ladders(self) -> list[dict]:
        out: list[dict] = []
        for lk in getattr(self.cm, "ladder_links", []) or []:
            lad = dict(lk.get("ladder") or {})
            if "bottom" not in lad:
                lad["bottom"] = lk.get("bottom")
                lad["top"] = lk.get("top")
            if lad.get("bottom") and lad.get("top"):
                out.append(lad)
        return out

    def _make_weapon(self) -> WeaponInfo:
        item_id, name = resolve_weapon(self.sim.weapon)
        if not item_id:
            item_id, name = 7, "ak47"
        clip = self.sim.weapon_clip
        if clip is None:
            clip = default_clip(item_id, name)
        return WeaponInfo(id=item_id, name=name, clip=max(0, int(clip)))

    # -------------------------------------------------------------- targets

    def _target_visible(self, t: SimTarget) -> bool:
        """Прямая видимость по карте (2D): отрезок игрок→мишень не пересекает стену."""
        if not t.alive:
            return False
        p = self.player
        dx, dy = t.x - p.x, t.y - p.y
        d = math.hypot(dx, dy)
        if d > self.sim.target_vis_dist:
            return False
        steps = max(1, int(d / 6.0))
        z = p.z
        for i in range(1, steps + 1):
            k = i / steps
            node = self.cm.layer_at(p.x + dx * k, p.y + dy * k, z, dz_max=self.cm.cfg.crouch_jump_height)
            if node is None:
                return False
            z = node.z
        return True

    def _resolve_shot(self) -> tuple[SimTarget | None, bool]:
        """Луч из глаз + отдача текущей пули: пересечение с овалом головы."""
        p = self.player
        w = self.reader._weapon
        ky, kp = recoil_offset(w.id, w.name, self.reader._shots_fired, 0.0)
        yaw = shortest_yaw_err(p.yaw - ky, 0.0)
        pitch = p.pitch - kp
        fwd = forward_from_angles(pitch, yaw)
        ex, ey, ez = p.x, p.y, p.z + 64.0
        best: SimTarget | None = None
        best_t = float("inf")
        for t in self.targets:
            if not t.alive:
                continue
            hit_t = self._ray_oval_t(ex, ey, ez, fwd.x, fwd.y, fwd.z, t)
            if hit_t is not None and 0.0 < hit_t < best_t:
                best, best_t = t, hit_t
        return best, best is not None

    @staticmethod
    def _ray_oval_t(
        ex: float, ey: float, ez: float,
        fx: float, fy: float, fz: float,
        t: SimTarget,
    ) -> float | None:
        rx, rz = max(0.2, t.rx), max(0.2, t.rz)
        ox, oy, oz = (ex - t.x) / rx, (ey - t.y) / rx, (ez - t.z) / rz
        dx, dy, dz = fx / rx, fy / rx, fz / rz
        a = dx * dx + dy * dy + dz * dz
        if a < 1e-12:
            return None
        b = 2.0 * (ox * dx + oy * dy + oz * dz)
        c = ox * ox + oy * oy + oz * oz - 1.0
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        s = math.sqrt(disc)
        t1 = (-b - s) / (2.0 * a)
        t2 = (-b + s) / (2.0 * a)
        hits = [u for u in (t1, t2) if u > 0.05]
        return min(hits) if hits else None

    def _apply_hit(self, t: SimTarget) -> bool:
        """Попадание: считаем, «убиваем» после hits_to_kill. Возвращает True, если мишень пропала."""
        t.hits += 1
        if t.hits_to_kill > 0 and t.hits >= t.hits_to_kill:
            t.alive = False
            t.hits = 0
            t.respawn_at = self.time + self.sim.target_respawn if self.sim.target_respawn > 0 else float("inf")
            return True
        return False

    def _respawn_targets(self) -> None:
        for t in self.targets:
            if t.alive and t.spawn_immunity and self.time >= t.immune_until:
                t.spawn_immunity = False
                t.vulnerable = True
                t.has_moved_since_spawn = True
            if not t.alive and self.time >= t.respawn_at:
                t.alive = True
                t.respawn_at = float("inf")
                t.spawn_immunity = True
                t.vulnerable = False
                t.has_moved_since_spawn = False
                t.immune_until = self.time + max(0.0, self.sim.spawn_protect)

    def _collide_targets(self) -> None:
        """Мишень — твёрдый цилиндр: выталкиваем игрока, гасим скорость в её сторону."""
        p = self.player
        hull = self.cm.cfg.hull_radius
        for t in self.targets:
            if not t.alive:
                continue
            dx, dy = p.x - t.x, p.y - t.y
            d = math.hypot(dx, dy)
            min_d = hull + t.radius
            if d >= min_d or abs(p.z + 64.0 - t.z) > 40.0:
                continue
            self.target_collisions += 1
            if d < 1e-6:
                dx, dy, d = 1.0, 0.0, 1.0
            nx, ny = dx / d, dy / d
            p.x, p.y = t.x + nx * min_d, t.y + ny * min_d
            into = p.vx * nx + p.vy * ny
            if into < 0:
                p.vx -= into * nx
                p.vy -= into * ny

    # -------------------------------------------------------------- physics

    def _apply_mouse(self) -> None:
        self._mouse_queue.append((self.inp.mouse_dx, self.inp.mouse_dy))
        self.inp.mouse_dx = 0
        self.inp.mouse_dy = 0
        if len(self._mouse_queue) <= self.sim.mouse_delay_ticks:
            return
        dx, dy = self._mouse_queue.pop(0)
        k = self.sim.deg_per_px
        sign = 1.0 if self.sim.invert_mouse else -1.0
        p = self.player
        p.yaw = shortest_yaw_err(p.yaw + sign * dx * k, 0.0)
        p.pitch = max(-89.0, min(89.0, p.pitch + dy * k))

    def _wishdir(self) -> tuple[float, float] | None:
        keys = self.inp.keys
        f = 0.0
        s = 0.0
        if "w" in keys:
            f += 1.0
        if "s" in keys:
            f -= 1.0
        if "a" in keys:
            s += 1.0   # влево = +yaw сторона
        if "d" in keys:
            s -= 1.0
        if f == 0.0 and s == 0.0:
            return None
        yr = math.radians(self.player.yaw)
        fx, fy = math.cos(yr), math.sin(yr)
        lx, ly = -fy, fx  # влево
        wx, wy = fx * f + lx * s, fy * f + ly * s
        m = math.hypot(wx, wy)
        return wx / m, wy / m

    def _move(self) -> None:
        p = self.player
        dt = self.dt
        sim = self.sim
        c = self.cm.cfg
        p.crouched = "ctrl" in self.inp.keys
        speed = math.hypot(p.vx, p.vy)
        on_lad = _player_on_ladder(p.x, p.y, p.z, self.reader._ladders)
        nav_lad = getattr(self.nav, "_ladder", None)
        # если nav не в ladder-режиме и игрок на nav-клетке — считаем, что сошли
        if on_lad and nav_lad is None and self.cm.layer_at(p.x, p.y, p.z, dz_max=4.0) is not None:
            on_lad = False
        # «залипание» лестницы: пока nav явно просит climb/mount, не выпускаем из ladder-режима
        if not on_lad:
            if nav_lad is not None and nav_lad.get("phase") in ("climb", "mount"):
                lad = nav_lad.get("lad") or {}
                bot = lad.get("bottom"); top = lad.get("top")
                if bot and top:
                    tb = min(bot[2], top[2]) + 4.0
                    tt = max(bot[2], top[2]) + 20.0
                    if tb <= p.z <= tt:
                        # сравним дистанцию до оси лестницы
                        bx, by = bot[0], bot[1]
                        tx, ty = top[0], top[1]
                        dx, dy = tx - bx, ty - by
                        seg = dx * dx + dy * dy
                        if seg <= 1.0:
                            d = math.hypot(p.x - bx, p.y - by)
                        else:
                            t = max(0.0, min(1.0, ((p.x - bx) * dx + (p.y - by) * dy) / seg))
                            d = math.hypot(p.x - (bx + dx * t), p.y - (by + dy * t))
                        if d <= 36.0:
                            on_lad = True

        if on_lad:
            self.inp.jump_requested = False
            p.airborne = 0.0
            p.vz = 0.0
            # на лестнице W/S двигают по оси лестницы в зависимости от pitch:
            # pitch > 0 (смотрит вниз) → W = спуск; pitch < 0 (вверх) → W = подъём.
            pitch_rad = math.radians(p.pitch)
            dir_sign = -math.sin(pitch_rad)  # +1 вверх при pitch<0, −1 вниз при pitch>0
            if abs(dir_sign) < 0.25 and nav_lad is not None:
                dir_sign = 1.0 if nav_lad.get("up") else -1.0
            if "w" in self.inp.keys:
                climb = dir_sign * _CS2_LADDER_SPEED
            elif "s" in self.inp.keys:
                climb = -dir_sign * _CS2_LADDER_SPEED
            else:
                climb = 0.0
            p.z += climb * dt
            # ограничение: нельзя пройти ниже нижней площадки / выше верхней;
            # у подножия лестницы телепортируем бот на ближайшую nav-клетку
            lad_src = (nav_lad.get("lad") if nav_lad is not None else None) \
                or _closest_ladder_dict(p.x, p.y, p.z, self.reader._ladders)
            if lad_src is not None:
                bot = lad_src.get("bottom"); top = lad_src.get("top")
                if bot and top:
                    zb = min(bot[2], top[2])
                    zt = max(bot[2], top[2])
                    if p.z <= zb + 2.0:
                        p.z = zb
                        land = self.cm.layer_at(p.x, p.y, zb, dz_max=c.step_height + 8.0)
                        if land is None:
                            best = None
                            best_d = 48.0
                            for n in self.cm.nodes:
                                if abs(n.z - zb) > 24.0:
                                    continue
                                d = math.hypot(n.x - p.x, n.y - p.y)
                                if d < best_d:
                                    best_d = d
                                    best = n
                            land = best
                        if land is not None:
                            p.x = land.x
                            p.y = land.y
                            p.z = land.z
                            p.ground_z = land.z
                            p.vx = p.vy = 0.0
                            return
                    elif p.z > zt + 4.0:
                        p.z = zt + 4.0
            # залипание на оси лестницы: тянем XY к ближайшей точке на линии bot_xy..top_xy
            if nav_lad is not None:
                lad = nav_lad.get("lad") or {}
                bot = lad.get("bottom"); top = lad.get("top")
                if bot and top:
                    bx, by = bot[0], bot[1]
                    tx, ty = top[0], top[1]
                    dx, dy = tx - bx, ty - by
                    seg = dx * dx + dy * dy
                    if seg > 1.0:
                        t = max(0.0, min(1.0, ((p.x - bx) * dx + (p.y - by) * dy) / seg))
                        p.x += (bx + dx * t - p.x) * min(1.0, dt * 8.0)
                        p.y += (by + dy * t - p.y) * min(1.0, dt * 8.0)
                    else:
                        p.x += (bx - p.x) * min(1.0, dt * 8.0)
                        p.y += (by - p.y) * min(1.0, dt * 8.0)
            # лёгкий сайд-страфинг
            if "a" in self.inp.keys or "d" in self.inp.keys:
                import math as _m
                yaw_r = _m.radians(p.yaw)
                side = -_m.sin(yaw_r), _m.cos(yaw_r)
                sgn = 1.0 if "d" in self.inp.keys else -1.0
                p.x += side[0] * sgn * 40.0 * dt
                p.y += side[1] * sgn * 40.0 * dt
            p.vx *= 0.2
            p.vy *= 0.2
            p.ground_z = p.z
            return

        if p.airborne <= 0.0 and self.inp.jump_requested:
            p.vz = sim.jump_impulse
            p.airborne = 1.0
        self.inp.jump_requested = False

        if p.airborne > 0.0:
            p.vz -= sim.gravity * dt
            p.z += p.vz * dt
            climb = c.crouch_jump_height if p.crouched else c.jump_height
            land = self.cm.layer_at(p.x, p.y, p.z, dz_max=max(12.0, climb))
            if p.vz <= 8.0 and land is not None and p.z <= land.z + 3.0:
                up = land.z - p.ground_z
                if up <= 0.2 or (p.crouched and up <= c.crouch_jump_height + 2.0) or (
                    not p.crouched and up <= c.jump_height + 2.0
                ):
                    p.z = land.z
                    p.vz = 0.0
                    p.airborne = 0.0
                    p.ground_z = land.z
            elif p.z < p.ground_z - c.drop_height:
                fall = self.cm.layer_at(p.x, p.y, p.z, dz_max=c.drop_height)
                if fall is not None:
                    p.z = fall.z
                    p.vz = 0.0
                    p.airborne = 0.0
                    p.ground_z = fall.z

        if speed > 0.1 and p.airborne <= 0.0:
            control = max(speed, sim.stop_speed)
            drop = control * sim.friction * dt
            new_speed = max(0.0, speed - drop)
            p.vx *= new_speed / speed
            p.vy *= new_speed / speed

        wish = self._wishdir()
        if wish is not None:
            wx, wy = wish
            cur = p.vx * wx + p.vy * wy
            add = sim.max_speed - cur
            accel_k = sim.accelerate if p.airborne <= 0.0 else 1.2
            if add > 0:
                accel = min(add, accel_k * sim.max_speed * dt)
                p.vx += wx * accel
                p.vy += wy * accel
            spd = math.hypot(p.vx, p.vy)
            cap = sim.max_speed
            if spd > cap:
                p.vx *= cap / spd
                p.vy *= cap / spd

        nx, ny = p.x + p.vx * dt, p.y + p.vy * dt
        if p.airborne > 0.0:
            dz_max = c.crouch_jump_height if p.crouched else c.jump_height
            ref_z = p.ground_z
        else:
            dz_max = c.step_height
            ref_z = p.z
        node = self.cm.layer_at(nx, ny, ref_z, dz_max)
        if node is None and p.airborne <= 0.0:
            # ступеньки нет вверх — может быть drop вниз (шаг за край)
            drop = self.cm.layer_at(nx, ny, ref_z, dz_max=c.drop_height)
            if drop is not None and drop.z < ref_z - c.step_height:
                # падаем: включаем airborne и уводим z вниз
                p.airborne = 1.0
                p.vz = 0.0
                node = drop
                ref_z = drop.z
        if node is not None:
            p.x, p.y = nx, ny
            if p.airborne <= 0.0:
                p.z += (node.z - p.z) * min(1.0, dt * 12.0)
                p.ground_z = node.z
            return
        self.collisions += 1
        node = self.cm.layer_at(nx, p.y, ref_z, dz_max)
        if node is None and p.airborne <= 0.0:
            drop = self.cm.layer_at(nx, p.y, ref_z, dz_max=c.drop_height)
            if drop is not None and drop.z < ref_z - c.step_height:
                p.airborne = 1.0
                p.vz = 0.0
                node = drop
        if node is not None:
            p.x = nx
            p.vy = 0.0
            if p.airborne <= 0.0:
                p.z += (node.z - p.z) * min(1.0, dt * 12.0)
                p.ground_z = node.z
            return
        node = self.cm.layer_at(p.x, ny, ref_z, dz_max)
        if node is None and p.airborne <= 0.0:
            drop = self.cm.layer_at(p.x, ny, ref_z, dz_max=c.drop_height)
            if drop is not None and drop.z < ref_z - c.step_height:
                p.airborne = 1.0
                p.vz = 0.0
                node = drop
        if node is not None:
            p.y = ny
            p.vx = 0.0
            if p.airborne <= 0.0:
                p.z += (node.z - p.z) * min(1.0, dt * 12.0)
                p.ground_z = node.z
            return
        p.vx = p.vy = 0.0

    def _poll_weapon_fire(self) -> None:
        """Автомат даёт выстрелы, пока ЛКМ зажата; тап — только по фронту нажатия."""
        w = self.reader._weapon
        held = "left" in self.inp.buttons
        if held:
            self.inp.left_hold_ticks += 1
        auto = weapon_is_auto(w.id, w.name)
        if auto and held:
            cycle = weapon_cycle_s(w.id, w.name)
            if self.inp.shot_requested:
                self._auto_next = self.time + cycle
                if w.clip > 0:
                    w.clip -= 1
            elif w.clip > 0 and self._auto_next > 0.0 and self.time + 1e-9 >= self._auto_next:
                self.inp.shot_requested = True
                w.clip -= 1
                self._auto_next = self.time + cycle
        elif (not auto) and self.inp.shot_requested and w.clip > 0:
            w.clip -= 1
        if not held:
            self._auto_next = 0.0
            if self.reader._shots_fired and self.reader._shots_idle_at is None:
                self.reader._shots_idle_at = self.time
            rec = spray_recover_s(w.id, w.name)
            if (
                self.reader._shots_idle_at is not None
                and self.time - self.reader._shots_idle_at >= rec
            ):
                self.reader._shots_fired = 0
                self.reader._shots_idle_at = None
        else:
            self.reader._shots_idle_at = None

    def _poll_scope(self) -> None:
        """ПКМ-клик циклит зум снайперки: 0 → 1 → 2 → 0. Отпускание само по себе не выходит."""
        w = self.reader._weapon
        right = "right" in self.inp.buttons
        if not weapon_needs_scope(w.id, w.name):
            self._zoom_level = 0
            self.reader._scoped = False
        elif right and not self._scope_was_right:
            self._zoom_level = (self._zoom_level + 1) % 3
            self.reader._scoped = self._zoom_level > 0
        self._scope_was_right = right
        if weapon_needs_scope(w.id, w.name):
            self.reader._scoped = self._zoom_level > 0

    # -------------------------------------------------------------- run

    def run(self, duration: float = 60.0, log_every: int = 40, goal_id: int | None = None) -> SimResult:
        res = SimResult()
        p = self.player
        self.reader.latch()
        self.nav.plan((p.x, p.y, p.z), goal_id=goal_id)
        assert self.nav.route is not None
        res.route = list(self.nav.route.pts)
        res.route_len = self.nav.route.length
        res.goal = self.nav.route.goal
        res.walked.append((p.x, p.y, p.z))
        yaw_rates: list[float] = []
        moving_rates: list[float] = []
        aim_errs: list[float] = []
        settle_times: list[float] = []
        first_shot_delays: list[float] = []
        shot_speeds: list[float] = []
        kill_times: list[float] = []
        kill_events: list[str] = []
        engage_start: float | None = None
        seen_at: float | None = None
        prev_target = None
        prev_yaw = p.yaw
        prev_srate = 0.0
        prev_acc = 0.0
        accels: list[float] = []
        jerks: list[float] = []
        curv: list[float] = []
        sign_flips = 0
        aim_prev = (p.yaw, p.pitch)
        aim_prev_step: tuple[float, float] | None = None
        max_ticks = int(duration / self.dt)
        heading_every = max(1, int(0.5 / self.dt))
        aim_every = max(1, int(0.25 / self.dt))
        res.targets = [(t.x, t.y, t.z) for t in self.targets]

        for tick in range(max_ticks):
            self.time = tick * self.dt
            self.reader.latch()
            info = self.nav.tick()
            self._poll_scope()
            self._poll_weapon_fire()
            # выстрел — по состоянию камеры на момент нажатия (мышь этого тика ещё не применена)
            if self.inp.shot_requested:
                self.inp.shot_requested = False
                spd_now = math.hypot(p.vx, p.vy)
                hit_t, hit = self._resolve_shot()
                self.reader._shots_fired += 1
                self.reader._shots_idle_at = None
                w = self.reader._weapon
                if weapon_needs_scope(w.id, w.name) and not weapon_is_auto(w.id, w.name):
                    self._zoom_level = 0
                    self.reader._scoped = False
                res.shots += 1
                res.hits += int(hit)
                shot_speeds.append(spd_now)
                tgt_now = info.get("target")
                if tgt_now is not None:
                    res.shot_lines.append((p.x, p.y, tgt_now[0], tgt_now[1], hit))
                if info.get("shots_on_target") == 1 and seen_at is not None:
                    first_shot_delays.append(self.time - seen_at)
                if hit and hit_t is not None and self._apply_hit(hit_t):
                    res.targets_killed += 1
                    if seen_at is not None:
                        kill_times.append(self.time - seen_at)
                    kill_events.append(f"t={self.time:.1f}s {hit_t.name} пропала (попаданий {hit_t.hits_to_kill})")
            self._respawn_targets()
            self._apply_mouse()
            self._move()
            self._collide_targets()
            res.ticks += 1
            if info.get("firing"):
                res.fire_ticks += 1

            tgt = info.get("target")
            if tgt is not None:
                # ошибка прицела по факту (после применения мыши), yaw игрока → мишень
                want = math.degrees(math.atan2(tgt[1] - p.y, tgt[0] - p.x))
                err = abs(shortest_yaw_err(want, p.yaw))
                res.flick_peak_rate = max(res.flick_peak_rate, abs(shortest_yaw_err(p.yaw, prev_yaw)) / self.dt)
                if tgt != prev_target:
                    engage_start = self.time
                    seen_at = self.time
                elif engage_start is not None and err < 3.0:
                    settle_times.append(self.time - engage_start)
                    engage_start = None
                if engage_start is None:
                    aim_errs.append(err)
                if tick % aim_every == 0:
                    res.aim_lines.append((p.x, p.y, tgt[0], tgt[1]))
            else:
                engage_start = None
            prev_target = tgt

            res.walked.append((p.x, p.y, p.z))
            res.walked_len += math.hypot(p.x - res.walked[-2][0], p.y - res.walked[-2][1])
            if tick % heading_every == 0:
                res.headings.append((p.x, p.y, p.yaw))
            cl = self.cm.clearance_at(p.x, p.y, p.z)
            res.min_clearance = min(res.min_clearance, cl)
            if cl < 12.0:
                res.low_clearance_ticks += 1
            spd = math.hypot(p.vx, p.vy)
            if tick > int(1.5 / self.dt) and spd < 40.0 and info.get("action") not in ("done", "arrived", "pause"):
                res.stop_ticks += 1
            srate = shortest_yaw_err(p.yaw, prev_yaw) / self.dt
            rate = abs(srate)
            prev_yaw = p.yaw
            yaw_rates.append(rate)
            if spd > 100.0:
                moving_rates.append(rate)
            # плавность: ускорение и рывок по yaw, смены знака скорости
            acc = (srate - prev_srate) / self.dt
            accels.append(abs(acc))
            jerks.append((acc - prev_acc) / self.dt)
            if abs(srate) > 4.0 and abs(prev_srate) > 4.0 and (srate > 0) != (prev_srate > 0):
                sign_flips += 1
            prev_srate, prev_acc = srate, acc
            # кривизна траектории прицела при наводке: угол между соседними шагами (yaw, pitch)
            if tgt is not None and rate > 40.0:
                step = (shortest_yaw_err(p.yaw, aim_prev[0]), p.pitch - aim_prev[1])
                if aim_prev_step is not None:
                    a = math.hypot(*step)
                    b = math.hypot(*aim_prev_step)
                    if a > 0.3 and b > 0.3:
                        cosang = (step[0] * aim_prev_step[0] + step[1] * aim_prev_step[1]) / (a * b)
                        curv.append(math.degrees(math.acos(max(-1.0, min(1.0, cosang)))))
                aim_prev_step = step
            else:
                aim_prev_step = None
            aim_prev = (p.yaw, p.pitch)

            if log_every and tick % log_every == 0:
                res.log.append(
                    f"t={tick * self.dt:5.1f}s pos=({p.x:.0f},{p.y:.0f},{p.z:.0f}) spd={spd:3.0f} "
                    f"yaw={p.yaw:6.1f} keys={info.get('keys')} err={info.get('yaw_err')} "
                    f"rel={info.get('rel')} s={info.get('s')}/{info.get('route_len')} goal_d={info.get('goal_d')}"
                )
            if info.get("action") == "done":
                res.done = True
                break

        res.time = res.ticks * self.dt
        if res.goal:
            res.final_goal_dist = math.hypot(res.goal[0] - p.x, res.goal[1] - p.y)
        yaw_rates.sort()
        moving_rates.sort()
        res.max_yaw_rate = yaw_rates[-1] if yaw_rates else 0.0
        res.yaw_rate_p95 = yaw_rates[int(len(yaw_rates) * 0.95)] if yaw_rates else 0.0
        res.moving_yaw_rate_p95 = moving_rates[int(len(moving_rates) * 0.95)] if moving_rates else 0.0
        res.key_events = self.inp.key_events
        res.stucks = self.nav.stucks
        res.replans = self.nav.replans
        res.collisions = self.collisions
        res.yaw_offset_detected = self.nav.yaw_offset
        res.yaw_per_px_final = self.nav.cfg.yaw_per_px
        res.events = list(self.nav.events)
        res.engaged_ticks = self.nav.engaged_ticks
        res.targets_engaged = self.nav.targets_engaged
        res.target_collisions = self.target_collisions
        aim_errs.sort()
        settle_times.sort()
        res.aim_err_p50 = aim_errs[len(aim_errs) // 2] if aim_errs else 0.0
        res.aim_settle_time = settle_times[len(settle_times) // 2] if settle_times else 0.0
        first_shot_delays.sort()
        shot_speeds.sort()
        res.first_shot_delay = first_shot_delays[len(first_shot_delays) // 2] if first_shot_delays else 0.0
        res.move_speed_at_shot = shot_speeds[len(shot_speeds) // 2] if shot_speeds else 0.0
        kill_times.sort()
        res.kill_time = kill_times[len(kill_times) // 2] if kill_times else 0.0
        res.events.extend(kill_events)
        accels.sort()
        res.yaw_accel_p95 = accels[int(len(accels) * 0.95)] if accels else 0.0
        res.yaw_jerk_rms = math.sqrt(sum(j * j for j in jerks) / len(jerks)) if jerks else 0.0
        res.rate_sign_flips_per_s = sign_flips / max(1e-6, res.time)
        res.path_curvature = sum(curv) / len(curv) if curv else 0.0
        return res
