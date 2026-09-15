"""Передвижение + чтение прицела из памяти (без навигации)."""

from __future__ import annotations

import time
from dataclasses import dataclass

from cs2_control.input import GameInput
from cs2_reader.game_state import Vec3
from cs2_reader.reader import Reader


@dataclass
class AimState:
    position: Vec3
    pitch: float
    yaw: float
    forward: Vec3
    aim_point: Vec3


class Movement:
    """WASD и мышь; на каждом шаге читает, куда смотрит прицел."""

    def __init__(
        self,
        reader: Reader,
        inp: GameInput,
        poll_hz: float = 20.0,
        deg_per_px: float = 0.08,
    ):
        self.reader = reader
        self.input = inp
        self.poll_interval = 1.0 / max(1.0, poll_hz)
        self.deg_per_px = deg_per_px

    def read_aim(self) -> AimState:
        pos = self.reader.local_position()
        eye = self.reader.local_eye_angles()
        return AimState(
            position=pos,
            pitch=eye.x,
            yaw=eye.y,
            forward=self.reader.local_aim_forward(),
            aim_point=self.reader.local_aim_point(),
        )

    def log_aim(self, tag: str = "") -> AimState:
        st = self.read_aim()
        p, a = st.position, st.aim_point
        prefix = f"[{tag}] " if tag else ""
        print(
            f"{prefix}pos=({p.x:.0f},{p.y:.0f},{p.z:.0f}) "
            f"yaw={st.yaw:.1f} pitch={st.pitch:.1f} "
            f"aim→({a.x:.0f},{a.y:.0f},{a.z:.0f})"
        )
        return st

    def _hold_key(self, key: str, seconds: float, tag: str) -> None:
        self.input.key_down(key)
        end = time.time() + seconds
        step = 0
        try:
            while time.time() < end:
                if step % 5 == 0:
                    self.log_aim(tag)
                step += 1
                time.sleep(self.poll_interval)
        finally:
            self.input.key_up(key)

    def walk_key(self, key: str, seconds: float) -> None:
        print(f"→ {key.upper()} {seconds:.1f}s")
        self.log_aim(f"start_{key}")
        self._hold_key(key, seconds, key)

    def cross(self, leg_seconds: float = 1.2) -> None:
        print("=== Крестик W→D→S→A ===")
        for key in ("w", "d", "s", "a"):
            self.walk_key(key, leg_seconds)
        self.log_aim("cross_done")

    def spin_mouse(self, steps: int = 4, yaw_deg: float = 90.0) -> None:
        print(f"=== Поворот мыши: {steps}×{yaw_deg:.0f}° ===")
        for i in range(steps):
            print(f"  шаг {i + 1}/{steps}")
            self.input.mouse_look_deg(yaw_deg, 0.0, self.deg_per_px)
            time.sleep(0.15)
            self.log_aim(f"spin_{i + 1}")

    def mouse_jerk(self, dx: int, dy: int) -> None:
        self.input.mouse_move(dx, dy)
        time.sleep(0.05)
        self.log_aim("mouse")

    def circle(
        self,
        seconds: float = 6.0,
        turn_px: int = 18,
        tick: float = 0.05,
    ) -> None:
        """Круг: W + резкие дёрганья мыши вправо."""
        print(f"=== Круг {seconds:.0f}s (W + mouse +{turn_px}px) ===")
        self.log_aim("circle_start")
        self.input.key_down("w")
        end = time.time() + seconds
        step = 0
        try:
            while time.time() < end:
                self.input.mouse_move(turn_px, 0)
                if step % 8 == 0:
                    self.log_aim("circle")
                step += 1
                time.sleep(tick)
        finally:
            self.input.key_up("w")
        self.log_aim("circle_done")

    def demo(self, leg: float = 1.2, circle_sec: float = 6.0) -> None:
        self.cross(leg)
        time.sleep(0.3)
        self.spin_mouse(steps=4, yaw_deg=90.0)
        time.sleep(0.3)
        self.circle(seconds=circle_sec)

    def stop(self) -> None:
        self.input.release_all()
