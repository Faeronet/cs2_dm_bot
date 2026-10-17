"""Геометрия окна CS2: client area из памяти vs внешний X11-фрейм."""

from __future__ import annotations

from dataclasses import dataclass

from cs2_menu.models import NormPoint


@dataclass(frozen=True)
class WindowLayout:
    """Client area (из памяти игры) внутри внешнего X11-окна."""

    outer_x: int
    outer_y: int
    outer_w: int
    outer_h: int
    client_w: int
    client_h: int

    @property
    def letterbox_x(self) -> int:
        return max(0, (self.outer_w - self.client_w) // 2)

    @property
    def letterbox_y(self) -> int:
        return max(0, (self.outer_h - self.client_h) // 2)

    @property
    def client_x(self) -> int:
        return self.outer_x + self.letterbox_x

    @property
    def client_y(self) -> int:
        return self.outer_y + self.letterbox_y

    def client_size(self) -> tuple[int, int]:
        return self.client_w, self.client_h

    def norm_to_client(self, pt: NormPoint) -> tuple[int, int]:
        """Пиксели в client area 0..client_w-1, 0..client_h-1."""
        cx = int(round(pt.x * max(1, self.client_w - 1)))
        cy = int(round(pt.y * max(1, self.client_h - 1)))
        cx = max(0, min(self.client_w - 1, cx))
        cy = max(0, min(self.client_h - 1, cy))
        return cx, cy

    def norm_to_window_rel(self, pt: NormPoint) -> tuple[int, int]:
        """Координаты для xdotool mousemove --window (относительно внешнего окна)."""
        cx, cy = self.norm_to_client(pt)
        wx = self.letterbox_x + cx
        wy = self.letterbox_y + cy
        wx = max(0, min(self.outer_w - 1, wx))
        wy = max(0, min(self.outer_h - 1, wy))
        return wx, wy

    def norm_to_screen_abs(self, pt: NormPoint) -> tuple[int, int]:
        cx, cy = self.norm_to_client(pt)
        return self.client_x + cx, self.client_y + cy

    def window_rel_in_client(self, wx: int, wy: int) -> bool:
        lb_x, lb_y = self.letterbox_x, self.letterbox_y
        return lb_x <= wx < lb_x + self.client_w and lb_y <= wy < lb_y + self.client_h

    @classmethod
    def from_executor_geo(
        cls,
        geo: dict[str, int],
        *,
        client_w: int,
        client_h: int,
    ) -> WindowLayout:
        cw = client_w or geo.get("width", 1280)
        ch = client_h or geo.get("height", 800)
        return cls(
            outer_x=int(geo.get("x", 0)),
            outer_y=int(geo.get("y", 0)),
            outer_w=int(geo.get("width", cw)),
            outer_h=int(geo.get("height", ch)),
            client_w=int(cw),
            client_h=int(ch),
        )

    def describe(self) -> dict:
        return {
            "outer": (self.outer_x, self.outer_y, self.outer_w, self.outer_h),
            "client_memory": (self.client_w, self.client_h),
            "client_screen": (self.client_x, self.client_y),
            "letterbox": (self.letterbox_x, self.letterbox_y),
        }
