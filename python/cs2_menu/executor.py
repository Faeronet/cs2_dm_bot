"""Исполнение UI: MenuInput (uinput + xdotool курсор)."""

from __future__ import annotations

from typing import Callable

from cs2_control.input import GameInput
from cs2_control.menu_input import MenuInput
from cs2_menu.geometry import WindowLayout
from cs2_menu.layout import LAYOUT
from cs2_menu.models import ActionKind, UIAction, UIState


class UIExecutor:
    def __init__(
        self,
        *,
        dry_run: bool = False,
        log: Callable[[str], None] | None = None,
        game_input: GameInput | None = None,
        menu_input: MenuInput | None = None,
    ):
        self.dry_run = dry_run
        self.log = log or (lambda _msg: None)
        self._menu = menu_input or MenuInput(game_input, log=self.log)
        self._owns_menu = menu_input is None

    @property
    def menu_input(self) -> MenuInput:
        return self._menu

    def close(self) -> None:
        if self._owns_menu:
            self._menu.close()

    def refresh_window(self) -> bool:
        return self._menu.refresh_window()

    def layout(self, state: UIState | None = None) -> WindowLayout | None:
        geo = self._menu.geometry()
        if not geo.get("width"):
            if not self.refresh_window():
                return None
            geo = self._menu.geometry()
        cw, ch = (1280, 800)
        if state and state.window_memory[0] > 0:
            cw, ch = state.window_memory
        return WindowLayout.from_executor_geo(geo, client_w=cw, client_h=ch)

    def _xdo_env(self) -> dict[str, str]:
        return self._menu._xdo_env()

    @property
    def _window_id(self) -> int | None:
        return self._menu._window_id

    def run_actions(self, actions: list[UIAction], state: UIState | None = None) -> None:
        for act in actions:
            for _ in range(max(1, act.repeat)):
                self._run_one(act, state)

    def click_target(self, target: str, state: UIState | None = None) -> None:
        self._run_one(UIAction.click(target), state)

    def move_target(self, target: str, state: UIState | None = None) -> None:
        pt = LAYOUT.get(target)
        layout = self.layout(state)
        if not pt or not layout:
            return
        wx, wy = layout.norm_to_window_rel(pt)
        cx, cy = layout.norm_to_client(pt)
        self.log(f"move {target} client=({cx},{cy}) window=({wx},{wy})")
        if self.dry_run:
            return
        self._menu.focus()
        self._menu.move_window_rel(wx, wy)

    def key(self, name: str, *, repeat: int = 1, state: UIState | None = None) -> None:
        self._run_one(UIAction.key(name, repeat=repeat), state)

    def _run_one(self, act: UIAction, state: UIState | None) -> None:
        import time

        if act.kind == ActionKind.WAIT:
            self.log(f"wait {act.hold_ms}ms")
            if not self.dry_run:
                time.sleep(max(0, act.hold_ms) / 1000.0)
            return

        if act.kind == ActionKind.KEY:
            self.log(f"key {act.target} x{act.repeat}")
            if self.dry_run:
                return
            self._menu.key_tap(act.target, repeat=act.repeat)
            return

        if act.kind == ActionKind.TYPE_TEXT:
            self.log(f"type {act.target!r}")
            if self.dry_run:
                return
            self._menu.type_text(act.target)
            return

        if act.kind == ActionKind.CLICK:
            pt = LAYOUT.get(act.target)
            if not pt:
                self.log(f"unknown click target {act.target}")
                return
            layout = self.layout(state)
            if not layout:
                self.log(f"click {act.target}: нет геометрии окна")
                return
            wx, wy = layout.norm_to_window_rel(pt)
            cx, cy = layout.norm_to_client(pt)
            in_client = layout.window_rel_in_client(wx, wy)
            self.log(
                f"click {act.target} client=({cx},{cy}) window=({wx},{wy}) "
                f"res={layout.client_w}x{layout.client_h} "
                f"{'OK' if in_client else 'OUT_OF_CLIENT'}"
            )
            if self.dry_run:
                return
            self._menu.click_window_rel(wx, wy, label=act.target)
