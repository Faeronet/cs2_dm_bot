"""MenuController — читает UI из памяти и исполняет сценарии."""

from __future__ import annotations

import time
from typing import Callable

from cs2_control.input import GameInput
from cs2_control.menu_input import MenuInput
from cs2_menu.driver import UIStateReader
from cs2_menu.executor import UIExecutor
from cs2_menu.models import MenuPhase, UIState
from cs2_menu.popups import POPUP_SIGNATURES, detect_popups, save_fingerprint
from cs2_menu.scenarios import SCENARIOS, Scenario, scenario_success
from cs2_menu.vision import (
    capture_window_xwd,
    expected_screen_for_target,
    identify_from_xwd,
    learn_screen,
    screens_differ,
)
from cs2_reader.reader import Reader

_XWD_PATH = "/tmp/cs2_menu_cap.xwd"


class MenuController:
    """
    API для бота:

        menu = MenuController(reader)
        menu.run("deathmatch")
        menu.ensure("main_menu")
        menu.dismiss_popups()
    """

    def __init__(
        self,
        reader: Reader,
        *,
        dry_run: bool = False,
        log: Callable[[str], None] | None = None,
        step_delay: float = 0.35,
        game_input: GameInput | None = None,
        menu_input: MenuInput | None = None,
    ):
        self.reader = reader
        self.dry_run = dry_run
        self.log = log or print
        self.step_delay = step_delay
        self.driver = UIStateReader(reader)
        self._menu_input = menu_input or MenuInput(game_input, log=self.log)
        if not dry_run:
            self._menu_input.prepare()
        self.executor = UIExecutor(
            dry_run=dry_run,
            log=self.log,
            game_input=game_input,
            menu_input=self._menu_input,
        )
        self._phase_fp: dict[str, str] = {}
        self._use_vision = not dry_run

    def close(self) -> None:
        self.executor.close()

    def read(self, *, vision: bool | None = None) -> UIState:
        state = self.driver.read()
        popups = detect_popups(state, min_conf=0.5)
        state.active_popups = [p.id for p, _c in popups]
        if state.phase != MenuPhase.UNKNOWN:
            self._phase_fp[state.phase.value] = state.ui_fingerprint
        if vision if vision is not None else self._use_vision:
            self._apply_vision(state)
        return state

    def _apply_vision(self, state: UIState) -> None:
        self.executor.refresh_window()
        wid = self.executor._window_id
        layout = self.executor.layout(state)
        if not wid or not layout:
            return
        if not capture_window_xwd(wid, _XWD_PATH, self.executor._xdo_env()):
            return
        sig = identify_from_xwd(_XWD_PATH, layout)
        state.region_hashes = sig.region_hashes
        state.screen_id = sig.screen_id
        state.screen_confidence = sig.confidence
        if sig.debug:
            state.raw["screen_debug"] = sig.debug

    def where(self) -> UIState:
        """Текущий экран меню (screen_id) + геометрия."""
        return self.read(vision=True)

    def learn_current_screen(self, screen_id: str) -> dict[str, str]:
        state = self.read(vision=True)
        if not state.region_hashes:
            raise RuntimeError("не удалось снять регионы экрана (xwd)")
        learn_screen(screen_id, state.region_hashes)
        return dict(state.region_hashes)

    def click_target(
        self,
        target: str,
        *,
        expected_screen: str | None = None,
        timeout: float = 4.0,
    ) -> bool:
        """Клик с проверкой: экран изменился и (опционально) совпал с expected."""
        expected = expected_screen or expected_screen_for_target(target)
        before = self.read(vision=True)

        self.executor.click_target(target, before)
        deadline = time.monotonic() + timeout
        after = before
        while time.monotonic() < deadline:
            time.sleep(self.step_delay)
            after = self.read(vision=True)
            if after.screen_id != before.screen_id and after.screen_confidence >= 0.55:
                if not expected or after.screen_id == expected:
                    self.log(f"экран {before.screen_id} -> {after.screen_id}")
                    return True
            if expected and after.screen_id == expected and after.screen_confidence >= 0.55:
                self.log(f"экран {before.screen_id} -> {after.screen_id}")
                return True
            if screens_differ(before.region_hashes, after.region_hashes):
                self.log(
                    f"UI изменился {before.screen_id} -> {after.screen_id}"
                    + (f" (ждали {expected})" if expected else "")
                )
                if not expected:
                    return True
                if after.screen_id == expected:
                    return True
                if after.screen_id != before.screen_id:
                    return True
                # патчи изменились — клик сработал, vision не распознал экран
                if target.startswith(("practice_", "mode_")):
                    return True
        self.log(
            f"клик {target}: остались на {before.screen_id} "
            f"(ожидали {expected}, сейчас {after.screen_id})"
        )
        return False

    def dismiss_popups(self, *, max_rounds: int = 3) -> int:
        """Закрыть popups. Возвращает число успешных dismiss-раундов."""
        closed = 0
        if self.read().phase.value in ("loading", "in_game"):
            return 0
        self.executor.refresh_window()
        for round_i in range(max_rounds):
            state = self.read()
            found = detect_popups(state, min_conf=0.55)
            if not found:
                break
            sig, conf = found[0]
            self.log(f"popup [{sig.id}] conf={conf:.2f} — dismiss")
            before = state.ui_fingerprint
            self.executor.run_actions(sig.dismiss, state)
            time.sleep(self.step_delay)
            after = self.read()
            if after.ui_fingerprint != before:
                save_fingerprint(sig.id, before)
                closed += 1
            else:
                # всё равно считаем попытку
                closed += 1
                if sig.id == "fallback_dismiss":
                    break
        return closed

    def ensure(self, target: str, *, timeout: float = 90.0) -> bool:
        return self.run(target, timeout=timeout)

    def run(self, scenario_id: str, *, timeout: float = 120.0) -> bool:
        scenario = SCENARIOS.get(scenario_id)
        if not scenario:
            raise KeyError(f"unknown scenario: {scenario_id}")
        return self._run_scenario(scenario, timeout=timeout)

    def queue_deathmatch(self) -> bool:
        """Меню: Play → Deathmatch → GO."""
        self.dismiss_popups()
        for _ in range(2):
            self.executor.key("escape", repeat=2)
            time.sleep(0.25)
        for target in ("nav_play", "mode_deathmatch", "go_button"):
            self.executor.click_target(target, self.read())
            time.sleep(0.65)
        return True

    def _play_hub_ready(self, state: UIState) -> bool:
        dbg = state.raw.get("screen_debug") or {}
        return state.screen_id == "play_hub" or (dbg.get("casual_luma") or 0) > 85

    def ensure_play_hub(self, *, max_tries: int = 8) -> bool:
        """Главное меню с сеткой режимов (Play hub)."""
        self.dismiss_popups()
        for i in range(max_tries):
            state = self.read(vision=True)
            if self._play_hub_ready(state):
                self.log(f"play hub ready ({state.screen_id})")
                return True
            if state.screen_id in (
                "news",
                "store",
                "inventory",
                "matchmaking",
                "mode_deathmatch",
                "mode_premier",
                "mode_wingman",
                "mode_practice",
                "unknown",
            ):
                self.log(f"escape из {state.screen_id}")
                self.executor.key("escape", repeat=2)
                time.sleep(0.35)
            self.log(f"→ nav_play (попытка {i + 1})")
            self.executor.click_target("nav_play", state)
            time.sleep(0.85)
        return self._play_hub_ready(self.read(vision=True))

    def queue_training_mirage(self) -> bool:
        """Меню: Play → Тренировка → Mirage → Соревновательный → Старт."""
        from cs2_menu.layout import TRAINING_MIRAGE_CLICKS
        from cs2_menu.vision import expected_screen_for_target

        if not self.ensure_play_hub():
            self.log("FAIL: не удалось открыть Play hub")
            return False

        steps = [t for t in TRAINING_MIRAGE_CLICKS if t != "nav_play"]
        step_pause = {
            "mode_practice": 1.1,
            "practice_map_mirage": 0.55,
            "practice_mode_competitive": 0.45,
            "practice_start": 0.75,
        }
        for target in steps:
            expected = expected_screen_for_target(target)
            ok = False
            for attempt in range(3):
                self.log(f"→ {target} (попытка {attempt + 1})")
                if self.click_target(target, expected_screen=expected, timeout=6.0):
                    ok = True
                    break
                time.sleep(0.35)
            if not ok:
                self.log(f"FAIL: не удалось перейти через {target}")
                return False
            time.sleep(step_pause.get(target, 0.5))
        return True

    def load_map_console(self, map_name: str = "de_mirage") -> bool:
        """Надёжный путь: ` map <name>` через консоль (~)."""
        self.dismiss_popups()
        for _ in range(2):
            self.executor.key("escape", repeat=2)
            time.sleep(0.2)
        self.executor.key("grave")
        time.sleep(0.3)
        self.executor.run_actions(
            [__import__("cs2_menu.models", fromlist=["UIAction"]).UIAction.type_text(f"map {map_name}")],
            self.read(),
        )
        time.sleep(0.15)
        self.executor.key("return")
        self.log(f"консоль: map {map_name}")
        return self.wait_in_game(timeout=120.0, map_id=map_name)

    def wait_in_game(self, *, timeout: float = 120.0, map_id: str | None = None) -> bool:
        """Ждать загрузку в матч (pawn + карта)."""
        want = (map_id or "").lower().replace("maps/", "")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snap = self.reader.capture()
            cur = self.normalize_map(self.reader.map_name() or "")
            if want and want in cur and self.reader.local_pawn():
                return True
            if snap.in_game and self.reader.local_pawn():
                return True
            if want and want in cur:
                self.log(f"карта {cur} загружена, ждём пешку…")
            state = self.read()
            if state.phase == MenuPhase.TEAM_SELECT:
                self.click_target("team_ct", timeout=8.0)
            elif state.phase == MenuPhase.ACCEPT_MATCH:
                self.click_target("accept_match", timeout=8.0)
            self.dismiss_popups()
            time.sleep(0.8)
        return False

    @staticmethod
    def normalize_map(name: str) -> str:
        n = (name or "").strip().lower()
        for prefix in ("maps/", "de_"):
            if n.startswith(prefix) and prefix == "maps/":
                n = n[5:]
        if n.startswith("de_"):
            return n.split(".")[0]
        return n

    def current_map(self) -> str:
        return self.normalize_map(self.reader.map_name() or "")

    def start_deathmatch(self, *, timeout: float = 180.0) -> bool:
        """Из любого состояния → deathmatch queue / карта."""
        deadline = time.monotonic() + timeout
        self.executor.refresh_window()
        loading_stuck = 0

        while time.monotonic() < deadline:
            state = self.read()
            self.log(f"phase={state.phase.value} map={state.map_name!r}")

            if state.in_map() and self.reader.local_pawn():
                self.log("уже в игре")
                return True

            if self.reader.local_pawn() and (state.in_game or self.reader.map_name()):
                return True

            self.dismiss_popups()
            state = self.read()

            if state.phase in (MenuPhase.MATCHMAKING, MenuPhase.LOADING) or (
                state.matchmaking_map and state.matchmaking_map not in ("", "<empty>")
            ):
                self.log("ожидание загрузки / матчмейкинг…")
                if self.wait_in_game(timeout=min(25, deadline - time.monotonic())):
                    return True
                loading_stuck += 1
                if loading_stuck >= 3:
                    self.log("застряли в loading — Escape и заново в меню")
                    self.executor.key("escape", repeat=4)
                    loading_stuck = 0
                continue

            loading_stuck = 0

            if state.in_map():
                return True

            if state.in_game:
                self.run("disconnect", timeout=min(30, deadline - time.monotonic()))
                continue

            if not self.queue_deathmatch():
                self.dismiss_popups()
                time.sleep(0.5)
                continue

            if self.wait_in_game(timeout=min(90, deadline - time.monotonic())):
                return True

            time.sleep(0.5)

        self.log("timeout")
        return False

    def ensure_map(self, map_id: str, *, timeout: float = 600.0) -> bool:
        """Deathmatch пока не загрузится нужная карта (DM — случайная)."""
        map_id = map_id.lower().replace("maps/", "")
        if not map_id.startswith("de_"):
            map_id = f"de_{map_id}"
        deadline = time.monotonic() + timeout
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            self.log(f"=== попытка {attempt}: нужна {map_id} ===")
            if not self.start_deathmatch(timeout=min(180, deadline - time.monotonic())):
                continue
            cur = self.current_map()
            self.log(f"загружена карта: {cur!r}")
            if cur == map_id or map_id in cur:
                return True
            self.log(f"не {map_id}, выходим и перезаходим…")
            self.run("disconnect", timeout=30)
            time.sleep(2.0)
        return False

    def _run_scenario(self, scenario: Scenario, *, timeout: float) -> bool:
        if scenario.id == "dismiss_popups":
            return self.dismiss_popups() > 0

        deadline = time.monotonic() + timeout
        self.executor.refresh_window()

        while time.monotonic() < deadline:
            state = self.read()
            if scenario_success(scenario, state):
                self.log(f"scenario {scenario.id}: success ({state.phase.value})")
                return True

            self.dismiss_popups()
            state = self.read()

            for step in scenario.steps:
                self.executor.run_actions([step], state)
                time.sleep(self.step_delay)

            state = self.read()
            if scenario_success(scenario, state):
                self.log(f"scenario {scenario.id}: success after steps")
                return True

            time.sleep(0.4)

        self.log(f"scenario {scenario.id}: timeout")
        return False

    def list_scenarios(self) -> list[str]:
        return sorted(SCENARIOS.keys())

    def describe(self) -> dict:
        state = self.read()
        return {
            "state": state.to_dict(),
            "scenarios": {k: v.description for k, v in SCENARIOS.items()},
            "popups": [s.id for s in POPUP_SIGNATURES],
            "known_fingerprints": self._phase_fp,
        }
