"""Сигнатуры всплывающих окон и уведомлений CS2."""

from __future__ import annotations

import json
from pathlib import Path

from cs2_menu.models import PopupSignature, UIAction, UIState

ROOT = Path(__file__).resolve().parents[2]
FP_CACHE = ROOT / "data" / "menu_fingerprints.json"


def _is_main_menu_fp(fp: str) -> bool:
    return fp in load_fingerprint_map().get("main_menu", set())


def _vote_popup(state: UIState) -> float:
    if state.in_map():
        return 0.0
    votes = state.entity_counts.get("vote_controller", 0)
    if votes >= 3:
        return 0.85
    if votes >= 2 and not state.in_game:
        return 0.55
    return 0.0


def _notification_banner(state: UIState) -> float:
    if state.in_map():
        return 0.0
    if _is_main_menu_fp(state.ui_fingerprint):
        return 0.0
    # navbar notification: мало entity, fingerprint часто стабилен, но popup меняет hash
    if state.entity_counts.get("cs_player_controller", 0) == 1 and not state.in_game:
        known = load_fingerprint_map().get("notification_open", set())
        if state.ui_fingerprint in known:
            return 0.9
    return 0.0


def _generic_popup(state: UIState) -> float:
    if state.in_map():
        return 0.0
    if _is_main_menu_fp(state.ui_fingerprint):
        return 0.0
    known = load_fingerprint_map().get("generic_popup", set())
    if state.ui_fingerprint in known:
        return 0.75
    return 0.0


def _accept_match(state: UIState) -> float:
    if state.phase.value == "accept_match":
        return 1.0
    known = load_fingerprint_map().get("accept_match", set())
    if state.ui_fingerprint in known:
        return 0.95
    if state.matchmaking_map and state.matchmaking_map not in ("", "<empty>"):
        return 0.4
    return 0.0


def _prime_upsell(state: UIState) -> float:
    known = load_fingerprint_map().get("prime_upsell", set())
    if state.ui_fingerprint in known:
        return 0.85
    return 0.0


DISMISS_ESCAPE = [
    UIAction.key("Escape"),
    UIAction.wait(250),
]

DISMISS_GENERIC = [
    UIAction.click("popup_ok_center"),
    UIAction.wait(200),
    UIAction.key("Return"),
    UIAction.wait(200),
    UIAction.click("popup_close_x"),
    UIAction.key("Escape", repeat=2),
]

POPUP_SIGNATURES: list[PopupSignature] = [
    PopupSignature(
        id="accept_match",
        description="Принять матч",
        match=_accept_match,
        dismiss=[
            UIAction.click("accept_match"),
            UIAction.wait(300),
            UIAction.key("Return"),
        ],
        priority=100,
    ),
    PopupSignature(
        id="vote_poll",
        description="Голосование / опрос",
        match=_vote_popup,
        dismiss=DISMISS_ESCAPE + [UIAction.click("popup_close_x")],
        priority=50,
    ),
    PopupSignature(
        id="notification",
        description="Уведомление в navbar / toast",
        match=_notification_banner,
        dismiss=[
            UIAction.click("notification_close"),
            UIAction.wait(150),
            UIAction.click("notification_body"),
            UIAction.key("Escape"),
        ],
        priority=80,
    ),
    PopupSignature(
        id="prime_upsell",
        description="Prime / магазин upsell",
        match=_prime_upsell,
        dismiss=DISMISS_GENERIC,
        priority=70,
    ),
    PopupSignature(
        id="generic_popup",
        description="Любой popup_generic / popup_custom",
        match=_generic_popup,
        dismiss=DISMISS_GENERIC,
        priority=10,
    ),
    PopupSignature(
        id="fallback_dismiss",
        description="Универсальный сброс Esc + OK",
        match=lambda s: 0.2 if (not s.in_map() and not _is_main_menu_fp(s.ui_fingerprint)) else 0.0,
        dismiss=DISMISS_GENERIC,
        priority=1,
    ),
]


def load_fingerprint_map() -> dict[str, set[str]]:
    if not FP_CACHE.is_file():
        return {}
    try:
        raw = json.loads(FP_CACHE.read_text(encoding="utf-8"))
        return {
            k: set(v) if isinstance(v, list) else {v}
            for k, v in raw.items()
            if k != "screens" and isinstance(v, (list, str))
        }
    except (json.JSONDecodeError, OSError):
        return {}


def save_fingerprint(kind: str, fp: str) -> None:
    raw: dict = {}
    if FP_CACHE.is_file():
        try:
            raw = json.loads(FP_CACHE.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            raw = {}
    screens = raw.get("screens")
    data = {k: sorted(v) for k, v in load_fingerprint_map().items()}
    bucket = set(data.get(kind, []))
    bucket.add(fp)
    data[kind] = sorted(bucket)
    if isinstance(screens, dict):
        data["screens"] = screens
    FP_CACHE.parent.mkdir(parents=True, exist_ok=True)
    FP_CACHE.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def detect_popups(state: UIState, *, min_conf: float = 0.5) -> list[tuple[PopupSignature, float]]:
    found: list[tuple[PopupSignature, float]] = []
    for sig in POPUP_SIGNATURES:
        conf = float(sig.match(state))
        if conf >= min_conf:
            found.append((sig, conf))
    found.sort(key=lambda x: (-x[0].priority, -x[1]))
    return found
