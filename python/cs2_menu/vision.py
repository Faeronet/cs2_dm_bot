"""Снимок окна и определение текущего экрана меню (устойчиво к анимации фона)."""

from __future__ import annotations

import hashlib
import json
import struct
import subprocess
from dataclasses import dataclass
from pathlib import Path

from cs2_menu.geometry import WindowLayout
from cs2_menu.layout import (
    GO_BUTTON,
    LAYOUT,
    MODE_CASUAL,
    MODE_DEATHMATCH,
    MODE_PRACTICE,
    MODE_PREMIER,
    MODE_WINGMAN,
    NAV_INVENTORY,
    NAV_NEWS,
    NAV_PLAY,
    NAV_STORE,
)
from cs2_menu.models import NormPoint

ROOT = Path(__file__).resolve().parents[2]
FP_CACHE = ROOT / "data" / "menu_fingerprints.json"

PATCH_SIZE = 20
NAV_ORANGE_MARGIN = 8.0  # min разница для уверенного выбора nav

# Ожидаемый экран после клика
TARGET_SCREEN: dict[str, str] = {
    "nav_play": "play_hub",
    "nav_store": "store",
    "nav_inventory": "inventory",
    "nav_news": "news",
    "mode_deathmatch": "mode_deathmatch",
    "mode_casual": "mode_casual",
    "mode_premier": "mode_premier",
    "mode_wingman": "mode_wingman",
    "mode_practice": "mode_practice",
    "mode_training": "mode_practice",
    "practice_map_mirage": "practice_maps",
    "practice_mode_competitive": "practice_settings",
    "practice_start": "practice_start",
    "go_button": "matchmaking",
    "back_button": "play_hub",
    "cancel_search": "mode_deathmatch",
}

NAV_POINTS: list[tuple[str, NormPoint]] = [
    ("play", NAV_PLAY),
    ("store", NAV_STORE),
    ("inventory", NAV_INVENTORY),
    ("news", NAV_NEWS),
]

MODE_POINTS: list[tuple[str, NormPoint]] = [
    ("mode_deathmatch", MODE_DEATHMATCH),
    ("mode_casual", MODE_CASUAL),
    ("mode_premier", MODE_PREMIER),
    ("mode_wingman", MODE_WINGMAN),
    ("mode_practice", MODE_PRACTICE),
]


@dataclass
class ScreenSignature:
    screen_id: str
    confidence: float
    region_hashes: dict[str, str]
    debug: dict[str, float | str] | None = None


@dataclass
class PatchStats:
    r: float
    g: float
    b: float

    @property
    def luma(self) -> float:
        return 0.299 * self.r + 0.587 * self.g + 0.114 * self.b

    @property
    def orange(self) -> float:
        return self.r - 0.55 * self.g


def load_screen_catalog() -> dict[str, dict[str, str]]:
    if not FP_CACHE.is_file():
        return {}
    try:
        raw = json.loads(FP_CACHE.read_text(encoding="utf-8"))
        screens = raw.get("screens", {})
        if isinstance(screens, dict):
            return {k: dict(v) for k, v in screens.items() if isinstance(v, dict)}
    except (json.JSONDecodeError, OSError):
        pass
    return {}


def save_screen_catalog(catalog: dict[str, dict[str, str]]) -> None:
    data: dict = {}
    if FP_CACHE.is_file():
        try:
            data = json.loads(FP_CACHE.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            data = {}
    data["screens"] = {k: dict(v) for k, v in sorted(catalog.items())}
    FP_CACHE.parent.mkdir(parents=True, exist_ok=True)
    FP_CACHE.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def learn_screen(screen_id: str, region_hashes: dict[str, str]) -> None:
    cat = load_screen_catalog()
    cat[screen_id] = dict(region_hashes)
    save_screen_catalog(cat)


def _parse_xwd(path: str) -> tuple[int, int, bytes] | None:
    try:
        data = open(path, "rb").read()
    except OSError:
        return None
    if len(data) < 32:
        return None
    header_size = struct.unpack(">I", data[0:4])[0]
    hdr_bytes = header_size * 4 if header_size < 4096 else header_size
    if hdr_bytes < 32:
        hdr_bytes = 468
    w = struct.unpack(">I", data[16:20])[0]
    h = struct.unpack(">I", data[20:24])[0]
    if w <= 0 or h <= 0 or w > 8192 or h > 8192:
        return None
    pixels = data[hdr_bytes:]
    need = w * h * 4
    if len(pixels) < need:
        return None
    return w, h, pixels[:need]


def capture_window_xwd(window_id: int, path: str, env: dict[str, str]) -> bool:
    try:
        subprocess.run(
            ["xwd", "-id", str(window_id), "-out", path],
            env=env,
            capture_output=True,
            timeout=4.0,
        )
        return Path(path).is_file()
    except (subprocess.TimeoutExpired, OSError):
        return False


def sample_patch_stats(
    pixels: bytes,
    img_w: int,
    img_h: int,
    layout: WindowLayout,
    pt: NormPoint,
    *,
    size: int = PATCH_SIZE,
) -> PatchStats:
    cx, cy = layout.norm_to_client(pt)
    ix = layout.letterbox_x + cx
    iy = layout.letterbox_y + cy
    half = size // 2
    rs: list[float] = []
    gs: list[float] = []
    bs: list[float] = []
    for y in range(max(0, iy - half), min(img_h, iy + half)):
        for x in range(max(0, ix - half), min(img_w, ix + half)):
            o = (y * img_w + x) * 4
            if o + 3 >= len(pixels):
                continue
            rs.append(pixels[o + 2])
            gs.append(pixels[o + 1])
            bs.append(pixels[o])
    if not rs:
        return PatchStats(0.0, 0.0, 0.0)
    n = len(rs)
    return PatchStats(sum(rs) / n, sum(gs) / n, sum(bs) / n)


def _patch_fingerprint(stats: PatchStats) -> str:
    """Компактный отпечаток патча — устойчивее полного MD5 региона."""
    return f"{int(stats.r)//16:02x}{int(stats.g)//16:02x}{int(stats.b)//16:02x}{int(stats.orange)//8:02x}"


def build_region_hashes(
    pixels: bytes,
    img_w: int,
    img_h: int,
    layout: WindowLayout,
) -> dict[str, str]:
    """Отпечатки ключевых точек UI (не больших анимированных регионов)."""
    out: dict[str, str] = {}
    for name, pt in NAV_POINTS:
        out[f"nav_{name}"] = _patch_fingerprint(sample_patch_stats(pixels, img_w, img_h, layout, pt))
    for name, pt in MODE_POINTS:
        out[name] = _patch_fingerprint(sample_patch_stats(pixels, img_w, img_h, layout, pt))
    out["go"] = _patch_fingerprint(sample_patch_stats(pixels, img_w, img_h, layout, GO_BUTTON))
    return out


def region_hashes_from_xwd(path: str, layout: WindowLayout) -> dict[str, str]:
    parsed = _parse_xwd(path)
    if not parsed:
        return {}
    img_w, img_h, pixels = parsed
    return build_region_hashes(pixels, img_w, img_h, layout)


def detect_screen_from_pixels(
    pixels: bytes,
    img_w: int,
    img_h: int,
    layout: WindowLayout,
) -> ScreenSignature:
    region_hashes = build_region_hashes(pixels, img_w, img_h, layout)

    nav_scores: dict[str, float] = {}
    for name, pt in NAV_POINTS:
        nav_scores[name] = sample_patch_stats(pixels, img_w, img_h, layout, pt).orange

    best_nav = max(nav_scores, key=nav_scores.get)
    sorted_scores = sorted(nav_scores.values(), reverse=True)
    nav_margin = sorted_scores[0] - sorted_scores[1] if len(sorted_scores) > 1 else sorted_scores[0]
    nav_confident = nav_margin >= NAV_ORANGE_MARGIN

    go_luma = sample_patch_stats(pixels, img_w, img_h, layout, GO_BUTTON).luma
    mode_lumas: dict[str, float] = {}
    for name, pt in MODE_POINTS:
        mode_lumas[name] = sample_patch_stats(pixels, img_w, img_h, layout, pt).luma
    bright_modes = sum(1 for v in mode_lumas.values() if v > 55)
    dm_luma = mode_lumas.get("mode_deathmatch", 0.0)

    debug: dict[str, float | str] = {
        **{f"nav_{k}_orange": round(v, 1) for k, v in nav_scores.items()},
        "go_luma": round(go_luma, 1),
        "dm_card_luma": round(dm_luma, 1),
        "bright_mode_cards": bright_modes,
        "nav_margin": round(nav_margin, 1),
    }

    # Matchmaking: яркая GO / Cancel внизу
    if go_luma > 95 and nav_scores.get("play", 0) >= sorted_scores[1]:
        return ScreenSignature("matchmaking", 0.85, region_hashes, debug)

    # Sidebar-экраны
    if nav_confident:
        if best_nav == "store":
            return ScreenSignature("store", 0.8, region_hashes, debug)
        if best_nav == "inventory":
            return ScreenSignature("inventory", 0.8, region_hashes, debug)
        if best_nav == "news":
            return ScreenSignature("news", 0.75, region_hashes, debug)

    # Play selected: play hub vs mode detail vs main home
    if nav_scores.get("play", 0) >= sorted_scores[0] - 3:
        if go_luma > 70:
            best_mode = max(mode_lumas, key=mode_lumas.get)
            if mode_lumas[best_mode] > 40:
                return ScreenSignature(best_mode, 0.7, region_hashes, debug)
        # play hub = все карточки режимов видны (на main home casual/practice тёмные)
        casual_l = mode_lumas.get("mode_casual", 0.0)
        practice_l = mode_lumas.get("mode_practice", 0.0)
        debug["casual_luma"] = round(casual_l, 1)
        debug["practice_luma"] = round(practice_l, 1)
        if bright_modes >= 4 and casual_l > 45 and practice_l > 45:
            return ScreenSignature("play_hub", 0.85, region_hashes, debug)
        return ScreenSignature("main_home", 0.78, region_hashes, debug)

    # Fallback: сравнение с каталогом patch-fingerprints
    catalog = load_screen_catalog()
    best_id = "unknown"
    best_score = 0.0
    for screen_id, ref in catalog.items():
        keys = set(ref) & set(region_hashes)
        if not keys:
            continue
        matches = sum(1 for k in keys if ref[k] == region_hashes[k])
        score = matches / max(len(ref), 1)
        if score > best_score:
            best_score = score
            best_id = screen_id

    if best_score >= 0.55:
        return ScreenSignature(best_id, best_score, region_hashes, debug)
    return ScreenSignature("unknown", best_score, region_hashes, debug)


def identify_screen(region_hashes: dict[str, str], *, min_match: float = 0.55) -> ScreenSignature:
    catalog = load_screen_catalog()
    if not catalog or not region_hashes:
        return ScreenSignature("unknown", 0.0, region_hashes)
    best_id = "unknown"
    best_score = 0.0
    for screen_id, ref in catalog.items():
        keys = set(ref) & set(region_hashes)
        if not keys:
            continue
        matches = sum(1 for k in keys if ref[k] == region_hashes[k])
        score = matches / max(len(ref), 1)
        if score > best_score:
            best_score = score
            best_id = screen_id
    if best_score < min_match:
        return ScreenSignature("unknown", best_score, region_hashes)
    return ScreenSignature(best_id, best_score, region_hashes)


def identify_from_xwd(path: str, layout: WindowLayout) -> ScreenSignature:
    parsed = _parse_xwd(path)
    if not parsed:
        return ScreenSignature("unknown", 0.0, {})
    img_w, img_h, pixels = parsed
    return detect_screen_from_pixels(pixels, img_w, img_h, layout)


def expected_screen_for_target(target: str) -> str | None:
    return TARGET_SCREEN.get(target)


def screens_differ(a: dict[str, str], b: dict[str, str]) -> bool:
    if not a or not b:
        return False
    keys = set(a) & set(b)
    if not keys:
        return a != b
    matches = sum(1 for k in keys if a[k] == b[k])
    return matches / len(keys) < 0.85
