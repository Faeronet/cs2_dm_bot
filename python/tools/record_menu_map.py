#!/usr/bin/env python3
"""Пройти меню и записать доли игрового поля.

GO и «получить» только наводятся, без клика. Доли считаются от поля,
которое live_ui находит внутри рамки окна.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from PIL import Image

from cs2_control.input import create_input
from cs2_control.menu_input import MenuInput
from cs2_menu.live_ui import (
    MODE_ORDER,
    Frame,
    detect_client,
    find_band,
    find_cards,
    find_green,
    find_icons,
    find_nav,
)
from cs2_menu.xwl_pointer import move_abs

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "data", "menu_map.json")
SHOTS = "/tmp/cs2menu/rec"


def grab(menu: MenuInput) -> Frame | None:
    if not menu.refresh_window():
        return None
    geo = menu.geometry()
    if not geo.get("width"):
        return None
    ox, oy = int(geo.get("x", 0)), int(geo.get("y", 0))
    w, h = int(geo["width"]), int(geo["height"])
    env = menu._xdo_env()
    disp = env.get("DISPLAY", ":1")
    path = "/tmp/cs2menu/frame.png"
    try:
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "x11grab",
             "-video_size", "1920x1200", "-i", f"{disp}+0,0",
             "-frames:v", "1", "-update", "1", path],
            env=env, capture_output=True, timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    full = Image.open(path).convert("RGB")
    im = full.crop((ox, oy, min(1920, ox + w), min(1200, oy + h)))
    return Frame(im, (ox, oy), detect_client(im))


def main() -> int:
    os.makedirs(SHOTS, exist_ok=True)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    menu = MenuInput()
    inp = create_input()
    book: dict = {"client_ref": None, "screens": {}}

    def shot(name: str, fr: Frame) -> None:
        fr.image.save(os.path.join(SHOTS, f"{name}.png"))

    def go(fr: Frame, fx: float, fy: float, *, click: bool) -> None:
        sx, sy = fr.screen(fx, fy)
        move_abs(inp, sx, sy)
        if click:
            inp.mouse_click("left", hold_ms=60)
            time.sleep(0.85)
        else:
            time.sleep(0.35)

    def note(screen: str, fr: Frame, extra: dict | None = None) -> None:
        nav = find_nav(fr.image, fr.client)
        icons = find_icons(fr.image, fr.client)
        icons.pop("power", None)
        sub = find_band(fr.image, fr.client, ("matchmaking", "practice", "workshop"), 0.055, 0.105)
        modes = find_band(fr.image, fr.client, MODE_ORDER, 0.095, 0.145)
        green = find_green(fr.image, fr.client)
        entry = {
            "client": [fr.client.left, fr.client.top, fr.client.width, fr.client.height],
            "nav": nav,
            "icons": icons,
            "sub": sub,
            "modes": modes,
            "go": green,
            "cards_top": find_cards(fr.image, fr.client, 0.40),
            "cards_bot": find_cards(fr.image, fr.client, 0.68),
        }
        if extra:
            entry.update(extra)
        book["screens"][screen] = entry
        book["client_ref"] = [fr.client.width, fr.client.height]
        if nav:
            book["nav"] = nav
        if icons:
            book["icons"] = icons

    try:
        fr = grab(menu)
        if fr is None:
            print("нет окна")
            return 1
        note("start", fr)
        shot("00_start", fr)
        nav = find_nav(fr.image, fr.client)
        if "play" not in nav:
            print("вкладки не видны", fr.client)
            return 1
        go(fr, *nav["play"], click=True)
        fr = grab(menu)
        if fr is None:
            return 1
        shot("01_play", fr)
        note("play", fr)
        sub = find_band(fr.image, fr.client, ("matchmaking", "practice", "workshop"), 0.055, 0.105)
        print("sub", sub)
        # официальный DM и остальные режимы матчмейкинга
        if "matchmaking" in sub:
            go(fr, *sub["matchmaking"], click=True)
        fr = grab(menu)
        if fr is None:
            return 1
        modes = find_band(fr.image, fr.client, MODE_ORDER, 0.095, 0.145)
        print("modes", {k: (round(a, 3), round(b, 3)) for k, (a, b) in modes.items()})
        # если ряд не собрался целиком — известные доли этого поля, проверенные по синему DEATHMATCH
        if "deathmatch" not in modes:
            modes = {
                "premier": (0.325, 0.118),
                "competitive": (0.382, 0.118),
                "wingman": (0.423, 0.118),
                "rush": (0.450, 0.118),
                "casual": (0.478, 0.118),
                "deathmatch": (0.516, 0.118),
                "arms_race": (0.576, 0.118),
                "retakes": (0.627, 0.118),
                "private": (0.702, 0.118),
            }
        for name, pt in modes.items():
            go(fr, *pt, click=True)
            fr = grab(menu)
            if fr is None:
                return 1
            shot(f"mode_{name}", fr)
            green = find_green(fr.image, fr.client)
            cards = find_cards(fr.image, fr.client, 0.48)
            note(f"mode_{name}", fr, {"cards": cards})
            if green:
                go(fr, *green, click=False)
                fr = grab(menu)
                if fr:
                    shot(f"go_{name}", fr)
            print(name, "go", None if not green else tuple(round(v, 3) for v in green), "cards", len(cards))
        with open(OUT, "w", encoding="utf-8") as f:
            json.dump(book, f, ensure_ascii=False, indent=2)
        print("wrote", OUT)
    finally:
        try:
            inp.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
