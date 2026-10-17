#!/usr/bin/env python3
"""Проба чтения Panorama-меню из памяти CS2 и клик по «Играть».

Что реально читается сейчас:
  - размер окна (libengine2.so)
  - MENU_CONTEXT / HUD_CONTEXT (libpanorama.so) — указатель на UI engine
  - признак «в меню» (нет карты, нет матча)

Что пока НЕ читается без reverse CPanel2D:
  - id/текст кнопок в рантайме (mainmenu_play только в VPK, в heap не лежит)
  - точные x/y/w/h панелей Panorama

Клик «Играть» — fallback: Enter или эвристические координаты (масштаб от размера окна).
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import re
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_control.input import create_input
from cs2_reader.reader import Reader

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
XDOTOOL = os.path.join(ROOT, "tools", "xdotool", "usr", "bin", "xdotool")
XDO_LIB = os.path.join(ROOT, "tools", "xdotool", "usr", "lib", "x86_64-linux-gnu")
MAX_PTR = 0x00007FFFFFFFFFFF

# Эвристика: левая колонка «Play» + большая кнопка (доля client area, подогнано под 16:10/16:9)
PLAY_NAV_NORM = (0.06, 0.38)
PLAY_ACTION_NORM = (0.15, 0.52)


def xdo_env() -> dict[str, str]:
    env = {**os.environ, "DISPLAY": os.environ.get("DISPLAY", ":0")}
    if os.path.isdir(XDO_LIB):
        prev = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = f"{XDO_LIB}:{prev}" if prev else XDO_LIB
    uid = os.getuid()
    for pat in (f"/run/user/{uid}/.mutter-Xwaylandauth.*", f"/run/user/{uid}/.Xauthority"):
        hits = glob.glob(pat)
        if hits:
            env["XAUTHORITY"] = hits[0]
            break
    return env


def run_xdo(*args: str) -> str:
    if not os.path.isfile(XDOTOOL):
        return ""
    try:
        out = subprocess.run(
            [XDOTOOL, *args],
            env=xdo_env(),
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        return (out.stdout or "").strip()
    except (subprocess.TimeoutExpired, OSError):
        return ""


def find_cs2_window() -> int | None:
    for query in (("search", "--name", "Counter-Strike 2"), ("search", "--class", "cs2")):
        raw = run_xdo(*query)
        if raw:
            try:
                return int(raw.splitlines()[0].strip())
            except ValueError:
                continue
    return None


def window_geometry(wid: int) -> dict[str, int]:
    geo: dict[str, int] = {}
    shell = run_xdo("getwindowgeometry", "--shell", str(wid))
    for line in shell.splitlines():
        if "=" not in line:
            continue
        k, v = line.split("=", 1)
        try:
            geo[k.lower()] = int(v)
        except ValueError:
            pass
    return geo


def load_panorama_offsets() -> dict[str, int]:
    path = os.path.join(ROOT, "data", "offsets.json")
    raw = json.load(open(path, encoding="utf-8"))
    return {item["name"]: item["value"] for item in raw.get("libpanorama.so", [])}


def usable_ptr(ptr: int | None) -> int | None:
    if not ptr or ptr < 0x10000 or ptr > MAX_PTR:
        return None
    return ptr


def read_menu_state(reader: Reader) -> dict:
    b = reader.backend
    pan = b.module_base("libpanorama.so")
    eng = b.module_base("libengine2.so")
    off = load_panorama_offsets()

    menu_ctx = hud_ctx = 0
    if pan:
        menu_ctx = b.read_u64(pan + off.get("MENU_CONTEXT", 0)) or 0
        hud_ctx = b.read_u64(pan + off.get("HUD_CONTEXT", 0)) or 0

    win_w = win_h = 0
    if eng:
        # offsets из data/offsets.json (актуальны для текущего дампа)
        win_w = b.read_u32(eng + 10490208) or 0
        win_h = b.read_u32(eng + 10490212) or 0

    snap = reader.capture()
    in_menu = not snap.in_game and not (snap.map_name or "").strip()

    return {
        "in_menu": in_menu,
        "map": snap.map_name,
        "in_game": snap.in_game,
        "menu_context": menu_ctx,
        "hud_context": hud_ctx,
        "same_context": menu_ctx == hud_ctx and menu_ctx != 0,
        "window_from_memory": (win_w, win_h),
        "players": len(snap.players),
    }


def probe_panorama_panels(reader: Reader) -> list[dict]:
    """Поиск строк/прямоугольников панелей — на практике id панелей interned, строк нет."""
    b = reader.backend
    pan = b.module_base("libpanorama.so")
    if not pan:
        return []
    off = load_panorama_offsets()
    ctx = usable_ptr(b.read_u64(pan + off.get("MENU_CONTEXT", 0)))
    if not ctx:
        return []

    hits: list[dict] = []
    data = b.read(ctx, 0x400)
    if not data:
        return hits

    for m in re.finditer(rb"[\x20-\x7e]{4,48}", data):
        s = m.group().decode("ascii", errors="ignore")
        if any(k in s.lower() for k in ("menu", "play", "button", "main")):
            hits.append({"kind": "string", "offset": m.start(), "text": s})

    for base_off in range(0x80, 0x280, 0x40):
        obj = usable_ptr(b.read_u64(ctx + base_off))
        if not obj:
            continue
        blob = b.read(obj, 0x200)
        if not blob:
            continue
        for off in range(0, len(blob) - 16, 4):
            x, y, w, h = struct.unpack("<4f", blob[off : off + 16])
            if 0 <= x <= 2000 and 0 <= y <= 1200 and 30 <= w <= 800 and 15 <= h <= 200:
                hits.append(
                    {
                        "kind": "rect",
                        "ctx_off": base_off,
                        "obj": obj,
                        "panel_off": off,
                        "x": round(x, 1),
                        "y": round(y, 1),
                        "w": round(w, 1),
                        "h": round(h, 1),
                    }
                )
                if len(hits) > 40:
                    return hits
    return hits


def scaled_click_points(mem_w: int, mem_h: int, geo: dict[str, int]) -> list[tuple[str, int, int]]:
    """Экранные координаты: масштаб эвристики от mem size к реальному окну."""
    wx = geo.get("x", 0)
    wy = geo.get("y", 0)
    ww = geo.get("width", mem_w or 1280)
    wh = geo.get("height", mem_h or 800)
    mw = mem_w or ww
    mh = mem_h or wh

    pts: list[tuple[str, int, int]] = []
    for label, (nx, ny) in (("play_nav", PLAY_NAV_NORM), ("play_action", PLAY_ACTION_NORM)):
        cx = int(wx + nx * ww)
        cy = int(wy + ny * wh)
        pts.append((label, cx, cy))
    return pts


def click_screen(x: int, y: int) -> bool:
    if not os.path.isfile(XDOTOOL):
        print("xdotool не найден", file=sys.stderr)
        return False
    wid = find_cs2_window()
    if wid:
        run_xdo("windowactivate", str(wid))
        time.sleep(0.15)
    run_xdo("mousemove", str(x), str(y))
    time.sleep(0.05)
    run_xdo("click", "1")
    return True


def press_enter() -> None:
    if os.path.isfile(XDOTOOL):
        wid = find_cs2_window()
        if wid:
            run_xdo("windowactivate", str(wid))
            time.sleep(0.1)
        run_xdo("key", "Return")
        return
    inp = create_input()
    inp.prepare()
    try:
        inp.key_tap("space", hold_ms=50)
    finally:
        inp.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Проба Panorama-меню CS2")
    parser.add_argument("--click", action="store_true", help="Реально нажать (Enter + клик)")
    parser.add_argument("--enter-only", action="store_true", help="Только Enter, без мыши")
    parser.add_argument("--dry-run", action="store_true", help="Только отчёт (по умолчанию если нет --click)")
    args = parser.parse_args()
    dry = args.dry_run or not args.click

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    state = read_menu_state(reader)
    print("=== Menu memory probe ===")
    for k, v in state.items():
        print(f"  {k}: {v}")

    panels = probe_panorama_panels(reader)
    print(f"\nPanorama panel hints: {len(panels)}")
    for p in panels[:12]:
        print(f"  {p}")
    if not panels:
        print("  (строк mainmenu_play / rect кнопок в MENU_CONTEXT не найдены — нужен RE CPanel2D)")

    wid = find_cs2_window()
    geo = window_geometry(wid) if wid else {}
    mem_w, mem_h = state["window_from_memory"]
    print(f"\nWindow X11: id={wid} geo={geo}")
    print(f"Window memory: {mem_w}x{mem_h}")

    clicks = scaled_click_points(mem_w, mem_h, geo)
    print("\nFallback click targets (эвристика, не из памяти):")
    for label, x, y in clicks:
        print(f"  {label}: screen ({x}, {y})")

    if dry:
        print("\nDry-run. Для клика: --click  (или --enter-only --click)")
        return 0

    if not state["in_menu"]:
        print("\nПохоже, уже не главное меню — отмена клика.", file=sys.stderr)
        return 2

    print("\nПробую Enter…")
    press_enter()
    time.sleep(0.8)

    if args.enter_only:
        print("Готово (Enter).")
        return 0

    for label, x, y in clicks:
        print(f"Клик {label} ({x}, {y})…")
        click_screen(x, y)
        time.sleep(0.6)

    after = read_menu_state(reader)
    print(f"\nПосле клика: in_menu={after['in_menu']} map={after['map']!r} players={after['players']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
