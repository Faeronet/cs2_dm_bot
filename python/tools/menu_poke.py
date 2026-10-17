#!/usr/bin/env python3
"""Инструмент калибровки меню на XWayland.

Снимок через root-xwd (пооконный xwd даёт мусор), клик по абсолютным
экранным координатам (xdotool mousemove + uinput button), обратная связь —
из памяти (ui_fingerprint, matchmaking_map, phase, pawn).

  shot [out.png]            — только скриншот root c сеткой
  click X Y [out.png]       — клик по экранным (X,Y) + фидбек + скриншот
  state                     — показать память (fp/mm_map/phase)
  key NAME [repeat]         — нажать клавишу (escape/grave/return/...)
  type TEXT                 — напечатать текст
"""

from __future__ import annotations

import subprocess
import sys
import time

sys.path.insert(0, __import__("os").path.join(__import__("os").path.dirname(__file__), ".."))

from PIL import Image, ImageDraw

from cs2_menu import MenuController
from cs2_menu.driver import UIStateReader
from cs2_menu.vision import _parse_xwd
from cs2_reader.reader import Reader

XDOTOOL = "/home/faeronet/Документы/cs2-memory-damp/tools/xdotool/usr/bin/xdotool"


SCREEN_W, SCREEN_H = 1920, 1200


def root_shot(env, out: str, grid: bool = True) -> None:
    # ffmpeg x11grab надёжнее xwd на XWayland (xwd даёт BadColor / мусор)
    disp = env.get("DISPLAY", ":1")
    subprocess.run(
        ["ffmpeg", "-y", "-f", "x11grab", "-video_size", f"{SCREEN_W}x{SCREEN_H}",
         "-i", f"{disp}+0,0", "-frames:v", "1", "-update", "1", "/tmp/ffshot.png"],
        env=env, capture_output=True, timeout=10,
    )
    try:
        rgb = Image.open("/tmp/ffshot.png").convert("RGB")
    except Exception as exc:  # noqa: BLE001
        print(f"ffmpeg shot failed: {exc}")
        return
    w, h = rgb.size
    if grid:
        d = ImageDraw.Draw(rgb)
        for x in range(0, w, 100):
            d.line([(x, 0), (x, h)], fill=(255, 0, 0), width=1)
            d.text((x + 1, 1), str(x), fill=(255, 255, 0))
        for y in range(0, h, 100):
            d.line([(0, y), (w, y)], fill=(0, 180, 255), width=1)
            d.text((1, y + 1), str(y), fill=(0, 255, 0))
    rgb.save(out)
    print(f"saved {out} ({w}x{h})")


def move_abs(inp, sx: int, sy: int) -> None:
    """Абсолютное позиционирование курсора через uinput (Wayland-safe).

    xdotool на XWayland двигает X-указатель, который НЕ совпадает с указателем
    композитора. Поэтому прижимаем курсор в угол большими дельтами, затем шагаем.
    """
    if getattr(inp, "_running", False):
        inp._running = False
        time.sleep(0.05)
    for _ in range(60):
        inp._emit_rel(-200, -200)
        time.sleep(0.0004)
    time.sleep(0.12)
    for _ in range(int(sx)):
        inp._emit_rel(1, 0)
        time.sleep(0.0003)
    for _ in range(int(sy)):
        inp._emit_rel(0, 1)
        time.sleep(0.0003)
    time.sleep(0.1)


def feedback(menu: MenuController, driver: UIStateReader) -> None:
    st = driver.read()
    print(
        f"  phase={st.phase.value} fp={st.ui_fingerprint} "
        f"mm_map={st.matchmaking_map!r} in_game={st.in_game} "
        f"pawn={menu.reader.local_pawn()} map={menu.reader.map_name()!r}"
    )


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    cmd = sys.argv[1]
    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1
    menu = MenuController(reader)
    driver = UIStateReader(reader)
    env = menu.executor._xdo_env()

    if cmd == "shot":
        out = sys.argv[2] if len(sys.argv) > 2 else "/tmp/menu.png"
        root_shot(env, out)
        feedback(menu, driver)
    elif cmd == "state":
        feedback(menu, driver)
    elif cmd == "click":
        x, y = int(sys.argv[2]), int(sys.argv[3])
        out = sys.argv[4] if len(sys.argv) > 4 else "/tmp/menu.png"
        print(f"before: ", end="")
        feedback(menu, driver)
        menu.executor.menu_input.focus()
        move_abs(menu.executor.menu_input.game_input, x, y)
        menu.executor.menu_input.game_input.mouse_click("left", hold_ms=70)
        time.sleep(1.2)
        print(f"after click ({x},{y}): ", end="")
        feedback(menu, driver)
        root_shot(env, out)
    elif cmd == "clearpopups":
        from cs2_menu.modal_vision import clear_modals
        from cs2_menu.xwl_pointer import ScreenClicker

        clicker = ScreenClicker(menu.executor.menu_input, env=env)
        ok = clear_modals(clicker, env)
        print(f"clear_modals -> {ok}")
        root_shot(env, sys.argv[2] if len(sys.argv) > 2 else "/tmp/after_clear.png")
    elif cmd in ("pen", "penhover"):
        from cs2_control.input import AbsPointer

        x, y = int(sys.argv[2]), int(sys.argv[3])
        out = sys.argv[4] if len(sys.argv) > 4 else "/tmp/menu.png"
        menu.executor.menu_input.focus()
        time.sleep(0.2)
        pen = AbsPointer(SCREEN_W, SCREEN_H)
        if cmd == "pen":
            print(f"before: ", end="")
            feedback(menu, driver)
            pen.click(x, y, hold_ms=80)
            time.sleep(1.2)
            print(f"after pen click ({x},{y}): ", end="")
            feedback(menu, driver)
        else:
            pen.move(x, y)
            time.sleep(0.6)
        pen.close()
        root_shot(env, out)
    elif cmd == "hover":
        x, y = int(sys.argv[2]), int(sys.argv[3])
        out = sys.argv[4] if len(sys.argv) > 4 else "/tmp/menu.png"
        menu.executor.menu_input.focus()
        move_abs(menu.executor.menu_input.game_input, x, y)
        time.sleep(0.6)
        root_shot(env, out)
    elif cmd == "key":
        name = sys.argv[2]
        rep = int(sys.argv[3]) if len(sys.argv) > 3 else 1
        menu.executor.menu_input.focus()
        menu.executor.key(name, repeat=rep)
        time.sleep(0.8)
        feedback(menu, driver)
    elif cmd == "type":
        menu.executor.menu_input.focus()
        menu.executor.menu_input.type_text(sys.argv[2])
        time.sleep(0.3)
        feedback(menu, driver)
    else:
        print("unknown cmd")
        return 1
    menu.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
