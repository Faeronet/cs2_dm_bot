#!/usr/bin/env python3
"""Снимок окна CS2 → PNG. Для визуальной калибровки координат меню.

  python3 python/tools/menu_screenshot.py [out.png] [--grid]

--grid рисует сетку нормализованных координат (0..1) по client area,
чтобы можно было считать x/y кнопок прямо с картинки.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from PIL import Image, ImageDraw

from cs2_menu import MenuController
from cs2_menu.vision import _parse_xwd, capture_window_xwd
from cs2_reader.reader import Reader


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = args[0] if args else "/tmp/cs2_menu.png"
    draw_grid = "--grid" in sys.argv

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1
    menu = MenuController(reader)
    menu.executor.refresh_window()
    st = menu.where()
    layout = menu.executor.layout(st)
    wid = menu.executor._window_id
    if not wid or not layout:
        print("нет окна/геометрии", file=sys.stderr)
        return 1

    xwd = "/tmp/cs2_shot.xwd"
    if not capture_window_xwd(wid, xwd, menu.executor._xdo_env()):
        print("xwd не снялся", file=sys.stderr)
        return 1
    parsed = _parse_xwd(xwd)
    if not parsed:
        print("xwd не распарсился", file=sys.stderr)
        return 1
    img_w, img_h, pixels = parsed

    # xwd хранит пиксели как B,G,R,A (little-endian). Переставляем в RGB.
    img = Image.frombytes("RGBA", (img_w, img_h), bytes(pixels))
    ch = img.split()  # ch[0]=B, ch[1]=G, ch[2]=R, ch[3]=A
    img = Image.merge("RGB", (ch[2], ch[1], ch[0]))

    lb_x, lb_y = layout.letterbox_x, layout.letterbox_y
    cw, ch = layout.client_w, layout.client_h

    if draw_grid:
        d = ImageDraw.Draw(img)
        for i in range(1, 10):
            fx = i / 10.0
            x = lb_x + int(fx * cw)
            d.line([(x, lb_y), (x, lb_y + ch)], fill=(255, 0, 0), width=1)
            d.text((x + 2, lb_y + 2), f"{fx:.1f}", fill=(255, 255, 0))
            y = lb_y + int(fx * ch)
            d.line([(lb_x, y), (lb_x + cw, y)], fill=(0, 200, 255), width=1)
            d.text((lb_x + 2, y + 1), f"{fx:.1f}", fill=(0, 255, 0))
        d.rectangle([lb_x, lb_y, lb_x + cw - 1, lb_y + ch - 1], outline=(0, 255, 0), width=2)

    img.save(out)
    print(f"screen={st.screen_id} client={cw}x{ch} letterbox=({lb_x},{lb_y})")
    print(f"saved: {out} ({img_w}x{img_h})")
    menu.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
