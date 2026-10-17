#!/usr/bin/env python3
"""Распознавание и закрытие окон CS2 по сигнатуре из памяти (офсет Panorama).

Сигнатура окна = хэш контекста Panorama (libpanorama.so + MENU_CONTEXT) из памяти.
По ней классифицируем окно (ошибка/уведомление/Prime/чистое меню), затем наводим
курсор на кнопку (OK / Close / «больше не показывать»), проверяем подсветку и кликаем.
Позиции кнопок хранятся по сигнатуре как доли КОНТЕНТА окна (устойчивы к разрешению).

Команды:
  scan                         показать текущую сигнатуру и вердикт (окно/нет)
  learn <id> [desc]            привязать ТЕКУЩИЙ fingerprint к сигнатуре <id>
    [--clean]                    пометить как «чистое меню» (окна нет)
    [--ok FX FY] [--close FX FY] [--dont FX FY] [--yes FX FY] [--accept FX FY]
                                 задать позиции кнопок (доли контента 0..1)
  hover <id|auto> [label]      навести курсор на кнопку (без клика) и показать подсветку
  close [auto]                 распознать и закрыть текущее окно (hover→клик→проверка)

Примеры:
  python3 python/tools/window_guard.py scan
  python3 python/tools/window_guard.py learn prime_upsell "Prime upsell" --close 0.715 0.912
  python3 python/tools/window_guard.py close
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu import MenuController
from cs2_menu.driver import UIStateReader
from cs2_menu.xwl_pointer import ScreenClicker, ensure_flat_accel
from cs2_menu.window_signatures import (
    Button, WindowSig, classify, close_window, hover_button,
    learn_fingerprint, load_registry,
)
from cs2_reader.reader import Reader


def _parse_buttons(argv: list[str]) -> list[Button]:
    labels = {"--ok": "ok", "--close": "close", "--dont": "dont_show",
              "--yes": "yes", "--accept": "accept"}
    out: list[Button] = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in labels and i + 2 < len(argv):
            out.append(Button(labels[a], float(argv[i + 1]), float(argv[i + 2])))
            i += 3
        else:
            i += 1
    return out


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    cmd = sys.argv[1]
    ensure_flat_accel()

    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен")
        return 1
    menu = MenuController(reader)
    driver = UIStateReader(reader)
    env = menu.executor._xdo_env()
    clicker = ScreenClicker(menu.executor.menu_input, env=env)

    def fp_now() -> str:
        return driver.read().ui_fingerprint

    try:
        if cmd == "scan":
            st = driver.read()
            verdict = classify(st.ui_fingerprint, clicker=clicker, env=env)
            print(f"fingerprint = {st.ui_fingerprint!r}")
            print(f"phase       = {st.phase.value}  in_game={st.in_game}  map={st.map_name!r}")
            sid = verdict.sig.id if verdict.sig else "—"
            print(f"сигнатура   = {sid}  (source={verdict.source})")
            print(f"окно открыто= {verdict.window_open}  modal={verdict.modal_present} "
                  f"ring={verdict.ring_mean:.1f}")
            print("\nреестр сигнатур:")
            for s in load_registry().values():
                btns = ", ".join(f"{b.label}({b.fx:.3f},{b.fy:.3f})" for b in s.buttons)
                print(f"  {s.id:14s} clean={s.is_clean} fps={len(s.fingerprints)} "
                      f"buttons=[{btns}]")
            return 0

        if cmd == "learn":
            if len(sys.argv) < 3:
                print("нужно: learn <id> [desc] [--clean] [--ok FX FY] …")
                return 1
            sid = sys.argv[2]
            rest = sys.argv[3:]
            is_clean = "--clean" in rest
            desc_parts = [a for a in rest if not a.startswith("--")
                          and not _is_float(a)]
            desc = desc_parts[0] if desc_parts else ""
            buttons = _parse_buttons(rest)
            fp = fp_now()
            sig = learn_fingerprint(sid, fp, description=desc,
                                    buttons=buttons or None, is_clean=is_clean)
            print(f"запомнено: {sid} += fp {fp!r}")
            print(f"  clean={sig.is_clean} fps={sorted(sig.fingerprints)}")
            if sig.buttons:
                print("  кнопки: " + ", ".join(
                    f"{b.label}({b.fx:.3f},{b.fy:.3f})" for b in sig.buttons))
            return 0

        if cmd == "hover":
            if len(sys.argv) < 3:
                print("нужно: hover <id|auto> [label]")
                return 1
            which = sys.argv[2]
            label = sys.argv[3] if len(sys.argv) > 3 else ""
            reg = load_registry()
            if which == "auto":
                v = classify(fp_now(), clicker=clicker, env=env, registry=reg)
                sig = v.sig
            else:
                sig = reg.get(which)
            if not sig or not sig.buttons:
                print(f"нет кнопок для '{which}'")
                return 2
            btn = next((b for b in sig.ordered_buttons() if not label or b.label == label),
                       sig.ordered_buttons()[0])
            res = hover_button(clicker, env, btn)
            print(f"hover {sig.id}/{btn.label} @({btn.fx:.3f},{btn.fy:.3f}) → {res['screen']}")
            print(f"  luma {res['luma_before']} → {res['luma_after']}  "
                  f"подсветка={'ДА' if res['highlighted'] else 'нет'}")
            return 0

        if cmd == "close":
            ok = close_window(clicker, env, reader_state_fn=fp_now)
            print(f"close_window -> {ok}")
            return 0 if ok else 2

        print(f"неизвестная команда: {cmd}")
        print(__doc__)
        return 1
    finally:
        menu.close()


def _is_float(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


if __name__ == "__main__":
    raise SystemExit(main())
