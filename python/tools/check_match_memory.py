#!/usr/bin/env python3
"""Быстрая проверка: видит ли reader карту / pawn / спавн (для отладки run_dm)."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_reader.reader import Reader
from tools.launch_dm_official import _signals, normalize_map


def main() -> int:
    r = Reader.from_live()
    if not r:
        print("CS2 не запущен")
        return 1
    client = r._client_base()
    print(f"libclient=0x{client:x}" if client else "no client")
    print(f"offsets: GlobalVars=0x{r.offsets.client.dw_global_vars:x} "
          f"LocalPawn=0x{r.offsets.client.dw_local_player_pawn:x}")
    sig = _signals(r)
    print("signals:", sig)
    print("normalize map:", normalize_map(sig["map"]) or "(empty)")
    if sig["map_loaded"] and sig["alive_fast"]:
        print("OK: бот должен стартовать без меню/GO")
    elif sig["map_loaded"]:
        print("WARN: карта есть, ждём спавн/команду")
    elif sig["pawn"] and sig["health"] > 0:
        print("FAIL: в игре, но map_name пустой — обнови offsets (scripts/update_offsets.py)")
        return 2
    else:
        print("INFO: не в матче (меню/очередь)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
