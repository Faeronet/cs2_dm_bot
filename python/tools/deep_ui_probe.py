#!/usr/bin/env python3
"""Глубокий поиск координат UI-кнопок в памяти CS2."""

from __future__ import annotations

import json
import os
import re
import struct
import sys
from collections import Counter, deque

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from cs2_menu.driver import UIStateReader
from cs2_reader.reader import Reader

MAX_PTR = 0x00007FFFFFFFFFFF


def main() -> int:
    reader = Reader.from_live()
    if not reader:
        print("CS2 не запущен", file=sys.stderr)
        return 1

    b = reader.backend
    pid = b.pid
    st = UIStateReader(reader).read()

    print("=== DEEP UI PROBE ===", flush=True)
    print(f"phase={st.phase} fp={st.ui_fingerprint} ctx=0x{st.menu_context:x}", flush=True)
    print(f"window_mem={st.window_memory} screen={st.window_screen}", flush=True)

    maps_txt = open(f"/proc/{pid}/maps", encoding="utf-8").read().splitlines()

    pan = b.module_base("libpanorama.so")
    eng = b.module_base("libengine2.so")
    off_pan = {x["name"]: x["value"] for x in json.load(open("data/offsets.json"))["libpanorama.so"]}
    menu_off = off_pan["MENU_CONTEXT"]

    # [1] UI modules
    mods: dict[str, int] = {}
    for line in maps_txt:
        parts = line.split()
        if len(parts) < 6:
            continue
        path = parts[-1]
        if path.startswith("/") and any(k in path.lower() for k in ("panorama", "v8", "scaleform")):
            mods.setdefault(path, int(parts[0].split("-")[0], 16))
    print(f"\n[1] UI modules: {len(mods)}", flush=True)
    for p, base in sorted(mods.items(), key=lambda x: x[1]):
        print(f"  0x{base:x} {os.path.basename(p)}", flush=True)

    # [2] libpanorama layout symbols (static code, not runtime coords)
    pan_blob = b.read(pan, 4 * 1024 * 1024) or b""
    print("\n[2] libpanorama.so symbols (static):", flush=True)
    for kw in (
        b"CPanel2D", b"CUIPanel", b"GetPositionWithinWindow", b"actualLayout",
        b"desiredLayout", b"mainmenu_play", b"PlayButton", b"Deathmatch",
    ):
        n = len(re.findall(re.escape(kw), pan_blob))
        if n:
            i = pan_blob.find(kw)
            sample = re.sub(rb"[^\x20-\x7e]", b".", pan_blob[i : i + 64]).decode()
            print(f"  {kw.decode():28s} x{n}  {sample!r}", flush=True)

    # [3] BFS from MENU_CONTEXT + nearby globals
    globals_scan: list[tuple[int, int]] = []
    for rel in range(menu_off - 0x200, menu_off + 0x400, 8):
        ptr = b.read_u64(pan + rel)
        if ptr and 0x100000000 < ptr < MAX_PTR:
            globals_scan.append((rel, ptr))

    def score_rect(x: float, y: float, w: float, h: float) -> int:
        s = 0
        if 0 <= x <= 1920 and 0 <= y <= 1200:
            s += 1
        if 10 <= w <= 900 and 5 <= h <= 300:
            s += 1
        if 50 <= w <= 500 and 15 <= h <= 120:
            s += 2
        if 0 <= x <= 1 and 0 <= y <= 1 and w <= 1 and h <= 1:
            s += 3
        return s

    candidates: list[dict] = []
    seen: set[int] = set()
    roots = [(pan + menu_off, "MENU_CONTEXT")]
    for rel, _ in globals_scan[:8]:
        roots.append((pan + rel, f"pan+0x{rel:x}"))

    for root_addr, root_name in roots:
        ptr = b.read_u64(root_addr)
        if not ptr or ptr in seen:
            continue
        q: deque[tuple[int, int, str]] = deque([(ptr, 0, root_name)])
        seen.add(ptr)
        while q and len(seen) < 2500:
            addr, depth, path = q.popleft()
            if depth > 5:
                continue
            blob = b.read(addr, 8192)
            if not blob:
                continue
            strs = [m.group().decode("ascii", "replace") for m in re.finditer(rb"[\x20-\x7e]{5,64}", blob)]
            ui_strs = [
                s
                for s in strs
                if any(k in s.lower() for k in ("mainmenu", "play", "button", "deathmatch", "matchmaking", "store"))
                and not any(b in s.lower() for b in ("player", "protobuf", "sound", "video"))
            ]
            rects = []
            for ro in range(0, len(blob) - 16, 4):
                x, y, w, h = struct.unpack("<4f", blob[ro : ro + 16])
                sc = score_rect(x, y, w, h)
                if sc >= 4:
                    rects.append((ro, round(x, 4), round(y, 4), round(w, 4), round(h, 4), sc))
            norm_pts = []
            for ro in range(0, len(blob) - 8, 4):
                x, y = struct.unpack("<2f", blob[ro : ro + 8])
                if 0.01 <= x <= 0.99 and 0.01 <= y <= 0.99:
                    norm_pts.append((ro, round(x, 4), round(y, 4)))
            if ui_strs or rects:
                candidates.append(
                    {"addr": hex(addr), "depth": depth, "path": path, "ui_strs": ui_strs[:6], "rects": rects[:4], "norm": norm_pts[:4]}
                )
            for po in range(0, min(len(blob), 4096), 8):
                p = struct.unpack("<Q", blob[po : po + 8])[0]
                if 0x100000000 < p < MAX_PTR and p not in seen:
                    seen.add(p)
                    q.append((p, depth + 1, f"{path}+0x{po:x}"))

    both = [c for c in candidates if c["ui_strs"] and c["rects"]]
    print(f"\n[3] BFS nodes={len(seen)} candidates={len(candidates)} BOTH(str+rect)={len(both)}", flush=True)
    for c in sorted(candidates, key=lambda x: (-len(x["ui_strs"]), -len(x["rects"])))[:12]:
        print(f"  {c['addr']} d={c['depth']} {c['path']}", flush=True)
        if c["ui_strs"]:
            print(f"    str: {c['ui_strs']}", flush=True)
        if c["rects"]:
            print(f"    rect: {c['rects']}", flush=True)
        if c["norm"]:
            print(f"    norm: {c['norm']}", flush=True)

    # [4] Heap: known layout fractions from layout.py
    targets = [("NAV_PLAY", 0.048, 0.21), ("NAV_STORE", 0.048, 0.36), ("MODE_DM", 0.19, 0.35), ("GO", 0.50, 0.845)]
    coord_hits: Counter[str] = Counter()
    print("\n[4] Heap: floats matching layout.py (±0.015):", flush=True)
    heap_regions = 0
    for line in maps_txt:
        perms = line.split()[1]
        if "w" not in perms:
            continue
        if len(line.split()) >= 6 and line.split()[-1].endswith(".so"):
            continue
        start, end = [int(x, 16) for x in line.split()[0].split("-")]
        size = end - start
        if size < 65536 or size > 16 * 1024 * 1024:
            continue
        heap_regions += 1
        chunk = b.read(start, min(size, 2 * 1024 * 1024))
        if not chunk:
            continue
        for name, tx, ty in targets:
            for ro in range(0, len(chunk) - 8, 4):
                x, y = struct.unpack("<2f", chunk[ro : ro + 8])
                if abs(x - tx) < 0.015 and abs(y - ty) < 0.015:
                    coord_hits[name] += 1
                    if coord_hits[name] <= 2:
                        print(f"  {name} @ 0x{start + ro:x} ({x:.4f},{y:.4f})", flush=True)
    print(f"  regions scanned={heap_regions} totals={dict(coord_hits)}", flush=True)

    # [5] Pixel rects in Play zone
    pixel_hits: list[tuple] = []
    for line in maps_txt:
        perms = line.split()[1]
        if "w" not in perms:
            continue
        if len(line.split()) >= 6 and ".so" in line.split()[-1]:
            continue
        start, end = [int(x, 16) for x in line.split()[0].split("-")]
        size = end - start
        if size < 65536 or size > 8 * 1024 * 1024:
            continue
        chunk = b.read(start, min(size, 2 * 1024 * 1024))
        if not chunk:
            continue
        for ro in range(0, len(chunk) - 16, 4):
            x, y, w, h = struct.unpack("<4f", chunk[ro : ro + 16])
            if 20 <= x <= 120 and 120 <= y <= 250 and 40 <= w <= 300 and 20 <= h <= 80:
                pixel_hits.append((start + ro, round(x, 1), round(y, 1), round(w, 1), round(h, 1)))
    print(f"\n[5] Pixel rects Play zone: {len(pixel_hits)}", flush=True)
    if pixel_hits:
        print(f"  sample: {pixel_hits[:8]}", flush=True)

    # [6] engine window size brute
    print("\n[6] engine2 window u32 pairs:", flush=True)
    if eng:
        known = 10490208
        found = False
        for delta in range(-1024, 1024, 4):
            w = b.read_u32(eng + known + delta) or 0
            h = b.read_u32(eng + known + delta + 4) or 0
            if w in (1280, 1308, 1920) and h in (720, 800, 866, 1080, 1200):
                print(f"  eng+0x{known + delta:x}: {w}x{h}", flush=True)
                found = True
        if not found:
            print("  (nothing sane near known offset — window_from_memory stays 0x0)", flush=True)

    # [7] mainmenu_play strings + nearby floats
    print("\n[7] mainmenu_play tokens in heap:", flush=True)
    mp = 0
    for line in maps_txt:
        if "r" not in line.split()[1]:
            continue
        start, end = [int(x, 16) for x in line.split()[0].split("-")]
        size = end - start
        if size > 20 * 1024 * 1024:
            continue
        chunk = b.read(start, min(size, 2 * 1024 * 1024))
        if not chunk:
            continue
        kw = b"mainmenu_play"
        idx = 0
        while True:
            i = chunk.find(kw, idx)
            if i < 0:
                break
            after = chunk[i + len(kw)] if i + len(kw) < len(chunk) else 0
            if after in (0, *range(0x20)) or after in b"\"'();._":
                mp += 1
                if mp <= 10:
                    ctx = re.sub(rb"[^\x20-\x7e]", b".", chunk[i : i + 40]).decode()
                    win = chunk[max(0, i - 128) : i + 128]
                    near = []
                    for ro in range(0, len(win) - 4, 4):
                        fv = struct.unpack("<f", win[ro : ro + 4])[0]
                        if 0.01 <= fv <= 1920:
                            near.append(round(fv, 3))
                    playish = [v for v in near if (0.03 <= v <= 0.25) or (50 <= v <= 250)]
                    print(f"  0x{start + i:x} {ctx!r} playish_floats={playish[:8]}", flush=True)
            idx = i + 1
    print(f"  total tokens: {mp}", flush=True)

    print("\n=== DONE ===", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
