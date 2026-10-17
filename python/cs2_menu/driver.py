"""Чтение состояния UI из памяти CS2 (Reader + Panorama + entities)."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from pathlib import Path

from cs2_menu.models import MenuPhase, UIState
from cs2_reader.ladders import entity_designer_name
from cs2_reader.reader import MAX_ENTITIES, Reader

ROOT = Path(__file__).resolve().parents[2]
MAX_PTR = 0x00007FFFFFFFFFFF

# offsets engine window (из data/offsets.json)
_ENGINE_WIN_W = 10490208
_ENGINE_WIN_H = 10490212


def _load_json_offsets(module: str) -> dict[str, int]:
    path = ROOT / "data" / "offsets.json"
    if not path.is_file():
        return {}
    raw = json.loads(path.read_text(encoding="utf-8"))
    items = raw.get(module, [])
    if isinstance(items, list):
        return {x["name"]: x["value"] for x in items}
    return items


def _usable_ptr(ptr: int | None) -> int | None:
    if not ptr or ptr < 0x10000 or ptr > MAX_PTR:
        return None
    return ptr


class UIStateReader:
    """Снимок UI: фаза меню, карта, entity-сигнатуры, fingerprint Panorama."""

    def __init__(self, reader: Reader):
        self.reader = reader
        self._pan_off = _load_json_offsets("libpanorama.so")
        self._mm_off = _load_json_offsets("libmatchmaking.so")

    def read(self) -> UIState:
        snap = self.reader.capture()
        entities = self._entity_counts()
        menu_ctx, fp = self._panorama_fingerprint()
        mm_map = self._matchmaking_map()
        pos = (snap.local_position.x, snap.local_position.y, snap.local_position.z)
        life = snap.local_life

        state = UIState(
            in_game=snap.in_game,
            in_match=snap.in_match,
            map_name=snap.map_name or "",
            matchmaking_map=mm_map,
            local_team=snap.local_team,
            local_alive=bool(life.alive) if life else False,
            local_pos=pos,
            player_count=len(snap.players),
            entity_counts=entities,
            menu_context=menu_ctx or 0,
            ui_fingerprint=fp,
            window_memory=self._window_memory(),
            window_screen={
                "width": int(snap.screen_width or 0),
                "height": int(snap.screen_height or 0),
            },
            raw={
                "build": snap.build_number,
                "weapon": snap.local_weapon.name if snap.local_weapon else "",
            },
        )
        state.phase = self._detect_phase(state)
        return state

    def _entity_counts(self) -> dict[str, int]:
        b = self.reader.backend
        schema = self.reader.schema
        counts: Counter[str] = Counter()
        for i in range(1, min(MAX_ENTITIES, 1024)):
            ent = self.reader.entity_by_index(i)
            if not ent or ent < 0x10000:
                continue
            name = entity_designer_name(b, schema, ent) or "-"
            counts[name] += 1
        return dict(counts)

    def _panorama_fingerprint(self) -> tuple[int | None, str]:
        b = self.reader.backend
        pan = b.module_base("libpanorama.so")
        if not pan:
            return None, ""
        off = self._pan_off.get("MENU_CONTEXT", 0)
        if not off:
            return None, ""
        ctx = _usable_ptr(b.read_u64(pan + off))
        if not ctx:
            return ctx, ""
        blob = b.read(ctx, 512) or b""
        return ctx, hashlib.md5(blob).hexdigest()[:16]

    def _matchmaking_map(self) -> str:
        b = self.reader.backend
        base = b.module_base("libmatchmaking.so")
        off = self._mm_off.get("dwGameTypes_mapName", 0)
        if not base or not off:
            return ""
        ptr = _usable_ptr(b.read_u64(base + off))
        if not ptr:
            return ""
        raw = b.read_string(ptr, 64)
        if not raw:
            return ""
        # иногда мусор + "<empty>"
        if "<empty>" in raw:
            return "<empty>"
        if raw.isprintable() and raw.startswith("de_"):
            return raw
        return ""

    def _window_memory(self) -> tuple[int, int]:
        b = self.reader.backend
        eng = b.module_base("libengine2.so")
        if not eng:
            return 0, 0
        w = b.read_u32(eng + _ENGINE_WIN_W) or 0
        h = b.read_u32(eng + _ENGINE_WIN_H) or 0
        return int(w), int(h)

    def _detect_phase(self, s: UIState) -> MenuPhase:
        ents = s.entity_counts
        if s.in_map():
            return MenuPhase.IN_GAME

        if s.matchmaking_map and s.matchmaking_map not in ("", "<empty>"):
            return MenuPhase.LOADING

        if any(k in ents for k in ("team_select_counterterrorist", "team_select_terrorist")):
            return MenuPhase.TEAM_SELECT

        if ents.get("vote_controller", 0) >= 2 and not s.in_map():
            # часто фоновые контроллеры + popup
            pass

        pos0 = abs(s.local_pos[0]) < 1.0 and abs(s.local_pos[1]) < 1.0
        if not s.in_game and (pos0 or s.local_team == 0):
            # без карты — меню; подэкран уточняется fingerprint-ами в controller
            return MenuPhase.MAIN_MENU

        if s.in_match and not s.in_game:
            return MenuPhase.LOADING

        return MenuPhase.UNKNOWN
