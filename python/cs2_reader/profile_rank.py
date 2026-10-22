"""Profile Rank (1–40) и XP прогресс — чтение из памяти CS2."""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from pathlib import Path

from cs2_reader.memory import MemoryBackend
from cs2_reader.schema import Schema

MAX_PTR = 0x00007FFFFFFFFFFF
MIN_HEAP_PTR = 0x000100000000
XP_PER_RANK = 5000
MAX_PROFILE_RANK = 40
PERSONA_META = 3635
SENTINEL_PTRS = frozenset({0x200000000, 0x400000000, 0x800000000})


@dataclass
class ProfileRankInfo:
    """XP Rank / Weekly Care Package (ранги 1–40, 5000 XP на уровень)."""

    rank: int = 0
    xp: int = 0
    xp_per_rank: int = XP_PER_RANK
    valid: bool = False
    source: str = ""

    @property
    def xp_to_next(self) -> int:
        if not self.valid:
            return 0
        return max(0, self.xp_per_rank - self.xp)

    @property
    def xp_total(self) -> int:
        if not self.valid or self.rank < 1:
            return 0
        return (self.rank - 1) * self.xp_per_rank + self.xp

    def to_dict(self) -> dict:
        return {
            "rank": self.rank,
            "xp": self.xp,
            "xp_per_rank": self.xp_per_rank,
            "xp_to_next": self.xp_to_next,
            "xp_total": self.xp_total,
            "valid": self.valid,
            "source": self.source,
        }


def _usable_ptr(ptr: int | None) -> int | None:
    if not ptr or ptr < MIN_HEAP_PTR or ptr > MAX_PTR:
        return None
    if ptr in SENTINEL_PTRS:
        return None
    return ptr


def _valid_pair(rank: int, xp: int) -> bool:
    return 1 <= rank <= MAX_PROFILE_RANK and 0 <= xp < XP_PER_RANK


def _score_pair(
    rank: int,
    xp: int,
    *,
    rank_hint: int | None = None,
    meta_after_xp: int | None = None,
    tail_after_rank: int | None = None,
) -> int:
    if not _valid_pair(rank, xp) or xp == PERSONA_META:
        return -1
    score = 50
    if xp >= 50:
        score += 30
    if xp >= 500:
        score += 20
    if rank_hint is not None and rank == rank_hint:
        score += 200
    if meta_after_xp == PERSONA_META:
        score += 80
    if tail_after_rank is not None and 5000 <= tail_after_rank <= 50000:
        score += 60
    return score


class ProfileRankReader:
    """Читает Profile Rank + XP: InventoryServices (ранг) + GC heap (XP)."""

    def __init__(self, backend: MemoryBackend, schema: Schema):
        self.backend = backend
        self.schema = schema
        self._sig_xp_addr: int | None = None
        self._xp_candidates: list[int] = []
        self._xp_prev: dict[int, int] = {}
        self._last_heap_scan = 0.0
        self._last: ProfileRankInfo = ProfileRankInfo()
        self._last_at = 0.0

    def read(self, controller: int | None) -> ProfileRankInfo:
        now = time.monotonic()
        rank_hint = self._fetch_rank_hint(controller) if controller else None

        sig = self._read_sig_cache(rank_hint)
        if sig.valid:
            self._remember(sig)
            return sig

        live = self._pick_live_xp(rank_hint)
        if live is not None:
            self._remember(live)
            return live

        heap_due = now - self._last_heap_scan >= 2.0
        cold_start = not self._sig_xp_addr and not self._xp_candidates
        if (heap_due or cold_start) and rank_hint is not None:
            self._last_heap_scan = now
            self._scan_heap_candidates(rank_hint)
            live = self._pick_live_xp(rank_hint)
            if live is not None:
                self._remember(live)
                return live

        if rank_hint is not None:
            partial = ProfileRankInfo(rank=rank_hint, xp=0, valid=True, source="gc_rank_only")
            return partial

        if self._last.valid and now - self._last_at < 60.0:
            return self._last
        return ProfileRankInfo()

    def _remember(self, info: ProfileRankInfo) -> None:
        self._last = info
        self._last_at = time.monotonic()

    def _fetch_rank_hint(self, controller: int) -> int | None:
        inv = _usable_ptr(
            self.backend.read_u64(controller + self.schema.m_p_inventory_services)
        )
        if inv:
            rank = self.backend.read_i32(inv + self.schema.m_n_persona_data_public_level)
            if rank is not None and 1 <= rank <= MAX_PROFILE_RANK:
                return rank

        blob = self.backend.read(controller, 4096) or b""
        hints: dict[int, int] = {}
        for po in range(0, min(len(blob), 4096), 8):
            p = struct.unpack("<Q", blob[po : po + 8])[0]
            if not _usable_ptr(p):
                continue
            sub = self.backend.read(p, 512) or b""
            if len(sub) < 0x7C:
                continue
            rank = struct.unpack("<i", sub[0x74 : 0x78])[0]
            if not (1 <= rank <= MAX_PROFILE_RANK):
                continue
            tail = struct.unpack("<i", sub[0x78 : 0x7C])[0]
            if tail not in range(0, 6):
                continue
            hints[rank] = hints.get(rank, 0) + 1
        if hints:
            return max(hints, key=lambda r: hints[r])
        if self._last.valid and self._last.rank > 0:
            return self._last.rank
        return None

    def _read_sig_cache(self, rank_hint: int | None) -> ProfileRankInfo:
        if not self._sig_xp_addr:
            return ProfileRankInfo()
        xp = self.backend.read_i32(self._sig_xp_addr)
        rank = self.backend.read_i32(self._sig_xp_addr + 16)
        if xp is None or rank is None or not _valid_pair(rank, xp):
            self._sig_xp_addr = None
            return ProfileRankInfo()
        if rank_hint is not None and rank != rank_hint:
            self._sig_xp_addr = None
            self._xp_candidates.clear()
            self._xp_prev.clear()
            return ProfileRankInfo()
        return ProfileRankInfo(rank=rank, xp=xp, valid=True, source="gc_cache")

    def _pick_live_xp(self, rank_hint: int | None) -> ProfileRankInfo | None:
        if rank_hint is None or not self._xp_candidates:
            return None

        readings: dict[int, tuple[int, int]] = {}
        for addr in self._xp_candidates:
            xp = self.backend.read_i32(addr)
            rank = self.backend.read_i32(addr + 16)
            if xp is None or rank is None or not _valid_pair(rank, xp):
                continue
            if rank != rank_hint:
                continue
            readings[addr] = (xp, rank)

        if not readings:
            return None

        if len(readings) == 1:
            addr, (xp, rank) = next(iter(readings.items()))
            self._sig_xp_addr = addr
            return ProfileRankInfo(rank=rank, xp=xp, valid=True, source="gc_heap")

        changed = [
            (addr, xp)
            for addr, (xp, _rank) in readings.items()
            if addr in self._xp_prev and xp != self._xp_prev[addr]
        ]
        self._xp_prev = {addr: xp for addr, (xp, _rank) in readings.items()}

        if len(changed) == 1:
            addr, xp = changed[0]
            self._sig_xp_addr = addr
            return ProfileRankInfo(rank=rank_hint, xp=xp, valid=True, source="gc_live")

        if self._sig_xp_addr and self._sig_xp_addr in readings:
            xp, rank = readings[self._sig_xp_addr]
            return ProfileRankInfo(rank=rank, xp=xp, valid=True, source="gc_cache")

        if self._last.valid and self._last.xp > 0:
            nearest = min(
                readings.items(),
                key=lambda item: abs(item[1][0] - self._last.xp),
            )
            addr, (xp, rank) = nearest
            self._sig_xp_addr = addr
            return ProfileRankInfo(rank=rank, xp=xp, valid=True, source="gc_heap")

        if self._xp_prev:
            addr = min(readings, key=lambda a: a)
            xp, rank = readings[addr]
            self._sig_xp_addr = addr
            return ProfileRankInfo(rank=rank, xp=xp, valid=True, source="gc_heap")

        return None

    def _scan_heap_candidates(self, rank_hint: int) -> None:
        pid = getattr(self.backend, "pid", None)
        if not pid:
            return
        maps_path = Path(f"/proc/{pid}/maps")
        if not maps_path.exists():
            return

        rank_pat = struct.pack("<i", rank_hint)
        scored: list[tuple[int, int, int]] = []
        scanned = 0
        max_scan = 64 * 1024 * 1024

        regions: list[tuple[int, int, int]] = []
        for line in maps_path.read_text().splitlines():
            parts = line.split()
            if len(parts) < 2 or "r" not in parts[1]:
                continue
            start_s, end_s = parts[0].split("-")
            start, end = int(start_s, 16), int(end_s, 16)
            size = end - start
            if size < 4096 or size > 64 * 1024 * 1024:
                continue
            priority = 2 if len(parts) >= 6 and parts[5] == "[heap]" else 1
            regions.append((priority, start, end))
        regions.sort(key=lambda item: (-item[0], item[1]))

        for _, start, end in regions:
            if scanned >= max_scan:
                break
            step = 1024 * 1024
            for base in range(start, end, step):
                if scanned >= max_scan:
                    break
                chunk = self.backend.read(base, min(step, end - base))
                if not chunk:
                    continue
                scanned += len(chunk)
                idx = 0
                while True:
                    hit = chunk.find(rank_pat, idx)
                    if hit < 0:
                        break
                    if hit < 16 or hit + 12 > len(chunk):
                        idx = hit + 4
                        continue
                    xp = struct.unpack("<i", chunk[hit - 16 : hit - 12])[0]
                    meta = struct.unpack("<i", chunk[hit - 8 : hit - 4])[0]
                    tail = struct.unpack("<i", chunk[hit + 8 : hit + 12])[0]
                    score = _score_pair(
                        rank_hint,
                        xp,
                        rank_hint=rank_hint,
                        meta_after_xp=meta,
                        tail_after_rank=tail,
                    )
                    if score < 400 or meta != PERSONA_META:
                        idx = hit + 4
                        continue
                    scored.append((score, base + hit - 16, xp))
                    idx = hit + 4

        if not scored:
            self._xp_candidates.clear()
            return

        top_score = max(item[0] for item in scored)
        self._xp_candidates = [addr for sc, addr, _xp in scored if sc == top_score]
