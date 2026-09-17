"""Видимость врага для локального игрока (EntitySpottedState_t)."""

from __future__ import annotations

from cs2_reader.memory import MemoryBackend


def read_spotted_mask(backend: MemoryBackend, spotted_state: int, mask_off: int) -> tuple[int, int]:
    """m_bSpottedByMask — uint32[2] (8 байт)."""
    data = backend.read(spotted_state + mask_off, 8)
    if not data or len(data) < 4:
        return 0, 0
    m0 = int.from_bytes(data[0:4], "little")
    m1 = int.from_bytes(data[4:8], "little") if len(data) >= 8 else 0
    return m0, m1


def mask_has_bit(m0: int, m1: int, bit: int) -> bool:
    if bit < 0:
        return False
    if bit < 32:
        return bool(m0 & (1 << bit))
    if bit < 64:
        return bool(m1 & (1 << (bit - 32)))
    return False


def spotted_by_viewer(
    backend: MemoryBackend,
    pawn: int,
    entity_spotted_off: int,
    mask_off: int,
    viewer_controller_index: int,
) -> bool:
    """
    Виден ли pawn локальному игроку (не радар, не «замечен кем-то»).

    Только m_bSpottedByMask: бит (controller_index - 1) в uint32[2].
    m_bSpotted намеренно не используем — он true за стеной / от радара.
    """
    if viewer_controller_index <= 0:
        return False
    state = pawn + entity_spotted_off
    m0, m1 = read_spotted_mask(backend, state, mask_off)
    bit = viewer_controller_index - 1
    return mask_has_bit(m0, m1, bit)


def read_radar_spotted(
    backend: MemoryBackend,
    pawn: int,
    entity_spotted_off: int,
    spotted_off: int,
) -> bool:
    """m_bSpotted — на радаре / замечен (не гарантирует прямую видимость)."""
    v = backend.read_u8(pawn + entity_spotted_off + spotted_off)
    return bool(v) if v is not None else False
