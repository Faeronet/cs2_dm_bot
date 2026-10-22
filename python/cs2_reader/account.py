"""Steam account ID локального игрока — чтение из памяти CS2."""

from __future__ import annotations

from dataclasses import dataclass

STEAM_ID64_BASE = 76561197960265728


@dataclass
class LocalAccountInfo:
    steam_id64: int = 0
    account_id: int = 0
    valid: bool = False

    def to_dict(self) -> dict:
        return {
            "steam_id64": str(self.steam_id64),
            "account_id": self.account_id,
            "valid": self.valid,
        }


def is_valid_steam_id64(steam_id64: int | None) -> bool:
    if not steam_id64 or steam_id64 <= 0:
        return False
    if steam_id64 < STEAM_ID64_BASE:
        return False
    if steam_id64 >= STEAM_ID64_BASE + (1 << 32):
        return False
    return True


def steam_account_id(steam_id64: int) -> int:
    """32-bit Steam account ID (GC / inventory)."""
    if steam_id64 >= STEAM_ID64_BASE:
        return steam_id64 - STEAM_ID64_BASE
    return steam_id64 & 0xFFFFFFFF
