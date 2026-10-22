"""Locate Steam config files (native / snap)."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SteamPaths:
    root: Path
    loginusers_vdf: Path
    registry_vdf: Path
    launch_cmd: list[str]

    def describe(self) -> str:
        return f"root={self.root} launch={' '.join(self.launch_cmd)}"


def _snap_paths(home: Path) -> SteamPaths | None:
    root = home / "snap/steam/common/.steam/steam"
    login = home / "snap/steam/common/.steam/steam/config/loginusers.vdf"
    reg = home / "snap/steam/common/.steam/registry.vdf"
    if login.is_file() and reg.is_file():
        return SteamPaths(
            root=root,
            loginusers_vdf=login,
            registry_vdf=reg,
            launch_cmd=["snap", "run", "steam"],
        )
    return None


def _native_paths(home: Path) -> SteamPaths | None:
    candidates = [
        home / ".local/share/Steam",
        home / ".steam/steam",
    ]
    for root in candidates:
        login = root / "config/loginusers.vdf"
        reg = home / ".steam/registry.vdf"
        if login.is_file() and reg.is_file():
            return SteamPaths(
                root=root,
                loginusers_vdf=login,
                registry_vdf=reg,
                launch_cmd=["steam"],
            )
    return None


def find_steam_paths() -> SteamPaths:
    home = Path.home()
    for finder in (_snap_paths, _native_paths):
        found = finder(home)
        if found:
            return found
    raise FileNotFoundError(
        "Steam config not found (tried snap and ~/.local/share/Steam). "
        "Log into Steam at least once with Remember password."
    )
