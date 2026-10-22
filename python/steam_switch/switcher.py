"""Switch saved Steam accounts without UI automation or injection."""

from __future__ import annotations

import os
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

from steam_switch.paths import SteamPaths, find_steam_paths
from steam_switch.vdf import VdfNode, get_ci, parse_vdf, serialize_vdf, set_ci


@dataclass
class SteamAccount:
    steam_id: str
    account_name: str
    persona_name: str
    remember_password: bool
    auto_login: bool

    @property
    def active(self) -> bool:
        return self.auto_login


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def _atomic_write(path: Path, text: str) -> None:
    backup = path.with_suffix(path.suffix + ".bak")
    shutil.copy2(path, backup)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    os.replace(tmp, path)


def _users_block(root: VdfNode) -> VdfNode | None:
    users_root = root.children.get("users") if root.children else None
    if users_root and users_root.children:
        return users_root
    if root.children:
        only = next(iter(root.children.values()))
        if only.children:
            return only
    return None


def list_accounts(paths: SteamPaths | None = None) -> list[SteamAccount]:
    paths = paths or find_steam_paths()
    root = parse_vdf(_read_text(paths.loginusers_vdf))
    users = _users_block(root)
    if not users or not users.children:
        return []

    out: list[SteamAccount] = []
    for steam_id, block in users.children.items():
        if not block.is_block():
            continue
        name = get_ci(block, "AccountName")
        persona = get_ci(block, "PersonaName")
        remember = get_ci(block, "RememberPassword")
        auto = get_ci(block, "AutoLogin")
        most = get_ci(block, "MostRecent")
        auto_login = (auto and auto.value == "1") or (most and most.value == "1")
        out.append(
            SteamAccount(
                steam_id=steam_id,
                account_name=(name.value if name else "") or "",
                persona_name=(persona.value if persona else "") or "",
                remember_password=(remember.value if remember else "") == "1",
                auto_login=auto_login,
            )
        )
    out.sort(key=lambda a: a.account_name.lower())
    return out


def _resolve_target(accounts: list[SteamAccount], query: str) -> SteamAccount:
    q = query.strip().lower()
    if not q:
        raise ValueError("empty account query")

    for acc in accounts:
        if acc.steam_id == query or acc.steam_id.endswith(q):
            return acc
    for acc in accounts:
        if acc.account_name.lower() == q:
            return acc
    for acc in accounts:
        if acc.persona_name.lower() == q:
            return acc
    partial = [
        a
        for a in accounts
        if q in a.account_name.lower() or q in a.persona_name.lower() or q in a.steam_id
    ]
    if len(partial) == 1:
        return partial[0]
    if len(partial) > 1:
        names = ", ".join(a.account_name for a in partial)
        raise ValueError(f"ambiguous account {query!r}: {names}")
    raise ValueError(f"account not found: {query!r}")


def _patch_loginusers(root: VdfNode, target_id: str) -> None:
    users = _users_block(root)
    if not users or not users.children:
        raise ValueError("loginusers.vdf has no users")
    found = False
    for steam_id, block in users.children.items():
        if not block.is_block():
            continue
        active = steam_id == target_id
        if active:
            found = True
        set_ci(block, "AutoLogin", "1" if active else "0")
        if get_ci(block, "MostRecent"):
            set_ci(block, "MostRecent", "1" if active else "0")
        if active:
            set_ci(block, "RememberPassword", "1")
    if not found:
        raise ValueError(f"steam id not in loginusers.vdf: {target_id}")


def _patch_registry(root: VdfNode, account_name: str) -> None:
    node = root
    for part in ("Registry", "HKCU", "Software", "Valve", "Steam"):
        child = get_ci(node, part)
        if not child or not child.is_block():
            raise ValueError(f"registry.vdf missing {part}")
        node = child
    set_ci(node, "AutoLoginUser", account_name)


def steam_running() -> bool:
    try:
        out = subprocess.run(
            ["pgrep", "-x", "steam"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if out.returncode == 0:
            return True
    except (subprocess.TimeoutExpired, OSError):
        pass
    try:
        out = subprocess.run(
            ["pgrep", "-f", "steam.sh"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        return out.returncode == 0
    except (subprocess.TimeoutExpired, OSError):
        return False


def shutdown_steam(paths: SteamPaths, *, timeout_s: float = 90.0) -> None:
    if not steam_running():
        return
    subprocess.run([*paths.launch_cmd, "-shutdown"], check=False, timeout=30)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not steam_running():
            time.sleep(1.0)
            return
        time.sleep(0.5)
    raise TimeoutError("Steam did not exit after -shutdown (close games and retry)")


def relaunch_steam(paths: SteamPaths, *, silent: bool = False) -> None:
    args = [*paths.launch_cmd]
    if silent:
        args.append("-silent")
    subprocess.Popen(args, start_new_session=True)


def switch_account(
    query: str,
    *,
    paths: SteamPaths | None = None,
    shutdown: bool = True,
    relaunch: bool = True,
    silent: bool = False,
    dry_run: bool = False,
) -> SteamAccount:
    """Switch to a saved account by AccountName, PersonaName, or SteamID64."""
    paths = paths or find_steam_paths()
    accounts = list_accounts(paths)
    if not accounts:
        raise RuntimeError("no saved Steam accounts in loginusers.vdf")
    target = _resolve_target(accounts, query)

    if not target.remember_password:
        raise RuntimeError(
            f"{target.account_name}: RememberPassword is off — log in once with "
            "'Remember password' checked"
        )

    login_root = parse_vdf(_read_text(paths.loginusers_vdf))
    reg_root = parse_vdf(_read_text(paths.registry_vdf))
    _patch_loginusers(login_root, target.steam_id)
    _patch_registry(reg_root, target.account_name)

    if dry_run:
        return target

    if shutdown:
        shutdown_steam(paths)
        time.sleep(0.5)

    _atomic_write(paths.loginusers_vdf, serialize_vdf(login_root))
    _atomic_write(paths.registry_vdf, serialize_vdf(reg_root))

    if relaunch:
        relaunch_steam(paths, silent=silent)
    return target
