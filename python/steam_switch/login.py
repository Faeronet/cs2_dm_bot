"""Log into Steam client with a new (or unsaved) account via `steam -login`."""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

from steam_switch.credentials import LoginCredentials, load_credentials, totp_code_from_mafile
from steam_switch.paths import SteamPaths, find_steam_paths
from steam_switch.switcher import SteamAccount, list_accounts, shutdown_steam, steam_running


def _find_saved(username: str, paths: SteamPaths) -> SteamAccount | None:
    want = username.strip().lower()
    for acc in list_accounts(paths):
        if acc.account_name.lower() == want:
            return acc
    return None


def _wait_for_saved_account(
    username: str,
    paths: SteamPaths,
    *,
    timeout_s: float = 180.0,
    poll_s: float = 2.0,
) -> SteamAccount:
    deadline = time.monotonic() + timeout_s
    want = username.strip().lower()
    while time.monotonic() < deadline:
        acc = _find_saved(username, paths)
        if acc and acc.remember_password:
            return acc
        if not steam_running():
            raise RuntimeError(
                "Steam exited before login finished — wrong password, bad Steam Guard code, "
                "or email Guard (need --auth-code from mail)"
            )
        time.sleep(poll_s)
    raise TimeoutError(
        f"account {username!r} did not appear in loginusers.vdf within {timeout_s:.0f}s"
    )


def login_account(
    username: str,
    password: str,
    *,
    auth_code: str | None = None,
    mafile: str | Path | None = None,
    paths: SteamPaths | None = None,
    shutdown: bool = True,
    silent: bool = False,
    timeout_s: float = 180.0,
    dry_run: bool = False,
) -> SteamAccount:
    """First-time (or fresh) client login. Requires password + Steam Guard.

    * Mobile Authenticator: pass --mafile path (.maFile with shared_secret).
    * Email Guard (first machine): pass --auth-code from email (one-shot).
    * After success the account is saved in loginusers.vdf; use switch_account() next time.

    Note: password is passed to `steam -login` and may briefly appear in `ps`.
    Use a credentials file with mode 0600, not shell history.
    """
    paths = paths or find_steam_paths()
    user = username.strip()
    if not user or not password:
        raise ValueError("username and password required")

    existing = _find_saved(user, paths)
    if existing and existing.remember_password:
        return existing

    code = (auth_code or "").strip() or None
    if not code and mafile:
        code = totp_code_from_mafile(mafile)

    if dry_run:
        return SteamAccount(
            steam_id="0",
            account_name=user,
            persona_name="",
            remember_password=False,
            auto_login=False,
        )

    if shutdown:
        shutdown_steam(paths)
        time.sleep(1.0)

    args = [*paths.launch_cmd, "-login", user, password]
    if code:
        args.append(code)
    if silent:
        args.append("-silent")

    subprocess.Popen(args, start_new_session=True)
    return _wait_for_saved_account(user, paths, timeout_s=timeout_s)


def login_from_files(
    credentials: str | Path,
    *,
    mafile: str | Path | None = None,
    auth_code: str | None = None,
    **kwargs,
) -> SteamAccount:
    creds = load_credentials(credentials)
    return login_account(
        creds.username,
        creds.password,
        auth_code=auth_code,
        mafile=mafile,
        **kwargs,
    )
