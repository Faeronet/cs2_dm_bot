#!/usr/bin/env python3
"""Steam accounts: switch saved logins or log in a new account (Linux, no injection).

Saved accounts: edit loginusers.vdf + registry.vdf, then restart Steam.
New accounts: `steam -login` + Steam Guard (maFile TOTP or --auth-code from email).

Disable "Ask which account to use each time Steam starts" in Steam settings.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from steam_switch.credentials import load_credentials
from steam_switch.login import login_account, login_from_files
from steam_switch.paths import find_steam_paths
from steam_switch.switcher import list_accounts, switch_account


def _cmd_list(_: argparse.Namespace) -> int:
    paths = find_steam_paths()
    accounts = list_accounts(paths)
    print(f"Steam: {paths.describe()}\n")
    for acc in accounts:
        mark = "*" if acc.active else " "
        print(f"{mark} {acc.account_name:24s}  {acc.persona_name:20s}  {acc.steam_id}")
    return 0


def _cmd_switch(args: argparse.Namespace) -> int:
    paths = find_steam_paths()
    target = switch_account(
        args.account,
        paths=paths,
        shutdown=not args.no_shutdown,
        relaunch=not args.no_relaunch,
        silent=args.silent,
        dry_run=args.dry_run,
    )
    mode = "dry-run" if args.dry_run else "switched"
    print(f"{mode}: {target.account_name} ({target.persona_name}) id={target.steam_id}")
    return 0


def _cmd_login(args: argparse.Namespace) -> int:
    common = dict(
        mafile=args.mafile,
        auth_code=args.auth_code,
        silent=args.silent,
        timeout_s=args.timeout,
        dry_run=args.dry_run,
        shutdown=not args.no_shutdown,
    )
    if args.credentials:
        target = login_from_files(args.credentials, **common)
        user = load_credentials(args.credentials).username
    else:
        if not args.username or not args.password_file:
            print("login needs --credentials FILE or --username + --password-file", file=sys.stderr)
            return 1
        creds = load_credentials(args.password_file)
        if creds.username and args.username and creds.username.lower() != args.username.lower():
            print("warning: --username overrides credentials file username", file=sys.stderr)
        user = args.username or creds.username
        target = login_account(user, creds.password, **common)

    if args.dry_run:
        print(f"dry-run login: {user}")
        return 0

    print(f"logged in: {target.account_name} ({target.persona_name}) id={target.steam_id}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Steam account switch / login (Linux)")
    sub = parser.add_subparsers(dest="cmd")

    p_list = sub.add_parser("list", help="List saved accounts")
    p_list.set_defaults(func=_cmd_list)

    p_sw = sub.add_parser("switch", help="Switch to a saved account (VDF)")
    p_sw.add_argument("account", help="AccountName / PersonaName / SteamID64")
    p_sw.add_argument("--dry-run", action="store_true")
    p_sw.add_argument("--no-shutdown", action="store_true")
    p_sw.add_argument("--no-relaunch", action="store_true")
    p_sw.add_argument("--silent", action="store_true")
    p_sw.set_defaults(func=_cmd_switch)

    p_in = sub.add_parser("login", help="Log in new/unsaved account (steam -login + Guard)")
    p_in.add_argument("--credentials", "-c", help="File: line1=user line2=pass (or JSON)")
    p_in.add_argument("--username", "-u", help="Account name (optional if in credentials file)")
    p_in.add_argument("--password-file", help="Same format as --credentials (legacy alias)")
    p_in.add_argument("--mafile", "-m", help=".maFile with shared_secret (mobile authenticator)")
    p_in.add_argument("--auth-code", help="Steam Guard code (email or app), if no --mafile")
    p_in.add_argument("--timeout", type=float, default=180.0, help="Wait for loginusers.vdf (sec)")
    p_in.add_argument("--dry-run", action="store_true")
    p_in.add_argument("--no-shutdown", action="store_true")
    p_in.add_argument("--silent", action="store_true")
    p_in.set_defaults(func=_cmd_login)

    args = parser.parse_args()
    if not args.cmd:
        # backward compat: `steam_switch.py --list` / `steam_switch.py ACCOUNT`
        legacy = argparse.ArgumentParser(add_help=False)
        legacy.add_argument("account", nargs="?")
        legacy.add_argument("--list", action="store_true")
        legacy.add_argument("--dry-run", action="store_true")
        legacy.add_argument("--no-shutdown", action="store_true")
        legacy.add_argument("--no-relaunch", action="store_true")
        legacy.add_argument("--silent", action="store_true")
        leg, _ = legacy.parse_known_args(sys.argv[1:])
        if leg.list or not leg.account:
            try:
                return _cmd_list(argparse.Namespace())
            except FileNotFoundError as exc:
                print(exc, file=sys.stderr)
                return 1
        try:
            return _cmd_switch(
                argparse.Namespace(
                    account=leg.account,
                    dry_run=leg.dry_run,
                    no_shutdown=leg.no_shutdown,
                    no_relaunch=leg.no_relaunch,
                    silent=leg.silent,
                )
            )
        except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2

    try:
        return args.func(args)
    except FileNotFoundError as exc:
        print(exc, file=sys.stderr)
        return 1
    except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
