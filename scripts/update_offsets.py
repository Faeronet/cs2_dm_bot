#!/usr/bin/env python3
"""Update offsets from live CS2 or GitHub fallback."""

from __future__ import annotations

import subprocess
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
DUMPER = ROOT / "external" / "cs2-dumper" / "target" / "release" / "cs2-dumper"

KEEP = {"offsets.json", "libclient.so.json", "info.json", "client_dll.json"}

GITHUB_OFFSETS = "https://raw.githubusercontent.com/a2x/cs2-dumper/main/output/offsets.json"
GITHUB_CLIENT = "https://raw.githubusercontent.com/a2x/cs2-dumper/main/output/client_dll.json"


def cs2_running() -> bool:
    return subprocess.run(["pgrep", "-f", "linuxsteamrt64/cs2"], capture_output=True).returncode == 0


def prune_data_dir() -> None:
    for path in DATA.iterdir():
        if path.is_file() and path.name not in KEEP:
            path.unlink()


def dump_live() -> None:
    if not DUMPER.exists():
        print("Building cs2-dumper (linux branch)...")
        dumper_dir = ROOT / "external" / "cs2-dumper"
        subprocess.run(["git", "checkout", "linux"], cwd=dumper_dir, check=True)
        subprocess.run(["cargo", "build", "--release"], cwd=dumper_dir, check=True)
    print("Dumping from live CS2...")
    subprocess.run(
        [str(DUMPER), "-o", str(DATA), "-f", "json"],
        cwd=DUMPER.parents[2],
        check=True,
    )
    prune_data_dir()


def download_fallback() -> None:
    print("CS2 not running — downloading Windows offsets from GitHub")
    for name, url in [("offsets.json", GITHUB_OFFSETS), ("client_dll.json", GITHUB_CLIENT)]:
        with urllib.request.urlopen(url, timeout=30) as resp:
            (DATA / name).write_bytes(resp.read())


def main() -> int:
    DATA.mkdir(parents=True, exist_ok=True)
    if cs2_running():
        dump_live()
    else:
        download_fallback()
    print(f"Done. Kept: {', '.join(sorted(KEEP & {p.name for p in DATA.iterdir() if p.is_file()}))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
