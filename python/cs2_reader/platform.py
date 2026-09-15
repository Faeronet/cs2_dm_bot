from __future__ import annotations

import re
import sys
from enum import Enum
from pathlib import Path


class Platform(Enum):
    LINUX_NATIVE = "linux_native"
    WINDOWS = "windows"
    WINE = "wine"


def detect_platform() -> Platform:
    if sys.platform == "win32":
        return Platform.WINDOWS
    return Platform.LINUX_NATIVE


def client_module_name(platform: Platform | None = None) -> str:
    p = platform or detect_platform()
    if p == Platform.LINUX_NATIVE:
        return "libclient.so"
    return "client.dll"


def engine_module_name(platform: Platform | None = None) -> str:
    p = platform or detect_platform()
    if p == Platform.LINUX_NATIVE:
        return "libengine2.so"
    return "engine2.dll"


def schema_file(platform: Platform | None = None) -> Path:
    p = platform or detect_platform()
    data = Path(__file__).resolve().parents[2] / "data"
    if p == Platform.LINUX_NATIVE:
        return data / "libclient.so.json"
    return data / "client_dll.json"


def offsets_file() -> Path:
    return Path(__file__).resolve().parents[2] / "data" / "offsets.json"


def find_cs2_pid() -> int | None:
    if sys.platform == "win32":
        import subprocess

        result = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq cs2.exe", "/FO", "CSV", "/NH"],
            capture_output=True,
            text=True,
        )
        for line in result.stdout.splitlines():
            if "cs2.exe" in line.lower():
                parts = line.replace('"', "").split(",")
                if len(parts) >= 2:
                    try:
                        return int(parts[1])
                    except ValueError:
                        continue
        return None

    best: tuple[int, int] | None = None  # (pid, libclient_count)
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmdline = (entry / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "replace")
            if "cs2.sh" in cmdline:
                continue
            exe = entry / "exe"
            exe_path = exe.resolve().as_posix() if exe.exists() else ""
            if not (exe_path.endswith("/cs2") or re.search(r"/cs2(\s|$|-)", cmdline)):
                continue
            maps_path = entry / "maps"
            lib_count = 0
            if maps_path.exists():
                lib_count = sum(
                    1 for line in maps_path.read_text(errors="replace").splitlines()
                    if "libclient.so" in line
                )
            if lib_count == 0:
                continue
            pid = int(entry.name)
            if best is None or lib_count > best[1]:
                best = (pid, lib_count)
        except (OSError, PermissionError, RuntimeError):
            continue
    return best[0] if best else None
