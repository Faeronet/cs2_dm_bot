"""Load Steam login secrets from local files (never from argv in library code)."""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import struct
import time
from dataclasses import dataclass
from pathlib import Path

_STEAM_GUARD_CHARS = "23456789BCDFGHJKMNPQRTVWXY"


@dataclass(frozen=True)
class LoginCredentials:
    username: str
    password: str


def load_credentials(path: str | Path) -> LoginCredentials:
    """Text file: line1=username, line2=password. Or JSON {\"username\",\"password\"}."""
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"credentials file not found: {p}")
    raw = p.read_text(encoding="utf-8").strip()
    if not raw:
        raise ValueError(f"empty credentials file: {p}")

    if raw.startswith("{"):
        data = json.loads(raw)
        user = str(data.get("username") or data.get("account_name") or "").strip()
        pw = str(data.get("password") or "").strip()
        if not user or not pw:
            raise ValueError(f"credentials JSON needs username and password: {p}")
        return LoginCredentials(user, pw)

    lines = [ln.strip() for ln in raw.splitlines() if ln.strip() and not ln.strip().startswith("#")]
    if len(lines) < 2:
        raise ValueError(f"credentials file needs username + password lines: {p}")
    return LoginCredentials(lines[0], lines[1])


def steam_guard_code(shared_secret_b64: str, *, when: float | None = None) -> str:
    """5-char Steam Guard code (same algorithm as Steam Desktop Authenticator)."""
    secret = base64.b64decode(shared_secret_b64)
    ts = int((when if when is not None else time.time()) // 30)
    msg = struct.pack(">Q", ts)
    digest = hmac.new(secret, msg, hashlib.sha1).digest()
    start = digest[19] & 0x0F
    code_int = struct.unpack(">I", digest[start : start + 4])[0] & 0x7FFFFFFF
    out: list[str] = []
    for _ in range(5):
        out.append(_STEAM_GUARD_CHARS[code_int % len(_STEAM_GUARD_CHARS)])
        code_int //= len(_STEAM_GUARD_CHARS)
    return "".join(out)


def totp_code_from_mafile(path: str | Path) -> str:
    """Steam Guard code from .maFile (shared_secret field)."""
    p = Path(path)
    data = json.loads(p.read_text(encoding="utf-8"))
    secret_b64 = data.get("shared_secret")
    if not secret_b64:
        raise ValueError(f"maFile missing shared_secret: {p}")
    return steam_guard_code(secret_b64)
