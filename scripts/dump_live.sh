#!/usr/bin/env bash
# Dump fresh offsets from running CS2 and keep only driver data files.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec python3 "$ROOT/scripts/update_offsets.py"
