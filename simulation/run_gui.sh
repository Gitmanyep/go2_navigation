#!/usr/bin/env bash
set -euo pipefail
FRAMEWORK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$FRAMEWORK_DIR/simulation.py" --gui "$@"
