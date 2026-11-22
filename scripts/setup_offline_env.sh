#!/usr/bin/env bash
# No sudo, no writes to system/site/user packages. Wheels must be supplied explicitly.
set -euo pipefail
NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# != 1 ]]; then echo "Usage: $0 /absolute/path/to/wheels" >&2; exit 2; fi
NAV_WHEELS="$(realpath -- "$1")"
if [[ -e "$NAV_ROOT/.venv" ]]; then echo 'Refusing to replace existing .venv' >&2; exit 2; fi
/usr/bin/python3 -m venv --system-site-packages --without-pip "$NAV_ROOT/.venv"
NAV_SITE="$NAV_ROOT/.venv/lib/python3.8/site-packages"
PYTHONNOUSERSITE=1 /usr/bin/python3 -m pip install --no-index --find-links "$NAV_WHEELS" \
    --target "$NAV_SITE" --ignore-installed --no-compile -r "$NAV_ROOT/config/requirements-py38.lock"
PYTHONNOUSERSITE=1 "$NAV_ROOT/.venv/bin/python3" -c \
    'import numpy,scipy,matplotlib; print(numpy.__version__,scipy.__version__,matplotlib.__version__)'
