#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
    echo "Run $ROOT/setup.sh first."
    exit 1
fi
exec "$ROOT/.venv/bin/python" "$ROOT/main.py" "$@"
