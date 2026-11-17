#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PRACTICE_PYTHON:-python3}"
if [[ -z "${PRACTICE_PYTHON:-}" && -x "$HOME/miniconda3/envs/cobot-native/bin/python" ]]; then
    PYTHON="$HOME/miniconda3/envs/cobot-native/bin/python"
fi
"$PYTHON" -c 'import sys; sys.exit("Python 3.10+ is required.") if sys.version_info < (3, 10) else None'
"$PYTHON" -m venv "$ROOT/.venv"
"$ROOT/.venv/bin/python" -m pip install --timeout 25 --retries 1 -r "$ROOT/requirements.txt"
echo "Ready: $ROOT/run.sh"
