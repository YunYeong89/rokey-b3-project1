#!/bin/bash
set -euo pipefail
NATIVE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${CONDA_EXE:-conda}" run --no-capture-output -n cobot-native \
  python "$NATIVE_DIR/session.py" "$@"
