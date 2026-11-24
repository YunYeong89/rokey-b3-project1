#!/bin/bash
set -euo pipefail
NATIVE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../native" && pwd)"
# session.py runs in the foreground and owns all cleanup: Ctrl+C / TERM / HUP reach it directly. Started as
# `conda run ... &`, it ignored SIGINT (background job) and conda run did not forward TERM, so Ctrl+C left the
# session running until RViz or the terminal was closed.
set +eu  # conda activation scripts are not nounset / errexit safe
eval "$("${CONDA_EXE:-conda}" shell.bash hook)"
conda activate cobot-native
set -eu
exec python "$NATIVE_DIR/session.py" "$@"
