#!/bin/bash
set -euo pipefail
PACKAGE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$PACKAGE/../../../../.." && pwd)"
set +eu
eval "$("${CONDA_EXE:-conda}" shell.bash hook)"
conda activate cobot-native
set -eu
export COBOT_WORKSPACE="$ROOT"
export PYTHONPATH="$PACKAGE${PYTHONPATH:+:$PYTHONPATH}"
RUNNER=python
if [[ "$(uname -s)" == Darwin ]]; then
    RUNNER=mjpython
    for arg in "$@"; do [[ "$arg" == --no-gui ]] && RUNNER=python; done
fi
exec "$RUNNER" -m dsr_mujoco.bootstrap --mode virtual "$@"
