#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

case "$(uname -s)" in
    Darwin)
        set +eu
        eval "$("${CONDA_EXE:-conda}" shell.bash hook)"
        conda activate cobot-native
        status=$?
        set -eu
        if ((status)); then exit "$status"; fi
        ;;
    Linux)
        # Ubuntu 24.04: distro Python, Gazebo Harmonic and its Python 3 bindings.
        # Set COBOT_SERVICE_PYTHON to use an equivalent environment.
        ;;
    *)
        echo 'run_service.sh supports macOS and Ubuntu Linux.' >&2
        exit 2
        ;;
esac
if [[ "$(uname -s)" == Darwin ]]; then
    exec "${COBOT_SERVICE_PYTHON:-python}" "$ROOT/native/service.py" "$@"
fi
exec "${COBOT_SERVICE_PYTHON:-python3}" "$ROOT/native/service.py" "$@"
