#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_HOST="${1:?Usage: ./run_real.sh ROBOT_IP [--car real|demo_front|demo_left|demo_right]}"
shift
exec "$ROOT/native/run_display.sh" --mode real --host "$ROBOT_HOST" --car real "$@"
