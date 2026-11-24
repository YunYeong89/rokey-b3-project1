#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
exec "$ROOT/ws_cobot_pjt/src/repair_car/launch/run_display.sh" --mode real --observe-only "$@"
