#!/bin/bash
# ./run_virtual.sh [Gazebo session options]
# ./run_virtual.sh --web [--car VIRTUAL_FIXTURE] [--car-pose X Y YAW] [--no-gazebo-gui | --no-gui] [--check]
#   --web: only the virtual /robot_2 of run_real.sh --web (Gazebo physics server, demo_final by default) and the web panel.
set -euo pipefail
LAUNCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for option in "$@"; do
    if [[ "$option" == --web ]]; then
        for other in "$@"; do
            [[ "$other" != --virtual-count* ]] || { echo '--web starts exactly one virtual robot (/robot_2).' >&2; exit 2; }
        done
        exec "$LAUNCH_DIR/run_service.sh" --virtual-count 1 --render --car demo_final "$@"
    fi
done
exec "$LAUNCH_DIR/run_display.sh" --mode virtual "$@"
