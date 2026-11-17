#!/bin/bash
# ./run_virtual.sh [--simulator gazebo|mujoco] [--acc [FACTOR]] [session options]
#   --acc [FACTOR]  MuJoCo without the Doosan emulator: a native controller replica in simulation time, FACTOR
#                   times real time (default 4, 0 = as fast as possible); implies --simulator mujoco.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIMULATOR=
ACC=
ARGS=()
while (($#)); do
    case "$1" in
        --simulator)
            if (($# < 2)); then echo '--simulator requires gazebo or mujoco' >&2; exit 2; fi
            SIMULATOR="$2"; shift 2 ;;
        --simulator=*) SIMULATOR="${1#*=}"; shift ;;
        --acc)
            ACC=1; ARGS+=(--acc)
            if (($# > 1)) && [[ "$2" =~ ^[0-9]+([.][0-9]+)?$ ]]; then ARGS+=("$2"); shift; fi
            shift ;;
        --acc=*) ACC=1; ARGS+=(--acc "${1#*=}"); shift ;;
        *) ARGS+=("$1"); shift ;;
    esac
done
if [[ -n "$ACC" ]]; then
    if [[ "$SIMULATOR" == gazebo ]]; then echo '--acc runs on MuJoCo only' >&2; exit 2; fi
    SIMULATOR=mujoco
fi
case "${SIMULATOR:-gazebo}" in
    gazebo) exec "$ROOT/native/run_display.sh" --mode virtual "${ARGS[@]+${ARGS[@]}}" ;;
    mujoco) exec "$ROOT/ws_cobot_pjt/src/cobot_rg2/doosan-robot2/dsr_mujoco/run_native.sh" "${ARGS[@]+${ARGS[@]}}" ;;
    *) echo "Unknown simulator: $SIMULATOR (choose gazebo or mujoco)" >&2; exit 2 ;;
esac
