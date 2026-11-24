#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'EOF'
Usage: ./run_real.sh ROBOT_IP [--car real|demo_front|demo_left|demo_right] [session options]
       ./run_real.sh ROBOT_IP --with-sim [--car REAL_FIXTURE] [--virtual-car VIRTUAL_FIXTURE]
                     [--web] [--no_gazebo] [--no-gazebo-gui | --no-gui] [--check]
--with-sim starts /robot_1 in RViz and /robot_2 in Gazebo.
--web also uses --with-sim. Both robots default to demo_final in this setup.
With --web, Gazebo runs only its physics server.
--no_gazebo (or --no-gazebo): /robot_1 only, no /robot_2 Gazebo session and no emulator (less CPU for the real robot).
EOF
    exit 0
fi
ROBOT_HOST="${1:?Usage: ./run_real.sh ROBOT_IP [--with-sim] [--car real|demo_front|demo_left|demo_right]}"
shift
for option in "$@"; do
    if [[ "$option" == --with-sim || "$option" == --web ]]; then
        fleet=(--real-host "$ROBOT_HOST" --virtual-count 1 --render --real-car demo_final --car demo_final)
        no_gazebo=''
        while (($#)); do
            case "$1" in
                --with-sim) shift ;;
                --no_gazebo|--no-gazebo) no_gazebo=1; shift ;;
                --virtual-count|--virtual-count=*) echo '--with-sim starts exactly one virtual robot.' >&2; exit 2 ;;
                --virtual-car|--virtual-car=*)
                    if [[ "$1" == --virtual-car ]]; then car="${2:?Missing vehicle}"; shift 2
                    else car="${1#*=}"; shift; fi
                    case "$car" in
                        real|demo|demo_front|demo_left|demo_right|demo_final) fleet+=(--car "$car") ;;
                        *) echo '--virtual-car must be real or a demo fixture.' >&2; exit 2 ;;
                    esac ;;
                --car=*) fleet+=("--real-car=${1#*=}"); shift ;;
                --car|--rt-host|--ros-domain|--probe)
                    name="$1"
                    [[ "$name" != --car ]] || name=--real-car
                    fleet+=("$name" "${2:?Missing value}")
                    shift 2 ;;
                --car-pose|--virtual-car-pose)
                    name=--real-car-pose
                    [[ "$1" != --virtual-car-pose ]] || name=--car-pose
                    fleet+=("$name" "${2:?Missing X}" "${3:?Missing Y}" "${4:?Missing YAW}")
                    shift 4 ;;
                *) fleet+=("$1"); shift ;;
            esac
        done
        [[ -z $no_gazebo ]] || fleet+=(--virtual-count 0)   # the last value wins: no virtual robot, no emulator
        exec "$ROOT/ws_cobot_pjt/src/repair_car/launch/run_service.sh" "${fleet[@]}"
    fi
done
exec "$ROOT/ws_cobot_pjt/src/repair_car/launch/run_display.sh" --mode real --host "$ROBOT_HOST" --car real "$@"
