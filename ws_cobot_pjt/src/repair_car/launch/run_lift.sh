#!/bin/bash
# The 90 x 60 scissor lift (ROS package repair_car) in one command: Gazebo as the physics engine only (server, no window),
# RViz as the view (robot_state_publisher of the lift URDF fed with the Gazebo joint states), the unchanged firmware on
# the Gazebo lift (TCP 5005), and in the container of run_virtual.sh (compose service cobot, ws_cobot_pjt) the ROS
# bridge /lift/command, /lift/status. Command lines typed here go through ROS (ros2 run repair_car lift_command).
# Closing RViz, q or Ctrl+C ends everything; the container is stopped only if this script started it.
#   ./run_lift.sh [--load 1kg|vehicle] [--no-gui] [--cmd 'lift up' --cmd 'lift down' ...]
#     --load    payload on the holder (lift config/device.yaml linkage.payloads), default 1kg
#     --no-gui  no RViz
#     --cmd     command lines sent in order through /lift/command, then exit; without it they are asked for here
# Logs: ws_cobot_pjt/src/repair_car/repair_car/lift/out/lift_*.log
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
PACKAGE="$ROOT/ws_cobot_pjt/src/repair_car"
COMPOSE="$PACKAGE/Docker/compose.yaml"
LIFT="$PACKAGE/repair_car/lift"
OUT="$LIFT/out"
CONTAINER=ws_cobot_pjt
ROS='source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash'
PIDFILE=/tmp/lift-bridge.pid
PORT=5005
LOAD=1kg
GUI=1
COMMANDS=()
while (($#)); do
    case "$1" in
        --load) LOAD="${2:?--load requires a payload name}"; shift 2 ;;
        --load=*) LOAD="${1#*=}"; shift ;;
        --no-gui) GUI=; shift ;;
        --cmd) COMMANDS+=("${2:?--cmd requires a command line}"); shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done

set +eu  # conda activation scripts are not nounset / errexit safe
eval "$("${CONDA_EXE:-conda}" shell.bash hook)"
conda activate cobot-native
set -eu
# Native topics apart from run_virtual's session (its own GZ_PARTITION, ROS_DOMAIN_ID 129)
export GZ_PARTITION=scissor_lift GZ_IP=127.0.0.1 ROS_DOMAIN_ID=130
cd "$PACKAGE"
mkdir -p "$OUT"
python -m repair_car.lift.turnlift gzlift > "$OUT/lift_gzlift.log"
python -m repair_car.lift.turnlift table > /dev/null

STARTED=
PIDS=()
alive() { [[ -n "$1" ]] && kill -0 "$1" 2>/dev/null; }
cleanup() {
    trap '' INT TERM HUP   # the watcher below signals again once RViz is gone; finish the cleanup regardless
    echo 'Stopping the lift session...'
    docker exec "$CONTAINER" bash -c "if [ -f $PIDFILE ]; then kill -INT -- -\$(cat $PIDFILE); rm -f $PIDFILE; fi" \
        > /dev/null 2>&1 || true
    for pid in ${PIDS[@]+"${PIDS[@]}"}; do kill -TERM "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    if [[ -n "$STARTED" ]]; then docker compose -f "$COMPOSE" stop cobot > /dev/null 2>&1 || true; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
start() { local log=$1; shift; "$@" > "$OUT/lift_$log.log" 2>&1 & PIDS+=($!); }
send() { docker exec "$CONTAINER" bash -c "$ROS && exec ros2 run repair_car lift_command \"\$@\"" _ "$@"; }

if [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != true ]]; then
    echo "Starting container $CONTAINER (the same compose service as run_virtual.sh)"
    docker compose -f "$COMPOSE" up -d --wait cobot
    STARTED=1
fi
docker exec "$CONTAINER" bash -c \
    'source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --symlink-install --packages-select custom_interfaces repair_car' \
    > "$OUT/lift_build.log" 2>&1 || { echo "Container build failed: $OUT/lift_build.log" >&2; exit 1; }

start gazebo gz sim -s -r "$OUT/worlds/scissor_lift_$LOAD.sdf"
GAZEBO_PID=$!
echo "Gazebo physics engine: scissor_lift_$LOAD"
until gz topic -l 2>/dev/null | grep -qx /scissor_lift/joint_state; do
    alive "$GAZEBO_PID" || { echo "Gazebo exited: $OUT/lift_gazebo.log" >&2; exit 1; }
    sleep 1
done
start firmware python -u -m repair_car.lift.turnlift gzserve --serve "$PORT"
FIRMWARE_PID=$!
until python -m repair_car.lift.firmware --port "$PORT" status > /dev/null 2>&1; do
    alive "$FIRMWARE_PID" || { echo "The firmware exited: $OUT/lift_firmware.log" >&2; exit 1; }
    sleep 0.5
done
docker exec "$CONTAINER" setsid --wait bash -c "echo \$\$ > $PIDFILE; $ROS && exec ros2 run repair_car lift_bridge" \
    > "$OUT/lift_bridge.log" 2>&1 &
PIDS+=($!)
RVIZ_PID=
if [[ -n "$GUI" ]]; then
    start relay python -m repair_car.lift.turnlift gzrelay
    start state "$CONDA_PREFIX/lib/robot_state_publisher/robot_state_publisher" \
        --ros-args --params-file "$OUT/rviz/scissor_lift_$LOAD.yaml" -r __ns:=/lift_native
    start rviz rviz2 -d "$OUT/rviz/lift.rviz"
    RVIZ_PID=$!
fi
send status

if ((${#COMMANDS[@]})); then
    for line in "${COMMANDS[@]}"; do send $line; done
    exit 0
fi
# Closing RViz (or the firmware exiting) ends the session while the prompt below waits for input.
( while alive "$FIRMWARE_PID" && { [[ -z "$GUI" ]] || alive "$RVIZ_PID"; }; do sleep 1; done; kill -TERM $$ ) &
PIDS+=($!)
while read -r -p 'Lift command (lift up | lift down | lift MM | heading DEG | status ..., q = quit): ' LINE; do
    [[ "$LINE" == q ]] && break
    [[ -n "${LINE// }" ]] && { send $LINE || true; }
done
