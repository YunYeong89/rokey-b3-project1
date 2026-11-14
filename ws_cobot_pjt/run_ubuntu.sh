#!/bin/bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/ws_cobot_pjt/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-29}"
cleanup() {
  docker compose -f "$ROOT/compose.yaml" stop emulator
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
docker compose -f "$ROOT/compose.yaml" --profile virtual up -d --wait emulator
ros2 launch m0609_rg2_bringup bringup.launch.py \
  mode:=virtual host:=127.0.0.1 port:=12349 "$@"
