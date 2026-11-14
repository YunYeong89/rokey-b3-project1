#!/bin/bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${1:-$ROOT/ws_cobot_pjt}"
if [ "$#" -gt 0 ]; then shift; fi
source /opt/ros/jazzy/setup.bash
cd "$WORKSPACE"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DDRCF_VER=2 "$@"
