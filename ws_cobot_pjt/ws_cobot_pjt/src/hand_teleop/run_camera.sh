#!/bin/bash
# macOS host: webcam hand tracking for `ros2 run hand_teleop teleop` (Docker), served on 127.0.0.1:5070.
#   conda env create -f environment.yml      (once)
#   ./run_camera.sh [--camera 0] [--port 5070] [--seconds 5]
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL="${HAND_MODEL:-$HOME/.cache/hand_teleop/hand_landmarker.task}"
if [[ ! -f "$MODEL" ]]; then
    mkdir -p "$(dirname "$MODEL")"
    curl -fL -o "$MODEL" \
        https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
fi
set +eu  # conda activation scripts are not nounset / errexit safe
eval "$("${CONDA_EXE:-conda}" shell.bash hook)"
conda activate hand-teleop
set -eu
cd "$ROOT"
exec python -m hand_teleop.vision --model "$MODEL" "$@"
