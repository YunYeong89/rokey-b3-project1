#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LABEL="${1:-rosetta_60}"
SECONDS_TO_RUN="${2:-60}"
CONTAINER=ws_cobot_pjt
docker cp "$ROOT/benchmark.cpp" "$CONTAINER:/tmp/cobot-benchmark.cpp"
docker exec "$CONTAINER" bash -lc '
    g++ -O2 -std=c++17 -DDRCF_VERSION=3 \
        -I/workspace/src/cobot_rg2/doosan-robot2/dsr_common2/include \
        /tmp/cobot-benchmark.cpp \
        /workspace/src/cobot_rg2/doosan-robot2/dsr_common2/lib/jazzy/arm64/libDRFL.a \
        -lPocoNet -lPocoFoundation -lpthread -o /tmp/cobot-benchmark
    mkdir -p /tmp/cobot-results'
docker exec "$CONTAINER" timeout "$((SECONDS_TO_RUN + 40))" /tmp/cobot-benchmark \
    host.docker.internal 12349 "$SECONDS_TO_RUN" "/tmp/cobot-results/$LABEL"
mkdir -p "$ROOT/results"
docker cp "$CONTAINER:/tmp/cobot-results/." "$ROOT/results/"
python3 "$ROOT/summarize.py"
