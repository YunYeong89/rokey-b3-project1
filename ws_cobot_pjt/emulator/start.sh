#!/bin/bash
set -euo pipefail
./DRCF "$ROBOT_MODEL" &
controller_pid=$!
./dart-drfl-api &
api_pid=$!
stop() {
    kill -TERM "$controller_pid" "$api_pid" 2>/dev/null || true
    wait "$controller_pid" "$api_pid" 2>/dev/null || true
}
trap stop EXIT
trap 'exit 0' TERM INT
set +e
wait -n "$controller_pid" "$api_pid"
status=$?
set -e
# Both services must remain alive for the emulator to be usable.
if [ "$status" -eq 0 ]; then status=1; fi
exit "$status"
