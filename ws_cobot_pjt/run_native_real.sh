#!/bin/bash
# Real robot + web on Ubuntu without Docker, in one terminal (sodreal + rosbridge + web_control + web).
#   ./run_native_real.sh            robot at 192.168.1.100 (ROBOT_IP=... to change), RViz on
#   ./run_native_real.sh --no-rviz  without RViz
# Ctrl+C stops everything: web_control first (it stops a running robot program and releases force/compliance),
# then rosbridge, the web and the robot driver. Output: one line per message with [robot] [bridge] [control] [web];
# the robot driver shows only errors, warnings and the RT connection (all of it in the log files).
# The driver comes from this repository's workspace (the team's dsr_hardware2: /dsr01/realtime/force_telemetry for
# the scan points), on top of ~/cobot1_ws/ws_cobot_pjt/ws_dsr.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$ROOT/.." && pwd)"
WS="$ROOT/ws_cobot_pjt"
DSR="${DSR_WS:-$HOME/cobot1_ws/ws_cobot_pjt/ws_dsr}"
ROBOT_IP="${ROBOT_IP:-192.168.1.100}"
LOGS="$ROOT/native/.cache/native_real"
GUI=true
for arg in "$@"; do
    case "$arg" in
        --no-rviz) GUI=false ;;
        -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
        *) echo "알 수 없는 옵션: $arg (--no-rviz, --help)" >&2; exit 2 ;;
    esac
done

fail() { echo "실행 안 함: $*" >&2; exit 1; }
[[ -f "$DSR/install/setup.bash" ]] || fail "Doosan 워크스페이스가 없습니다: $DSR (DSR_WS=... 로 지정)"
[[ -f "$WS/install/setup.bash" ]] || fail "프로젝트 워크스페이스가 빌드되지 않았습니다: $WS/install"
ss -ltn | grep -q ':5050 ' && fail "포트 5050 사용 중: 이미 웹(app.py)이 떠 있습니다. 먼저 끄세요"
ss -ltn | grep -q ':9090 ' && fail "포트 9090 사용 중: 이미 rosbridge 가 떠 있습니다. 먼저 끄세요"
pgrep -f 'ros2_control_node.*__ns:=/dsr01' > /dev/null && fail "로봇 드라이버(sodreal)가 이미 실행 중입니다. 먼저 끄세요"
pgrep -f 'rokey[.]web_control|rokey/web_control' > /dev/null && fail "web_control 이 이미 실행 중입니다. 먼저 끄세요"
ping -c1 -W1 "$ROBOT_IP" > /dev/null 2>&1 || fail "로봇($ROBOT_IP)에 ping 응답이 없습니다. 랜선·IP를 확인하세요"

# ~/.bashrc does this for interactive shells only
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-70}" RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
source /opt/ros/jazzy/setup.bash
source "$DSR/install/setup.bash"
source "$WS/install/setup.bash"   # last: the team's driver, bringup, dsr_msgs2 and rokey

mkdir -p "$LOGS"
PIDS=()   # process groups, in start order
start() {  # start NAME FILTER COMMAND...: own process group; output to $LOGS/NAME.log and, through FILTER, here
    local name=$1 filter=$2; shift 2
    : > "$LOGS/$name.log"
    # env --default-signal: a script's background job starts with SIGINT ignored, and the programs (web_control's stop
    # and release, ros2 launch) would never see the stop.
    setsid env --default-signal=INT bash -c '
        name=$1 filter=$2 log=$3; shift 3
        # The filters ignore the stop SIGINT and read on to the end: the program keeps printing while it stops and
        # releases, and a closed pipe would kill it with SIGPIPE half-way.
        "$@" 2>&1 | (trap "" INT; tee -a "$log" | grep --line-buffered -E "$filter" | sed -u "s/^/[$name] /")' \
        _ "$name" "$filter" "$LOGS/$name.log" "$@" &
    PIDS+=("$name:$!")
}

stop_group() {  # stop_group NAME SECONDS: SIGINT to its group, SIGKILL after SECONDS
    local entry pid
    for entry in "${PIDS[@]}"; do
        [[ ${entry%%:*} == "$1" ]] || continue
        pid=${entry#*:}
        kill -0 "$pid" 2>/dev/null || return 0
        kill -INT -- "-$pid" 2>/dev/null || true
        for ((i = 0; i < $2 * 10; i++)); do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.1; done
        echo "[$1] ${2}초 안에 끝나지 않아 강제 종료"
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
}

STOPPING=
shutdown() {
    [[ -n $STOPPING ]] && return
    STOPPING=1
    trap '' INT TERM
    echo; echo "종료 중: web_control → rosbridge → 웹 → 로봇 드라이버"
    stop_group control 15   # its robot program stops and releases first
    stop_group bridge 5
    stop_group web 5
    stop_group robot 15
    echo "모두 종료했습니다. 로그: $LOGS"
}
trap shutdown EXIT
trap 'exit 130' INT TERM

echo "로봇 $ROBOT_IP · 로그 $LOGS"
start robot 'ERROR|Error|WARN|Unable|Connected RT|rt_host' \
    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:="$ROBOT_IP" port:=12345 model:=m0609 gui:=$GUI
start bridge '.' ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090
start control '.' ros2 run rokey web_control
start web '.' bash -c "cd '$REPO/web' && exec python3 -u app.py"

echo "웹: http://localhost:5050/admin?tab=robot   (멈추려면 Ctrl+C 한 번)"
echo "확인: ros2 topic hz /dsr01/realtime/force_telemetry  → 약 100 Hz"
# Until Ctrl+C, or until one of the four ends by itself (then everything stops)
while true; do
    for entry in "${PIDS[@]}"; do
        if ! kill -0 "${entry#*:}" 2>/dev/null; then
            echo "[${entry%%:*}] 종료됨 → 전체 종료 (로그: $LOGS/${entry%%:*}.log)"
            exit 1
        fi
    done
    sleep 1
done
