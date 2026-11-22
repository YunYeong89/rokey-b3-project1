#!/bin/bash
# Web robot control attached to the robot that is already running: start the session as usual (./run_virtual.sh ...
# or ./run_real.sh ROBOT_IP) and this script only adds the web; it never starts, stops or locks a robot session.
# Flask (:5050) runs here, rosbridge (:9090) and web_control in the ws_cobot_pjt container; ones another program
# started (a session with --web) are reused and left alone.
#   ./run_web.sh start [--lan]   runs until Ctrl+C (or ./run_web.sh stop), then shuts the web down; the robot runs on
#                                --lan: other devices open https://<this PC's IP>:5443 (or http on :5050); install
#                                the local CA once from http://<IP>:5050/ca.crt (rosbridge has no login: lab LAN only)
#   ./run_web.sh stop | status | log
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE="$ROOT/native/.cache"
# web: 작업 공간 안에 저장소를 clone 한 구성(ws_cobot_pjt/rokey-b3-project1/web) 또는 이 저장소 안의 ws_cobot_pjt (../web)
WEB="$ROOT/rokey-b3-project1/web"; [[ -d "$WEB" ]] || WEB="$(cd "$ROOT/.." && pwd)/web"
LOG="$STATE/web.log"
CONTAINER=ws_cobot_pjt
PORT=5050
RELAY=9091        # --lan: rosbridge for http pages
HTTPS=5443        # --lan: the web with TLS
WSS=9443          # --lan: rosbridge with TLS for https pages
TLS="$STATE/web_tls"
LABELS="control bridge"  # container processes started here, web_control first on stop (it stops its program)

alive() { [[ -f "$STATE/web_$1.pid" ]] && kill -0 "$(cat "$STATE/web_$1.pid")" 2>/dev/null; }
pattern() { if [[ $1 == control ]]; then echo 'rokey[.]web_control'; else echo 'rosbridge'; fi; }
owned() {  # our <label> process is still running: its pid file outlives a container restart, the pid may be reused
    docker exec "$CONTAINER" sh -c "p=\$(cat /tmp/rokey-web-$1.pid 2>/dev/null) && ps -o args= -p \"\$p\" | grep -q '$(pattern "$1")'" 2>/dev/null
}
found() { docker exec "$CONTAINER" pgrep -f "$1" > /dev/null 2>&1; }
running() { [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" == true ]]; }

lan_ip() {  # the address of the interface of the default route (Wi-Fi or a wired adapter)
    ipconfig getifaddr "$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')" 2>/dev/null ||
        hostname -I 2>/dev/null | awk '{print $1}'
}

urls() {
    echo "웹: http://localhost:$PORT/admin?tab=robot"
    if alive relay; then
        local ip; ip=$(lan_ip)
        echo "다른 기기 (https): https://$ip:$HTTPS/admin?tab=robot"
        echo "  처음 한 번: Safari로 http://$ip:$PORT/ca.mobileconfig 열기 (또는 $TLS/rokey-web-ca.mobileconfig AirDrop)"
        echo "             → 설정 > 다운로드된 프로파일 > 설치"
        echo "             → 설정 > 일반 > 정보 > 인증서 신뢰 설정에서 'rokey web local CA' 켜기"
        echo "다른 기기 (http): http://$ip:$PORT/admin?tab=robot"
    fi
}

tls() {  # the local CA (installed once on each device) and a server certificate for this PC's current IP
    local ip; ip=$(lan_ip)
    mkdir -p "$TLS"
    if [[ ! -f "$TLS/ca.crt" ]]; then
        printf 'basicConstraints=critical,CA:TRUE\nkeyUsage=critical,keyCertSign,cRLSign\n' > "$TLS/ca.ext"
        openssl req -new -newkey rsa:2048 -nodes -keyout "$TLS/ca.key" -out "$TLS/ca.csr" \
            -subj "/CN=rokey web local CA" 2>> "$LOG"
        openssl x509 -req -in "$TLS/ca.csr" -signkey "$TLS/ca.key" -days 3650 -sha256 -extfile "$TLS/ca.ext" \
            -out "$TLS/ca.crt" 2>> "$LOG"
    fi
    [[ -f "$TLS/rokey-web-ca.mobileconfig" ]] || (cd "$WEB" && .venv/bin/python -c "from app import ca_profile
open('$TLS/rokey-web-ca.mobileconfig', 'wb').write(ca_profile('$TLS/ca.crt'))") >> "$LOG" 2>&1
    if ! openssl x509 -in "$TLS/server.crt" -noout -text 2>/dev/null | grep -q "IP Address:$ip,"; then
        # iOS: SAN, serverAuth, RSA 2048, SHA-256, at most 825 days; a new IP gets a new certificate, same CA
        printf 'subjectAltName=IP:%s,IP:127.0.0.1,DNS:localhost\nextendedKeyUsage=serverAuth\n' "$ip" > "$TLS/server.ext"
        printf 'basicConstraints=CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\n' >> "$TLS/server.ext"
        openssl req -new -newkey rsa:2048 -nodes -keyout "$TLS/server.key" -out "$TLS/server.csr" \
            -subj "/CN=$ip" 2>> "$LOG"
        openssl x509 -req -in "$TLS/server.csr" -CA "$TLS/ca.crt" -CAkey "$TLS/ca.key" -CAcreateserial -days 825 \
            -sha256 -extfile "$TLS/server.ext" -out "$TLS/server.crt" 2>> "$LOG"
    fi
}

container_start() {  # label command: its own process group in the container, pid in /tmp/rokey-web-<label>.pid
    docker exec -d -e REPAIR_DB_URL="http://host.docker.internal:$PORT" \
        -e REPAIR_API_KEY="${REPAIR_API_KEY:-rokey-robot-key}" "$CONTAINER" setsid bash -lc \
        "echo \$\$ > /tmp/rokey-web-$1.pid; source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
         exec $2 > /tmp/rokey-web-$1.log 2>&1"
}

robot_model() {  # the running session's robot model (calibrated TCP) for the 3D view; a browser reload shows it
    if [[ -f "$STATE/robot.urdf" ]]; then
        "$WEB/.venv/bin/python" "$WEB/robot_model.py" --urdf "$STATE/robot.urdf" \
            --source-root "$ROOT/ws_cobot_pjt/src" --tcp-frame calibrated_tcp >> "$LOG" 2>&1
    fi
}

attach() {  # rosbridge and web_control in the container when missing: a robot session restart restarts the container
    ATTACHED=''
    if [[ "$(docker exec "$CONTAINER" printenv COBOT_SERVICE_MODE 2>/dev/null)" == 1 ]]; then return; fi
    if ! found rosbridge_websocket; then
        container_start bridge 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090'
        ATTACHED+=' rosbridge'
    fi
    if ! found 'rokey[.]web_control'; then container_start control 'python3 -u -m rokey.web_control'; ATTACHED+=' web_control'; fi
    if [[ -n $ATTACHED ]]; then robot_model; fi
}

native_start() {  # name command...: in the background, pid in native/.cache/web_<name>.pid
    local name=$1
    shift
    nohup "$@" >> "$LOG" 2>&1 < /dev/null &
    echo $! > "$STATE/web_$name.pid"
    MINE="$MINE $name"
}

start() {
    if ! running; then
        echo "$CONTAINER 컨테이너가 실행 중이 아닙니다. 먼저 로봇 세션을 시작하세요 (./run_virtual.sh ... / ./run_real.sh IP)" >&2
        exit 1
    fi
    if [[ ! -x "$WEB/.venv/bin/python" ]]; then
        echo '웹 서버 의존성 설치 (최초 1회)'
        python3 -m venv "$WEB/.venv"
        "$WEB/.venv/bin/pip" install -q -r "$WEB/requirements.txt"
    fi
    mkdir -p "$STATE"
    stop > /dev/null  # whatever an earlier start of this script left running: started again with these settings
    robot_model
    attach
    echo "컨테이너: 시작함 =${ATTACHED:- 없음 (rosbridge·web_control 실행 중인 것을 사용)}"
    MINE=''
    local bridge_port=9090
    if [[ ${1:-} == --lan ]]; then
        tls
        native_start relay "$WEB/.venv/bin/python" -u "$WEB/lan_relay.py" --route "$RELAY:9090" \
            --route "$HTTPS:$PORT:tls" --route "$WSS:9090:tls" --cert "$TLS/server.crt" --key "$TLS/server.key"
        bridge_port=$RELAY
    fi
    if curl -fs -o /dev/null "http://127.0.0.1:$PORT/login"; then
        echo "웹 서버: 다른 프로그램이 연 것을 사용 (:$PORT, --lan 설정은 적용되지 않을 수 있음)"
    else
        PORT=$PORT ROSBRIDGE_PORT=$bridge_port ROSBRIDGE_TLS_PORT=$WSS WEB_CA="$TLS/ca.crt" \
            ROKEY_DIR="$ROOT/ws_cobot_pjt/src/rokey" SESSION_DIR="$ROOT/ws_cobot_pjt/session" \
            SCENE_DIRS="$STATE/scene:$ROOT/ws_cobot_pjt/src/rokey/meshes" \
            native_start flask "$WEB/.venv/bin/python" -u "$WEB/app.py"
        for _ in $(seq 30); do
            curl -fs -o /dev/null "http://127.0.0.1:$PORT/login" && break
            alive flask || { echo '웹 서버가 시작 중에 종료되었습니다:' >&2; tail -n 20 "$LOG" >&2; stop; exit 1; }
            sleep 1
        done
        echo "웹 서버: 시작 (:$PORT)"
    fi
    urls
    if [[ "$(uname -s)" == Darwin ]]; then open "http://localhost:$PORT/admin?tab=robot"; fi
    trap 'echo; stop; exit 0' INT TERM HUP
    echo 'Ctrl+C: 웹 종료 (로봇 세션은 계속 실행됩니다). 로봇 세션을 다시 켜면 자동으로 다시 연결합니다. 로그: ./run_web.sh log'
    local down=''
    while sleep 3; do
        for name in $MINE; do
            if ! alive "$name"; then
                echo "$name 이(가) 종료되었습니다. 마지막 로그:" >&2
                tail -n 10 "$LOG" >&2
                stop
                exit 1
            fi
        done
        if ! running; then
            [[ -n $down ]] || echo "$(date +%H:%M:%S) 로봇 세션 컨테이너가 꺼졌습니다. 다시 켜지면 자동으로 연결합니다"
            down=1
            continue
        fi
        attach
        if [[ -n $ATTACHED ]]; then
            echo "$(date +%H:%M:%S) 다시 연결:$ATTACHED 시작 (3D 로봇 모델은 브라우저를 새로고침하면 새 세션 기준)"
        elif [[ -n $down ]]; then
            echo "$(date +%H:%M:%S) 로봇 세션 컨테이너가 다시 켜졌습니다 (rosbridge·web_control 실행 중인 것을 사용)"
        fi
        down=''
    done
}

stop() {
    for label in $LABELS; do
        if owned "$label"; then
            echo "$label 종료 중"
            docker exec "$CONTAINER" bash -c "p=\$(cat /tmp/rokey-web-$label.pid); kill -INT -- -\$p 2>/dev/null
                for i in \$(seq 40); do kill -0 \$p 2>/dev/null || break; sleep .5; done
                kill -KILL -- -\$p 2>/dev/null" || true
        fi
        docker exec "$CONTAINER" rm -f "/tmp/rokey-web-$label.pid" 2>/dev/null || true
    done
    for name in flask relay; do
        if alive "$name"; then kill -TERM "$(cat "$STATE/web_$name.pid")"; echo "$name 종료"; fi
        rm -f "$STATE/web_$name.pid"
    done
    echo '웹을 종료했습니다 (로봇 세션은 그대로 실행 중)'
}

status() {
    for name in flask relay; do alive "$name" && echo "$name: 실행 중 (PID $(cat "$STATE/web_$name.pid"))"; done
    for label in $LABELS; do owned "$label" && echo "$label: 컨테이너에서 실행 중 (run_web.sh가 시작)"; done
    found 'rokey[.]web_control' && echo 'web_control: 응답 가능' || echo 'web_control: 없음'
    urls
}

case "${1:-}" in
    start) start "${2:-}" ;;
    stop) stop ;;
    status) status ;;
    log) tail -n 40 "$LOG"
         for label in $LABELS; do echo "== container $label"; docker exec "$CONTAINER" tail -n 20 "/tmp/rokey-web-$label.log" 2>/dev/null || true; done ;;
    *) sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
