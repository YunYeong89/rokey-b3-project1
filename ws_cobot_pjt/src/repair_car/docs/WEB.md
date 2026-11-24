# 웹

| 구성 | 위치 | 실행 위치 |
|---|---|---|
| Flask 웹 서버 (:5050) | `repair_car/web/app.py` · `db.py` · `robot_files.py` · `templates/` · `static/` | 호스트 (`run_web.sh`, Flask 가상환경) |
| SQLite DB | `data/web/repair.db` | 호스트 |
| rosbridge (:9090) | `rosbridge_server` | 컨테이너 |
| `web_control` | `repair_car/web/web_control.py` | 컨테이너 |

## 화면과 경로

- **관리자 `/admin`**: 로봇 탭(프로그램 실행·정지·teleop·힘 그래프·결과 파일), 부품 검사, 수리(자동/수동), 재고·회원·차량 관리, 이력
- **고객 `/my`**: 내 차량의 진행 중 수리와 실시간 측정점(`/api/jobs/<id>/cloud`)
- 브라우저 ↔ Flask: HTTP `/api/*` (잡·차량·재고·로봇 결과 파일)
- 브라우저 ↔ 로봇: rosbridge WebSocket으로 `/robot/command` · `/repair/command` · `/robot/teleop` 발행, `/robot/status` · `/repair/status` · `/scan/*` · `/robot/force_graph` 구독, `/scan_part` 액션

## web_control

- 한 번에 하나의 로봇 프로그램을 `ManagedProcess`(별도 프로세스 그룹)로 실행합니다.
- 실행 가능한 모듈과 옵션은 `web_commands.py`의 목록으로 제한합니다 (CLI 인자 화이트리스트).
- 부품 검사는 `web_inspection.py`가 단계(탐침 확인 → 부품별 `real_scan --controller mesh-height`)로 나눠 실행합니다. 실기에서는 `--inspection-position`을 붙입니다.
- 다중 로봇(`/robot_N`)이면 자식 프로세스에 `--ros-args -r __ns:=/robot_N`을 붙이고, 결과는 `data/robot_N/` 아래에 저장됩니다.
- 정지: DSR `move_stop` · `release_force`, 가상이면 `surface_follow/end`, 실행 중인 자식에 SIGINT.
- 실시간 힘 그래프는 `draw_graph --no-png --no-save`로 스트림(`/robot/force_graph`)만 보내고 파일은 남기지 않습니다.

수리 흐름은 [REPAIR.md](REPAIR.md)를 봅니다.
