# 자동차 자동 수리 로봇 — 웹 + DB

Flask + SQLite3 + HTML/JS(roslibjs, three.js). 로봇 없이도 **데모 모드**로 전체 흐름을 확인할 수 있다.

```
repair_web/
├── app.py              웹 서버 + REST API + 권한 검사
├── db.py               SQLite 연결, 초기 데이터 (부품 목록 PARTS, 샘플 데이터 insert_sample)
├── schema.sql          테이블 7개 (ERD 참고)
├── templates/
│   ├── login.html      SCR-001 로그인
│   ├── admin.html      SCR-101~104 관리자 (수리 / 재고 / 이력 / 회원·차량)
│   └── my.html         SCR-201~202 고객 (내 차량 3D 수리 현황 / 내 이력)
├── static/
│   ├── common.js       공통 함수 (API 호출, 단계 표시, 상세 표)
│   ├── admin.js        관리자 화면 + 데모 시뮬레이터 + rosbridge
│   ├── my.js           고객 화면 + three.js 3D 뷰어
│   ├── viewer3d.js     3D 차량 뷰어 (관리자·고객 공용)
│   ├── robot3d.js      3D 로봇팔 M0609 (관리자만, 설정 상수는 파일 맨 위)
│   ├── robot_ik.js     로봇팔 FK + 수치 IK (DLS)
│   ├── style.css
│   ├── models/car.glb  (선택) 실제 3D 모델. 없으면 임시 상자 모델
│   └── models/m0609/   두산 M0609 white 메쉬 + m0609.json (ros 브랜치에서 복사)
└── ros/
    ├── repair_db_client.py     ROS 노드에서 DB 쓰는 클라이언트
    └── process_manager_stub.py 웹↔ROS 연결 확인용 뼈대 노드 (자동/수동)
```

## 실행 (노트북 하나로, 로봇 없이)

```bash
pip install flask requests          # Ubuntu 24.04 는 --break-system-packages 또는 venv
cd repair_web
rm -f repair.db                     # 예전 버전 DB 가 있으면 지우기 (스키마 바뀜, 자동 변환 없음)
python3 app.py
```

- **처음 실행하면 샘플 데이터가 자동으로 들어감**: 회원 36명 · 차량 40대 · 지난 30일 수리 이력 100건 · 부품 17종 재고 (발표·테스트용, seed 고정)
- **처음 상태로 되돌리기**: `rm repair.db` 후 다시 `python3 app.py`
- **샘플 없이 실행** (실제 로봇 시연용, 기본 계정·부품·차량 2대만): `rm repair.db` 후 `REPAIR_SAMPLE=0 python3 app.py`

- http://localhost:5000 → 로그인하면 권한에 따라 관리자 / 고객 화면으로 이동
- 태블릿: 같은 와이파이에서 http://<PC IP>:5000 (`hostname -I`)
- three.js, roslibjs 를 CDN 에서 받으므로 인터넷 연결 필요

| 계정 | 비밀번호 | 권한 | 차량 |
|---|---|---|---|
| admin | admin1234 | 관리자 | 전체 |
| user1 | user1234 | 사용자(차주) | 모형 A-01 |
| user2 | user1234 | 사용자(차주) | 모형 A-02 |
| admin2 | admin1234 | 관리자 | 전체 (샘플) |
| user3 ~ user34 | user1234 | 사용자(차주) | 샘플 차량 A-03 ~ A-40 (user3 은 2대) |

→ 발표 전에 비밀번호 바꾸기 (회원·차량 탭에서 새 계정 만들기 가능)

## 데모 순서 (발표용)

1. 브라우저 두 개: 하나는 **admin**, 하나는 **user1** (시크릿 창)
2. admin: 상단 **데모 모드** 체크 → 차량 `모형 A-01` 선택
3. **자동 모드** → 수리 시작: 표면 스캔(연속 접촉) → 의심 구역 상세 → 판정 → 재고 → 교체 → 재검사
4. **수동 모드** → 교체할 부품 체크(또는 3D 화면에서 부품 클릭) → 수리 시작: 스캔·판정 건너뜀
   (목록에는 재고 탭에서 **로봇 교체**를 켠 부품만 나옴. 기본: 보닛 · 미러 2 · 문 4)
5. user1 화면: 3D 모델에서 교체 예정(노랑) → 교체 완료(초록)로 바뀌는 모습, 진행 단계, 단차 결과
6. 재고 탭에서 보닛을 0 으로 → 수리하면 **수리 보류** (샘플 데이터는 사이드미러(우)가 처음부터 0)

## 로봇과 연결

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml    # 터미널 2
python3 ros/process_manager_stub.py                            # 터미널 3 (또는 ros2 run car_repair process_manager)
```

### 토픽 약속

| 토픽 | 방향 | 내용 (std_msgs/String, JSON) |
|---|---|---|
| `/repair/command` | 웹 → 로봇 | `{"cmd":"start","job_id":3,"vehicle_id":1,"mode":"manual","parts":["bonnet"]}` / `{"cmd":"stop","job_id":3}` |
| `/repair/status` | 로봇 → 웹 | `{"type":"point","x":120,"y":40,"dz":-0.8,"state":"dent"}` · `{"type":"wrench","fz":-4.2}` · `{"type":"flush","mm":0.18}` · `{"type":"robot","state":"idle\|moving\|error","message":"..."}` (선택, 오면 상단 로봇 상태에 우선 표시) |

단계·판정·결과는 토픽이 아니라 **DB(API)** 로 기록한다 (`RepairDB`). 화면이 0.7~1.5초마다 읽어 표시.
수동 모드에서 지정한 부품은 `findings` 에 `decision='requested'` 로 들어가므로 로봇은 `db.requested_parts(job_id)` 로 읽으면 된다.

## REST API

브라우저는 로그인 세션, 로봇 노드는 헤더 `X-API-Key` (환경변수 `REPAIR_API_KEY`, 기본 `rokey-robot-key`).

| 메서드 | 경로 | 권한 | 설명 |
|---|---|---|---|
| GET | `/api/me` | 로그인 | 내 정보 |
| GET/POST | `/api/users` | 관리자 | 회원 목록 / 등록 `{username, password, name, role}` |
| GET | `/api/vehicles` | 로그인 | 관리자는 전체, 사용자는 본인 차량만 |
| POST | `/api/vehicles` | 관리자 | 차량 등록 `{name, model, owner_id}` |
| PATCH | `/api/vehicles/<id>` | 관리자 | 차주 변경 `{owner_id}` |
| GET | `/api/parts` | 관리자·로봇 | 재고 목록 |
| GET | `/api/parts/<code>/check` | 관리자·로봇 | `{stock, available, robot_ok}` |
| POST | `/api/parts/<code>/consume` | 관리자·로봇 | 1개 차감, 재고 0이면 409 |
| PATCH | `/api/parts/<code>` | 관리자 | `{stock}` 와/또는 `{robot_ok: 0/1}` (로봇 교체 가능 여부) |
| POST | `/api/jobs` | 관리자·로봇 | 수리 시작 `{vehicle_id, mode, parts}` (진행 중이면 409, 수동 모드에 robot_ok=0 부품이면 400) |
| GET | `/api/jobs` `?vehicle_id=` | 로그인 | 목록 (사용자는 본인 차량만) |
| GET | `/api/jobs/current` `?vehicle_id=` | 로그인 | 진행 중 작업 |
| GET | `/api/jobs/<id>` | 로그인 | 상세 (판정·교체·로그·requested). 남의 차는 404 |
| POST | `/api/jobs/<id>/events` | 관리자·로봇 | `{stage, level, message}` |
| POST | `/api/jobs/<id>/findings` | 관리자·로봇 | `[{part_code, x, y, z, depth, decision}]` (z 는 문처럼 옆면 부품만) |
| POST | `/api/jobs/<id>/replacements` | 관리자·로봇 | `{part_code, success, retries, peak_force, flush_mm}` |
| GET | `/api/alarms` `?limit=100` | 관리자 | warn/error 이벤트 최신순 (job_id, vehicle_name 포함) |
| POST | `/api/jobs/<id>/finish` | 관리자·로봇 | `{status: done/no_damage/on_hold/failed/stopped, summary}` |

## 실제 3D 모델 넣기

`static/models/car.glb` 를 넣으면 고객 화면이 자동으로 그 모델을 쓴다. 모델링 규칙:
- 단위 **mm**, 위쪽 **+Y** (glTF 기본)
- 원점 = 차량 좌표계 원점 (차 뒤쪽 왼쪽 모서리 바닥), **+X = 앞쪽, +Z = 오른쪽**
- 교체 부품은 따로 분리된 메쉬, 이름 **bonnet / mirror_l / mirror_r / door_fl / door_fr / door_rl / door_rr**

이렇게 하면 로봇이 찍은 손상 좌표 (x, y) 가 3D 모델의 (X, Z) 위치에 그대로 표시된다. 높이 z 가 있으면 (문) 3D 높이 Y=z 에 표시.
모형 크기가 다르면 `viewer3d.js` 의 `CAR`·임시 모델, `admin.js` 의 `DOOR_AREA`(데모), `db.py` 의 `BONNET_*`·`DOOR_*`(샘플) 도 같이 수정.

## 주의

- 화면의 정지 버튼은 소프트웨어 정지. 비상 시에는 **티치 펜던트 비상정지**.
- `REPAIR_SECRET`, `REPAIR_API_KEY`, 기본 비밀번호는 바꿔서 사용.
- Flask 내장 서버이므로 수업 실습망 안에서만 사용.
