# 작업 기록 (ws_cobot_pjt)

날짜별 작업 흐름입니다. 각 항목의 근거는 파일 수정 시각, `native/results/`, `verification.json`, `native/.cache/*.log`, 팀 저장소(`rokey-b3-project1`) 커밋입니다. 근거로 확인한 범위만 적었습니다.

---

## 2026-11-12 (목) — 개발 환경 구축

**목표**: macOS(Apple Silicon)에서 Docker ROS 2 + Doosan 에뮬레이터 + 네이티브 Gazebo/RViz 환경 구성

- **워크스페이스 구성** (13:42–13:52)
  - `cobot_rg2`(doosan-robot2, onrobot-ros2, m0609_rg2_bringup)를 배치했습니다.
  - `Dockerfile`, `docker-entrypoint.sh`, `compose.yaml`로 ROS Jazzy 컨테이너를 구성했습니다.
- **에뮬레이터 검증** (15:50–16:14, `emulator/`)
  - `doosanrobot/dsr_emulator:3.0.1`(GF03020000)을 Docker Desktop의 Rosetta로 실행합니다.
  - CPU 분리: 에뮬레이터 0–3번, ROS 4–9번 코어.
  - amd64, box64, Rosetta 벤치마크를 비교했습니다.
  - 결과(`emulator/verification.json`): 2° 왕복 60초 동안 조회 실패 0회, `dsr_controller2` 활성화 확인. 한계: RT UDP는 테스트하지 않음.
- **실행 스크립트** (16:20–19:07)
  - `run_virtual.sh`, `run_real.sh`, `run_rviz.sh` → `native/run_display.sh`(conda `cobot-native`) → `native/session.py`
  - `native/gazebo.config` 추가
- **네이티브 표시 연동** (21:57–22:05)
  - `joint_interpolator.py`, `ros_state_relay.py`로 컨테이너의 관절 상태를 네이티브 Gazebo와 RViz에 중계합니다.
  - `rokey`의 `move.py`, `simple_input.py`
  - 결과: `results/simple_move_*.log`, `joint_topology.log`, `state_timing.json`(서비스 콜백 평균 9.1 ms)

---

## 2026-11-13 (금) — 실기 연결 점검, 가상 힘제어, 탐침 픽업 및 첫 스캔

- **실기 네트워크 점검** (11:17–11:19)
  - `robot_network_shim.c`, `robot_connection_probe.cpp`, `robot_network.py`로 Docker Desktop을 거쳐 DRFL에 연결합니다.
  - 결과(`.cache/real_connection.json`): 컨트롤러 GF02120100, RT 텔레메트리 100회 수신, 모션 명령은 보내지 않음.
- **그리퍼와 픽앤플레이스** (11:31–11:41): `pick_and_place.py`, `gripper_virtual_node.py`
- **에뮬레이터 힘제어 한계 확인** (12:04, `results/force_support.json`)
  - `task_compliance_ctrl`은 성공했지만 `set_stiffnessx`와 `set_desired_force`는 응답이 없었습니다.
  - 그래서 힘제어를 네이티브 가상 백엔드로 대체했습니다.
- **네이티브 가상 힘제어 백엔드** (12:18–12:54)
  - `virtual_force.py`: Gazebo F/T 센서(200 Hz) + 100 Hz 작업공간 어드미턴스
  - `cobot_virtual` 브리지: Doosan 힘 서비스를 가로채 Gazebo로 연결
  - `kinematics.py`
  - 결과(`results/virtual_force_system.json`): 5 N 접촉, 15 N 상대 접촉, 2 N 하중 순응 테스트 통과
- **실기 프로파일 반영** (14:03–16:48)
  - `robot_profile_probe.cpp`, `virtual_profile_loader.py`, `virtual_robot.yaml`, `rokey/calibration.json`, `inspect_real.py`
  - 실기 설정: TCP `GripperDA_v1`(240 mm), Tool Weight 1.28 kg
  - 무부하 외력 추정값이 약 1–2 N(`real_profiles.log`)
- **탐침 맵과 픽업** (23:14–23:57)
  - `maps/donut.json`, `scene_map.py`, `probe_tool.py`, `probe_grasp.py`, `model_export.py`, `calibration.py`
  - `pick_probe_and_scan.py`: 랙에서 탐침을 집는 7단계 → `make_plt_prac.py` / `surface_following.py` 스캔
  - 결과(`results/probe_pickup_validation.json`): 픽업 성공, BASE +X 20 mm 스캔에서 **14점**
- **팀 저장소** (23:14–23:30): `rokey-b3-project1` 초기 구조, 웹/DB 1차(Flask), README

---

## 2026-11-14 (토) — 도넛 스캔 재설계, Tugal 방법론 적용, 접촉 검출 캘리브레이션

### 오전
- 08:15: 가상 픽업 실행 기록이 있습니다(3/7단계에서 로그 종료).
- 09:45 팀 저장소: `docs/INTERFACE.md` 웹↔로봇 인터페이스 약속서 v0.1

### 기존 스캔 로직 리뷰
14점밖에 안 나온 원인을 분석했습니다.
- **시작점과 범위 문제**: home에서 바로 20 mm 직선 한 줄만 스캔합니다. 이 직선은 링 중심선에 접하는 선이라 높이 변화가 약 0.4 mm뿐입니다.
- **법선 추종 꺼짐**: `orientation_control=False`인데 점 저장 조건이 "정렬 오차 2° 이하"라서 곡면 점이 버려졌습니다.
- **`alter_motion` 문제**: 매뉴얼상 DRL user thread에서만 동작하는데, 외부 ROS 서비스로 호출하고 있었습니다.
- **기타**: 2D 법선 추정, 구형 팁 반경 미보정, 실기와 가상의 힘 부호 불일치.
- **가상 모드 위치 오류**: `get_current_posx`가 에뮬레이터의 명령 위치를 반환해서, 가상 pointcloud가 실제 표면이 아니라 명령한 직선을 기록하고 있었습니다.

### 1mm 스텝 + Y 래스터 도넛 스캔 구현
- **신규 모듈**: `rokey/scan_geometry.py`, `contact_probe.py`, `scan_output.py`, `donut_scan.py`
- **라인 순서**: 도넛 중심선 y=cy부터 바깥으로 진행하고, 구멍이나 바깥 구간은 들어 올려 건너뜁니다.
- **가상 백엔드**: 힘 기준 프레임을 다시 잡을 때 목표가 튀지 않도록(bumpless) 수정했습니다.
- **브리지**: `get_current_posx` = 에뮬레이터 명령 위치 + Gazebo 실제 편차(`bringup.launch.py` 리맵 추가).
- **기타**: `probe_tool.py`에 `probe_radius_mm` 추가, `pick_probe_and_scan.py`가 픽업 후 `donut_scan.py`를 실행.

### 컨테이너 수동 실행 구조
- 세션이 플랜을 `ws_cobot_pjt/session/probe_plan.json`(컨테이너 `/workspace/session/`)에 직접 씁니다. 세션 종료 시 삭제됩니다.
- 스캔 옵션은 `donut_scan.py` 명령행으로 옮겼습니다(`--lines`, `--line-spacing`, `--step-mm` 등).
- 실기 설정 `src/rokey/config/donut_scan_real.json`: 중심값이 `MEASURE_ME`인 동안은 움직이기 전에 멈춥니다.
- 맵 없이 세션을 띄우면 `--map donut`이 필요하다고 안내합니다.

### 선행연구 조사
"힘/촉각으로 표면 pointcloud 얻기"를 조사했습니다.
- 연속 추종: Tugal et al.(UKAEA), Nasiri & Wang
- 한 점씩 찍기: VISHAC/Act-VH, CMM 볼 반경 보정
- 힘으로 접촉점 계산: Bicchi, KTH contact_point_estimation
- 촉각 센서: Lepora 촉각 서보, Meta 3D Vision&Touch, NeuralFeels

### Tugal et al. 적용
- **적용 가능성 점검**
  - 에뮬레이터는 RT를 지원하지 않습니다(`dsr_hw_interface2`).
  - `speedl_stream`이 에뮬레이터에서 동작하는지, 실기 힘제어 중에 허용되는지는 미확인입니다.
  - 그래서 "내부 힘 루프는 컨트롤러, 외부 기하 루프는 ROS 노드(30–50 Hz)"로 나누는 구조를 결정했습니다.
- **버그 수정**: `ContactProbe.move`가 `amovel` 직후 `check_motion`을 바로 확인하던 문제(50 ms 대기 추가). 탐침이 표면에 닿지 않은 원인 후보입니다.
- **신규**
  - `surface_normal.py`(`NormalEstimator`): 마찰 보정 반응형 + 2차 곡면(공간 창) + GPR look-ahead
  - `followers.py`: `StepFollower`(Tugal-lite), `StreamFollower`(`speedl_stream`으로 연속 추종)
  - `robot_services.speedl_stream()`, `interface_check.py`(`rate` / `free` / `contact`)
  - `donut_scan.py --mode step|stream`
- **오프라인 검증**(해석적 토러스): 기록되는 법선 오차 평균 1.7–2.6°, 마찰계수 추정 0.30(실제 0.30).

### 접촉 검출 캘리브레이션
- **증상**: 노이즈 때문에 실제로 닿지 않았는데 접촉으로 판정됩니다.
- **대응**
  - 탐색 직전 영점 조정(tare)
  - 3번 연속 임계값 초과 시에만 접촉 판정(디바운스)
  - 누르는 힘에 바이어스 보정 적용
- **측정 도구**: `interface_check.py noise`로 자유공간 노이즈를 측정해 `--touch-threshold`와 `--search-speed` 권장값을 얻습니다.

### 임계값 최적화 계획 (구현 전)
- **공통**: RT 신호 로깅 → 자유공간·접촉 데이터셋 → 오프라인 재생 최적화(허위 경보 0 조건에서 검출 지연 최소화, Optuna)
- **학습 없이**: 자세별 정적 바이어스 모델, 속도·가속도 의존 임계값, CUSUM, 위치 막힘을 보조 단서로 사용
- **학습 기반**: 자유공간 잔차 모델(SVGP 또는 MLP), 작업 전 보정 동작 후 마지막 층만 재추정
- **추론 시간**(M4, numpy 실측)

| 항목 | 시간 |
|---|---|
| MLP 추론 | 5 µs |
| SVGP 추론 | 35 µs |
| 1D-CNN 추론 | 52 µs |
| LSTM 추론 | 428 µs |
| 마지막 층 재추정 | 18 ms |
| GP q(u) 재적합 | 0.19 s |
| MLP 전체 재학습 | 6.1 s |

  - 결론: 제어 주기(10–20 ms) 대비 무시할 수준이고, 작업 전 준비 시간은 보정 동작(30–60초)이 좌우합니다.

### 이 대화 밖에서 진행된 변경 (파일 기준)
- 21:21 팀 저장소 커밋 "Add ROS simple motion and RG2 gripper runtime"
- 22:11 `Dockerfile`: `ros-jazzy-rosbridge-server` 설치
- 22:11 `compose.yaml`: 9090 포트(rosbridge)
- 22:11 `bridge.py`: `sim_joint_states` 발행 추가
- 22:58 `native/session.py`: `--web`, `--web-port 5050`, `--web-dir` 옵션(가상 전용, `rokey-b3-project1/web`의 `SimulationWeb` 실행)
- 팀 저장소 `web/`: `robot_session.py`, `robot_model.py`, 로봇 뷰어 JS 추가(커밋 전)

### 실행 구조 정리 (병렬 작업 정리)
- **`./run_virtual.sh`**: 로봇만 있는 월드와 로봇 노드(컨트롤러, 가상 힘제어 브리지, RViz)를 띄웁니다.
- **`./run_virtual.sh --map <맵>`**: 위와 같고 맵 장면만 추가됩니다.
  - 탐침 맵이면 탐침 툴 프로파일과 `ws_cobot_pjt/session/probe_plan.json`이 생성됩니다.
  - 가상 힘제어 TCP는 맵이 있어도 바꾸지 않습니다. 이전에는 `probe_tip`으로 바뀌었습니다. 도넛 배치는 그대로입니다.
- **제거한 항목**
  - `--pickup-probe`(세션이 작업을 자동 실행)
  - `--force-test-fixture`(맵과 별개의 테스트 장면), 이를 쓰던 `cobot_virtual/demo.py`, `scripts/virtual_force_demo.py`
  - 이전 세션이 남긴 `session/probe_plan.json`
- **로봇 프로그램은 모두 패키지 실행파일**입니다. 맵 유무와 관계없이 컨테이너 bash에서 `ros2 run rokey <program>`으로 실행합니다(최초 1회 `colcon build`).
  - 캘리브레이션: `force_physics_calibrate`, `force_collect`, `force_learning_train`, `force_learning_calibrate`
  - 점검과 작업: `interface_check`, `pick_probe_and_scan`, `donut_scan`(탐침 맵 필요)

- **`--web`**: 웹 뷰어가 시뮬레이션과 같은 장면을 보여줍니다.
  - 맵이 없으면 로봇만, `--map`이면 로봇과 맵이 보입니다. 맵 마커는 `/dsr01/sim_scene`(5 Hz)으로 받고, 도넛 메시는 STL로 내보냅니다.
  - 그 외 동작은 바꾸지 않았습니다(가상 전용, 로봇 동작에 영향 없음).
- **"서비스 발견 실패" 원인**: 에뮬레이터 컨테이너가 compose 네트워크에서 떨어져 있었습니다. 그래서 컨트롤러가 `emulator`에 연결하지 못했고(`Host not found`), `dsr_controller2` 서비스가 하나도 없었습니다. 이제 세션이 시작할 때 이를 확인하고, 문제가 있으면 에뮬레이터를 다시 만듭니다.

### 검증 상태
- 오늘 바꾼 스캔과 캘리브레이션 코드는 **문법 확인과 오프라인 계산 검증까지만** 했습니다.
- Gazebo 시뮬레이션과 실기 실행 결과는 아직 없습니다.

### 다음 할 일
1. 컨테이너에서 `colcon build --symlink-install --packages-select rokey m0609_rg2_bringup`
2. `./run_virtual.sh` → `ros2 run rokey force_physics_calibrate`로 `images/force_physics_virtual_*.png` 확인
3. `./run_virtual.sh --map donut` → `ros2 run rokey pick_probe_and_scan --pick-only`
4. `ros2 run rokey interface_check noise`로 `--touch-threshold`와 `--search-speed` 결정
5. `ros2 run rokey donut_scan --lines 0 --touch-threshold X`로 실제 접촉 확인
6. `interface_check free`, `contact` 결과로 `--mode stream` 사용 여부 결정
7. 임계값 최적화 P0: RT 로깅과 오프라인 재생 최적화 기반 구축
