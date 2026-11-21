# 탐침 표면 추종 추가 구현 계획

작성 기준: 2026-11-15의 실제 파일, README, `WORKLOG.md` 검토. 이 문서는 계획이며 구현·동작 검증 완료를 뜻하지 않는다.

**1. 현재 구현 방향과 적용 위치**

개발 대상은 이 디렉토리 아래 `ws_cobot_pjt/src/rokey`이다. `rokey-b3-project1/ws_cobot_pjt`의 기본 제어 배포본과 구분한다.

현재 파일과 작업 기록에서 확인한 방향은 다음과 같다.

```text
run_virtual.sh --car sedan|suv|sports|random [--web]
└─ native/session.py
   ├─ Docker: DRCF / ros2_control / 가상 힘 제어 브리지
   ├─ native: Gazebo 물리 / RViz
   └─ 선택: 웹 5050 / rosbridge 9090

rokey/repair_manager.py
├─ pick_probe.py                    가상 환경에서 미파지 시 탐침 픽업
├─ calibration.py                   힘 오차 모델 / 이동 중 노이즈 / 반복 접촉 높이 σ
├─ choose_car.py                    접촉으로 차종·차량 위치·yaw 식별
├─ scan_surface.py                  정상 차량 모델과 비교하는 부품 검사
│  ├─ PartPlan                      보닛 외곽선·왕복 래스터 / 작은 부품 터치
│  ├─ touch.py / VerticalTouch       수직 자세 고정, 접촉 확인, slide 후 재안정화
│  └─ DamageAnalysis                높이차·평면 잔차 보정·중앙값 필터·손상 분류
├─ repair_db.py / check_db.py        DB 기록과 재고 확인
└─ replace_parts.py                 교체 인터페이스 뼈대
```

- `repair_manager`는 현재 `scan_parts()`를 직접 호출한다. Action 클라이언트가 아니다.
- `replace_parts`는 가상 환경에서 동작 없이 로그와 성공 결과를 내는 뼈대이며 실기 교체는 거절한다.
- 현재 차량 검사는 미지 물체 전체 복원보다, 식별한 정상 모델과 실제 표면의 차이를 구하는 방향이다.
- 현재 설정은 접촉 이동 5 mm/s, 누름 힘 1.5 N, 점 간격 4 mm, 라인 간격 7 mm다. 새 비교용 설정은 별도로 둔다.
- `calibration/force_physics_<mode>.json`은 생성되지만 현재 `ContactProbe.read()`는 tare를 적용한다. 오차 모델이 모든 스캔 샘플에 실시간 적용된다고 가정하지 않는다.
- 예전 `make_plt_prac.py`, `donut_scan.py`, `followers.py`, `surface_normal.py`를 확장하는 계획으로 진행하지 않는다. 현재 개발 패키지에 해당 파일들이 없다.
- `native/virtual_force.py`의 어드미턴스는 가상 힘 제어 백엔드다. 실기 공용 표면 추종 제어기와 구분한다.

**2. 변경 경계: 기존 알고리즘과 호출 흐름 보존**

기존 `rokey` Python 파일, `setup.py`, `package.xml`, YAML, `repair.launch.py`, native 세션·가상 백엔드 및 기존 웹 명령 처리는 수정하지 않는다.

새 기능은 같은 colcon 워크스페이스에 두 패키지로 추가한다. 기존 `rokey_interfaces`와의 이름 충돌을 피한다.

```text
ws_cobot_pjt/src/
├─ rokey/                                  기존 구현 유지
├─ rokey_surface_interfaces/                신규 ament_cmake 패키지
│  ├─ action/FollowSurface.action
│  ├─ CMakeLists.txt
│  ├─ package.xml
│  └─ README.md
└─ rokey_surface_scan/                      신규 ament_python 패키지
   ├─ rokey_surface_scan/
   │  ├─ action_server.py                   Goal / Feedback / Result / Cancel
   │  ├─ client.py                          방법 선택·부품 순차 호출
   │  ├─ context.py                         설정·차량·TCP·보정 결과 스냅샷
   │  ├─ robot_adapter.py                   새 비동기 DSR 클라이언트와 명령 전송
   │  ├─ state_source.py                    가상/실기 상태 획득과 시간 정렬
   │  ├─ geometry.py                        탐침 기하·경로·법선·자세 정렬
   │  ├─ controllers.py                     HybridFollower / AdmittanceFollower
   │  ├─ measurement.py                     측정점 판정·검사 결과 변환
   │  ├─ recording.py                       PointCloud2 / CSV / PLT / 지표
   │  └─ native_cloud_bridge.py             선택: 표준 PointCloud2만 host로 중계
   ├─ config/surface_following.yaml
   ├─ launch/surface_following.launch.py
   ├─ setup.py / package.xml
   └─ README.md
```

`VehicleModel`, 설정·결과 형식, 회전 계산 등 재사용 가능한 순수 계산은 읽기 전용으로 사용한다. 기존 `RobotProgram`의 `rclpy.init()` 및 `RobotServices`의 내부 executor를 새 Action executor와 중복 실행하지 않는다. 새 로봇 어댑터는 동일 DSR 서비스·메시지 계약을 사용한다.

새 launch는 기존 `repair_manager`를 시작하지 않는다. 기존 웹 버튼과 `scan_surface`는 원래 방식으로 동작한다. 새 방식을 기존 자동 수리 공정에 연결하는 것은 별도의 연결 변경이며 이번 무수정 범위에 포함하지 않는다.

새 Action 서버는 한 번에 Goal 하나만 받는다. 기존 프로그램은 새 서버의 소유권을 알지 못하므로, 기존 `repair_manager`나 다른 이동 프로그램과 새 추종기를 같은 로봇에 동시 실행하지 않는 것이 운영 전제다. namespace 분리만으로 물리 로봇 제어가 분리되지는 않는다.

**3. ROS Action 계약**

Action endpoint: `/dsr01/surface_scan/follow`.

한 Goal은 부품 하나의 스캔이다. 부품 내부의 outline/raster는 서버가 관리하고, 여러 부품은 클라이언트가 순차 호출한다. 두 제어기는 같은 Action 타입과 서버를 사용하되 구현 클래스를 구분한다.

예정 인터페이스:

```text
# Goal: 숫자 단위는 SI, method와 profile은 실행 도중 변경하지 않는다.
uint8 HYBRID=0
uint8 ADMITTANCE=1
uint8 method
string part_code
string profile
float64 target_force_n
float64 tangential_speed_mps
float64 point_spacing_m
---
# Result: ROS의 SUCCEEDED / ABORTED / CANCELED 상태와 함께 사용한다.
uint16 error_code
string message
string output_directory
uint32 valid_points
float64 force_rmse_n
---
# Feedback: 제어 주기와 분리하여 5~10 Hz 발행한다.
string run_id
string phase
float32 progress
float64 normal_force_n
float64 travelled_m
float64 alignment_error_rad
uint32 valid_points
geometry_msgs/PoseStamped tcp_pose
```

- Action은 작업 수명 주기를 관리한다. 샘플마다 Goal을 보내거나 Feedback으로 로봇을 제어하지 않는다.
- 인터페이스 생성은 `ament_cmake` + `rosidl_default_generators`, 실행 서버·클라이언트는 `ament_python` + `rclpy.action`으로 구성한다.
- 추가 custom message는 우선 만들지 않는다. 실제 측정점은 표준 `PointCloud2`, 상세 값은 파일과 Action Feedback으로 전달한다.
- Goal 수락 시 `vehicle.json`, calibration 결과, TCP/Tool 정보, 차량 모델·설정의 버전을 고정하여 실행 중 파일 변경의 영향을 받지 않게 한다.
- 기존 `/repair/command`, `/repair/status`, REST API의 형식은 바꾸지 않는다. 새 Action이 기존 웹 시작 버튼에서 자동 호출된다고 문서화하지 않는다.
- 동시 Goal이나 잘못된 설정은 이동 전에 거절한다. 실행 후 오류 코드는 `CONTEXT_INVALID`, `COMMAND_UNSUPPORTED`, `STATE_STALE`, `FORCE_LIMIT`, `CONTACT_LOST`, `GEOMETRY_LIMIT`, `STOP_NOT_CONFIRMED`로 구분하고 숫자 값은 interface 상수로 고정한다. 취소 여부는 ROS Action 상태로 구분한다.

**4. 공통 실행 상태와 중단 처리**

```text
CONTEXT_CHECK → HOME → APPROACH → SEARCH → SETTLE
                                      ↓
                                     TRACK → RETRACT → HOME → SUCCEEDED
                                      ↓
                              STOPPING → CANCELED / ABORTED
```

- 선행 준비는 기존 `pick_probe`, `calibration`, `choose_car`를 그대로 실행한다. 새 클라이언트의 준비 옵션을 제공한다면 이 실행파일들을 순차 호출한다.
- 실기 탐침 픽업은 현재 구현 범위를 넘어 자동화하지 않는다. 등록된 탐침 TCP/Tool과 파지 상태를 준비 조건으로 둔다.
- Action 서버의 callback 처리와 주기 제어·상태 I/O를 분리한다. 각 ROS node의 executor 소유자를 하나로 정하고, 제어 루프에서 동기 서비스 응답을 기다리지 않는다.
- 취소 시 새 명령 발생을 먼저 차단하고, 지원되는 0 속도 명령 및 별도 비동기 정지 요청을 보낸다. 정지 확인·제어 해제 후 부분 결과를 저장하고 `CANCELED`를 확정한다.
- 취소 중 HOME 복귀를 강행하지 않는다. 정상 완료는 표면에서 먼저 이탈한 뒤 자세를 복원하고 HOME으로 복귀한다.
- 필수 제한은 힘/횡력, 상태 신선도, 명령 만료, 이동·회전·충돌 범위로 제한한다. 무한 재시도나 무조건적인 자동 재접촉은 넣지 않는다.

**5. 공통 상태·기하·측정 처리**

1. 가상 상태는 기존 `/dsr01/sim_joint_states`와 `/dsr01/virtual_control/wrench`를 stamp로 짝짓는다. 실제 관절값과 활성 탐침 TCP 변환으로 FK를 계산한다. 현재 가상 `get_current_posx`는 위치 편차만 더하므로 그 회전값을 실제 Gazebo 자세로 가정하지 않는다.
2. 실기는 현재 드라이버의 RT 출력 접근 가능 여부를 먼저 확인한다. 동일 RT 데이터의 pose/force를 우선 사용한다. 에뮬레이터에는 RT 연결을 요구하지 않는다. 서비스 분리 읽기를 사용하는 경우 시각·지연·보간 여부를 기록하고 충분한 갱신률을 실측한다.
3. 내부 계산 및 신규 ROS 인터페이스는 BASE 기준 m/rad/N/Nm로 통일한다. DSR API와 기존 YAML의 mm/deg 변환은 어댑터 경계에서 수행한다. 웹의 rear-left/right 방향 좌표는 `to_interface()` 경계에서만 사용한다.
4. 새 실시간 힘 보정기는 저장된 오차 모델을 명시적으로 한 번만 적용한다. 기존 배치용 `ForceData`의 중앙 차분·평활화를 실시간 처리라고 간주하지 않고 인과적인 추정기로 구분한다. 적용 범위가 확인된 none/static부터 시작하며 원시·보정 값을 함께 기록한다.
5. 바깥쪽 법선을 `n`, TOOL +Z 방향을 `z_tool`로 정의한다. 접촉 반력은 `Fn = F_base · n`, 자세 목표는 `z_tool = -n`이다. 힘 부호를 절댓값으로 없애지 않는다.
6. 현재 ProbeTCP는 탐침 구의 축 방향 끝점이다. 따라서 `c = p_TCP - r*z_tool`, `p_contact = c - r*n`으로 계산한다. TCP와 구 중심을 혼동하여 반지름을 두 번 보정하지 않는다.
7. 정상 차량 모델은 경로와 초기 법선의 사전 정보로 사용한다. 접촉 중에는 실제 힘과 인접 측정점의 3차원 국소 기하로 법선을 보정한다. 단일 선분의 점만으로 3차원 법선 전체가 관측된다고 가정하지 않는다. `scene_truth.json`의 손상 정보는 제어 입력에서 제외하고 평가에만 쓴다.
8. 첫 적용은 기존 `PartPlan`의 보닛 outline/raster를 읽어 연속 경로로 만드는 방식이다. 이후 별도 3차원 mesh 경로 생성부를 추가한다. 기존 높이맵이나 `PartPlan`을 수정해 수직면을 억지로 표현하지 않는다.
9. 곡면 속도는 BASE X 성분이 아닌 표면 접선의 이동거리 기준으로 일정하게 한다. 기본 비교값은 5 mm/s, 기록 간격은 0.5 mm이며 기존 4 mm 설정은 변경하지 않는다. `1/tangent_x`로 속도를 확대하지 않는다.
10. 자세는 회전행렬/Quaternion으로 보간하고 각속도를 제한한다. 수직면 90도는 기하학적 추종 목표이며, 로봇 도달 범위와 탐침 몸체·RG2 충돌을 통과한 구간만 대상으로 한다. 수직용 `shadow()` 검사를 기울어진 그리퍼의 전체 충돌 검사로 간주하지 않는다.

**6. 방향 A: 내장 힘 제어 기반 HybridFollower**

접선 위치/속도 추종, 법선 내장 힘 제어, 법선에 대한 자세 정렬을 분리한다.

```text
P_t = I - n nᵀ
v_t = feed * tangent + Kp * P_t * (reference - measured_center)
omega = Kr * (z_tool × (-n))
normal channel = Doosan TOOL Z force control
```

- 초기 접촉 탐색·정지 확인 후 내장 순응/힘 제어를 설정하고 목표 힘을 서서히 적용한다. 초기 부호는 현행 ContactProbe의 TOOL Z 정의와 일치시킨다.
- 법선 방향 위치 보정을 외부에서 중복 주입하지 않는다. 내장 `dir=0` 축을 강체 위치 제어라고 설명하지 않고 경로 명령과 강성 설정을 함께 다룬다.
- 평면 기준선은 연속 `movel`을 사용할 수 있다. 법선과 자세가 바뀌는 곡면은 지원이 확인된 연속 명령 전송부를 사용한다. 0.5 mm마다 정지하는 `movel` 반복으로 대체하지 않는다.
- TOOL 힘 제어 축이 자세 변화에 따라 언제 갱신되는지, 힘 제어와 병진·회전 streaming이 함께 허용되는지를 가상/실기 각각 확인한다. 기존 firmware가 거절한 `alter_motion`을 무조건 다시 사용하지 않는다.
- 지원되지 않는 조합은 명확한 capability 오류를 반환한다. 힘 제어를 켰다 끄는 단계 이동을 연속 Hybrid 성공으로 표시하지 않는다.

**7. 방향 B: 외부 AdmittanceFollower**

접선 경로·법선 추정·자세 정렬은 Hybrid와 같게 두고, 법선 움직임 생성부만 교체한다.

진입 방향 속도 `u`를 양수로 정의한 초기 법칙:

```text
M * du/dt + D * u = F_target - F_normal
v_command = v_t - u * n
angular_command = 동일한 법선 자세 정렬
```

- 첫 구현은 법선 1축의 질량·감쇠 모델이다. 법선 강성은 0으로 시작하고, 일반적인 6축 자유 어드미턴스로 확장하지 않는다.
- 명령은 병진·회전을 합친 하나의 연속 명령 스트림으로 보낸다. 오차가 양수이면 진입, 음수이면 이탈하도록 단위·부호를 검증한다.
- 내장 force/compliance는 해제하고 외부 어드미턴스만 법선 움직임을 결정하게 한다. 가상 환경에서도 기존 backend의 compliance 어드미턴스와 중복 작동하지 않는 모드를 선택한다. 기존 backend 코드는 수정하지 않는다.
- 접촉 시작·재시작 시 속도 상태를 초기화하고 목표 힘을 ramp한다. 실제 dt로 적분하며 속도·변위 제한에 닿으면 누적 상태가 계속 커지지 않도록 처리한다.
- 초기 질량·감쇠 모델은 `u_next = (M*u_previous + dt*(F_target-F_normal)) / (M+D*dt)`의 후진 Euler로 이산화한다. M은 kg, D는 N·s/m이며 양수로 둔다. 이 이산화만으로 통신 지연·표면 강성을 포함한 전체 접촉 안정성이 보장된다고 주장하지 않는다. M/D의 실제 값은 짧은 평면 접촉에서 정하고 가상 backend 값을 실기로 그대로 복사하지 않는다.
- 신규 제어 목표는 우선 50 Hz로 두되 실측 지연과 갱신률을 통과해야 사용한다. RT라는 이름이나 빠른 timer 설정만으로 주기 충족을 주장하지 않는다.

두 방식 모두 연속 명령이 실제로 로봇을 움직이는지 확인하는 것이 선행 조건이다. 현재 emulator/firmware에서 명령 조합이 지원되지 않으면, 기존 로직을 변경하는 우회 대신 신규 어댑터 추가 범위와 지원 가능한 모드를 먼저 확정한다.

**8. 검사 호환성·출력·시각화**

- 기존 검사 결과의 `findings`, `replace`, 부품 코드, 상태 이름을 유지하는 별도 ResultAdapter를 둔다. 기존 `scan_parts()`를 교체하거나 `session/scan_result.json`을 덮어쓰지 않는다.
- 상면의 기존 높이차 비교와 3차원 접촉점의 법선 거리차를 구분한다. 수직면을 기존 `dz`와 같은 값으로 취급하지 않는다. 기존 상면 분석을 재사용할 때도 구형 탐침 기준 높이에 맞게 변환한 입력만 전달한다.
- 상면 호환 입력은 구 중심의 실제 XY에서 평가하고, 수직 등가 탐침 끝 높이 `c_z-r`와 기존 `model.probe_height()`를 비교한다. 3차원 검사는 `d_n = (p_contact-p_reference)·n_reference`를 사용해 함몰이 음수가 되게 한다. `d_n`을 기존 웹의 `dz` 필드에 그대로 넣지 않는다.
- 제어 실패/힘 부족으로 버린 점을 바로 `missing`으로 분류하지 않는다. 정해진 탐색 범위를 확인했는데 접촉하지 못한 경우와 구분한다. 작은 부품은 우선 현행 터치 검사와 함께 사용하고 보닛에서 두 연속 제어기를 먼저 완성한다.
- 현재 `press_force=1.5 N`, 보정된 `touch_threshold`, ContactProbe의 유효 접촉 하한은 서로 다른 설정이다. 신규 profile에서도 이를 분리한다. 과거의 1.5 N 이하 일괄 제외 정책을 현행 목표 힘에 그대로 덮어씌우지 않는다.
- 실제 접촉 XYZ를 `/dsr01/surface_scan/hybrid/points`, `/dsr01/surface_scan/admittance/points`에 PointCloud2로 발행한다. `base_link`, m 단위, transient-local depth 1을 사용하고 제어 루프와 별도의 주기로 갱신한다.
- 기존 `pointcloud.py`는 기준 표면과 `dz`로 점을 재구성하므로 신규의 실제 측정점 출력기로 대체 사용하지 않는다.
- native RViz는 domain 129, Docker는 domain 29다. 같은 토픽 이름만으로 연결되지 않는다. 기존 relay 수정 없이 선택 실행하는 `native_cloud_bridge.py`가 rosbridge 9090에서 표준 PointCloud2를 구독해 native domain에 재발행한다. 이 스크립트는 Mac에서 Linux custom-interface 바이너리를 import하지 않는다.
- 웹의 기존 로봇·차량 시각화는 유지한다. 신규 제어 선택 UI와 신규 3D 점구름 웹 표시까지 완료되었다고 표시하지 않는다.
- 결과는 설정 가능한 별도 run 디렉토리(기본: `rokey/pointcloud/surface_following/<run_id>/`)에 기록한다. `result.json`, 원시/보정 pose·wrench·contact CSV, PLT, 힘/자세 오차 그래프와 메타데이터를 저장한다.
- 메타데이터에는 method, 입력 snapshot, 모델/TCP/보정 버전, 설정, 실제 dt·지연, rejected 이유, 취소/오류 및 부분 결과 여부를 포함한다. DB 쓰기는 제어 루프에서 하지 않는다.

**9. 구현 순서와 완료 기준**

| 단계 | 구현 산출물 | 확인할 내용 |
|---|---|---|
| P0 | 현재 입력·실행 경계 고정 | 기존 12개 executable/공정이 유지되고 새 프로그램과 동시 제어하지 않음 |
| P1 | interfaces, Action server/client, 새 DSR/상태 adapter | 타입 생성, Goal 수락/거절, 취소 callback, 실측 데이터와 명령 capability |
| P2 | 공통 경로·기하·기록부 | TCP/구 중심 변환, 힘 부호, 좌표·단위, 동기화·거부점 기록 |
| P3 | HybridFollower | 가상 평면/보닛 짧은 한 줄, 내장 힘·접선 이동·자세 갱신 조합 |
| P4 | AdmittanceFollower | 동일 입력·경로에서 외부 법선 제어, 중복 힘 제어 없음 |
| P5 | 3D 경로·경사 확장 | 경사면부터 수직 근처까지 단계적으로, 각속도/접촉 유지/몸체 간섭 |
| P6 | 결과·문서 완료 | 실측 PointCloud2/CSV/PLT, 제한된 비교 결과, README와 실행파일 일치 |

- 처음부터 차량 전체·전체 수리 공정을 반복 실행하지 않는다. 수학/제어의 의미 있는 소수 검증과 방법별 짧은 한 줄 실행, 취소·접촉 소실 확인부터 한다.
- 힘 허용오차는 초기에는 현행 ContactProbe의 ±0.5 N을 비교 기준으로 삼고, 유지율·RMS·최대 힘·법선 정렬 오차·실제 속도·접촉 소실률·점 간격·실제 제어 주기를 함께 보고한다. 실기 합격 수치는 짧은 측정 후 확정하며 계획 단계에서 보장하지 않는다.
- `--simulate`처럼 모델에서 점을 직접 만든 결과와 Gazebo 실제 접촉 결과를 구분한다. 가상 실행 결과를 실기 검증으로 표시하지 않는다.
- 수직면의 거리 오차 임계값은 별도 반복 측정으로 정한다. 수직 반복 접촉의 높이 σ를 모든 자세에 그대로 적용하지 않는다.
- 명령 지원이 미확정이거나 실제 접촉/취소가 확인되지 않은 방법은 완료로 표시하지 않는다.

**10. 예정 실행 형태 — 신규 패키지 구현 후 사용**

```bash
# 개발용 세션(호스트). 기본 제어 배포본 경로와 혼동하지 않는다.
cd /Users/wonjun/Desktop/wonjun/ws_cobot_pjt
./run_virtual.sh --car sedan --web

# 컨테이너 안. 선행 준비는 현재 실행파일 그대로 사용한다.
ros2 run rokey pick_probe
ros2 run rokey calibration
ros2 run rokey choose_car

# 신규 Action 서버: 기존 repair_manager와 같은 로봇을 동시에 제어하지 않는다.
ros2 launch rokey_surface_scan surface_following.launch.py

# 별도 컨테이너 터미널: 같은 조건으로 방법별 순차 실행
ros2 run rokey_surface_scan scan --method hybrid --part bonnet
ros2 run rokey_surface_scan scan --method admittance --part bonnet
```

클라이언트 Ctrl+C는 Goal cancel로 연결하고, 서버의 정지 확정을 기다린다. 빌드는 새 패키지와 의존 패키지를 `--packages-up-to rokey_surface_scan`으로 빌드한다. 위 신규 실행파일과 launch는 현재 존재하지 않는다.

**11. 구현 완료 후 README 반영 범위**

현재 README의 미반영·불일치를 다음 목록으로 관리하고, 실제 구현/검증 상태에 맞춰 마지막 단계에서 반영한다.

| 문서 | 반영할 내용 |
|---|---|
| 이 디렉토리의 `README.md` 신규 | 개발용 세션 구조, `--car`/`--car-pose`/`--web`, 5050/9090, domain 29/129, 기본 배포본과의 위치 관계 |
| 개발용 `rokey/README.md` 신규 | 현재 12개 실행파일, HOME/탐침/보정/식별/스캔/DB 흐름, config·session·출력 관계 트리 |
| 새 interfaces/scan 패키지 README | Action 필드/단위/상태/취소, 두 알고리즘, 기본 설정·실행 명령·지원 범위·검증 상태 |
| `rokey-b3-project1/web/README.md` | 실제 5050 포트, 실제 파일 트리, robot_model/robot_session/로봇 뷰어/차종별 GLB/scene, 현재 존재하지 않는 `ros/*stub.py` 안내 정정 |
| 기본 제어 배포본 README | 그 배포본에 실제 포함된 웹 연동 옵션을 설명하고 기본 제어 범위를 유지. 개발용 전체 공정이나 새 스캔 패키지가 포함되었다고 쓰지 않음 |

특히 아래 항목을 누락하지 않는다.

- `calibration.json`의 TCP/Tool 프로파일, `calibration/force_physics_<mode>.json`의 오차 모델, `session/calibration_result.json`의 임계값/높이 σ를 구분한다.
- `probe_plan.json`, `vehicle.json`, `scene_truth.json`, `scan_result.json`, `stock_result.json`의 생성자·사용자·유지 기간을 실제 코드 기준으로 적는다.
- choose_car는 고정 3점 정합이 아니라 차종/위치/yaw 가설을 접촉으로 줄이는 방식임을 설명한다.
- scan_surface의 outline/raster·작은 부품 터치·안정화·손상 판정과 신규 연속 추종을 구분한다.
- 기존 `/repair/pointcloud`는 기준 모델 점구름이고 `/repair/scan_points`는 모델+높이차로 재구성된 점임을 적는다. 신규 실제 접촉점 토픽과 구분한다.
- `repair_manager`의 직접 호출 방식과 새 Action의 선택 실행 방식, 수리 DB의 기록 책임을 설명한다.
- 부품 교체는 아직 뼈대이며 가상 로그상의 성공이 실제 교체 완료가 아님을 명시한다.
- 삭제된 make_plt/donut/interface_check 계열 명령을 현행 실행 명령으로 다시 안내하지 않는다.
- 코드 있음/오프라인 검증/가상 접촉 검증/실기 검증을 나누어 적는다.

모든 신규 README에는 코드 관계 트리를 포함한다. 기존 `docs/INTERFACE.md`의 웹/DB 계약은 임의로 변경하지 않고, 실제 구현과 차이가 있는 지점을 README에 설명한다. 현재 계획 단계에서는 기존 코드·README·인터페이스 계약을 수정하지 않는다.

참조:

- ROS 2 Jazzy custom Action: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Creating-an-Action.html
- ROS 2 Jazzy Python Action server/client: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
- Doosan SetDesiredForce 서비스: https://manual.doosanrobotics.com/en/ros/1.14/publish/setdesiredforce-srv
- ROS 2 admittance 구성: https://control.ros.org/jazzy/doc/ros2_controllers/admittance_controller/doc/userdoc.html
