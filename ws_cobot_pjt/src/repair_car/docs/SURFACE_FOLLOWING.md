# 선택 실행하는 탐침 표면 추종

`surface_follow`는 **native Gazebo 가상 세션 전용**이다. 기존 `scan_surface`, `repair_manager`, 픽업·보정·차종 식별 알고리즘은 유지한다. 새 명령으로 두 제어기를 각각 선택한다. 기존 자동 수리 공정에 자동 연결하거나 실기에서 실행하는 기능은 추가하지 않았다.

## 실행 구조

```text
run_virtual.sh --car sedan [--web]
├── native/virtual_force.py                 기존 백엔드 + 새 스트림 분기
│   └── native/surface_stream.py            100 Hz Cartesian 참조 / IK / watchdog
└── m0609_rg2_bringup/cobot_virtual/bridge.py
    ├── 기존 DSR 힘 서비스                  기존 실행 경로
    └── /dsr01/surface_follow/*              새 상태 / 연속 명령 / 시작·해제

ros2 run repair_car surface_follow --method hybrid|admittance
└── repair_car/surface_follow/
    ├── cli.py / config.py                  실행 옵션 / SI 설정
    ├── geometry.py                         기존 PartPlan 경로 + 법선 prior / 관측 보정
    ├── controllers.py                      공통 상태 전이 + 두 법선 제어기
    ├── transport.py                        상태 수신 / 영점 / 연속 명령
    ├── runtime.py                          HOME / 접근 / 추종 / 이탈 / HOME
    └── recording.py                        측정 PointCloud2 / CSV / PLT / 요약
config/surface_following.yaml                새 추종기 설정
```

Action custom interface는 만들지 않았다. 현재 실행 단위는 CLI 작업 하나이며 표준 `String`, `Trigger`, `PointCloud2`를 사용한다. 작업 큐·원격 Goal/Cancel이 필요해지면 이 실행부를 Action으로 감쌀 수 있다.

## 두 제어기의 역할

공통으로 40 Hz 외부 루프에서 경로 접선, 모델 prior와 실제 힘 기반 법선, 탐침 자세를 갱신한다. 내부 계산은 BASE 기준 m/rad/N/Nm이다. 기존 DSR 이동 서비스와 연결되는 접근/HOME 경계에서만 mm/deg로 변환한다.

| 선택 | 접선 채널 | 법선 채널 |
|---|---|---|
| `hybrid` | 위치 참조 | 외부 PI가 목표 힘을 보정하고 native의 가상 법선 힘 조절기가 조절 |
| `admittance` | 위치 참조 | 외부 `M*u_dot + D*u = F_target - F_normal`이 법선 이동을 생성. native는 위치 참조만 추종 |

하이브리드의 내부 힘 조절기도 Gazebo의 위치 구동기에 연결하기 위해 M=8 kg, D=800 Ns/m인 가상 법선 동역학을 사용한다. 제조사의 내장 힘제어기를 재현한 것은 아니다. 어드미턴스 모드에서는 이 법선 동역학 및 기존 compliance 분기를 실행하지 않아 법선 제어가 중복되지 않는다. YAML의 `mass`, `damping`은 외부 어드미턴스에 적용한다.

바깥쪽 법선을 `n`이라 할 때 접촉 반력은 `Fn = F · n`, 탐침 TOOL +Z 목표는 `-n`이다. 양의 힘 오차는 표면으로 진입하는 방향이다. 하이브리드 가압 축은 native에서 **실제 TOOL 축**을 사용한다.

기존 백엔드의 calibrated TCP와 픽업 후 ProbeTCP는 다를 수 있다. 새 스트림은 URDF의 `probe_tip`을 직접 FK/IK 기준으로 삼고 힘 모멘트도 그 지점으로 옮긴다. ProbeTCP는 구 중심이 아니라 축 방향 구 끝점이므로 다음처럼 위치를 계산한다.

```text
sphere_centre = actual_TCP - radius * actual_TOOL_Z
contact_point = sphere_centre - radius * estimated_outward_normal
```

상태 메시지에는 같은 100 Hz 제어 표본에서 계산한 실제 FK 자세·속도와 당시의 보정 힘을 묶는다. 원본 힘 센서는 200 Hz, 기존 힘 필터는 80 ms, 브리지 상태 전달은 약 60 Hz이다. 힘 센서의 원래 표본 시각까지 완전히 동기화한 하드웨어 측정은 아니다. filtered 힘과 필터 전 보정 힘을 함께 보내며 후자를 빠른 과부하 경로에 사용한다. 예전 위치 서비스의 Euler 각을 실제 Gazebo 회전값으로 사용하지 않는다.

## 시작과 정지

```text
가상 모드 / 탐침 파지 / TCP·Tool 확인
→ HOME → HOME 비접촉 영점 → 안전 높이로 접근
→ SEARCH → CONFIRM → SETTLE → TRACK
→ 정상 완료: RETRACT → 스트림 해제·에뮬레이터 동기화 → HOME

과부하: UNLOAD → RECOVER → CONFIRM → SETTLE → TRACK
취소·오류: 명령 중단 → 실제 관절 위치 유지 → 정지 확인 / 부분 결과 저장
```

접근 지점에서 예상 밖 접촉이 있으면 그 접촉을 영점으로 지우지 않고 중단한다. 목표 힘 램프, PI 적분 포화 방지, 어드미턴스 속도/참조 오차 제한, 힘 증가율 기반 감속, 제한된 후퇴·재접촉을 적용한다. 지연·접촉 강성·제동 가정으로 계산한 속도 제한은 요청 속도를 상당히 낮출 수 있다. 기본 가정은 100 ms 지연 예산, 3500 N/m 강성, 0.08 m/s² 제동이며 실측값이 아니다.

native는 명령이 150 ms 넘게 오지 않거나, 피드백이 오래되거나, 보정 힘이 8 N을 넘거나, 다른 프로그램이 nominal 관절 명령을 바꾸면 위치 유지로 전환한다. 잘못된 소유권의 명령은 수락하지 않는다. 힘/모멘트가 구 접촉 기하와 맞지 않으면 외부 루프도 중지한다.

Ctrl+C 취소 때는 자동 HOME이나 탈출 경로를 강행하지 않는다. 새 피드백에서 병진·회전 속도가 작아졌는지 확인한 결과를 `stop_motion_confirmed`로 저장한다. 이 값은 **접촉 하중 해제 확인이 아니다**. 위치 유지는 접촉 하중을 남길 수 있다. 정지 확인 실패는 성공으로 기록하지 않는다. 별도 정지 요청은 다음과 같다.

```bash
ros2 service call /dsr01/surface_follow/end std_srvs/srv/Trigger '{}'
```

## 빌드와 실행 순서

실행 중인 세션에는 새 Python 모듈이 자동 반영되지 않는다. 컨테이너가 켜져 있을 때 새 엔트리와 브리지를 빌드한다.

```bash
docker exec -it ws_cobot_pjt bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon build --symlink-install --packages-select repair_car m0609_rg2_bringup
exit
```

현재 native 세션을 종료하고 호스트에서 다시 시작한다.

```bash
cd /Users/wonjun/Desktop/wonjun/ws_cobot_pjt
./run_virtual.sh --car sedan --web
```

다른 터미널에서 컨테이너에 들어가 같은 세션의 탐침을 파지하고 차량을 식별한다. 이미 이번 세션에서 완료했다면 해당 준비 명령은 생략한다. 다른 이동 프로그램이나 `repair_manager`와 동시 실행하지 않는다.

```bash
docker exec -it ws_cobot_pjt bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

ros2 run repair_car pick_probe
ros2 run repair_car choose_car

# 먼저 경로와 설정만 확인: ROS 초기화 / 이동 없음
ros2 run repair_car surface_follow --plan-only --part bonnet --path-index 1

# 아래 두 명령 중 하나를 실행
ros2 run repair_car surface_follow --method hybrid --part bonnet --path-index 1
ros2 run repair_car surface_follow --method admittance --part bonnet --path-index 1
```

기본 실행은 식별 결과 `session/vehicle.json`과 정상 차량 모델을 사용해 **보닛의 첫 번째 raster 경로 하나**를 수행한다. 단면 전체/차량 전체 검사가 아니다. `path-index=0`은 기존 outline이며 다른 번호는 같은 PartPlan의 경로다. `--force`, `--speed-mm-s`, `--duration`을 선택할 수 있다. 손상된 장면의 `scene_truth.json`은 새 제어기의 입력으로 읽지 않는다.

별도 BASE 경로는 `--path path.json`으로 전달한다. JSON 필드는 구 중심 `centres_m: Nx3`, 바깥쪽 법선 prior `normals: Nx3`이다. 이 경우에도 가상 세션·탐침 파지·HOME 시작 조건은 동일하다.

## 출력과 확인

- `/dsr01/surface_follow/state`: 실제 탐침 TCP / 힘 / stream 상태, `std_msgs/String` JSON.
- `/dsr01/surface_follow/status`: phase, 진행 거리, 힘, 유효 점 수.
- `/dsr01/surface_follow/points`: 실제 측정점 `PointCloud2`, frame `base_link`, m 단위.
- `repair_car/pointcloud/<시각>_<방법>.plt`: `x_m y_m z_m normal_force_n` 네 열의 공백 구분 ASCII.

PointCloud2는 기본 0.5 mm 간격으로, TRACK 중 힘 오차·과부하 위험·정렬 오차가 허용 범위인 점만 기록한다. 1.5 N 목표에 1.5 N 이하 일괄 제외 기준을 적용하지 않는다. RViz는 **컨테이너 ROS 그래프에 접근 가능한 환경**에서 Fixed Frame=`base_link`, PointCloud2 Topic=`/dsr01/surface_follow/points`로 설정한다. native RViz의 별도 domain 129로 이 점구름을 넘기는 relay는 이번 구현에 포함하지 않았다.

## 검증 상태와 한계

오프라인 계약 검사 6개(힘/이동 부호, 포화, 과부하 전이, 소유권·만료·watchdog, TCP 접촉점, 모멘트 기준점 이동), 문법 검사, 정상 sedan 보닛 경로 생성과 `--plan-only`를 확인했다. 캐시된 실제 URDF에서 ProbeTCP와 link_6 변환도 확인했다. Gazebo 접촉 추종 및 Docker colcon 빌드는 이번 작업에서 실행하지 않았다. 2D 모델의 성능 수치를 이 구현의 검증 수치로 사용하면 안 된다.

현재 경로 prior는 기존 상면 높이 모델이다. 단일 측정 선만으로 3D 법선 전체를 독립적으로 추정하지 않는다. 수직면용 mesh 경로 생성, RG2/탐침 전체의 사전 충돌 검사, 특이점 회피, 물리 파라미터 식별은 별도 작업이다. 관절 제한과 추적 오차 제한이 이를 대신하지 않는다. 8 N 측정 보호도 짧은 충격을 놓칠 수 있으며 로봇 보호정지 인증이나 오버슈트 제거 보장을 뜻하지 않는다.
