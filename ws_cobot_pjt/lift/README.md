# turn_lift — 회전 리프트 차량 거치대 (독립 프로젝트)

4륜 TT 모터 차(TB6612) 위에 MG996R 시저 리프트를 얹고, 그 위에 시뮬레이션 차량(rokey `--car real`, `b3_project1/assembly.stl`)을 올립니다.
거치대가 **차량을 돌리고 올리고 내리므로**, 로봇은 탐침을 두 방향으로만 씁니다.

- **위(top)**: BASE −z로 누르며 윗면을 측정
- **정면(front)**: BASE +x로 누르며 로봇을 향한 면을 측정

이렇게 하면 형상·크랙 검출이 고정축 스캔(rokey `real_scan`)과 같은 방식이 됩니다.

이 디렉토리는 rokey 패키지·시뮬레이션과 **독립**입니다.
- 차량 메시를 읽기만 하고, rokey 코드는 import하지 않습니다.
- 필요한 것은 numpy와 PyYAML입니다(conda `cobot-native` 환경).

---

## 1. 가능성 검토 결과

`python -m turnlift report`와 `plan`의 결과입니다. 값 대부분은 실측 전 가정값이며(§6), 실측 후 다시 돌리면 갱신됩니다.

| 항목 | 계산 결과 | 판정 | 대책 |
|---|---|---|---|
| 질량 | 차 0.90(가정) + 리프트 0.17 + 어댑터판 0.08 + 차량 0.18(메시 부피×밀도×채움 0.25) = **1.32 kg** | — | 차량을 실측해 `vehicle.mass_kg`에 입력 |
| 높이 | 차량 바닥 136 mm(내림) → 198 mm(올림), 차량 꼭대기 최대 348 mm | — | — |
| 리프트 서보 (MG996R, 6 V) | 최악 각도 136°. 필요 토크와 연속 허용 4.99 kg·cm(정지 토크의 50 %) 대비 여유: 힘 0 N 1.85 kg·cm(**2.7배**), 1.5 N 2.77 kg·cm(**1.8배**), 8 N 6.77 kg·cm(**0.74배**, 정지 토크 11보다는 작음) | 1.5 N 스캔 가능, 8 N 연속 불가 | DSR 힘 가드를 위 방향 ≤ 5 N으로. 서보는 6 V 3 A 별도 전원 |
| 제자리 회전 (TT ×4, 스키드) | 회전 저항 0.57 N·m → 바퀴당 0.73 kg·cm(TT 정지 토크 0.8의 91 %). 90°/3 s에 필요한 출력은 **96 %**(펌웨어 35 %), 견인 여유 1.18 | **가장 큰 위험**: 회전이 멈추거나 각도가 들쭉날쭉할 수 있음 | ① 리프트 아래 회전판(lazy-susan 베어링 + 스텝/서보)으로 교체 ② 엔코더 달린 금속 기어모터(JGA25 등) ③ 바퀴 마찰 줄이기(측면 마찰 0.6 → 0.3이면 필요 토크 절반) |
| 방향 정확도 | 엔코더가 없어 시간(90° = 3 s)으로 회전 → 회전마다 각도·위치 오차 누적 | 형상 비교에 직접 영향 | 방향·스테이션마다 **기준면 터치 재정합**(어댑터판 모서리 2점 + 윗면 1점), 또는 IMU(자이로) |
| 버티기: 위 누름 | 8 N까지 전도 여유 23 mm 이상, 미끄럼·비틀림 없음 | OK | — |
| 버티기: 정면 누름 | 1.5 N: 전도 여유 37 mm, 미끄럼 여유 자유 정지(coast) 1.08 / **단락 제동 6.9**. 8 N: **넘어짐**(여유 −94 mm), 미끄럼 | 1.5 N은 제동하면 가능. 정면 힘 한계 **약 3.3 N** | 펌웨어 `BRAKE_HOLD`(구현함), 정면 힘 가드 ≤ 3 N, 필요하면 바퀴 고정 클램프 |
| 상판 유격 | 핀 3개 × 0.2 mm → 0.6 mm 밀림 + 0.2–0.4° 기울기 → 차량 꼭대기에서 최대 **1.6 mm** | 크랙보다 큰 편향일 수 있음 | 핀 구멍 공차 축소(+0.4 → +0.2), 한 방향 힘으로 예압된 상태에서만 측정, 재정합 |
| 리프트 하중 유지 | 원래 펌웨어는 종료 시 서보 PWM을 끔 → 차량 무게와 탐침 힘에 리프트가 내려앉음 | 측정 중 상주가 필요 | `--serve` 상주 모드 구현(PWM 유지) |
| 회전 반경 | 중심에서 136 mm, 로봇 HOME 탐침 끝까지 192 mm | OK | — |
| 탐침 도달 | 탐침 272 mm + 손목 121 mm = **393 mm**가 탐침 끝 뒤로 수평으로 뻗어야 정면을 누를 수 있음. 회전 중심 560 mm 하나로는 정면이 거의 닿지 않고, 720 mm에서는 윗면이 닿지 않음(아래 표) | 한 자리로는 불가 | **두 스테이션**: 위 측정 560 mm, 정면 측정 740 mm(차가 heading 0에서 180 mm 후진) |

### 탐침 도달과 스테이션

**스테이션 결정용 민감도.**
- 팔 도달과 공구–베이스 기둥만 검사했고, 차량·거치대 간섭은 제외했습니다.
- 샘플 4 mm 기준입니다.
- 위는 리프트를 내린 상태, 정면은 올린 상태입니다.

| 회전 중심 x | 위: 보닛 | 위: 몸체 | 정면: 범퍼 | 정면: 문 | 정면: 몸체 |
|---|---|---|---|---|---|
| 560 mm | 990/1308 | 2455/4580 | 0/115 | 201/893 | 338/1875 |
| 640 mm | 340/1308 | 579/4580 | 0/115 | 818/893 | 1471/1875 |
| 720 mm | 23/1308 | 18/4580 | 115/115 | 893/893 | 1875/1875 |

**전체 계획 결과.**
- 기준: `python -m turnlift plan`, 스테이션 위 560 / 정면 700, 4방향 × 리프트 2단, 차량·거치대·탐침 거치대 간섭을 포함, 샘플 1.5 mm. 값은 닿는 샘플 / 보이는 샘플입니다.
- 700 mm에서는 범퍼가 낮고 맨 앞이라 공구 뒤끝(손목부)이 로봇 베이스 기둥에 걸립니다.
  - 같은 간섭 검사로 정면 스테이션을 **740 mm**로 두면 범퍼가 모두 닿습니다(리프트 올림): 앞 범퍼 342/342, 뒤 범퍼 419/419.
  - 그래서 정면 스테이션을 **740 mm로 확정**했습니다.
  - 문·몸체 값은 700 mm에서 계산한 것입니다. 정면의 제약은 "너무 가까움"이므로 740 mm에서도 줄지 않을 것으로 보지만, 실측값을 넣은 뒤 `plan`을 다시 돌려 확인하세요.

| 부품 | 한 자리(560)일 때 | 두 스테이션 (560 / 700) |
|---|---|---|
| 범퍼 | 0 / 891 | 45 / 891 (700 mm) → 앞·뒤 정면 761 / 761 (740 mm) |
| 보닛 | 2152 / 2578 | 2152 / 2578 (83 %) |
| 문 | 1054 / 5754 | **5589 / 5754 (97 %)** |
| 몸체 | 8327 / 19790 | **15584 / 19790 (79 %)** |

- 리프트의 역할
  - 위 측정은 **내림**이 유리합니다. 손목이 탐침 끝보다 393 mm 위에 있어야 하므로, 차량이 낮을수록 멀리 닿습니다.
  - 정면 측정은 **올림**이 유리합니다. 손목이 어깨(134.5 mm)에서 떨어져 팔이 접히는 한계를 피합니다.
- 상태별 상세는 `out/plan.log`와 `out/plan.json`에 있습니다.

**결론.**
- 구성 자체는 가능합니다. 조건은 다음과 같습니다.
  1. 두 스테이션을 사용합니다.
  2. 정면 힘 ≤ 3 N이고 단락 제동을 겁니다.
  3. 리프트 서보가 상주합니다(PWM 유지).
  4. 방향·스테이션마다 재정합합니다.
- 제작 전에 결정할 위험은 **TT 모터 스키드 회전의 토크 부족**입니다. 회전판이나 엔코더 모터로 바꾸는 것을 권장합니다.

---

## 2. 계획

| 단계 | 내용 | 상태 |
|---|---|---|
| 1. 오프라인 모델 | 질량, 리프트 토크, 회전 토크, 버티기, 유격, 도달, 스테이션, Gazebo 모델 생성 (`turnlift/`) | **완료** |
| 2. 펌웨어 | 단락 제동 유지, `--serve` TCP 상주(heading / lift / station / drive), 높이(mm) 지정(`lift_table.json`) | **완료** (실기 미시험) |
| 3. Gazebo 단독 검증 | `out/worlds/turn_lift.sdf`로 회전 정지 여부, 리프트 상승, 탐침 힘에 대한 전도·미끄럼 확인 (§5) | 준비 완료, Docker 사용 가능할 때 |
| 4. 제작·실측 | 어댑터판 출력, 조립(§3), `measure` 값 실측 → `report`·`plan` 재실행, 'c' 회전 보정, `DRIVE_MM_S` 실측 | 대기 |
| 5. rokey 통합 | `real_scan`에 축 `top`(−z 누름)·`front`(+x 누름) 추가, 거치대 TCP 클라이언트, 재정합 터치, 스테이션·방향별 결과를 차량 좌표로 합치기 | 대기 |

---

## 3. 조립

1. **리프트**: [hardware/ASSEMBLY.md](hardware/ASSEMBLY.md) 순서대로 조립합니다. 부품 STL은 `hardware/stl/`(17개)에 있습니다.
2. **차 위에 리프트 고정**
   - 리프트 중심을 차의 회전 중심(앞뒤·좌우 바퀴 중심의 가운데)에 맞춥니다.
   - 리프트 길이(120 mm)를 차 앞뒤 방향으로 둡니다(`lift.mount_mm`, `yaw_on_car_deg`).
   - 중심이 어긋나면 회전할 때 차량이 원을 그립니다.
3. **어댑터판**
   - 230 × 140 × 4 mm PLA, 채움 30 %.
   - 리프트 상판(120 × 80)에 나사나 양면테이프로 고정하고, 차량 바퀴를 올립니다.
   - 차량 앞을 차 앞과 같은 방향으로 둡니다(`vehicle_yaw_on_car_deg: 0`). 차량은 바퀴 둘레 스토퍼나 양면테이프로 고정합니다.
4. **전원**
   - 서보: 6 V 3 A를 별도로 씁니다.
   - 모터: TB6612 VM에 6 V를 넣습니다.
   - 라즈베리파이 GND를 모두 공통으로 묶습니다.
5. **배치**
   - 로봇 BASE 기준 (560, 0)에 차를 놓고, 차 앞을 로봇 쪽(−x)으로 향합니다(heading 0). 이 자리가 위 측정 스테이션입니다.
   - 정면 측정 스테이션은 (740, 0)입니다.
   - 두 자리에 멈춤 블록(L자)을 두면 직진 오차를 줄일 수 있습니다.

---

## 4. 작동 순서

```
라즈베리파이: sudo python3 firmware/car_turn_lift.py --serve 5005     # 상주, 리프트 PWM 유지

[위 스테이션, 560 mm]  lift down
  heading 0 → 90 → 180 → 270 마다: 재정합 터치 → 로봇이 −z로 눌러 윗면 스캔
station front           # heading 0으로 돌린 뒤 180 mm 후진
[정면 스테이션, 740 mm] lift up
  heading 0(앞 범퍼) → 90(문) → 180(뒤 범퍼) → 270(반대쪽 문) 마다: 재정합 → 로봇이 +x로 눌러 정면 스캔
quit                    # 위 스테이션으로 돌아와 0°, 리프트 내림
```

- **TCP 명령**(한 줄 명령, 한 줄 JSON 응답)
  - `heading 90`, `turn -90`
  - `lift up` / `lift down` / `lift 100`(상판 높이 mm)
  - `station top` / `station front`, `drive -180`
  - `status`, `quit`
- **응답 예**: `{"ok": true, "heading": 90.0, "station": "front", "lift_deg": 32.0, "lift_mm": 118.7, "quit": false}`
- **heading 규칙**
  - 위에서 볼 때 시계 방향이 +입니다(키 `6` = 90° = 오른쪽 회전).
  - BASE에서 차 앞의 yaw = 180° − heading입니다.
- **키보드 모드**(인자 없이 실행)는 원래대로 동작합니다: 5/6/7/8, 3/4, 1/2, c, q.

---

## 5. 시뮬레이션 준비 (Gazebo Harmonic, 아직 실행 안 함)

`python -m turnlift sdf` 명령으로 다음 두 파일을 만듭니다.
- `out/models/turn_lift/`: `model.sdf`, `model.config`, `meshes/vehicle.stl`
- `out/worlds/turn_lift.sdf`

| 요소 | SDF | 값의 근거 |
|---|---|---|
| chassis | 차 + 리프트 고정부 질량·관성, 박스 형상 | `car`, 리프트 부품 질량 |
| 바퀴 ×4 | revolute, effort **0.0785 N·m**, velocity **20.9 rad/s**, 마찰 mu 0.8 / mu2 0.6 | TT 정지 토크 0.8 kg·cm, 무부하 200 rpm |
| 리프트 | prismatic 0–62.2 mm, effort **8.94 N**, velocity 3.9 mm/s | MG996R 연속 허용 토크 × 효율 ÷ 최악 전달비 |
| lift_top | 상판 + 어댑터 + 차량 메시, 질량 0.31 kg | 메시 질량 특성 |
| 플러그인 | DiffDrive(`/turn_lift/cmd_vel`), JointPositionController(`/turn_lift/lift`, 힘 제한), JointStatePublisher, ApplyLinkWrench(월드) | — |

실행과 확인은 Docker가 비었을 때 합니다. 네이티브 `cobot-native` 환경에도 gz 8이 있습니다.

```bash
gz sim out/worlds/turn_lift.sdf
gz topic -t /turn_lift/cmd_vel -m gz.msgs.Twist -p 'angular: {z: 0.524}'        # 90°/3 s 회전: 멈추는지(토크 부족) 확인
gz topic -t /turn_lift/lift -m gz.msgs.Double -p 'data: 0.062'                  # 리프트 올림
gz topic -t /world/turn_lift/wrench -m gz.msgs.EntityWrench \
  -p 'entity: {name: "turn_lift::lift_top", type: LINK}, wrench: {force: {x: 3.0}}'   # 탐침 힘: 전도·미끄럼
```

- 바퀴 토크 제한이 TT 정지 토크와 같으므로, 회전 명령이 실제로 도는지가 §1 "제자리 회전" 판정을 확인하는 시험입니다.
- 단락 제동은 SDF에 없습니다. 미끄럼 시험은 정지 상태(DiffDrive 0 명령)로 근사합니다.

---

## 6. 실측해서 넣을 값 (`config/device.yaml`의 `measure`)

- `car`
  - 질량, 축거, 윤거, 차체 크기, 무게중심 높이
  - 바퀴–테이블 마찰(앞뒤·옆): 경사 시험으로 측정
  - 기어박스 역구동 토크
- `vehicle.mass_kg`: 실제 차량 무게. null이면 메시로 추정합니다.
- `motors.wheel`: 실제 전압에서의 TT 정지 토크와 무부하 rpm
- 펌웨어
  - `TURN_DUTY`: 'c' 보정으로 정합니다.
  - `DRIVE_MM_S`: 직진 속도
  - `STATION_MM`: 스테이션 간격. `scan.directions`와 맞춥니다.
- `lift.pin_play_mm`: 출력한 핀 구멍의 실제 유격

---

## 7. 사용법

```bash
conda activate cobot-native
cd lift
python -m turnlift report   # 질량·리프트 토크·회전 토크·버티기·유격·회전 반경 → out/report.json
python -m turnlift plan     # 스테이션 × 방향 × heading × 리프트별 도달 커버리지 → out/plan.json (수 분)
python -m turnlift sdf      # Gazebo 모델·월드 → out/models/turn_lift, out/worlds/turn_lift.sdf
python -m turnlift table    # 서보 각도 ↔ 상판 높이 표 → firmware/lift_table.json
python -m turnlift report --config other.yaml
```

---

## 8. 코드 관계

```
lift/
├── README.md
├── config/device.yaml                 모든 치수·질량·모터·로봇·탐침·스캔(스테이션) 설정
├── turnlift/                          python -m turnlift (numpy, PyYAML)
│   ├── __main__.py                    report | plan | sdf | table
│   │   ├── device.TurnLift            프레임(car_pose / vehicle_pose, 스테이션 centre), 박스 형상, 질량·무게중심, 회전 반경
│   │   │   ├── scissor.ScissorLift    서보각 → 크랭크각 → 상판 높이, 전달비 dz/dθ, 부품 질량
│   │   │   │   └── hardware/source/compact/lc.py, lift3d/   리프트 기하 (import, 복사하지 않음)
│   │   │   ├── vehicle.Vehicle        차량 STL 정렬·축척, 부품 분할(범퍼/보닛/문/몸체), 문 창 제외, 질량 특성
│   │   │   │   └── ../ws_cobot_pjt/src/rokey/meshes/vehicles/b3_project1/assembly.stl  (읽기만)
│   │   │   └── mesh                   STL 입출력, 법선, 질량 특성(부피·중심·관성), 면적 균일 샘플링, 박스 삼각형
│   │   ├── loads                      LiftTorque · Support · TurnTorque · Holding · Play
│   │   │   └── motors                 Servo(MG996R) · GearMotor(TT) 토크–속도 모델
│   │   ├── plan.ScanPlan              방향별 가장 바깥 면 → 기울기 → 팔 도달·간섭 → 부품별 커버리지
│   │   │   └── reach                  SimpleArm(M0609 단순 IK 캡슐) · Clearance(공구 원통·팔 캡슐·테이블·베이스·탐침 거치대)
│   │   └── sdf.SdfExport              Gazebo 모델(바퀴·리프트 관절의 토크 제한) + 월드
│   └── config.py                      ROOT, OUT, load()
├── firmware/
│   ├── car_turn_lift.py               라즈베리파이: 키보드 모드 / --serve TCP (heading, turn, lift, station, drive, status)
│   └── lift_table.json                ← table (서보 각도 ↔ 상판 높이)
├── hardware/                          리프트 하드웨어 (원래 소형리프트_120x80)
│   ├── ASSEMBLY.md                    리프트 조립 설명
│   ├── source/compact/{lc,check_c,render_c}.py, source/lift3d/{gen,meshlib,check,render}.py
│   ├── stl/                           출력 부품 17개
│   └── 부품일람_120x80.png, preview_c.png
└── out/                               생성물: report/plan/sdf/table.json, plan.log, models/turn_lift/, worlds/turn_lift.sdf
```

---

## 9. rokey 통합 메모 (나중)

- **축**: `real_scan`의 고정축에 `top`(BASE +z 법선, −z로 누름)과 `front`(BASE −x 법선, +x로 누름)만 추가합니다. 힘·순응 제어와 측정 전용 원칙은 그대로입니다.
- **장면**: rokey `--car real`의 고정 거치대 대신, 이 모델(`out/models/turn_lift`)을 스테이션 위치에 둡니다.
- **좌표 합치기**
  - 각 상태의 측정점을 `TurnLift.vehicle_pose(heading, servo, centre)`의 역변환으로 차량 좌표에 모읍니다.
  - 재정합 터치로 구한 보정 변환을 곱합니다.
- **힘 가드**: 위 ≤ 5 N, 정면 ≤ 3 N(§1).
