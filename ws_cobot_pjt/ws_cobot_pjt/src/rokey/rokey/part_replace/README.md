# part_replace — 보닛·문 분해·조립

펜던트에서 만든 보닛 분해·조립 DRL 을 DSR_ROBOT2 파이썬으로 옮긴 것. 속도는 DRL 기본값의 **20%** (`SPEED_SCALE = 0.2`).

**이 폴더가 유일한 실행본**이에요 (2026-11-21 정리: 저장소 루트 `part_replace/` 와 저장소 밖 복사본은 지움, 이전 내용은 git 기록).
웹 수리는 `web_control → repair_manager → replace_parts → python -m rokey.part_replace.bonnet_replace` 로 이 파일을 실행하고,
`rokey` 는 symlink 설치라 좌표를 고치면 다시 빌드하지 않아도 다음 수리부터 반영돼요.

| 파일 | 내용 |
|---|---|
| `bonnet_replace.py` | `disassemble()`, `assemble()` (다른 코드에서 import 가능) |
| `drl/m0609_bonnet.drl` | 원본 분해 DRL |
| `drl/m0609_bonnet_assembly.drl` | 원본 조립 DRL |
| `door_replace.py` | 왼쪽 문: `disassemble_left()`, `assemble_left()` (아래 "문") |
| `drl/m0609_door.drl` | 원본 문 분해 DRL (13~49줄 왼쪽 문 → door_replace.py, 51~81줄 오른쪽 문은 아직 이식 안 함) |
| `drl/m0609_door2.drl` | 원본 문 조립 DRL (왼쪽 문만) |

## 상태
- 가상 로봇: 분해·조립 확인 완료 (2026-11-20)
- 실물: 시험 중. 좌표 일부는 현장에서 조정한 값 (아래 "원본 DRL 과 다른 곳")

## 실행
```bash
# 1) 드라이버 켜기 (터미널 1)
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual                      # 가상
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<로봇 IP>          # 실물

# 2) 실행 (터미널 2) — 웹 없이 단독으로
source ~/cobot1_ws/ws_cobot_pjt/ws_dsr/install/setup.bash
source ~/cobot1_ws/ws_cobot_pjt/rokey-b3-project1/ws_cobot_pjt/ws_cobot_pjt/install/setup.bash
ros2 run rokey bonnet_replace disassembly|assembly|all [--virtual] [--until N]
```
웹에서는 자동 모드(실차: 스캔 → 판정 → 교체)나 수동 모드(보닛 지정)로 수리를 시작하면 분해 → 조립 순서로 실행돼요.
- `--virtual`: Tool/TCP 설정·확인을 건너뜀 (가상 로봇에는 이 이름이 없음). 실물에서는 쓰지 말 것.
- `--until N`: N 단계까지만 하고 멈춤 (분해 1~9, 조립 1~8). `all` 과는 같이 못 씀.
  예) `ros2 run rokey bonnet_replace assembly --until 5` → 장착 자세까지 가서 보닛을 잡은 채 멈춤.
  멈출 때 켜져 있는 힘·순응만 해제하고, 홈 이동은 하지 않음. 이어서 하는 기능은 없음 (복귀는 펜던트로).

다른 코드에서 쓸 때는 `DR_init` 의 id/model/node 를 먼저 설정한 뒤 `disassemble(until=None)`, `assemble(until=None)` 을 부름.
속도 설정(`set_velj` 등)은 DSR_ROBOT2 전역값이라 호출 뒤 다른 동작의 기본 속도도 바뀜.

## 속도
| 항목 | DRL | 이 코드 (× 0.2) |
|---|---|---|
| velj / accj | 60 / 100 | 12 / 20 |
| velx / accx | (250, 80.625) / (1000, 322.5) | (50, 16.125) / (200, 64.5) |
| 분해 힘 제어 이동 2개 (`time=`) | 9.33초 | 9.33 / 0.2 = **46.65초** (`T_FORCE_MOVE`) |

## Tool / TCP
`"Tool Weight v2"` / `"GripperDA_v1_probe"` — DRL 을 티칭할 때 펜던트에서 선택했던 것과 같아야 함.
시작할 때 `set_tool`/`set_tcp` 를 부르고 (실물에서는 제대로 적용돼도 -1 을 돌려줘서 반환값은 무시),
`get_tool`/`get_tcp` 이름이 다르면 움직이지 않고 종료.

## 그리퍼
- `grip_close()` = DO2 OFF → DO1 ON
- `grip_open()` = DO1 OFF → DO2 ON
- 그리퍼 동작 뒤 모두 `wait(1.0)` (조립 25줄은 원래 있던 wait 사용)

## 원본 DRL 과 다른 곳
**안전 (코드 주석 "원본과 다름: 안전")**
- 조립 시작: 홈 다음, 부품으로 내려가기 전에 `grip_open()` + `wait(1.0)`
- 조립 끝: 보닛 틈에서 바로 movej 하지 않고 48줄 자세로 movel 후퇴 → z +50 → movej 홈

**실물 시험 중 조정한 좌표**
| 위치 | 원본 DRL | 지금 코드 |
|---|---|---|
| 분해 27~29줄 보닛 앞 접근 | (572.25, 2.85, 127.70) → (592.25, 2.85, 127.70) | (572.25, 2.85, 148.70) → (587.25, 2.85, 138.70) → (593.25, 2.85, 129.70) |
| 분해 48줄 빼내기 | 상대 (-20, 0, +2) | 상대 (-8, 0, 0) → (-5, 0, +5) |
| 조립 15·17·19줄 새 보닛 잡기 | Y 225.00 | Y **223.00** (잡는 위치 -2mm → 장착 자세에서 보닛이 base Y +2mm) |
| 조립 36줄 순응 이동 | 상대 (+10, 0, 0) | 상대 (+10, 0, +1) |
| 조립 38줄 순응 회전 | 상대 ry -10 | 상대 ry **-20** |

**설치된 DSR_ROBOT2 에 맞춘 것**
- `movel(app_type=DR_MV_APP_NONE)`, `set_velx(..., DR_OFF)` 인자는 없어서 뺌 (APP_NONE 은 동작 차이 없음)
- DSR 서비스 연결을 먼저 기다리고 (최대 10초), 명령이 실패(-1)하면 다음 줄로 가지 않고 멈춤

## 안전
- Ctrl+C 또는 에러 → `move_stop`(DR_QSTOP) → 힘 해제 → 순응 해제 → 멈춘 단계 로그. **자동 홈 복귀 없음** (보닛을 든 채 관절 이동하면 차와 부딪힐 수 있음 → 복귀는 펜던트로).
- 드라이버가 응답하지 않으면 Ctrl+C 가 먹지 않음 → 그 터미널에서 **Ctrl+\\** 로 종료. 로봇은 펜던트 비상정지로 멈출 것.
- 다른 터미널에서 소프트웨어 정지 (펜던트 비상정지 버튼이 우선):
  ```bash
  ros2 service call /dsr01/dsr_controller2/motion/move_stop dsr_msgs2/srv/MoveStop "{stop_mode: 1}"
  ```
- 위험 구간은 실행 로그에 "주의" 로 표시됨 (자세가 크게 바뀌는 직선 이동, 힘 감지 없는 하강 등).
- 분해 5~6단계(각 46.65초)는 +x 힘 3N 제어 중 → 닿는 물체가 없으면 +x 로 밀려 나감. **보닛 없이 분해를 돌리지 말 것.**

## 실물 실행 전 점검
- [ ] 분해: **탐침을 잡은 상태로 시작** (첫 동작이 탐침 거치대에 내려놓기)
- [ ] 조립은 탐침을 다시 집지 않고 끝남
- [ ] 탐침 거치대 (350, -224.5), 헌 보닛 받침 (354.6, 224.6), 새 보닛 (250.9~259.3, 223) 위치가 티칭 때와 같은지
- [ ] 로봇을 홈 (0, 0, 90, 0, 90, 0) 근처에 두고 시작 (첫 동작이 현재 위치 → 홈 movej)
- [ ] 펜던트 Tool/TCP 등록 이름 확인, 펜던트 속도 슬라이더 낮게
- [ ] 비상정지 버튼 옆에 사람 대기, 로봇 작업 범위 안 비우기
- [ ] 처음엔 `disassembly` / `assembly` 를 따로 실행 (`all` 은 둘 다 확인한 뒤)
- [ ] 중간에 멈췄으면 바로 다시 실행하지 말고, 펜던트로 안전한 곳까지 뺀 뒤 보닛·탐침을 정리하고 시작

## 왼쪽 문 (`door_replace.py`)
보닛과 같은 틀(Tool/TCP 확인, 속도 20%, 반환값 확인, 실패 시 정리, 자동 홈 복귀 없음)을 `bonnet_replace.py` 에서 가져다 씀.
웹 수리(`replace_parts.TAUGHT`)에는 아직 연결 안 함 — 단독 실행만.
```bash
ros2 run rokey door_replace disassembly           # 왼쪽 문 분해
ros2 run rokey door_replace assembly              # 왼쪽 문 조립
ros2 run rokey door_replace all                   # 왼쪽 문 분해 → 조립
ros2 run rokey door_replace disassembly --until 4
```
- 단계 수: 왼문 분해 7, 왼문 조립 7. `--until` 은 `all` 과 같이 못 씀.
- DRL 에서 `vel=`/`acc=` 를 직접 준 movel 2개(조립 32·50줄): 직선 속도(1, 10 mm/s)는 그대로, 회전 속도·가속도는 × 0.2.
- 좌표는 DRL 그대로 (자동 대조로 확인). 원본과 다른 곳:
  - 조립 시작: **원본과 다름: 안전** — 원본은 현재 위치에서 바로 13줄 movel. 홈 movej → 그리퍼 열기 후 시작.
  - 그리퍼 동작 뒤 `wait(1.0)` 추가 (조립의 원래 `wait(2.00)`, `wait(0.50)` 은 그대로).

**실행 전 점검 (원본 DRL 그대로라 확인할 것)**
- [ ] 분해는 그리퍼를 열거나 닫지 않고 시작해서 3단계에서 그 상태 그대로 문을 밀어 냄 → **탐침을 잡고 있으면 안 됨** (펜던트로 그리퍼를 연 상태로 시작)
- [ ] 왼쪽 문은 헌 문 거치대 (344.8, 264.5)에 놓음
- [ ] 분해·조립 모두 끝에 홈 복귀 없음
- [ ] 새 문 위치 (440.7, 239.2, z 2.08)가 티칭 때와 같은지 (힘 감지 없이 z 2.08까지 하강)
- [ ] DRL 에 Tool/TCP 설정이 없어서 보닛과 같은 `Tool Weight v2` / `GripperDA_v1_probe` 로 가정함 → 문 티칭 때 펜던트에서 다른 TCP 였다면 좌표가 어긋남
