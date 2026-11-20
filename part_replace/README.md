# part_replace — 보닛 분해·조립

펜던트에서 만든 보닛 분해·조립 DRL 을 DSR_ROBOT2 파이썬으로 옮긴 것. 속도는 DRL 기본값의 **20%** (`SPEED_SCALE = 0.2`).

| 파일 | 내용 |
|---|---|
| `bonnet_replace.py` | `disassemble()`, `assemble()` (다른 코드에서 import 가능) |
| `drl/m0609_bonnet.drl` | 원본 분해 DRL |
| `drl/m0609_bonnet_assembly.drl` | 원본 조립 DRL |

## 상태
- 가상 로봇: 분해·조립 확인 완료 (2026-11-20)
- **실물: 미확인**

## 실행
```bash
# 1) 드라이버 켜기 (터미널 1)
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual                      # 가상
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<로봇 IP>          # 실물

# 2) 실행 (터미널 2)
source <ws_dsr>/install/setup.bash
cd part_replace
python3 bonnet_replace.py disassembly|assembly|all [--virtual]
```
`--virtual`: Tool/TCP 설정·확인을 건너뜀 (가상 로봇에는 이 이름이 없음). 실물에서는 쓰지 말 것.

## Tool / TCP
`"Tool Weight v2"` / `"GripperDA_v1_probe"` — DRL 을 티칭할 때 펜던트에서 선택했던 것과 같아야 함.
시작할 때 `set_tool`/`set_tcp` 후 `get_tool`/`get_tcp` 로 확인하고, 다르면 움직이지 않고 종료.

## 그리퍼
- `grip_close()` = DO1 ON, DO2 OFF
- `grip_open()` = DO1 OFF, DO2 ON

## 원본 DRL 과 다른 곳
- 조립 시작: 홈 다음, 부품으로 내려가기 전에 `grip_open()` + `wait(1.0)` (코드 주석 "원본과 다름: 안전")
- 조립 끝: 보닛 틈에서 바로 movej 하지 않고 48줄 자세로 movel 후퇴 → z +50 → movej 홈 (코드 주석 "원본과 다름: 안전")
- 그리퍼 동작 뒤 모두 `wait(1.0)` (조립 25줄은 원래 있던 wait 사용)
- 그 밖: 설치된 DSR_ROBOT2 에 없는 `movel(app_type=...)`, `set_velx(..., DR_OFF)` 인자는 뺌. DSR 서비스 연결을 먼저 기다리고, 명령이 실패(-1)하면 멈춤.

## 안전
- Ctrl+C 또는 에러 → `move_stop`(DR_QSTOP) → 힘 해제 → 순응 해제 → 멈춘 단계 로그. **자동 홈 복귀 없음** (보닛을 든 채 관절 이동하면 차와 부딪힐 수 있음 → 복귀는 펜던트로).
- 다른 터미널에서 비상 정지 (소프트웨어 정지, 펜던트 비상정지 버튼이 우선):
  ```bash
  ros2 service call /dsr01/dsr_controller2/motion/move_stop dsr_msgs2/srv/MoveStop "{stop_mode: 1}"
  ```
- 위험 구간은 실행 로그에 "주의" 로 표시됨 (자세가 크게 바뀌는 직선 이동, 힘 감지 없는 하강 등).

## 실물 실행 전 점검
- [ ] 분해: **탐침을 잡은 상태로 시작** (첫 동작이 탐침 거치대에 내려놓기)
- [ ] 조립은 탐침을 다시 집지 않고 끝남
- [ ] 탐침 거치대 (350, -224.5), 헌 보닛 받침 (354.6, 224.6), 새 보닛 (250.9~259.3, 225) 위치가 티칭 때와 같은지
- [ ] 펜던트 Tool/TCP 등록 이름 확인, 펜던트 속도 슬라이더 낮게
- [ ] 비상정지 버튼 옆에 사람 대기, 로봇 작업 범위 안 비우기
- [ ] 처음엔 `disassembly` / `assembly` 를 따로 실행 (`all` 은 둘 다 확인한 뒤)
