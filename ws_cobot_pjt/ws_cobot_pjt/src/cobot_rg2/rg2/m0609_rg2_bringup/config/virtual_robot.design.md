TCP·Tool 설정의 입력 원본은 실제 티치펜던트에 등록한 수치이며, 저장 위치는 같은 폴더의 `virtual_robot.yaml`이다. virtual bringup이 `dsr_controller2` 활성화 이후 자동 등록한다.

```text
티치펜던트 TCP / Tool Weight 설정
  → virtual_robot.yaml (이름별 수치 저장)
  → cobot_virtual.profiles.VirtualProfileLoader (virtual 모드 전용)
  → tcp/config_create_tcp · tool/config_create_tool
  → active 항목의 기본 이름 선택 → 실행 코드의 set_tcp(name) · set_tool(name)
  → get_tcp() · get_tool()로 선택된 이름 확인
```

- `tcps.<이름>.pos`: TCP 좌표 6개. 위치는 mm, 자세는 컨트롤러의 각도 규약을 그대로 사용한다. URDF의 rpy 값을 그대로 복사하지 않는다.
- `tools.<이름>.weight`, `cog`, `inertia`: 무게(kg), 무게중심 3개(mm), 관성 6개(API 규약). 관성 성분 순서는 Ixx, Iyy, Izz, Ixy, Ixz, Iyz이다.
- `Tool Weight`와 `Tool Weight_1`은 사용자 지정에 따라 YAML 앵커로 같은 설정을 공유하며, 제어기에는 두 이름으로 각각 등록한다.
- 위치·무게·무게중심은 필수이다. 관성만 `null`을 허용하며 미측정 상태를 뜻한다. 이 경우 등록 API에는 제조사 예제와 같이 0 배열을 전달한다. 실제 측정 관성이 0이라는 의미는 아니다.
- `active.tcp`와 `active.tool`은 시작 시 선택할 기본 이름이다. 작업 코드에서 등록된 다른 이름을 선택할 수 있다.

`scripts/virtual_profile_loader.py`가 로더를 실행한다. bringup의 `virtual_profile_file` 인자는 기본적으로 이 패키지의 `config/virtual_robot.yaml`을 가리킨다. virtual 모드에서만 실행하고, 로더도 `get_robot_system()`이 virtual인지 확인한다. ROS 서비스 경로는 로봇 namespace 아래의 `dsr_controller2/`를 사용한다.

로더는 입력 수치와 배열 길이를 확인하고 등록·선택 서비스 응답 및 선택된 이름을 확인한다. 실패하면 bringup을 종료한다. 기존 항목 삭제는 호출하지 않는다. 수치 변경 후에는 에뮬레이터를 새로 시작한다.

등록 API가 수동 모드를 요구하므로 virtual 제어기를 잠시 수동 모드로 전환하고, 등록·기본 이름 선택이 끝나면 원래 모드로 복귀한다.

API로 등록한 설정은 제어기 메모리에 저장되므로 에뮬레이터를 새로 시작할 때마다 YAML에서 다시 등록한다. [제조사 API 설명](https://manual.doosanrobotics.com/en/api/1.33/publish/cdrflex-add_tool)

real 모드에서는 이 로더를 실행하지 않고 실제 제어기의 설정을 사용한다. 패키지 내부에 설정 파일과 로더를 두어 Ubuntu에 workspace를 clone한 경우에도 같은 경로로 사용할 수 있게 한다.

이 프로필은 DRCF의 TCP/Tool 설정이다. Gazebo URDF 질량·관성 및 native 힘제어의 `tool0` 기준은 이 로더가 변경하지 않는다.
