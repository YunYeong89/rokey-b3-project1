# M0609 + RG2 기본 제어

`run_virtual.sh`를 기준으로 정리한 `rokey-b3-project1`의 ROS 배포본입니다.
`simple_move`의 관절·직선 이동과 RG2 그리퍼 제어를 포함합니다.
가상 컨트롤러는 Doosan DRCF, ROS 배포판은 Jazzy, 로봇 모델은 M0609입니다.

법선 벡터 추정, 순응·힘 제어 애플리케이션, 표면 스캔, 포인트클라우드,
탐침·거치대·도넛 맵, 실측 calibration 파일은 배포 범위에서 제외했습니다.
벤더 SDK와 ROS 인터페이스의 일반 API 정의는 호환성을 위해 유지합니다.

Docker 이미지 파일을 별도로 공유하지 않습니다. 이 저장소를 받은 뒤
`Dockerfile`과 `compose.yaml`로 각 PC에서 이미지와 컨테이너를 생성합니다.
ROS 소스도 해당 PC의 워크스페이스에서 빌드합니다.

## 파일 구조와 실행 관계

저장소 루트 아래의 `ws_cobot_pjt/`가 이 문서의 작업 디렉토리입니다.
그 안의 `ws_cobot_pjt/`는 colcon 워크스페이스입니다.

```text
rokey-b3-project1/
├── README.md                      # 프로젝트 소개 및 ROS 문서 링크
├── web/                           # 기존 웹 프로젝트
└── ws_cobot_pjt/                   # 아래 명령을 실행하는 디렉토리
    ├── README.md
    ├── THIRD_PARTY.md
    ├── run_virtual.sh             # macOS native 실행 진입점
    ├── run_ubuntu.sh              # Ubuntu ROS 직접 실행 진입점(virtual)
    ├── build_workspace.sh         # 두 환경에서 공통으로 사용하는 colcon 빌드
    ├── Dockerfile
    ├── compose.yaml               # ROS 컨테이너 + DRCF 에뮬레이터
    ├── docker-entrypoint.sh
    ├── emulator/
    │   └── start.sh               # DRCF 및 API 프로세스 시작/종료
    ├── native/
    │   ├── environment.yml        # cobot-native Conda 환경
    │   ├── run_display.sh         # Conda → session.py
    │   ├── session.py             # Docker·Gazebo·RViz 세션 관리
    │   ├── model_export.py        # 공통 xacro → native URDF/SDF
    │   ├── displays.py            # Gazebo/RViz 관절 상태 표시
    │   ├── joint_interpolator.py  # 표시용 관절 상태 보간
    │   ├── ros_state_relay.py     # Docker ROS 상태 → host TCP 전달
    │   └── gazebo.config
    └── ws_cobot_pjt/
        └── src/
            ├── rokey/
            │   ├── setup.py      # simple_move = rokey.move:main
            │   └── rokey/move.py # HOME → 기본 이동 3회 → HOME
            └── cobot_rg2/
                ├── doosan-robot2/
                │   ├── dsr_common2/      # Python SDK·DRFL 헤더/라이브러리
                │   ├── dsr_msgs2/        # ROS 메시지·서비스
                │   ├── dsr_hardware2/    # DRCF 하드웨어 인터페이스
                │   ├── dsr_controller2/  # 이동 및 I/O 서비스
                │   └── dsr_description2/ # M0609 메시만 포함
                ├── onrobot-ros2/
                │   ├── onrobot_rg_msgs/
                │   ├── onrobot_rg_control/     # 실제 RG2 Modbus 드라이버
                │   └── onrobot_rg_description/# RG2 메시만 포함
                └── rg2/m0609_rg2_bringup/
                    ├── launch/bringup.launch.py
                    ├── config/virtual_robot.yaml
                    ├── cobot_virtual/profiles.py # 가상 Tool/TCP 등록
                    ├── scripts/gripper_virtual_node.py
                    ├── scripts/gripper_joint_state_publisher.py
                    ├── scripts/virtual_profile_loader.py
                    ├── xacro/                   # M0609 + ros2_control
                    ├── urdf/                    # RG2 결합
                    └── rviz/default.rviz
```

```text
native(macOS)
run_virtual.sh → native/session.py
  ├─ Docker DRCF ← Docker ros2_control ← simple_move / 그리퍼 서비스
  └─ Docker JointState → ros_state_relay(TCP) → native Gazebo + RViz

Ubuntu
run_ubuntu.sh → Docker DRCF ← Ubuntu ros2_control ← simple_move / 그리퍼 서비스
                                        └─ JointState → Ubuntu RViz
```

## 공통: 소스 받기와 빌드 구분

```bash
git clone --branch ros https://github.com/YunYeong89/rokey-b3-project1.git
cd rokey-b3-project1/ws_cobot_pjt
```

이미 저장소가 있으면 그 안의 `ws_cobot_pjt/`로 이동합니다.
아래 Docker 및 실행 명령은 모두 `Dockerfile`과 `compose.yaml`이 있는 이 디렉토리에서 실행합니다.

이 프로젝트에는 두 가지 빌드가 있습니다.

| 단계 | 입력 | 결과 | 필요한 환경 |
|---|---|---|---|
| Docker 이미지 빌드 | `Dockerfile`, ROS `package.xml`, `docker-entrypoint.sh` | 로컬 `rokey-ros:jazzy` 이미지 | macOS native |
| ROS 워크스페이스 빌드 | `ws_cobot_pjt/src/`의 소스 | `ws_cobot_pjt/build/`, `install/`, `log/` | 두 환경 모두 |

Docker 이미지는 ROS Jazzy, 컴파일 도구, `rosdep`으로 설치한 의존성과
그리퍼용 `pymodbus==3.6.9`를 담는 개발 환경입니다.
**이미지 빌드만으로 ROS 패키지가 빌드되는 것은 아닙니다.**
`compose.yaml`이 호스트의 `./ws_cobot_pjt`를 컨테이너의 `/workspace`에 연결하므로,
컨테이너에서 수행한 colcon 빌드 결과는 호스트에도 남습니다.

가상 DRCF는 Doosan의 공개 이미지를 내려받아 사용합니다. 첫 이미지 빌드 및
에뮬레이터 다운로드에는 인터넷 연결이 필요합니다. 자체 이미지 레지스트리 로그인은 필요하지 않습니다.

## 1. macOS native 실행

### 준비 및 최초 빌드

Docker Desktop과 Conda가 필요합니다. Docker Desktop을 실행한 상태에서 진행합니다.
Linux/amd64 DRCF 이미지는 Apple Silicon에서 Docker의 에뮬레이션으로 실행하며,
Gazebo와 RViz 창은 macOS에서 직접 실행합니다.

#### 1) Docker 사용 가능 여부 확인

```bash
docker version
docker compose version
docker compose --profile virtual config --quiet
```

`docker version`에 Server 정보가 나와야 합니다. 마지막 명령은 Compose 설정을
검사하며, 정상이라면 출력 없이 끝납니다.

#### 2) native 표시 환경 설치

```bash
conda env create -f native/environment.yml
```

이미 `cobot-native` 환경이 있으면 다음 명령으로 갱신합니다.

```bash
conda env update -f native/environment.yml
```

#### 3) ROS 개발용 Docker 이미지 빌드

```bash
docker compose build cobot
```

Compose의 `cobot` 서비스에 지정된 현재 디렉토리의 `Dockerfile`로
`rokey-ros:jazzy` 이미지를 생성합니다. Dockerfile은 다음 작업을 수행합니다.

1. `ros:jazzy-ros-base-noble` 베이스 이미지를 받습니다.
2. 컴파일 도구와 colcon, rosdep을 설치합니다.
3. `ws_cobot_pjt/src/`의 manifest를 읽어 ROS 의존성을 설치합니다.
4. 그리퍼 통신용 Python 의존성과 컨테이너 시작 스크립트를 설치합니다.

진행 로그를 자세히 보려면 아래 명령을 사용합니다.

```bash
docker compose --progress plain build cobot
```

완료 후 로컬 이미지가 생성되었는지 확인합니다.

```bash
docker image inspect rokey-ros:jazzy --format '{{.Id}}'
```

#### 4) 컨테이너에서 ROS 패키지 빌드

```bash
docker compose run --rm cobot /opt/rokey-native/build_workspace.sh /workspace
```

이 명령은 방금 만든 이미지로 임시 `cobot` 컨테이너를 생성하여 빌드합니다.
`build_workspace.sh`는 `/workspace`에서 아래 colcon 명령을 실행합니다.

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DDRCF_VER=2
```

끝나면 임시 컨테이너는 `--rm`에 의해 삭제됩니다. 다음 빌드 결과는 호스트에 남습니다.

```text
ws_cobot_pjt/                  # 현재 작업 디렉토리
└── ws_cobot_pjt/              # 컨테이너에서는 /workspace
    ├── src/                  # Git으로 받은 소스
    ├── build/                # 중간 빌드 결과
    ├── install/setup.bash    # 실행 환경 설정
    └── log/                  # colcon 로그
```

빌드 로그 마지막의 `Summary: 10 packages finished`를 확인합니다.
실행 파일 등록은 로봇을 구동하지 않고 다음과 같이 확인할 수 있습니다.

```bash
docker compose run --rm cobot bash -lc \
  'source /workspace/install/setup.bash && ros2 pkg executables rokey'
```

출력에 `rokey simple_move`가 있어야 합니다.

#### 5) DRCF 에뮬레이터 이미지 받기

```bash
docker compose --profile virtual pull emulator
```

이 명령은 `compose.yaml`에 지정된 Doosan 이미지와 digest를 사용합니다.
에뮬레이터 이미지를 직접 빌드하거나 이미지 파일로 공유할 필요는 없습니다.
여기까지는 이동·그리퍼 동작을 실행하지 않습니다.

### 터미널 1: 가상 세션 시작

```bash
./run_virtual.sh
```

`run_virtual.sh`가 Compose의 `cobot`·`emulator` 컨테이너를 생성·시작한 뒤
ROS 컨트롤러와 native Gazebo/RViz를 실행합니다.
`Virtual profiles ready` 로그가 나온 후 이동 및 그리퍼 명령을 실행합니다.
세션 시작만으로 `simple_move`를 자동 실행하지 않습니다.

다른 터미널에서 컨테이너 상태를 확인할 수 있습니다.

```bash
docker compose --profile virtual ps
```

### 터미널 2: simple_move

같은 `ws_cobot_pjt/` 디렉토리에서 실행합니다.

```bash
docker compose exec cobot bash -lc \
  'source /workspace/install/setup.bash && ros2 run rokey simple_move'
```

### 터미널 2: 그리퍼 열기 / 닫기

```bash
# 열기
docker compose exec cobot bash -lc \
  'source /workspace/install/setup.bash && ros2 service call /onrobot/sendCommand onrobot_rg_msgs/srv/SetCommand "{command: o}"'

# 닫기
docker compose exec cobot bash -lc \
  'source /workspace/install/setup.bash && ros2 service call /onrobot/sendCommand onrobot_rg_msgs/srv/SetCommand "{command: c}"'
```

가상 그리퍼는 `o`/`c` 및 부분 열림 각도(rad)를 받습니다.
컨트롤러의 가상 디지털 출력도 연결되어 있으며 1번은 닫기, 2번은 열기입니다.
`simple_move`와 그리퍼 명령은 각각 실행합니다.

### 종료와 로그

터미널 1에서 `Ctrl+C`로 세션과 해당 Compose 프로젝트의 컨테이너를 종료합니다.
필요하면 다음 명령으로 남은 컨테이너를 정리합니다.

```bash
docker compose --profile virtual down
```

실행 로그 및 생성된 모델은 `native/.cache/`에 저장되며 Git에서 제외됩니다.
`./run_virtual.sh --no-gui --duration 10`으로 화면 없이 제한 시간 동안 실행할 수도 있습니다.

native Gazebo는 관절 위치 표시용입니다. 중력은 0으로 설정하며 힘 센서·순응·법선 방향
힘 제어를 계산하지 않습니다. Docker ROS 도메인은 29, host 표시 도메인은 129이고,
두 환경의 상태는 DDS 대신 TCP relay로 연결합니다.

## 2. 일반 Ubuntu 실행

Ubuntu 24.04와 ROS 2 Jazzy가 설치된 환경을 기준으로 합니다.
ROS 노드와 RViz는 Ubuntu에서 직접 실행하고, 가상 DRCF만 Docker에서 실행합니다.
Conda와 native Gazebo는 사용하지 않습니다.

### 준비 및 빌드

Docker Engine/Compose 플러그인 및 ROS 2 Jazzy를 먼저 설치합니다.
아래 명령은 저장소의 `ws_cobot_pjt/` 디렉토리에서 실행합니다.
Ubuntu 직접 실행에서는 `cobot` 이미지를 빌드하지 않습니다.
컴파일 도구와 ROS 의존성을 Ubuntu에 설치하고, DRCF 에뮬레이터 이미지만 받습니다.

```bash
sudo apt update
sudo apt install -y build-essential python3-colcon-common-extensions python3-rosdep \
  python3-pymodbus python3-numpy
source /opt/ros/jazzy/setup.bash

# rosdep을 아직 초기화하지 않은 PC에서만 한 번 실행
sudo rosdep init

rosdep update --rosdistro jazzy
rosdep install --from-paths ws_cobot_pjt/src --ignore-src --rosdistro jazzy -y

# ROS 소스 빌드: 호스트의 ws_cobot_pjt/build, install, log 생성
./build_workspace.sh

# 가상 컨트롤러 이미지 다운로드
docker compose --profile virtual pull emulator
```

빌드는 `BUILD_TESTING=OFF`, `DRCF_VER=2`로 수행합니다. 테스트와 빌드 산출물은
배포 파일에 포함하지 않습니다. Doosan SDK의 Jazzy용 `libDRFL.a`는 x86_64/arm64
빌드에 필요한 벤더 입력 파일이므로 소스와 함께 포함합니다.

빌드 완료 후 다음 명령으로 실행 파일 등록을 확인합니다.

```bash
source ws_cobot_pjt/install/setup.bash
ros2 pkg executables rokey
```

`rokey simple_move`가 출력되면 다음 실행 단계로 진행합니다.

### 터미널 1: 가상 실행

```bash
./run_ubuntu.sh
```

호스트 ROS는 `127.0.0.1:12349`를 통해 DRCF에 연결합니다.
화면을 생략하려면 `./run_ubuntu.sh gui:=false`로 실행합니다.
`Virtual profiles ready` 로그 이후 다음 명령을 실행합니다.

### 터미널 2: 이동 및 그리퍼

```bash
source /opt/ros/jazzy/setup.bash
source ws_cobot_pjt/install/setup.bash
export ROS_DOMAIN_ID=29

ros2 run rokey simple_move

# 열기 / 닫기
ros2 service call /onrobot/sendCommand onrobot_rg_msgs/srv/SetCommand "{command: o}"
ros2 service call /onrobot/sendCommand onrobot_rg_msgs/srv/SetCommand "{command: c}"
```

터미널 1의 `Ctrl+C`로 ROS와 에뮬레이터를 종료합니다.
기본 DDS 도메인은 29입니다. 변경할 경우 ROS를 실행하는 모든 터미널에서 같은 값을 사용합니다.

### Ubuntu에서 실제 로봇 연결

가상 세션을 종료하고, Ubuntu ROS 터미널에서 실제 IP를 지정합니다.
실제 컨트롤러에는 `Tool Weight_1`과 `GripperDA_v1`을 미리 등록해야 합니다.
가상용 Tool/TCP 등록기는 실제 모드에서 실행하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ws_cobot_pjt/install/setup.bash
export ROS_DOMAIN_ID=29

ros2 launch m0609_rg2_bringup bringup.launch.py \
  mode:=real host:=192.168.1.100 rt_host:=192.168.1.100 port:=12345 \
  gripper_host:=192.168.1.1
```

IP는 실제 연결 환경에 맞게 수정합니다. 이후 이동·그리퍼 명령은 위 Ubuntu 터미널 2와 같습니다.
이 배포본의 native 진입점은 가상 실행용이며, 실제 로봇 연결은 Ubuntu 직접 실행 경로를 사용합니다.

## 변경 후 재빌드

먼저 실행 중인 가상 세션을 `Ctrl+C`로 종료합니다.

| 변경 내용 | macOS native | Ubuntu 직접 실행 |
|---|---|---|
| ROS 소스, launch, xacro, 설정, `setup.py` | 컨테이너에서 워크스페이스 재빌드 | `./build_workspace.sh` |
| ROS 의존성(`package.xml`), Dockerfile의 설치 항목 | 이미지 재빌드 후 워크스페이스 재빌드 | rosdep 의존성 설치 후 워크스페이스 재빌드 |
| `compose.yaml` | 컨테이너를 재생성하고 세션 재시작 | 변경된 에뮬레이터 설정으로 재시작 |
| `native/environment.yml` | Conda 환경 갱신 | 해당 없음 |

macOS에서 ROS 코드만 변경했다면 다음 명령으로 충분합니다.

```bash
docker compose run --rm cobot /opt/rokey-native/build_workspace.sh /workspace
./run_virtual.sh
```

Docker 이미지의 의존성이 바뀌었다면 다음 순서로 실행합니다.

```bash
docker compose --profile virtual down
docker compose build cobot
docker compose run --rm cobot /opt/rokey-native/build_workspace.sh /workspace
./run_virtual.sh
```

이미지 설치 단계를 캐시 없이 다시 수행해야 할 때만 아래 명령을 사용합니다.

```bash
docker compose build --no-cache cobot
```

Ubuntu에서 `package.xml` 의존성을 변경했다면 다음과 같이 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ws_cobot_pjt/src --ignore-src --rosdistro jazzy -y
./build_workspace.sh
```

`--symlink-install`을 사용하더라도 실행 파일 추가나 manifest 변경은 재빌드해야 합니다.
다른 PC나 운영 환경으로 옮길 때는 `build/`, `install/`, `log/`를 복사하지 않고
소스에서 다시 빌드합니다. Git에는 이 산출물을 포함하지 않습니다.

## simple_move와 가상 설정

`simple_move`는 기존 `rokey/move.py`에 연결됩니다.
HOME `[0, 0, 90, 0, 90, 0]`으로 이동한 뒤 관절 자세와 두 Cartesian 위치를 3회 반복하고
정상 완료 시 HOME으로 복귀합니다. 관절 속도는 30 deg/s, 직선 속도는 30 mm/s입니다.
이동 위치와 반복 횟수는 `move.py`에서 수정합니다.

가상 컨트롤러의 Tool/TCP 이름과 값은
`ws_cobot_pjt/src/cobot_rg2/rg2/m0609_rg2_bringup/config/virtual_robot.yaml`에 있습니다.
`simple_move`에서 사용하는 이름은 `Tool Weight_1`, `GripperDA_v1`입니다.
실측 `calibration.json` 대신 기본 RG2 예제 프로파일을 사용합니다.

## Git 배포 준비

빌드·실행 산출물, Python 캐시, `.env`, 로그는 `.gitignore`로 제외합니다.
개발 폴더의 스캔·힘 제어·포인트클라우드 코드, 실험 결과, 벤치마크,
중복 rokey 패키지, MoveIt 예제, 다른 로봇/그리퍼 모델은 복사하지 않았습니다.
벤더 라이선스와 변경 범위는 [THIRD_PARTY.md](THIRD_PARTY.md)에 정리했습니다.

변경 파일을 검토한 뒤 저장소 루트에서 커밋하고 push합니다.

```bash
cd ..
git switch ros
git status --short
git add README.md ws_cobot_pjt
git diff --cached --stat
git commit -m "Add ROS simple motion and RG2 gripper runtime"
git push -u origin ros
```
