# 수정된 bringup.launch.py jazzy 용
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, GroupAction, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────
    # (virtual) ros2 launch m0609_rg2_bringup bringup.launch.py
    # (real)    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
    args = [
        DeclareLaunchArgument("robot_ns", default_value="dsr01"),
        DeclareLaunchArgument("mode", default_value="virtual",   description="Operation mode: real | virtual"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="Robot IP (real mode)"),
        DeclareLaunchArgument("rt_host", default_value=LaunchConfiguration("host"), description="Robot RT interface IP"),
        DeclareLaunchArgument("port", default_value="12345",     description="Robot port"),
        DeclareLaunchArgument("manage_emulator", default_value="true", description="Start and stop a local emulator"),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz"),
        DeclareLaunchArgument("virtual_force", default_value="false", description="Use an external Gazebo force backend"),
        DeclareLaunchArgument("virtual_profile_file", default_value=PathJoinSubstitution([
            FindPackageShare("rokey"), "calibration.json"
        ]), description="Real TCP/tool calibration applied only in virtual mode"),
    ]

    namespace = LaunchConfiguration("robot_ns")
    root = ["/", namespace]
    is_real    = PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real'"])
    is_virtual = PythonExpression(["'", LaunchConfiguration("mode"), "' == 'virtual'"])
    virtual_force = PythonExpression([
        "'", LaunchConfiguration("mode"), "' == 'virtual' and '",
        LaunchConfiguration("virtual_force"), "' == 'true'",
    ])
    nominal_topic = PythonExpression([
        "'/", namespace, "/nominal_joint_states' if (", virtual_force,
        ") else '/", namespace, "/arm_joint_states'",
    ])
    is_managed_virtual = PythonExpression([
        "'", LaunchConfiguration("mode"), "' == 'virtual' and '",
        LaunchConfiguration("manage_emulator"), "' == 'true'",
    ])

    # ── [virtual] DRCF 에뮬레이터 (Docker) ───────────────────────────
    # virtual mode 시 localhost에서 DRCF 에뮬레이터를 실행
    # 종료 시 terminate_drcf()로 컨테이너 자동 정리
    run_emulator_node = Node(
        package    = "dsr_bringup2",
        executable = "run_emulator",
        namespace=namespace,
        parameters = [
            {"name": namespace},
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")},
            {"mode": LaunchConfiguration("mode")},
            {"model": "m0609"},
            {"gripper": "none"},
            {"mobile": "none"},
        ],
        condition=IfCondition(is_managed_virtual),
        output="screen",
    )

    # ── [virtual] 이전 run 잔여 에뮬레이터 컨테이너 정리 ──────────────
    # run_drcf.sh의 중복 컨테이너 체크는 'docker ps -q'(running 상태만) 기반이라
    # Exited 상태로 남은 --rm 미정리 컨테이너를 놓친다. 그 경우 다음 bringup의
    # 'docker run --name dsr01_emulator'가 이름 충돌로 실패 → 에뮬레이터 미기동 →
    # ros2_control 하드웨어 init 실패로 연쇄. run_emulator 시작 전 동명 컨테이너를
    # 강제 제거해 launch를 idempotent하게 만든다.
    emulator_cleanup = ExecuteProcess(
        cmd=["bash", "-c", "docker rm -f dsr01_emulator 2>/dev/null || true"],
        condition=IfCondition(is_managed_virtual),
        output="log",
    )

    start_emulator = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action = emulator_cleanup,
            on_exit       = [run_emulator_node],
        ),
    )

    # ── 커스텀 URDF (M0609 + RG2) ────────────────────────────────────
    xacro_file       = os.path.join(get_package_share_directory("m0609_rg2_bringup"), "xacro", "my_m0609_rg2.urdf.xacro")
    rviz_config_file = os.path.join(get_package_share_directory("m0609_rg2_bringup"), "rviz", "default.rviz" )

    # ── [real] Doosan URDF (ros2_control 하드웨어 인터페이스용) ───────
    # doosan_xacro = PathJoinSubstitution([
    #     FindPackageShare('dsr_description2'), 'urdf', 'm0609.urdf'
    # ])

    doosan_robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' name:=', namespace,
        ' host:=', LaunchConfiguration('host'),
        ' rt_host:=', LaunchConfiguration('rt_host'),
        ' port:=', LaunchConfiguration('port'),
        ' mode:=', LaunchConfiguration('mode'),
        ' model:=m0609',
        ' update_rate:=100',
    ])

    # ── ros2_control_node (virtual/real 공통) ─────────────────────────
    # virtual: run_emulator_node가 먼저 Docker DRCF를 띄우고,
    #          dsr_hw_interface2 재시도 루프(0.5s × 20회)로 연결
    control_node = Node(
        package    = "controller_manager",
        executable = "ros2_control_node",
        namespace=namespace,
        additional_env={"LD_PRELOAD": os.environ["COBOT_NETWORK_SHIM"]}
                       if os.environ.get("COBOT_NETWORK_SHIM") else {},
        parameters = [
            {
                "robot_description": ParameterValue(doosan_robot_description, value_type=str)
            },
            {"update_rate": 100},
            PathJoinSubstitution(
                [FindPackageShare("dsr_controller2"), "config", "dsr_controller2.yaml"]
            ),
        ],
        output="both",
    )
    force_services = [f'force/{name}' for name in [
        'task_compliance_ctrl', 'set_stiffnessx', 'set_desired_force',
        'release_force', 'release_compliance_ctrl', 'check_force_condition',
    ]] + ['aux_control/get_tool_force', 'aux_control/get_current_posx']
    control_group = GroupAction([
        *[SetRemap(src=[*root, f'/dsr_controller2/{service}'],
                   dst=[*root, f'/dsr_controller2/vendor/{service}'],
                   condition=IfCondition(virtual_force)) for service in force_services],
        control_node,
    ])

    # ── joint_state_broadcaster (/dsr01/arm_joint_states 퍼블리시) ────────
    joint_state_broadcaster_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        namespace=namespace,
        arguments  = ["joint_state_broadcaster", "-c", "controller_manager",
                      "--controller-ros-args=--remap",
                      ["--controller-ros-args=/", namespace, "/joint_states:=", nominal_topic]],
    )

    # ── dsr_controller2 (motion service 등록) ─────────────────────────
    robot_controller_spawner = Node(
        package    = "controller_manager",
        executable = "spawner",
        namespace=namespace,
        arguments  = ["dsr_controller2", "-c", "controller_manager"],
    )
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action = joint_state_broadcaster_spawner,
            on_exit       = [robot_controller_spawner],
        ),
    )

    virtual_profile_loader = Node(
        package="m0609_rg2_bringup", executable="virtual_profile_loader.py",
        namespace=namespace, condition=IfCondition(is_virtual), output="screen",
        parameters=[{"profile_file": LaunchConfiguration("virtual_profile_file")}],
    )
    load_virtual_profiles = RegisterEventHandler(OnProcessExit(
        target_action=robot_controller_spawner,
        on_exit=lambda event, context: [virtual_profile_loader] if not event.returncode else [],
    ))
    profile_failure = RegisterEventHandler(OnProcessExit(
        target_action=virtual_profile_loader,
        on_exit=lambda event, context: [EmitEvent(event=Shutdown(
            reason='Virtual TCP/tool profile registration failed'))] if event.returncode else [],
    ))

    # ── [virtual] GripperVirtualNode (/onrobot/sendCommand 서비스) ───
    is_virtual_gripper = PythonExpression(
        ["'", LaunchConfiguration("mode"), "' == 'virtual'"]
    )

    is_fleet = PythonExpression(["'", namespace, "' != 'dsr01'"])
    def gripper_group(node):
        return GroupAction([
            *(SetRemap(src=topic, dst=[*root, topic], condition=IfCondition(is_fleet))
              for topic in ('/onrobot/sendCommand', '/onrobot/restartPower', '/onrobot/pose')),
            node,
        ])

    gripper_virtual_node = Node(
        package    = "m0609_rg2_bringup",
        executable = "gripper_virtual_node.py",
        namespace=namespace,
        name       = "gripper_virtual_node",
        condition  = IfCondition(is_virtual_gripper),
        output     = "screen",
    )

    # ── [real] OnRobot RG2 드라이버 ──────────────────────────────────
    # /joint_states → /onrobot_joint_states 로 remap (joint_state_publisher와 충돌 방지)
    onrobot_driver = Node(
        package    = "onrobot_rg_control",
        executable = "OnRobotRGControllerServer",
        namespace=namespace,
        name       = "OnRobotRGControllerServer",
        output     = "screen",
        parameters = [
            {
                "/onrobot/control"      : "modbus",
                "/onrobot/ip"           : "192.168.1.1",
                "/onrobot/port"         : 502,
                "/onrobot/changer_addr" : 65,
                "/onrobot/gripper"      : "rg2",
                "/onrobot/offset"       : 5,
            }
        ],
        remappings=[("/joint_states", "/onrobot_joint_states")],
        condition=IfCondition(is_real),
    )

    # ── [real] 그리퍼 너비 → rg2_finger_joint 변환 노드 ──────────────
    # OnRobotRGInput.ggwd → /gripper_joint_states (rg2_finger_joint)
    gripper_joint_state_publisher = Node(
        package    = "m0609_rg2_bringup",
        executable = "gripper_joint_state_publisher.py",
        namespace=namespace,
        name       = "gripper_joint_state_publisher",
        condition  = IfCondition(is_real),
        output     = "screen",
    )

    # ── joint_state_publisher (virtual/real 공통) ─────────────────────
    # arm_joint_states와 gripper_joint_states 통합 (자기 구독 방지)
    # virtual 환경에서는 /gripper_joint_states 없음(DRCF 문제) → gripper joint 0으로 채워짐
    joint_state_publisher_node = Node(
        package    = "joint_state_publisher",
        executable = "joint_state_publisher",
        namespace=namespace,
        name       = "joint_state_publisher",
        parameters = [{"source_list": ["arm_joint_states", "gripper_joint_states"],
                       "rate": 60.0}],
        condition=UnlessCondition(virtual_force),
    )
    virtual_joint_state_publisher = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        namespace=namespace, name="joint_state_publisher",
        parameters=[{"source_list": ["virtual_control/joint_states", "gripper_joint_states"],
                     "rate": 60.0}],
        condition=IfCondition(virtual_force),
    )

    # ── robot_state_publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package    = "robot_state_publisher",
        executable = "robot_state_publisher",
        namespace=namespace,
        name       = "robot_state_publisher",
        output     = "both",
        parameters = [
            {
                "robot_description": ParameterValue(doosan_robot_description, value_type=str)
            }
        ],
    )

    # ── Static TF (world → base_link) ────────────────────────────────
    static_tf = Node(
        package    = "tf2_ros",
        executable = "static_transform_publisher",
        namespace=namespace,
        name      = "static_transform_publisher",
        output    = "log",
        arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package    = "rviz2",
        executable = "rviz2",
        namespace=namespace,
        name       = "rviz2",
        output     = "log",
        arguments  = ["-d", rviz_config_file],
        condition  = IfCondition(LaunchConfiguration("gui")),
    )

    tf_remaps = [SetRemap(src=topic, dst=[*root, topic], condition=IfCondition(is_fleet))
                 for topic in ('/tf', '/tf_static')]
    return LaunchDescription(args + [
            emulator_cleanup,
            start_emulator,
            gripper_group(gripper_virtual_node),
            control_group,
            joint_state_broadcaster_spawner,
            delay_controller,
            load_virtual_profiles,
            profile_failure,
            gripper_group(onrobot_driver),
            gripper_joint_state_publisher,
            joint_state_publisher_node,
            virtual_joint_state_publisher,
            GroupAction([*tf_remaps, robot_state_publisher, static_tf]),
            rviz_node,
        ]
    )
