"""M0609/RG2 joint motion and gripper control, without external force control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('m0609_rg2_bringup')
    mode, host, port = [LaunchConfiguration(key) for key in ['mode', 'host', 'port']]
    virtual = PythonExpression(["'", mode, "' == 'virtual'"])
    real = PythonExpression(["'", mode, "' == 'real'"])
    args = [
        DeclareLaunchArgument('mode', default_value='virtual', choices=['virtual', 'real']),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('rt_host', default_value=host),
        DeclareLaunchArgument('gripper_host', default_value='192.168.1.1'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('virtual_profile_file', default_value=PathJoinSubstitution([
            share, 'config', 'virtual_robot.yaml'])),
    ]
    description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', PathJoinSubstitution([share, 'xacro', 'my_m0609_rg2.urdf.xacro']),
        ' name:=dsr01 mode:=', mode, ' host:=', host, ' port:=', port,
        ' rt_host:=', LaunchConfiguration('rt_host'), ' model:=m0609 update_rate:=100',
    ]), value_type=str)
    controller = Node(
        package='controller_manager', executable='ros2_control_node', namespace='dsr01',
        parameters=[{'robot_description': description, 'update_rate': 100},
                    PathJoinSubstitution([FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml'])],
        output='screen')
    broadcaster = Node(
        package='controller_manager', executable='spawner', namespace='dsr01',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager',
                   '--controller-ros-args=--remap',
                   '--controller-ros-args=/dsr01/joint_states:=/dsr01/arm_joint_states'])
    robot = Node(package='controller_manager', executable='spawner', namespace='dsr01',
                 arguments=['dsr_controller2', '-c', 'controller_manager'])
    profiles = Node(
        package='m0609_rg2_bringup', executable='virtual_profile_loader.py', namespace='dsr01',
        condition=IfCondition(virtual), output='screen',
        parameters=[{'profile_file': LaunchConfiguration('virtual_profile_file')}])
    sequencing = [
        RegisterEventHandler(OnProcessExit(target_action=broadcaster,
            on_exit=lambda event, context: [robot] if not event.returncode else [])),
        RegisterEventHandler(OnProcessExit(target_action=robot,
            on_exit=lambda event, context: [profiles] if not event.returncode else [])),
        RegisterEventHandler(OnProcessExit(target_action=profiles,
            on_exit=lambda event, context: [EmitEvent(event=Shutdown(
                reason='Virtual Tool/TCP profile registration failed'))] if event.returncode else [])),
    ]
    virtual_gripper = Node(
        package='m0609_rg2_bringup', executable='gripper_virtual_node.py', namespace='dsr01',
        condition=IfCondition(virtual), output='screen')
    real_gripper = Node(
        package='onrobot_rg_control', executable='OnRobotRGControllerServer', namespace='dsr01',
        condition=IfCondition(real), output='screen',
        parameters=[{'/onrobot/control': 'modbus', '/onrobot/ip': LaunchConfiguration('gripper_host'),
                     '/onrobot/port': 502, '/onrobot/changer_addr': 65,
                     '/onrobot/gripper': 'rg2', '/onrobot/offset': 5}],
        remappings=[('/joint_states', '/onrobot_joint_states')])
    gripper_states = Node(
        package='m0609_rg2_bringup', executable='gripper_joint_state_publisher.py', namespace='dsr01',
        condition=IfCondition(real))
    combined_states = Node(
        package='joint_state_publisher', executable='joint_state_publisher', namespace='dsr01',
        parameters=[{'source_list': ['arm_joint_states', 'gripper_joint_states'], 'rate': 60.}])
    transforms = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='dsr01',
                      parameters=[{'robot_description': description}])
    rviz = Node(package='rviz2', executable='rviz2', namespace='dsr01',
                condition=IfCondition(LaunchConfiguration('gui')),
                arguments=['-d', PathJoinSubstitution([share, 'rviz', 'default.rviz'])])
    return LaunchDescription(args + sequencing + [controller, broadcaster, virtual_gripper,
                             real_gripper, gripper_states, combined_states, transforms, rviz])
