"""Repair process: repair_manager (waits for /repair/command) and the vehicle point cloud node.
With rosbridge:=true also the websocket bridge for the web app (port 9090).
  ros2 launch repair_car repair.launch.py [rosbridge:=true] [calibrate:=true]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calibrate = LaunchConfiguration('calibrate')
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge', default_value='false'),
        DeclareLaunchArgument('calibrate', default_value='false', description='calibrate at every job start'),
        Node(package='repair_car', executable='pointcloud', output='screen'),
        Node(package='repair_car', executable='repair_manager', output='screen', emulate_tty=True,
             arguments=[PythonExpression(["'--calibrate' if '", calibrate, "' == 'true' else ''"])]),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])),
            condition=IfCondition(LaunchConfiguration('rosbridge'))),
    ])
