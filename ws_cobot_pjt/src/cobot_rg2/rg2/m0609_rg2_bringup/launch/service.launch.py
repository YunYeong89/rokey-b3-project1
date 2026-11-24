"""One namespaced robot for the fleet launcher. Uses the normal robot bringup."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names = ('robot_ns', 'mode', 'host', 'rt_host', 'virtual_force', 'virtual_profile_file',
             'bridge_port', 'bridge_token')
    config = {name: LaunchConfiguration(name) for name in names}
    ns = config['robot_ns']
    bringup = {key: config[key] for key in names[:-2]}
    bringup.update(manage_emulator='false', gui='false')
    return LaunchDescription([
        *(DeclareLaunchArgument(name) for name in names),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(__file__).with_name('bringup.launch.py'))),
            launch_arguments=bringup.items()),
        Node(package='m0609_rg2_bringup', executable='virtual_force_bridge.py', namespace=ns,
             arguments=['host.docker.internal', config['bridge_port'], config['bridge_token']],
             condition=IfCondition(config['virtual_force']), output='screen',
             on_exit=Shutdown(reason='Gazebo bridge exited')),
        ExecuteProcess(cmd=[sys.executable, '-u', '-m', 'repair_car.web.web_control', '--ros-args', '-r',
                            ['__ns:=/', ns]],
                       output='screen', on_exit=Shutdown(reason='Robot web control exited')),
    ])
