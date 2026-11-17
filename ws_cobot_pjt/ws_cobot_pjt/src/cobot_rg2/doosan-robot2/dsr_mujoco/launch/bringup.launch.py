"""Container-side ROS bringup for an independently running native MuJoCo session.

run_virtual.sh already owns bringup and the token-authenticated bridge; do not launch this twice.
This entry point supports split host/container orchestration without changing existing launch files.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='emulator'),
        DeclareLaunchArgument('native_host', default_value='host.docker.internal'),
        DeclareLaunchArgument('native_port'), DeclareLaunchArgument('token'),
        DeclareLaunchArgument('profile_file'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('m0609_rg2_bringup'), 'launch', 'bringup.launch.py'])),
            launch_arguments={'mode': 'virtual', 'host': LaunchConfiguration('host'),
                              'virtual_force': 'true', 'manage_emulator': 'false', 'gui': 'false',
                              'virtual_profile_file': LaunchConfiguration('profile_file')}.items()),
        ExecuteProcess(cmd=['ros2', 'run', 'm0609_rg2_bringup', 'virtual_force_bridge.py',
                            LaunchConfiguration('native_host'), LaunchConfiguration('native_port'),
                            LaunchConfiguration('token')], output='screen')])
