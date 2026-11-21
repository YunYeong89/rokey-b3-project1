"""Container side of run_virtual.sh --acc, started by path (no rebuild needed): no emulator, ros2_control or
dsr_controller2. The URDF (robot_state_publisher: TF and the robot programs' kinematics) and the joint states of the
native simulation (sim_joint_states of the controller bridge, cobot_virtual/controller.py)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro = os.path.join(get_package_share_directory('m0609_rg2_bringup'), 'xacro', 'my_m0609_rg2.urdf.xacro')
    description = Command([FindExecutable(name='xacro'), ' ', xacro, ' name:=dsr01 host:=127.0.0.1',
                           ' rt_host:=127.0.0.1 port:=12345 mode:=virtual model:=m0609 update_rate:=100'])
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='dsr01',
             name='robot_state_publisher', output='both',
             parameters=[{'robot_description': ParameterValue(description, value_type=str)}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher', namespace='dsr01',
             name='joint_state_publisher', parameters=[{'source_list': ['sim_joint_states'], 'rate': 60.0}]),
        Node(package='tf2_ros', executable='static_transform_publisher', namespace='dsr01',
             name='static_transform_publisher', output='log',
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']),
    ])
