"""Select a robot through the ROS namespace remap, with legacy single-robot names."""
import re
import sys


def namespace_args(argv=None):
    ros_args = False
    namespace = '/dsr01'
    for arg in sys.argv[1:] if argv is None else argv:
        if arg == '--ros-args':
            ros_args = True
        elif arg == '--':
            ros_args = False
        elif ros_args and arg.startswith('__ns:='):
            namespace = '/' + arg.partition(':=')[2].strip('/')
    return namespace


def fleet_id(namespace):
    name = namespace.strip('/')
    return name if re.fullmatch(r'robot_[1-9][0-9]*', name) else None


def robot_topic(node, absolute):
    namespace = node.get_namespace()
    return namespace + absolute if fleet_id(namespace) else absolute


def program_args():
    from rclpy.utilities import remove_ros_args
    return remove_ros_args()[1:]
