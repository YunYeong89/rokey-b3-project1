"""Hand teleoperation of the M0609 TCP and the RG2 width, in the virtual (Gazebo) session or on the real robot. The
camera side runs on the macOS host (run_camera.sh); this node connects to it over TCP.

  ros2 run hand_teleop teleop [--host host.docker.internal] [--port 5070]
"""
import argparse
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.signals import SignalHandlerOptions
from rokey.robot_services import RobotServices

from .link import HandLink
from .session import TeleopSession


def main():
    config = yaml.safe_load((Path(get_package_share_directory('hand_teleop')) / 'config' / 'teleop.yaml').read_text())
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default=config['camera']['host'])
    parser.add_argument('--port', type=int, default=config['camera']['port'])
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('hand_teleop', namespace='dsr01')
    robot = RobotServices(node, config['service_timeout'])
    try:
        TeleopSession(node, robot, HandLink(args.host, args.port), config).run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            robot.stop()
        finally:
            robot.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
