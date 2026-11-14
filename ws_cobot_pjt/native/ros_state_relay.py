"""Forward observed ROS joint states to a Mac display session."""
import argparse
import json
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self, stream):
        super().__init__('cobot_native_state_relay')
        self.stream = stream
        self.stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.positions = {}
        self.subscriptions_kept = [self.create_subscription(
            JointState, topic, self.receive, qos_profile_sensor_data)
            for topic in ['/dsr01/joint_states', '/dsr01/gripper_joint_states']]
        self.create_timer(1 / 60, self.send)

    def receive(self, message):
        self.positions.update(zip(message.name, message.position))

    def send(self):
        if self.positions:
            self.stream.sendall((json.dumps(self.positions) + '\n').encode())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('host')
    parser.add_argument('port', type=int)
    parser.add_argument('token')
    args = parser.parse_args()
    rclpy.init()
    with socket.create_connection((args.host, args.port), timeout=5) as stream:
        stream.settimeout(2)
        stream.sendall((args.token + '\n').encode())
        node = JointStateRelay(stream)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, OSError):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
