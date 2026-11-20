"""Shared context of the robot programs: a node in /dsr01, controller services, the contact probe, the
configuration and the /repair/status and /repair/vehicle publishers (std_msgs/String JSON, INTERFACE.md)."""
import json

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

if __package__:
    from .configs import Config, load
    from .contact_probe import ContactProbe
    from .robot_services import RobotServices
    from .robot_namespace import robot_topic
    from .rotations import rotation_zyz
else:
    from configs import Config, load
    from contact_probe import ContactProbe
    from robot_services import RobotServices
    from robot_namespace import robot_topic
    from rotations import rotation_zyz


class RobotProgram:
    def __init__(self, name):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(name, namespace='dsr01')
        self.robot = RobotServices(self.node, load('robot')['robot']['service_timeout'])
        self.mode = 'virtual' if self.robot.get_robot_system() == 1 else 'real'
        self.config = Config(self.mode)
        scan = self.config['scan']
        self.probe = ContactProbe(self.robot, scan['press_force'], scan['touch_threshold'])
        self.status = self.node.create_publisher(String, robot_topic(self.node, '/repair/status'), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.vehicle = self.node.create_publisher(String, robot_topic(self.node, '/repair/vehicle'), latched)  # choose_car result
        self.top_mm = 0.0  # tallest obstacle a program adds to the travel height (safe_z)
        self.topics = None  # real_scan: part_scan.scan_topics.ScanTopics (/scan/* samples, progress, ROI, result)

    def reload(self):
        """Re-read the configuration (a new calibration result or session plan)."""
        self.config = Config(self.mode)
        self.probe.touch_threshold = self.config['scan']['touch_threshold']

    def log(self, text):
        self.node.get_logger().info(text)

    def publish(self, **message):
        self.status.publish(String(data=json.dumps(message)))

    def safe_z(self):
        """Travel height above every vehicle in the zone, or above `top_mm` set by a program for a taller one
        (the raised real vehicle) (BASE mm)."""
        zone = self.config['vehicle_zone']
        return max(zone['table_z_mm'] + zone['max_height_mm'], self.top_mm) + self.config['scan']['clearance_mm']

    def home(self, joints=None):
        """movej home in position control (force and compliance released first). From the work area (up to
        50 mm above the travel height) the probe first goes straight up to the travel height and across to
        above the home TCP: a joint-space move from above the vehicle would cut down over it."""
        self.probe.close()
        motion = self.config['motion']
        joints = self.config['robot']['home_joints_deg'] if joints is None else joints  # teleop: any pose
        pose, _ = self.probe.read()
        target, rotation = self.robot.fkin(joints), rotation_zyz(pose[3:])
        offset = np.linalg.norm(np.subtract(pose[:3], target[:3]))
        turn = np.degrees(np.arccos(np.clip((np.trace(rotation.T @ rotation_zyz(target[3:])) - 1) / 2, -1, 1)))
        if offset < 1.0 and turn < 1.0:
            return  # already HOME: the HOME tip (z ~153 mm) is below the 180 mm travel height, so the lift below
            #         made the probe bob up and back down on every call (twice before each surface run)
        if pose[2] <= self.safe_z() + 50.0 and np.linalg.norm(np.subtract(pose[:2], target[:2])) > 5.0:
            z = max(pose[2], self.safe_z())  # lift only when it has to travel across the work area
            self.probe.move([*pose[:2], z], rotation, motion['approach_speed'], acc=motion['approach_acc'])
            self.probe.move([*target[:2], z], rotation, motion['linear_speed'], acc=motion['linear_acc'])
        self.robot.movej(joints, motion['joint_speed'], motion['joint_acc'])

    def close(self):
        try:
            self.probe.close()
        finally:
            self.robot.close()
            self.node.destroy_node()
            rclpy.shutdown()
