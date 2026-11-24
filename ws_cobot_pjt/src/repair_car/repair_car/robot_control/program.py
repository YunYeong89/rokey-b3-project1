"""Shared context of the robot programs: a node in /dsr01, controller services, the contact probe, the
configuration and the /repair/status and /repair/vehicle publishers (std_msgs/String JSON, INTERFACE.md)."""
import json

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

if __package__:
    from ..config import Config, load
    from repair_car.robot_control.contact_probe import ContactProbe
    from repair_car.robot_control.home_motion import HomeMotion
    from .robot_bridge import RobotServices, robot_topic
    from .rotations import rotation_zyz
else:
    from config import Config, load
    from repair_car.robot_control.contact_probe import ContactProbe
    from repair_car.robot_control.home_motion import HomeMotion
    from robot_control.robot_bridge import RobotServices, robot_topic
    from robot_control.rotations import rotation_zyz


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
        self.scan_blend = True
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

    def scan_transfer(self, pose, xy, z, rotation):
        """Lift and traverse above the scan. Round the corner within the clearance above the vehicle."""
        motion = self.config['motion']
        transfer = motion['scan_transfer']
        start = rotation_zyz(pose[3:])
        lift, across = np.r_[pose[:2], z], np.r_[xy, z]
        rise = z - pose[2]
        distance = np.linalg.norm(across[:2] - lift[:2])
        radius = min(transfer['blend_radius_mm'], self.config['scan']['clearance_mm'] / 3.)
        turn = self.probe.turn_deg(pose[3:], rotation)
        if not self.scan_blend:
            if rise > 0. or turn >= .5:
                self.probe.blend([(lift, rotation)], transfer['lift_speed'], 0., acc=motion['approach_acc'],
                                 retry_speed=motion['approach_speed'])
            self.probe.blend([(across, rotation)], transfer['blend_speed'], 0., acc=motion['approach_acc'],
                             retry_speed=motion['approach_speed'])
            return
        if radius > 0. and min(rise, distance) > 2. * radius and turn < .5:
            self.probe.blend([(lift, start), (across, rotation)], transfer['blend_speed'], radius,
                             acc=motion['approach_acc'])
            return
        if rise > 0. or turn >= .5:
            self.probe.move(lift, rotation, transfer['lift_speed'], acc=motion['approach_acc'])
        self.probe.move(across, rotation, transfer['linear_speed'], acc=motion['linear_acc'])

    def home(self, joints=None, *, scan_travel=False):
        """movej home in position control (force and compliance released first). From the work area (up to
        50 mm above the travel height) the probe first goes straight up to the travel height and across to
        above the home TCP: a joint-space move from above the vehicle would cut down over it."""
        transfer = self.scan_transfer if scan_travel and self.mode == 'real' else None
        HomeMotion(self.robot, self.probe, self.config, self.safe_z()).run(joints, scan_transfer=transfer)

    def close(self):
        try:
            self.probe.close()
        finally:
            self.robot.close()
            self.node.destroy_node()
            rclpy.shutdown()
