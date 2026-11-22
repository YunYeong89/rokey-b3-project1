"""RG2 feedback shared by real probe pickup and the web controller."""
import math
import time

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class RG2State:
    FINGER = 'rg2_finger_joint'
    FRESH_S = 2.
    HELD_WIDTH_MM = (26., 31.)

    def __init__(self, node, held_width):
        self.held_width = held_width
        self.width = self.grip = None
        self.busy = False
        self.width_at = self.grip_at = 0.
        node.create_subscription(JointState, 'gripper_joint_states', self.joints, qos_profile_sensor_data)
        node.create_subscription(Bool, 'gripper/grip_detected', self.gripping, 10)

    @staticmethod
    def width_mm(angle):
        return 2000. * (math.cos(angle + .76794) * .055 - .0144 + .108505 * math.cos(1.41371))

    def joints(self, message):
        if self.FINGER in message.name:
            index = message.name.index(self.FINGER)
            if index < len(message.position) and math.isfinite(message.position[index]):
                self.width = self.width_mm(message.position[index])
                self.width_at = time.monotonic()
                self.busy = any(message.effort)

    def gripping(self, message):
        self.grip, self.grip_at = message.data, time.monotonic()

    def held(self):
        """True for a held probe, False for empty jaws, None for missing or ambiguous feedback."""
        now = time.monotonic()
        if self.busy or now - self.width_at > self.FRESH_S:
            return None
        low, high = self.held_width
        if low <= self.width <= high:
            return True
        if self.grip and now - self.grip_at <= self.FRESH_S:
            return None
        if self.width <= 3. or self.width >= high + 5.:
            return False
