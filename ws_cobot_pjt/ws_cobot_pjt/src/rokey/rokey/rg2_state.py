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
    SETTLE_S = .5
    WIDTH_TOLERANCE_MM = .2

    def __init__(self, node, held_width):
        self.held_width = held_width
        self.width = self.grip = None
        self.busy = False
        self.width_at = self.grip_at = 0.
        self.settled_width, self.settling_since = None, 0.
        node.create_subscription(JointState, 'gripper_joint_states', self.joints, qos_profile_sensor_data)
        node.create_subscription(Bool, 'gripper/grip_detected', self.gripping, 10)

    @staticmethod
    def width_mm(angle):
        return 2000. * (math.cos(angle + .76794) * .055 - .0144 + .108505 * math.cos(1.41371))

    def joints(self, message):
        if self.FINGER in message.name:
            index = message.name.index(self.FINGER)
            if index < len(message.position) and math.isfinite(message.position[index]):
                width, now = self.width_mm(message.position[index]), time.monotonic()
                if (self.settled_width is None or now - self.width_at > self.FRESH_S
                        or abs(width - self.settled_width) > self.WIDTH_TOLERANCE_MM):
                    self.settled_width, self.settling_since = width, now
                self.width, self.width_at = width, now
                self.busy = any(message.effort)

    def gripping(self, message):
        self.grip, self.grip_at = message.data, time.monotonic()

    def held(self):
        """True for a held probe, False for empty jaws, None for missing or ambiguous feedback."""
        now = time.monotonic()
        if self.width is None or now - self.width_at > self.FRESH_S:
            return None
        low, high = self.held_width
        if low <= self.width <= high:
            # With DO held on, RG2 can keep reporting busy even after the jaws have settled on the probe.
            if not self.busy or self.width_at - self.settling_since >= self.SETTLE_S:
                return True
            return None
        if self.busy:
            return None
        if self.grip and now - self.grip_at <= self.FRESH_S:
            return None
        if self.width <= 3. or self.width >= high + 5.:
            return False
