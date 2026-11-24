"""Read the real RG2's measured opening and grip flags from one device sample."""
import time

from onrobot_rg_msgs.msg import OnRobotRGInput
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RG2State:
    BUSY, GRIP, SAFETY = 0x01, 0x02, 0x7c
    FRESH_S = 1.
    HELD_WIDTH_MM = (26., 31.)
    MAX_REPORTED_WIDTH = 1200  # reject invalid register values, with margin above the rated 110 mm stroke
    SETTLE_S = .5
    WIDTH_TOLERANCE = 2   # tenths of a millimetre, as reported by the driver
    WAIT_S = 3.

    def __init__(self, node, held_width):
        self.held_width = held_width
        self.sample = self.anchor = None
        self.received_at = self.settling_since = 0.
        self.subscription = node.create_subscription(
            OnRobotRGInput, 'gripper/status', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def receive(self, message):
        now = time.monotonic()
        width, flags = message.ggwd, message.gsta
        if (self.anchor is None or now - self.received_at > self.FRESH_S or
                flags != self.anchor[1] or abs(width - self.anchor[0]) > self.WIDTH_TOLERANCE):
            self.anchor, self.settling_since = (width, flags), now
        self.sample, self.received_at = (width, flags), now

    @property
    def width(self):
        if self.sample is not None and time.monotonic() - self.received_at <= self.FRESH_S:
            return self.sample[0] / 10.

    def gripped(self, *, settled=True):
        """Grip bit only, also usable when transporting a bonnet of a different width."""
        if self.width is None or self.sample[0] > self.MAX_REPORTED_WIDTH:
            return None
        flags = self.sample[1]
        if flags & (self.BUSY | self.SAFETY):
            return None
        if settled and self.received_at - self.settling_since < self.SETTLE_S:
            return None
        return bool(flags & self.GRIP)

    def opened(self, *, settled=True):
        return self.gripped(settled=settled) is False and self.sample[0] / 10. >= self.held_width[1] + 5.

    def held(self, *, settled=True):
        """True for a probe, False for empty jaws, None when feedback cannot prove either."""
        grip = self.gripped(settled=settled)
        width = self.sample[0] / 10. if grip is not None else None
        if grip is True and self.held_width[0] <= width <= self.held_width[1]:
            return True
        if grip is False and (width <= 3. or width >= self.held_width[1] + 5.):
            return False

    def description(self):
        width = self.width
        if width is None:
            return 'no recent RG2 feedback'
        return f'RG2 width {width:.1f} mm, status 0x{self.sample[1]:02x}'

    def wait(self, read, expected, spin, message, *, after=0.):
        deadline = time.monotonic() + self.WAIT_S
        while time.monotonic() < deadline:
            result = read()
            if result in expected and self.received_at - after >= self.SETTLE_S:
                return result
            spin()
        raise RuntimeError(f'{message} ({self.description()})')
