"""RG2 width through /onrobot/sendCommand. The real OnRobot driver reads an integer string as the width in 0.1 mm
(onrobot_rg_control OnRobotRGControllerServer.genCommand), the virtual node a float string as the finger joint angle
in rad (m0609_rg2_bringup gripper_virtual_node); the width is converted with the driver's RG2 linkage
(OnRobotRGControllerServer.widthToJointValue)."""
import time

import numpy as np
from onrobot_rg_msgs.srv import SetCommand

L1, L3, THETA1, THETA3, DY = 0.108505, 0.055, 1.41371, 0.76794, -0.0144  # RG2 linkage (m, rad)


class RG2Gripper:
    def __init__(self, node, virtual, config):
        self.client = node.create_client(SetCommand, '/onrobot/sendCommand')
        self.virtual = virtual
        self.max_width, self.step, self.period = config['max_width_mm'], config['step_mm'], config['period_s']
        self.width, self.sent = None, 0.0

    def command(self, width):
        if self.virtual:
            return str(np.arccos((width / 2000 - DY - L1 * np.cos(THETA1)) / L3) - THETA3)
        return str(int(round(width * 10)))

    def open_to(self, openness):
        """Openness 0 (closed) ... 1 (max width), sent on a change of at least one step, at most once per period."""
        width = round(openness * self.max_width / self.step) * self.step
        now = time.monotonic()
        if width == self.width or now - self.sent < self.period:
            return
        self.client.call_async(SetCommand.Request(command=self.command(width)))
        self.width, self.sent = width, now
