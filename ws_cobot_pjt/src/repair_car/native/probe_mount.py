"""One Gazebo pose sample defines the fixed probe-to-flange mounting transform."""
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation


class ProbeMount:
    def __init__(self, node):
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import SubscribeOptions
        self.message = Pose_V
        self.condition = threading.Condition()
        self.sample = None
        node.subscribe_raw('/world/cobot_world/pose/info', self.receive,
                           'gz.msgs.Pose_V', SubscribeOptions())

    @staticmethod
    def matrix(pose):
        result = np.eye(4)
        result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        q = pose.orientation
        result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return result

    def receive(self, data, _info):
        poses = {pose.name: pose for pose in self.message.FromString(data).pose}
        names = ('cobot', 'link_6', 'docked_probe', 'body')
        if all(name in poses for name in names):
            robot, flange, probe, body = [self.matrix(poses[name]) for name in names]
            with self.condition:
                self.sample = (time.monotonic(), np.linalg.inv(robot @ flange) @ probe @ body)
                self.condition.notify_all()

    def transform(self):
        with self.condition:
            if not self.condition.wait_for(lambda: self.sample is not None and
                                           time.monotonic() - self.sample[0] < .2, timeout=2):
                raise TimeoutError('Fresh Gazebo probe/flange poses unavailable for mounting')
            return self.sample[1].copy()
