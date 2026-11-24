"""Stroke samples from the controller's RT stream (realtime/force_telemetry, dsr_hardware2 at its 100 Hz read cycle)
on their own node and thread, so no service call of the scan delays them. The controller answers one service at a
time: each set_desired_force (0.08-0.18 s, sent without waiting too) held the pose and force queries behind it and
left the stroke unsampled (064319, 4 mm/s: 52 gaps, 11.7 s of 45 s, 0.3-0.7 mm each).

The running M0609 reports this wrench in the same frame as get_tool_force(DR_BASE), verified against the service
at the bonnet. Average the RT samples to reduce the wrench noise seen in the faster stream."""
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class Telemetry:
    def __init__(self, node, average):
        from dsr_msgs2.msg import ForceTelemetry

        self.node = rclpy.create_node('mesh_height_telemetry', namespace=node.get_namespace(), context=node.context)
        self.samples, self.wrenches, self.clock = deque(), deque(maxlen=average), None
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(ForceTelemetry, 'realtime/force_telemetry', self.receive, qos)
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def receive(self, message):
        """(t: monotonic on the controller's clock, controller TCP pose [mm, ZYZ deg], BASE wrench [N, N m])."""
        if self.clock is None:
            self.clock = time.monotonic() - message.time_stamp
        pose = np.array(message.actual_tcp_pose_mm_deg)
        self.wrenches.append(np.asarray(message.external_tcp_wrench_n_nm))
        self.samples.append((message.time_stamp + self.clock, pose, np.mean(self.wrenches, axis=0)))

    def drain(self):
        while self.samples:
            yield self.samples.popleft()

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
