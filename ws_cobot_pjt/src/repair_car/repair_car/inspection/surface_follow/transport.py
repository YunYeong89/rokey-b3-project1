"""Native Gazebo TCP / force state for the virtual scans (surface_follow/state)."""
from dataclasses import dataclass, field
import json
import time

import numpy as np
from std_msgs.msg import String


@dataclass
class State:
    sequence: int
    sampled_at: float
    position: np.ndarray
    rotation: np.ndarray
    wrench: np.ndarray
    fast_wrench: np.ndarray
    velocity: np.ndarray
    control: dict = field(default_factory=dict)


class VirtualTransport:
    def __init__(self, program):
        self.robot, self.node = program.robot, program.node
        self.data, self.received = {}, 0.
        self.subscription = self.node.create_subscription(String, 'surface_follow/state', self.receive, 1)

    def receive(self, message):
        self.data, self.received = json.loads(message.data), time.monotonic()

    def spin(self, seconds=.005):
        self.robot.executor.spin_once(timeout_sec=seconds)

    def wait_ready(self, seconds=5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin(.02)
            if self.data.get('ready') and self.data.get('simulation_only') and self.data.get('sequence'):
                return self.state()
        raise TimeoutError('Surface stream state unavailable; rebuild the bridge and restart run_virtual.sh')

    def state(self):
        d = self.data
        age = time.monotonic() - self.received
        if (not d.get('ready') or not d.get('simulation_only') or age + d.get('state_age_s', 1.) > .15
                or age + d.get('sensor_age_s', 1.) > .15 or age + d.get('joint_age_s', 1.) > .15):
            raise TimeoutError('Surface TCP / force state is unavailable or stale')
        values = [np.asarray(d[key], float) for key in ('position', 'rotation', 'wrench', 'fast_wrench', 'velocity')]
        if (tuple(v.shape for v in values) != ((3,), (3, 3), (6,), (6,), (6,))
                or not np.isfinite(np.concatenate([v.ravel() for v in values])).all()):
            raise ValueError('Invalid surface state dimensions / values')
        return State(int(d['sequence']), float(d['sampled_at']), values[0], values[1],
                     values[2], values[3], values[4], d.get('control', {}))
