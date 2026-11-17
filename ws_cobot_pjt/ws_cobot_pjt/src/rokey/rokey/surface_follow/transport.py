"""ROS transport for the simulation-only native stream; no synchronous per-sample services."""
import json
import time

import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .controllers import State


class VirtualTransport:
    def __init__(self, program):
        self.program, self.robot, self.node = program, program.robot, program.node
        self.data, self.received = {}, 0.
        self.bias = np.zeros(6)
        self.run_id = ''
        self.sequence = 0
        self.publisher = self.node.create_publisher(String, 'surface_follow/setpoint', 1)
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
                     values[2] - self.bias, values[3] - self.bias, values[4], d.get('control', {}))

    def tare(self, samples=20):
        values, previous = [], None
        end = time.monotonic() + 3
        while len(values) < samples:
            self.spin(.02)
            state = self.state()
            if state.sequence != previous:
                previous = state.sequence
                values.append(np.asarray(self.data['fast_wrench'], float))
            if time.monotonic() >= end:
                raise TimeoutError('Not enough fresh free-space tare samples')
        self.bias = np.mean(values, axis=0)
        if np.linalg.norm(self.bias[:3]) > 5:
            raise RuntimeError('Free-space tare is too large; check the probe / payload / clearance')

    def wait_still(self, seconds=3):
        deadline, stable = time.monotonic() + seconds, None
        while time.monotonic() < deadline:
            self.spin(.02)
            state = self.state()
            still = np.linalg.norm(state.velocity[:3]) < .0008 and np.linalg.norm(state.velocity[3:]) < .01
            stable = (stable or time.monotonic()) if still else None
            if stable is not None and time.monotonic() - stable > .15:
                return state
        raise TimeoutError('Actual Gazebo TCP did not become stationary')

    def begin(self, method):
        response = self.robot.request(f'/dsr01/surface_follow/{method}', Trigger)
        self.run_id = json.loads(response.message)['run_id']
        # States already in flight still carry the previous (stopped) run_id; own the stream only once the
        # native side reports this run, well inside its 150 ms command watchdog.
        deadline = time.monotonic() + .1
        while self.data.get('run_id') != self.run_id or not self.data.get('active'):
            self.spin(.005)
            if time.monotonic() > deadline:
                raise TimeoutError('Native surface stream did not report the new run')

    def send(self, command, radius):
        self.sequence += 1
        payload = {**command, 'run_id': self.run_id, 'sequence': self.sequence,
                   'stamp_ns': self.node.get_clock().now().nanoseconds,
                   'radius': radius, 'bias': self.bias.tolist()}
        self.publisher.publish(String(data=json.dumps(payload)))

    def check_owner(self):
        if self.data.get('run_id') == self.run_id and not self.data.get('active'):
            raise RuntimeError(self.data.get('reason') or 'Native surface stream stopped')
        if self.data.get('run_id') not in ('', self.run_id):
            raise RuntimeError('Native surface stream ownership changed')

    def end(self):
        if not self.run_id:
            return False
        if self.data.get('active') and self.data.get('run_id') not in ('', self.run_id):
            raise RuntimeError('Will not stop a different active surface stream owner')
        previous = self.robot.timeout
        try:
            self.robot.timeout = 10.
            self.robot.request('/dsr01/surface_follow/end', Trigger)
            after, deadline = self.data.get('sequence', 0), time.monotonic() + 2
            while time.monotonic() < deadline:
                self.spin(.02)
                state = self.state()
                if (state.sequence > after and not self.data.get('active')
                        and np.linalg.norm(state.velocity[:3]) < .0008
                        and np.linalg.norm(state.velocity[3:]) < .01):
                    self.run_id = ''
                    return True
            return False
        finally:
            self.robot.timeout = previous
