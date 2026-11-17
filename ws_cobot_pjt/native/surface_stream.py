"""Optional native-Gazebo Cartesian stream. No DRCF realtime or real-robot commands.

Hybrid (rokey defect_scan) only: tangential position and a virtual inner normal force regulator.
"""
import time
from uuid import uuid4

import numpy as np

from kinematics import KinematicChain, rotation, rotation_error
from normal_force import NormalForceServo


class StaleSetpoint(ValueError):
    """A late or out-of-order setpoint: dropped, the stream keeps running. Halting on one such packet aborted whole
    scans after a single delivery hiccup; the watchdog still halts when no valid setpoint arrives for `timeout`."""


def bounded(vector, maximum):
    return vector * min(1., maximum / max(np.linalg.norm(vector), 1e-12))


class CartesianStream:
    timeout = .15

    def __init__(self, backend, chain=None):
        self.backend = backend
        self.chain = chain or KinematicChain(backend.model.robot,
                            tcp_link='probe_tip' if backend.probe else backend.chain.tcp_link)
        self.active = False
        self.run_id = self.method = self.reason = ''
        self.sequence = 0
        self.sampled_at = self.last_command = 0.
        self.state = {}
        self.packet_sequence = -1
        self.radius = .0015
        self.bias = np.zeros(6)
        self.tracking_error = 0.
        self.control = {}
        self.dropped = 0

    def observe(self, actual, pose, jacobian, now):
        b = self.backend
        probe_pose, probe_jacobian = self.chain.pose_jacobian(actual)
        offset = probe_pose[:3, 3] - pose[:3, 3]
        self.filtered_wrench = b.wrench.copy()
        self.fast_wrench = b.compensated_wrench.copy()
        self.filtered_wrench[3:] -= np.cross(offset, self.filtered_wrench[:3])
        self.fast_wrench[3:] -= np.cross(offset, self.fast_wrench[:3])
        self.sequence += 1
        self.sampled_at = now
        qd = np.array([b.feedback_velocity.get(name, 0.) for name in self.chain.names])
        self.state = dict(sequence=self.sequence, sampled_at=now, position=probe_pose[:3, 3].tolist(),
                          wall_sampled_at=time.monotonic(),
                          joint_positions=[float(actual[name]) for name in self.chain.names],
                          rotation=probe_pose[:3, :3].tolist(), velocity=(probe_jacobian @ qd).tolist(),
                          wrench=self.filtered_wrench.tolist(), fast_wrench=self.fast_wrench.tolist())

    def begin(self, method):
        b = self.backend
        self.timeout = .3 if method == 'position' else type(self).timeout
        if method not in ('hybrid', 'position'):
            raise ValueError('Unknown Cartesian stream method')
        if self.active or b.mode != 'following':
            raise ValueError('Release the current controller before starting a surface stream')
        ready = self.motion_ready() if method == 'position' else b.ready()
        if not ready or b.probe is None or not b.probe.design.attached:
            raise ValueError('An attached probe and fresh Gazebo feedback are required')
        clock = b.clock if method == 'position' else time.monotonic
        if not self.state or clock() - b.joint_time > .1:
            raise ValueError('Actual joint feedback is stale')
        if max(abs(b.feedback_velocity.get(name, 1.)) for name in b.chain.names) > .02:
            raise ValueError('Robot must be stationary before starting a surface stream')
        self.method, self.reason, self.run_id = method, '', uuid4().hex
        self.active = True
        self.last_command = time.monotonic()
        self.packet_sequence = -1
        self.dropped = 0
        self.pose = self.chain.pose_jacobian(b.feedback)[0]
        self.reference = self.pose[:3, 3] - self.radius * self.pose[:3, 2]
        self.orientation = self.pose[:3, :3].copy()
        self.goal = self.reference.copy()
        self.goal_rotation = self.orientation.copy()
        self.velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.angular_speed, self.angular_acceleration = 1.047, 2.094
        self.force = None
        self.force_velocity = self.applied_force = 0.
        self.regulating = False
        self.normal_servo = None
        self.pid_controller = None
        self.raw_peak = 0.
        self.control = {}
        self.overload = 0.
        self.bias = np.zeros(6)
        self.q = {name: b.feedback[name] for name in b.chain.names}
        self.nominal = {name: b.nominal[name] for name in b.chain.names}
        self.position_controller = None
        if method == 'position':
            from position_stream import PositionController
            self.position_controller = PositionController(self)
        b.mode, b.error = 'surface_' + method, ''
        return {'success': True, 'run_id': self.run_id, 'simulation_only': True}

    def accept(self, packet):
        if not self.active or packet.get('run_id') != self.run_id:
            raise ValueError('Surface stream ownership mismatch')
        if self.method == 'position':
            return self.position_controller.accept(packet)
        seq, source = int(packet['sequence']), int(packet['source_sequence'])
        age = time.monotonic() - float(packet['source_sampled_at'])
        if seq <= self.packet_sequence or not 0 <= self.sequence - source <= 15 or not 0 <= age <= self.timeout:
            raise StaleSetpoint('Expired or reordered surface setpoint')
        p, r = np.asarray(packet['position'], float), np.asarray(packet['rotation'], float)
        bias = np.asarray(packet['bias'], float)
        radius = float(packet['radius'])
        force = packet.get('force')
        angular_speed = float(packet.get('angular_speed', 1.047))
        angular_acceleration = float(packet.get('angular_acceleration', 2.094))
        if not (0 < angular_speed <= 1.047 and 0 < angular_acceleration <= 2.094):
            raise ValueError('Invalid surface angular limits')
        if (p.shape != (3,) or r.shape != (3, 3) or bias.shape != (6,)
                or not np.isfinite(np.r_[p, r.ravel(), bias, radius]).all()
                or not .0001 <= radius <= .01 or np.linalg.norm(bias[:3]) > 5
                or np.linalg.norm(r.T @ r - np.eye(3)) > 1e-5 or np.linalg.det(r) < .999):
            raise ValueError('Invalid surface pose / tare / radius')
        if force is not None and (self.method != 'hybrid' or not np.isfinite(force) or not 0 <= force <= 5):
            raise ValueError('Normal force target is supported only in hybrid mode, in [0, 5] N')
        centre = p - radius * r[:, 2]
        actual = np.asarray(self.state['position']) - radius * np.asarray(self.state['rotation'])[:, 2]
        if np.linalg.norm(centre - actual) > .015:
            raise ValueError('Surface reference exceeds the local workspace')
        # Radius changes are allowed only on the first packet, before any motion.
        if self.packet_sequence < 0:
            self.reference += (self.radius - radius) * self.orientation[:, 2]
        elif abs(radius - self.radius) > 1e-9:
            raise ValueError('Probe radius cannot change during a run')
        self.radius, self.bias = radius, bias
        self.goal, self.goal_rotation, self.force = centre, r, force
        self.angular_speed, self.angular_acceleration = angular_speed, angular_acceleration
        if packet.get('controller') == 'pid':
            if self.backend.__class__.__name__ != 'MuJoCoForceBackend':
                raise ValueError('PID surface controller is restricted to MuJoCo')
            if self.pid_controller is None:
                from rokey.pid_controller import SurfacePID
                self.pid_controller = SurfacePID(self)
            self.pid_controller.accept(p, packet)
        self.packet_sequence, self.last_command = seq, time.monotonic()

    def halt(self, reason):
        b = self.backend
        self.active, self.reason = False, reason
        b.hold = {name: b.feedback[name] for name in b.chain.names}
        released = reason in ('Surface stream released by operator', 'Cartesian stream released by operator')
        b.mode, b.error = 'holding', '' if released else reason
        return {'success': True, 'sync_q': [b.hold[name] for name in b.chain.names]}

    def step(self, actual, pose, jacobian, dt, now):
        if self.method == 'position':
            return self.position_controller.step(actual, dt)
        b = self.backend
        pose, jacobian = self.chain.pose_jacobian(actual)
        reason = ''
        # A single 200 Hz sample of Gazebo's sphere contact can spike past 8 N while the 40 Hz trace stays near
        # 1 N: 8 N must last two control steps; 20 N stops at once.
        force = float(np.linalg.norm(self.fast_wrench[:3] - self.bias[:3]))
        self.raw_peak = max(self.raw_peak, force)
        self.overload = self.overload + dt if force > 8 else 0.
        if not b.ready() or now - b.joint_time > .1:
            reason = 'Surface feedback expired'
        elif now - self.last_command > self.timeout:
            reason = 'Surface command watchdog expired'
        elif force > 20 or self.overload >= .02 - 1e-9:
            reason = f'Surface measured force exceeded {"20 N" if force > 20 else "8 N for 20 ms"}'
        elif any(abs(b.nominal[name] - self.nominal[name]) > .003 for name in b.chain.names):
            reason = 'Another robot program changed the nominal joints during surface control'
        if reason:
            self.halt(reason)
            return {**b.nominal, **b.hold}
        if self.pid_controller is not None:
            return self.pid_controller.step(actual, pose, jacobian, dt)
        target_velocity = bounded(20 * (self.goal - self.reference), .01)
        self.velocity += bounded(target_velocity - self.velocity, .1 * dt)
        increment = self.velocity * dt
        if self.method == 'hybrid' and self.force is not None:
            n = -pose[:3, 2]  # Actual TOOL axis; never an instantaneous ideal normal.
            increment -= n * (increment @ n)
            # Tick-averaged (not the 80 ms filtered) force: the servo's Kalman filter does the smoothing.
            fn = float((self.fast_wrench[:3] - self.bias[:3]) @ n)
            centre = pose[:3, 3] - self.radius * pose[:3, 2]
            qd = np.array([b.feedback_velocity.get(name, 0.) for name in self.chain.names])
            twist = jacobian @ qd
            centre_velocity = twist[:3] - self.radius * np.cross(twist[3:], pose[:3, 2])
            if not self.regulating:
                self.normal_servo, self.regulating = NormalForceServo(fn), True
            normal_error = float((self.reference - centre) @ n)
            correction = self.normal_servo.update(fn,
                                  float(centre_velocity @ n), self.force, normal_error, dt)
            increment += n * correction
            self.control = {**self.normal_servo.diagnostics(), 'normal_reference_error_m': normal_error}
        else:
            self.force_velocity = self.applied_force = 0.
            self.regulating = False
            self.control = {}
        self.reference += increment
        desired_omega = bounded(8 * rotation_error(self.goal_rotation, self.orientation), self.angular_speed)
        self.angular_velocity += bounded(desired_omega - self.angular_velocity, self.angular_acceleration * dt)
        self.orientation = rotation(self.angular_velocity * dt) @ self.orientation
        self.control.update(raw_peak_n=self.raw_peak,
            angular_tracking_error_rad=float(np.linalg.norm(rotation_error(self.orientation, pose[:3, :3]))),
            joint_tracking_error_rad=max(abs(self.q[name] - actual[name]) for name in self.chain.names))
        target = self.reference + self.radius * self.orientation[:, 2]
        self.tracking_error = float(np.linalg.norm(target - pose[:3, 3]))
        if self.tracking_error > .01:
            self.halt('Surface actual TCP tracking error exceeded 10 mm')
            return {**b.nominal, **b.hold}
        commanded_pose, j = self.chain.pose_jacobian({**actual, **self.q})
        error = np.r_[target - commanded_pose[:3, 3],
                      rotation_error(self.orientation, commanded_pose[:3, :3])]
        dq = j.T @ np.linalg.solve(j @ j.T + np.eye(6) * 1e-5, error)
        dq = np.clip(dq, -.5 * dt, .5 * dt)
        candidate = {name: self.q[name] + dq[i] for i, name in enumerate(b.chain.names)}
        if any(not b.chain.limits[name][0] <= value <= b.chain.limits[name][1]
               for name, value in candidate.items()):
            self.halt('Surface joint position limit reached')
            return {**b.nominal, **b.hold}
        self.q = candidate
        return {**b.nominal, **self.q}

    def snapshot(self):
        now = time.monotonic()
        return {**self.state, 'active': self.active, 'method': self.method, 'run_id': self.run_id,
                'control': {**self.control, 'raw_peak_n': getattr(self, 'raw_peak', 0.)},
                'tcp_link': self.chain.tcp_link,
                'reason': self.reason, 'simulation_only': True, 'tracking_error_m': self.tracking_error,
                'dropped_setpoints': self.dropped,
                'state_age_s': self.backend.clock() - self.sampled_at,
                'joint_age_s': self.backend.clock() - self.backend.joint_time,
                'sensor_age_s': self.backend.clock() - self.backend.sensor_time}

    def motion_ready(self):
        """Pose streaming depends on joint feedback, never on force values or sensor freshness."""
        b = self.backend
        return (all(name in b.feedback and name in b.nominal for name in self.chain.names)
                and b.nominal_available and not b.closed.is_set()
                and b.clock() - b.nominal_time < .5 and b.clock() - b.joint_time < .1)
