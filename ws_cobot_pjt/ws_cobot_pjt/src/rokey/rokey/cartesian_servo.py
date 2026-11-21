"""Cartesian position and force servo shared by simulation and the real stream."""
from dataclasses import asdict, dataclass
import time

import numpy as np


class StaleSetpoint(ValueError):
    """A late or out-of-order setpoint."""


def skew(vector):
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def rotation(vector):
    angle = np.linalg.norm(vector)
    if angle < 1e-10:
        return np.eye(3) + skew(vector)
    axis = skew(vector / angle)
    return np.eye(3) + np.sin(angle) * axis + (1 - np.cos(angle)) * axis @ axis


def rotation_error(target, current):
    matrix = target @ current.T
    angle = np.arccos(np.clip((np.trace(matrix) - 1) / 2, -1, 1))
    vector = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0],
                       matrix[1, 0] - matrix[0, 1]])
    return vector / 2 if angle < 1e-7 else vector * angle / (2 * np.sin(angle))


def bounded(vector, limit):
    return vector * min(1., limit / max(np.linalg.norm(vector), 1e-12))


@dataclass
class PDSettings:
    enabled: bool = False
    position_kp: float = 1.
    position_kd_s: float = .04
    orientation_kp: float = 1.
    orientation_kd_s: float = .04
    derivative_filter_s: float = .03
    position_feedback_speed: float = .002
    angular_feedback_limit: float = .03490658503988659
    angular_feedback_speed: float = .06981317007977318

    @classmethod
    def parse(cls, packet=None):
        try:
            settings = cls(**(packet or {}))
        except TypeError as error:
            raise ValueError('Unknown PD configuration field; integral control is not implemented') from error
        values = np.array([v for k, v in asdict(settings).items() if k != 'enabled'], float)
        if (not np.isfinite(values).all() or np.any(values < 0) or settings.derivative_filter_s <= 0
                or min(settings.position_feedback_speed, settings.angular_feedback_limit,
                       settings.angular_feedback_speed) <= 0):
            raise ValueError('PD gains must be finite/nonnegative; filter and output limits must be positive')
        return settings


class VectorPD:
    def __init__(self, filter_s):
        self.filter_s = filter_s
        self.measured_rate = None
        self.p, self.d, self.offset, self.offset_rate = (np.zeros(3) for _ in range(4))
        self.limited = False

    def update(self, error, reference_rate, measured_rate, dt, kp, kd, limit, speed):
        if self.measured_rate is None:
            self.measured_rate = measured_rate.copy()
        self.measured_rate += (1 - np.exp(-dt / self.filter_s)) * (measured_rate - self.measured_rate)
        self.p = kp * error
        self.d = kd * (reference_rate - self.measured_rate)
        demand = self.p + self.d
        target = bounded(demand, limit)
        increment = target - self.offset
        self.limited = bool(np.linalg.norm(demand) > limit or np.linalg.norm(increment) > speed * dt)
        self.offset_rate = bounded(increment / dt, speed)
        self.offset += self.offset_rate * dt
        return self.offset


class PosePD:
    def __init__(self, settings):
        self.settings = settings
        self.position = VectorPD(settings.derivative_filter_s)
        self.orientation = VectorPD(settings.derivative_filter_s)
        self.error, self.velocity_error = np.zeros(6), np.zeros(6)

    def update(self, reference_p, reference_r, reference_v, reference_w, actual_pose, actual_twist,
               stiffness_scale, position_limit, dt):
        c = self.settings
        self.error = np.r_[reference_p - actual_pose[:3, 3], rotation_error(reference_r, actual_pose[:3, :3])]
        position = self.position.update(self.error[:3], reference_v, actual_twist[:3], dt,
                                        c.position_kp * (stiffness_scale - 1.), c.position_kd_s,
                                        position_limit, c.position_feedback_speed)
        orientation = self.orientation.update(self.error[3:], reference_w, actual_twist[3:], dt,
                                              c.orientation_kp, c.orientation_kd_s,
                                              c.angular_feedback_limit, c.angular_feedback_speed)
        self.velocity_error = np.r_[reference_v - self.position.measured_rate,
                                    reference_w - self.orientation.measured_rate]
        return position, orientation

    def diagnostics(self):
        return dict(pd_enabled=True, pd_error=self.error.tolist(), pd_velocity_error=self.velocity_error.tolist(),
                    pd_p=np.r_[self.position.p, self.orientation.p].tolist(),
                    pd_d=np.r_[self.position.d, self.orientation.d].tolist(),
                    pd_offset=np.r_[self.position.offset, self.orientation.offset].tolist(),
                    pd_saturated=self.position.limited or self.orientation.limited)


PRESS_LIMIT = .005  # admittance offset along the TCP axis beyond the path, m: a stop (lifted, then resumed)


class PositionController:
    def __init__(self, stream, pd=None):
        self.stream = stream
        pose = stream.chain.pose_jacobian(stream.backend.feedback)[0]
        self.position, self.orientation = pose[:3, 3].copy(), pose[:3, :3].copy()
        self.goal, self.goal_rotation = self.position.copy(), self.orientation.copy()
        self.velocity, self.omega, self.feed = np.zeros(3), np.zeros(3), np.zeros(6)
        self.joint_velocity = np.zeros(6)
        self.linear_speed, self.linear_acceleration = .02, .05
        self.angular_speed, self.angular_acceleration = .15, .3
        self.joint_speed, self.joint_acceleration = .35, .7
        self.stiffness_scale, self.feedback_limit = 1., .0005
        self.tracking_offset = np.zeros(3)
        self.pd = PosePD(pd or PDSettings())
        self.command_position, self.command_orientation = self.position.copy(), self.orientation.copy()
        self.command_velocity, self.command_omega = np.zeros(3), np.zeros(3)
        self.ball_radius = 0.
        self.force_target = self.press = self.press_velocity = 0.  # N; m and m/s along the TCP +z axis

    def accept(self, packet):
        s = self.stream
        seq, source = int(packet['sequence']), int(packet['source_sequence'])
        age = time.monotonic() - float(packet['source_sampled_at'])
        if seq <= s.packet_sequence or not 0 <= s.sequence - source <= 30 or not 0 <= age <= s.timeout:
            raise StaleSetpoint('Expired or reordered Cartesian setpoint')
        p, r = np.asarray(packet['position'], float), np.asarray(packet['rotation'], float)
        feed = np.asarray(packet['feedforward'], float)
        stiffness = float(packet.get('position_stiffness_scale', 1.))
        feedback_limit = float(packet.get('position_feedback_limit', .0005))
        if not np.isfinite([stiffness, feedback_limit]).all() or not 1 <= stiffness <= 10 or not 0 < feedback_limit <= .002:
            raise ValueError('Invalid position stiffness scale or correction limit')
        limits = np.array([packet[key] for key in
                          ('linear_speed', 'linear_acceleration', 'angular_speed', 'angular_acceleration',
                           'joint_speed', 'joint_acceleration')], float)
        if (p.shape != (3,) or r.shape != (3, 3) or feed.shape != (6,)
                or not np.isfinite(np.r_[p, r.ravel(), feed, limits]).all()
                or np.any(limits <= 0) or np.any(limits > [.05, .2, 1.047, 2.094, .5, 1.])
                or np.linalg.norm(r.T @ r - np.eye(3)) > 1e-5 or np.linalg.det(r) < .999):
            raise ValueError('Invalid Cartesian pose, feedforward or motion limits')
        if np.linalg.norm(feed[3:]) > limits[2] + 1e-8:
            raise ValueError('Cartesian feedforward exceeds the requested limits')
        force = float(packet.get('contact_force', 0.))
        if not np.isfinite(force) or not 0 <= force <= 5:
            raise ValueError('Contact force target must lie in [0, 5] N')
        radius = float(packet.get('ball_radius', 0.))
        if not np.isfinite(radius) or not 0 <= radius <= .02:
            raise ValueError('Invalid sphere pivot radius')
        if s.packet_sequence < 0:
            self.position -= radius * self.orientation[:, 2]
            self.command_position = self.position.copy()
            self.ball_radius = radius
        elif radius != self.ball_radius:
            raise ValueError('Sphere pivot cannot change during detection')
        p = p - radius * r[:, 2]
        feed = feed.copy()
        feed[:3] -= radius * np.cross(feed[3:], r[:, 2])
        if np.linalg.norm(feed[:3]) > limits[0] + 1e-8:
            raise ValueError('Sphere centre feedforward exceeds the requested speed')
        actual = np.asarray(s.state['position']) - radius * np.asarray(s.state['rotation'])[:, 2]
        if np.linalg.norm(p - actual) > .015:
            raise ValueError('Cartesian target exceeds the local 15 mm workspace')
        self.goal, self.goal_rotation, self.feed = p, r, feed
        self.stiffness_scale, self.feedback_limit, self.force_target = stiffness, feedback_limit, force
        (self.linear_speed, self.linear_acceleration, self.angular_speed, self.angular_acceleration,
         self.joint_speed, self.joint_acceleration) = limits
        s.packet_sequence, s.last_command = seq, time.monotonic()

    def step(self, actual, dt):
        s, b = self.stream, self.stream.backend
        if not s.motion_ready() or b.clock() - b.joint_time > .1:
            return self.stop('Cartesian joint feedback expired')
        if time.monotonic() - s.last_command > s.timeout:
            return self.stop('Cartesian command watchdog expired')
        if any(abs(b.nominal[name] - s.nominal[name]) > .003 for name in s.chain.names):
            return self.stop('Another program changed the nominal joints during Cartesian control')
        wanted = bounded(self.feed[:3] + 12 * (self.goal - self.position), self.linear_speed)
        if getattr(s, 'approach_speed', None) and not s.press_latched:
            axis = self.orientation[:, 2]
            normal_speed = wanted @ axis
            wanted += (float(np.clip(normal_speed, -s.approach_speed, s.approach_speed)) - normal_speed) * axis
        self.velocity += bounded(wanted - self.velocity, self.linear_acceleration * dt)
        self.position += self.velocity * dt
        wanted_omega = bounded(self.feed[3:] + 8 * rotation_error(self.goal_rotation, self.orientation),
                               self.angular_speed)
        self.omega += bounded(wanted_omega - self.omega, self.angular_acceleration * dt)
        self.orientation = rotation(self.omega * dt) @ self.orientation
        pose, actual_jacobian = s.chain.pose_jacobian(actual)
        pose = pose.copy()
        actual_jacobian = actual_jacobian.copy()
        if self.ball_radius:
            z = pose[:3, 2].copy()
            pose[:3, 3] -= self.ball_radius * z
            actual_jacobian[:3] += self.ball_radius * np.cross(z, actual_jacobian[3:].T).T
        axis = getattr(s, 'fixed_force_axis', None)
        if axis is None:
            axis = (self.command_orientation if self.pd.settings.enabled else self.orientation)[:, 2]
        pressing = float(-s.filtered_wrench[:3] @ axis)
        if self.force_target:
            effort = self.force_target - pressing
            self.press_velocity += (effort - b.minimum_damping[2] * self.press_velocity) / b.mass[2] * dt
            self.press_velocity = float(np.clip(self.press_velocity, -b.maximum_velocity[2], b.maximum_velocity[2]))
            self.press += self.press_velocity * dt
            if abs(self.press) > PRESS_LIMIT:
                return self.stop(f'Contact force offset exceeded {PRESS_LIMIT * 1000:.0f} mm along the TCP axis')
        reference = self.position + self.press * axis
        s.tracking_error = float(np.linalg.norm(reference - pose[:3, 3]))
        if s.tracking_error > .01:
            return self.stop('Cartesian TCP tracking error exceeded 10 mm')
        # With a force target only the admittance acts along the TCP axis, as the DSR force control: the position
        # PD / restoration see the error across it (a position loop along it too stiffened the contact into a bounce).
        across = (lambda vector: vector - axis * (vector @ axis)) if self.force_target else (lambda vector: vector)
        qd = np.array([b.feedback_velocity.get(name, 0.) for name in s.chain.names])
        twist = actual_jacobian @ qd
        twist[:3] = self.velocity + across(twist[:3] - self.velocity)
        pd_diagnostics = dict(pd_enabled=False)
        command_limited = False
        if self.pd.settings.enabled:
            seen = pose.copy()
            seen[:3, 3] = reference - across(reference - pose[:3, 3])
            offset, turn = self.pd.update(reference, self.orientation, self.velocity, self.omega,
                                         seen, twist, self.stiffness_scale, self.feedback_limit, dt)
            self.tracking_offset = offset.copy()
            target_p = reference + offset
            target_r = rotation(turn) @ self.orientation
            wanted_w = self.omega + 4 * rotation_error(target_r, self.command_orientation)
            command_limited = (np.linalg.norm(wanted_w) > self.angular_speed or
                               np.linalg.norm(bounded(wanted_w, self.angular_speed) - self.command_omega) >
                               self.angular_acceleration * dt)
            self.command_omega += bounded(bounded(wanted_w, self.angular_speed) - self.command_omega,
                                           self.angular_acceleration * dt)
            # The position correction is already slew-limited. A second integrated position/acceleration
            # loop delayed its damping and generated a ~3 Hz limit cycle in the virtual robot.
            self.command_velocity = (target_p - self.command_position) / dt
            self.command_position = target_p
            self.command_orientation = rotation(self.command_omega * dt) @ self.command_orientation
            command_p, command_r = self.command_position, self.command_orientation
            pd_diagnostics = self.pd.diagnostics()
        else:
            restoration = bounded((self.stiffness_scale - 1.) * across(reference - pose[:3, 3]), self.feedback_limit)
            self.tracking_offset += bounded(restoration - self.tracking_offset, min(self.linear_speed, .002) * dt)
            command_p, command_r = reference + self.tracking_offset, self.orientation
        # Re-aim the gripper around the ball, keeping its centre on the translation path.
        command_p = command_p + self.ball_radius * command_r[:, 2]
        command_pose, jacobian = s.chain.pose_jacobian({**actual, **s.q})
        error = np.r_[command_p - command_pose[:3, 3], rotation_error(command_r, command_pose[:3, :3])]
        inverse = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + np.eye(6) * 1e-5, np.eye(6))
        angular_velocity = self.command_omega if self.pd.settings.enabled else self.omega
        tcp_velocity = (self.velocity + self.press_velocity * axis
                        + self.ball_radius * np.cross(angular_velocity, command_r[:, 2]))
        # Follow the path velocity directly. Solving pose_error/dt alone requests an instantaneous joint
        # arrival; the acceleration limiter then overshoots and reverses around each moving target.
        joint_feed = inverse @ np.r_[tcp_velocity, angular_velocity]
        desired = joint_feed + 8 * (inverse @ error)
        wanted_joints = np.clip(desired, -self.joint_speed, self.joint_speed)
        joint_limited = bool(np.any(np.abs(desired) > self.joint_speed) or
                             np.any(np.abs(wanted_joints - self.joint_velocity) > self.joint_acceleration * dt))
        self.joint_velocity += np.clip(wanted_joints - self.joint_velocity,
                                       -self.joint_acceleration * dt, self.joint_acceleration * dt)
        candidate = {name: s.q[name] + self.joint_velocity[i] * dt for i, name in enumerate(s.chain.names)}
        if any(not s.chain.limits[name][0] <= value <= s.chain.limits[name][1]
               for name, value in candidate.items()):
            return self.stop('Cartesian joint position limit reached')
        s.q = candidate
        s.control = dict(controller='mesh-normal', force_feedback_used=bool(self.force_target),
                         contact_force_target_n=self.force_target, contact_force_n=pressing,
                         contact_offset_mm=self.press * 1000,
                         position_stiffness_scale=self.stiffness_scale,
                         tracking_offset_mm=(self.tracking_offset * 1000).tolist(),
                         angular_tracking_error_rad=float(np.linalg.norm(
                             rotation_error(self.orientation, pose[:3, :3]))), **pd_diagnostics)
        s.control['pd_saturated'] = bool(pd_diagnostics.get('pd_saturated', False) or command_limited or joint_limited)
        s.control.update(
            reference_to_goal_angle_rad=float(np.linalg.norm(rotation_error(self.goal_rotation, self.orientation))),
            command_to_reference_angle_rad=float(np.linalg.norm(rotation_error(self.orientation, command_r))),
            command_to_actual_angle_rad=float(np.linalg.norm(rotation_error(command_r, pose[:3, :3]))),
            command_to_actual_position_m=(command_p - self.ball_radius * command_r[:, 2] - pose[:3, 3]).tolist(),
            reference_omega=self.omega.tolist(), command_omega=angular_velocity.tolist(),
            joint_command_error=[candidate[name] - actual[name] for name in s.chain.names],
            pd_position_limited=self.pd.position.limited, pd_orientation_limited=self.pd.orientation.limited,
            cartesian_limited=bool(command_limited), joint_limited=joint_limited)
        return {**b.nominal, **s.q}

    def stop(self, reason):
        self.stream.halt(reason)
        b = self.stream.backend
        return {**b.nominal, **b.hold}
