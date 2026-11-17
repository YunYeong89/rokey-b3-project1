"""Native ProbeTCP pose servo. Force measurements never enter this controller."""
import time

import numpy as np

from kinematics import rotation, rotation_error


def bounded(value, limit):
    return value * min(1., limit / max(np.linalg.norm(value), 1e-12))


class PositionController:
    def __init__(self, stream):
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

    def accept(self, packet):
        s = self.stream
        seq, source = int(packet['sequence']), int(packet['source_sequence'])
        age = time.monotonic() - float(packet['source_sampled_at'])
        if seq <= s.packet_sequence or not 0 <= s.sequence - source <= 30 or not 0 <= age <= s.timeout:
            from surface_stream import StaleSetpoint
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
        if (np.linalg.norm(feed[:3]) > limits[0] + 1e-8
                or np.linalg.norm(feed[3:]) > limits[2] + 1e-8):
            raise ValueError('Cartesian feedforward exceeds the requested limits')
        actual = np.asarray(s.state['position'])
        if np.linalg.norm(p - actual) > .015:
            raise ValueError('Cartesian target exceeds the local 15 mm workspace')
        self.goal, self.goal_rotation, self.feed = p, r, feed
        self.stiffness_scale, self.feedback_limit = stiffness, feedback_limit
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
        self.velocity += bounded(wanted - self.velocity, self.linear_acceleration * dt)
        self.position += self.velocity * dt
        wanted_omega = bounded(self.feed[3:] + 8 * rotation_error(self.goal_rotation, self.orientation),
                               self.angular_speed)
        self.omega += bounded(wanted_omega - self.omega, self.angular_acceleration * dt)
        self.orientation = rotation(self.omega * dt) @ self.orientation
        pose, _ = s.chain.pose_jacobian(actual)
        s.tracking_error = float(np.linalg.norm(self.position - pose[:3, 3]))
        if s.tracking_error > .01:
            return self.stop('Cartesian TCP tracking error exceeded 10 mm')
        restoration = bounded((self.stiffness_scale - 1.) * (self.position - pose[:3, 3]), self.feedback_limit)
        self.tracking_offset += bounded(restoration - self.tracking_offset, min(self.linear_speed, .002) * dt)
        command_pose, jacobian = s.chain.pose_jacobian({**actual, **s.q})
        error = np.r_[self.position + self.tracking_offset - command_pose[:3, 3],
                      rotation_error(self.orientation, command_pose[:3, :3])]
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + np.eye(6) * 1e-5, error)
        wanted_joints = np.clip(delta / dt, -self.joint_speed, self.joint_speed)
        self.joint_velocity += np.clip(wanted_joints - self.joint_velocity,
                                       -self.joint_acceleration * dt, self.joint_acceleration * dt)
        candidate = {name: s.q[name] + self.joint_velocity[i] * dt for i, name in enumerate(s.chain.names)}
        if any(not s.chain.limits[name][0] <= value <= s.chain.limits[name][1]
               for name, value in candidate.items()):
            return self.stop('Cartesian joint position limit reached')
        s.q = candidate
        s.control = dict(controller='mesh-normal', force_feedback_used=False,
                         position_stiffness_scale=self.stiffness_scale,
                         tracking_offset_mm=(self.tracking_offset * 1000).tolist(),
                         angular_tracking_error_rad=float(np.linalg.norm(
                             rotation_error(self.orientation, pose[:3, :3]))))
        return {**b.nominal, **s.q}

    def stop(self, reason):
        self.stream.halt(reason)
        b = self.stream.backend
        return {**b.nominal, **b.hold}
