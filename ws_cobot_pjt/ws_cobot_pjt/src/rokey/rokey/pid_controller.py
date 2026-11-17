"""MuJoCo surface PID: native force/orientation loop and ROS path adapter in one module.

The TCP path owns translation; orientation correction rotates about that same TCP.
Baseline data is measured on a plane, not treated as ground truth.
"""
from collections import deque
from datetime import datetime
import csv
import json
import time

import numpy as np

from .configs import PACKAGE
from .rotations import align, angles_zyz, axis_angle, unit

RESULTS = PACKAGE / 'images'
REFERENCE = RESULTS / 'pid_force_reference.json'
STIFFNESS = [3000., 3000., 100., 200., 200., 200.]


def bounded(value, limit):
    value = np.asarray(value, float)
    return value * min(1., limit / max(np.linalg.norm(value), 1e-12))


def rotation_error(target, actual):
    matrix = target @ actual.T
    angle = np.arccos(np.clip((np.trace(matrix) - 1) / 2, -1, 1))
    vector = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]])
    return vector / 2 if angle < 1e-6 else vector * angle / (2 * np.sin(angle))


class PID:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, integral_limit
        self.integral = self.derivative = 0.

    def update(self, error, measured_rate, dt, limit, active=True):
        self.derivative += (1 - np.exp(-dt / .04)) * (measured_rate - self.derivative)
        candidate = np.clip(self.integral + self.ki * error * dt, -self.limit, self.limit) if active else 0.
        demand = self.kp * error + candidate - self.kd * self.derivative
        if np.linalg.norm(demand) < limit or np.sum(error * demand) < 0:
            self.integral = candidate
        return bounded(demand, limit)


class SurfacePID:
    """100 Hz native loop. Contact wins over alignment; nominal feed is never gated on an angle error."""
    def __init__(self, stream):
        self.stream = stream
        stream.timeout = .3  # Local force feedback stays active across brief ROS delivery gaps (3 mm at 10 mm/s).
        pose = stream.chain.pose_jacobian(stream.backend.feedback)[0]
        self.position = pose[:3, 3].copy()
        self.initial_rotation = pose[:3, :3].copy()
        self.orientation = self.initial_rotation.copy()
        self.normal = -pose[:3, 2].copy()
        self.goal = self.position.copy()
        self.force_pid = PID(.00025, .00015, .00001, .0002)
        self.angle_pid = PID(1.5, .01, .4, .01)
        self.history = deque(maxlen=3)
        self.surface_points = deque(maxlen=150)
        self.filtered = np.zeros(3)
        self.last_fn = 0.
        self.omega, self.compliance = np.zeros(3), np.zeros(3)
        self.mu, self.target_force = .0, 1.5
        self.angles = angles_zyz(pose[:3, :3], [0., 180., 0.])
        self.feed_velocity, self.feed = np.zeros(3), np.zeros(3)
        self.feed_clock = time.monotonic()
        self.goal_clock = self.feed_clock
        self.motion_samples = deque()

    def accept(self, position, packet):
        self.goal = position.copy()
        self.target_force = float(packet.get('force') or 0.)
        self.mu = float(packet.get('friction_mu', 0.))
        self.prior = unit(packet.get('normal', -self.initial_rotation[:, 2]))
        self.baseline = packet.get('pid_baseline', False)
        self.feed_velocity = np.asarray(packet.get('feed_velocity', [0., 0., 0.]), float)
        if (self.feed_velocity.shape != (3,) or not np.isfinite(self.feed_velocity).all()
                or np.linalg.norm(self.feed_velocity) > .015):
            raise ValueError('PID feed velocity must be a finite vector below 15 mm/s')
        self.goal_clock = float(packet['source_sampled_at'])

    def step(self, actual, pose, jacobian, dt):
        s, b = self.stream, self.stream.backend
        force = s.fast_wrench[:3] - s.bias[:3]
        self.history.append(force.copy())
        median = np.median(self.history, axis=0)
        self.filtered += (1 - np.exp(-dt / .04)) * (median - self.filtered)
        outward = -pose[:3, 2]
        fn = float(self.filtered @ outward)
        contact = fn > .3
        now = time.monotonic()
        wall_dt = min(now - self.feed_clock, .03)
        self.feed_clock = now
        moving_goal = self.goal + self.feed_velocity * min(now - self.goal_clock, s.timeout)
        feed_delta = moving_goal - self.position
        feed_delta -= self.prior * (feed_delta @ self.prior)
        velocity = self.feed_velocity - self.prior * (self.feed_velocity @ self.prior)
        requested = (velocity + bounded(20 * feed_delta, .002) if np.linalg.norm(velocity) > 0
                     else bounded(20 * feed_delta, .01))
        self.feed += bounded(requested - self.feed, .1 * wall_dt)
        feed = self.feed.copy()
        tangent = feed / max(np.linalg.norm(feed), 1e-9)
        # Remove the learned sliding-friction contribution before estimating the outward reaction direction.
        corrected = self.filtered + self.mu * max(fn, 0.) * tangent
        if contact and not self.baseline and corrected @ self.prior > .3:
            centre = pose[:3, 3] - s.radius * pose[:3, 2]
            if .8 < fn < 2.2 and (not self.surface_points or np.linalg.norm(centre - self.surface_points[-1]) > .0002):
                self.surface_points.append(centre.copy())
            geometric = self.prior
            if len(self.surface_points) > 8:
                points = np.asarray(self.surface_points)
                values, vectors = np.linalg.eigh(np.cov(points.T))
                if values[1] > 2.5e-8:
                    geometric = vectors[:, 0]
                    if geometric @ self.prior < 0:
                        geometric = -geometric
                else:
                    line = vectors[:, -1]
                    projected = self.prior - (self.prior @ line) * line
                    if np.linalg.norm(projected) > .2:
                        geometric = unit(projected)
            estimate = unit(.9 * geometric + .1 * unit(corrected))
            angle = np.arccos(np.clip(estimate @ self.prior, -1, 1))
            if angle > np.deg2rad(25):
                cross = np.cross(self.prior, estimate)
                estimate = axis_angle(unit(cross), np.deg2rad(25)) @ self.prior
            self.normal = unit(self.normal + (1 - np.exp(-dt / .3)) * (estimate - self.normal))
        desired = align(self.initial_rotation, self.prior if self.baseline else self.normal, np.pi)
        moment = s.fast_wrench[3:] - s.bias[3:]
        angular_compliance = bounded(moment / STIFFNESS[3], np.deg2rad(.5))
        desired = axis_angle(angular_compliance / max(np.linalg.norm(angular_compliance), 1e-12),
                             np.linalg.norm(angular_compliance)) @ desired
        error = rotation_error(desired, pose[:3, :3])
        qd = np.array([b.feedback_velocity.get(name, 0.) for name in s.chain.names])
        velocity = jacobian @ qd
        if np.linalg.norm(error) < np.deg2rad(2):
            error[:] = 0.
        omega = self.angle_pid.update(error, velocity[3:], dt, .08, contact)
        self.omega += bounded(omega - self.omega, .2 * dt)
        turn = self.omega * dt
        self.orientation = axis_angle(turn / max(np.linalg.norm(turn), 1e-12), np.linalg.norm(turn)) @ self.orientation
        lead = bounded(rotation_error(self.orientation, pose[:3, :3]), np.deg2rad(.5))
        commanded_rotation = axis_angle(lead / max(np.linalg.norm(lead), 1e-12), np.linalg.norm(lead)) @ pose[:3, :3]
        rate = (fn - self.last_fn) / dt
        self.last_fn = fn
        force_error = self.target_force - fn
        inward = float(self.force_pid.update(force_error, rate, dt, .0008,
                                            contact and abs(force_error) < .3))
        if not contact:
            inward = min(inward, .0005)
        if fn > self.target_force + .75:
            inward = min(inward, 0.)
        if not self.target_force:
            inward = 0.
            feed = bounded(20 * (self.goal - self.position), .005)
        self.position += feed * wall_dt - inward * outward * dt
        lag = float((self.position - pose[:3, 3]) @ outward)
        self.position -= outward * (lag - np.clip(lag, -.0005, .0005))
        tangential_force = self.filtered - self.prior * (self.filtered @ self.prior)
        # Tangential compliance: Kx=Ky=3000 N/m. The force-controlled normal axis has no restoring spring.
        self.compliance += (tangential_force - 3000 * self.compliance) / 800 * dt
        self.compliance = bounded(self.compliance, .0005)
        target = self.position + self.compliance
        # Target translation is independent of commanded_rotation: pivot is the actual probe TCP.
        command_pose, j = s.chain.pose_jacobian({**actual, **s.q})
        delta = np.r_[target - command_pose[:3, 3], rotation_error(commanded_rotation, command_pose[:3, :3])]
        dq = j.T @ np.linalg.solve(j @ j.T + np.eye(6) * 1e-5, delta)
        dq = np.clip(dq, -.5 * dt, .5 * dt)
        candidate = {name: s.q[name] + dq[i] for i, name in enumerate(s.chain.names)}
        if any(not s.chain.limits[name][0] <= value <= s.chain.limits[name][1] for name, value in candidate.items()):
            s.halt('PID joint position limit reached')
            return {**b.nominal, **b.hold}
        s.q = candidate
        self.angles = angles_zyz(desired, self.angles)
        self.motion_samples.append((now, pose[:3, 3].copy()))
        while len(self.motion_samples) > 2 and now - self.motion_samples[1][0] > .25:
            self.motion_samples.popleft()
        first_time, first_position = self.motion_samples[0]
        wall_velocity = (pose[:3, 3] - first_position) / max(now - first_time, .001)
        tangent_velocity = wall_velocity - outward * (wall_velocity @ outward)
        s.control = dict(controller='pid', force_feedback_n=fn, force_target_n=self.target_force,
                         desired_euler_deg=self.angles, orientation_error_deg=float(np.rad2deg(np.linalg.norm(error))),
                         tip_tracking_error_mm=float(np.linalg.norm(target - pose[:3, 3]) * 1000),
                         feed_speed_mm_s=float(np.linalg.norm(tangent_velocity) * 1000),
                         requested_feed_mm_s=float(np.linalg.norm(self.feed_velocity) * 1000))
        if np.linalg.norm(target - pose[:3, 3]) > .01:
            s.halt('PID TCP tracking error exceeded 10 mm')
            return {**b.nominal, **b.hold}
        return {**b.nominal, **s.q}


class PIDScan:
    """Existing planned rows in BASE millimetres; contact points are recorded from actual native feedback."""
    def __init__(self, program, scan, channel=None, baseline=False):
        from .surface_follow.transport import VirtualTransport
        self.program, self.scan, self.channel = program, scan, channel
        self.transport = VirtualTransport(program)
        self.points, self.rows = [], []
        self.baseline = baseline
        self.completed = False
        self.feed_velocity = np.zeros(3)
        if not baseline and not REFERENCE.exists():
            raise RuntimeError('평면 기준 데이터를 먼저 수집하세요: ros2 run rokey pid_surface --baseline')
        self.mu = 0. if baseline else json.loads(REFERENCE.read_text())['friction_mu']
        self.filter_force, self.filter_time = 0., time.monotonic()

    def record(self, state, phase):
        normal = self.axis
        fn = float(state.fast_wrench[:3] @ normal)
        angles = angles_zyz(state.rotation, [0., 180., 0.])
        c = self.transport.data.get('control', {})
        now = time.monotonic()
        self.filter_force += (1 - np.exp(-(now - self.filter_time) / .04)) * (fn - self.filter_force)
        self.filter_time = now
        feed = state.velocity[:3] - normal * (state.velocity[:3] @ normal)
        self.rows.append([time.monotonic(), phase, *(state.position * 1000), *angles, *state.fast_wrench[:3],
                          c.get('force_feedback_n', self.filter_force), c.get('force_target_n', 1.5),
                          float(np.rad2deg(np.arccos(np.clip(-state.rotation[:, 2] @ normal, -1, 1)))),
                          c.get('tip_tracking_error_mm', 0.),
                          c.get('feed_speed_mm_s', float(np.linalg.norm(feed) * 1000)),
                          *c.get('desired_euler_deg', angles)])
        if phase == 'stroke' and fn > .3:
            point = state.position - self.program.config['probe']['tip_radius_mm'] / 1000 * (state.rotation[:, 2] + normal)
            self.points.append([*point, fn])

    def send(self, point, rotation, force=1.5):
        state = self.transport.state()
        self.transport.send(dict(position=list(map(float, point)), rotation=rotation.tolist(), force=force,
                                 normal=self.axis.tolist(), controller='pid', pid_baseline=self.baseline,
                                 feed_velocity=self.feed_velocity.tolist(),
                                 friction_mu=self.mu, source_sequence=state.sequence,
                                 source_sampled_at=state.sampled_at), self.radius)
        return state

    def run(self, positions, axis):
        p, t = self.program, self.transport
        if p.mode != 'virtual':
            raise RuntimeError('PID controller is simulation-only')
        self.axis = unit(axis)
        self.radius = p.config['probe']['tip_radius_mm'] / 1000
        positions = np.asarray(positions, float) / 1000
        state = t.wait_ready()
        if t.data.get('simulator') != 'mujoco':
            raise RuntimeError('PID controller requires the native MuJoCo backend')
        if not t.data.get('probe_attached'):
            raise RuntimeError('탐침을 먼저 집어주세요: ros2 run rokey pick_probe')
        p.home()
        rotation = align(t.wait_ready().rotation, self.axis, np.pi)
        pre = positions[0] + .003 * self.axis
        pose, _ = p.probe.read()
        height = max(p.safe_z(), pre[2] * 1000 + 20)
        p.probe.move([*pose[:2], height], rotation, 20.)
        p.probe.move([*pre[:2] * 1000, height], rotation, 30.)
        p.probe.move((pre + .012 * self.axis) * 1000, rotation, 20.)
        p.probe.move(pre * 1000, rotation, 5.)
        t.wait_still(); t.tare()
        if self.baseline:
            try:
                self.baseline_path(positions, rotation)
                self.completed = True
            finally:
                self.save()
            return
        lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        distance = np.r_[0., np.cumsum(lengths)]
        progress, start, contact_since = 0., time.monotonic(), None
        t.begin('hybrid')
        phase, ended = 'search', False
        try:
            previous = time.monotonic()
            deadline = previous
            while True:
                t.spin(.005)
                now = time.monotonic()
                if now < deadline:
                    time.sleep(min(deadline - now, .002))
                    continue
                deadline = now + .02
                dt, previous = now - previous, now
                t.check_owner()
                state = t.state()
                c = t.data.get('control', {})
                fn = c.get('force_feedback_n', 0.)
                if phase == 'search':
                    if abs(fn - 1.5) < .15:
                        contact_since = contact_since or now
                        if now - contact_since > .5:
                            phase = 'stroke'
                            contact_since = None
                    else:
                        contact_since = None
                    if now - start > 25:
                        raise TimeoutError('PID contact search timed out')
                elif phase == 'stroke':
                    if fn < .2:
                        contact_since = contact_since or now
                        if now - contact_since > 1.:
                            raise RuntimeError('PID lost surface contact')
                    else:
                        contact_since = None
                        segment = min(np.searchsorted(distance, progress, side='right') - 1, len(lengths) - 1)
                        direction = (positions[segment + 1] - positions[segment]) / max(lengths[segment], 1e-9)
                        observed = distance[segment] + np.clip((state.position - positions[segment]) @ direction,
                                                              0., lengths[segment])
                        # Keep the target local when precise collision physics runs slower than wall time.
                        wanted = progress + self.scan.get('speed_mm_s', 10.) / 1000 * dt
                        progress = max(progress, min(distance[-1], wanted, observed + .001))
                index = min(np.searchsorted(distance, progress, side='right') - 1, len(lengths) - 1)
                alpha = (progress - distance[index]) / max(lengths[index], 1e-9)
                point = positions[index] + alpha * (positions[index + 1] - positions[index])
                self.feed_velocity = ((positions[index + 1] - positions[index]) / max(lengths[index], 1e-9)
                                      * self.scan.get('speed_mm_s', 10.) / 1000 if phase == 'stroke'
                                      and fn >= .2 and progress < distance[-1] else np.zeros(3))
                self.send(point, rotation)
                self.record(state, phase)
                if progress >= distance[-1] and np.linalg.norm((state.position - positions[-1]) - self.axis * ((state.position - positions[-1]) @ self.axis)) < .001:
                    break
                if now - start > distance[-1] / .001 + 30:
                    raise TimeoutError('PID path duration exceeded')
            origin = state.position.copy()
            self.feed_velocity[:] = 0.
            while (state.position - origin) @ self.axis < .005:
                t.spin(.01)
                point = state.position + .0005 * self.axis
                state = self.send(point, state.rotation, 0.)
                self.record(state, 'retract')
            ended = t.end()
            if not ended:
                raise RuntimeError('PID stop was not confirmed')
            if self.channel:
                self.channel.write('measured', self.points)
            p.home()
            self.completed = True
        finally:
            if not ended and t.run_id:
                t.end()
            self.save()

    def baseline_path(self, positions, rotation):
        p, t = self.program, self.transport
        probe = p.probe
        probe.stiffness = STIFFNESS.copy()
        probe.enable(p.robot.DR_TOOL)
        p.robot.set_desired_force([0., 0., 1.5, 0., 0., 0.], [0, 0, 1, 0, 0, 0], mod=p.robot.DR_FC_MOD_ABS)
        probe.forcing = True
        start, contact = time.monotonic(), None
        while True:
            t.spin(.01); state = t.state()
            self.record(state, 'search')
            probe.guard(state.fast_wrench)
            if state.fast_wrench[:3] @ self.axis > .5:
                contact = contact or time.monotonic()
                if time.monotonic() - contact > 1.:
                    break
            else:
                contact = None
            if time.monotonic() - start > 25:
                raise TimeoutError('Baseline contact search timed out')
        level = state.position @ self.axis

        def measure(_pose, _wrench):
            state = t.state()
            probe.guard(state.fast_wrench)
            self.record(state, 'stroke')
            return False

        for point in positions[1:]:
            probe.move((point + (level - point @ self.axis) * self.axis) * 1000, rotation, 10.,
                       until=measure, acc=20.)
        probe.release()
        state = t.state()
        probe.move((state.position + .005 * self.axis) * 1000, rotation, 5.)
        probe.close()
        p.home()

    def save(self):
        if not self.rows:
            return
        RESULTS.mkdir(exist_ok=True)
        stem = RESULTS / f'pid_{"baseline" if self.baseline else "scan"}_{datetime.now():%Y%m%d_%H%M%S}'
        columns = ['t_s', 'phase', 'x_mm', 'y_mm', 'z_mm', 'a_deg', 'b_deg', 'c_deg', 'Fx_N', 'Fy_N', 'Fz_N',
                   'filtered_fn_N', 'target_fn_N', 'angle_error_deg', 'tip_error_mm', 'speed_mm_s',
                   'desired_a_deg', 'desired_b_deg', 'desired_c_deg']
        rows = [[r[0] - self.rows[0][0], *r[1:]] for r in self.rows]
        with stem.with_suffix('.csv').open('w', newline='') as file:
            writer = csv.writer(file); writer.writerow(columns); writer.writerows(rows)
        values = np.array([[r[0], *r[2:]] for r in rows], float)
        stroke = np.array([r[1] == 'stroke' for r in rows])
        metrics = {'completed': self.completed, 'controller': 'default' if self.baseline else 'pid',
                   'stiffness': STIFFNESS, 'target_reaction_tool_z_n': -1.5}
        if stroke.any():
            metrics.update(filtered_force_mean_n=float(values[stroke, 10].mean()),
                           force_rmse_n=float(np.sqrt(np.mean((values[stroke, 10] - 1.5) ** 2))),
                           feed_mean_mm_s=float(values[stroke, 14].mean()),
                           feed_median_mm_s=float(np.median(values[stroke, 14])),
                           feed_p95_mm_s=float(np.percentile(values[stroke, 14], 95)),
                           alignment_p95_deg=float(np.percentile(values[stroke, 12], 95)))
            metrics['stability_accepted'] = bool(self.completed and metrics['force_rmse_n'] < .2
                and metrics['alignment_p95_deg'] < 5 and 8 < metrics['feed_median_mm_s'] < 12
                and metrics['feed_p95_mm_s'] < 15)
            segment = values[stroke]
            centres = np.arange(segment[0, 0] + .6, segment[-1, 0] - .25, .05)
            wall_speeds = []
            for centre in centres:
                window = segment[np.abs(segment[:, 0] - centre) <= .25]
                slope = np.polyfit(window[:, 0] - centre, window[:, 1:4], 1)[0]
                wall_speeds.append(np.linalg.norm(slope - self.axis * (slope @ self.axis)))
            if wall_speeds:
                requested = self.scan.get('speed_mm_s', 10.)
                accepted = bool(np.all(np.abs(np.asarray(wall_speeds) - requested) <= requested * .05))
                metrics.update(wall_speed_window_s=.5, steady_start_exclusion_s=.6,
                               steady_end_exclusion_s=.25,
                               steady_wall_speed_min_mm_s=float(min(wall_speeds)),
                               steady_wall_speed_median_mm_s=float(np.median(wall_speeds)),
                               steady_wall_speed_max_mm_s=float(max(wall_speeds)),
                               constant_speed_accepted=accepted)
                metrics['stability_accepted'] = metrics['stability_accepted'] and accepted
        stem.with_suffix('.json').write_text(json.dumps(metrics, indent=2))
        if self.baseline and self.completed and stroke.sum() > 20:
            forces, poses = values[stroke, 7:10], values[stroke, 1:4]
            tangent = unit(poses[-1] - poses[0])
            fn = forces @ self.axis
            mu = float(np.clip(np.median(-(forces @ tangent) / np.maximum(fn, .3)), 0., 1.))
            REFERENCE.write_text(json.dumps({'friction_mu': mu, 'source_csv': str(stem.with_suffix('.csv')),
                                             'target_reaction_tool_z_n': -1.5, 'speed_mm_s': 10.,
                                             'filtered_force_mean_n': float(values[stroke, 10].mean())}, indent=2))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(4, 1, figsize=(13, 14))
        axes[0].plot(values[:, 0], -(values[:, 7:10] @ self.axis), lw=.6, label='Raw reaction')
        axes[0].plot(values[:, 0], -values[:, 10], lw=1, label='Denoised measured reference')
        axes[0].plot(values[:, 0], -values[:, 11], '--', label='Desired reaction -1.5 N')
        axes[0].set(ylabel='Reaction [N]')
        axes[1].plot(values[:, 0], values[:, 12], label='Alignment error [deg]')
        axes[1].plot(values[:, 0], values[:, 13], label='TCP error [mm]')
        axes[2].plot(values[:, 0], values[:, 14], label='Measured feed [mm/s]')
        axes[2].axhline(10., ls='--', color='0.4'); axes[2].set(xlabel='Time [s]')
        for index, (name, color) in enumerate(zip('ABC', ['tab:red', 'tab:green', 'tab:blue'])):
            axes[3].plot(values[:, 0], values[:, 4 + index], color=color, label=f'{name} measured', lw=.7)
            axes[3].plot(values[:, 0], values[:, 15 + index], color=color, label=f'{name} desired', lw=.7, ls='--')
        axes[3].set(xlabel='Time [s]', ylabel='ZYZ Euler [deg]')
        for ax in axes:
            ax.grid(alpha=.3); ax.legend()
        fig.tight_layout(); fig.savefig(stem.with_suffix('.png'), dpi=120); plt.close(fig)
        self.program.log(f'Saved {stem}.png / .csv')


def main():
    import argparse
    from .program import RobotProgram
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', action='store_true')
    options = parser.parse_args()
    program = RobotProgram('pid_surface')
    try:
        program.home()
        # A clear table plane, outside the real vehicle and rack, reachable with a downward probe.
        points = np.array([[300., -110., 0.], [350., -110., 0.]])
        PIDScan(program, {'speed_mm_s': 10.}, baseline=options.baseline).run(points, [0, 0, 1])
    finally:
        program.close()


if __name__ == '__main__':
    main()
