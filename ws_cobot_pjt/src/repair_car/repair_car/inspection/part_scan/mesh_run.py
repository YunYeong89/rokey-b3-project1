"""Custom pose streaming during detection only; ordinary transfers use the existing DSR moves."""
from collections import deque
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from repair_car.robot_control.cartesian_servo import POSITION_WORKSPACE_M
from repair_car.inspection.part_scan.cartesian_transport import CartesianTransport
from repair_car.inspection.part_scan.mesh_motion import ScanTransfers
from repair_car.inspection.part_scan.progress_watch import ProgressWatch
from repair_car.inspection.part_scan.sampling import PointSampler
from repair_car.inspection.part_scan.scan_timing import StrokeTiming


class StrokeInterrupted(RuntimeError):
    def __init__(self, reason, elapsed):
        super().__init__(reason)
        self.elapsed = elapsed  # trajectory time reached, s


class MeshHeightScan:
    def __init__(self, program, plan, channel, report):
        self.program, self.plan, self.channel, self.report = program, plan, channel, report
        self.scan, self.settings = plan.scan, plan.scan['mesh_height']
        # mesh-height presses along one fixed BASE axis (bonnet -Z, door +X: its TCP axis tilted), else the TCP axis.
        self.press = -plan.paths[0].axis
        self.door_entry = plan.surface.part == 'door'
        self.transport = CartesianTransport(program, self.settings, self.scan['speed_mm_s'],
                                            force_axis=self.press if self.door_entry else None)
        self.records, self.points = [], []
        self.sampler = PointSampler(self.scan['sample_mm'], self.settings['point_decimation'])
        self.timings = []
        self.completed = False
        self.radius = self.settings.get('ball_radius_mm', 0.)
        # Pressed along the TCP +z axis like the default controller: its survey force, ramp and hold.
        self.force = self.scan['default_scan']['survey']['force_n']
        self.contact = self.scan['default_scan']['contact_threshold_n']  # measured points only in contact, as default
        # Surface normals per path (the tool axis is the smoothed normal): contact points and normal force.
        self.surfaces = [(cKDTree(path.positions), path.normals) for path in plan.paths]

    def axis(self, rotation):
        """Press direction into the surface: mesh-height's fixed BASE axis, else the TCP +z axis."""
        return self.press

    def centre_error(self, position, actual_rotation, target, rotation):
        """Ball-centre error across the press axis: along it the force control owns the depth."""
        error = position - self.radius * actual_rotation[:, 2] - target + self.radius * rotation[:, 2]
        axis = self.axis(rotation)
        return np.linalg.norm(error - axis * (error @ axis))

    @staticmethod
    def length(trajectory):
        """Stroke length of a trajectory (ball-centre arc length, mm), for the progress feedback."""
        return sum(span.length for span in trajectory.spans)

    @staticmethod
    def pose(state):
        return np.array([state.position.x, state.position.y, state.position.z]) * 1000, np.array(state.rotation).reshape(3, 3)

    def prepare(self):
        state = self.transport.wait_ready()
        if not state.probe_attached:
            raise RuntimeError('Pick up the probe first: ros2 run repair_car pick_probe')
        position, rotation = self.pose(state)
        reach = self.plan.reach
        q = np.asarray(state.joint_positions)
        model_pose = reach.chain.forward(q)[0]
        if (np.linalg.norm(model_pose[:3, 3] * 1000 - position) > 2.
                or (Rotation.from_matrix(model_pose[:3, :3]) * Rotation.from_matrix(rotation).inv()).magnitude()
                > np.radians(2.)):
            raise RuntimeError('Active ProbeTCP does not match the planning URDF/TCP profile')
        outward = -self.press if self.door_entry else None
        self.transfers = ScanTransfers(self.program, self.scan, reach.frame, outward)
        self.report.route([(path.trajectory, 'stroke', number)
                           for number, path in enumerate(self.plan.paths, 1)])
        self.program.log('Reusing the detection path validation; ordinary moves run at the existing DSR speeds')
        self.program.home()

    def wait_at_entry(self, trajectory, elapsed=0.):
        state = self.transport.wait_still()
        if self.door_entry:
            target, rotation, _ = trajectory.sample(elapsed)
            error, turn = self.entry_error(state, target, rotation)
            if error > POSITION_WORKSPACE_M * 1000 or turn > self.settings['progress_angle_deg']:
                raise RuntimeError(f'Door measurement point approach incomplete: TCP position error {error:.1f} mm, angle error {turn:.1f}°')
        return state

    def entry_error(self, state, target, rotation):
        position, actual_rotation = self.pose(state)
        distance = np.linalg.norm(position - target)
        turn = np.degrees((Rotation.from_matrix(rotation) * Rotation.from_matrix(actual_rotation).inv()).magnitude())
        return distance, turn

    def align_door_entry(self, trajectory, elapsed):
        """Correct a short DSR stop within the local stream workspace before ramping contact force."""
        target, rotation, _ = trajectory.sample(elapsed)
        t = self.transport
        tolerance, angle = self.settings['progress_error_mm'], self.settings['progress_angle_deg']
        period = 1 / self.settings['command_hz']
        next_command = time.monotonic()
        deadline = next_command + 6.
        error, turn = self.entry_error(t.state(), target, rotation)
        if error <= tolerance and turn <= angle:
            return
        self.program.log(f'Correcting the door measurement start position: TCP error {error:.1f} mm (before force control)')
        while time.monotonic() < deadline:
            self.program.robot.check_cancelled()
            now = time.monotonic()
            if now >= next_command:
                t.send(target, rotation, np.zeros(6), force=0.)
                next_command = now + period
            t.spin(.005)
            error, turn = self.entry_error(t.state(), target, rotation)
            if error <= tolerance and turn <= angle:
                self.program.log(f'Door measurement start correction done: TCP error {error:.2f} mm')
                return
        raise RuntimeError(f'Door start correction timed out: TCP position error {error:.1f} mm, angle error {turn:.1f}°')

    def record(self, state, target, rotation, phase, number):
        position, actual_rotation = self.pose(state)
        wrench = state.measured_wrench
        values = np.array([wrench.force.x, wrench.force.y, wrench.force.z,
                           wrench.torque.x, wrench.torque.y, wrench.torque.z])
        tree, normals = self.surfaces[number - 1]
        normal = normals[tree.query(target)[1]]  # outward surface normal under the planned TCP
        # Measurement only: these values do not gate recording, motion, progress or completion.
        force = float(values[:3] @ normal)
        error = np.linalg.norm(position - target)
        angle = np.degrees((Rotation.from_matrix(rotation) * Rotation.from_matrix(actual_rotation).inv()).magnitude())
        self.records.append([state.sampled_at, number, phase, *position, *target, *normal, *values,
                             force, error, angle, state.sensor_age_s,
                             state.velocity.linear.x * 1000, state.velocity.linear.y * 1000,
                             state.velocity.linear.z * 1000,
                             *np.degrees([state.velocity.angular.x, state.velocity.angular.y,
                                          state.velocity.angular.z]), *np.degrees(state.joint_positions)])
        pd = []
        for field in ('pd_error', 'pd_p', 'pd_d', 'pd_offset'):
            values = np.asarray(getattr(state, field), float)
            pd.extend([*(values[:3] * 1000), *np.degrees(values[3:])])
        v = state.pd_velocity_error
        self.records[-1].extend([int(state.pd_enabled), int(state.pd_saturated), *pd,
                                 v.linear.x * 1000, v.linear.y * 1000, v.linear.z * 1000,
                                 *np.degrees([v.angular.x, v.angular.y, v.angular.z])])
        velocity = state.velocity
        omega = np.array([velocity.angular.x, velocity.angular.y, velocity.angular.z])
        ball_velocity = np.array([velocity.linear.x, velocity.linear.y, velocity.linear.z]) * 1000
        ball_velocity -= self.radius * np.cross(omega, actual_rotation[:, 2])
        self.records[-1].extend([*(position - self.radius * actual_rotation[:, 2]),
                                 *(target - self.radius * rotation[:, 2]), *ball_velocity])
        radius = self.program.config['probe']['tip_radius_mm']
        point = position - radius * (actual_rotation[:, 2] + normal)
        if phase == 'stroke' and force > self.contact and self.sampler.keep(point):
            self.points.append([*(point / 1000), force])
        self.program.topics.sample(state.sampled_at, phase, normal, position, actual_rotation, values, force,
                                   force > self.contact, point,
                                   len(self.points), state.sequence)

    def execute(self, trajectory, phase, number, timing, start=0.):
        """Stream the trajectory from `start` (its time, s); a resumed stroke eases its time in over resume.ramp_s.
        A stalled or stopped stream (or stale feedback) raises StrokeInterrupted with the time reached."""
        t = self.transport
        elapsed, started, previous, next_write, last_sequence = start, time.monotonic(), time.monotonic(), 0., -1
        next_command = previous
        period = 1 / self.settings['command_hz']
        ramp = self.settings['resume']['ramp_s'] if start else 0.
        press_ramp, press_s = self.scan['ramp_s'], self.scan['ramp_s'] + self.scan['hold_s']
        progress = ProgressWatch(self.settings['stall_timeout_s'], self.settings['stall_motion_mm'],
                                 self.settings['progress_error_mm'])
        target, rotation, _ = trajectory.sample(start)
        settled_since, settle_positions = None, deque()
        try:
            while True:
                t.spin(min(.005, period))
                now = time.monotonic()
                self.program.robot.check_cancelled()
                state = t.state()
                if state.sequence != last_sequence:
                    self.record(state, target, rotation, phase, number)
                    last_sequence = state.sequence
                if now < next_command:
                    continue
                next_command = now + period
                dt, previous = min(now - previous, .04), now
                position, actual_rotation = self.pose(state)
                pose_error = self.centre_error(position, actual_rotation, target, rotation)
                if progress.blocked(now, position, pose_error):
                    raise RuntimeError(f'TCP progress stalled: moved less than {self.settings["stall_motion_mm"]} mm '
                                       f'in {self.settings["stall_timeout_s"]} s, position error {pose_error:.2f} mm')
                angle_error = np.degrees((Rotation.from_matrix(rotation) *
                                          Rotation.from_matrix(actual_rotation).inv()).magnitude())
                position_ok = pose_error <= self.settings['progress_error_mm']
                angle_ok = angle_error <= self.settings['progress_angle_deg']
                progressing = position_ok and angle_ok
                at_end = elapsed >= trajectory.duration
                pressed = now - started - press_s  # the force ramped up and held, as the default controller
                scale = min(1., max(0., pressed) / ramp) if ramp else 1.
                moving = progressing and pressed >= 0.
                if moving:
                    elapsed = min(trajectory.duration, elapsed + dt * scale)
                timing.observe(now, position_ok, angle_ok, at_end, elapsed)
                self.program.topics.within(elapsed / trajectory.duration * self.length(trajectory))
                target, rotation, feed = trajectory.sample(elapsed)
                feed *= scale if moving else 0.
                t.send(target, rotation, feed, self.force * min(1., (now - started) / press_ramp))
                if now >= next_write:
                    self.channel.write('measured', self.points)
                    next_write = now + .5
                if elapsed >= trajectory.duration:
                    settle_positions.append((now, position.copy()))
                    while len(settle_positions) > 1 and now - settle_positions[1][0] >= .3:
                        settle_positions.popleft()
                    endpoint = np.mean([sample[1] for sample in settle_positions], axis=0)
                    pose_error = self.centre_error(endpoint, actual_rotation, target, rotation)
                if (elapsed >= trajectory.duration and pose_error <= self.settings.get('virtual_settle_position_mm', self.settings['settle_position_mm'])
                        and angle_error <= self.settings['settle_angle_deg']):
                    settled_since = settled_since or now
                    if now - settled_since >= self.settings['pd']['settle_hold_s']:
                        return
                else:
                    settled_since = None
                if now - started > 3 * (trajectory.duration - start) + press_s + 15:
                    raise TimeoutError(f'Mesh {phase} path {number} did not settle on its pose target')
        except (RuntimeError, TimeoutError) as error:
            if self.program.robot.cancelled:
                raise
            raise StrokeInterrupted(str(error), elapsed) from error

    def stroke(self, path, number):
        """Detection along one path. An interrupted stream does not end the scan: the probe lifts resume.lift_mm off
        along its axis, comes back down onto the stroke where it stopped and detects on from there; after
        resume.attempts interruptions without progress the rest of the path is skipped. -> whether it completed"""
        topics, resume, trajectory = self.program.topics, self.settings['resume'], path.trajectory
        timing = StrokeTiming(number, trajectory.duration)
        self.timings.append(timing)
        start, failures = 0., 0
        try:
            while True:
                try:
                    self.transport.begin()
                    if self.door_entry:
                        self.align_door_entry(trajectory, start)
                    self.program.log(f'native PD {"ON" if self.transport.state().pd_enabled else "OFF"}; '
                                     f'detection {start:.1f} -> {trajectory.duration:.1f} s')
                    topics.stage(topics.SCANNING)
                    self.execute(trajectory, 'stroke', number, timing, start)
                    timing.completed = True
                    topics.advance(self.length(trajectory))
                    return True
                except StrokeInterrupted as error:
                    failures = 1 if error.elapsed > start + 1. else failures + 1
                    if failures > resume['attempts']:
                        self.program.log(f'mesh path {number}: stopped {failures} times at the same place, skipping the remaining '
                                         f'{trajectory.duration - error.elapsed:.1f} s ({error})')
                        return False
                    start = error.elapsed
                    timing.resumes += 1
                    self.program.log(f'mesh path {number}: detection stopped ({error}); lifting {resume["lift_mm"]} mm, then '
                                     f'redetecting from {start:.1f} s ({failures}/{resume["attempts"]})')
                finally:
                    self.transport.end()
                topics.stage(topics.MOVING)
                self.transfers.resume(trajectory, start, resume['lift_mm'])
                self.wait_at_entry(trajectory, start)
        finally:
            timing.finish()
            self.program.log(f'Detection {timing.export()["wall_s"]:.1f} s: tracking wait {timing.pause:.1f} s, '
                             f'endpoint settle {timing.settle:.1f} s, {timing.resumes} redetections')

    def run(self):
        failure = None
        try:
            self.prepare()
            topics, skipped = self.program.topics, []
            topics.plan([[self.length(path.trajectory)] for path in self.plan.paths], self.scan['speed_mm_s'])
            for number, path in enumerate(self.plan.paths, 1):
                topics.stage(topics.MOVING, path=number, segment=number)
                self.program.log(f'mesh path {number}/{len(self.plan.paths)}: DSR approach')
                self.transfers.approach(path.trajectory)
                state = self.wait_at_entry(path.trajectory)
                path.trajectory.entry(self.plan.reach, np.asarray(state.joint_positions), self.scan['speed_mm_s'],
                                      self.scan['measurable']['joint_jump_deg'])
                self.program.log(f'mesh path {number}: custom detection started, up to {self.scan["speed_mm_s"]} mm/s')
                if not self.stroke(path, number):
                    skipped.append(number)
                self.program.log(f'mesh path {number}: detection ended; DSR retract')
                topics.stage(topics.MOVING)
                self.transfers.retract()
            self.program.home()
            if skipped:
                raise RuntimeError(f'Inspection paths {skipped}: measurement incomplete after repeated stops')
            self.completed = True
        except BaseException as error:
            failure = str(error) or type(error).__name__
            raise
        finally:
            try:
                self.transport.end()
            finally:
                try:
                    self.transport.close()
                finally:
                    try:
                        self.channel.write('measured', self.points)
                    finally:
                        self.report.save(self.records, self.completed, [timing.export() for timing in self.timings],
                                         failure=failure)
