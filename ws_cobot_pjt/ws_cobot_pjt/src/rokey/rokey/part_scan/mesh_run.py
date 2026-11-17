"""Custom pose streaming during detection only; ordinary transfers use the existing DSR moves."""
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .cartesian_transport import CartesianTransport
from .mesh_motion import ScanTransfers
from .progress_watch import ProgressWatch


class MeshNormalScan:
    def __init__(self, program, plan, channel, report):
        self.program, self.plan, self.channel, self.report = program, plan, channel, report
        self.scan, self.settings = plan.scan, plan.scan['mesh_normal']
        self.transport = CartesianTransport(program, self.settings, self.scan['speed_mm_s'])
        self.records, self.points = [], []
        self.completed = False

    @staticmethod
    def pose(state):
        return np.array([state.position.x, state.position.y, state.position.z]) * 1000, np.array(state.rotation).reshape(3, 3)

    def prepare(self):
        state = self.transport.wait_ready()
        if not state.probe_attached:
            raise RuntimeError('탐침을 먼저 집어주세요: ros2 run rokey pick_probe')
        position, rotation = self.pose(state)
        reach = self.plan.reach
        q = np.asarray(state.joint_positions)
        model_pose = reach.chain.forward(q)[0]
        if (np.linalg.norm(model_pose[:3, 3] * 1000 - position) > 2.
                or (Rotation.from_matrix(model_pose[:3, :3]) * Rotation.from_matrix(rotation).inv()).magnitude()
                > np.radians(2.)):
            raise RuntimeError('Active ProbeTCP does not match the planning URDF/TCP profile')
        self.transfers = ScanTransfers(self.program, self.scan, reach.frame)
        self.report.route([(path.trajectory, 'stroke', number)
                           for number, path in enumerate(self.plan.paths, 1)])
        self.program.log('검출 경로 검증 결과 재사용; 일반 이동은 기존 DSR 속도로 실행합니다')
        self.program.home()

    def record(self, state, target, rotation, phase, number):
        position, actual_rotation = self.pose(state)
        wrench = state.measured_wrench
        values = np.array([wrench.force.x, wrench.force.y, wrench.force.z,
                           wrench.torque.x, wrench.torque.y, wrench.torque.z])
        normal = -rotation[:, 2]
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
        if phase == 'stroke':
            radius = self.program.config['probe']['tip_radius_mm']
            point = position - radius * (actual_rotation[:, 2] + normal)
            if not self.points or np.linalg.norm(point / 1000 - np.array(self.points[-1][:3])) >= self.scan['sample_mm'] / 1000:
                self.points.append([*(point / 1000), force])

    def execute(self, trajectory, phase, number):
        t = self.transport
        elapsed, started, previous, next_write, last_sequence = 0., time.monotonic(), time.monotonic(), 0., -1
        next_command = previous
        period = 1 / self.settings['command_hz']
        progress = ProgressWatch(self.settings['stall_timeout_s'], self.settings['stall_motion_mm'],
                                 self.settings['progress_error_mm'])
        target, rotation, _ = trajectory.sample(0.)
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
            pose_error = np.linalg.norm(position - target)
            if progress.blocked(now, position, pose_error):
                raise RuntimeError(f'TCP 진행 정체: {self.settings["stall_timeout_s"]}초 동안 이동량이 '
                                   f'{self.settings["stall_motion_mm"]} mm 미만이며 위치 오차는 {pose_error:.2f} mm입니다. '
                                   '턱/경로 높이/위치 추종을 확인하세요.')
            angle_error = np.degrees((Rotation.from_matrix(rotation) * Rotation.from_matrix(actual_rotation).inv()).magnitude())
            progressing = (pose_error <= self.settings['progress_error_mm'] and
                           angle_error <= self.settings['progress_angle_deg'])
            if progressing:
                elapsed = min(trajectory.duration, elapsed + dt)
            target, rotation, feed = trajectory.sample(elapsed)
            if not progressing:
                feed[:] = 0.
            t.send(target, rotation, feed)
            if now >= next_write:
                self.channel.write('measured', self.points)
                next_write = now + .5
            if (elapsed >= trajectory.duration and pose_error <= self.settings['settle_position_mm']
                    and angle_error <= self.settings['settle_angle_deg']):
                return
            if now - started > 3 * trajectory.duration + 15:
                raise TimeoutError(f'Mesh {phase} path {number} did not settle on its pose target')

    def run(self):
        try:
            self.prepare()
            for number, path in enumerate(self.plan.paths, 1):
                self.program.log(f'mesh path {number}/{len(self.plan.paths)}: DSR 접근')
                self.transfers.approach(path.trajectory)
                state = self.transport.wait_still()
                path.trajectory.entry(self.plan.reach, np.asarray(state.joint_positions), self.scan['speed_mm_s'],
                                      self.scan['measurable']['joint_jump_deg'])
                self.program.log(f'mesh path {number}: custom 검출 시작, 최대 {self.scan["speed_mm_s"]} mm/s')
                try:
                    self.transport.begin()
                    self.execute(path.trajectory, 'stroke', number)
                finally:
                    self.transport.end()
                self.program.log(f'mesh path {number}: 검출 종료; DSR 이탈')
                self.transfers.retract()
            self.program.home()
            self.completed = True
        finally:
            try:
                self.transport.end()
            finally:
                self.transport.close()
                self.channel.write('measured', self.points)
                self.report.save(self.records, self.completed)
