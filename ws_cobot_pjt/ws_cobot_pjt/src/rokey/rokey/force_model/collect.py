"""Free-space excitation for the joint-space force model (force_model/model.py), vehicle removed, the holder in place
(posts up to 125 mm): every TCP (probe tip, 283 mm below tool0) target at least min_z_mm high and tool0 at least
min_flange_z_mm (the tool points down, so the whole tool is above the tip, clear of the posts) and never below its station (no -z sweeps: the probe is held). At each force_model.stations_mm
TCP position, reached in a straight line (the outer reach is a ball: two feasible ends keep the line within it), holds
for the gravity terms (tool down, tilted about the tip tilts_deg x azimuths_deg, turned about the tool axis spins_deg;
the turns recorded, each hold hold_s recorded from the stop), then out-and-back sweeps (sweep_directions, span_mm) at
each speeds_mm_s, each followed by rest_s recorded still (the reading's settling and its memory of the last travel
direction after a stop, which a 0.3 s unrecorded wait had hidden); last the held-out
validation_mm station with other motions (diagonals, a tilted sweep). Position control (--compliance: under the
mesh-height task compliance, as the scan strokes). Every target is IK- and outer-reach-checked first (reach_margin_mm
in) and skipped when it fails. Rows as
force_data.ForceRecorder (joint state from the 100 Hz arm_joint_states) -> calibration/force_model_<mode>_<time>.npz
  ros2 run rokey force_model_collect --cell-clear [--plan-only] [--compliance] [--compare-telemetry]"""
import argparse
from datetime import datetime
import json
import time

import numpy as np

from ..configs import read_session
from ..force_data import CALIBRATION_DIR, CalibrationRun
from ..part_scan.tool import robot_chain
from ..robot_namespace import program_args
from ..rotations import axis_angle, rotation_zyz


class StationPlan:
    """The stations' motions that the controller can make: IK from HOME and the outer reach, reach_margin_mm in."""

    def __init__(self, run, settings):
        self.settings = settings
        self.tcp = ((np.round(run.robot.get_active_tcp_profile(), 2) + 0.).tolist() if run.mode == 'real'
                    else read_session('probe_plan.json')['tcp_mm_deg'])
        self.chain = robot_chain(run.robot, self.tcp, run.config['real_scan']['ik'])
        self.home = np.deg2rad(run.config['robot']['home_joints_deg'])
        self.down = rotation_zyz(run.robot.fkin(run.config['robot']['home_joints_deg'])[3:])

    def flange_z(self, position, rotation):
        """tool0 (flange) height (BASE mm) at a TCP target, None when out of IK reach."""
        q = self.chain.solve(np.asarray(position, float) / 1000, rotation, self.home)
        return None if q is None else float(self.chain.forward(q)[2][-1][2] * 1000)

    def feasible(self, position, rotation):
        """TCP (probe tip) at least min_z_mm and tool0 at least min_flange_z_mm high, within IK and the outer reach."""
        flange = self.flange_z(position, rotation) if position[2] >= self.settings['min_z_mm'] else None
        return (flange is not None and flange >= self.settings['min_flange_z_mm'] and bool(
            self.chain.within_outer_reach(np.asarray(position, float)[None] / 1000, rotation,
                                          self.settings['reach_margin_mm'] / 1000)[0]))

    def tilted(self, tilt, azimuth):
        axis = np.array([np.cos(np.radians(azimuth)), np.sin(np.radians(azimuth)), 0.])
        return axis_angle(axis, np.radians(tilt)) @ self.down

    def motions(self, station):
        """(label, target BASE mm, rotation, speed mm/s or None: a turn and a hold) at one station, feasible only."""
        s, station = self.settings, np.asarray(station, float)
        moves = []
        for tilt, azimuth in [(t, a) for t in s['tilts_deg'] for a in s['azimuths_deg']] + [(0., 0.)]:
            moves.append((f'tilt {tilt:g}/{azimuth:g}', station, self.tilted(tilt, azimuth), None))
        for spin in [*s['spins_deg'], 0.]:  # about the tool axis: the tool's centre of mass off the axis
            moves.append((f'spin {spin:g}', station, self.down @ axis_angle(np.array([0., 0., 1.]), np.radians(spin)),
                          None))
        for speed in s['speeds_mm_s']:
            for direction in s['sweep_directions']:
                axis = (1. if direction[0] == '+' else -1.) * np.eye(3)['xyz'.index(direction[1])]
                moves += [(f'sweep {speed:g} {direction}', station + s['span_mm'] * axis, self.down, speed),
                          (f'sweep {speed:g} back', station, self.down, speed)]
        return [move for move in moves if self.feasible(move[1], move[2])], len(moves)

    def validation(self, station):
        """Held-out motions unlike the stations': x-y diagonals and a sweep tilted 20 deg, at 10 mm/s."""
        span, station = self.settings['span_mm'], np.asarray(station, float)
        tilted = self.tilted(20., 45.)
        moves = [move for d in ([1., 1.], [1., -1.], [-1., 1.], [-1., -1.]) for move in (
            (f'diagonal {d}', station + span * np.r_[d, 0.] / np.sqrt(2), self.down, 10.),
            ('diagonal back', station, self.down, 10.))]
        moves += [('tilt 20/45', station, tilted, None), ('tilted sweep +x', station + [span, 0., 0.], tilted, 10.),
                  ('tilted sweep -x', station - [span, 0., 0.], tilted, 10.), ('tilted back', station, tilted, 10.),
                  ('tilt 0', station, self.down, None)]
        return [move for move in moves if self.feasible(move[1], move[2])], len(moves)

    def duration(self, moves):
        s, rotation_speed = self.settings, 20.  # ContactProbe.move: 20 deg/s
        turns = sum(s['hold_s'] + np.degrees(np.arccos(np.clip((np.trace(self.down.T @ r) - 1) / 2, -1, 1))) / rotation_speed
                    for _, _, r, speed in moves if speed is None)
        return turns + sum(s['span_mm'] / speed + s['rest_s'] for _, _, _, speed in moves if speed is not None)


class TelemetryRecorder:
    """Record the RT stream alongside the service-based recorder within the same motion phases."""

    def __init__(self, node):
        from dsr_msgs2.msg import ForceTelemetry
        from rclpy.qos import qos_profile_sensor_data

        self.label, self.previous, self.received = '', None, 0
        self.rows, self.labels = [], []
        self.subscription = node.create_subscription(
            ForceTelemetry, 'realtime/force_telemetry', self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        receipt = time.monotonic()
        stamp = msg.time_stamp
        if not np.isfinite(stamp) or self.previous is not None and stamp <= self.previous:
            return
        self.previous = stamp
        self.received += 1
        if self.label:
            self.rows.append(np.r_[stamp, np.deg2rad(msg.actual_joint_position_deg),
                                  np.deg2rad(msg.actual_joint_velocity_deg_s), msg.actual_tcp_pose_mm_deg,
                                  msg.external_tcp_wrench_n_nm, msg.actual_tcp_velocity_mm_deg_s[:3],
                                  receipt])
            self.labels.append(self.label)

    def wait_ready(self, executor):
        deadline = time.monotonic() + 5.
        while self.received < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        if self.received < 2:
            raise RuntimeError('realtime/force_telemetry에서 새 표본을 받지 못했습니다. 실기기 드라이버를 확인하세요.')

    def arrays(self):
        rows = np.asarray(self.rows, float).reshape(-1, 29)
        return dict(rt_rows=rows[:, :25], rt_labels=np.asarray(self.labels, dtype=str),
                    rt_tcp_velocity_mm_s=rows[:, 25:28], rt_receipt_s=rows[:, 28])


class ForceModelCollect:
    def __init__(self, run, compliance, compare_telemetry=False):
        self.run, self.probe, self.recorder = run, run.probe, run.recorder
        self.settings = run.config['force_model']
        self.plan, self.compliance = StationPlan(run, self.settings), compliance
        self.telemetry = TelemetryRecorder(run.node) if compare_telemetry else None
        self.phase = 0

    def transfer(self, station):
        m = self.run.config['motion']
        self.probe.move(station, self.plan.down, m['linear_speed'], acc=m['linear_acc'])

    def label_phase(self, name):
        if self.telemetry is not None:
            self.phase += 1
            name = f'{name} #{self.phase}'
            self.telemetry.label = name
        self.recorder.label = name

    def excite(self, name, moves):
        acc = self.run.config['scan']['acc']
        try:
            for label, target, rotation, speed in moves:
                self.label_phase(f'{name}:{label}')
                self.probe.move(target, rotation, speed or self.run.config['motion']['approach_speed'],
                                until=self.recorder.collect, acc=acc)
                # Recorded from the stop: holds after turns, rests after moves.
                self.label_phase(f'{name}:{"hold" if speed is None else "rest"} {label}')
                self.recorder.static(self.settings['hold_s'] if speed is None else self.settings['rest_s'])
        finally:
            if self.telemetry is not None:
                self.telemetry.label = ''

    def collect(self, plan_only):
        stations, total = [], 0.
        named = [(f'S{index}', station, self.plan.motions) for index, station in enumerate(self.settings['stations_mm'])]
        for name, station, motions in [*named, ('V', self.settings['validation_mm'], self.plan.validation)]:
            moves, count = motions(station)
            stations.append((name, np.asarray(station, float), moves))
            total += self.plan.duration(moves)
            flange = min(self.plan.flange_z(target, rotation) for _, target, rotation, _ in moves)
            self.run.log(f'{name} {station} mm: {len(moves)}/{count} 동작 가능, 약 {self.plan.duration(moves):.0f} s, '
                         f'탐침 끝 최저 {min(target[2] for _, target, _, _ in moves):.0f} mm, tool0 최저 {flange:.0f} mm')
        self.run.log(f'전체 약 {total / 60:.1f}분 (이동 제외)')
        if plan_only:
            return None
        if self.telemetry is not None:
            self.telemetry.wait_ready(self.run.robot.executor)
        if self.run.robot.get_control_mode() == 4:  # left in compliance / force control: released before HOME
            self.probe.compliant = self.probe.forcing = True
        self.run.home()
        if self.compliance:
            self.probe.enable(self.run.robot.DR_BASE, self.run.config['real_scan']['mesh_height']['stiffness'])
        try:
            for name, station, moves in stations:
                self.run.log(f'{name}: {station.tolist()} mm')
                self.transfer(station)
                self.excite(name, moves)
        finally:
            self.probe.close()
            data, path = self.recorder.take() if self.recorder.rows else None, None
            if data is not None:
                path = self.save(data, stations)
        self.run.home()
        return path

    def save(self, data, stations):
        path = CALIBRATION_DIR / f'force_model_{self.run.mode}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.npz'
        meta = {**self.run.meta, 'tcp_mm_deg': self.plan.tcp, 'urdf': self.plan.chain.description,
                'compliance': self.compliance, 'settings': self.settings,
                'stations_mm': {name: station.tolist() for name, station, _ in stations}}
        additional = self.telemetry.arrays() if self.telemetry is not None else {}
        np.savez(path, rows=data.rows, labels=data.labels, meta=json.dumps(meta), **additional)
        count = f'{len(data.rows)} service samples'
        if self.telemetry is not None:
            count += f', {len(self.telemetry.rows)} RT samples'
        self.run.log(f'{count} -> {path}')
        return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--cell-clear', action='store_true', help='confirm: vehicle removed from the work area')
    parser.add_argument('--plan-only', action='store_true', help='list the stations and their feasible motions only')
    parser.add_argument('--compliance', action='store_true', help='under the mesh-height task compliance')
    parser.add_argument('--compare-telemetry', action='store_true',
                        help='record the real robot RT topic alongside the existing input')
    options = parser.parse_args(program_args())
    if not (options.cell_clear or options.plan_only):
        parser.error('--cell-clear: 작업 영역에서 차량을 치운 뒤 실행하세요 (스테이션이 보닛 높이까지 내려갑니다)')
    run = CalibrationRun('force_model_collect')
    try:
        if options.compare_telemetry and run.mode != 'real':
            parser.error('--compare-telemetry requires the real robot')
        ForceModelCollect(run, options.compliance, options.compare_telemetry).collect(options.plan_only)
    finally:
        run.close()


if __name__ == '__main__':
    main()
