"""Check of the moving-offset calibration (motion_offset.py) on the bonnet mesh-height paths, in the air: force and
compliance off (released when left on, teleop), the paths of a mesh-height plan (its plan.json; default the
latest real_bonnet_mesh_height report) lifted --lift mm and followed as the scan does (DSR approach, constant-speed
movesx through the checkpoints, the stroke speed) in position control. In the air the true force is 0, so the
reading is its offset: raw and corrected (less the model at the TCP and the planned travel direction) BASE Fz as
subplots -> images/motion_offset_check_<time>/report.png (+ samples.csv).
  ros2 run rokey motion_offset_check [--plan PLAN_JSON] [--lift 10] [--path N]"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from .configs import output_dir
from .teleop import PoseBook, Teleop
from .motion_offset import OffsetModel
from .part_scan.height_dsr import MeshHeightDsr
from .program import RobotProgram
from .robot_namespace import program_args
from .rotations import rotation_zyz


class MotionOffsetCheck:
    def __init__(self, program, plan, lift, paths):
        self.program, self.probe, self.lift = program, program.probe, lift
        data = json.loads(Path(plan).read_text())
        self.paths = [(number, [np.asarray(span['positions_mm']) + [0., 0., lift] for span in path['spans']])
                      for number, path in enumerate(data['paths'], 1) if not paths or number in paths]
        self.model = OffsetModel.load(program)
        if self.model is None:
            raise RuntimeError('이동 오프셋 보정이 없습니다: ros2 run rokey motion_offset을 먼저 실행하세요')
        self.samples = []  # [t, path, x, y, z, dx, dy, dz, fx, fy, fz]

    def follow(self, number, spans):
        """One path in the air: across at the travel height, down to its lifted start, the splines, up."""
        p, m = self.program, self.program.config['motion']
        speed = p.config['real_scan']['mesh_height']['cruise_speed_mm_s']
        dense = np.vstack([np.vstack([np.linspace(a, b, max(2, int(np.linalg.norm(b - a) / .1) + 1))
                                      for a, b in zip(knots[:-1], knots[1:])]) for knots in spans])
        tangent = np.gradient(dense, axis=0)
        directions = tangent / np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        tree = cKDTree(dense[:, :2])

        def record(pose, wrench):
            self.samples.append([time.monotonic(), number, *pose[:3],
                                 *directions[tree.query(pose[:2])[1]], *wrench[:3]])
            return False
        start, pose = spans[0][0], self.probe.read()[0]
        travel = max(p.safe_z(), pose[2])
        self.probe.move([*pose[:2], travel], self.down, m['approach_speed'], acc=m['approach_acc'])
        self.probe.move([*start[:2], travel], self.down, m['linear_speed'], acc=m['linear_acc'])
        self.probe.move(start, self.down, m['approach_speed'], acc=m['approach_acc'])
        p.log(f'path {number}: 공중 {self.lift:g} mm 위에서 {speed:g} mm/s로 추종 (힘제어 없음)')
        for knots in spans:
            for piece in MeshHeightDsr.chunks(MeshHeightDsr.thin(knots)):
                self.probe.spline(piece[1:], self.down, speed, until=record, acc=p.config['scan']['acc'])
        pose, _ = self.probe.read()
        self.probe.move([*pose[:2], max(p.safe_z(), pose[2])], self.down, m['approach_speed'], acc=m['approach_acc'])

    def run(self):
        Teleop(self.program).go(PoseBook().joints('home'), 'HOME')  # force/compliance released if left on,
        # travel height above the vehicle
        self.down = rotation_zyz(self.probe.read()[0][3:])
        for number, spans in self.paths:
            self.follow(number, spans)
        self.program.home()
        return self.save()

    def save(self):
        s = np.asarray(self.samples)
        model = np.array([self.model(row[2:5], row[5:8]) for row in s])
        directory = output_dir('images') / f'motion_offset_check_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        directory.mkdir(parents=True, exist_ok=True)
        np.savetxt(directory / 'samples.csv', np.c_[s, model], delimiter=',', comments='',
                   header='t_s,path,x_mm,y_mm,z_mm,dx,dy,dz,fx_n,fy_n,fz_n,model_fx_n,model_fy_n,model_fz_n')
        t, raw, corrected = s[:, 0] - s[0, 0], s[:, 10], s[:, 10] - model[:, 2]
        threshold = self.program.config['real_scan']['mesh_height']['contact_threshold_n']
        stats = lambda f: (f'mean {f.mean():+.3f} N, std {f.std():.3f} N, |F| p95 {np.percentile(np.abs(f), 95):.3f} N, '
                           f'|F| > {threshold:g} N {np.mean(np.abs(f) > threshold):.1%}')
        fig, (top, bottom) = plt.subplots(2, 1, sharex=True, sharey=True, figsize=(14, 8))
        top.plot(t, raw, color='.45', linewidth=.5, label='raw Fz')
        top.plot(t, model[:, 2], color='tab:red', linewidth=.8, linestyle='--', label='offset model')
        top.set(ylabel='BASE Fz (N)', title=f'Uncorrected, in the air ({self.lift:g} mm above the plan): {stats(raw)}')
        bottom.plot(t, corrected, color='tab:blue', linewidth=.5, label='raw - model')
        bottom.set(xlabel='Time (s)', ylabel='BASE Fz (N)', title=f'Corrected: {stats(corrected)}')
        for panel in (top, bottom):
            for level in (threshold, -threshold):
                panel.axhline(level, color='tab:orange', linewidth=.7, linestyle=':')
            for change in t[np.flatnonzero(np.diff(s[:, 1])) + 1]:
                panel.axvline(change, color='k', linewidth=.6)
            panel.axhline(0, color='k', linewidth=.4)
            panel.legend(loc='upper right')
            panel.grid(alpha=.3)
        fig.tight_layout()
        fig.savefig(directory / 'report.png', dpi=130)
        plt.close(fig)
        return directory, stats(raw), stats(corrected)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--plan', help='mesh-height plan.json (default: the latest real_bonnet_mesh_height report)')
    parser.add_argument('--lift', type=float, default=10., help='height above the planned path, mm (default 10)')
    parser.add_argument('--path', type=int, nargs='*', help='path numbers to follow (default: all)')
    options = parser.parse_args(program_args())
    plan = options.plan or str(sorted(output_dir('images').glob('real_bonnet_mesh_height_*/plan.json'))[-1])
    program = RobotProgram('motion_offset_check')
    try:
        program.log(f'계획: {plan}')
        directory, raw, corrected = MotionOffsetCheck(program, plan, options.lift, options.path).run()
        program.log(f'보정 전: {raw}')
        program.log(f'보정 후: {corrected}')
        program.log(f'-> {directory / "report.png"}')
    finally:
        program.close()


if __name__ == '__main__':
    main()
