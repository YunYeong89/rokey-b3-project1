"""Force-estimate offset while moving, in the air over a part: run once per setup (probe held, vehicle in place) and
applied by every real_scan mesh-height run. From HOME up and across, then lines along +-x and +-y motion_offset.air_mm
above the part's highest reference point at the mesh-height stroke speed and +-z moves of vertical_mm at the contact
search speed, each after a still reading, in position control. Fitted per BASE force component
  F = c0 + c_moving |d| + C_d d + C_p (p - centre)      (d: unit travel direction, 0 still; p: TCP, BASE mm)
-> config/motion_offset.json (+ the samples). The scan takes the model's change from the press's still reading
off the stroke samples and the contact search: the reading moved with the travel direction (~+-0.2 N, joint
friction) and the pose (~+0.9 N over the bonnet), past a 0.5 N contact rise.
  ros2 run repair_car motion_offset [--part bonnet]"""
import argparse
import json

import numpy as np

from repair_car.config import PACKAGE, SESSION, read_session, write_session
from repair_car.robot_control.program import RobotProgram
from repair_car.robot_control.robot_bridge import program_args
from repair_car.robot_control.rotations import rotation_zyz


class OffsetModel:
    FEATURES = ['one', 'moving', 'dx', 'dy', 'dz', 'x', 'y', 'z']

    def __init__(self, theta, centre):
        self.theta, self.centre = np.asarray(theta), np.asarray(centre)

    @classmethod
    def fit(cls, samples):
        """samples: rows [x, y, z (mm), dx, dy, dz, fx, fy, fz (N)] -> (model, residual std, raw std) per component"""
        samples = np.asarray(samples)
        centre = samples[:, :3].mean(0)
        X = np.c_[np.ones(len(samples)), np.linalg.norm(samples[:, 3:6], axis=1), samples[:, 3:6],
                  samples[:, :3] - centre]
        theta, *_ = np.linalg.lstsq(X, samples[:, 6:9], rcond=None)
        return cls(theta, centre), (samples[:, 6:9] - X @ theta).std(0), samples[:, 6:9].std(0)

    def __call__(self, position, direction):
        """Predicted BASE force reading (N) at `position` (mm) moving along the unit `direction` (zeros: still)."""
        direction = np.asarray(direction, float)
        x = np.r_[1., np.linalg.norm(direction), direction, np.asarray(position, float) - self.centre]
        return x @ self.theta

    @classmethod
    def load(cls, program):
        """The stored model for this mode, tool and TCP, else None."""
        data = read_session('motion_offset.json')
        robot = program.robot
        if data and (data['mode'], data['tool'], data['tcp']) == (program.mode, robot.get_tool(), robot.get_tcp()):
            return cls(data['theta'], data['centre_mm'])
        return None


class MotionOffset:
    def __init__(self, program, part):
        self.program, self.probe, self.part = program, program.probe, part
        self.settings = program.config['motion_offset']
        self.samples = []

    def region(self):
        """Line positions over the part (quantiles of its reference points), the air height (BASE mm) and the scene's
        TCP profile."""
        from repair_car.pointcloud.cloud import PartCloud
        from repair_car.geometry.real_vehicle import RealVehicle
        from repair_car.inspection.scan_scene import ScanScene
        from repair_car.geometry.vehicle_model import pose_matrix
        truth, plan = ScanScene.load(self.program.mode, 'real', holder=False)
        pose = pose_matrix(*truth['pose_base_mm_deg'], truth['base_z_mm'])
        self.program.top_mm = pose[2, 3] + truth['size_mm'][2]
        scan = self.program.config['real_scan']
        points = PartCloud(RealVehicle(), self.part, pose, scan['spacing_mm'],
                           PACKAGE / 'pointcloud' / f'real_{self.part}_reference.ply').points
        x, y = np.quantile(points[:, :2], self.settings['quantiles'], axis=0).T
        return x, y, points[:, 2].max() + self.settings['air_mm'], plan['tcp_mm_deg']

    def reachable(self, tcp):
        """Outer-reach check of BASE points (mm) at the tool-down orientation, reach_margin_mm in (RobotChain, as the
        mesh plan's lifted entries): the controller refused a lift 2 mm beyond it at the far corner (x 610 mm)."""
        from repair_car.inspection.part_scan.tool import robot_chain
        chain = robot_chain(self.program.robot, tcp, self.program.config['real_scan']['ik'])
        margin = self.settings['reach_margin_mm'] / 1000
        return lambda points: bool(chain.within_outer_reach(np.atleast_2d(points) / 1000, self.down, margin).all())

    def still(self, seconds):
        for _ in range(max(1, int(seconds / .01))):
            pose, wrench = self.probe.read()
            self.samples.append([*pose[:3], 0., 0., 0., *wrench[:3]])

    def stroke(self, target, speed):
        """A recorded move from the still TCP straight to `target` (its unit direction the travel direction)."""
        self.still(self.settings['rest_s'])
        pose, _ = self.probe.read()
        direction = np.subtract(target, pose[:3])
        direction /= np.linalg.norm(direction)

        def record(pose, wrench):
            self.samples.append([*pose[:3], *direction, *wrench[:3]])
            return False
        self.probe.move(target, self.down, speed, until=record, acc=self.program.config['scan']['acc'])

    def run(self):
        """Measured over the part and saved, then back HOME: in and out over the corner nearest the robot only, the
        far lines pulled in until every pose is within the outer reach."""
        p, m, s = self.program, self.program.config['motion'], self.settings
        speed = p.config['real_scan']['mesh_height']['cruise_speed_mm_s']
        search = p.config['real_scan']['default_scan']['real_search_speed_mm_s']
        x, y, z, tcp = self.region()  # x, y: the quantile positions along BASE x and y (x grows away from the robot)
        p.home()
        pose, _ = self.probe.read()
        self.down = rotation_zyz(pose[3:])
        travel, ok = max(p.safe_z(), z), self.reachable(tcp)
        far = x[-1]
        while not ok([[x[-1], y[0], z], [x[-1], y[-1], z]]):
            x = np.minimum(x, x[-1] - 1.)
        if not ok([x[0], y[0], travel]):
            raise RuntimeError(f'Entry point ({x[0]:.0f}, {y[0]:.0f}, {travel:.0f}) mm is out of reach')
        p.log(f'Moving offset measurement in the air over {self.part}: x {x.round(0).tolist()} (far side pulled in {far - x[-1]:.0f} mm), '
              f'y {y.round(0).tolist()}, z {z:.1f} mm; horizontal {speed:g} mm/s, vertical {search:g} mm/s')
        self.probe.move([*pose[:2], travel], self.down, m['approach_speed'], acc=m['approach_acc'])
        self.probe.move([x[0], y[0], travel], self.down, m['linear_speed'], acc=m['linear_acc'])
        self.probe.move([x[0], y[0], z], self.down, m['approach_speed'], acc=m['approach_acc'])
        lines = [((a, y[0]), (a, y[-1])) for a in x] + [((x[0], b), (x[-1], b)) for b in y]
        for start, end in lines:  # each both ways: +-y, then +-x
            self.probe.move([*start, z], self.down, m['linear_speed'], acc=m['linear_acc'])
            self.stroke([*end, z], speed)
            self.stroke([*start, z], speed)
        for a in x:
            for b in y:  # down and up at each line crossing
                self.probe.move([a, b, z], self.down, m['linear_speed'], acc=m['linear_acc'])
                self.stroke([a, b, z - s['vertical_mm']], search)
                self.stroke([a, b, z], search)
        self.probe.move([x[0], y[0], z], self.down, m['linear_speed'], acc=m['linear_acc'])
        self.still(s['rest_s'])
        result = self.save(z)
        self.probe.move([x[0], y[0], travel], self.down, m['approach_speed'], acc=m['approach_acc'])
        p.home()
        return result

    def save(self, z):
        p = self.program
        model, residual, raw = OffsetModel.fit(self.samples)
        samples = np.asarray(self.samples)
        path = write_session('motion_offset.json', dict(
            mode=p.mode, tool=p.robot.get_tool(), tcp=p.robot.get_tcp(), part=self.part, air_z_mm=round(float(z), 2),
            features=OffsetModel.FEATURES, theta=model.theta.round(6).tolist(), centre_mm=model.centre.round(3).tolist(),
            residual_std_n=residual.round(4).tolist(), raw_std_n=raw.round(4).tolist(), samples=len(samples)))
        SESSION.mkdir(parents=True, exist_ok=True)
        np.savetxt(SESSION / 'motion_offset_samples.csv', samples, delimiter=',', comments='',
                   header='x_mm,y_mm,z_mm,dx,dy,dz,fx_n,fy_n,fz_n')
        p.log(f'Saved -> {path}')
        return path, model, residual, raw


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--part', default='bonnet', help='part to calibrate over (default bonnet)')
    options = parser.parse_args(program_args())
    program = RobotProgram('motion_offset')
    try:
        path, model, residual, raw = MotionOffset(program, options.part).run()
        fz = dict(zip(OffsetModel.FEATURES, model.theta[:, 2].round(4).tolist()))
        program.log(f'Fz model {json.dumps(fz)}; standard deviation {raw[2]:.3f} -> {residual[2]:.3f} N '
                    f'(Fx {raw[0]:.3f} -> {residual[0]:.3f}, Fy {raw[1]:.3f} -> {residual[1]:.3f})')
        program.log(f'-> {path}')
    finally:
        program.close()


if __name__ == '__main__':
    main()
