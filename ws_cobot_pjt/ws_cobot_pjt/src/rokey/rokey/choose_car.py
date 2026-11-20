"""Identify the vehicle type and its pose in the zone by touching characteristic body points.

Hypotheses are type x (x, y, yaw, front/back) on a grid over robot.yaml `vehicle_zone`. Each touch goes
where the predicted height of the likely hypotheses is most uncertain (highest entropy over sigma-wide
height bins, i.e. the most informative touch; body only: bonnet and mirrors may be damaged, wheels are
soft, and never next to a taller surface the gripper would hit), and every touch re-weights them with a
robust height likelihood. Once one type dominates, a fine
pose grid around the best hypothesis is refined the same way.

Run after `pick_probe` (and `calibration`); the result goes to
/workspace/session/vehicle.json and /repair/vehicle. Offline check without the robot:
  ros2 run rokey choose_car
  ros2 run rokey choose_car --simulate 50
"""
import argparse

if __package__:
    from .robot_namespace import program_args
else:
    from robot_namespace import program_args
import json
import time

import numpy as np

if __package__:
    from .configs import load, read_session, write_session
    from .vehicle_model import VehicleModel
else:
    from configs import load, read_session, write_session
    from vehicle_model import VehicleModel

AVOID = ('bonnet', 'mirror_l', 'mirror_r', 'wheel')


def rotate(points, yaw_deg):
    c, s = np.cos(np.deg2rad(yaw_deg)), np.sin(np.deg2rad(yaw_deg))
    return np.stack([c * points[..., 0] - s * points[..., 1], s * points[..., 0] + c * points[..., 1]], -1)


class Hypotheses:
    """Vehicle type index and BASE pose (x, y mm, yaw deg) per hypothesis, with a log weight."""

    def __init__(self, kinds, x, y, yaw):
        grid = np.meshgrid(kinds, x, y, yaw, indexing='ij')
        self.kind, self.x, self.y, self.yaw = (axis.ravel() for axis in grid)
        self.log_weight = np.zeros(len(self.kind))

    def weights(self):
        w = np.exp(self.log_weight - self.log_weight.max())
        return w / w.sum()

    def best(self):
        return int(np.argmax(self.log_weight))


class VehicleIdentifier:
    def __init__(self, models, zone, settings, probe):
        self.models, self.names = models, list(models)
        self.zone, self.settings, self.radius = zone, settings, probe['tip_radius_mm']
        self.body_radius, self.body_clearance = probe['body_radius_mm'], probe['body_clearance_mm']
        self.touches, self.phase, self.rng = [], 'type', np.random.default_rng(0)
        tolerance, step = zone['position_tolerance_mm'], settings['grid_mm']
        x0, y0 = zone['center_mm']
        offsets = np.arange(-tolerance, tolerance + 1e-6, step)
        yaw = zone['yaw_deg'] + np.arange(-zone['yaw_tolerance_deg'], zone['yaw_tolerance_deg'] + 1e-6,
                                          settings['grid_deg'])
        self.hypotheses = Hypotheses(np.arange(len(self.names)), x0 + offsets, y0 + offsets,
                                     np.r_[yaw, yaw + 180.0])
        reach = max(np.hypot(*model.size[:2] / 2) for model in models.values()) + tolerance + 5
        self.candidates = self.grid(x0, y0, reach, reach, 0.0, settings['candidate_mm'][0])

    @staticmethod
    def grid(x, y, half_x, half_y, yaw, step):
        u, v = np.meshgrid(np.arange(-half_x, half_x + 1e-6, step), np.arange(-half_y, half_y + 1e-6, step))
        return rotate(np.c_[u.ravel(), v.ravel()], yaw) + [x, y]

    def predict(self, index, points, probe=False):
        """Predicted tip z (BASE mm) of hypotheses `index` at BASE xy `points`, (H, N), and where a touch
        is not wanted: a replaceable part or a wheel, or a surface within the probe body radius rising
        more than its clearance above the tip (the gripper would hit it)."""
        h = self.hypotheses
        z = np.zeros((len(index), len(points)))
        blocked = np.zeros(z.shape, bool)
        for k, name in enumerate(self.names):
            rows = np.nonzero(h.kind[index] == k)[0]
            if not len(rows):
                continue
            i = index[rows]
            local = rotate(points[None] - np.c_[h.x[i], h.y[i]][:, None], -h.yaw[i][:, None])
            model = self.models[name]
            if probe:
                z[rows] = model.probe_height(local.reshape(-1, 2), self.radius).reshape(len(rows), -1)
            else:
                surface, label = model.surface(local)
                z[rows] = np.nan_to_num(surface, nan=0.0)
                blocked[rows] = np.isin(label, AVOID) | \
                    (model.shadow(local, self.body_radius) - z[rows] > self.body_clearance)
        return z + self.zone['table_z_mm'], blocked

    def sigma(self):
        return self.settings['sigma_mm'][0 if self.phase == 'type' else 1]

    def active(self, limit=3000):
        """The most likely hypotheses (ties in random order)."""
        w = self.hypotheses.weights()
        index = np.nonzero(w > 1e-4 * w.max())[0]
        return index[np.lexsort((self.rng.random(len(index)), -w[index]))[:limit]]

    def next_point(self):
        """(x, y, top, bottom) of the most informative body point."""
        index = self.active()
        w = self.hypotheses.weights()[index]
        w /= w.sum()
        z, blocked = self.predict(index, self.candidates)
        bins = np.floor(z / self.sigma()).astype(int)
        bins -= bins.min()
        entropy = np.empty(len(self.candidates))
        for c in range(len(self.candidates)):
            p = np.bincount(bins[:, c], w)
            p = p[p > 0]
            entropy[c] = -(p * np.log(p)).sum()
        entropy[w @ blocked > 0.05] = -1
        for touch in self.touches:  # one touch per spot
            entropy[np.hypot(*(self.candidates - touch[:2]).T) < self.settings['candidate_mm'][-1]] = -1
        c = int(np.argmax(entropy))
        likely = w > 1e-3 * w.max()
        return (*self.candidates[c], z[likely, c].max(), z[likely, c].min())

    def update(self, x, y, z):
        self.touches.append(np.array([x, y, z]))
        self.rescore()
        if self.phase == 'type' and len(self.touches) >= self.settings['min_touches'] and \
                self.pose_mass() >= self.settings['type_confidence']:
            self.refine()
        elif self.phase == 'pose' and self.at_edge():
            self.refine()  # the best pose is on the fine grid border: re-centre on it

    def rescore(self):
        """Robust Gaussian log likelihood of all touches (residual capped at 3 sigma)."""
        sigma = self.sigma()
        touches = np.array(self.touches)
        predicted, _ = self.predict(np.arange(len(self.hypotheses.kind)), touches[:, :2], probe=True)
        r = np.minimum(np.abs(predicted - touches[:, 2]) / sigma, 3.0)
        self.hypotheses.log_weight = -0.5 * (r ** 2).sum(1)

    def type_mass(self):
        h, w = self.hypotheses, self.hypotheses.weights()
        return np.bincount(h.kind, w, len(self.names)).max()

    def near_best(self, span, turn):
        h = self.hypotheses
        best = h.best()
        dyaw = (h.yaw - h.yaw[best] + 180) % 360 - 180
        return (h.kind == h.kind[best]) & (np.abs(h.x - h.x[best]) <= span + 1e-6) & \
            (np.abs(h.y - h.y[best]) <= span + 1e-6) & (np.abs(dyaw) <= turn + 1e-6)

    def pose_mass(self):
        """Weight of the best type within one coarse step of the best pose (front/back settled too)."""
        return self.hypotheses.weights()[self.near_best(self.settings['grid_mm'], self.settings['grid_deg'])].sum()

    def at_edge(self):
        settings = self.settings
        inner = self.near_best(settings['grid_mm'] - settings['fine_mm'], settings['grid_deg'] - settings['fine_deg'])
        return inner.sum() < len(inner)  # some neighbours of the best fall outside the grid

    def refine(self):
        """Fine pose grid around the best hypothesis of the winning type; candidates on that vehicle."""
        h, settings = self.hypotheses, self.settings
        best = h.best()
        kind, x, y, yaw = h.kind[best], h.x[best], h.y[best], h.yaw[best]
        span, step = settings['grid_mm'], settings['fine_mm']
        offsets = np.arange(-span, span + 1e-6, step)
        turns = np.arange(-settings['grid_deg'], settings['grid_deg'] + 1e-6, settings['fine_deg'])
        self.hypotheses, self.phase = Hypotheses([kind], x + offsets, y + offsets, yaw + turns), 'pose'
        size = self.models[self.names[kind]].size
        self.candidates = self.grid(x, y, size[0] / 2 + 5, size[1] / 2 + 5, yaw, settings['candidate_mm'][1])
        self.rescore()

    def spread(self):
        h, w = self.hypotheses, self.hypotheses.weights()
        return [float(np.sqrt(w @ (value - w @ value) ** 2)) for value in (h.x, h.y, h.yaw)]

    def done(self):
        settings = self.settings
        if len(self.touches) >= settings['max_touches']:
            return True
        if self.phase != 'pose':
            return False
        sx, sy, syaw = self.spread()
        return max(sx, sy) <= settings['pose_std_mm'] and syaw <= settings['pose_std_deg']

    def result(self):
        h = self.hypotheses
        best = h.best()
        predicted, _ = self.predict(np.array([best]), np.array(self.touches)[:, :2], probe=True)
        residual = predicted[0] - np.array(self.touches)[:, 2]
        sx, sy, syaw = self.spread()
        return {'type': self.names[h.kind[best]], 'confident': self.phase == 'pose',
                'type_probability': float(self.type_mass()) if self.phase == 'type' else 1.0,
                'pose_base_mm_deg': [float(h.x[best]), float(h.y[best]), float((h.yaw[best] + 180) % 360 - 180)],
                'table_z_mm': self.zone['table_z_mm'], 'pose_std_mm_deg': [sx, sy, syaw],
                'rms_mm': float(np.sqrt(np.mean(residual ** 2))),
                'touches': [[round(float(v), 3) for v in touch] for touch in self.touches]}


def compare(result, truth):
    """Errors against the simulation scene truth (verification only)."""
    dx, dy = np.subtract(result['pose_base_mm_deg'][:2], truth['pose_base_mm_deg'][:2])
    dyaw = (result['pose_base_mm_deg'][2] - truth['pose_base_mm_deg'][2] + 180) % 360 - 180
    return {'type_ok': result['type'] == truth['type'], 'position_error_mm': float(np.hypot(dx, dy)),
            'yaw_error_deg': float(abs(dyaw))}


def identifier(config):
    models = {name: VehicleModel(name) for name in load('vehicles')['types']}
    return VehicleIdentifier(models, config['vehicle_zone'], config['choose_car'], config['probe'])


def simulate(trials, sigma, seed=0):
    """Random intact vehicles in the zone, touches from the true height map plus noise."""
    config, rng = load('robot'), np.random.default_rng(seed)
    zone, names, records = config['vehicle_zone'], list(load('vehicles')['types']), []
    truths = {name: VehicleModel(name) for name in names}
    for trial in range(trials):
        kind = names[rng.integers(len(names))]
        pose = [*(np.array(zone['center_mm']) + rng.uniform(-1, 1, 2) * zone['position_tolerance_mm']),
                zone['yaw_deg'] + rng.uniform(-1, 1) * zone['yaw_tolerance_deg'] + rng.choice([0.0, 180.0])]
        solver, start = identifier(config), time.monotonic()
        while not solver.done():
            x, y, _, _ = solver.next_point()
            local = rotate(np.array([x, y]) - pose[:2], -pose[2])
            z = truths[kind].probe_height(local, config['probe']['tip_radius_mm'])[0] + zone['table_z_mm']
            solver.update(x, y, z + rng.normal(0, sigma))
        result = solver.result()
        error = compare(result, {'type': kind, 'pose_base_mm_deg': pose})
        records.append({**error, 'touches': len(result['touches']), 'seconds': time.monotonic() - start})
        print(f'{trial + 1:3d} {kind:6s} -> {result["type"]:6s} {"ok " if error["type_ok"] else "BAD"} '
              f'pos {error["position_error_mm"]:5.2f} mm yaw {error["yaw_error_deg"]:4.2f} deg, '
              f'{records[-1]["touches"]} touches, {records[-1]["seconds"]:.1f} s compute', flush=True)
    ok = np.array([r['type_ok'] for r in records])
    position = np.array([r['position_error_mm'] for r in records])[ok]
    yaw = np.array([r['yaw_error_deg'] for r in records])[ok]
    print(f'type accuracy {ok.mean() * 100:.1f}% ({ok.sum()}/{len(ok)}), position error mean {position.mean():.2f} / '
          f'max {position.max():.2f} mm, yaw mean {yaw.mean():.2f} / max {yaw.max():.2f} deg, touches mean '
          f'{np.mean([r["touches"] for r in records]):.1f}')


def identify(program):
    """Touch until the vehicle is identified; writes session/vehicle.json and publishes /repair/vehicle."""
    from std_msgs.msg import String
    if __package__:
        from .touch import VerticalTouch
    else:
        from touch import VerticalTouch
    probe = program.config['probe']
    program.robot.verify_profiles(probe['tool_name'], probe['tcp_name'])
    program.home()
    touch, solver = VerticalTouch(program), identifier(program.config)
    depth, clearance = program.config['scan']['search_depth_mm'], program.config['scan']['clearance_mm']
    while not solver.done():
        x, y, high, low = solver.next_point()
        sample = touch.touch(x, y, high + clearance, low - depth, height=touch.safe_z)
        if sample is None:
            raise RuntimeError(f'({x:.0f}, {y:.0f})에서 예상 높이 {low:.0f}~{high:.0f} mm 안에 접촉이 없습니다: '
                               '차량이 지정 구역(vehicle_zone) 안에 있는지 확인하세요')
        solver.update(sample['x'], sample['y'], sample['z'])
        best = solver.hypotheses.best()
        program.log(f'touch {len(solver.touches)}: ({sample["x"]:.1f}, {sample["y"]:.1f}) z={sample["z"]:.2f} mm '
                    f'-> {solver.phase}: {solver.names[solver.hypotheses.kind[best]]} '
                    f'p={solver.type_mass():.3f}, spread x/y/yaw ' + '/'.join(f'{v:.1f}' for v in solver.spread()))
    result = solver.result()
    truth = read_session('scene_truth.json') if program.mode == 'virtual' else None
    if truth:
        result['check_against_scene_truth'] = compare(result, truth)
    path = write_session('vehicle.json', result)
    program.vehicle.publish(String(data=json.dumps(result)))
    program.log(f'{result["type"]} at {np.round(result["pose_base_mm_deg"], 1).tolist()} '
                f'(rms {result["rms_mm"]:.2f} mm, {len(result["touches"])} touches, confident={result["confident"]})'
                + (f', truth: {result["check_against_scene_truth"]}' if truth else '') + f' -> {path}')
    program.home()
    return result


def fixed(program):
    """The fixed vehicle pose instead of touches: the virtual session's scene (scene_truth.json) or robot.yaml
    vehicle_zone.fixed. Same session/vehicle.json and /repair/vehicle as identify()."""
    from std_msgs.msg import String
    zone, types = program.config['vehicle_zone'], load('vehicles')['types']
    truth = read_session('scene_truth.json') if program.mode == 'virtual' else None
    if truth and truth.get('type') in types:
        kind, pose, table_z, source = truth['type'], truth['pose_base_mm_deg'], truth['table_z_mm'], 'scene_truth'
    else:
        spec = zone['fixed']
        kind, pose, table_z, source = spec['type'], spec['pose_base_mm_deg'], zone['table_z_mm'], 'robot.yaml'
    if kind not in types:
        raise ValueError(f'고정 차량 종류 {kind} 없음 (vehicles.yaml types: {", ".join(types)})')
    result = {'type': kind, 'confident': True, 'type_probability': 1.0,
              'pose_base_mm_deg': [float(v) for v in pose], 'table_z_mm': float(table_z),
              'pose_std_mm_deg': [0.0, 0.0, 0.0], 'rms_mm': 0.0, 'touches': [], 'source': source}
    path = write_session('vehicle.json', result)
    program.vehicle.publish(String(data=json.dumps(result)))
    program.log(f'{kind} at {result["pose_base_mm_deg"]} (fixed, {source}) -> {path}')
    return result


def run():
    if __package__:
        from .program import RobotProgram
    else:
        from program import RobotProgram
    program = RobotProgram('choose_car')
    try:
        identify(program)
        time.sleep(0.5)  # let late /repair/vehicle subscribers get the latched result
    finally:
        program.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--simulate', type=int, metavar='TRIALS', help='offline trials without the robot')
    parser.add_argument('--sigma', type=float, default=0.2, help='simulated height noise, mm (default: 0.2)')
    options = parser.parse_args(program_args())
    simulate(options.simulate, options.sigma) if options.simulate else run()


if __name__ == '__main__':
    main()
