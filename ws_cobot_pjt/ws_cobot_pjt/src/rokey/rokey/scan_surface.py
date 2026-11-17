"""Inspect parts of the identified vehicle (session/vehicle.json) by rubbing them under TOOL Z force control
with compliance: per part its outline (inset from the edge), then a raster along the vehicle x; small parts
(mirrors) get single touches on a grid. Every point is compared with the reference shape at the identified
pose: dz = measured - reference (negative into the surface).

  missing   no contact down to search_depth below the reference, or contact more than missing_mm below it
  dent      dz below -dent_sigma x sigma        suspect   below -suspect_sigma x sigma, or a crack: a step
                                                          between neighbours above crack_step_mm
dz is first corrected by a robust plane per part (registration residual of the identified pose) and
median-filtered along each path. Points go to /repair/status; the result (findings and the parts to
replace) to /workspace/session/scan_result.json.
  ros2 run rokey scan_surface [--parts bonnet mirror_l mirror_r]
  ros2 run rokey scan_surface --simulate          # offline: the simulation scene damage, no robot
"""
import argparse
import json

import numpy as np

if __package__:
    from .configs import load, read_session, write_session
    from .vehicle_model import VehicleModel, pose_matrix, transform
else:
    from configs import load, read_session, write_session
    from vehicle_model import VehicleModel, pose_matrix, transform


def erode(mask, cells):
    for _ in range(cells):
        mask = mask & np.roll(mask, 1, 0) & np.roll(mask, -1, 0) & np.roll(mask, 1, 1) & np.roll(mask, -1, 1)
    return mask


class PartPlan:
    """Paths (vehicle frame xy, mm) over a part's top surface: (kind, points)."""

    def __init__(self, model, part, inspection, radius):
        self.part, r = part, model.resolution
        mask = model.label == part
        if not mask.any():
            raise ValueError(f'{model.kind} 모델에 {part} 영역이 없습니다')
        self.xy = lambda cells: model.origin + np.asarray(cells) * r
        spacing, lines = inspection['point_spacing_mm'], inspection['line_spacing_mm']
        if mask.sum() * r * r < inspection['small_part_mm2']:  # the tip only has to stay on the part
            inner = erode(mask, int(np.ceil(radius / r)))
            points = self.grid(inner if inner.any() else mask, inspection['small_part_spacing_mm'])
            self.paths = [('touch', point[None]) for point in points]
        else:
            inner = erode(mask, int(np.ceil((radius + inspection['edge_margin_mm']) / r)))
            inner = inner if inner.any() else mask
            self.paths = [('outline', self.outline(inner, spacing))] + \
                         [('raster', line) for line in self.raster(inner, spacing, lines)]

    def grid(self, mask, spacing):
        points = self.xy(np.argwhere(mask))
        keys = np.round(points / spacing)
        return points[np.unique(keys, axis=0, return_index=True)[1]]

    def outline(self, mask, spacing):
        """Boundary cells of the inset region, ordered around its centre, one every `spacing`, closed."""
        points = self.xy(np.argwhere(mask & ~erode(mask, 1)))
        centre = points.mean(0)
        points = points[np.argsort(np.arctan2(*(points - centre).T[::-1]))]
        kept = [points[0]]
        for point in points[1:]:
            if np.linalg.norm(point - kept[-1]) >= spacing:
                kept.append(point)
        return np.array(kept + [kept[0]])

    def raster(self, mask, spacing, line_spacing):
        """Lines along x every `line_spacing` in y, alternating direction, split where the mask is broken."""
        columns = np.nonzero(mask.any(0))[0]
        r = self.xy([1, 1])[0] - self.xy([0, 0])[0]
        result = []
        for j in columns[::max(1, int(round(line_spacing / r)))]:
            rows = np.nonzero(mask[:, j])[0]
            for run in np.split(rows, np.nonzero(np.diff(rows) > 1)[0] + 1):
                line = self.xy(np.c_[run, np.full(len(run), j)])[::max(1, int(round(spacing / r)))]
                result.append(line if len(result) % 2 == 0 else line[::-1])
        return result


class DamageAnalysis:
    """Per-part classification of the measured points (see the module docstring)."""

    def __init__(self, model, inspection, sigma):
        self.model, self.inspection = model, inspection
        self.suspect, self.dent = inspection['suspect_sigma'] * sigma, inspection['dent_sigma'] * sigma

    @staticmethod
    def plane(points, dz, sigma):
        """Robust least-squares plane a + b x + c y through dz (3 sigma / MAD rejection)."""
        inliers = np.ones(len(dz), bool)
        design = np.c_[np.ones(len(dz)), points]
        for _ in range(3):
            coef = np.linalg.lstsq(design[inliers], dz[inliers], rcond=None)[0]
            r = dz - design @ coef
            scale = max(3 * 1.4826 * np.median(np.abs(r[inliers] - np.median(r[inliers]))), 3 * sigma)
            inliers = np.abs(r) <= scale
        return design @ coef

    def median(self, values, paths):
        window, result = self.inspection['median_window'], values.copy()
        for path in np.unique(paths):
            index = np.nonzero(paths == path)[0]
            if len(index) >= window:
                padded = np.pad(values[index], window // 2, mode='edge')
                result[index] = np.median(np.lib.stride_tricks.sliding_window_view(padded, window), axis=1)
        return result

    def part(self, part, records, sigma):
        inspection = self.inspection
        xy = np.array([[r['vx'], r['vy']] for r in records])
        paths = np.array([r['path'] for r in records])
        dz = np.array([np.nan if r['z'] is None else r['z'] - r['reference'] for r in records])
        missing = np.isnan(dz) | (dz < -inspection['missing_mm'])
        state = np.where(missing, 'missing', 'ok').astype(object)
        corrected = dz.copy()
        if (~missing).sum() >= 6:
            corrected[~missing] -= self.plane(xy[~missing], dz[~missing], sigma)
            corrected[~missing] = self.median(corrected[~missing], paths[~missing])
            for path in np.unique(paths[~missing]):  # cracks: steps between neighbours along a path
                index = np.nonzero(~missing & (paths == path))[0]
                step = np.abs(np.diff(corrected[index])) > inspection['crack_step_mm']
                state[index[1:][step]] = state[index[:-1][step]] = 'suspect'
            state[~missing & (corrected < -self.suspect)] = 'suspect'
            state[~missing & (corrected < -self.dent)] = 'dent'
        if missing.mean() >= inspection['missing_fraction']:
            decision = 'missing'
            where, depth = xy.mean(0), None
        else:
            decision = next((name for name in ('dent', 'suspect') if (state == name).any()), 'ok')
            deepest = int(np.nanargmin(np.where(missing, np.nan, corrected)))
            where, depth = xy[deepest], float(np.round(corrected[deepest], 3))
        for record, value, label in zip(records, corrected, state):
            record.update(dz=None if np.isnan(value) else float(value), state=str(label))
        x, y, _ = self.model.to_interface([*where, 0.0])[0]
        finding = None if decision == 'ok' else {'part_code': part, 'x': round(float(x), 1), 'y': round(float(y), 1),
                                                 'depth': depth, 'decision': decision}
        counts = {name: int((state == name).sum()) for name in ('ok', 'suspect', 'dent', 'missing')}
        return {'decision': decision, 'counts': counts, 'finding': finding, 'points': records}


class Inspection:
    """Robot-independent part of the scan: reference heights, point records and the per-part result."""

    def __init__(self, vehicle, config):
        self.config, self.inspection = config, config['inspection']
        self.model = VehicleModel(vehicle['type'])
        self.pose = pose_matrix(*vehicle['pose_base_mm_deg'], vehicle['table_z_mm'])
        self.radius = config['probe']['tip_radius_mm']
        self.sigma = self.inspection['height_sigma_mm']
        self.analysis = DamageAnalysis(self.model, self.inspection, self.sigma)

    def targets(self, points):
        """BASE xy and reference tip z (BASE) of vehicle-frame points."""
        base = transform(self.pose, np.c_[points, np.zeros(len(points))])
        return base[:, :2], self.model.probe_height(points, self.radius) + self.pose[2, 3]

    def record(self, part, path, kind, local, reference, sample):
        z = None if sample is None else sample['z']
        dz = None if z is None else z - reference
        x, y, _ = self.model.to_interface([*local, 0.0])[0]
        missing = dz is None or dz < -self.inspection['missing_mm']
        state = 'dent' if missing or dz < -self.analysis.dent else 'suspect' if dz < -self.analysis.suspect else 'ok'
        status = {'type': 'point', 'x': round(float(x), 2), 'y': round(float(y), 2),
                  'dz': round(float(dz), 3) if dz is not None else -self.inspection['missing_mm'],
                  'state': state, 'part': part, 'missing': bool(missing)}
        return {'path': path, 'kind': kind, 'vx': float(local[0]), 'vy': float(local[1]), 'reference': float(reference),
                'z': z, 'fn': None if sample is None else sample['fn']}, status

    def result(self, vehicle, parts):
        """Findings for the DB (INTERFACE frame) and the replaceable damaged parts."""
        findings = [item['finding'] for item in parts.values() if item['finding']]
        replace = [part for part, item in parts.items()
                   if item['decision'] in ('dent', 'missing') and part in self.inspection['replaceable']]
        return {'vehicle': vehicle, 'height_sigma_mm': self.sigma, 'suspect_mm': self.analysis.suspect,
                'dent_mm': self.analysis.dent, 'parts': parts, 'findings': findings, 'replace': replace}


class SurfaceScanner:
    def __init__(self, program, inspection):
        if __package__:
            from .touch import VerticalTouch
        else:
            from touch import VerticalTouch
        self.program, self.inspection = program, inspection
        self.touch = VerticalTouch(program)
        self.scan = program.config['scan']

    def part(self, name):
        inspection, touch, scan = self.inspection, self.touch, self.scan
        plan = PartPlan(inspection.model, name, inspection.inspection, inspection.radius)
        records, top = [], None
        self.program.log(f'{name}: {len(plan.paths)} paths, {sum(len(p) for _, p in plan.paths)} points')
        for path, (kind, points) in enumerate(plan.paths):
            base, reference = inspection.targets(points)
            top = reference.max() + scan['clearance_mm'] if top is None else max(top, reference.max() + scan['clearance_mm'])
            pressing = False
            for index, (local, (x, y), z) in enumerate(zip(points, base, reference)):
                sample = touch.slide(x, y, z, scan['speed']) if pressing else None
                if sample is None:
                    height = (touch.safe_z if path == 0 else top) if index == 0 else None  # parts apart: over all
                    sample = touch.touch(x, y, z + scan['clearance_mm'], z - scan['search_depth_mm'],
                                         height=height, stay=kind != 'touch')
                pressing = bool(sample and sample['settled']) and kind != 'touch'
                record, status = inspection.record(name, path, kind, local, z, sample)
                records.append(record)
                self.program.publish(**status)
                if sample:
                    self.program.publish(type='wrench', fz=round(sample['fz'], 2))
            if pressing:
                touch.lift(top)
            lost = np.mean([r['z'] is None or r['z'] - r['reference'] < -inspection.inspection['missing_mm']
                            for r in records])
            if len(records) >= 8 and lost >= inspection.inspection['missing_fraction']:
                self.program.log(f'{name}: {lost * 100:.0f}% of {len(records)} points missing, rest skipped')
                break
        return inspection.analysis.part(name, records, inspection.sigma)


def simulate(parts):
    """The planner and the analysis on the simulation scene damage (or a default), touches from the damaged
    height map plus noise. Prints the per-part decision and what the robot would touch."""
    config, truth = load('robot'), read_session('scene_truth.json')
    damage = truth['damage'] if truth else [{'type': 'dent', 'part': 'bonnet', 'offset_mm': [17, 14], 'radius': 11,
                                            'depth': 2.5}, {'type': 'missing', 'part': 'mirror_r'}]  # native/scene.json
    vehicle = {'type': truth['type'] if truth else 'sedan', 'pose_base_mm_deg': [0.0, 0.0, 0.0], 'table_z_mm': 0.0}
    inspection, rng = Inspection(vehicle, config), np.random.default_rng(0)
    damaged = VehicleModel(vehicle['type'], damage)
    result = {}
    for name in parts:
        plan, records = PartPlan(inspection.model, name, inspection.inspection, inspection.radius), []
        for path, (kind, points) in enumerate(plan.paths):
            _, reference = inspection.targets(points)
            measured = damaged.probe_height(points, inspection.radius) + rng.normal(0, inspection.sigma, len(points))
            for local, z, actual in zip(points, reference, measured):
                found = actual >= z - config['scan']['search_depth_mm']
                records.append(inspection.record(name, path, kind, local, z, {'z': actual, 'fn': 0} if found else None)[0])
        result[name] = inspection.analysis.part(name, records, inspection.sigma)
        print(f'{name}: {len(plan.paths)} paths, {len(records)} points -> {result[name]["decision"]} '
              f'{result[name]["counts"]} {result[name]["finding"]}')
    print(f'damage: {damage}\nreplace: {inspection.result(vehicle, result)["replace"]}')


def scan_parts(program, parts):
    """Scan the parts of the identified vehicle; writes and returns session/scan_result.json."""
    vehicle = read_session('vehicle.json')
    if vehicle is None:
        raise RuntimeError('session/vehicle.json 없음: 먼저 choose_car를 실행하세요')
    probe = program.config['probe']
    program.robot.verify_profiles(probe['tool_name'], probe['tcp_name'])
    program.home()
    inspection = Inspection(vehicle, program.config)
    scanner = SurfaceScanner(program, inspection)
    result = {}
    for name in parts:
        result[name] = scanner.part(name)
        program.log(f'{name}: {result[name]["decision"]} {result[name]["counts"]} {result[name]["finding"]}')
    output = inspection.result(vehicle, result)
    output['false_touches'] = scanner.touch.false_touches
    path = write_session('scan_result.json', output)
    program.log(f'findings {output["findings"]}, replace {output["replace"]} -> {path}')
    program.home()
    return output


def run(parts):
    if __package__:
        from .program import RobotProgram
    else:
        from program import RobotProgram
    program = RobotProgram('scan_surface')
    try:
        scan_parts(program, parts)
    finally:
        program.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--parts', nargs='+', help='part codes (default: robot.yaml inspection.replaceable)')
    parser.add_argument('--simulate', action='store_true', help='offline, without the robot')
    options = parser.parse_args()
    parts = options.parts or load('robot')['inspection']['replaceable']
    simulate(parts) if options.simulate else run(parts)


if __name__ == '__main__':
    main()
