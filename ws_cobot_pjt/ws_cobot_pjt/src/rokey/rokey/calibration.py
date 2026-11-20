"""Pre-task calibration with the probe held (after `pick_probe`), under the same
compliance as the contact search:
  1. force model    static poses, sweeps and held-out motions -> calibration/force_physics_<mode>.json
                    (best held-out of none / static / inertia / friction) and its stability plot
  2. motion noise   vertical strokes at the touch speeds -> the debounced false-touch level -> touch threshold
  3. repeatability  touches on a free table spot -> height sigma (damage thresholds of scan_surface)
The result goes to /workspace/session/calibration_result.json; choose_car and scan_surface read it via Config.
  ros2 run rokey calibration [--skip-force] [--touches 20]
"""
import argparse

if __package__:
    from .robot_namespace import program_args
else:
    from robot_namespace import program_args

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

if __package__:
    from .configs import write_session
    from .force_data import CALIBRATION_DIR, CalibrationRun, Excitation
    from .force_physics import PhysicsCompensator
    from .force_report import save_stability_plot
    from .rotations import rotation_zyz
    from .touch import VerticalTouch
else:
    from configs import write_session
    from force_data import CALIBRATION_DIR, CalibrationRun, Excitation
    from force_physics import PhysicsCompensator
    from force_report import save_stability_plot
    from rotations import rotation_zyz
    from touch import VerticalTouch


class Calibration:
    def __init__(self, run):
        self.run, self.probe = run, run.probe
        self.settings, self.scan = run.config['calibration'], run.config['scan']

    def force_model(self):
        run, settings = self.run, self.settings
        excitation = Excitation(self.probe, run.recorder, settings['lift_mm'], settings['span_mm'], compliance=True)
        run.log('force model 1/3: static payload poses')
        excitation.static_set()
        run.log('force model 2/3: sweeps')
        excitation.sweeps(settings['sweep_speeds'])
        train = run.recorder.take()
        run.log(f'force model 3/3: held-out motions ({len(train.t)} training samples)')
        excitation.validation()
        test = run.recorder.take()
        best, model, scores = PhysicsCompensator.select(train, test)
        path = CALIBRATION_DIR / f'force_physics_{run.mode}.json'
        model.save(path, {**run.meta, 'model': best, 'compliance': True, 'train_samples': len(train.t),
                          'held_out_samples': len(test.t), 'held_out': scores})
        image = save_stability_plot(test, model.correct(test), f'Force calibration ({run.mode}, tcp={run.meta["tcp"]}, '
                                    f'model={best}) - held-out free-space motion', f'force_physics_{run.mode}')
        run.log('  held-out Fn RMS / peak [N]: ' + ', '.join(
            f'{name} {score["fn_rms"]:.3f}/{score["fn_peak"]:.3f}' for name, score in scores.items()) + f' -> {best}')
        excitation.finish()
        return {'file': str(path), 'plot': str(image), 'model': best, 'held_out': scores}

    def noise(self):
        """Tared Fn while descending in free space like a search. A touch needs `hits_required` samples in a
        row above the threshold, so the debounced level (largest value held that long) is where a false
        touch would fire."""
        run, probe, settings = self.run, self.probe, self.settings
        run.home()
        pose, _ = probe.read()
        rotation, start = rotation_zyz(pose[3:]), pose[:3] + [0.0, 0.0, settings['lift_mm']]
        probe.move(start, rotation, run.config['motion']['approach_speed'])
        if not probe.compliant:
            probe.enable()
        result = {}
        for speed in settings['touch_speeds']:
            forces, levels = [], []
            for _ in range(settings['strokes']):
                probe.tare()
                stroke = []
                probe.move(start - [0.0, 0.0, settings['stroke_mm']], rotation, speed,
                           until=lambda pose, wrench: stroke.append(probe.normal_force(pose, wrench)))
                probe.move(start, rotation, run.config['motion']['approach_speed'])
                forces += stroke
                levels.append(sliding_window_view(stroke, probe.hits_required).min(1).max())
            forces = np.asarray(forces)
            result[f'{speed:g}'] = {'fn_std': float(forces.std()), 'fn_peak': float(np.abs(forces).max()),
                                    'debounced': float(max(levels)), 'samples': len(forces)}
            run.log(f'noise {speed:g} mm/s: ' + ', '.join(f'{key} {value:.3f}' for key, value in result[f'{speed:g}'].items()))
        level = max(item['debounced'] for item in result.values())
        threshold = max(settings['min_threshold_n'], settings['threshold_margin'] * level)
        if threshold > 0.5 * self.scan['press_force']:
            run.log(f'  경고: 임계값 {threshold:.2f} N이 누름 힘 {self.scan["press_force"]} N의 절반을 넘습니다 '
                    '(탐색 속도를 낮추거나 press_force를 올리세요)')
        return result, threshold

    def repeatability(self, count):
        run, settings = self.run, self.settings
        touch = VerticalTouch(run)
        table, (x, y) = run.config['vehicle_zone']['table_z_mm'], settings['reference_xy_mm']
        touches = []
        for index in range(count):
            touch.speed = settings['touch_speeds'][index % len(settings['touch_speeds'])]
            sample = touch.touch(x, y, table + self.scan['clearance_mm'], table - self.scan['search_depth_mm'],
                                 height=touch.safe_z)
            run.log(f'reference {index + 1}/{count} at {touch.speed:g} mm/s: '
                    + (f'z={sample["z"]:.3f} mm, Fn={sample["fn"]:.2f} N' if sample else '접촉 없음'))
            if sample:
                touches.append({**sample, 'speed': touch.speed})
        if len(touches) < 3:
            raise RuntimeError(f'기준면 접촉이 {len(touches)}회뿐입니다: reference_xy_mm와 table_z_mm를 확인하세요')
        z = np.array([item['z'] for item in touches])
        by_speed = {f'{speed:g}': float(z[[item['speed'] == speed for item in touches]].std())
                    for speed in settings['touch_speeds']}
        search = [item['z'] for item in touches if item['speed'] == self.scan['search_speed']]
        sigma = float(np.std(search) if len(search) >= 3 else z.std())
        return {'xy_mm': [x, y], 'table_z_mm': table, 'touches': touches, 'sigma_by_speed_mm': by_speed,
                'z_mean_mm': float(z.mean()), 'bias_mm': float(z.mean() - table),
                'false_touches': touch.false_touches}, sigma



def calibrate(run, touches=None, skip_force=False):
    """All three steps on a CalibrationRun holding the probe; writes and returns the session result."""
    probe = run.config['probe']
    run.robot.verify_profiles(probe['tool_name'], probe['tcp_name'])
    run.meta.update(tool=run.robot.get_tool(), tcp=run.robot.get_tcp())
    calibration = Calibration(run)
    force = None if skip_force else calibration.force_model()
    noise, threshold = calibration.noise()
    run.probe.touch_threshold = threshold
    reference, sigma = calibration.repeatability(touches or run.config['calibration']['reference_touches'])
    inspection = run.config['inspection']
    height_sigma = max(sigma, inspection['min_height_sigma_mm'])
    result = {**{key: run.meta[key] for key in ('mode', 'tool', 'tcp')}, 'force_model': force, 'noise': noise,
              'touch_threshold_n': threshold, 'search_speed': run.config['scan']['search_speed'],
              'reference': reference, 'measured_sigma_mm': sigma, 'height_sigma_mm': height_sigma,
              'suspect_mm': inspection['suspect_sigma'] * height_sigma, 'dent_mm': inspection['dent_sigma'] * height_sigma}
    path = write_session('calibration_result.json', result)
    run.log(f'touch threshold {threshold:.3f} N, height sigma {height_sigma:.3f} mm (measured {sigma:.3f}), '
            f'table bias {reference["bias_mm"]:+.3f} mm, false touches {reference["false_touches"]} -> {path}')
    run.reload()
    run.home()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--skip-force', action='store_true', help='keep the existing force model (steps 2-3 only)')
    parser.add_argument('--touches', type=int, help='reference touches (default: robot.yaml)')
    options = parser.parse_args(program_args())
    run = CalibrationRun('calibration')
    try:
        calibrate(run, options.touches, options.skip_force)
    finally:
        run.close()


if __name__ == '__main__':
    main()
