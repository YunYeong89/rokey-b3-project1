"""Free-space force per BASE axis in three stages: raw, the physics model applied (held out), and the model + Kalman
layer (force_model/kalman.py). In the air no external force acts, so the corrected curves should stay in the noise band
around 0 (+-2 kalman.noise_n). The third stage is the model's residual less the Kalman bias state, the filter told
"no contact" throughout, as on a scan's approaches and retracts (its bias then follows what the model leaves);
printed alongside, the filter anchored only in the still segments (shaded), as if every move were in contact.
force_model_fit draws it for collected data; this program for
the motion_offset recording (session/motion_offset_samples.csv: positions, travel directions and BASE forces only, the
tool down: q by IK, qd from the direction at the calibration speeds, the time from the distance at that speed; folds
of contiguous eighths as stations; forces only, moments unweighted).
  ros2 run rokey force_model_view [--csv PATH]"""
import argparse
from datetime import datetime

import numpy as np

from .kalman import ForceKalman
from .model import ForceSamples
from .selection import Selection, label, rms


def stats(force, threshold):
    return (f'RMS {rms(force):.3f} N, |F| p95 {np.percentile(np.abs(force), 95):.3f} N, '
            f'>{threshold:g} N {np.mean(np.abs(force) > threshold):.1%}')


def evaluate(samples, best, settings, free):
    """-> raw, model-corrected (held out), the model's residual less the Kalman bias state (the reading corrected by
    the filter) and that bias, BASE x y z (N); free: where the filter is told there is no contact."""
    model = best['corrected'][:, :3]
    _, bias = ForceKalman.run(settings, samples.t, model, free)
    return samples.wrench[:, :3], model, model - bias, bias


def report(samples, best, settings, still, threshold):
    """The three stages in the air (no contact throughout) and, printed, the filter anchored at the still segments
    only. -> the stages"""
    stages = evaluate(samples, best, settings, np.ones(len(still), bool))[:3]
    scan_like = evaluate(samples, best, settings, still)[2]
    for k, axis in enumerate('xyz'):
        for stage, name in zip((*stages, scan_like), ('raw', 'model', 'model+Kalman', 'scan-like')):
            print(f'F{axis} {name:13s} {stats(stage[:, k], threshold)}')
    return stages


def axes_figure(path, t, stages, still, noise, threshold, title):
    """stages: (raw, model, model + Kalman), each (N, 3) BASE force; one panel per axis, the still segments shaded."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = t - t[0]
    fig, panels = plt.subplots(3, 1, sharex=True, figsize=(16, 11))
    for k, (panel, axis) in enumerate(zip(panels, 'xyz')):
        values = np.concatenate([stage[:, k] for stage in stages])
        low, high = np.percentile(values, [.5, 99.5])
        panel.fill_between(t, low, high, where=still, color='.92', step='mid', label='still')
        panel.axhspan(-2 * noise, 2 * noise, color='tab:green', alpha=.15, label=f'noise band +-{2 * noise:.2f} N')
        panel.axhline(0, color='k', linewidth=.6)
        for stage, color, name, width in zip(stages, ('.55', 'tab:orange', 'tab:blue'),
                                             ('raw', 'physics model', 'physics model + Kalman'), (.5, .5, .7)):
            panel.plot(t, stage[:, k], color=color, linewidth=width, label=f'{name}: {stats(stage[:, k], threshold)}')
        panel.set(ylabel=f'BASE F{axis} (N)', ylim=(low, high))
        panel.legend(loc='upper right', fontsize=8)
        panel.grid(alpha=.3)
    panels[0].set_title(title)
    panels[-1].set_xlabel('Time (s)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


class OffsetCsv:
    """The motion_offset recording as ForceSamples (see the module docstring); free: the still readings."""

    def __init__(self, path, chain, home, config, folds=8):
        data = np.loadtxt(path, delimiter=',', skiprows=1)
        position, direction, force = data[:, :3], data[:, 3:6], data[:, 6:9]
        speeds = (config['real_scan']['default_scan']['real_search_speed_mm_s'],
                  config['real_scan']['mesh_height']['cruise_speed_mm_s'])
        moving = np.linalg.norm(direction, axis=1) > 0
        speed = np.where(np.abs(direction[:, 2]) > .5, *speeds) * moving
        step = np.zeros(len(data))
        for k in np.split(np.arange(len(data)), np.flatnonzero(np.any(np.diff(direction, axis=0), axis=1)) + 1):
            if moving[k[0]] and len(k) > 1:  # a run of one direction: its length at its speed, spread evenly
                step[k] = np.linalg.norm(position[k[-1]] - position[k[0]]) / speed[k[0]] / (len(k) - 1)
        step[step == 0] = np.median(step[step > 0])
        down, seed = chain.forward(home)[0][:3, :3], home
        q, qd = np.zeros((len(data), 6)), np.zeros((len(data), 6))
        for i in range(len(data)):
            q[i] = seed = chain.solve(position[i] / 1000, down, seed)
            velocity = np.r_[direction[i] * speed[i] / 1000, 0., 0., 0.]
            qd[i] = np.linalg.lstsq(chain.forward(q[i])[1], velocity, rcond=None)[0]
        samples = ForceSamples.__new__(ForceSamples)
        samples.t, samples.q, samples.qd = np.cumsum(step), q, qd
        samples.qdd, samples.acceleration = np.zeros_like(qd), np.zeros((len(data), 3))
        samples.pose, samples.wrench = np.c_[position, np.zeros((len(data), 3))], np.c_[force, np.zeros((len(data), 3))]
        samples.station = np.array([f'F{k}' for k in np.minimum(np.arange(len(data)) * folds // len(data), folds - 1)])
        samples.labels = np.where(moving, 'move', 'still')
        self.samples, self.free = samples, ~moving


def main():
    from ..configs import PACKAGE, SESSION, load, output_dir
    from ..part_scan.kinematics import RobotChain
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--csv', default=str(SESSION / 'motion_offset_samples.csv'))
    options = parser.parse_args()
    config = load('robot')
    settings = dict(config['force_model'], moment_weight_m=0.)  # forces only
    urdf = PACKAGE.parent / 'cobot_rg2' / 'doosan-robot2' / 'dsr_description2' / 'urdf' / 'm0609.urdf'
    chain = RobotChain(urdf.read_text(), load('real_scene')['probe']['tcp_mm_deg'], config['real_scan']['ik'])
    recording = OffsetCsv(options.csv, chain, np.deg2rad(config['robot']['home_joints_deg']), config)
    selection = Selection(recording.samples, chain, settings)
    _, _, best = selection.run()
    still = recording.free[selection.keep]
    threshold = config['real_scan']['mesh_height']['contact_threshold_n']
    stages = report(selection.samples, best, settings['kalman'], still, threshold)
    path = axes_figure(output_dir('images') / f'force_model_view_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png',
                       selection.samples.t, stages, still, settings['kalman']['noise_n'], threshold,
                       f'motion_offset recording in the air: physics model {label(best, ", ")} (held out) + Kalman')
    print(f'-> {path}')


if __name__ == '__main__':
    main()
