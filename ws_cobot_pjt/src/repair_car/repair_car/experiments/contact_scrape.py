"""Scraping test, after `pick_probe`: strokes along BASE y under force control over the contact-test objects
(robot.yaml `contact_test`; the --car scene has them), recording the controller's BASE force Fx, Fy, Fz:
  block               vertical tool, force along BASE -z, straight stroke across the block top
  hemisphere_z        vertical tool, force along BASE -z, stroke across the dome (z follows it by force control)
  hemisphere_normal   tool on the dome normal, force along the tool axis; the dome is crossed in normal_step_deg
                      steps, re-aiming the tool and re-latching the force axis every step
Each run (robot.yaml `scrape_test`): approach with approach_force_n, ramp to force_n on contact, hold, stroke at
speed_mm_s, hold, release. The force is plotted against the stroke position in BASE (Fx, Fy, Fz) and on the
surface (normal Fn, tangential Ft along the stroke = friction, binormal Fb), with the contact gap
-> repair_car/images/contact_scrape_<run>_dsr_<time>.png/.csv, plus contact_scrape_hemisphere_dsr_<time>.png comparing
the two hemisphere runs. The DSR force services only: task compliance + set_desired_force, movel strokes.
  ros2 run repair_car contact_scrape [--runs block hemisphere_z hemisphere_normal]
"""
import argparse

if __package__:
    from repair_car.robot_control.robot_bridge import program_args
else:
    from robot_control.robot_bridge import program_args
import csv
from datetime import datetime
import time

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

if __package__:
    from repair_car.calibration.force_data import IMAGE_DIR
    from repair_car.robot_control.program import RobotProgram
    from repair_car.robot_control.rotations import axis_angle, rotation_zyz
else:
    from calibration.force_data import IMAGE_DIR
    from robot_control.program import RobotProgram
    from robot_control.rotations import axis_angle, rotation_zyz

RUNS = ['block', 'hemisphere_z', 'hemisphere_normal']
STROKE = np.array([0.0, 1.0, 0.0])  # BASE y


class ForceRecord:
    """Rows of time, BASE TCP pose (mm, deg) and BASE force (N) with the phase they were taken in."""
    columns = ['t_s', 'x_mm', 'y_mm', 'z_mm', 'a_deg', 'b_deg', 'c_deg', 'Fx_N', 'Fy_N', 'Fz_N', 'phase']

    def __init__(self):
        self.rows, self.phase, self.start = [], 'approach', time.monotonic()

    def add(self, pose, wrench):
        self.rows.append([time.monotonic() - self.start, *pose[:6], *wrench[:3], self.phase])
        return False  # also the `until` callback of ContactProbe.move: never stops the motion

    def arrays(self):
        values = np.array([row[:10] for row in self.rows], dtype=float)
        return values[:, 0], values[:, 1:7], values[:, 7:10], np.array([row[10] for row in self.rows])

    def save(self, path):
        with path.open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.columns)
            writer.writerows([[round(value, 5) for value in row[:10]] + [row[10]] for row in self.rows])


class Stroke:
    """One run: TCP waypoints (BASE mm) with tool rotations, the force frame and the surface (plane z or sphere)."""

    def __init__(self, name, frame, waypoints, continuous, plane=None, sphere=None):
        self.name, self.frame, self.waypoints, self.continuous = name, frame, waypoints, continuous
        self.plane, self.sphere = plane, sphere

    def normals(self, centres):
        """Outward surface normals at tip-sphere centres (sphere-sphere / sphere-plane contact)."""
        if self.plane is not None:
            return np.tile([0.0, 0.0, 1.0], (len(centres), 1))
        d = centres - self.sphere[0]
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def outward(self):
        """Approach direction (reversed) at the first waypoint: BASE z, or the normal the tool is aimed along."""
        rotation = self.waypoints[0][1]
        return np.array([0.0, 0.0, 1.0]) if self.continuous else -rotation[:, 2]


class StrokePlanner:
    def __init__(self, config, home, frames):
        self.test, self.settings, self.frames = config['contact_test'], config['scrape_test'], frames
        self.vertical, self.centre = rotation_zyz(home[3:]), np.asarray(home[:2])  # the block is below home
        self.tip = config['probe']['tip_radius_mm']
        table = config['vehicle_zone']['table_z_mm']
        self.dome = (np.r_[self.test['hemisphere_center_mm'], table], self.test['hemisphere_radius_mm'])

    def block(self):
        half, top = self.settings['block_stroke_mm'] / 2, self.test['surface_z_mm']
        ends = [np.r_[self.centre, top] + side * half * STROKE for side in (-1, 1)]
        return Stroke('block', self.frames['base'], [(p, self.vertical) for p in ends], True, plane=top)

    def normal(self, degrees):
        a = np.deg2rad(degrees)
        return np.array([0.0, np.sin(a), np.cos(a)])

    def hemisphere(self, mode):
        (c, radius), angle = self.dome, self.settings['hemisphere_angle_deg']
        if mode == 'z':  # vertical tool: its sphere touches the dome where the normal is at +-angle
            ends = [c + (radius + self.tip) * self.normal(side * angle) - [0, 0, self.tip] for side in (-1, 1)]
            return Stroke('hemisphere_z', self.frames['base'], [(p, self.vertical) for p in ends], True,
                          sphere=self.dome)
        step = self.settings['normal_step_deg']
        angles = np.arange(-angle, angle + step / 2, step)
        points = [(c + radius * self.normal(a), axis_angle(np.array([1.0, 0, 0]), -np.deg2rad(a)) @ self.vertical)
                  for a in angles]
        return Stroke('hemisphere_normal', self.frames['tool'], points, False, sphere=self.dome)

class ForcePress:
    """Force control along BASE z (pushing down) or the tool axis: approach force until contact (debounced
    reaction above the threshold, relative to the static reading), then ramped to the full force."""
    hits_required = 3

    def __init__(self, program, record, settings):
        self.program, self.probe, self.robot, self.record = program, program.probe, program.robot, record
        self.approach, self.force, self.ramp = settings['approach_force_n'], settings['force_n'], settings['ramp_s']
        self.threshold = program.config['scan']['touch_threshold']
        if not 0.0 <= self.ramp <= 1.0 or not self.threshold < self.approach <= self.force:
            raise ValueError(f'scrape_test: requires contact threshold {self.threshold:.2f} N < approach force <= final force and a ramp of 0..1 s')
        self.frame = None

    def outward(self, pose):
        return np.array([0.0, 0.0, 1.0]) if self.frame == self.robot.DR_BASE else -rotation_zyz(pose[3:])[:, 2]

    def command(self, magnitude, ramp=0.0):
        """Desired force (N) into the surface: -z in BASE, +z in TOOL (tool z points into the surface)."""
        fz = -magnitude if self.frame == self.robot.DR_BASE else magnitude
        self.robot.set_desired_force([0.0, 0.0, fz, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=ramp,
                                     mod=self.robot.DR_FC_MOD_ABS)

    def watch(self, pose, wrench):
        """Record and guard a sample; also the `until` callback of the stroke moves (never stops them)."""
        self.record.add(pose, wrench)
        return False

    def sample(self):
        pose, wrench = self.probe.read()
        self.watch(pose, wrench)
        return pose, float(wrench[:3] @ self.outward(pose))

    def down(self, frame, timeout):
        """Static reading (reference for contact detection), approach force, contact, ramp to the full force."""
        self.frame, still, static = frame, time.monotonic() + 0.5, []
        while time.monotonic() < still:
            static.append(self.sample()[1])
        offset = float(np.mean(static))
        self.probe.enable(frame)
        self.command(self.approach)
        self.probe.forcing = True
        hits, deadline = 0, time.monotonic() + timeout
        while hits < self.hits_required:
            pose, reaction = self.sample()
            hits = hits + 1 if reaction - offset > self.threshold else 0
            if time.monotonic() > deadline:
                raise RuntimeError(f'No contact within {timeout:.0f} s: check the contact_test object positions')
        self.record.phase = 'press'
        self.command(self.force, self.ramp)
        self.program.log(f'  contact at {np.round(pose[:3], 1).tolist()}: {self.approach:.2f} -> {self.force:.2f} N '
                         f'over {self.ramp:.2f} s')

    def hold(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.sample()


class Scraper:
    def __init__(self, program, vertical):
        self.program, self.probe, self.vertical = program, program.probe, vertical
        self.settings, self.motion = program.config['scrape_test'], program.config['motion']

    def run(self, stroke, record):
        program, probe, motion, settings = self.program, self.probe, self.motion, self.settings
        press, gap = ForcePress(program, record, settings), settings['approach_gap_mm']
        start, rotation = stroke.waypoints[0]
        pre = start + gap * stroke.outward()
        hover = pre + [0.0, 0.0, 40.0]
        program.home()
        pose, _ = probe.read()
        program.log(f'{stroke.name}: to the start {np.round(start, 1).tolist()}, '
                    f'{"BASE z" if stroke.continuous else "tool axis"} force, {len(stroke.waypoints)} waypoints')
        probe.move([*hover[:2], max(pose[2], hover[2])], rotation, motion['linear_speed'], acc=motion['linear_acc'])
        probe.move(hover, rotation, motion['approach_speed'], acc=motion['approach_acc'])
        probe.move(pre, rotation, motion['approach_speed'], acc=motion['approach_acc'])
        press.down(stroke.frame, settings['timeout_s'])
        press.hold(settings['hold_s'])
        record.phase = 'stroke'
        speed, acc = settings['speed_mm_s'], program.config['scan']['acc']
        if stroke.continuous:  # z is force-controlled: the end keeps the start height
            end, rotation = stroke.waypoints[-1]
            probe.move([end[0], end[1], start[2]], rotation, speed, until=press.watch, acc=acc)
        else:
            for point, rotation in stroke.waypoints[1:]:
                probe.move(point, rotation, speed, until=press.watch, acc=acc)
                press.command(press.force)  # re-latch the force axis on the new tool axis
        record.phase = 'hold'
        press.hold(settings['hold_s'])
        record.phase = 'release'
        probe.release()
        pose, _ = probe.read()
        probe.move(pose[:3] + gap * press.outward(pose), rotation_zyz(pose[3:]), motion['approach_speed'],
                   until=record.add, acc=motion['approach_acc'])
        probe.close()
        pose, _ = probe.read()
        lifted = pose[:3] + [0.0, 0.0, 40.0]
        probe.move(lifted, rotation_zyz(pose[3:]), motion['approach_speed'], acc=motion['approach_acc'])
        probe.move(lifted, self.vertical, motion['approach_speed'], acc=motion['approach_acc'])  # tool upright again
        program.home()


class ScrapeAnalysis:
    """Forces on the surface along the stroke: Fn (normal, pressing > 0), Ft (along the stroke, friction < 0
    when moving +y), Fb (binormal), at the stroke coordinate (block: y from its centre, dome: normal angle)."""

    def __init__(self, record, stroke, tip):
        self.stroke = stroke
        self.t, pose, self.force, self.phase = record.arrays()
        z_tool = np.array([rotation_zyz(p[3:])[:, 2] for p in pose])
        self.tilt = np.degrees(np.arccos(np.clip(-z_tool[:, 2], -1, 1)))
        centre = pose[:, :3] - tip * z_tool
        n = stroke.normals(centre)
        tangent = STROKE - (n @ STROKE)[:, None] * n
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        self.fn, self.ft = np.einsum('ij,ij->i', self.force, n), np.einsum('ij,ij->i', self.force, tangent)
        self.fb = np.einsum('ij,ij->i', self.force, np.cross(n, tangent))
        if stroke.plane is not None:
            mid = np.mean([p[1] for p, _ in stroke.waypoints])
            self.x, self.label = centre[:, 1] - mid, 'stroke position: y from the block centre [mm]'
            self.gap = centre[:, 2] - (stroke.plane + tip)
        else:
            c, radius = stroke.sphere
            self.x, self.label = np.degrees(np.arctan2(n[:, 1], n[:, 2])), 'surface normal angle on the dome [deg]'
            self.gap = np.linalg.norm(centre - c, axis=1) - (radius + tip)
        stroking = self.phase == 'stroke'
        self.metrics = {'Fn_mean_N': float(self.fn[stroking].mean()), 'Fn_std_N': float(self.fn[stroking].std()),
                        'Ft_mean_N': float(self.ft[stroking].mean()),
                        'mu': float(np.median(-self.ft[stroking] / np.maximum(self.fn[stroking], 1e-3))),
                        'F_max_N': float(np.linalg.norm(self.force[stroking], axis=1).max()),
                        'gap_max_mm': float(self.gap[stroking].max()), 'gap_min_mm': float(self.gap[stroking].min())}

    def save(self, path, title, force):
        m, on = self.metrics, np.isin(self.phase, ['press', 'stroke', 'hold'])
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        ax = axes[0, 0]
        for index, (name, color) in enumerate(zip(['Fx', 'Fy', 'Fz'], ['tab:red', 'tab:green', 'tab:blue'])):
            ax.plot(self.x[on], self.force[on, index], color=color, lw=1.2, label=name)
        if self.stroke.continuous:
            ax.axhline(force, color='tab:blue', ls='-.', lw=0.8, label=f'commanded {force:.2f} N along BASE -z')
        ax.set(title='BASE force along the stroke', xlabel=self.label, ylabel='F [N]')
        ax = axes[0, 1]
        for values, name, color in [(self.fn, 'Fn normal', 'k'), (self.ft, 'Ft tangential (stroke +y)', 'tab:orange'),
                                    (self.fb, 'Fb binormal', 'tab:purple')]:
            ax.plot(self.x[on], values[on], color=color, lw=1.2, label=name)
        ax.axhline(force, color='k', ls='-.', lw=0.8, label=f'{force:.2f} N')
        ax.set(title=f'Force on the surface (Fn {m["Fn_mean_N"]:.2f} +- {m["Fn_std_N"]:.2f} N, '
                     f'friction mu ~ {m["mu"]:.2f})', xlabel=self.label, ylabel='F [N]')
        ax = axes[1, 0]
        ax.plot(self.x[on], self.gap[on], color='0.2', lw=1.2, label='tip sphere off the surface (+) / into it (-)')
        ax.axhline(0, color='0.5', ls='--', lw=0.8)
        ax.set(title='Contact gap and tool tilt', xlabel=self.label, ylabel='gap [mm]')
        twin = ax.twinx()
        twin.plot(self.x[on], self.tilt[on], color='tab:cyan', lw=1, label='tool tilt from vertical')
        twin.set_ylabel('tilt [deg]')
        twin.legend(fontsize=7, loc='upper right')
        ax = axes[1, 1]
        for index, (name, color) in enumerate(zip(['Fx', 'Fy', 'Fz'], ['tab:red', 'tab:green', 'tab:blue'])):
            ax.plot(self.t, self.force[:, index], color=color, lw=1, label=name)
        for phase, color in [('approach', '0.85'), ('press', 'tab:orange'), ('stroke', 'tab:green'),
                             ('hold', 'tab:orange'), ('release', '0.85')]:
            selected = self.t[self.phase == phase]
            if len(selected):
                ax.axvspan(selected[0], selected[-1], color=color, alpha=0.12, label=phase)
        ax.set(title='BASE force over time', xlabel='time [s]', ylabel='F [N]')
        for axis in axes.flat:
            axis.grid(alpha=0.3)
            axis.legend(fontsize=7, loc='upper left')
        fig.suptitle(title)
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)


def compare(analyses, path, force):
    """The two hemisphere runs over the dome: normal force, friction and gap."""
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    for analysis, style in zip(analyses, ['-', '--']):
        on = analysis.phase == 'stroke'
        name = analysis.stroke.name.replace('hemisphere_', '')
        axes[0].plot(analysis.x[on], analysis.fn[on], ls=style, lw=1.3, label=f'{name}: Fn')
        axes[1].plot(analysis.x[on], analysis.ft[on], ls=style, lw=1.3, label=f'{name}: Ft')
        axes[2].plot(analysis.x[on], analysis.gap[on], ls=style, lw=1.3, label=f'{name}: gap')
    axes[0].axhline(force, color='k', ls='-.', lw=0.8, label=f'{force:.2f} N')
    for axis, title, unit in zip(axes, ['Normal force', 'Tangential force (friction)', 'Contact gap'], ['N', 'N', 'mm']):
        axis.set(title=title, xlabel='surface normal angle on the dome [deg]', ylabel=unit)
        axis.grid(alpha=0.3)
        axis.legend(fontsize=8)
    fig.suptitle('Hemisphere strokes: force along BASE -z (z) vs along the surface normal (normal)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--runs', nargs='+', choices=RUNS, default=RUNS, help='default: all three')
    parser.add_argument('--speed', type=float, help='Stroke speed [mm/s]')
    options = parser.parse_args(program_args())
    program = RobotProgram('contact_scrape')
    try:
        probe = program.config['probe']
        program.robot.verify_profiles(probe['tool_name'], probe['tcp_name'])
        if options.speed is not None:
            program.config['scrape_test']['speed_mm_s'] = options.speed
        program.home()
        home, _ = program.probe.read()
        planner = StrokePlanner(program.config, home, {'base': program.robot.DR_BASE, 'tool': program.robot.DR_TOOL})
        strokes = {'block': planner.block, 'hemisphere_z': lambda: planner.hemisphere('z'),
                   'hemisphere_normal': lambda: planner.hemisphere('normal')}
        scraper, stamp, analyses = Scraper(program, planner.vertical), f'{datetime.now():%Y%m%d_%H%M%S}', {}
        force, tip = program.config['scrape_test']['force_n'], probe['tip_radius_mm']
        IMAGE_DIR.mkdir(exist_ok=True)
        for name in options.runs:
            stroke, record = strokes[name](), ForceRecord()
            try:
                scraper.run(stroke, record)
            finally:
                stem = IMAGE_DIR / f'contact_scrape_{name}_dsr_{stamp}'
                record.save(stem.with_suffix('.csv'))
                if 'stroke' in [row[-1] for row in record.rows]:
                    analyses[name] = ScrapeAnalysis(record, stroke, tip)
                    analyses[name].save(stem.with_suffix('.png'), f'Scrape {name} (dsr, {program.mode}, {force:.2f} N, '
                                        f'{program.config["scrape_test"]["speed_mm_s"]:.1f} mm/s along BASE y)', force)
                    program.log(f'{stem}.png / .csv: ' + ', '.join(
                        f'{key} {value:.2f}' for key, value in analyses[name].metrics.items()))
        if {'hemisphere_z', 'hemisphere_normal'} <= set(analyses):
            path = IMAGE_DIR / f'contact_scrape_hemisphere_dsr_{stamp}.png'
            compare([analyses['hemisphere_z'], analyses['hemisphere_normal']], path, force)
            program.log(f'{path}')
    finally:
        program.close()


if __name__ == '__main__':
    main()
