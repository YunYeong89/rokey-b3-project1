"""Force-controlled contact test (overshoot), after `pick_probe`:
  1. movel     from home straight down to start_above_mm above the contact block (robot.yaml `contact_test`;
               the --car scene places it below the home TCP)
  2. approach  BASE z force control with the smaller approach_force_n (e.g. -1.0 N = down) carries the probe
               onto the block
  3. contact   once contact is detected the desired force ramps to force_n (e.g. -1.5 N) over ramp_s (the
               set_desired_force transition time, 0..1 s) instead of stepping, and is held for hold_s; then
               released, back home
Pressing at force_n from the start overshoots: the probe reaches the block still moving, and the force loop
(force filter lag against a stiff contact) carries the push past the target. Approaching with less force
and ramping the rest keeps the force rise slower than that loop.
The controller's BASE force Fx, Fy, Fz (as read, no tare) is recorded from the start of the movel until
hold_s after contact and plotted against the height above the surface (+z) and over time, with the commanded
force and the overshoot of Fz -> repair_car/images/contact_overshoot_<time>.png and .csv
  ros2 run repair_car contact_overshoot [--approach-force -1.0] [--ramp 0.5] [--force -1.5] [--hold 5] [--start-above 30]
  ros2 run repair_car contact_overshoot --approach-force -1.5 --ramp 0      # previous test: full force, no ramp
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
    from repair_car.robot_control.rotations import rotation_zyz
else:
    from calibration.force_data import IMAGE_DIR
    from robot_control.program import RobotProgram
    from robot_control.rotations import rotation_zyz


class ContactRecord:
    """Rows of time, BASE TCP position (mm) and BASE force (N) with the phase they were taken in."""
    columns = ['t_s', 'x_mm', 'y_mm', 'z_mm', 'Fx_N', 'Fy_N', 'Fz_N', 'phase']

    def __init__(self):
        self.rows, self.phase, self.start = [], 'movel', time.monotonic()

    def add(self, pose, wrench):
        self.rows.append([time.monotonic() - self.start, *pose[:3], *wrench[:3], self.phase])
        return False  # also the `until` callback of ContactProbe.move: never stops the motion

    def arrays(self):
        values = np.array([row[:7] for row in self.rows], dtype=float)
        return values[:, 0], values[:, 1:4], values[:, 4:7], np.array([row[7] for row in self.rows])

    def save(self, path):
        with path.open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.columns)
            writer.writerows([[round(value, 5) for value in row[:7]] + [row[7]] for row in self.rows])


class ContactOvershootTest:
    hits_required = 3

    def __init__(self, program, force, approach, ramp, hold, start_above):
        self.program, self.probe, self.robot = program, program.probe, program.robot
        self.settings = program.config['contact_test']
        self.force, self.approach, self.ramp = force, approach, ramp
        self.hold, self.start_above = hold, start_above
        self.surface = self.settings['surface_z_mm']
        self.threshold = program.config['scan']['touch_threshold']
        if not 0.0 <= ramp <= 1.0:
            raise ValueError(f'ramp {ramp} s: the set_desired_force transition time is 0..1 s')
        if np.sign(approach) != np.sign(force) or abs(approach) <= self.threshold:
            raise ValueError(f'Approach force {approach:+.2f} N must point the same way as the final force {force:+.2f} N and exceed the contact '
                             f'threshold {self.threshold:.2f} N')
        self.record = ContactRecord()
        self.offset = self.contact_time = self.ramp_time = None

    def reaction(self, wrench):
        """Contact force along the push direction above the static reading before force control."""
        return -np.sign(self.force) * (wrench[2] - self.offset)

    def run(self):
        program, probe, robot, motion = self.program, self.probe, self.robot, self.program.config['motion']
        program.home()
        pose, _ = probe.read()
        rotation, start = rotation_zyz(pose[3:]), [pose[0], pose[1], self.surface + self.start_above]
        program.log(f'1/3 movel {pose[2] - start[2]:.1f} mm down to {self.start_above:.0f} mm above the surface '
                    f'(z={self.surface:.1f} mm)')
        self.record.add(*probe.read())
        probe.move(start, rotation, motion['linear_speed'], until=self.record.add, acc=motion['linear_acc'])
        still = time.monotonic() + 0.5  # static reading at the start height: reference for contact detection
        while time.monotonic() < still:
            self.record.add(*probe.read())
        self.offset = float(np.mean([row[6] for row in self.record.rows[-10:]]))
        program.log(f'2/3 BASE z force control approach {self.approach:+.2f} N (static Fz {self.offset:+.3f} N, '
                    f'contact at {self.threshold:.2f} N above it)')
        self.record.phase = 'force'
        probe.enable(robot.DR_BASE)
        self.command(self.approach)
        probe.forcing = True
        hits, first, deadline = 0, None, time.monotonic() + self.settings['timeout_s']
        while self.contact_time is None or time.monotonic() - self.record.start - self.contact_time < self.hold:
            pose, wrench = probe.read()
            self.record.add(pose, wrench)
            if self.contact_time is not None:
                continue
            hits = hits + 1 if self.reaction(wrench) > self.threshold else 0
            first = self.record.rows[-1][0] if hits == 1 else first
            if hits >= self.hits_required:
                self.contact_time, self.record.phase = first, 'contact'
                self.ramp_time = time.monotonic() - self.record.start
                self.command(self.force, self.ramp)
                program.log(f'3/3 contact at z={pose[2]:.2f} mm: {self.approach:+.2f} -> {self.force:+.2f} N over '
                            f'{self.ramp:.2f} s, holding {self.hold:.0f} s')
            elif time.monotonic() > deadline:
                raise RuntimeError(f'No contact within {self.settings["timeout_s"]:.0f} s: check contact_test.surface_z_mm')
        probe.close()  # release force and compliance
        probe.move(start, rotation, motion['approach_speed'], acc=motion['approach_acc'])
        program.home()

    def command(self, force, ramp=0.0):
        """BASE z desired force, reached over `ramp` s from the current one (controller transition)."""
        self.robot.set_desired_force([0.0, 0.0, force, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0], time=ramp,
                                     mod=self.robot.DR_FC_MOD_ABS)

    def commanded(self, since):
        """Commanded BASE z force at times `since` from contact detection (approach force before the ramp)."""
        start = self.ramp_time - self.contact_time
        fraction = np.clip((since - start) / self.ramp, 0, 1) if self.ramp else (since >= start).astype(float)
        return self.approach + (self.force - self.approach) * fraction


class OvershootPlot:
    """Fx, Fy, Fz against the height above the surface (full and near contact) and over time."""
    colors = {'Fx': 'tab:red', 'Fy': 'tab:green', 'Fz': 'tab:blue'}
    styles = {'movel': dict(ls=':', lw=1.0), 'force': dict(ls='-', lw=1.2), 'contact': dict(ls='-', lw=1.8)}

    def __init__(self, test):
        self.test = test
        self.t, self.position, self.force, self.phase = test.record.arrays()
        self.height = self.position[:, 2] - test.surface
        self.since = self.t - test.contact_time
        # Contact force along the push, absolute: DR_FC_MOD_ABS regulates the reading itself to |force|
        # (the static reading before force control only serves contact detection).
        self.push = -np.sign(test.force) * self.force[:, 2]
        self.metrics = self.measure()

    def measure(self):
        test, after, fz = self.test, self.since >= 0, self.push
        target = abs(test.force)
        before = (self.since < 0) & (self.since > -0.5) & (self.phase == 'force')
        speed = np.polyfit(self.t[before], self.position[before, 2], 1)[0] if before.sum() > 2 else np.nan
        outside = after & (np.abs(fz - target) > 0.1 * target)
        touch = np.nonzero(self.height <= 0)[0]
        return {'target_N': target, 'peak_N': float(fz[after].max()),
                'overshoot_pct': float((fz[after].max() - target) / target * 100),
                'settling_s': float(self.since[outside].max()) if outside.any() else 0.0,
                'steady_N': float(fz[after & (self.since > self.since.max() - 2)].mean()),
                'approach_mm_s': float(-speed), 'penetration_mm': float(-self.height[after].min()),
                'surface_reached_s': float(self.since[touch[0]]) if len(touch) else None}

    def lines(self, axis, x, phases=('movel', 'force', 'contact')):
        """Dotted: movel, thin: force-controlled approach, thick: in contact."""
        for index, (name, color) in enumerate(self.colors.items()):
            for phase in phases:
                selected = self.phase == phase
                if selected.any():
                    axis.plot(x[selected], self.force[selected, index], color=color, **self.styles[phase],
                              label=name if phase == phases[-1] else None)

    def save(self, path):
        test, m = self.test, self.metrics
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        reaction = -test.force  # BASE Fz the controller regulates to while pressing
        ax = axes[0, 0]
        self.lines(ax, self.height)
        ax.set(title='BASE force vs height above the surface, from home\n(dotted: movel, thin: force approach, '
               'thick: contact)', xlabel='height above surface, +z [mm]', ylabel='F [N]')
        near = axes[0, 1]
        self.lines(near, self.height, ('force', 'contact'))
        near.set_xlim(min(-0.5, -m['penetration_mm'] - 0.2), 5.0)
        near.set(title='Near the surface (force-controlled approach and contact)',
                 xlabel='height above surface, +z [mm]', ylabel='F [N]')
        for axis in (ax, near):
            axis.axvline(0, color='0.3', lw=1, ls='--', label='surface (block top)')
            axis.axhline(reaction, color='tab:blue', lw=0.8, ls='-.', alpha=0.6, label=f'Fz target {reaction:+.2f} N')
        timed = axes[1, 0]
        self.lines(timed, self.since)
        after = self.since >= 0
        timed.plot(self.since[after], -test.commanded(self.since[after]), color='k', ls='--', lw=1,
                   label='commanded (as reaction)')
        if test.ramp:
            ramp = test.ramp_time - test.contact_time
            timed.axvspan(ramp, ramp + test.ramp, color='tab:orange', alpha=0.15, label=f'force ramp {test.ramp:.2f} s')
        timed.axhspan(reaction - 0.1 * m['target_N'], reaction + 0.1 * m['target_N'], color='tab:blue', alpha=0.08,
                      label='target +-10 %')
        timed.axhline(reaction, color='tab:blue', lw=0.8, ls='-.', alpha=0.6)
        peak = int(np.argmax(np.where(self.since >= 0, self.push, -np.inf)))
        timed.annotate(f'peak {self.force[peak, 2]:+.2f} N ({m["overshoot_pct"]:+.0f} %)', (self.since[peak], self.force[peak, 2]),
                       xytext=(15, 10), textcoords='offset points', arrowprops=dict(arrowstyle='->'))
        timed.set(title='BASE force over time', xlabel='time from contact [s]', ylabel='F [N]')
        zoom, near_contact = timed.inset_axes([0.5, 0.08, 0.47, 0.42]), (self.since > -0.5) & (self.since < 2.0)
        zoom.plot(self.since[near_contact], self.force[near_contact, 2], color=self.colors['Fz'], lw=1.5)
        zoom.plot(self.since[after & near_contact], -test.commanded(self.since[after & near_contact]),
                  color='k', ls='--', lw=1)
        zoom.axhspan(reaction - 0.1 * m['target_N'], reaction + 0.1 * m['target_N'], color='tab:blue', alpha=0.08)
        zoom.set_title('Fz around contact (-0.5..2 s)', fontsize=8)
        zoom.tick_params(labelsize=7)
        zoom.grid(alpha=0.3)
        heights = axes[1, 1]
        for phase in self.styles:
            selected = self.phase == phase
            heights.plot(self.since[selected], self.height[selected], color='0.2', **self.styles[phase], label=phase)
        heights.axhline(0, color='0.3', lw=1, ls='--')
        heights.set(title='Height above the surface over time', xlabel='time from contact [s]', ylabel='height [mm]')
        reached = m['surface_reached_s']
        heights.text(0.02, 0.97, '\n'.join([
            f'BASE z force: approach {test.approach:+.2f} N, ramp {test.ramp:.2f} s to {test.force:+.2f} N, '
            f'contact threshold {test.threshold:.2f} N',
            f'approach speed before contact {m["approach_mm_s"]:.1f} mm/s',
            f'peak {m["peak_N"]:.2f} N vs target {m["target_N"]:.2f} N -> overshoot {m["overshoot_pct"]:+.0f} %',
            f'settling (within +-10 %) {m["settling_s"]:.2f} s, steady {m["steady_N"]:.2f} N (last 2 s)',
            f'penetration below the surface {m["penetration_mm"]:.2f} mm'
            + (f', surface reached {reached:+.2f} s' if reached is not None else '')]),
            transform=heights.transAxes, va='top', fontsize=9, bbox=dict(facecolor='white', alpha=0.85, edgecolor='0.7'))
        for axis in (timed, heights):
            axis.axvline(0, color='k', lw=0.8, ls='--', label='contact detected')
        for axis in axes.flat:
            axis.grid(alpha=0.3)
            axis.legend(fontsize=7, loc='upper left' if axis is timed else 'best')
        fig.suptitle(f'Contact overshoot ({test.program.mode}, tcp={test.program.config["probe"]["tcp_name"]}, '
                     f'{len(self.t)} samples)')
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--force', type=float, help='final BASE z desired force, N (default: robot.yaml contact_test)')
    parser.add_argument('--approach-force', type=float, help='BASE z desired force until contact, N (default: robot.yaml)')
    parser.add_argument('--ramp', type=float, help='ramp to the final force after contact, 0..1 s (default: robot.yaml)')
    parser.add_argument('--hold', type=float, help='recording after contact, s (default: robot.yaml)')
    parser.add_argument('--start-above', type=float, help='movel target above the surface, mm (default: robot.yaml)')
    options = parser.parse_args(program_args())
    program = RobotProgram('contact_overshoot')
    try:
        settings, probe = program.config['contact_test'], program.config['probe']
        program.robot.verify_profiles(probe['tool_name'], probe['tcp_name'])
        value = lambda option, key: settings[key] if option is None else option
        test = ContactOvershootTest(program, value(options.force, 'force_n'),
                                    value(options.approach_force, 'approach_force_n'), value(options.ramp, 'ramp_s'),
                                    value(options.hold, 'hold_s'), value(options.start_above, 'start_above_mm'))
        try:
            test.run()
        finally:
            if test.contact_time is not None:
                IMAGE_DIR.mkdir(exist_ok=True)
                stem = IMAGE_DIR / f'contact_overshoot_{datetime.now():%Y%m%d_%H%M%S}'
                test.record.save(stem.with_suffix('.csv'))
                plot = OvershootPlot(test)
                plot.save(stem.with_suffix('.png'))
                program.log(f'{stem}.png / .csv: ' + ', '.join(
                    f'{key} {value:.2f}' for key, value in plot.metrics.items() if value is not None))
    finally:
        program.close()


if __name__ == '__main__':
    main()
