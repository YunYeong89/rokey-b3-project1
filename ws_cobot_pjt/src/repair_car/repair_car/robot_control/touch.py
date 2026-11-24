"""Vertical probe touches (TOOL Z down, BASE mm, z of the probe TCP = sphere bottom), shared by calibration,
choose_car and scan_surface: travel above the point, guarded descent under compliance, a static check that
rejects motion-induced force spikes, then press and settle."""
import time

import numpy as np

if __package__:
    from .rotations import rotation_zyz
else:
    from robot_control.rotations import rotation_zyz


class VerticalTouch:
    confirm_wait, confirm_samples, retries = 0.15, 5, 3

    def __init__(self, program):
        """Takes the current orientation (home: TOOL Z down) for every touch."""
        self.program, self.probe = program, program.probe
        self.motion, self.speed = program.config['motion'], program.config['scan']['search_speed']
        pose, _ = self.probe.read()
        self.vertical = rotation_zyz(pose[3:])
        self.safe_z = program.safe_z()
        self.false_touches = 0

    def travel(self, x, y, z, height=None):
        """Up to the travel height (at least z), across at the linear speed, down to z."""
        pose, _ = self.probe.read()
        motion, height = self.motion, max(z, height or z)
        if height > pose[2] + 0.5:
            self.probe.move([*pose[:2], height], self.vertical, motion['approach_speed'], acc=motion['approach_acc'])
        self.probe.move([x, y, max(height, pose[2])], self.vertical, motion['linear_speed'], acc=motion['linear_acc'])
        self.probe.move([x, y, z], self.vertical, motion['approach_speed'], acc=motion['approach_acc'])

    def confirmed(self):
        """Contact persists once the robot is still; an inertial spike does not."""
        time.sleep(self.confirm_wait)
        forces = [self.probe.normal_force(*self.probe.read()) for _ in range(self.confirm_samples)]
        return float(np.mean(forces)) > self.probe.touch_threshold

    def touch(self, x, y, top, bottom, height=None, stay=False):
        """Contact under (x, y) searching from `top` down to `bottom`: {'x', 'y', 'z', 'fn', 'settled'}, or
        None without contact. The probe is left at `top` above the point (compliance stays on); with
        `stay`, a settled contact keeps pressing there instead (to slide on with `slide`)."""
        if not self.probe.compliant:
            self.probe.enable()
        self.travel(x, y, top, height)
        probe, result = self.probe, None
        for _ in range(self.retries):
            probe.tare()
            if not probe.move([x, y, bottom], self.vertical, self.speed, until=probe.touched):
                break
            if self.confirmed():
                probe.press()
                result = self.sample(probe.settle())
                if stay and result['settled']:
                    return result
                probe.release()
                break
            self.false_touches += 1
            self.program.log(f'  false touch at z={probe.read()[0][2]:.2f} mm rejected (spike while moving)')
        self.lift(top)
        return result

    def slide(self, x, y, z, speed):
        """While pressing: move along the surface to (x, y) (z is held by the force control, `z` is only the
        expected height) and settle; None once contact is lost (force released)."""
        self.probe.move([x, y, z], self.vertical, speed, acc=self.program.config['scan']['acc'])
        self.probe.press()
        result = self.sample(self.probe.settle())
        if not result['settled']:
            self.probe.release()
            return None
        return result

    def sample(self, settled):
        pose, wrench = self.probe.read()
        if settled is not None:
            pose, fn = settled['pose'], settled['fn']
        else:
            fn = self.probe.normal_force(pose, wrench)
        return {'x': float(pose[0]), 'y': float(pose[1]), 'z': float(pose[2]), 'fn': float(fn),
                'fz': float(wrench[2]), 'settled': settled is not None}

    def lift(self, top):
        self.probe.release()
        pose, _ = self.probe.read()
        self.probe.move([pose[0], pose[1], max(top, pose[2])], self.vertical, self.motion['approach_speed'],
                        acc=self.motion['approach_acc'])
