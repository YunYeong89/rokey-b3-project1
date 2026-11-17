"""Execution of the planned paths with the DSR force services, as contact_scrape scrapes the block: per path (a
whole pass, the probe in contact throughout), from HOME or the last path's exit with the tool down across at the
travel height, down to the standoff point off its start, turned onto the fixed axis there, in to the approach gap;
task compliance (BASE) + desired force along -axis, contact, ramp to the force, then movel strokes through the row
ends at the contact level while the controller keeps the force. The program only measures (TCP, force along the
axis): nothing it commands depends on the force. Out the same way up to the travel height; HOME only after the last
path (real_scan)."""
import time

import numpy as np

from ..configs import SESSION
from ..rotations import align, angles_zyz, rotation_zyz


class CloudChannel:
    """Clouds for the native RViz (native/clouds.py): session/clouds/<name>.npy, float32 BASE metres, written whole
    and then renamed so that a half-written file is never read."""

    def __init__(self):
        self.directory = SESSION / 'clouds'
        self.directory.mkdir(parents=True, exist_ok=True)

    def write(self, name, points_m):
        points = np.asarray(points_m, np.float32)
        temporary = self.directory / f'.{name}.npy'
        np.save(temporary, points[:, :3] if points.size else np.zeros((0, 3), np.float32))
        temporary.replace(self.directory / f'{name}.npy')


def turning_points(positions, axis, angle_deg=30.):
    """Row ends of a serpentine path: where its direction across the axis turns, and its end."""
    flat = positions - np.outer(positions @ axis, axis)
    step = np.diff(flat, axis=0)
    step /= np.maximum(np.linalg.norm(step, axis=1, keepdims=True), 1e-9)
    turns = np.nonzero(np.sum(step[1:] * step[:-1], axis=1) < np.cos(np.radians(angle_deg)))[0] + 1
    return positions[[*turns, len(positions) - 1]]


class AxisScraper:
    def __init__(self, program, scan, channel):
        self.program, self.probe, self.robot = program, program.probe, program.robot
        self.scan, self.channel, self.motion = scan, channel, program.config['motion']
        self.threshold = program.config['scan']['touch_threshold']
        self.points, self.next_write = [], 0.  # measured [x, y, z, Fn] (BASE m, N)
        self.records, self.path = [], 0  # every stroke sample [t s, x y z mm, force along the axis N, axis, path]
        self.telemetry = []  # measured pose/wrench, kept separately from the existing report columns
        self.transport = None
        if 'search_depth_mm' in scan:
            from ..surface_follow.transport import VirtualTransport
            self.transport = VirtualTransport(program)
            self.transport.wait_ready()
            self.sequence = -1
        self.down = None  # tool orientation between paths: HOME's, read at the first path

    def command(self, axis, magnitude, ramp=0.):
        """Desired force (N) into the surface along the fixed BASE axis."""
        k = int(np.argmax(np.abs(axis)))
        force, mask = [0.] * 6, [0] * 6
        force[k], mask[k] = -magnitude * float(np.sign(axis[k])), 1
        self.robot.set_desired_force(force, mask, time=ramp, mod=self.robot.DR_FC_MOD_ABS)

    def reaction(self, axis):
        pose, wrench = self.probe.read()
        return pose, float(wrench[:3] @ axis)

    def measure(self, axis):
        """movel `until` callback: record the TCP in contact, never stops the move (no force limit either: climbing a
        slope raises the reaction, which is measured, not a fault)."""
        def until(pose, wrench):
            guard_wrench = wrench
            if self.transport:
                state = self.transport.state()
                if state.sequence == self.sequence:
                    return False
                self.sequence = state.sequence
                pose = np.r_[state.position*1000,angles_zyz(state.rotation,pose[3:])]
                wrench = state.fast_wrench
                guard_wrench = state.wrench  # native filtered feedback; retain raw contact impulses in the report
            force = float(wrench[:3] @ axis)
            now = time.monotonic()
            if 'search_depth_mm' in self.scan:
                self.probe.guard(guard_wrench)
                self.lost_since = (self.lost_since or now) if force < self.threshold else None
                if (pose[:3] @ axis < self.reference_level-self.scan['search_depth_mm'] or
                        self.lost_since is not None and now-self.lost_since > self.scan['contact_loss_s']):
                    raise RuntimeError('Performance scan lost contact or exceeded its search depth')
            self.records.append([now, *pose[:3], force, *axis, self.path])
            if 'search_depth_mm' in self.scan:
                self.telemetry.append([now,*pose,*wrench])
            if force > self.threshold:
                point = np.asarray(pose[:3],float)
                if 'search_depth_mm' in self.scan:
                    point = point-self.program.config['probe']['tip_radius_mm']*(rotation_zyz(pose[3:])[:,2]+axis)
                # the measured cloud at sample_mm spacing (not set: every reading)
                if not self.points or np.linalg.norm(point - np.asarray(self.points[-1][:3]) * 1000) >= \
                        self.scan.get('sample_mm', 0.):
                    self.points.append([*(point / 1000), force])
            if time.monotonic() >= self.next_write:
                self.next_write = time.monotonic() + 1.
                self.channel.write('measured', self.points)
            return False
        return until

    def press(self, axis):
        """Static reading as the contact reference, approach force until contact, ramp to the scan force."""
        s, static, still = self.scan, [], time.monotonic() + .5
        while time.monotonic() < still:
            static.append(self.reaction(axis)[1])
        offset = float(np.mean(static))
        self.probe.enable(self.robot.DR_BASE, s['stiffness'])
        self.command(axis, s['approach_force_n'])
        self.probe.forcing = True
        hits, deadline = 0, time.monotonic() + s['timeout_s']
        while hits < 3:
            if 'search_depth_mm' in s and self.reaction(axis)[0][:3] @ axis < self.reference_level-s['search_depth_mm']:
                raise RuntimeError('Performance approach exceeded its contact search depth')
            hits = hits + 1 if self.reaction(axis)[1] - offset > self.threshold else 0
            if time.monotonic() > deadline:
                raise RuntimeError(f'{s["timeout_s"]:.0f} s 안에 접촉이 없습니다')
        self.command(axis, s['force_n'], s['ramp_s'])
        time.sleep(s['ramp_s'] + s['hold_s'])

    def run(self, positions, axis):
        p, probe, s, m = self.program, self.probe, self.scan, self.motion
        self.reference_level, self.lost_since = float(positions[0] @ axis), None
        out, pre = positions[0] + s['standoff_mm'] * axis, positions[0] + s['gap_mm'] * axis
        self.path += 1
        pose, _ = probe.read()
        if self.down is None:
            self.down = rotation_zyz(pose[3:])
        down = self.down
        turned = align(down, axis, np.pi)
        travel = max(p.safe_z(), out[2], pose[2])
        probe.move([*pose[:2], travel], down, m['approach_speed'], acc=m['approach_acc'])
        probe.move([*out[:2], travel], down, m['linear_speed'], acc=m['linear_acc'])
        probe.move(out, down, m['approach_speed'], acc=m['approach_acc'])
        probe.move(out, turned, m['approach_speed'])
        probe.move(pre, turned, m['approach_speed'], acc=m['approach_acc'])
        self.press(axis)
        level = probe.read()[0][:3] @ axis  # strokes keep the contact level: the force controller owns the axis
        for point in turning_points(positions, axis):
            probe.move(point + (level - point @ axis) * axis, turned, s['speed_mm_s'], until=self.measure(axis),
                       acc=p.config['scan']['acc'])
        probe.release()
        pose, _ = probe.read()
        probe.move(pose[:3] + s['gap_mm'] * axis, turned, m['approach_speed'], acc=m['approach_acc'])
        probe.close()
        away = pose[:3] + s['standoff_mm'] * axis
        probe.move(away, turned, m['approach_speed'], acc=m['approach_acc'])
        probe.move(away, down, m['approach_speed'])
        probe.move([*away[:2], max(p.safe_z(), away[2])], down, m['approach_speed'], acc=m['approach_acc'])
