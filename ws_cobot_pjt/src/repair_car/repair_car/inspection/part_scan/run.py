"""Execution of the planned paths with the DSR force services, as contact_scrape scrapes the block: per path (a
whole pass) its segments (the probe in contact throughout each); from HOME or the last exit with the tool down across
at the travel height, down to the standoff point off the segment's start, turned onto the fixed axis there, in to the
approach gap; task compliance (BASE) + desired force along -axis, contact, ramp to the force, then movel strokes
through the row ends at the contact level while the controller keeps the force. The program only measures (TCP,
force along the axis): nothing it commands depends on the force. Out to the standoff; on to the next segment's
standoff straight across where the planner found that clear, else up to the travel height the same way; HOME only
after the last path (real_scan). Records number the segments (`path`)."""
import json
import time

import numpy as np

from repair_car.config import SESSION
from repair_car.robot_control.rotations import align, angles_zyz, rotation_zyz
from repair_car.inspection.scan_scene import INSPECTION_SCENE
from repair_car.inspection.part_scan.sampling import PointSampler


class CloudChannel:
    """Clouds for the native RViz (native/clouds.py): data/clouds/<name>.npy, float32 BASE metres, written whole
    and then renamed so that a half-written file is never read."""

    def __init__(self):
        self.directory = SESSION / 'clouds'
        self.directory.mkdir(parents=True, exist_ok=True)

    def begin(self, points_m, scene=None):
        """Start the cloud stream and bind it to the BASE placement used for the real scan."""
        self.write('measured', [])
        if scene is not None:
            path = self.directory.parent / INSPECTION_SCENE
            partial = path.with_suffix('.tmp')
            partial.write_text(json.dumps(scene))
            partial.replace(path)
        self.write('reference', points_m)

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
        self.point_samples = []  # [timestamp, path] corresponding to each retained point
        self.records, self.path = [], 0  # every stroke sample [t s, x y z mm, force along the axis N, axis, path]
        self.sweeps = 0  # paths (passes) run
        self.telemetry = []  # measured pose/wrench, kept separately from the existing report columns
        self.sampler = PointSampler(scan.get('sample_mm', 0.), scan.get('point_decimation', 1))
        self.missed_samples = 0
        self.transport = None
        if scan.get('record_pose_wrench'):
            from repair_car.inspection.surface_follow.transport import VirtualTransport
            self.transport = VirtualTransport(program)
            self.transport.wait_ready()
            self.sequence = -1
        self.down = None  # tool orientation between paths: HOME's, read at the first path

    def command(self, axis, magnitude, ramp=0., wait=True):
        """Desired force (N) into the surface along the fixed BASE axis, on the tared scale: the controller regulates
        its raw estimate, so the probe's tare (force_offset on the real robot) along the axis is added."""
        magnitude += float(self.probe.bias[:3] @ axis)
        k = int(np.argmax(np.abs(axis)))
        force, mask = [0.] * 6, [0] * 6
        force[k], mask[k] = -magnitude * float(np.sign(axis[k])), 1
        self.robot.set_desired_force(force, mask, time=ramp, mod=self.robot.DR_FC_MOD_ABS, wait=wait)

    def reaction(self, axis):
        pose, wrench = self.probe.read()
        return pose, float(wrench[:3] @ axis)

    def measure(self, axis):
        """movel `until` callback: record the TCP in contact, never stops the move (no force limit either: climbing a
        slope raises the reaction, which is measured, not a fault)."""
        def until(pose, wrench):
            if self.transport:
                try:
                    state = self.transport.state()
                except TimeoutError:
                    self.missed_samples += 1
                    return False  # Missing telemetry is not a request to stop the DSR stroke.
                if state.sequence == self.sequence:
                    return False
                self.sequence = state.sequence
                pose = np.r_[state.position*1000,angles_zyz(state.rotation,pose[3:])]
                wrench = state.fast_wrench
            force = float(wrench[:3] @ axis)
            now = time.monotonic()
            # Contact gaps and force peaks are recorded without interrupting the stroke.
            self.records.append([now, *pose[:3], force, *axis, self.path])
            if self.scan.get('record_pose_wrench'):
                self.telemetry.append([now,*pose,*wrench])
            rotation, point = rotation_zyz(pose[3:]), np.asarray(pose[:3], float)
            if self.scan.get('record_pose_wrench'):
                point = point - self.program.config['probe']['tip_radius_mm'] * (rotation[:, 2] + axis)
            if force > self.threshold and self.sampler.keep(point):
                self.points.append([*(point / 1000), force])
                self.point_samples.append([now, self.path])
            self.program.topics.sample(now, 'stroke', axis, pose[:3], rotation, wrench, force,
                                       force > self.threshold, point, len(self.points))
            if time.monotonic() >= self.next_write:
                self.next_write = time.monotonic() + 1.
                self.channel.write('measured', self.points)
            return False
        return until

    def drift(self, position, direction, axis):
        """Change of the reading along `axis` from the press's still one, at `position` moving along `direction`:
        none here (part_scan/height_dsr.py: the motion_offset calibration)."""
        return 0.

    def tare_at_gap(self, axis):
        """Measure the unloaded reaction at the current approach gap."""
        static, still = [], time.monotonic() + .5
        while time.monotonic() < still:
            pose, value = self.reaction(axis)
            static.append(value)
        offset = float(np.mean(static))
        # Tared reaction at rest off the surface here (the estimate's offset depends on the pose), and where.
        self.static, self.static_at = offset, pose[:3]
        return offset

    def search_contact(self, axis, surface, offset=None):
        """Find contact and confirm the reaction at rest while still in position control; the rise over `offset`
        (the reading at the gap by default)."""
        d, hits, offset = self.scan['default_scan'], 0, self.static if offset is None else offset
        rise = d['real_contact_offset_n']

        def touched(pose, wrench):
            nonlocal hits
            hits = hits + 1 if float(wrench[:3] @ axis) - offset - self.drift(pose[:3], -axis, axis) > rise else 0
            return hits >= 2

        pose, _ = self.probe.read()
        self.program.log(f'Contact search: position-controlled descent {d["real_search_speed_mm_s"]:g} mm/s, '
                         f'contact threshold +{rise:g} N (resting reaction {offset:.3f} N)')
        while True:
            if not self.probe.move(surface - d['real_search_depth_mm'] * axis, rotation_zyz(pose[3:]),
                                   d['real_search_speed_mm_s'], until=touched, acc=self.motion['approach_acc']):
                pose, force = self.reaction(axis)
                raise RuntimeError(f'No contact down to {d["real_search_depth_mm"]:g} mm below the planned surface: reaction '
                                   f'{force:.2f} N (resting {offset:.2f} N), check the vehicle position in real_scene.yaml')
            # Confirmed at rest: while moving, the estimate's offset (travel direction, pose) can pass the rise
            # (a stop 5.7 mm above the bonnet at +0.5 N); at rest its noise is 0.01-0.07 N.
            time.sleep(.2)
            readings = [self.reaction(axis) for _ in range(10)]
            rest = float(np.mean([value for _, value in readings])) - offset
            if rest > rise:
                return readings[-1][0]
            hits = 0
            self.program.log(f'Contact candidate: resting reaction +{rest:.2f} N < +{rise:g} N, judged as air, continuing the search')

    def press(self, axis, surface):
        """Find contact, then engage the survey force."""
        s, offset = self.scan, self.tare_at_gap(axis)
        if self.program.mode == 'real':
            self.contact = self.search_contact(axis, surface)[:3]
            self.probe.enable(self.robot.DR_BASE, s['stiffness'])
            self.probe.forcing = True
        else:
            self.probe.enable(self.robot.DR_BASE, s['stiffness'])
            self.command(axis, s['approach_force_n'])
            self.probe.forcing = True
            self.program.log(f'Contact search: press {-s["approach_force_n"]:g} N (resting reaction {offset:.3f} N)')
            hits, deadline = 0, time.monotonic() + s['timeout_s']
            while hits < 3:
                pose, force = self.reaction(axis)
                hits = hits + 1 if force - offset > self.threshold else 0
                if time.monotonic() > deadline:
                    raise RuntimeError(f'No contact within {s["timeout_s"]:.0f} s: TCP is '
                                       f'{float((pose[:3] - surface) @ axis):+.1f} mm from the planned surface, reaction {force:.2f} N')
        self.command(axis, s['force_n'], s['ramp_s'])
        time.sleep(s['ramp_s'] + s['hold_s'])
        pose, force = self.reaction(axis)
        self.program.log(f'Contact confirmed: reaction {force:.3f} N, TCP {pose[:3].round(2).tolist()} mm '
                         f'(planned surface {float((pose[:3] - surface) @ axis):+.2f} mm); following the planned path')

    def run(self, segments, axis, straight, *, lift_after=True):
        """One path (pass): `segments` (BASE mm points) in order, `straight[k]`: lift from segment k to k + 1 straight
        across at the standoff. `lift_after=False` leaves the last lift to the following transfer or HOME."""
        p, probe, s, m = self.program, self.probe, self.scan, self.motion
        self.sweeps += 1
        p.topics.stage(p.topics.MOVING, path=self.sweeps)
        pose, _ = probe.read()
        if self.down is None:
            self.down = rotation_zyz(pose[3:])
        down = self.down
        turned = self.tool(down, axis)
        turn_at_standoff = p.mode != 'real' or not np.allclose(down, turned, atol=1e-6)
        for k, positions in enumerate(segments):
            out = positions[0] + s['standoff_mm'] * axis
            if k and straight[k - 1]:
                probe.move(out, turned, m['linear_speed'], acc=m['linear_acc'])
            else:
                pose, _ = probe.read()
                travel = max(p.safe_z(), out[2], pose[2])
                if p.mode == 'real':
                    p.scan_transfer(pose, out[:2], travel, down)
                else:
                    probe.move([*pose[:2], travel], down, m['approach_speed'], acc=m['approach_acc'])
                    probe.move([*out[:2], travel], down, m['linear_speed'], acc=m['linear_acc'])
                if p.mode != 'real' or travel > out[2]:
                    probe.move(out, down, m['approach_speed'], acc=m['approach_acc'])
                if turn_at_standoff:
                    probe.move(out, turned, m['approach_speed'])
            away = self.segment(positions, axis, turned) + s['standoff_mm'] * axis
            probe.move(away, turned, m['approach_speed'], acc=m['approach_acc'])
            if k == len(segments) - 1 or not straight[k]:
                if turn_at_standoff:
                    probe.move(away, down, m['approach_speed'])
                if lift_after or k < len(segments) - 1:
                    probe.move([*away[:2], max(p.safe_z(), away[2])], down, m['approach_speed'], acc=m['approach_acc'])

    def tool(self, down, axis):
        """Tool orientation along a path: TOOL +Z straight into the surface (`axis` outward)."""
        return align(down, axis, np.pi)

    def segment(self, positions, axis, turned):
        """In to the approach gap, press, the strokes in contact, out to the gap. -> the last contact point (BASE mm)"""
        p, probe, s, m = self.program, self.probe, self.scan, self.motion
        self.path += 1
        probe.move(positions[0] + s['gap_mm'] * axis, turned, m['approach_speed'], acc=m['approach_acc'])
        self.press(axis, positions[0])
        p.topics.stage(p.topics.SCANNING, segment=self.path)
        self.strokes(positions, axis, turned)
        p.topics.advance(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
        p.topics.stage(p.topics.MOVING)
        probe.release()
        pose, _ = probe.read()
        probe.move(pose[:3] + s['gap_mm'] * axis, turned, m['approach_speed'], acc=m['approach_acc'])
        probe.close()
        return pose[:3]

    def strokes(self, positions, axis, turned):
        """Movel through the row ends at the contact level: the force controller owns the axis."""
        level = self.probe.read()[0][:3] @ axis
        for point in turning_points(positions, axis):
            self.probe.move(point + (level - point @ axis) * axis, turned, self.scan['speed_mm_s'],
                            until=self.measure(axis), acc=self.program.config['scan']['acc'])
