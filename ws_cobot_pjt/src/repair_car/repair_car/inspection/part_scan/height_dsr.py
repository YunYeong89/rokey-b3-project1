"""Real mesh-height scan with DSR position-controlled contact search and spline approach.

Each path starts at the verified TCP after the extra descent. Force control is enabled there before scanning the
shifted mesh path with constant-speed splines. Samples use the unloaded reaction at each approach gap as the baseline.
"""
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from repair_car.robot_control.contact_probe import SplineProgress
from repair_car.robot_control.rotations import rotation_zyz
from repair_car.inspection.part_scan.height_plan import press_label
from repair_car.inspection.part_scan.run import AxisScraper
from repair_car.inspection.part_scan.sampling import PointSampler
from repair_car.inspection.part_scan.scan_timing import StrokeTiming
from repair_car.inspection.part_scan.telemetry import Telemetry


class MeshHeightDsr(AxisScraper):
    SPLINE_POINTS = 100  # DSR movesx limit
    MOTION_RETRIES = 2
    # Position-only descent end, pressed 5-6.4 N: the TCP ends 0.10-0.15 mm past the target along the axis and
    # 0.05-0.06 mm toward -x (033819 path 2, 061706 path 1 stopped at 0.1).
    APPROACH_TOLERANCE_MM = 0.2
    GROUND_REACTION_THRESHOLD_N = 0.0
    # movesx points along the planned curve itself, this far apart in travel: the knots thinned to the 1 mm movesx
    # minimum dropped 52-63 % of them where the curvature packed them (spline off the plan up to 0.55 mm on the
    # bonnet); 1.2, not 1.0: a 1.0 mm arc of the 0.7 mm row-end turn is a 0.99 mm chord.
    SPACING_MM = 1.2

    def __init__(self, program, plan, channel, report):
        # One axis for every path. Compliance (real_scan.mesh_height.stiffness, given for the bonnet's BASE Z) soft
        # along it, stiff across: the door's x and z swapped.
        self.axis = plan.paths[0].axis
        self.k = int(np.argmax(np.abs(self.axis)))
        self.plane = [i for i in range(3) if i != self.k]  # across the axis: rows, nearest targets
        stiffness = list(plan.scan['mesh_height']['stiffness'])
        stiffness[:3] = [stiffness[2] if i == self.k else stiffness[0] for i in range(3)]
        scan = dict(plan.scan, force_n=plan.scan['default_scan']['survey']['force_n'], stiffness=stiffness)
        super().__init__(program, scan, channel)
        self.plan, self.report = plan, report
        self.threshold = scan['default_scan']['contact_threshold_n']
        self.cloud = scan['mesh_height']['cloud_threshold_n']  # the drawn cloud only; samples.csv keeps every sample
        self.telemetry = Telemetry(program.node, scan['mesh_height']['telemetry_average']) \
            if program.mode == 'real' else None  # stroke samples off the RT stream (virtual: none, service reads)
        self.sampler = PointSampler(scan['sample_mm'], scan['mesh_height']['point_decimation'])
        self.radius = program.config['probe']['tip_radius_mm']
        self.approaches = []
        self.timings, self.completed, self.last = [], False, None
        self.probe.motion_retries = self.MOTION_RETRIES
        self.probe.on_motion_retry = self.record_resume
        self.trim = 0.  # force command trim (N), kept from path to path
        self.pushed, self.below = 0., None  # floating: the extra push (N) and since when the reaction is under

    def tool(self, down, axis):
        """The plan's fixed tool orientation (the door's TCP axis tilted toward down), not straight along the axis."""
        return self.trajectory.sample(0.)[1]

    def press(self, axis, surface):
        """The first contact (touch), then force control."""
        settings = self.scan['mesh_height']
        depth = settings['extra_depth_mm']
        approach = dict(path=self.path, air_stops=[])
        self.approaches.append(approach)
        contact, target, pose, wrench, rise = self.touch(axis, surface, approach)
        approach.update(search_tcp_mm=contact.tolist(), target_tcp_mm=target.tolist(), confirm_rise_n=rise)
        error = np.linalg.norm(pose[:3] - target)
        approach.update(position_tcp_mm=pose[:3].tolist(), position_error_mm=float(error),
                        position_reaction_n=float(wrench[:3] @ axis) - self.static)
        if not error <= self.APPROACH_TOLERANCE_MM:  # logged, the scan goes on from where the TCP is
            self.program.log(f'Warning: position-controlled {depth:g} mm descent target error {error:.2f} mm, continuing from the current position')

        self.contact = pose[:3]
        self.program.log(f'Position-controlled spline {depth:g} mm descent done (TCP error {error:.2f} mm, reaction rise {rise:+.2f} N), '
                         f'starting force control')
        self.probe.enable(self.robot.DR_BASE, self.scan['stiffness'])
        self.probe.forcing = True
        ramp = self.scan['ramp_s']
        self.command(axis, self.scan['force_n'] + self.trim, ramp)
        time.sleep(ramp)
        # Trimmed at rest as on the strokes: under a fixed command the reaction here settled near 0 N whatever the
        # command (033819 +0.11 N at 0.8 N; 061800 +0.08 and -0.02 N at 1.2 N, path 2 stopped).
        readings, end = [], time.monotonic() + settings['ground_s']
        self.window, self.trim_at = [], time.monotonic() + settings['force_trim']['period_s']
        while time.monotonic() < end:
            pose, force = self.reaction(axis)
            readings.append([*pose[:3], force])
            self.retrim(axis, time.monotonic(), force - self.static)
        settled = np.mean(readings[3 * len(readings) // 4:], axis=0)  # the last period
        ground_force = float(settled[3]) - self.static
        approach.update(force_tcp_mm=settled[:3].tolist(), force_reaction_n=ground_force)
        axis_change = float((settled[:3] - self.contact) @ axis)
        self.program.log(f'Force control settled: reaction {approach["position_reaction_n"]:+.2f} → {ground_force:+.2f} N, '
                         f'TCP press axis displacement {axis_change:+.2f} mm (outward +)')
        contact = ground_force > self.GROUND_REACTION_THRESHOLD_N  # logged, the strokes' trim and push go on pressing
        self.program.log(f'Contact after force control {"confirmed" if contact else "not confirmed (warning)"}: reaction {ground_force:+.2f} N, starting the scan')

    def touch(self, axis, surface, approach):
        """Settle and zero at the gap, search, press extra_depth_mm. Contact: the press raises the reaction
        press_confirm_n over the search's stop, or reaches press_limit_n (a stop already past it is not pressed: the
        press ended at once, +0.03/+0.06 N, read as the air, and the search went on into 8.3/8.9 N, 073012-073054).
        Otherwise in the air: the left door (063056) stopped 5-6.5 mm off the panel at +0.5 N, +0.93/+1.20 N after the
        0.3 mm; searched on from there, its reading the new zero (the force's stays the gap's).
        -> (search stop, press target, pose, wrench, rise)"""
        settings = self.scan['mesh_height']
        depth, limit = settings['extra_depth_mm'], settings['press_limit_n']
        time.sleep(settings['press_settle_s'])
        self.tare_at_gap(axis)
        self.gap = np.mean([self.probe.read()[1][:3] for _ in range(5)], axis=0)
        approach['unloaded_reaction_n'] = baseline = self.static
        while True:
            contact = self.search_contact(axis, surface, baseline)[:3]
            stop = float(np.mean([self.probe.read()[1] for _ in range(5)], axis=0)[:3] @ axis)
            target = contact - depth * axis
            if stop - self.static < limit:
                # The controller requires at least two explicit waypoints, excluding the current TCP.
                # This is one straight press, so a retry needs only the endpoint and can use movel.
                progress = SplineProgress([contact, target], tolerance=.2)
                self.probe.spline([contact - 0.5 * depth * axis, target], self.trajectory.sample(0.)[1],
                                  settings['extra_depth_speed_mm_s'], until=self.guard_approach,
                                  acc=self.motion['approach_acc'], constant_speed=False, progress=progress)
            pose, wrench = self.probe.read()
            reaction = float(wrench[:3] @ axis)
            if reaction - stop >= settings['press_confirm_n'] or reaction - self.static >= limit:
                return contact, target, pose, wrench, reaction - stop
            approach['air_stops'].append([*pose[:3].tolist(), reaction - stop])
            baseline = float(np.mean([self.reaction(axis)[1] for _ in range(5)]))
            self.program.log(f'Contact not confirmed: reaction rise after {depth:g} mm extra press {reaction - stop:+.2f} N < '
                             f'{settings["press_confirm_n"]:g} N, judged as air (or a door not fully closed), continuing the search')

    def guard_approach(self, _pose, wrench):
        """The position-only descent ends early once the reaction along the axis reaches press_limit_n: on the way to
        0.3 mm it reached 6.4-6.6 N (072408) and 8 N with the unloaded ~1 N (075045, 075132)."""
        return float(wrench[:3] @ self.axis) - self.static >= self.scan['mesh_height']['press_limit_n']

    def command(self, axis, magnitude, ramp=0., wait=True):
        """The force on the press's zero (at rest just off the surface), not only HOME's tare: the estimate's offset
        depends on the pose (+0.7..1.0 N over the bonnet), so the controller's raw target carries it too."""
        super().command(axis, magnitude + self.static, ramp, wait)

    @classmethod
    def chunks(cls, points, plane=(0, 1)):
        """Pieces sharing their ends, at most SPLINE_POINTS after the start, each cut at the sharpest turn across the
        axis (`plane` coordinates) in the second half of its window (a raster row end, where the probe slows anyway)."""
        step = np.diff(points[:, list(plane)], axis=0)
        step /= np.maximum(np.linalg.norm(step, axis=1, keepdims=True), 1e-9)
        cosine = np.r_[1., np.sum(step[1:] * step[:-1], axis=1), 1.]
        size, start, last = cls.SPLINE_POINTS, 0, len(points) - 1
        while start < last:
            end = last if last - start <= size else start + size // 2 + int(np.argmin(
                cosine[start + size // 2:start + size + 1]))
            yield points[start:end + 1]
            start = end

    def record_resume(self):
        if self.timings and self.timings[-1].stopped is None:
            self.timings[-1].resumes += 1

    def measure(self, axis, progress=None):
        """movesx `until` callback: the RT samples since the last call (real robot; the service read otherwise), each
        a samples.csv row and a contact point; never stops the stroke."""
        def until(pose, wrench):
            self.robot.flush()  # a force command waiting for the one in flight
            if self.telemetry is None:
                self.sample(axis, time.monotonic(), pose, wrench)
                return False
            for now, pose, wrench in self.telemetry.drain():
                pose[:3] += rotation_zyz(pose[3:]) @ self.probe.offset  # the probe TCP, as ContactProbe.read
                if progress is not None:
                    progress.observe(pose[:3])
                self.sample(axis, now, pose, wrench - self.probe.bias)
            return False
        return until

    def sample(self, axis, now, pose, wrench):
        """One stroke sample: samples.csv row, retrim and push, the drawn point and the topic."""
        position, rotation = np.asarray(pose[:3], float), rotation_zyz(pose[3:])
        nearest = self.tree.query(position[self.plane])[1]
        target = self.targets[nearest]
        velocity = np.zeros(6)
        if self.last is not None:
            turn = (Rotation.from_matrix(rotation) * Rotation.from_matrix(self.last[2]).inv()).as_rotvec()
            velocity = np.r_[position - self.last[1], np.degrees(turn)] / max(now - self.last[0], 1e-3)
        self.last = now, position, rotation
        # Zero: still just off the surface (press), with the moving offset along the planned travel direction.
        # Recorded however far below the stroke path the TCP goes (the dive stop of 2 mm ended runs; 8 N still does).
        force = float(wrench[:3] @ axis) - self.static - self.drift(position, self.directions[nearest], axis)
        angle = np.degrees((Rotation.from_matrix(self.rotation) * Rotation.from_matrix(rotation).inv()).magnitude())
        self.records.append([now, self.path, 'stroke', *position, *target, *axis, *wrench, force,
                             float(np.linalg.norm((position - target)[self.plane])), angle, 0., *velocity, *[np.nan] * 6,
                             0, 0, *[0.] * 30, *(position - self.radius * rotation[:, 2]),
                             *(target - self.radius * self.rotation[:, 2]), *velocity[:3]])
        self.retrim(axis, now, force)
        self.push(axis, now, force)
        point = position - self.radius * (rotation[:, 2] + axis)
        if force > self.cloud and self.sampler.keep(point):
            self.points.append([*(point / 1000), force])
        self.program.topics.sample(now, 'stroke', axis, position, rotation, wrench, force,
                                   force > self.cloud, point, len(self.points))
        if now >= self.next_write:
            self.next_write = now + 1.
            self.channel.write('measured', self.points)

    def retrim(self, axis, now, force):
        """Outer loop on the stroke reaction: each period, the force command moves by `gain` x the shortfall of the
        period's median reaction from force_n, within +-max_n and never below a zero command (a negative one pulls the
        probe off). The reaction under a fixed command fell with speed and travel direction (033819, 2 mm/s at 0.8 N:
        median 1.27 N; 061800, 6 mm/s at 1.2 N: 0.65 N, +y rows 0.52, -y rows 0.77, 24 % under record_threshold_n)."""
        trim = self.scan['mesh_height']['force_trim']
        self.window.append(force)
        if now < self.trim_at:
            return
        median = float(np.median(self.window))
        self.trim = float(np.clip(self.trim + trim['gain'] * (self.scan['force_n'] - median),
                                  -min(trim['max_n'], self.scan['force_n']), trim['max_n']))
        self.command(axis, self.scan['force_n'] + self.trim + self.pushed, trim['ramp_s'], wait=False)
        self.approaches[-1].setdefault('force_trims', []).append([now, median, self.trim])  # [t, median N, trim N]
        self.window, self.trim_at = [], now + trim['period_s']

    def push(self, axis, now, force):
        """Floating: the reaction under threshold_n for hold_s -> the command presses depth_mm further on the
        compliance along the axis (its stiffness x depth_mm) until the reaction is back over release_n. 062732, 6 mm/s:
        where the reaction fell under -0.2 N (9 % of samples) the TCP rode 0.7 mm higher off the path than over 2 N and
        came back down at only 0.4 mm/s, gaps of a few mm in the cloud."""
        push = self.scan['mesh_height']['lift_push']
        if self.pushed:
            if force > push['release_n']:
                self.pushed = 0.
                self.command(axis, self.scan['force_n'] + self.trim, push['ramp_s'], wait=False)
            return
        self.below = (self.below or now) if force < push['threshold_n'] else None
        if self.below is not None and now - self.below >= push['hold_s']:
            self.pushed, self.below = self.scan['stiffness'][self.k] * push['depth_mm'] / 1000, None
            self.command(axis, self.scan['force_n'] + self.trim + self.pushed, push['ramp_s'], wait=False)
            self.approaches[-1]['lift_pushes'] = self.approaches[-1].get('lift_pushes', 0) + 1

    @classmethod
    def resample(cls, span):
        """The span's planned TCP at least SPACING_MM apart in travel, ends kept."""
        return span.tcp_position(np.linspace(0., span.length, max(1, int(span.length // cls.SPACING_MM)) + 1))

    @staticmethod
    def thin(points, spacing=1.):
        """Points at least `spacing` apart, the ends kept (constant-speed movesx refuses segments under 1 mm)."""
        kept = [points[0]]
        for point in points[1:-1]:
            if np.linalg.norm(point - kept[-1]) >= spacing:
                kept.append(point)
        if len(kept) > 1 and np.linalg.norm(points[-1] - kept[-1]) < spacing:
            kept.pop()
        return np.array([*kept, points[-1]])

    def strokes(self, positions, axis, turned):
        """Shift the mesh height path to the verified approach endpoint and follow it under force control."""
        self.rotation, self.last, done = turned, None, 0.
        shift_mm = float((self.contact - self.trajectory.sample(0.)[0]) @ axis)
        shift = shift_mm * axis
        self.targets = self.targets + shift
        self.program.log(f'Scan path: shifted {shift_mm:+.2f} mm along the press axis, keeping the mesh height profile')
        self.window, self.trim_at = [], time.monotonic() + self.scan['mesh_height']['force_trim']['period_s']
        self.pushed, self.below = 0., None
        if self.telemetry:
            list(self.telemetry.drain())  # the approach and the grounding, before the stroke
        tolerance = self.scan['mesh_height']['progress_error_mm']
        for span in self.trajectory.spans:
            for piece in self.chunks(self.thin(self.resample(span) + shift), self.plane):
                progress = SplineProgress(piece, self.plane, tolerance)
                self.probe.spline(piece[1:], turned, self.scan['speed_mm_s'], until=self.measure(axis, progress),
                                  acc=self.program.config['scan']['acc'], progress=progress,
                                  read_feedback=self.telemetry is None)
                done += np.linalg.norm(np.diff(piece, axis=0), axis=1).sum()
                self.program.topics.within(done)
        if self.telemetry:
            self.measure(axis)(None, None)  # the last samples of the path
        self.program.log(f'Force command correction: path end {self.trim:+.2f} N (command {self.scan["force_n"] + self.trim:.2f} N), '
                         f'{self.approaches[-1].get("lift_pushes", 0)} extra presses after lift-off detection')

    def run_bonnet_path(self, path):
        """Run the planned approach and retract at position control, with contact only on the stroke."""
        topics, motion = self.program.topics, self.motion
        rotation = path.trajectory.sample(0.)[1]
        entry, exit_route = path.transfers
        approach = motion['approach_speed']

        def move(point):
            self.probe.blend([(point, rotation)], approach, 0., acc=motion['approach_acc'], retry_speed=approach)

        self.sweeps += 1
        topics.stage(topics.MOVING, path=self.sweeps)
        if self.sweeps == 1:
            move(entry[-1])
        for point in entry[-2::-1]:
            move(point)
        self.segment(path.positions, path.axis, rotation)
        for point in exit_route:
            move(point)

    def run(self):
        """Every path of the plan from HOME, HOME after the last; samples and timings saved in the report."""
        p, reach, topics = self.program, self.plan.reach, self.program.topics
        length = lambda path: np.linalg.norm(np.diff(path.positions, axis=0), axis=1).sum()
        failure = None
        try:
            p.home(scan_travel=True)
            actual, _ = self.probe.read()  # the defined TCP (real_scan: real_scene.yaml)
            if np.linalg.norm(reach.chain.forward(reach.home)[0][:3, 3] * 1000 - actual[:3]) > 2.:
                raise RuntimeError('The controller HOME TCP differs from the plan URDF/ProbeTCP by more than 2 mm')
            self.report.route([(path.trajectory, 'stroke', number) for number, path in enumerate(self.plan.paths, 1)])
            topics.plan([[length(path)] for path in self.plan.paths], self.scan['speed_mm_s'])
            for number, path in enumerate(self.plan.paths, 1):
                timing = StrokeTiming(number, path.trajectory.duration)
                self.timings.append(timing)
                self.trajectory = path.trajectory
                self.targets = np.vstack([span.tcp_position(np.linspace(0., span.length, int(span.length / .1) + 2))
                                          for span in path.trajectory.spans])
                self.tree = cKDTree(self.targets[:, self.plane])
                tangent = np.gradient(self.targets, axis=0)
                self.directions = tangent / np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
                p.log(f'mesh-height path {number}/{len(self.plan.paths)}: DSR approach -> position-controlled contact search -> '
                      f'BASE {press_label(self.axis)} {self.scan["force_n"]:g} N, movesx {self.scan["speed_mm_s"]:g} mm/s, '
                      f'{length(path):.0f} mm')
                try:
                    if path.transfers:
                        self.run_bonnet_path(path)
                    else:
                        super().run([path.positions], path.axis, [], lift_after=False)
                    timing.completed, timing.elapsed = True, path.trajectory.duration
                finally:
                    timing.finish()
            p.home(scan_travel=True)
            self.completed = True
        except BaseException as error:
            failure = str(error) or type(error).__name__
            raise
        finally:
            if self.telemetry:
                self.telemetry.close()
            try:
                self.probe.close()
            finally:
                try:
                    self.channel.write('measured', self.points)
                finally:
                    self.report.save(self.records, self.completed, [timing.export() for timing in self.timings],
                                     approaches=self.approaches, failure=failure)
