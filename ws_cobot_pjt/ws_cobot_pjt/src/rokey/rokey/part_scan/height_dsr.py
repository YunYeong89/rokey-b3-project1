"""Real mesh-height scan with DSR position-controlled contact search and spline approach.

Each path starts at the verified TCP after the extra descent. Force control is enabled there before scanning the
shifted mesh path with constant-speed splines. Samples use the unloaded reaction at each approach gap as the baseline.
"""
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from ..rotations import rotation_zyz
from .height_plan import press_label
from .run import AxisScraper
from .sampling import PointSampler
from .scan_timing import StrokeTiming
from .telemetry import Telemetry


class MeshHeightDsr(AxisScraper):
    SPLINE_POINTS = 100  # DSR movesx limit
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
        stiffness = list(plan.scan['mesh_normal']['stiffness'])
        stiffness[:3] = [stiffness[2] if i == self.k else stiffness[0] for i in range(3)]
        scan = dict(plan.scan, force_n=plan.scan['default_scan']['survey']['force_n'], stiffness=stiffness)
        super().__init__(program, scan, channel)
        self.plan, self.report = plan, report
        self.threshold = scan['default_scan']['contact_threshold_n']
        self.cloud = scan['mesh_normal']['cloud_threshold_n']  # the drawn cloud only; samples.csv keeps every sample
        self.telemetry = Telemetry(program.node, scan['mesh_normal']['telemetry_average']) \
            if program.mode == 'real' else None  # stroke samples off the RT stream (virtual: none, service reads)
        self.sampler = PointSampler(scan['sample_mm'], scan['mesh_normal']['point_decimation'])
        self.radius = program.config['probe']['tip_radius_mm']
        self.approaches = []
        self.timings, self.completed, self.last = [], False, None
        self.trim = 0.  # force command trim (N), kept from path to path
        self.pushed, self.below = 0., None  # floating: the extra push (N) and since when the reaction is under

    def tool(self, down, axis):
        """The plan's fixed tool orientation (the door's TCP axis tilted toward down), not straight along the axis."""
        return self.trajectory.sample(0.)[1]

    def press(self, axis, surface):
        """Verify the end of the position-only descent before enabling force and confirming contact."""
        settings = self.scan['mesh_normal']
        time.sleep(settings['press_settle_s'])
        self.tare_at_gap(axis)
        approach = dict(path=self.path, unloaded_reaction_n=self.static)
        self.approaches.append(approach)
        contact = self.search_contact(axis, surface)[:3]
        depth = settings['extra_depth_mm']
        target = contact - depth * axis
        approach.update(search_tcp_mm=contact.tolist(), target_tcp_mm=target.tolist())
        # The controller requires at least two explicit waypoints, excluding the current TCP.
        waypoints = [contact - 0.5 * depth * axis, target]
        self.probe.spline(waypoints, self.trajectory.sample(0.)[1], settings['extra_depth_speed_mm_s'],
                          until=self.guard_approach, acc=self.motion['approach_acc'], constant_speed=False)
        pose, wrench = self.probe.read()
        self.probe.guard(wrench)
        error = np.linalg.norm(pose[:3] - target)
        approach.update(position_tcp_mm=pose[:3].tolist(), position_error_mm=float(error),
                        position_reaction_n=float(wrench[:3] @ axis) - self.static)
        if not error <= self.APPROACH_TOLERANCE_MM:  # logged, the scan goes on from where the TCP is
            self.program.log(f'경고: 위치 제어 {depth:g} mm 하강 목표 오차 {error:.2f} mm, 현재 위치에서 계속합니다')

        self.contact = pose[:3]
        self.program.log(f'위치 제어 스플라인 {depth:g} mm 하강 완료 (TCP 오차 {error:.2f} mm), 힘 제어 시작')
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
        self.program.log(f'힘 제어 안정화: 반력 {approach["position_reaction_n"]:+.2f} → {ground_force:+.2f} N, '
                         f'TCP 누름 축 변위 {axis_change:+.2f} mm (바깥 방향 +)')
        contact = ground_force > self.GROUND_REACTION_THRESHOLD_N  # logged, the strokes' trim and push go on pressing
        self.program.log(f'힘 제어 후 접촉 {"확인" if contact else "미확인 (경고)"}: 반력 {ground_force:+.2f} N, 스캔 시작')

    def guard_approach(self, _pose, wrench):
        """The position-only descent ends early once the reaction along the axis reaches press_limit_n: on the way to
        0.3 mm it reached 6.4-6.6 N (072408) and the probe guard's 8 N with the unloaded ~1 N (075045, 075132)."""
        self.probe.guard(wrench)
        return float(wrench[:3] @ self.axis) - self.static >= self.scan['mesh_normal']['press_limit_n']

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

    def measure(self, axis):
        """movesx `until` callback: the RT samples since the last call (real robot; the service read otherwise), each
        a samples.csv row and a contact point; never stops the stroke."""
        def until(pose, wrench):
            self.robot.flush()  # a force command waiting for the one in flight
            if self.telemetry is None:
                self.sample(axis, time.monotonic(), pose, wrench)
                return False
            for now, pose, wrench in self.telemetry.drain():
                pose[:3] += rotation_zyz(pose[3:]) @ self.probe.offset  # the probe TCP, as ContactProbe.read
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
        trim = self.scan['mesh_normal']['force_trim']
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
        push = self.scan['mesh_normal']['lift_push']
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
        self.program.log(f'스캔 경로: 누름 축 방향 {shift_mm:+.2f} mm 이동, 메시 높낮이 유지')
        self.window, self.trim_at = [], time.monotonic() + self.scan['mesh_normal']['force_trim']['period_s']
        self.pushed, self.below = 0., None
        if self.telemetry:
            list(self.telemetry.drain())  # the approach and the grounding, before the stroke
        for span in self.trajectory.spans:
            for piece in self.chunks(self.thin(self.resample(span) + shift), self.plane):
                self.probe.spline(piece[1:], turned, self.scan['speed_mm_s'], until=self.measure(axis),
                                  acc=self.program.config['scan']['acc'])
                done += np.linalg.norm(np.diff(piece, axis=0), axis=1).sum()
                self.program.topics.within(done)
        if self.telemetry:
            self.measure(axis)(None, None)  # the last samples of the path
        self.program.log(f'힘 명령 보정: 경로 끝 {self.trim:+.2f} N (명령 {self.scan["force_n"] + self.trim:.2f} N), '
                         f'뜸 감지 후 추가 누름 {self.approaches[-1].get("lift_pushes", 0)}회')

    def run(self):
        """Every path of the plan from HOME, HOME after the last; samples and timings saved in the report."""
        p, reach, topics = self.program, self.plan.reach, self.program.topics
        length = lambda path: np.linalg.norm(np.diff(path.positions, axis=0), axis=1).sum()
        failure = None
        try:
            p.home()
            actual, _ = self.probe.read()  # the defined TCP (real_scan: real_scene.yaml)
            if np.linalg.norm(reach.chain.forward(reach.home)[0][:3, 3] * 1000 - actual[:3]) > 2.:
                raise RuntimeError('컨트롤러 HOME TCP가 계획의 URDF/ProbeTCP와 2 mm 넘게 다릅니다')
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
                p.log(f'mesh-height path {number}/{len(self.plan.paths)}: DSR 접근 -> 위치 제어 접촉 탐색 -> '
                      f'BASE {press_label(self.axis)} {self.scan["force_n"]:g} N, movesx {self.scan["speed_mm_s"]:g} mm/s, '
                      f'{length(path):.0f} mm')
                try:
                    super().run([path.positions], path.axis, [])
                    timing.completed, timing.elapsed = True, path.trajectory.duration
                finally:
                    timing.finish()
            p.home()
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
