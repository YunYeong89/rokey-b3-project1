"""Force-controlled sphere-probe contact on the DSR services (BASE mm/deg, N)."""
import time

import numpy as np

if __package__:
    from .rotations import angles_zyz, rotation_zyz
else:
    from rotations import angles_zyz, rotation_zyz


class SplineProgress:
    """Locate the TCP along consecutive knots, so a restart cannot jump to another raster row."""

    def __init__(self, points, axes=(0, 1, 2), tolerance=0.5):
        self.points, self.axes, self.tolerance = np.asarray(points, float), list(axes), tolerance
        self.projected = self.points[:, self.axes]
        self.steps = np.diff(self.projected, axis=0)
        self.squared = np.maximum(np.sum(self.steps ** 2, axis=1), 1e-9)
        self.segment = 0

    def locate(self, position):
        start = max(0, self.segment - 1)
        window = slice(start, self.segment + 3)
        delta = position[self.axes] - self.projected[:-1][window]
        steps = self.steps[window]
        fraction = np.clip(np.sum(delta * steps, axis=1) / self.squared[window], 0., 1.)
        error = np.linalg.norm(delta - fraction[:, None] * steps, axis=1)
        nearest = int(np.argmin(error))
        return start + nearest, error[nearest]

    def observe(self, position):
        segment, error = self.locate(position)
        if error <= self.tolerance:
            self.segment = max(segment, self.segment)

    def reached(self, position):
        self.observe(position)
        return (self.segment == len(self.steps) - 1
                and self.distance(position, self.points[-1]) <= self.tolerance)

    def distance(self, position, point):
        return np.linalg.norm((position - point)[self.axes])

    def remaining(self, position):
        segment, error = self.locate(position)
        if error > self.tolerance:
            raise RuntimeError(f'정지한 TCP가 마지막 스캔 구간에서 {error:.2f} mm 벗어나 재시도할 수 없습니다')
        self.segment = segment
        first = segment + 1
        if first < len(self.points) - 1 and self.distance(position, self.points[first]) <= 0.2:
            first += 1
        return self.points[first:]


class ContactProbe:
    """TOOL +Z points from the flange into the surface: the robot presses with fd_z > 0 and the
    measured reaction appears along TOOL -Z, so Fn = -F . z_tool is positive in contact."""
    stiffness = [3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0]
    limit_force, tolerance, max_force, hits_required = 0.5, 0.5, 8.0, 3

    def __init__(self, robot, force, touch_threshold=0.5):
        self.robot, self.force, self.touch_threshold = robot, force, touch_threshold
        self.compliant = self.forcing = False
        self.motion_retries, self.on_motion_retry = 0, None
        self.bias, self.hits = np.zeros(6), 0
        # The defined TCP less the controller's active one, in the tool frame (mm): the controller moves and reports
        # its own TCP, the program commands and reads the defined one (real_scan: real_scene.yaml probe.tcp_mm_deg).
        self.offset = np.zeros(3)

    def read(self):
        """BASE pose of the defined TCP and the tared BASE wrench."""
        return self.position(), np.asarray(self.robot.get_tool_force()) - self.bias

    def position(self):
        pose, _ = self.robot.get_current_posx()
        pose = np.asarray(pose, float)
        pose[:3] += rotation_zyz(pose[3:]) @ self.offset
        return pose

    def tare(self, samples=10):
        """Zero the wrench in free space while static, right before a contact search."""
        self.bias = np.mean([self.robot.get_tool_force() for _ in range(samples)], axis=0)
        self.hits = 0

    @staticmethod
    def normal_force(pose, wrench):
        return -float(wrench[:3] @ rotation_zyz(pose[3:])[:, 2])

    def touched(self, pose, wrench):
        """Debounced: Fn above the calibrated threshold on consecutive samples."""
        self.hits = self.hits + 1 if self.normal_force(pose, wrench) > self.touch_threshold else 0
        return self.hits >= self.hits_required

    def guard(self, wrench):
        if np.linalg.norm(wrench[:3]) > self.max_force:
            raise RuntimeError(f'힘 초과: F={wrench[:3].round(2).tolist()} N')

    def enable(self, frame=None, stiffness=None):
        """Task compliance in TOOL (default) or `frame`, at `stiffness` (default the class's); set_desired_force then
        uses the same frame."""
        self.robot.use_force_frame(self.robot.DR_TOOL if frame is None else frame)
        self.robot.task_compliance_ctrl(list(stiffness or self.stiffness))
        self.compliant = True

    def press(self):
        """(Re)latch the force axis to the current TOOL Z. The controller regulates the raw reading,
        so the tare bias along TOOL Z is added to get the requested force on the tared scale."""
        pose, _ = self.robot.get_current_posx()
        bias = self.normal_force(np.asarray(pose), self.bias)
        self.robot.set_desired_force([0.0, 0.0, self.force + bias, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0],
                                     mod=self.robot.DR_FC_MOD_ABS)
        self.forcing = True

    def release(self):
        if self.forcing:
            self.robot.release_force()
            self.forcing = False

    def close(self):
        if self.robot.motion_active:
            self.robot.stop()
        try:
            self.release()
        finally:
            if self.compliant:
                self.robot.release_compliance_ctrl()
                self.compliant = self.forcing = False

    @staticmethod
    def turn_deg(angles, rotation):
        """Angle (deg) from the ZYZ orientation `angles` to the matrix `rotation`."""
        return np.degrees(np.arccos(np.clip((np.trace(rotation_zyz(angles).T @ rotation) - 1) / 2, -1, 1)))

    def move(self, position, rotation, speed=10.0, until=None, acc=50.0, *, angular_speed=20.0, angular_acc=60.0):
        """Blocking BASE movel; returns True when `until(pose, wrench)` stopped it early."""
        position = np.asarray(position, float)
        for attempt in range(self.motion_retries + 1):
            faults = self.robot.motion_fault_count if self.motion_retries else 0
            try:
                return self._move_once(position, rotation, speed, until, acc, angular_speed, angular_acc)
            except TimeoutError as error:
                pose, wrench = self._recover_motion(error, attempt, faults)
                if until is not None and until(pose, wrench):
                    return True
                if np.linalg.norm(pose[:3] - position) <= self.tolerance and self.turn_deg(pose[3:], rotation) <= 1.:
                    return False
                self._log_retry(error, attempt)

    def _move_once(self, position, rotation, speed, until, acc, angular_speed, angular_acc):
        current, _ = self.robot.get_current_posx()
        goal = position - rotation @ self.offset  # the controller's TCP there
        target = [*map(float, goal), *angles_zyz(rotation, current[3:])]
        distance = np.linalg.norm(np.subtract(goal, current[:3]))
        turn = self.turn_deg(current[3:], rotation)
        duration = max(distance / speed + speed / acc, turn / angular_speed + angular_speed / angular_acc)
        stopped, started = self._command(lambda: self.robot.amovel(target, [speed, angular_speed], [acc, angular_acc]),
                                         duration, until, f'movel {target}')
        if stopped:
            return True
        if not started and (distance > 1.0 or turn > 1.0):
            # The controller can refuse a move with only a warning alarm (e.g. unreachable target), but a short move
            # (a 1.5 mm row change at 4 mm/s, a few mm down to the standoff) can also be over before the first
            # check_motion answers on the emulated controller: refused only when the TCP has not moved.
            after, _ = self.robot.get_current_posx()
            if (np.linalg.norm(np.subtract(after[:3], current[:3])) < 0.5
                    and self.turn_deg(after[3:], rotation_zyz(current[3:])) < 0.5):
                raise RuntimeError(f'movel이 실행되지 않았습니다: 목표={[round(v, 1) for v in target]}, '
                                   f'마지막 컨트롤러 알람: {self.robot.last_alarm or "없음"}')
        return False

    def blend(self, poses, speed, radius, until=None, acc=50., *, angular_speed=20., angular_acc=60.):
        """Join pre-cleared linear travel legs and verify the final stop before a subsequent command."""
        current, _ = self.robot.get_current_posx()
        angles = current[3:]
        targets, turn = [], 0.
        for position, rotation in poses:
            goal = position - rotation @ self.offset
            turn += self.turn_deg(angles, rotation)
            angles = angles_zyz(rotation, angles)
            targets.append([*map(float, goal), *angles])
        length = np.linalg.norm(np.diff(np.vstack([current, targets])[:, :3], axis=0), axis=1)
        if radius <= 0. or length.min() <= 2. * radius:
            stopped = any(self.move(position, rotation, speed, until, acc,
                                    angular_speed=angular_speed, angular_acc=angular_acc)
                          for position, rotation in poses)
        else:
            duration = length.sum() / speed + speed / acc + turn / angular_speed + angular_speed / angular_acc
            stopped, _ = self._command(lambda: self.robot.amoveb(targets, radius, [speed, angular_speed],
                                                                [acc, angular_acc]), duration, until, 'moveb')
        if not stopped:
            reached = self.position()
            position, rotation = poses[-1]
            if np.linalg.norm(reached[:3] - position) > self.tolerance or self.turn_deg(reached[3:], rotation) > .5:
                raise RuntimeError(f'직선 이동 최종 목표 미도달: {np.round(position, 2).tolist()} mm')
        return stopped

    def spline(self, positions, rotation, speed, until=None, acc=50.0, *, constant_speed=True,
               progress=None, read_feedback=True):
        """Blocking BASE spline (movesx) from the TCP through `positions` (at most 100) at one
        orientation; returns True when `until(pose, wrench)` stopped it early."""
        pending = np.asarray(positions, float)
        if self.motion_retries and progress is None:
            initial = self.position()
            progress = SplineProgress(np.vstack([initial[:3], pending]), tolerance=0.2)
        feedback = until
        if progress is not None and read_feedback:
            def feedback(pose, wrench):
                progress.observe(pose[:3])
                return until is not None and until(pose, wrench)

        for attempt in range(self.motion_retries + 1):
            faults = self.robot.motion_fault_count if self.motion_retries else 0
            try:
                stopped = self._spline_once(pending, rotation, speed, feedback, acc, constant_speed,
                                            read_feedback, progress if attempt else None)
                if stopped or progress is None:
                    return stopped
                pose = self.position()
                if progress.reached(pose[:3]) and self.turn_deg(pose[3:], rotation) <= 3.:
                    return False
                raise TimeoutError(f'movesx {len(pending)}점 종료 후 목표 미도달')
            except TimeoutError as error:
                pose, wrench = self._recover_motion(error, attempt, faults)
                if read_feedback and until is not None and until(pose, wrench):
                    return True
                if self.turn_deg(pose[3:], rotation) > 3.:
                    raise RuntimeError('정지한 TCP 자세가 스플라인과 달라 재시도할 수 없습니다') from error
                if progress.reached(pose[:3]):
                    return False
                pending = progress.remaining(pose[:3])
                self._log_retry(error, attempt, f' (남은 스플라인 {len(pending)}점)')

    def _spline_once(self, positions, rotation, speed, until, acc, constant_speed, read_feedback, resume):
        current, _ = self.robot.get_current_posx()
        angles = angles_zyz(rotation, current[3:])
        offset = rotation @ self.offset
        goals = positions - offset
        targets = [[*map(float, point), *angles] for point in goals]
        velocity, acceleration = [speed, 20.0], [acc, 60.0]
        opt = int(constant_speed)
        if resume is not None and resume.distance(np.asarray(current[:3]) + offset, positions[0]) < 1.:
            opt = 0  # the constant-speed spline refuses a leading leg below 1 mm
        if resume is not None and len(targets) == 1:
            send = lambda: self.robot.amovel(targets[0], velocity, acceleration)
        else:
            send = lambda: self.robot.amovesx(targets, velocity, acceleration, opt=opt)
        length = np.linalg.norm(np.diff(np.vstack([current[:3], goals]), axis=0), axis=1).sum()
        return self._command(send, length / speed + speed / acc, until, f'movesx {len(goals)}점', read_feedback)[0]

    def _command(self, send, duration, until, label, read_feedback=True):
        try:
            send()
        except TimeoutError:
            # A service reply can go missing after the controller has accepted the motion.
            if not self.motion_retries or self.robot.check_motion() == 0:
                raise
        margin = 0.5 * duration if self.motion_retries else 0.
        return self.wait(time.monotonic() + duration + margin + 5.0, until, label, read_feedback)

    def _recover_motion(self, error, attempt, faults):
        if attempt >= self.motion_retries:
            raise error
        self.robot.check_cancelled()
        if self.robot.motion_fault_count != faults:
            raise RuntimeError(f'컨트롤러 알람으로 재시도 중단: {self.robot.last_alarm}') from error
        self.robot.stop()  # acknowledge a stop even if the original request timed out before its reply
        deadline = time.monotonic() + 3.
        while self.robot.check_motion() != 0:
            self.robot.check_cancelled()
            if time.monotonic() >= deadline:
                raise RuntimeError('정지 확인 실패로 재시도 중단') from error
            time.sleep(.05)
        if self.robot.get_robot_state() != 1 or self.robot.motion_fault_count != faults:
            raise RuntimeError(f'컨트롤러 준비 상태 확인 실패로 재시도 중단: {self.robot.last_alarm}') from error
        if self.forcing and self.robot.get_control_mode() != 4:
            raise RuntimeError('힘 제어 상태가 풀려 스캔 재시도 중단') from error
        pose, wrench = self.read()
        self.guard(wrench)
        return pose, wrench

    def _log_retry(self, error, attempt, detail=''):
        message = f'{error} → 정지 확인 후 재시도 {attempt + 1}/{self.motion_retries}{detail}'
        self.robot.node.get_logger().warn(message)
        if self.on_motion_retry is not None:
            self.on_motion_retry()

    def wait(self, deadline, until, label, read_feedback=True):
        """Until the requested motion ends -> (stopped by `until`, seen moving)."""
        time.sleep(0.05)  # check_motion can still report idle right after the request
        started = False
        while self.robot.check_motion() != 0:
            started = True
            self.robot.check_cancelled()
            if until is not None and until(*(self.read() if read_feedback else (None, None))):
                self.robot.stop()
                return True, started
            if time.monotonic() > deadline:
                raise TimeoutError(f'{label} 시간 초과')
            if not read_feedback:
                time.sleep(.02)
        if until is not None and not read_feedback:
            until(None, None)  # drain the final RT samples as the controller becomes idle
        return False, started

    def settle(self, samples=4, timeout=3.0, lost_time=0.3):
        """Mean of consecutive still, in-band samples; None once contact is lost or never settles."""
        still, lost = [], None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pose, wrench = self.read()
            self.guard(wrench)
            fn = self.normal_force(pose, wrench)
            if fn < self.limit_force:
                lost = lost or time.monotonic()
                if time.monotonic() - lost > lost_time:
                    return None
                still = []
                continue
            lost = None
            in_band = abs(fn - self.force) <= self.tolerance
            if not in_band or (still and np.linalg.norm(pose[:3] - still[-1][0][:3]) > 0.02):
                still = []
            if in_band:
                still.append((pose, wrench, fn))
            if len(still) >= samples:
                poses, wrenches, forces = zip(*still)
                return {'pose': np.r_[np.mean(poses, axis=0)[:3], poses[-1][3:]],
                        'wrench': np.mean(wrenches, axis=0), 'fn': float(np.mean(forces))}
        return None
