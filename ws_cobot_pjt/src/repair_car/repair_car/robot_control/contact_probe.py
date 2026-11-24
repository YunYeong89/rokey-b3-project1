"""Force-controlled sphere-probe contact on the DSR services (BASE mm/deg, N)."""
import time

import numpy as np

if __package__:
    from repair_car.robot_control.rotations import angles_zyz, rotation_zyz
else:
    from robot_control.rotations import angles_zyz, rotation_zyz


class SplineProgress:
    """Locate the TCP along consecutive knots, so a restart cannot jump to another raster row."""

    REJOIN_MARGIN_MM = 0.3

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
        limit = self.tolerance + self.REJOIN_MARGIN_MM
        if error > limit:
            raise RuntimeError(f'The stopped TCP is {error:.3f} mm off the last scan segment, cannot retry '
                               f'(return limit {limit:g} mm)')
        if error > self.tolerance:
            # Rejoin the next waypoint without advancing past the last segment confirmed by feedback.
            self.segment = min(segment, self.segment)
            return self.points[self.segment + 1:]
        self.segment = segment
        first = segment + 1
        if first < len(self.points) - 1 and self.distance(position, self.points[first]) <= 0.2:
            first += 1
        return self.points[first:]


class ContactProbe:
    """TOOL +Z points from the flange into the surface: the robot presses with fd_z > 0 and the
    measured reaction appears along TOOL -Z, so Fn = -F . z_tool is positive in contact."""
    stiffness = [3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0]
    limit_force, tolerance, hits_required = 0.5, 0.5, 3
    TRANSFER_RETRY_MM = 2.0

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
                pose, wrench = self._recover_motion(error, attempt, faults, check_reaction=until is not None)
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
                raise RuntimeError(f'movel was not executed: target={[round(v, 1) for v in target]}, '
                                   f'last controller alarm: {self.robot.last_alarm or "none"}')
        return False

    def blend(self, poses, speed, radius, until=None, acc=50., *, angular_speed=20., angular_acc=60.,
              retry_speed=None, verify_endpoint=True):
        """Join pre-cleared linear travel legs and verify the final stop before a subsequent command."""
        if retry_speed is not None:
            faults, alarm = self.robot.motion_fault_count, self.robot.last_alarm
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
        if stopped:
            return True
        if not verify_endpoint:
            return False
        position, rotation = poses[-1]
        reached = self.position()
        error = np.linalg.norm(reached[:3] - position)
        angle = self.turn_deg(reached[3:], rotation)
        if error <= self.tolerance and angle <= .5:
            return False
        if (retry_speed is not None and self.tolerance < error <= self.TRANSFER_RETRY_MM and angle <= .5
                and self.robot.check_motion() == 0 and self.robot.get_robot_state() == 1
                and self.robot.motion_fault_count == faults and self.robot.last_alarm == alarm):
            self.robot.check_cancelled()
            self.robot.node.get_logger().warn(f'Linear move TCP residual error {error:.2f} mm: retrying once at low speed')
            return self.blend([(position, rotation)], retry_speed, 0., until, acc,
                              angular_speed=angular_speed, angular_acc=angular_acc)
        raise RuntimeError(f'Linear move did not reach the final target: target {np.round(position, 2).tolist()} mm, '
                           f'measured {reached[:3].round(2).tolist()} mm, error {error:.2f} mm / {angle:.2f}°')

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
                raise TimeoutError(f'Target not reached after movesx of {len(pending)} points')
            except TimeoutError as error:
                pose, wrench = self._recover_motion(error, attempt, faults)
                if read_feedback and until is not None and until(pose, wrench):
                    return True
                if self.turn_deg(pose[3:], rotation) > 3.:
                    raise RuntimeError('The stopped TCP pose differs from the spline, cannot retry') from error
                if progress.reached(pose[:3]):
                    return False
                pending = progress.remaining(pose[:3])
                self._log_retry(error, attempt, f' ({len(pending)} spline points left)')

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
        return self._command(send, length / speed + speed / acc, until, f'movesx {len(goals)} points', read_feedback)[0]

    def _command(self, send, duration, until, label, read_feedback=True):
        try:
            send()
        except TimeoutError:
            # A service reply can go missing after the controller has accepted the motion.
            if not self.motion_retries or self.robot.check_motion() == 0:
                raise
        margin = 0.5 * duration if self.motion_retries else 0.
        return self.wait(time.monotonic() + duration + margin + 5.0, until, label, read_feedback)

    def _recover_motion(self, error, attempt, faults, *, check_reaction=True):
        if attempt >= self.motion_retries:
            raise error
        self.robot.check_cancelled()
        if self.robot.motion_fault_count != faults:
            raise RuntimeError(f'Retry aborted by a controller alarm: {self.robot.last_alarm}') from error
        self.robot.stop()  # acknowledge a stop even if the original request timed out before its reply
        deadline = time.monotonic() + 3.
        while self.robot.check_motion() != 0:
            self.robot.check_cancelled()
            if time.monotonic() >= deadline:
                raise RuntimeError('Retry aborted: the stop could not be confirmed') from error
            time.sleep(.05)
        if self.robot.get_robot_state() != 1 or self.robot.motion_fault_count != faults:
            raise RuntimeError(f'Retry aborted: the controller ready state could not be confirmed: {self.robot.last_alarm}') from error
        if self.forcing and self.robot.get_control_mode() != 4:
            raise RuntimeError('Scan retry aborted: force control was released') from error
        if not check_reaction:
            return self.position(), None
        return self.read()

    def _log_retry(self, error, attempt, detail=''):
        message = f'{error} → retry after confirming the stop {attempt + 1}/{self.motion_retries}{detail}'
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
                raise TimeoutError(f'{label} timed out')
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
