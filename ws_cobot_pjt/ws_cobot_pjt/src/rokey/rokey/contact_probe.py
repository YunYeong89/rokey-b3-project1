"""Force-controlled sphere-probe contact on the DSR services (BASE mm/deg, N)."""
import time

import numpy as np

if __package__:
    from .rotations import angles_zyz, rotation_zyz
else:
    from rotations import angles_zyz, rotation_zyz


class ContactProbe:
    """TOOL +Z points from the flange into the surface: the robot presses with fd_z > 0 and the
    measured reaction appears along TOOL -Z, so Fn = -F . z_tool is positive in contact."""
    stiffness = [3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0]
    limit_force, tolerance, max_force, hits_required = 0.5, 0.5, 8.0, 3

    def __init__(self, robot, force, touch_threshold=0.5):
        self.robot, self.force, self.touch_threshold = robot, force, touch_threshold
        self.compliant = self.forcing = False
        self.bias, self.hits = np.zeros(6), 0
        # The defined TCP less the controller's active one, in the tool frame (mm): the controller moves and reports
        # its own TCP, the program commands and reads the defined one (real_scan: real_scene.yaml probe.tcp_mm_deg).
        self.offset = np.zeros(3)

    def read(self):
        """BASE pose of the defined TCP and the tared BASE wrench."""
        pose, _ = self.robot.get_current_posx()
        pose = np.asarray(pose, float)
        pose[:3] += rotation_zyz(pose[3:]) @ self.offset
        return pose, np.asarray(self.robot.get_tool_force()) - self.bias

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

    def move(self, position, rotation, speed=10.0, until=None, acc=50.0):
        """Blocking BASE movel; returns True when `until(pose, wrench)` stopped it early."""
        current, _ = self.robot.get_current_posx()
        position = np.asarray(position, float) - rotation @ self.offset  # the controller's TCP there
        target = [*map(float, position), *angles_zyz(rotation, current[3:])]
        distance = np.linalg.norm(np.subtract(position, current[:3]))
        turn = self.turn_deg(current[3:], rotation)
        self.robot.amovel(target, [speed, 20.0], [acc, 60.0])
        deadline = time.monotonic() + max(distance / speed + speed / acc, turn / 20. + 20. / 60.) + 5.0
        stopped, started = self.wait(deadline, until, f'movel {target}')
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

    def spline(self, positions, rotation, speed, until=None, acc=50.0, *, constant_speed=True):
        """Blocking BASE spline (movesx) from the TCP through `positions` (at most 100) at one
        orientation; returns True when `until(pose, wrench)` stopped it early."""
        current, _ = self.robot.get_current_posx()
        angles = angles_zyz(rotation, current[3:])
        positions = np.asarray(positions, float) - rotation @ self.offset  # the controller's TCP there
        self.robot.amovesx([[*map(float, point), *angles] for point in positions],
                           [speed, 20.0], [acc, 60.0], opt=int(constant_speed))
        length = np.linalg.norm(np.diff(np.vstack([current[:3], positions]), axis=0), axis=1).sum()
        return self.wait(time.monotonic() + length / speed + speed / acc + 5.0, until, f'movesx {len(positions)}점')[0]

    def wait(self, deadline, until, label):
        """Until the requested motion ends -> (stopped by `until`, seen moving)."""
        time.sleep(0.05)  # check_motion can still report idle right after the request
        started = False
        while self.robot.check_motion() != 0:
            started = True
            self.robot.check_cancelled()
            if until is not None and until(*self.read()):
                self.robot.stop()
                return True, started
            if time.monotonic() > deadline:
                raise TimeoutError(f'{label} 시간 초과')
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
