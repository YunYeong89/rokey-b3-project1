"""Shared force-calibration data path (virtual and real): free-space excitation motions and synchronized
samples of joint state, BASE TCP pose and the controller's wrench expressed in TOOL.

The robot must have about 60 mm of free space around and below the start pose."""
from datetime import datetime
from pathlib import Path
import time

import numpy as np
from sensor_msgs.msg import JointState

if __package__:
    from .program import RobotProgram
    from .rotations import axis_angle, rotation_zyz
else:
    from program import RobotProgram
    from rotations import axis_angle, rotation_zyz

PACKAGE = Path(__file__).resolve().parents[1]
CALIBRATION_DIR, IMAGE_DIR = PACKAGE / 'calibration', PACKAGE / 'images'
EPSILON = 0.01  # rad/s, smoothing of the Coulomb friction sign


def smooth(values, window=5):
    if len(values) < window:
        return values
    kernel = np.ones(window) / window
    return np.column_stack([np.convolve(column, kernel, mode='same') for column in values.T])


class ForceData:
    """Column views of recorded rows: t, q, qd (rad, rad/s), BASE pose (mm, deg), TOOL wrench (N, Nm),
    plus the gravity direction and the smoothed TCP acceleration (m/s^2), both in TOOL."""

    def __init__(self, rows, labels):
        rows = np.asarray(rows, dtype=float)
        self.rows, self.labels = rows, np.asarray(labels)
        self.t, self.q, self.qd = rows[:, 0], rows[:, 1:7], rows[:, 7:13]
        self.pose, self.wrench = rows[:, 13:19], rows[:, 19:25]
        rotations = np.array([rotation_zyz(pose[3:]) for pose in self.pose])
        self.gravity = rotations[:, 2, :] * -1.0  # R^T [0, 0, -1]
        velocity = smooth(np.gradient(self.pose[:, :3] / 1000, self.t, axis=0))
        self.acceleration = np.einsum('nji,nj->ni', rotations, smooth(np.gradient(velocity, self.t, axis=0)))

    @staticmethod
    def normal(wrench):
        """Fn = -F . z_tool, the contact force convention of ContactProbe."""
        return -wrench[:, 2]

    @classmethod
    def load(cls, path):
        data = np.load(path)
        return cls(data['rows'], data['labels'])


class ForceRecorder:
    """Joint state from the controller's own stream (velocity by filtered differentiation of stamped
    positions) plus pose and wrench read through ContactProbe."""

    def __init__(self, node, probe, topic):
        self.probe = probe
        self.q, self.qd, self.stamp = np.zeros(6), np.zeros(6), None
        self.rows, self.labels, self.label = [], [], ''
        self.subscription = node.create_subscription(JointState, topic, self.joints, 50)

    def joints(self, message):
        position = dict(zip(message.name, message.position))
        q = np.array([position[f'joint_{index}'] for index in range(1, 7)])
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.stamp is not None and stamp > self.stamp:
            self.qd += 0.3 * ((q - self.q) / (stamp - self.stamp) - self.qd)
        self.q, self.stamp = q, stamp

    def collect(self, pose, wrench):
        rotation = rotation_zyz(pose[3:])
        self.rows.append(np.r_[time.monotonic(), self.q, self.qd, pose, rotation.T @ wrench[:3], rotation.T @ wrench[3:]])
        self.labels.append(self.label)
        return False

    def static(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.collect(*self.probe.read())

    def take(self):
        """Recorded samples since the last take."""
        data = ForceData(self.rows, self.labels)
        self.rows, self.labels = [], []
        return data


class Excitation:
    """Free-space motions around a start pose lifted above home (same start with or without a map).
    With compliance, the robot moves under the same task compliance control as the contact search."""

    def __init__(self, probe, recorder, lift=50.0, span=40.0, compliance=False):
        probe.robot.movej(probe.robot.HOME, 30.0, 60.0)
        pose, _ = probe.read()
        self.probe, self.recorder, self.span = probe, recorder, span
        self.rotation, self.origin = rotation_zyz(pose[3:]), pose[:3]
        self.start = self.origin + [0.0, 0.0, lift]
        probe.move(self.start, self.rotation)
        if compliance:
            probe.enable()

    def go(self, label, position, rotation=None, speed=20.0):
        self.recorder.label = label
        self.probe.move(position, self.rotation if rotation is None else rotation, speed, until=self.recorder.collect)

    def tilt(self, degrees, azimuth):
        axis = [np.cos(np.deg2rad(azimuth)), np.sin(np.deg2rad(azimuth)), 0.0]
        return axis_angle(np.array(axis), np.deg2rad(degrees)) @ self.rotation

    def static_set(self, tilts=(15.0, 30.0), azimuths=range(0, 360, 60), hold=1.0):
        """Payload-gravity poses: vertical plus tilts around the TCP."""
        for tilt, azimuth in [(0.0, 0.0)] + [(tilt, azimuth) for tilt in tilts for azimuth in azimuths]:
            self.probe.move(self.start, self.tilt(tilt, azimuth))
            time.sleep(0.5)
            self.recorder.label = f'static {tilt:.0f}/{azimuth:.0f}'
            self.recorder.static(hold)
        self.probe.move(self.start, self.rotation)

    def sweeps(self, speeds=(10.0, 30.0, 50.0), span=None):
        """Friction excitation: out-and-back along BASE x, y, z at each speed."""
        span = self.span if span is None else span
        for speed in speeds:
            for axis in np.eye(3):
                for target in (self.start + span * axis, self.start - span * axis, self.start):
                    self.go(f'sweep {speed:.0f}', target, speed=speed)

    def random(self, count, rng, max_tilt=30.0):
        """Random targets in the box with random tilt and speed, for pretraining diversity."""
        for _ in range(count):
            target = self.start + rng.uniform(-self.span, self.span, 3)
            rotation = self.tilt(rng.uniform(0, max_tilt), rng.uniform(0, 360))
            self.go('random', target, rotation, rng.uniform(5.0, 50.0))
        self.probe.move(self.start, self.rotation)

    def validation(self):
        """Held-out motions: search-like vertical strokes, a diagonal and a tilted sweep."""
        for speed in (5.0, 10.0, 20.0):
            self.go(f'vertical {speed:.0f}', self.start - [0.0, 0.0, 30.0], speed=speed)
            self.go(f'vertical {speed:.0f}', self.start, speed=speed)
        self.go('diagonal 15', self.start + [30.0, 30.0, -20.0], speed=15.0)
        self.go('diagonal 15', self.start, speed=15.0)
        tilted = self.tilt(20.0, 45.0)
        self.go('tilt 20', self.start, tilted, speed=10.0)
        self.go('tilted sweep 15', self.start + [self.span, 0.0, 0.0], tilted, speed=15.0)
        self.go('tilted sweep 15', self.start - [self.span, 0.0, 0.0], tilted, speed=15.0)
        self.go('tilt 20', self.start, self.rotation, speed=10.0)

    def finish(self):
        self.probe.move(self.start, self.rotation)
        self.probe.move(self.origin, self.rotation)


class CalibrationRun(RobotProgram):
    """Robot program context plus the joint-state recorder for a free-space calibration run."""

    def __init__(self, name):
        super().__init__(name)
        topic = 'virtual_control/joint_states' if self.mode == 'virtual' else 'arm_joint_states'
        self.recorder = ForceRecorder(self.node, self.probe, topic)
        self.meta = {'mode': self.mode, 'tool': self.robot.get_tool(), 'tcp': self.robot.get_tcp(),
                     'created': datetime.now().isoformat(timespec='seconds')}
        CALIBRATION_DIR.mkdir(exist_ok=True)
        self.log(f'{name}: mode={self.mode}, tool={self.meta["tool"]}, tcp={self.meta["tcp"]}, joints={topic}')


def residual_stats(wrench):
    """RMS / peak / std of Fn and RMS of |F| for a TOOL wrench array."""
    normal, norm = ForceData.normal(wrench), np.linalg.norm(wrench[:, :3], axis=1)
    return {'fn_rms': float(np.sqrt(np.mean(normal ** 2))), 'fn_peak': float(np.abs(normal).max()),
            'fn_std': float(normal.std()), 'force_rms': float(np.sqrt(np.mean(norm ** 2)))}
