"""Adaptive TCP pose checkpoints and smooth, force-independent path timing (mm, radians, seconds)."""
import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator
from scipy.spatial.transform import Rotation, RotationSpline


class PoseSpan:
    """A smooth surface stroke; stops occur at path corners, never at normal checkpoints. `checkpoints`: dense
    indices always kept as knots."""
    def __init__(self, positions, rotations, settings, checkpoints=()):
        self.settings = settings
        distance = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        angles = (Rotation.from_matrix(rotations[1:]) * Rotation.from_matrix(rotations[:-1]).inv()).magnitude()
        # A virtual length also parameterises rotations at a stationary TCP.
        self.dense_s = np.r_[0., np.cumsum(np.maximum(distance, angles * 10.))]
        if self.dense_s[-1] < 1e-9:
            raise ValueError('Empty pose span')
        self.length = self.dense_s[-1]
        selected = {0, len(positions) - 1, *map(int, checkpoints)}
        for at in np.arange(settings['checkpoint_max_mm'], self.length, settings['checkpoint_max_mm']):
            selected.add(int(np.searchsorted(self.dense_s, at)))
        selected.update(np.flatnonzero(angles > np.radians(settings['normal_step_deg'])).tolist())
        while True:
            indices = np.array(sorted(selected))
            self.knots = self.dense_s[indices]
            self.position = self.fit_position(self.knots, positions[indices], settings)
            self.rotation = RotationSpline(self.knots, Rotation.from_matrix(rotations[indices]))
            position_error = np.linalg.norm(self.position(self.dense_s) - positions, axis=1)
            angle_error = (self.rotation(self.dense_s) * Rotation.from_matrix(rotations).inv()).magnitude()
            error = np.maximum(position_error / settings['position_interp_error_mm'],
                               angle_error / np.radians(settings['normal_interp_error_deg']))
            added = []
            for a, b in zip(indices[:-1], indices[1:]):
                if b > a + 1:
                    worst = a + 1 + np.argmax(error[a + 1:b])
                    if error[worst] > 1:
                        added.append(worst)
                    elif (self.dense_s[b] - self.dense_s[a] > settings['checkpoint_max_mm'] + 1e-8
                          or (Rotation.from_matrix(rotations[b]) * Rotation.from_matrix(rotations[a]).inv()).magnitude()
                          > np.radians(settings['normal_step_deg'])):
                        added.append((a + b) // 2)
            if not added:
                break
            selected.update(added)
        self.indices, self.checkpoints = indices, positions[indices]
        self.max_position_error = float(position_error.max())
        self.max_angle_error_deg = float(np.degrees(angle_error.max()))

    @staticmethod
    def fit_position(knots, positions, settings):
        if settings.get('position_interpolator', 'cubic') == 'pchip':
            return PchipInterpolator(knots, positions)
        return CubicSpline(knots, positions, bc_type='natural')

    def validation_samples(self, step_mm=None):
        step = self.settings['validation_step_mm'] if step_mm is None else step_mm
        # Subdivide each knot interval; merging two grids made almost coincident samples and noisy derivatives.
        s = np.concatenate([self.knots[:1], *[
            np.linspace(a, b, int(np.ceil((b - a) / step)) + 1)[1:]
            for a, b in zip(self.knots[:-1], self.knots[1:])]])
        # Include enough intermediate orientations for pure turns too.
        while True:
            rotations = self.rotation(s)
            angle = (rotations[1:] * rotations[:-1].inv()).magnitude()
            split = angle > np.radians(self.settings['validation_angle_deg'])
            if not split.any():
                return s, self.position(s), rotations.as_matrix()
            s = np.unique(np.r_[s, ((s[:-1] + s[1:]) / 2)[split]])

    def retime(self, s, joints, speed, jacobians=None):
        p1, p2 = self.position(s, 1), self.position(s, 2)
        w1, w2 = self.rotation(s, 1), self.rotation(s, 2)
        if jacobians is not None:
            base_rates = self.rotation(s).apply(w1)
            twist = np.c_[p1 / 1000, base_rates]
            q1 = np.linalg.solve(np.asarray(jacobians), twist[..., None])[..., 0]
        else:
            q1 = np.gradient(joints, s, axis=0)
        q2 = np.gradient(q1, s, axis=0)
        self.validation_s, self.joints = np.asarray(s), np.asarray(joints)
        self.joint_rates, self.joint_accelerations = q1, q2
        self.set_duration(speed)

    def set_duration(self, speed):
        s = self.validation_s
        p1, p2 = self.position(s, 1), self.position(s, 2)
        w1, w2 = self.rotation(s, 1), self.rotation(s, 2)
        q1, q2 = self.joint_rates, self.joint_accelerations
        # Quintic progress has max u'=1.875 and max |u''|=10/sqrt(3).
        d1, d2 = 1.875 * self.length, 10 / np.sqrt(3) * self.length
        norm = lambda values: float(np.linalg.norm(values, axis=1).max())
        c = self.settings
        duration = max(
            d1 * norm(p1) / speed,
            np.sqrt((d1**2 * norm(p2) + d2 * norm(p1)) / c['linear_acceleration_mm_s2']),
            d1 * norm(w1) / np.radians(c['angular_speed_deg_s']),
            np.sqrt((d1**2 * norm(w2) + d2 * norm(w1)) / np.radians(c['angular_acceleration_deg_s2'])),
            d1 * float(np.abs(q1).max()) / np.radians(c['joint_speed_deg_s']),
            np.sqrt((d1**2 * float(np.abs(q2).max()) + d2 * float(np.abs(q1).max())) /
                    np.radians(c['joint_acceleration_deg_s2'])), .1)
        self.duration = 1.1 * duration

    def cache_data(self):
        return dict(knots=self.knots.tolist(), positions=self.checkpoints.tolist(),
                    rotations=self.rotation(self.knots).as_matrix().tolist(),
                    validation_s=self.validation_s.tolist(), joints=self.joints.tolist(),
                    joint_rates=self.joint_rates.tolist(), joint_accelerations=self.joint_accelerations.tolist(),
                    position_error_mm=self.max_position_error, angle_error_deg=self.max_angle_error_deg)

    @classmethod
    def from_cache(cls, data, settings, speed):
        span = cls.__new__(cls)
        span.settings = settings
        span.knots, span.checkpoints = np.asarray(data['knots']), np.asarray(data['positions'])
        span.length = float(span.knots[-1])
        span.position = cls.fit_position(span.knots, span.checkpoints, settings)
        span.rotation = RotationSpline(span.knots, Rotation.from_matrix(data['rotations']))
        span.validation_s, span.joints = np.asarray(data['validation_s']), np.asarray(data['joints'])
        span.joint_rates, span.joint_accelerations = np.asarray(data['joint_rates']), np.asarray(data['joint_accelerations'])
        span.max_position_error, span.max_angle_error_deg = data['position_error_mm'], data['angle_error_deg']
        span.set_duration(speed)
        return span

    def sample(self, elapsed):
        u = np.clip(elapsed / self.duration, 0., 1.)
        s = self.length * (10 * u**3 - 15 * u**4 + 6 * u**5)
        rate = self.length * (30 * u**2 - 60 * u**3 + 30 * u**4) / self.duration
        orientation = self.rotation(s)
        # RotationSpline rates are body-frame; the native interface expects BASE angular velocity.
        feed = np.r_[self.position(s, 1) * rate, orientation.apply(self.rotation(s, 1)) * rate]
        return self.position(s), orientation.as_matrix(), feed


class PoseTrajectory:
    def __init__(self, positions, rotations, settings, corners=None, checkpoints=()):
        positions, rotations = np.asarray(positions), np.asarray(rotations)
        distance = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        angles = (Rotation.from_matrix(rotations[1:]) * Rotation.from_matrix(rotations[:-1]).inv()).magnitude()
        keep = np.r_[True, (distance > 1e-8) | (angles > 1e-8)]
        original_indices = np.flatnonzero(keep)
        positions, rotations = positions[keep], rotations[keep]
        if len(positions) < 2:
            raise ValueError('A trajectory requires at least two different poses')
        cuts = [0, len(positions) - 1]
        if corners is not None:
            mapped = np.searchsorted(original_indices, corners)
            cuts += mapped[(mapped > 0) & (mapped < len(positions) - 1)].tolist()
        cuts = np.unique(cuts)
        forced = np.searchsorted(original_indices, np.asarray(checkpoints, int))
        self.spans = [PoseSpan(positions[a:b + 1], rotations[a:b + 1], settings, forced[(forced > a) & (forced < b)] - a)
                      for a, b in zip(cuts[:-1], cuts[1:]) if b > a]
        self.dense_count = len(positions)

    def validate(self, reach, seed, speed, jump_deg, geometry=None):
        q = np.asarray(seed).copy()
        for span in self.spans:
            s, positions, rotations = span.validation_samples()
            if geometry is not None and not geometry(positions, rotations):
                raise ValueError('Interpolated mesh path crosses an opening or exceeds its surface/normal tolerance')
            joints = []
            jacobians = []
            for position, rotation in zip(positions, rotations):
                following = reach.pose(position, rotation, q)
                if following is None or np.max(np.abs(following - q)) > np.radians(jump_deg):
                    raise ValueError(f'Unreachable or discontinuous mesh pose at {position.round(2).tolist()}')
                joints.append(following)
                jacobians.append(reach.chain.forward(following)[1])
                q = following
            span.retime(s, np.asarray(joints), speed, jacobians)
        self.set_duration()
        return q

    def set_duration(self):
        self.ends = np.cumsum([span.duration for span in self.spans])
        self.duration = float(self.ends[-1])

    def entry(self, reach, seed, speed, jump_deg):
        """Reuse the checked stroke when the approached robot is on its validated IK branch."""
        position, rotation, _ = self.sample(0.)
        q = reach.pose(position, rotation, seed)
        if q is None:
            raise ValueError('Mesh stroke entry pose is unreachable')
        if np.max(np.abs(q - self.spans[0].joints[0])) > np.radians(jump_deg):
            self.validate(reach, q, speed, jump_deg)

    def cache_data(self):
        return dict(dense_count=self.dense_count, spans=[span.cache_data() for span in self.spans])

    @classmethod
    def from_cache(cls, data, settings, speed):
        trajectory = cls.__new__(cls)
        trajectory.dense_count = data['dense_count']
        trajectory.spans = [PoseSpan.from_cache(span, settings, speed) for span in data['spans']]
        trajectory.set_duration()
        return trajectory

    def sample(self, elapsed):
        index = min(np.searchsorted(self.ends, elapsed, side='right'), len(self.spans) - 1)
        return self.spans[index].sample(elapsed - (self.ends[index - 1] if index else 0.))

    def export(self):
        return dict(duration_s=getattr(self, 'duration', None), dense_points=self.dense_count,
                    spans=[dict(positions_mm=span.checkpoints.tolist(),
                                rotations=span.rotation(span.knots).as_matrix().tolist(),
                                position_error_mm=span.max_position_error,
                                orientation_error_deg=span.max_angle_error_deg) for span in self.spans])
