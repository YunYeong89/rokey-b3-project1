"""Physical model (no learning) of the controller's external-wrench estimate error. The controller turns joint torques
into the TCP wrench, W = J(q)^-T tau, so its errors are joint-torque errors seen through J^-T (amplified near full
reach and singularities, as over the bonnet). Per joint j and body k (the links after joint k, the tool on link 6):
  dtau_j = d_j + c_j s_j + m_j |s_j| + b_j qd_j                 offset, Coulomb, moving, viscous (s_j = tanh(qd_j/eps))
         + h_j r_j                                              friction memory: r_j = s_j low-passed (time constant T)
         + i_j qdd_j                                            joint inertia error
         + sum_{k>=j} z_j . [(o_k - o_j) x g dm_k + (R_k dh_k) x g]   link / tool gravity model error (body k)
  dW = J(q)^-T dtau - [a; 0] dm_tool                            + tool inertia error
(o, z, R: joint origins, axes and body rotations, BASE m; a: TCP acceleration). The memory is what a stop leaves: the
reading settled over ~0.3-1 s after each stop and kept the last travel direction (the frozen pre-sliding friction of
the joints), which the instantaneous terms cannot hold. Linear in the 61 parameters, fitted in wrench space (ridge least
squares on standardized columns, moments weighted by moment_weight_m to N) so that the fit optimizes the TCP force;
samples with J's smallest singular value below minimum_singular_value are left out of fits. The corrected reading is
W - dW; OffsetStream keeps the memory state on a live joint-state stream."""
import json

import numpy as np

from ..rotations import axis_angle, rotation_zyz

GRAVITY = np.array([0., 0., -9.81])
GROUPS = {'offset': slice(0, 6), 'coulomb': slice(6, 12), 'moving': slice(12, 18), 'viscous': slice(18, 24),
          'gravity': slice(24, 48), 'memory': slice(48, 54), 'joint_inertia': slice(54, 60), 'inertia': slice(60, 61)}
PARAMETERS = 61
# Term sets compared by force_model.fit / view (held out), and the terms the Coulomb width changes.
FULL = ('offset', 'gravity', 'coulomb', 'moving', 'viscous', 'memory', 'joint_inertia', 'inertia')
CANDIDATES = {
    'offset': ('offset',),
    'no gravity': tuple(group for group in FULL if group != 'gravity'),  # the gravity terms' share, against full
    'offset+gravity': ('offset', 'gravity'),
    '+coulomb': FULL[:4],
    '+viscous': FULL[:5],
    '+memory': FULL[:6],
    'without coulomb': tuple(group for group in FULL if group != 'coulomb'),
    'full (+inertia)': FULL,
}
FRICTION = {'coulomb', 'moving', 'memory'}  # the terms the Coulomb width changes
# Per-joint excitation (RMS over the data, before J^-T) below which a joint's terms are left out of fits: the sign-like
# terms of a joint that hardly moved, its viscous term (rad/s) and its inertia term (rad/s^2).
EXCITATION = {'coulomb': .05, 'moving': .05, 'memory': .05, 'viscous': 1e-3, 'joint_inertia': 1e-3}


def skew(v):
    return np.array([[0., -v[2], v[1]], [v[2], 0., -v[0]], [-v[1], v[0], 0.]])


def smooth(values, window=5):
    kernel = np.ones(window) / window
    return np.column_stack([np.convolve(column, kernel, mode='same') for column in values.T])


def low_pass(t, values, tau):
    """First-order low-pass of the rows of `values` over time `t` (exact for a held input; restarted over a gap of more
    than 5 s or a clock reset, as between recordings)."""
    out, state, previous = np.zeros_like(values), values[0], t[0]
    for k, (now, value) in enumerate(zip(t, values)):
        dt = now - previous
        state = state + (1. - np.exp(-dt / tau)) * (value - state) if 0. <= dt < 5. else value
        out[k], previous = state, now
    return out


class ForceSamples:
    """Rows recorded by force_data.ForceRecorder (t, q, qd rad, BASE pose mm/deg, TOOL wrench) as BASE arrays: the
    wrench (N, N m), TCP and joint accelerations (m/s^2, rad/s^2, smoothed within each motion), the station of each row."""

    def __init__(self, rows, labels, *, wrench_in_base=False, tcp_velocity_mm_s=None):
        rows = np.asarray(rows, float)
        self.t, self.q, self.qd, self.pose = rows[:, 0], rows[:, 1:7], rows[:, 7:13], rows[:, 13:19]
        self.wrench = rows[:, 19:25]
        if not wrench_in_base:
            rotations = np.array([rotation_zyz(pose[3:]) for pose in self.pose])
            self.wrench = np.c_[np.einsum('nij,nj->ni', rotations, rows[:, 19:22]),
                                np.einsum('nij,nj->ni', rotations, rows[:, 22:25])]
        if tcp_velocity_mm_s is not None:
            tcp_velocity_mm_s = np.asarray(tcp_velocity_mm_s, float)
        self.labels = np.asarray(labels)
        self.station = np.array([label.split(':')[0] for label in self.labels])
        self.acceleration, self.qdd = np.zeros((len(rows), 3)), np.zeros((len(rows), 6))
        cuts = np.flatnonzero((self.labels[1:] != self.labels[:-1]) | (np.diff(self.t) > .1)) + 1
        for k in np.split(np.arange(len(rows)), cuts):
            if len(k) > 10:
                velocity = (smooth(np.gradient(self.pose[k, :3] / 1000, self.t[k], axis=0))
                            if tcp_velocity_mm_s is None else tcp_velocity_mm_s[k] / 1000)
                self.acceleration[k] = smooth(np.gradient(velocity, self.t[k], axis=0))
                self.qdd[k] = smooth(np.gradient(self.qd[k], self.t[k], axis=0))

    def subset(self, keep):
        samples = ForceSamples.__new__(ForceSamples)
        for name in ('t', 'q', 'qd', 'qdd', 'pose', 'wrench', 'labels', 'station', 'acceleration'):
            setattr(samples, name, getattr(self, name)[keep])
        return samples


class JointForceModel:
    def __init__(self, chain, epsilon=.005, memory=.3, moment_weight=10., theta=None, weights=None):
        self.chain, self.epsilon, self.memory, self.moment_weight = chain, epsilon, memory, moment_weight
        # Fit weights of BASE Fx Fy Fz Mx My Mz (by default forces 1 and moments moment_weight).
        self.weights = np.r_[1., 1., 1., [moment_weight] * 3] if weights is None else np.asarray(weights, float)
        self.theta = np.zeros(PARAMETERS) if theta is None else np.asarray(theta)
        self.excited = np.ones(PARAMETERS, bool)  # set by design()

    def sign(self, qd):
        return np.tanh(np.asarray(qd) / self.epsilon)

    def kinematics(self, q):
        """Per joint: its origin and axis (BASE m) and the rotation of the body after it; the TCP Jacobian."""
        transform, frames, index = np.eye(4), [], 0
        for fixed, axis, active in self.chain.joints:
            transform = transform @ fixed
            if active:
                origin, z = transform[:3, 3].copy(), transform[:3, :3] @ axis
                motion = np.eye(4)
                motion[:3, :3] = axis_angle(axis, q[index])
                transform = transform @ motion
                index += 1
                frames.append((origin, z, transform[:3, :3].copy()))
        return frames, self.chain.forward(q)[1]

    def regressor(self, q, qd, qdd, memory, acceleration, *, minimum_singular_value=0.):
        """BASE TCP wrench error regressor (6 x PARAMETERS) at one state (memory: the low-passed sign) and J's
        smallest singular value."""
        frames, jacobian = self.kinematics(q)
        singular = np.linalg.svd(jacobian, compute_uv=False)[-1]
        if singular <= np.finfo(float).eps or singular < minimum_singular_value:
            return np.zeros((6, PARAMETERS)), singular
        sign = self.sign(qd)
        gravity = np.zeros((6, 24))
        for k, (o_k, _, r_k) in enumerate(frames):
            moment = -skew(GRAVITY) @ r_k  # (R_k h) x g = -[g]x R_k h
            for j, (o_j, z_j, _) in enumerate(frames[:k + 1]):
                gravity[j, 4 * k] = z_j @ np.cross(o_k - o_j, GRAVITY)
                gravity[j, 4 * k + 1:4 * k + 4] = z_j @ moment
        torque = np.hstack([np.eye(6), np.diag(sign), np.diag(np.abs(sign)), np.diag(qd), gravity, np.diag(memory),
                            np.diag(qdd)])
        wrench = np.linalg.solve(jacobian.T, torque)
        return np.c_[wrench, np.r_[-np.asarray(acceleration), 0., 0., 0.]], singular

    def design(self, samples, memory=None, *, basis=None, minimum_singular_value=0.):
        """(N, 6, PARAMETERS) regressors and (N,) smallest singular values of J; the memory run in time order (or given:
        low-passed at the full rate before samples were thinned). Reuse the same geometry when only friction changes."""
        memory = low_pass(samples.t, self.sign(samples.qd), self.memory) if memory is None else memory
        sign = self.sign(samples.qd)
        level = dict(coulomb=sign, moving=np.abs(sign) - np.abs(sign).mean(0), memory=memory, viscous=samples.qd,
                     joint_inertia=samples.qdd)
        for group, threshold in EXCITATION.items():
            self.excited[GROUPS[group]] = np.sqrt(np.mean(level[group] ** 2, axis=0)) > threshold
        if basis is not None:
            rows, singular = basis
            regressors = rows.copy()
            inverse = rows[:, :, GROUPS['offset']]
            for group, values in [('coulomb', sign), ('moving', np.abs(sign)), ('memory', memory)]:
                regressors[:, :, GROUPS[group]] = inverse * values[:, None, :]
            return regressors, singular
        rows = [self.regressor(*state, minimum_singular_value=minimum_singular_value)
                for state in zip(samples.q, samples.qd, samples.qdd, memory, samples.acceleration)]
        return np.asarray([row[0] for row in rows]), np.asarray([row[1] for row in rows])

    def fit(self, regressors, wrench, groups, ridge):
        """Weighted ridge least squares over the parameters of `groups` (the others zero)."""
        columns = np.concatenate([np.arange(PARAMETERS)[GROUPS[group]] for group in groups])
        x = (regressors[:, :, columns] * self.weights[None, :, None]).reshape(-1, len(columns))
        y = (wrench * self.weights).reshape(-1)
        scale = np.sqrt(np.mean(x ** 2, axis=0))
        # A term barely excited stays zero (EXCITATION, and a column at round-off): a joint that hardly moved, only
        # along with the others, got coefficients fitting their collinear by-product, which blow up wherever it moves.
        excited = (scale > 1e-4 * scale.max()) & self.excited[columns]
        columns, x, scale = columns[excited], x[:, excited] / scale[excited], scale[excited]
        self.theta = np.zeros(PARAMETERS)
        self.theta[columns] = np.linalg.solve(x.T @ x + ridge * len(x) * np.eye(len(columns)), x.T @ y) / scale
        return self

    def save(self, path, meta):
        path.write_text(json.dumps({**meta, 'epsilon': self.epsilon, 'memory_s': self.memory,
                                    'moment_weight_m': self.moment_weight, 'wrench_weights': self.weights.tolist(),
                                    'groups': {name: [part.start, part.stop] for name, part in GROUPS.items()},
                                    'theta': self.theta.tolist()}, indent=2))

    @classmethod
    def load(cls, path, chain):
        data = json.loads(path.read_text())
        return cls(chain, data['epsilon'], data['memory_s'], data['moment_weight_m'], data['theta'],
                   data.get('wrench_weights')), data


class OffsetStream:
    """The model on a live joint-state stream: the friction memory carried from sample to sample."""

    def __init__(self, model):
        self.model, self.state, self.previous = model, None, None

    def __call__(self, t, q, qd, qdd, acceleration):
        """Predicted BASE wrench reading error (N, N m) at time t (s) and this joint state (rad, rad/s, rad/s^2)."""
        sign = self.model.sign(qd)
        dt = None if self.previous is None else t - self.previous
        self.state = (sign if dt is None or not 0. <= dt < 5. else
                      self.state + (1. - np.exp(-dt / self.model.memory)) * (sign - self.state))
        self.previous = t
        return self.model.regressor(q, qd, qdd, self.state, acceleration)[0] @ self.model.theta
