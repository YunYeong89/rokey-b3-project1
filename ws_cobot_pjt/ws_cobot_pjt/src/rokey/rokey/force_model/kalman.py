"""Kalman layer after the physics model (force_model/model.py), per BASE force axis. The model's residual is
  r = W - dW(q, qd, qdd, memory) = F + b + v
with two random-walk states: the contact force F (fast, force_rate_n2_s) and the reading bias b the model leaves
(slow drift, bias_rate_n2_s), v the reading noise (noise_n). Where contact is known absent (free: approach, retract,
the still reading before a search; in a free-space recording its holds and rests) a pseudo-measurement F = 0 (free_n)
pins the force, so the bias takes the residual; elsewhere (possibly in contact) the bias is held and the force follows
r - b, smoothed against the noise. Without the free information F and b could not be told apart in contact."""
import numpy as np


class ForceKalman:
    def __init__(self, settings, axes=3):
        self.rates = np.array([settings['force_rate_n2_s'], settings['bias_rate_n2_s']])
        self.noise, self.free = settings['noise_n'] ** 2, settings['free_n'] ** 2
        self.x = np.zeros((axes, 2))  # [F, b] per axis
        self.p = np.tile(np.diag([settings['initial_n'] ** 2] * 2), (axes, 1, 1))
        self.t = None

    def update(self, measured, h, variance):
        s = np.einsum('i,aij,j->a', h, self.p, h) + variance
        gain = np.einsum('aij,j->ai', self.p, h) / s[:, None]
        self.x += gain * (measured - self.x @ h)[:, None]
        self.p -= np.einsum('ai,j,ajk->aik', gain, h, self.p)

    def __call__(self, t, residual, free):
        """One sample: time (s), the model residual per axis (N), whether contact is known absent -> (F, b) per axis."""
        dt = 0. if self.t is None else max(t - self.t, 0.)
        self.t = t
        self.p[:, 0, 0] += self.rates[0] * dt
        measured = np.asarray(residual, float)
        force_only = np.array([1., 0.])
        if free:
            self.p[:, 1, 1] += self.rates[1] * dt
            self.update(measured, np.array([1., 1.]), self.noise)
            self.update(np.zeros(len(self.x)), force_only, self.free)
        else:
            # Treat the last free-space bias as a fixed offset while contact is possible.
            self.p[:, 0, 1] = self.p[:, 1, 0] = 0.
            self.update(measured - self.x[:, 1], force_only, self.noise)
        return self.x[:, 0].copy(), self.x[:, 1].copy()

    @classmethod
    def run(cls, settings, t, residual, free):
        """The filter over a recording -> (N, axes) force and bias estimates."""
        kalman = cls(settings, residual.shape[1])
        out = [kalman(*sample) for sample in zip(t, residual, free)]
        return np.array([force for force, _ in out]), np.array([bias for _, bias in out])
