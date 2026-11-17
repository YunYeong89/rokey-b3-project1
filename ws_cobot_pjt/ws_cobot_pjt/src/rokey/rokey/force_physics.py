"""Non-learning force calibration: least-squares identification of a physical error model of the
controller's TOOL wrench, then a raw-vs-calibrated stability plot on held-out free-space motions.

  error = theta^T [1, g_tool, tanh(qd/eps), qd, a_tool]   (bias, payload gravity/mounting, Coulomb,
                                                          viscous, payload inertia)

Models (none / static: bias+gravity / inertia: static+acceleration / friction: static+joint friction) are
fitted and the one with the lowest held-out Fn RMS is kept, so the calibration never makes the reading
worse than raw. Used by `ros2 run rokey calibration` (step 1) and `draw_graph --compensated`.
"""
import json

import numpy as np

if __package__:
    from .force_data import EPSILON, residual_stats
else:
    from force_data import EPSILON, residual_stats


class PhysicsCompensator:
    names = ['bias'] + [f'gravity_{axis}' for axis in 'xyz'] + \
            [f'coulomb_j{index}' for index in range(1, 7)] + [f'viscous_j{index}' for index in range(1, 7)] + \
            [f'inertia_{axis}' for axis in 'xyz']
    models = {'none': [], 'static': list(range(4)), 'inertia': [*range(4), 16, 17, 18], 'friction': list(range(16))}

    def __init__(self, theta=None):
        self.theta = theta

    @staticmethod
    def regressors(data):
        return np.column_stack([np.ones(len(data.t)), data.gravity, np.tanh(data.qd / EPSILON), data.qd,
                                data.acceleration])

    def fit(self, data, model='friction', ridge=1e-6):
        columns = self.models[model]
        phi = self.regressors(data)[:, columns]
        self.theta = np.zeros((len(self.names), data.wrench.shape[1]))
        if columns:
            self.theta[columns] = np.linalg.solve(phi.T @ phi + ridge * np.eye(len(columns)), phi.T @ data.wrench)
        return self

    def correct(self, data):
        return data.wrench - self.regressors(data)[:, :len(self.theta)] @ self.theta

    @classmethod
    def select(cls, train, test):
        """Every model fitted on `train`; the one with the lowest Fn RMS on `test`, all held-out scores."""
        fitted = {name: cls().fit(train, name) for name in cls.models}
        scores = {name: residual_stats(model.correct(test)) for name, model in fitted.items()}
        best = min(scores, key=lambda name: scores[name]['fn_rms'])
        return best, fitted[best], scores

    def save(self, path, meta):
        path.write_text(json.dumps({**meta, 'epsilon': EPSILON, 'regressors': self.names,
                                    'outputs': ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'],
                                    'theta': self.theta.tolist()}, indent=2))

    @classmethod
    def load(cls, path):
        return cls(np.asarray(json.loads(path.read_text())['theta']))
