"""Learning force calibration (after Shan & Pham, RA-L): an MLP ensemble pretrained on free-space excitation
data predicts the controller's TOOL wrench error; before each task a short calibration motion re-solves only
the last layer in closed form, regularized toward the pretrained weights.

Run inside the container with the session up and free space around the TCP:
  ros2 run rokey force_collect [--moves 60]           # pretraining data; repeat per payload / day
  ros2 run rokey force_learning_train [--mode virtual] # numpy training on all collected files, no robot
  ros2 run rokey force_learning_calibrate             # ~30 s calibration motion + held-out check
"""
import argparse
from datetime import datetime
import json

import numpy as np

if __package__:
    from .force_data import CALIBRATION_DIR, EPSILON, CalibrationRun, Excitation, ForceData, residual_stats
else:
    from force_data import CALIBRATION_DIR, EPSILON, CalibrationRun, Excitation, ForceData, residual_stats

DATA_DIR = CALIBRATION_DIR / 'force_data'


def inputs(data):
    return np.column_stack([np.sin(data.q), np.cos(data.q), data.qd, np.tanh(data.qd / EPSILON), data.gravity])


class MLPEnsemble:
    """tanh MLPs on standardized inputs/outputs; members differ by initialization (spread = uncertainty)."""

    def __init__(self, sizes=(27, 64, 64, 6), members=3, seed=0, weights=None, stats=None):
        rng = np.random.default_rng(seed)
        self.weights = weights or [[(rng.normal(0, np.sqrt(1 / a), (a, b)), np.zeros(b))
                                    for a, b in zip(sizes[:-1], sizes[1:])] for _ in range(members)]
        self.stats = stats  # x_mean, x_std, y_mean, y_std

    def normalize(self, x, y=None):
        z = (x - self.stats[0]) / self.stats[1]
        return z if y is None else (z, (y - self.stats[2]) / self.stats[3])

    @staticmethod
    def hidden(layers, z):
        for w, b in layers[:-1]:
            z = np.tanh(z @ w + b)
        return z

    def predict(self, x):
        """Mean and member spread in output units."""
        z = self.normalize(x)
        outputs = np.array([self.hidden(layers, z) @ layers[-1][0] + layers[-1][1] for layers in self.weights])
        outputs = outputs * self.stats[3] + self.stats[2]
        return outputs.mean(0), outputs.std(0)

    @staticmethod
    def gradients(layers, z, target):
        activations = [z]
        for w, b in layers[:-1]:
            activations.append(np.tanh(activations[-1] @ w + b))
        error = (activations[-1] @ layers[-1][0] + layers[-1][1] - target) / len(z)
        grads = []
        for index in range(len(layers) - 1, -1, -1):
            grads[:0] = [activations[index].T @ error, error.sum(0)]
            if index:
                error = (error @ layers[index][0].T) * (1 - activations[index] ** 2)
        return grads

    def train(self, x, y, epochs, batch=256, rate=1e-3, log=print):
        self.stats = (x.mean(0), x.std(0) + 1e-9, y.mean(0), y.std(0) + 1e-9)
        z, target = self.normalize(x, y)
        rng = np.random.default_rng(1)
        for member, layers in enumerate(self.weights):
            params = [array for layer in layers for array in layer]
            first, second, step = [np.zeros_like(p) for p in params], [np.zeros_like(p) for p in params], 0
            for epoch in range(epochs):
                order = rng.permutation(len(z))
                for start in range(0, len(z), batch):
                    index = order[start:start + batch]
                    step += 1
                    for p, g, m, v in zip(params, self.gradients(layers, z[index], target[index]), first, second):
                        m += 0.1 * (g - m)
                        v += 0.001 * (g * g - v)
                        p -= rate * (m / (1 - 0.9 ** step)) / (np.sqrt(v / (1 - 0.999 ** step)) + 1e-8)
                if (epoch + 1) % 10 == 0:
                    loss = np.mean((self.hidden(layers, z) @ layers[-1][0] + layers[-1][1] - target) ** 2)
                    log(f'member {member + 1}/{len(self.weights)} epoch {epoch + 1}: normalized MSE {loss:.4f}')

    def calibrate(self, x, y, ridge=30.0):
        """Closed-form last-layer update toward the calibration data, regularized toward the pretrained layer.
        A short motion mostly reveals offsets (payload, drift), so the bias is nearly free while the hidden
        feature weights are held strongly to the pretrained values."""
        z, target = self.normalize(x, y)
        for layers in self.weights:
            features = np.column_stack([self.hidden(layers, z), np.ones(len(z))])
            current = np.vstack([layers[-1][0], layers[-1][1]])
            penalty = np.diag(np.r_[np.full(features.shape[1] - 1, ridge), 1e-3])
            delta = np.linalg.solve(features.T @ features + penalty, features.T @ (target - features @ current))
            layers[-1] = ((current + delta)[:-1], (current + delta)[-1])

    def save(self, path, meta):
        arrays = {f'm{member}_l{index}_{kind}': value for member, layers in enumerate(self.weights)
                  for index, layer in enumerate(layers) for kind, value in zip('wb', layer)}
        np.savez(path, x_mean=self.stats[0], x_std=self.stats[1], y_mean=self.stats[2], y_std=self.stats[3],
                 meta=json.dumps(meta), **arrays)

    @classmethod
    def load(cls, path):
        data = np.load(path)
        members = len({key.split('_')[0] for key in data.files if key[0] == 'm' and key[1].isdigit()})
        depth = len({key.split('_')[1] for key in data.files if key.startswith('m0_')})
        weights = [[(data[f'm{m}_l{l}_w'], data[f'm{m}_l{l}_b']) for l in range(depth)] for m in range(members)]
        stats = (data['x_mean'], data['x_std'], data['y_mean'], data['y_std'])
        return cls(weights=weights, stats=stats), json.loads(str(data['meta']))


def collect_main():
    parser = argparse.ArgumentParser(description='Collect free-space pretraining data for force_learning_train')
    parser.add_argument('--moves', type=int, default=60, help='random moves after the static/sweep set')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--span', type=float, default=40.0, help='motion box half-size, mm')
    parser.add_argument('--lift', type=float, default=50.0, help='rise above home before moving, mm')
    parser.add_argument('--max-tilt', type=float, default=30.0, help='random tilt limit, deg')
    parser.add_argument('--compliance', action='store_true',
                        help='move under the contact-search compliance control (default: position control)')
    options = parser.parse_args()
    run = CalibrationRun('force_collect')
    try:
        excitation = Excitation(run.probe, run.recorder, options.lift, options.span, options.compliance)
        run.log('1/3 static payload poses')
        excitation.static_set()
        run.log('2/3 friction sweeps')
        excitation.sweeps()
        run.log(f'3/3 {options.moves} random moves')
        excitation.random(options.moves, np.random.default_rng(options.seed), options.max_tilt)
        data = run.recorder.take()
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        path = DATA_DIR / f'{run.mode}_{datetime.now():%Y%m%d_%H%M%S}.npz'
        np.savez(path, rows=data.rows, labels=data.labels, meta=json.dumps(run.meta))
        run.log(f'saved {len(data.t)} samples -> {path}')
        excitation.finish()
    finally:
        run.close()


def train_main():
    parser = argparse.ArgumentParser(description='Pretrain the force-error MLP ensemble on collected data')
    parser.add_argument('--mode', choices=['virtual', 'real'], default='virtual')
    parser.add_argument('--epochs', type=int, default=60)
    parser.add_argument('--members', type=int, default=3)
    options = parser.parse_args()
    files = sorted(DATA_DIR.glob(f'{options.mode}_*.npz'))
    if not files:
        parser.error(f'{DATA_DIR}/{options.mode}_*.npz 없음: 먼저 ros2 run rokey force_collect')
    train, test = [], []
    for data in map(ForceData.load, files):  # last 15% of each run is held out (time-contiguous)
        split = int(0.85 * len(data.t))
        train.append((inputs(data)[:split], data.wrench[:split]))
        test.append((inputs(data)[split:], data.wrench[split:]))
    x, y = (np.vstack(part) for part in zip(*train))
    x_test, y_test = (np.vstack(part) for part in zip(*test))
    print(f'{len(files)} files: {len(x)} train / {len(x_test)} held-out samples')
    model = MLPEnsemble((x.shape[1], 64, 64, y.shape[1]), options.members)
    model.train(x, y, options.epochs)
    before, after = residual_stats(y_test), residual_stats(y_test - model.predict(x_test)[0])
    path = CALIBRATION_DIR / f'force_learned_{options.mode}.npz'
    model.save(path, {'mode': options.mode, 'files': [file.name for file in files], 'samples': len(x),
                      'created': datetime.now().isoformat(timespec='seconds'), 'held_out': after})
    print(f'held-out Fn RMS {before["fn_rms"]:.3f} -> {after["fn_rms"]:.3f} N, '
          f'peak {before["fn_peak"]:.3f} -> {after["fn_peak"]:.3f} N; saved {path}')


def calibrate_main():
    parser = argparse.ArgumentParser(description='Short pre-task calibration of the pretrained force model')
    parser.add_argument('--speed', type=float, default=20.0, help='calibration sweep speed, mm/s')
    parser.add_argument('--span', type=float, default=20.0, help='calibration sweep half-length, mm')
    parser.add_argument('--lift', type=float, default=50.0, help='rise above home before moving, mm')
    parser.add_argument('--compliance', action='store_true',
                        help='move under the contact-search compliance control (default: position control)')
    options = parser.parse_args()
    run = CalibrationRun('force_learning_calibrate')
    try:
        path = CALIBRATION_DIR / f'force_learned_{run.mode}.npz'
        model, meta = MLPEnsemble.load(path)
        excitation = Excitation(run.probe, run.recorder, options.lift, options.span, options.compliance)
        run.log('1/3 short calibration motion')
        excitation.static_set(tilts=(20.0,), azimuths=(0, 120, 240), hold=0.5)
        excitation.sweeps((options.speed,))
        calibration = run.recorder.take()
        model.calibrate(inputs(calibration), calibration.wrench)
        output = CALIBRATION_DIR / f'force_learned_{run.mode}_calibrated.npz'
        model.save(output, {**meta, **run.meta, 'calibration_samples': len(calibration.t)})
        run.log(f'2/3 last layer re-solved on {len(calibration.t)} samples -> {output}')
        excitation.validation()
        test = run.recorder.take()
        x = inputs(test)
        raw = residual_stats(test.wrench)
        pretrained = residual_stats(test.wrench - MLPEnsemble.load(path)[0].predict(x)[0])
        calibrated = residual_stats(test.wrench - model.predict(x)[0])
        run.log(f'3/3 held-out {len(test.t)} samples, Fn RMS / peak [N]: raw {raw["fn_rms"]:.3f} / '
                f'{raw["fn_peak"]:.3f}, pretrained {pretrained["fn_rms"]:.3f} / {pretrained["fn_peak"]:.3f}, '
                f'calibrated {calibrated["fn_rms"]:.3f} / {calibrated["fn_peak"]:.3f}')
        excitation.finish()
    finally:
        run.close()
