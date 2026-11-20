"""Compare force model candidates with the raw baseline on held-out stations."""
import numpy as np

from .model import CANDIDATES, FRICTION, JointForceModel, low_pass


def rms(values):
    return float(np.sqrt(np.mean(values ** 2)))


def stationary(labels):
    return np.asarray([value == 'still' or ':hold ' in value or ':rest ' in value for value in labels], bool)


def axis_errors(wrench, still):
    """Axis-wise RMS and absolute P95 across all samples, moving samples and stationary samples when present."""
    force = wrench[:, :3]
    return np.asarray([(np.sqrt(np.mean(group ** 2, axis=0)), np.percentile(np.abs(group), 95, axis=0))
                       for group in (force, force[~still], force[still]) if len(group)])


def preserves_axes(corrected, raw, labels):
    """Check RMS and absolute P95 against the baseline on every force axis."""
    still = stationary(labels)
    return bool(np.all(axis_errors(corrected, still) <= axis_errors(raw, still) + 1e-9))


def label(result, separator='\n'):
    return (result['name'] + ('' if result['epsilon'] is None else f'{separator}eps {result["epsilon"]:g}')
            + ('' if result['memory'] is None else f'{separator}T {result["memory"]:g} s'))


class Selection:
    def __init__(self, full, chain, settings):
        self.full, self.chain, self.settings = full, chain, settings
        stride = max(1, int(np.ceil(len(full.t) / settings['max_fit_samples'])))
        self.keep = np.arange(0, len(full.t), stride)
        self.samples = full.subset(self.keep)
        self.stations = sorted(set(self.samples.station))

    def held_out(self, model, regressors, usable, groups):
        """Leave-one-station-out predictions of the wrench error for every sample."""
        predicted = np.zeros_like(self.samples.wrench)
        for station in self.stations:
            own = self.samples.station == station
            train, test = ~own & usable, own & usable
            if not np.any(train) or not np.any(test):
                continue
            model.fit(regressors[train], self.samples.wrench[train], groups, self.settings['ridge'])
            predicted[test] = regressors[test] @ model.theta
        return predicted

    def raw_result(self):
        wrench = self.samples.wrench
        return dict(name='raw', epsilon=None, memory=None, groups=(), corrected=wrench, accepted=True,
                    force_rms=rms(np.linalg.norm(wrench[:, :3], axis=1)), fz_rms=rms(wrench[:, 2]),
                    model=JointForceModel(self.chain, moment_weight=self.settings['moment_weight_m'],
                                          weights=self.settings['wrench_weights']))

    def validate(self, best, samples):
        """Apply the candidate to independent data and fall back to raw if any axis regresses."""
        raw = samples.wrench
        if not best['groups']:
            return best, raw
        model = best['model']
        regressors, _ = model.design(samples, minimum_singular_value=self.settings['minimum_singular_value'])
        corrected = raw - regressors @ model.theta
        if preserves_axes(corrected, raw, samples.labels):
            return best, corrected
        return self.raw_result(), raw

    def run(self):
        """Select among the raw baseline and candidates that pass the axis-wise held-out error checks."""
        wrench, results = self.samples.wrench, []
        raw = best = self.raw_result()
        if len(self.stations) < 2:
            return raw, results, best
        still = stationary(self.samples.labels)
        baseline = axis_errors(wrench, still)
        epsilons, memories = self.settings['epsilons_rad_s'], self.settings['memory_s']
        minimum = self.settings['minimum_singular_value']
        basis, selected = None, None
        for epsilon in epsilons:
            for memory in memories:
                model = JointForceModel(self.chain, epsilon, memory, self.settings['moment_weight_m'],
                                        weights=self.settings['wrench_weights'])
                regressors, singular = model.design(self.samples, low_pass(
                    self.full.t, model.sign(self.full.qd), memory)[self.keep], basis=basis,
                    minimum_singular_value=minimum)
                if basis is None:
                    basis = regressors, singular
                usable = singular >= minimum
                for name, groups in CANDIDATES.items():
                    if (memory != memories[0] and 'memory' not in groups or
                            epsilon != epsilons[0] and not FRICTION & set(groups)):
                        continue  # the width and the time constant only change their own terms
                    corrected = wrench - self.held_out(model, regressors, usable, groups)
                    accepted = bool(np.all(axis_errors(corrected, still) <= baseline + 1e-9))
                    result = dict(name=name, epsilon=epsilon if FRICTION & set(groups) else None,
                                  memory=memory if 'memory' in groups else None, groups=groups,
                                  corrected=corrected, force_rms=rms(np.linalg.norm(corrected[:, :3], axis=1)),
                                  fz_rms=rms(corrected[:, 2]), accepted=accepted)
                    results.append(result)
                    if accepted and result['force_rms'] < best['force_rms']:
                        best, selected = result, (model, regressors, usable)
        if selected is not None:
            model, regressors, usable = selected
            best['model'] = model.fit(regressors[usable], wrench[usable], best['groups'], self.settings['ridge'])
        return raw, results, best
