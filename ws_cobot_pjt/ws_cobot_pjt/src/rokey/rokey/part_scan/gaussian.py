"""Offline Gaussian smoothing on measured timestamps, independently for each contact path."""
import numpy as np


class GaussianSmoother:
    def __init__(self, sigma_s):
        self.sigma = float(sigma_s)

    def apply(self, times, values, paths):
        times, values, paths = np.asarray(times), np.asarray(values, float), np.asarray(paths)
        result = values.copy()
        if self.sigma <= 0:
            return result
        for path in np.unique(paths):
            at = np.flatnonzero(paths == path)
            t, own = times[at], values[at]
            if len(at) < 2:
                continue
            widths = np.gradient(t)
            for k, now in enumerate(t):
                left, right = np.searchsorted(t, [now - 4 * self.sigma, now + 4 * self.sigma])
                weights = np.exp(-.5 * ((t[left:right] - now) / self.sigma)**2) * widths[left:right]
                result[at[k]] = np.tensordot(weights / weights.sum(), own[left:right], axes=1)
        return result
