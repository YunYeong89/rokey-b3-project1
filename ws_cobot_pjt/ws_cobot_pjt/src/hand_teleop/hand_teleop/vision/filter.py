"""One Euro filter (Casiez et al. 2012): strong smoothing while the hand is still, little lag when it moves fast."""
import numpy as np


class OneEuroFilter:
    def __init__(self, min_cutoff=1.0, beta=0.5, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def reset(self):
        self.value = None

    @staticmethod
    def alpha(dt, cutoff):
        return 1.0 / (1.0 + 1.0 / (2 * np.pi * cutoff * dt))

    def __call__(self, value, now):
        value = np.asarray(value, dtype=float)
        if self.value is None:
            self.value, self.slope, self.time = value, np.zeros_like(value), now
            return value
        dt, self.time = now - self.time, now
        self.slope = self.slope + self.alpha(dt, self.d_cutoff) * ((value - self.value) / dt - self.slope)
        cutoff = self.min_cutoff + self.beta * np.abs(self.slope)
        self.value = self.value + self.alpha(dt, cutoff) * (value - self.value)
        return self.value
