"""Local arc-length timing: cruise at the requested ball speed, slow only near constraints. Speed changes are eased
exponentially: the speed is smoothed in time by a two-sided exponential kernel (acceleration_smoothing_s), so the
acceleration rises and falls as 1 - e^(-t/tau) instead of in steps. The limits are first eroded over the travel of
two time constants either way, where most of the kernel's weight lies, so the smoothed speed (a weighted mean of
nearby speeds) stays within the speed limits (bonnet plan: at most 0.9997 of them, the whole kernel's travel took
46 % longer instead of 13 %)."""
import numpy as np


class PathTiming:
    STEP_S = .002
    REACH = 4.  # kernel half-width in time constants

    def __init__(self, span, speed):
        s = np.unique(np.r_[span.validation_s, span.knots,
                            np.linspace(0., span.length, max(3, int(np.ceil(span.length / .25)) + 1))])
        s = s[np.r_[True, np.diff(s) > 1e-8]]
        s[-1] = span.length
        c = span.settings
        p1, p2 = span.position(s, 1), span.position(s, 2)
        w1, w2 = span.rotation(s, 1), span.rotation(s, 2)
        q1, q2 = [np.column_stack([np.interp(s, span.validation_s, values[:, k]) for k in range(6)])
                  for values in (span.joint_rates, span.joint_accelerations)]
        norm = lambda a: np.maximum(np.linalg.norm(a, axis=1), 1e-12)
        maximum = lambda a: np.maximum(np.abs(a).max(axis=1), 1e-12)
        first = np.array([norm(p1), norm(w1), maximum(q1)])
        second = np.array([norm(p2), norm(w2), maximum(q2)])
        velocities = .999 * np.array([speed, np.radians(c['angular_speed_deg_s']),
                                      np.radians(c['joint_speed_deg_s'])])[:, None]
        accelerations = np.array([c['linear_acceleration_mm_s2'],
                                  np.radians(c['angular_acceleration_deg_s2']),
                                  np.radians(c['joint_acceleration_deg_s2'])])[:, None]
        # Reserve acceleration for both curve bending and scalar speed changes.
        cap = np.minimum(velocities / first, np.sqrt(.75 * accelerations / second)).min(axis=0)
        # Bound each interval with its neighbours, including derivative peaks between grid points.
        cap = np.minimum.reduce([cap, np.r_[cap[0], cap[:-1]], np.r_[cap[1:], cap[-1]]])
        acceleration = (.25 * accelerations / first).min(axis=0)
        tau = c['acceleration_smoothing_s']
        rate = self.profile(s, cap, acceleration)
        window = 2 * tau * rate.max()
        low, high = np.searchsorted(s, s - window), np.searchsorted(s, s + window, 'right')
        erode = lambda values: np.array([values[a:b].min() for a, b in zip(low, high)])
        rate = self.profile(s, erode(cap), erode(acceleration))
        times = np.r_[0., np.cumsum(2 * np.diff(s) / (rate[:-1] + rate[1:]))]
        count = max(1, int(np.ceil(times[-1] / self.STEP_S)))
        step = times[-1] / count
        reach = int(np.ceil(self.REACH * tau / step))
        self.t = np.arange(count + 2 * reach + 1) * step
        kernel = np.exp(-np.abs(np.arange(-reach, reach + 1)) * step / tau)
        # At rest (rate 0) for one kernel reach before and after, so the smoothed speed starts and ends at zero.
        self.rate = np.convolve(np.interp(self.t - reach * step, times, rate), kernel / kernel.sum(), 'same')
        self.s = np.r_[0., np.cumsum((self.rate[1:] + self.rate[:-1]) / 2 * step)]
        scale = span.length / self.s[-1]
        self.s, self.rate = self.s * scale, self.rate * scale
        self.duration = float(self.t[-1])

    @staticmethod
    def profile(s, cap, acceleration):
        """Fastest rate within `cap` from rest to rest, at most `acceleration` per interval (forward/backward)."""
        rate, ds = cap.copy(), np.diff(s)
        limit = np.minimum(acceleration[:-1], acceleration[1:])
        rate[0] = rate[-1] = 0.
        for k in range(len(ds)):
            rate[k + 1] = min(rate[k + 1], np.sqrt(rate[k]**2 + 2 * limit[k] * ds[k]))
        for k in range(len(ds) - 1, -1, -1):
            rate[k] = min(rate[k], np.sqrt(rate[k + 1]**2 + 2 * limit[k] * ds[k]))
        return rate

    def sample(self, elapsed):
        return np.interp(elapsed, self.t, self.s), np.interp(elapsed, self.t, self.rate)
