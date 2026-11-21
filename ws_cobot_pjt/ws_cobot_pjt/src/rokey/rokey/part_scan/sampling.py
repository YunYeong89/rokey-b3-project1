"""Spatial contact-cloud sampling, independent of force/pose telemetry and motion."""
import numpy as np


class PointSampler:
    def __init__(self, step_mm, decimation=1):
        self.step, self.decimation = step_mm, decimation
        self.last, self.candidates = None, 0

    def keep(self, point):
        if self.last is not None and np.linalg.norm(point - self.last) < self.step:
            return False
        self.last = np.asarray(point).copy()
        self.candidates += 1
        return (self.candidates - 1) % self.decimation == 0
