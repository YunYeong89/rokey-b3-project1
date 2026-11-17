"""Detect a stationary TCP with a persistent position error, without using force feedback."""
from collections import deque

import numpy as np


class ProgressWatch:
    def __init__(self, timeout, motion_mm, error_mm):
        self.timeout, self.motion, self.error = timeout, motion_mm, error_mm
        self.samples = deque()

    def blocked(self, now, position, error):
        self.samples.append((now, np.asarray(position).copy()))
        while len(self.samples) > 1 and now - self.samples[1][0] >= self.timeout:
            self.samples.popleft()
        first_time, first_position = self.samples[0]
        return (now - first_time >= self.timeout and error > self.error and
                np.linalg.norm(position - first_position) < self.motion)
