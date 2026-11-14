"""Delay and interpolate display samples without changing robot commands."""
from collections import deque


class JointInterpolator:
    def __init__(self, delay=0.15):
        self.delay = delay
        self.samples = {}

    def receive(self, positions, timestamp):
        for name, value in positions.items():
            history = self.samples.setdefault(name, deque(maxlen=32))
            if history and abs(history[-1][1] - value) < 1e-9:
                continue
            # A new move after a long idle must not interpolate across that idle.
            if history and timestamp - history[-1][0] > 0.25:
                history.append((timestamp - 0.1, history[-1][1]))
            history.append((timestamp, value))

    def snapshot(self, timestamp):
        target = timestamp - self.delay
        positions = {}
        for name, history in self.samples.items():
            while len(history) > 2 and history[1][0] <= target:
                history.popleft()
            if len(history) == 1 or target <= history[0][0]:
                positions[name] = history[0][1]
            elif target >= history[-1][0]:
                positions[name] = history[-1][1]
            else:
                for first, second in zip(history, list(history)[1:]):
                    if first[0] <= target <= second[0]:
                        alpha = (target - first[0]) / (second[0] - first[0])
                        positions[name] = first[1] + alpha * (second[1] - first[1])
                        break
        return positions
