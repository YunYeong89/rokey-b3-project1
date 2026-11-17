"""Joint setpoints to Gazebo at about the physics rate, ramped over one control period."""
import threading
import time


class SetpointRamp:
    """Gazebo's JointPositionController differentiates the position error, so a setpoint stepping every 10 ms
    kicked each joint with d_gain * dq / (2 ms physics step) for one step: re-aiming the 270 mm probe about its
    tip gave ~30 N tip impulses 100 times a second. Each control-rate setpoint is ramped over the next control
    period (one period of delay) and published at about the physics rate."""

    def __init__(self, publish, period, rate=500.):
        self.publish, self.period, self.interval = publish, period, 1 / rate
        self.start, self.end, self.time = {}, {}, 0.
        self.lock = threading.Lock()
        self.closed = threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def value(self, now):
        alpha = min(1., (now - self.time) / self.period)
        return {name: self.start.get(name, end) + alpha * (end - self.start.get(name, end))
                for name, end in self.end.items()}

    def set(self, positions):
        now = time.monotonic()
        with self.lock:
            self.start, self.end, self.time = self.value(now), dict(positions), now

    def run(self):
        while not self.closed.wait(self.interval):
            with self.lock:
                positions = self.value(time.monotonic())
            if positions:
                self.publish(positions)

    def close(self):
        self.closed.set()
