"""Measured-state overload prediction; no collision geometry or true force inputs."""
from .geometry import limit


class ForceSupervisor:
    def __init__(self, settings):
        self.settings = settings
        self.fast = self.slope = self.predicted = self.risk = 0.0
        self.last_stamp = None

    def update(self, state):
        if self.last_stamp is None:
            self.last_stamp = state.sampled_at
        dt = state.sampled_at - self.last_stamp
        if dt > 1e-8:
            before = self.fast
            self.fast += dt / (.012 + dt) * (state.sensor.raw.length - self.fast)
            derivative = (self.fast - before) / dt
            self.slope += dt / (.04 + dt) * (derivative - self.slope)
            self.last_stamp = state.sampled_at
        s = self.settings
        horizon = s.feedback_delay + s.command_delay + s.servo_tau + 1 / s.control_hz
        self.predicted = self.fast + horizon * max(0, self.slope)
        self.risk = limit((self.predicted - s.target_force - .2) / max(.1, s.soft_force_margin - .2), 0, 1)

    @property
    def unload(self):
        s = self.settings
        return self.fast > s.target_force + s.soft_force_margin or self.predicted > s.target_force + s.soft_force_margin + .25
