"""Sampled sensor / finite-rate delayed communication / held actuator commands.

All deadlines use simulation time. Clock rates need not divide the physics rate.
The model is a Cartesian robot surrogate, not an identified M0609 firmware model.
"""
from collections import deque
from dataclasses import dataclass
from random import Random

from pymunk import Vec2d

from .geometry import angle_error, limit
from .physics import Command, Sensor


@dataclass
class State:
    sampled_at: float
    position: Vec2d
    velocity: Vec2d
    angle: float
    sensor: Sensor
    ready: bool
    angular_velocity: float = 0.0


class DelayLine:
    def __init__(self, seed):
        self.random = Random(seed)
        self.pending = deque()

    def send(self, now, value, delay, jitter=0.0, dropout=0.0):
        if self.random.random() < dropout:
            return
        arrival = now + max(0, delay + self.random.uniform(-jitter, jitter))
        if self.pending:
            arrival = max(arrival, self.pending[-1][0])  # Preserve a stream's ordering.
        self.pending.append((arrival, value))

    def receive(self, now):
        result = None
        while self.pending and self.pending[0][0] <= now + 1e-10:
            _, result = self.pending.popleft()
        return result


class SensorModel:
    """Residual wrist-force error after nominal gravity compensation, with free-space tare."""
    def __init__(self, settings):
        self.settings = settings
        self.random = Random(7)
        self.filtered = self.bias = Vec2d(0, 0)
        self.tare = []
        self.ready = False
        self.last_velocity = Vec2d(0, 0)

    def sample(self, now, dt, physics):
        s = self.settings
        truth = physics.sample(dt)
        acceleration = (physics.body.velocity - self.last_velocity) / dt
        self.last_velocity = Vec2d(*physics.body.velocity)
        measurement = (truth.raw + Vec2d(s.sensor_bias_x, s.sensor_bias_y).rotated(physics.body.angle)
                       + Vec2d(0, s.bias_drift_y * now)
                       + s.inertia_error_mass * acceleration
                       + Vec2d(self.random.gauss(0, s.noise), self.random.gauss(0, s.noise)))
        if not self.ready:
            self.tare.append(measurement)
            self.bias = sum(self.tare, Vec2d(0, 0)) / len(self.tare)
            self.ready = now >= s.tare_seconds
        raw = measurement - self.bias
        self.filtered += dt / (s.sensor_tau + dt) * (raw - self.filtered)
        # Contact location and shaft collision identities are deliberately NOT sent to control.
        sensor = Sensor(raw, self.filtered, truth.torque, 0, None)
        return State(now, Vec2d(*physics.body.position), Vec2d(*physics.body.velocity),
                     physics.body.angle, sensor, self.ready, physics.body.angular_velocity)


class RobotDriver:
    """Command delay, expiry, 100 Hz reference updates and finite motion limits."""
    def __init__(self, settings, position):
        self.settings = settings
        self.line = DelayLine(31)
        self.target = Command(Vec2d(*position), Vec2d(0, 0), Vec2d(0, 1), 0)
        self.command = self.target
        self.last_command = 0.0
        self.reference = Vec2d(*position)
        self.velocity = Vec2d(0, 0)
        self.angle = self.omega = 0.0
        self.force = 0.0
        self.braking = False
        self.expired = False

    def submit(self, now, command):
        s = self.settings
        self.line.send(now, (now, command), s.command_delay, s.jitter)

    def brake(self, now, physics):
        self.line.pending.clear()
        self.reference = Vec2d(*physics.body.position)
        self.angle = physics.body.angle
        self.velocity = Vec2d(0, 0)
        self.omega = self.force = 0.0
        self.target = Command(self.reference, self.velocity, physics.axis, self.angle)
        self.command = self.target
        self.last_command = now
        self.braking = True

    def resume(self, now, physics):
        self.brake(now, physics)
        self.braking = self.expired = False

    def update(self, now, dt, physics):
        s = self.settings
        packet = self.line.receive(now)
        if packet is not None and not self.braking:
            self.last_command, self.target = packet
        if not self.braking and now - self.last_command > s.command_timeout:
            self.expired = True
            self.brake(now, physics)
        if self.braking:
            return self.command
        if s.preset == 'ideal':
            self.command = self.target
            return self.command
        wanted = self.target.velocity + 20 * (self.target.position - self.reference)
        if wanted.length > s.max_linear_speed:
            wanted *= s.max_linear_speed / wanted.length
        change = wanted - self.velocity
        if change.length > s.max_acceleration * dt:
            change *= s.max_acceleration * dt / change.length
        self.velocity += change
        self.reference += self.velocity * dt
        # A reference servo, not error/dt: the latter chatters under delayed/noisy setpoints
        # once angular acceleration limits are added.
        wanted_omega = limit(8 * angle_error(self.target.angle, self.angle),
                             -s.max_angular_speed, s.max_angular_speed)
        self.omega += limit(wanted_omega - self.omega,
                            -s.max_angular_acceleration * dt, s.max_angular_acceleration * dt)
        self.angle += self.omega * dt
        target_force = self.target.normal_force
        if target_force is not None:
            self.force += limit(target_force - self.force, -s.force_slew * dt, s.force_slew * dt)
        self.command = Command(self.reference, self.velocity, self.target.normal,
                               self.angle, self.omega, self.force if target_force is not None else None)
        return self.command
