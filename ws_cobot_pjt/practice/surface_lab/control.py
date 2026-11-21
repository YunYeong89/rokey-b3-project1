"""Delayed-state surface following, bounded loading and measured-force overload recovery."""
from math import cos, radians, sqrt

from pymunk import Vec2d

from .geometry import NormalObserver, angle_error, limit, normal_angle
from .physics import Command
from .safety import ForceSupervisor


class SurfaceFollower:
    def __init__(self, settings, position):
        self.settings = settings
        self.observer = NormalObserver()
        self.supervisor = ForceSupervisor(settings)
        self.reference = Vec2d(*position)
        self.phase, self.reason = 'TARE', ''
        self.phase_time = self.stable_time = self.lost_time = 0.0
        self.force = self.active_target = self.feed = self.normal_speed = 0.0
        self.angle_reference = self.integral = 0.0
        self.ramp_start = 0.0
        self.recovery_start = Vec2d(*position)
        self.unload_normal = Vec2d(0, 1)
        self.unload_events = 0
        self.normal_saturated = False
        self.normal_measurement = Vec2d(0, 0)
        self.normal_stamp = 0.0
        self.moment_residual = self.moment_bad_time = 0.0
        self.speed_limit = settings.feed
        self.last_unload_position = None
        self.failed_recoveries = 0

    @property
    def normal(self):
        return self.observer.normal

    def transition(self, phase, position, keep_reference=False):
        self.phase, self.phase_time, self.stable_time, self.lost_time = phase, 0.0, 0.0, 0.0
        if not keep_reference:
            self.reference = Vec2d(*position)
        self.normal_speed = self.integral = 0.0
        self.recovery_start = Vec2d(*position)
        if phase == 'SETTLE':
            self.ramp_start = limit(self.force, 0, self.settings.target_force)
            self.active_target = self.ramp_start
        if phase == 'UNLOAD':
            self.unload_normal = self.normal
            self.unload_events += 1
            if self.last_unload_position is not None and (position - self.last_unload_position).length < .001:
                self.failed_recoveries += 1
            else:
                self.failed_recoveries = 0
            self.last_unload_position = Vec2d(*position)

    def _position_command(self, position, velocity, normal, angle, omega):
        error = self.reference - position
        tangent = Vec2d(normal.y, -normal.x)
        s = self.settings
        en = limit(error.dot(normal), -s.max_reference_normal, s.max_reference_normal)
        et = limit(error.dot(tangent), -s.max_reference_tangent, s.max_reference_tangent)
        self.reference = position + et * tangent + en * normal
        return Command(self.reference, velocity, normal, angle, omega)

    def update(self, state, dt):
        s = self.settings
        position, velocity, angle, sensor = state.position, state.velocity, state.angle, state.sensor
        self.phase_time += dt
        sample_dt = state.sampled_at - self.normal_stamp
        if sample_dt > 1e-8:
            self.normal_measurement += sample_dt / (.015 + sample_dt) * (sensor.raw - self.normal_measurement)
            self.normal_stamp = state.sampled_at
            if self.phase not in ('TARE', 'RECOVER'):
                self.observer.update(position, self.normal_measurement, sample_dt)
        n = self.normal
        self.force = sensor.filtered.dot(n)
        self.supervisor.update(state)
        tangent = Vec2d(n.y, -n.x)
        tool_axis = Vec2d(0, 1).rotated(angle)
        expected_moment = (-s.shaft_length * tool_axis - s.tip_radius * n).cross(sensor.raw)
        self.moment_residual = sensor.torque - expected_moment
        inconsistent = sensor.raw.length > .5 and abs(self.moment_residual) > s.max_moment_residual
        self.moment_bad_time = self.moment_bad_time + dt if inconsistent else 0
        if self.moment_bad_time > .08:
            self.phase, self.reason = 'STOPPED', 'Wrench inconsistent with tip contact (possible shaft contact).'
            return Command(position, Vec2d(0, 0), n, angle)
        turn = limit(angle_error(normal_angle(n), self.angle_reference),
                     -s.max_angular_speed * dt, s.max_angular_speed * dt)
        self.angle_reference += turn
        if self.phase == 'TARE':
            if state.ready:
                self.transition('SEARCH', position)
            else:
                return Command(self.reference, Vec2d(0, 0), n, self.angle_reference)
        if self.phase == 'SEARCH':
            self.active_target = self.feed = 0.0
            if sensor.raw.dot(n) > .3:
                self.transition('CONFIRM', position, keep_reference=True)
                # Preserve a small compressive reference while confirming contact at rest.
                en = limit((self.reference - position).dot(n), -.00035, 0)
                self.reference = position + en * n
            elif self.phase_time > 12 or (position - self.recovery_start).length > .025:
                self.phase, self.reason = 'STOPPED', 'Contact search exceeded its time / travel limit.'
            else:
                v = Vec2d(0, -s.search_speed)
                self.reference += v * dt
                return self._position_command(position, v, n, self.angle_reference, turn / dt)
        if self.phase == 'CONFIRM':
            self.feed = 0.0
            if self.phase_time > .15:
                if sensor.raw.dot(n) > .2:
                    self.transition('SETTLE', position)
                else:
                    self.transition('SEARCH', position)
            else:
                return Command(self.reference, Vec2d(0, 0), n, self.angle_reference, turn / dt)
        if s.adaptive and self.phase in ('SETTLE', 'TRACK') and self.supervisor.unload:
            self.transition('UNLOAD', position)
            if self.failed_recoveries >= 3:
                self.phase, self.reason = 'STOPPED', 'Repeated overload without 1 mm progress; lower gains / speed.'
                return Command(position, Vec2d(0, 0), n, angle)
        if self.phase == 'UNLOAD':
            self.feed = self.active_target = 0.0
            self.normal_speed = self.integral = 0.0
            v = .003 * self.unload_normal
            self.reference += v * dt
            self.stable_time = self.stable_time + dt if self.supervisor.fast < .8 * s.target_force else 0
            if self.stable_time > .15:
                self.transition('RECOVER', position)
            elif self.phase_time > 2.0:
                self.phase, self.reason = 'STOPPED', 'Overload did not clear during bounded unloading.'
            return self._position_command(position, v, self.unload_normal, self.angle_reference, 0)
        if self.phase == 'RECOVER':
            self.feed = self.active_target = 0.0
            if sensor.raw.dot(n) > .3:
                self.transition('CONFIRM', position, keep_reference=True)
            elif self.phase_time > 3 or (position - self.recovery_start).length > .006:
                self.phase, self.reason = 'STOPPED', 'Bounded contact recovery failed.'
            v = -.0015 * n
            self.reference += v * dt
            return self._position_command(position, v, n, self.angle_reference, 0)
        if self.phase == 'SETTLE':
            in_band = abs(self.force - s.target_force) < .25
            self.stable_time = self.stable_time + dt if in_band else 0
            if self.stable_time > .35:
                self.transition('TRACK', position, keep_reference=True)
            elif self.phase_time > 10:
                self.phase, self.reason = 'STOPPED', 'Force did not settle; inspect timing / gains.'
        elif self.phase == 'TRACK':
            self.lost_time = self.lost_time + dt if self.force < .15 else 0
            if self.lost_time > .2:
                self.transition('RECOVER', position)
        target = s.target_force
        if self.phase == 'SETTLE':
            target = self.ramp_start + min(1, self.phase_time / .8) * (target - self.ramp_start)
        self.active_target += limit(target - self.active_target, -.8 * dt, .8 * dt)
        alignment = abs(angle_error(normal_angle(n), angle))
        scale = max(0, cos(alignment)) ** 2
        if s.adaptive:
            scale *= (1 - self.supervisor.risk)
            scale *= limit((radians(18) - alignment) / radians(12), 0, 1)
        self.speed_limit = s.feed
        if s.adaptive:
            # Bound worst-case inward travel while the command is in flight and the axis brakes.
            horizon = s.feedback_delay + s.command_delay + s.servo_tau + 1 / s.control_hz + 2 * s.jitter
            budget_n = max(.05, s.target_force + s.soft_force_margin - self.supervisor.fast)
            distance = budget_n / max(s.contact_stiffness_bound, s.position_stiffness)
            a = min(s.braking_deceleration, s.max_acceleration)
            self.speed_limit = min(s.feed, a * (sqrt(horizon * horizon + 2 * distance / a) - horizon))
        wanted_feed = self.speed_limit * scale if self.phase == 'TRACK' else 0.0
        self.feed += limit(wanted_feed - self.feed, -s.max_acceleration * dt, s.max_acceleration * dt)
        self.reference += self.feed * tangent * dt
        force_command, normal_velocity = self.normal_command(dt)
        error = self.reference - position
        et = limit(error.dot(tangent), -s.max_reference_tangent, s.max_reference_tangent)
        raw_en = error.dot(n)
        en = limit(raw_en, -s.max_reference_normal, s.max_reference_normal)
        self.normal_saturated = abs(raw_en) > s.max_reference_normal
        if force_command is not None:
            en = 0.0
        elif self.normal_saturated and en * normal_velocity > 0:
            normal_velocity = self.normal_speed = 0.0
        self.reference = position + et * tangent + en * n + normal_velocity * n * dt
        return Command(self.reference, self.feed * tangent + normal_velocity * n,
                       n, self.angle_reference, turn / dt, force_command)


class HybridFollower(SurfaceFollower):
    """Surrogate normal force servo; these are not identified Doosan internal gains."""
    def normal_command(self, dt):
        s = self.settings
        error = self.active_target - self.force
        candidate = limit(self.integral + error * dt, -1, 1)
        demand = self.active_target + s.hybrid_kp * error + s.hybrid_ki * candidate
        clipped = limit(demand, 0, min(5, s.target_force + s.soft_force_margin))
        if demand == clipped or error * (demand - clipped) < 0:
            self.integral = candidate
        return clipped, 0.0


class AdmittanceFollower(SurfaceFollower):
    """Implicit-Euler admittance feeding a finite-rate position servo."""
    def normal_command(self, dt):
        s = self.settings
        error = self.active_target - self.force
        self.normal_speed = limit((s.admittance_mass * self.normal_speed + dt * error)
                                  / (s.admittance_mass + s.admittance_damping * dt),
                                  -s.max_normal_speed, s.max_normal_speed)
        return None, -self.normal_speed
