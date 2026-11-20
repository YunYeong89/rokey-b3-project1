"""Rigid-body contact and a finite-stiffness robot actuator, implemented with Pymunk."""
from dataclasses import dataclass
from math import cos, sin

import pymunk
from pymunk import Vec2d

from .geometry import angle_error, limit


@dataclass
class Sensor:
    raw: Vec2d = Vec2d(0, 0)
    filtered: Vec2d = Vec2d(0, 0)
    torque: float = 0.0
    shaft_force: float = 0.0
    contact: Vec2d | None = None


@dataclass
class Command:
    position: Vec2d
    velocity: Vec2d
    normal: Vec2d
    angle: float
    angular_velocity: float = 0.0
    normal_force: float | None = None


class ProbePhysics:
    """SI world, gravity compensated robot actuator, colliding spherical tip and slender shaft.

    Mass/inertia are lumped at the tip reference. The force sensor is an ideal external
    contact-wrench sensor: total collision impulse / observation interval, not a terrain query.
    The separate I/O model adds residual wrist-sensor error and sampled communication.
    """

    def __init__(self, settings, terrain):
        self.settings, self.terrain = settings, terrain
        self.space = pymunk.Space()
        self.space.gravity = (0, -9.81)
        self.space.iterations = 30
        self.space.collision_slop = .00003
        self.space.collision_bias = (1 - .15) ** 60
        self.ground = []
        points = terrain.points
        for index, (a, b) in enumerate(zip(points, points[1:])):
            segment = pymunk.Segment(self.space.static_body, a, b, .0003)
            segment.set_neighbors(points[max(0, index - 1)], points[min(index + 2, len(points) - 1)])
            segment.friction, segment.elasticity, segment.collision_type = settings.friction, 0, 2
            self.ground.append(segment)
        self.space.add(*self.ground)
        self.body = pymunk.Body(settings.mass, settings.inertia)
        point, _ = terrain.nearest(terrain.start)
        self.body.position = (terrain.start.x, point.y + settings.tip_radius + .010)
        self.tip = pymunk.Circle(self.body, settings.tip_radius)
        self.tip.collision_type = 1
        self.shaft = pymunk.Segment(self.body, (0, settings.tip_radius * 1.8), (0, settings.shaft_length), .0018)
        self.shaft.collision_type = 3
        for shape in (self.tip, self.shaft):
            shape.friction, shape.elasticity = 1.0, 0.0
        self.space.add(self.body, self.tip, self.shaft)
        self.space.on_collision(1, 2, post_solve=self._contact)
        self.space.on_collision(3, 2, post_solve=self._contact)
        self.sensor = Sensor()
        self.actuator_force = Vec2d(0, 0)
        self.step_force = self.step_impulse = Vec2d(0, 0)
        self.disturbance_steps = 0
        self.clear_impulses()

    @property
    def axis(self):
        return Vec2d(-sin(self.body.angle), cos(self.body.angle))

    @property
    def head(self):
        return self.body.local_to_world((0, self.settings.shaft_length))

    def clear_impulses(self):
        self.impulse = Vec2d(0, 0)
        self.angular_impulse = self.shaft_impulse = self.contact_weight = 0.0
        self.contact_sum = Vec2d(0, 0)

    def _contact(self, arbiter, _space, _data):
        impulse = arbiter.total_impulse
        self.step_impulse += impulse
        self.impulse += impulse
        contacts = arbiter.contact_point_set.points
        if contacts:
            point = sum((p.point_b for p in contacts), Vec2d(0, 0)) / len(contacts)
            self.angular_impulse += (point - self.head).cross(impulse)
            if arbiter.shapes[0].collision_type == 1:
                self.contact_sum += point * impulse.length
                self.contact_weight += impulse.length
        if arbiter.shapes[0].collision_type == 3:
            self.shaft_impulse += impulse.length

    def sample(self, dt):
        raw = self.impulse / dt
        self.sensor = Sensor(raw, raw, self.angular_impulse / dt, self.shaft_impulse / dt,
                             self.contact_sum / self.contact_weight if self.contact_weight else None)
        self.clear_impulses()
        return self.sensor

    def step(self, command, dt):
        p, s = self.body, self.settings
        error = command.position - p.position
        velocity_error = command.velocity - p.velocity
        if command.normal_force is None:
            force = s.position_stiffness * error + s.velocity_damping * velocity_error
        else:
            # Robot-like pressing follows the actual probe axis, not an instantly rotating ideal axis.
            n = self.axis if s.preset == 'robot' else command.normal
            tangent = Vec2d(n.y, -n.x)
            force = (s.position_stiffness * error.dot(tangent)
                     + s.velocity_damping * velocity_error.dot(tangent)) * tangent
            force -= (command.normal_force + s.normal_damping * p.velocity.dot(n)) * n
        if force.length > s.max_actuator_force:
            force *= s.max_actuator_force / force.length
        self.actuator_force += dt / (s.servo_tau + dt) * (force - self.actuator_force)
        # Compensation is an actuator force; neither body pose nor velocity is overwritten.
        p.force = self.actuator_force - s.mass * self.space.gravity
        if self.disturbance_steps:
            p.force += Vec2d(0, -1.5)
            self.disturbance_steps -= 1
        p.torque = limit(s.angular_stiffness * angle_error(command.angle, p.angle)
                         + s.angular_damping * (command.angular_velocity - p.angular_velocity),
                         -s.max_torque, s.max_torque)
        self.step_impulse = Vec2d(0, 0)
        self.space.step(dt)
        self.step_force = self.step_impulse / dt

    def set_friction(self, value):
        self.settings.friction = value
        for segment in self.ground:
            segment.friction = value

    def disturb(self):
        # A bounded load pulse is visible to a sampled sensor, unlike a one-step impulse.
        self.disturbance_steps = round(.12 * self.settings.physics_hz)
