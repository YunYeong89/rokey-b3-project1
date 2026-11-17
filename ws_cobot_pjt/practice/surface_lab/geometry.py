"""Terrain generation and sensor-only normal estimation. Truth is for display/evaluation."""
from collections import deque
from math import atan2, cos, pi, sin

from pymunk import Vec2d


def limit(value, low, high):
    return max(low, min(value, high))


def angle_error(target, actual):
    return (target - actual + pi) % (2 * pi) - pi


def normal_angle(normal):
    return atan2(-normal.x, normal.y)


class Terrain:
    names = ('smooth', 'corners', 'mixed', 'wall')

    def __init__(self, name):
        self.name = name
        if name == 'smooth':
            self.points = [Vec2d(i * .70 / 240, .16 + .045 * (1 - cos(4 * pi * i / 240)))
                           for i in range(241)]
        elif name == 'corners':
            self.points = [Vec2d(*p) for p in [(0, .16), (.13, .16), (.22, .23), (.30, .23),
                                              (.39, .14), (.46, .14), (.56, .22), (.70, .22)]]
        elif name == 'wall':
            self.points = [Vec2d(*p) for p in [(0, .14), (.22, .14), (.22, .32), (.70, .32)]]
        else:
            self.points = [Vec2d(i * .30 / 100, .16 + .035 * (1 - cos(2 * pi * i / 100)))
                           for i in range(101)]
            self.points += [Vec2d(*p) for p in [(.36, .16), (.43, .23), (.50, .23),
                                               (.57, .17), (.70, .17)]]
        self.start = self.points[0] + Vec2d(.035, 0)
        self.end_x = self.points[-1].x - .025

    def nearest(self, point):
        """Debug truth only. Controllers never receive a Terrain object."""
        best = None
        for a, b in zip(self.points, self.points[1:]):
            edge = b - a
            closest = a + limit((point - a).dot(edge) / edge.length_squared, 0, 1) * edge
            distance = (closest - point).length_squared
            normal = Vec2d(-edge.y, edge.x).normalized()
            if best is None or distance < best[0]:
                best = distance, closest, normal
        return best[1], best[2]


class NormalObserver:
    """Reaction with estimated friction removed, fused with a short measured-center chord.

    No collision normals, terrain slope, or ideal path heights enter this observer.
    A single planar track supplies a tangent in 2D; this is not a 3D normal estimator.
    """

    def __init__(self):
        self.normal = Vec2d(0, 1)
        self.mu = .15
        self.centers = deque()

    def update(self, position, force, dt):
        if force.length < .20:
            return self.normal
        if not self.centers or (position - self.centers[-1]).length > .00015:
            self.centers.append(Vec2d(*position))
        while len(self.centers) > 2 and (position - self.centers[0]).length > .003:
            self.centers.popleft()
        tangent = Vec2d(self.normal.y, -self.normal.x)
        geometric = None
        if len(self.centers) >= 3:
            chord = self.centers[-1] - self.centers[0]
            if chord.length > .0008:
                observed_tangent = chord.normalized()
                if observed_tangent.dot(tangent) < 0:
                    observed_tangent = -observed_tangent
                geometric = Vec2d(-observed_tangent.y, observed_tangent.x)
                load = force.dot(geometric)
                if load > .3:
                    estimate = limit(-force.dot(observed_tangent) / load, 0, .8)
                    self.mu += dt / (1.5 + dt) * (estimate - self.mu)
        reactive = force + self.mu * max(force.dot(self.normal), 0) * tangent
        if reactive.length < .05:
            return self.normal
        desired = reactive.normalized()
        if geometric is not None:
            combined = .8 * desired + .2 * geometric
            if combined.length > .01:
                desired = combined.normalized()
        delta = atan2(self.normal.cross(desired), self.normal.dot(desired))
        self.normal = self.normal.rotated(limit(delta * dt / (.025 + dt), -5 * dt, 5 * dt))
        return self.normal
