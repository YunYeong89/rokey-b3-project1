"""defect_scan's hybrid surface follower: tangential position on the model path's own clock and a force target for
the native inner normal force regulator, the tool kept on the path normal. Shared delayed-state logic. SI."""
from dataclasses import dataclass, field
from math import sqrt

import numpy as np

from .geometry import align, bounded, unit
from .governor import ContactGovernor


@dataclass
class State:
    sequence: int
    sampled_at: float
    position: np.ndarray
    rotation: np.ndarray
    wrench: np.ndarray
    fast_wrench: np.ndarray
    velocity: np.ndarray
    control: dict = field(default_factory=dict)

    def centre(self, radius):
        # ProbeTCP is the axial bottom of the sphere, not its centre.
        return self.position - radius * self.rotation[:, 2]


class LevelKalman:
    """Random-walk Kalman filter of a vector level from noisy samples (independent axes sharing one variance):
    `noise` the sample variance, `process` the level's variance growth per second."""

    def __init__(self, size, noise, process):
        self.x, self.p, self.noise, self.process = np.zeros(size), noise, noise, process

    def update(self, measured, dt):
        self.p += self.process * dt
        gain = self.p / (self.p + self.noise)
        self.x = self.x + gain * (measured - self.x)
        self.p *= 1 - gain
        return self.x


class HybridFollower:
    """Tangential position + force target for the native inner normal force regulator. The outer PI works on the
    force along the actual tool axis, the axis the native regulator uses, so the two never pull apart."""

    def __init__(self, settings, path, initial):
        self.s, self.path = settings, path
        self.normal = unit(path.normals[0])
        self.reference = initial.centre(settings.radius)
        self.rotation = initial.rotation.copy()
        self.phase, self.reason = 'SEARCH', ''
        self.time = self.stable = self.lost = self.ramp = 0.0
        self.force = self.axial = self.fast = self.slope = self.risk = self.predicted = self.residual = 0.0
        self.integral = self.feed = self.goal = self.omega = 0.0
        self.last_sequence = None
        self.last_sampled_at = initial.sampled_at
        self.anchor = self.reference.copy()
        self.unload_normal = self.normal.copy()
        self.unloads = self.repeated = 0
        self.last_unload = None
        self.moment_bad = self.overloaded = self.moment_level = 0.
        self.moment_filter = LevelKalman(2, settings.moment_noise, settings.moment_process)
        self.scale = 1.
        self.command_progress = 0.
        self.touch_time = 0.
        self.ft = self.edge = self.stalled = self.hop_from = 0.
        self.edging = False
        self.governor = ContactGovernor(settings)

    def transition(self, phase, centre):
        self.phase, self.time, self.stable, self.lost = phase, 0., 0., 0.
        self.integral = 0.
        self.touch_time = 0.
        self.edge = self.stalled = 0.
        self.edging = False
        self.anchor = centre.copy()
        if phase != 'CONFIRM':
            self.reference = centre.copy()
        if phase == 'SETTLE':
            self.goal = self.ramp = float(np.clip(self.force, 0, self.s.force))
        if phase in ('RECOVER', 'HOP'):  # lost support / blocked: an anomaly, steep for the next slope_hold of travel
            self.governor.hold = max(self.governor.hold, 1.)
        if phase == 'UNLOAD':
            self.unloads += 1
            self.unload_normal = self.normal.copy()
            self.repeated = self.repeated + 1 if (self.last_unload is not None
                            and np.linalg.norm(centre - self.last_unload) < .001) else 0
            self.last_unload = centre.copy()

    def update(self, state, dt):
        s, centre = self.s, state.centre(self.s.radius)
        self.time += dt
        if self.phase == 'TRACK':
            # The model path has its own smooth clock: contact jitter must not repeatedly re-aim a long tool
            # through measured-progress feedback. The measured progress still follows the probe.
            self.path.observe(centre)
            self.command_progress = min(self.path.length, self.command_progress + self.feed * dt)
        guide, tangent, n = self.path.sample(self.command_progress)
        self.normal = n
        fresh = state.sequence != self.last_sequence
        if fresh:
            self.last_sequence = state.sequence
            sample_dt = max(1e-6, state.sampled_at - self.last_sampled_at)
            self.last_sampled_at = state.sampled_at
            # Overload trend on the filtered wrench: in Gazebo contact the unfiltered one swings 0.3-1.9 N every
            # 25 ms (spikes to 7 N), and its slope x latency predicted 4-11 N at ~1.3 N, forcing false UNLOADs.
            # The native side keeps guarding the unfiltered force (8 N for 20 ms).
            previous = self.fast
            self.fast += sample_dt / (.012 + sample_dt) * (np.linalg.norm(state.wrench[:3]) - self.fast)
            self.slope += sample_dt / (.04 + sample_dt) * ((self.fast - previous) / sample_dt - self.slope)
        self.force = float(state.wrench[:3] @ n)
        if fresh:
            self.touch_time = self.touch_time + sample_dt if self.force > s.touch else 0.
        self.axial = float(state.wrench[:3] @ -state.rotation[:, 2])  # along the actual tool axis, as native sees it
        self.predicted = self.fast + s.latency_budget * max(0., self.slope)
        self.risk = float(np.clip((self.predicted - s.force - .2) / (s.soft_margin - .2), 0, 1))
        if np.linalg.norm(state.wrench[:3]) > s.force_limit:
            raise RuntimeError('Measured force limit exceeded')
        # Off-sphere contact (shaft / gripper) tilts the tool: a lasting moment about the TOOL x / y axes. Those two
        # signed components of the residual go through a Kalman filter, so the payload's zero-mean vibration averages
        # out (|M| rectified it: 0.03-0.14 N m during ridge crossings aborted scans); the friction torque about the
        # tool axis is left out. Judged on the filtered wrench and only when it persists.
        lever = -s.radius * (state.rotation[:, 2] + n)
        residual = state.wrench[3:] - np.cross(lever, state.wrench[:3])
        self.residual = float(np.linalg.norm(residual))
        self.moment_level = float(np.linalg.norm(self.moment_filter.update(state.rotation[:, :2].T @ residual, dt)))
        inconsistent = np.linalg.norm(state.wrench[:3]) > .5 and self.moment_level > s.moment_residual
        self.moment_bad = self.moment_bad + dt if inconsistent else 0.
        if self.moment_bad > .2:
            raise RuntimeError('Wrench inconsistent with sphere contact; possible shaft / gripper collision')
        # The path supplies the otherwise unobservable cross-track direction in 3D.
        tangent = unit(tangent - n * (tangent @ n))
        self.ft = float(state.wrench[:3] @ tangent)  # reaction along the feed: friction / an edge ahead < 0
        angle = np.arccos(np.clip(-state.rotation[:, 2] @ n, -1, 1))
        self.omega += np.clip(min(s.angular_speed, 4 * angle) - self.omega,
                              -s.angular_acceleration * dt, s.angular_acceleration * dt)
        self.rotation = align(self.rotation, n, self.omega * dt)
        velocity, demand = np.zeros(3), None
        if self.phase == 'SEARCH':
            if self.touch_time >= .03:
                self.transition('CONFIRM', centre)
                error = np.clip((self.reference - centre) @ n, -.00035, 0)
                self.reference = centre + error * n
            elif self.time > 12 or np.linalg.norm(centre - self.anchor) > s.clearance + s.search_depth:
                raise RuntimeError('Contact search exceeded time / depth')
            else:
                velocity = -min(s.search_speed, .0003 if self.force > .1 else s.search_speed) * n
        if self.phase == 'CONFIRM':
            if self.time > .15:  # filtered: a single unfiltered sample sent a held contact back to SEARCH 7 times
                self.transition('SETTLE' if self.force > s.touch * .7 else 'SEARCH', centre)
            if self.phase == 'CONFIRM':
                demand = s.touch  # hold the light contact by force, not by a position inside a stiff surface
        # Sliding contact in Gazebo gives impulsive force spikes (also in the DSR runs): only a lasting filtered
        # overload unloads; the slope-based prediction only scales the feed (below).
        self.overloaded = self.overloaded + dt if self.fast > s.force + s.soft_margin else 0.
        if self.phase == 'TRACK' and self.overloaded and -self.ft > s.edge_force:
            # Pushed into an edge: back the trajectory clock off to the probe (less push) instead of UNLOAD, whose
            # RECOVER landed in the same groove again.
            self.command_progress = min(self.command_progress, self.path.progress + .0002)
            self.overloaded = 0.
        overload = self.overloaded >= .15
        if self.phase in ('SETTLE', 'TRACK') and overload:
            self.transition('UNLOAD', centre)
            if self.repeated >= 3:
                raise RuntimeError('Repeated overload without progress')
        if self.phase == 'UNLOAD':
            self.feed = self.goal = 0.
            velocity = .003 * self.unload_normal
            self.stable = self.stable + dt if self.fast < .8 * s.force else 0.
            if self.stable > .15:
                self.transition('RECOVER', centre)
            elif self.time > 2:
                raise RuntimeError('Bounded unloading failed')
        elif self.phase == 'HOP':  # lift, pass the blocking edge along the path in the air, then search down again
            self.feed = self.goal = 0.
            height = float((centre - self.anchor) @ n)
            if height < s.hop_height:
                velocity = .003 * n
            else:
                self.command_progress = min(self.path.length, self.command_progress + s.speed * dt)
                velocity = s.speed * tangent + 2 * (s.hop_height - height) * n
                if self.command_progress >= min(self.path.length, self.hop_from + s.hop_length):
                    self.transition('RECOVER', centre)
                    velocity[:] = 0
            if self.phase == 'HOP' and self.time > 5:
                raise RuntimeError('Hop over a blocked edge failed')
        elif self.phase == 'RECOVER':
            self.feed = self.goal = 0.
            velocity = -.0015 * n
            if self.touch_time >= .03:
                self.transition('CONFIRM', centre)
                velocity[:] = 0
            elif self.time > 3 or np.linalg.norm(centre - self.anchor) > .006:
                raise RuntimeError('Bounded contact recovery failed')
        if self.phase == 'SETTLE':
            self.stable = self.stable + dt if abs(self.force - s.force) < .25 else 0.
            if self.stable > .35:
                self.transition('TRACK', centre)
            elif self.time > 10:
                raise RuntimeError('Contact force failed to settle')
        if self.phase == 'TRACK':
            # Edge: a crack wall / step resists the feed (the ball wedged on a ~50 deg contact while the force channel
            # held 1.5 N and the trajectory clock waited: a static deadlock). The event itself is crack evidence.
            lag = self.command_progress - self.path.progress
            blocked = -self.ft > max(s.edge_force, s.edge_ratio * self.force) or lag >= s.edge_lag
            self.edge = self.edge + dt if blocked else 0.
            self.edging = self.edge >= .15
            self.stalled = self.stalled + dt if lag >= s.edge_lag else 0.
            if self.stalled >= s.hop_after:  # the clock restarts from where the probe is: no jump after landing
                self.hop_from = self.command_progress = self.path.progress
                self.transition('HOP', centre)
        if self.phase in ('SETTLE', 'TRACK'):
            target = self.ramp + min(1., self.time / .8) * (s.force - self.ramp) if self.phase == 'SETTLE' else s.force
            if self.phase == 'TRACK':  # climb: less force lets the servo lift the ball over
                target = min(target, self.governor.target(-self.ft))
            self.goal += np.clip(target - self.goal, -3 * dt, .8 * dt)
            budget = max(.05, s.force + s.soft_margin - self.fast) / s.stiffness_bound
            safe = s.braking * (sqrt(s.latency_budget ** 2 + 2 * budget / s.braking) - s.latency_budget)
            target = (1 - self.risk) * np.clip((np.deg2rad(18) - angle) / np.deg2rad(12), 0, 1)
            self.scale += dt / (.3 + dt) * (target - self.scale)  # smoothed: an instant cut made stop-go motion
            wanted = min(s.speed, safe) * self.scale if self.phase == 'TRACK' else 0.
            self.governor.observe(self.path.progress, float((centre - guide) @ n))
            wanted *= self.governor.feed(self.force, self.goal, -self.ft)
            lead = self.command_progress - self.path.progress
            wanted *= float(np.clip((.001 - lead) / .0005, 0, 1))
            if self.command_progress >= self.path.length:
                wanted = 0.
            self.feed += np.clip(wanted - self.feed, -s.acceleration * dt, s.acceleration * dt)
            depth = (centre - guide) @ n
            if depth < -s.search_depth:  # the native normal travel is no longer capped; the model depth is
                raise RuntimeError(f'Probe {-depth * 1000:.1f} mm below the expected surface; surface missing?')
            demand = self.force_demand(dt)
            velocity = self.feed * tangent
            if self.phase == 'TRACK':
                cross_error = guide - centre
                cross_error -= n * (cross_error @ n) + tangent * (cross_error @ tangent)
                velocity += bounded(2 * cross_error, .002) * self.scale
            self.lost = self.lost + dt if self.force < .15 else 0.
            if self.phase == 'TRACK' and self.lost > .2:
                self.transition('RECOVER', centre)
                velocity[:], demand = 0, None
            if self.phase == 'TRACK' and self.path.progress >= self.path.length - s.finish_tolerance:
                self.phase = 'FINISHED'
                velocity[:], demand = 0, None
        if self.phase == 'TRACK':
            # Tangential reference and orientation use one path parameter. The native
            # force regulator retains ownership of normal displacement.
            self.reference = guide
        else:
            self.reference += velocity * dt
        error = self.reference - centre
        en = np.clip(error @ n, -s.reference_limit, s.reference_limit)
        et = bounded(error - n * (error @ n), s.reference_limit)
        self.reference = centre + et + en * n
        if demand is not None:
            self.reference -= n * ((self.reference - centre) @ n)
        return dict(position=(self.reference + s.radius * self.rotation[:, 2]).tolist(),
                    rotation=self.rotation.tolist(), normal=n.tolist(), force=demand,
                    angular_speed=s.angular_speed, angular_acceleration=s.angular_acceleration,
                    source_sequence=state.sequence, source_sampled_at=state.sampled_at)

    def force_demand(self, dt):
        """Outer PI on the force along the actual tool axis -> the native regulator's force target."""
        s, error = self.s, self.goal - self.axial
        candidate = np.clip(self.integral + error * dt, -1, 1)
        requested = self.goal + s.kp * error + s.ki * candidate
        low = s.edge_min_force if self.edging else 0.  # keep contact while climbing an edge
        demand = float(np.clip(requested, low, min(5., s.force + s.soft_margin)))
        if demand == requested or error * (requested - demand) < 0:
            self.integral = candidate
        return demand
