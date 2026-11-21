"""Causal normal-force servo for the position-driven Gazebo robot. SI units.

The force channel owns only displacement along the actual tool axis. Measured
normal velocity damps contact motion; a bounded reference prevents stored push-in
while a stiff contact, joint servo or rotation prevents the tip from following.
"""
import numpy as np


class NormalForceKalman:
    """Normal force estimate x = [F, d] (force, unmodelled force rate) from the tick-averaged measurement.
    The servo's own commanded displacement is the known input (F falls by stiffness x outward travel), so the
    force change it causes is predicted without lag; d absorbs curvature, rotation and arm dynamics. Replaces
    the 80 + 25 ms filters (~105 ms of lag in the loop): 63 % of an unmodelled step in ~70 ms instead of 140 ms.
    The innovation (measurement - prediction) and its sigma are kept for defect detection."""
    stiffness = 5000.            # N/m, commanded outward displacement -> force drop (arm servo + contact, ~3-7k)
    noise = .04                  # N^2, tick-averaged measurement variance
    process = np.array([.05, 20.])  # per second: force random walk N^2, rate random walk (N/s)^2

    def __init__(self, force):
        self.x, self.P = np.array([force, 0.]), np.diag([.05, 1.])
        self.innovation, self.sigma = 0., float(np.sqrt(self.P[0, 0] + self.noise))

    def update(self, measured, outward, dt):
        A = np.array([[1., dt], [0., 1.]])
        self.x = A @ self.x - [self.stiffness * outward, 0.]
        self.P = A @ self.P @ A.T + np.diag(self.process * dt)
        variance = self.P[0, 0] + self.noise
        self.innovation, self.sigma = float(measured - self.x[0]), float(np.sqrt(variance))
        gain = self.P[:, 0] / variance
        self.x = self.x + gain * self.innovation
        self.P = self.P - np.outer(gain, self.P[0])
        return float(self.x[0])


class NormalForceServo:
    mass = 8.0
    damping = 2000.0
    velocity_damping = 1400.0
    velocity_tau = .020
    speed_limit = .0015
    acceleration = .015
    reference_limit = .0005

    def __init__(self, force):
        self.force = self.target = float(np.clip(force, 0, 5))
        self.kalman = NormalForceKalman(self.force)
        self.velocity = self.measured_velocity = self.correction = 0.
        self.saturated = False

    def update(self, force, velocity, target, reference_error, dt):
        self.force = self.kalman.update(force, self.correction, dt)
        self.measured_velocity += dt / (self.velocity_tau + dt) * (velocity - self.measured_velocity)
        # Unloading fast is safe (rokey drops the target to climb a blocking edge); loading stays gentle.
        self.target += float(np.clip(target - self.target, -3 * dt, .8 * dt))
        effort = self.force - self.target - self.velocity_damping * self.measured_velocity
        wanted = (self.mass * self.velocity + dt * effort) / (self.mass + self.damping * dt)
        # Reacquire gently instead of accelerating into an unloaded surface.
        inward_limit = .0005 if self.force < .2 else self.speed_limit
        wanted = float(np.clip(wanted, -inward_limit, self.speed_limit))
        self.velocity += float(np.clip(wanted - self.velocity, -self.acceleration * dt, self.acceleration * dt))
        requested = reference_error + self.velocity * dt
        limited = float(np.clip(requested, -self.reference_limit, self.reference_limit))
        self.saturated = abs(requested - limited) > 1e-12
        if self.saturated:
            self.velocity = 0.
        self.correction = limited - reference_error
        return self.correction

    def diagnostics(self):
        return dict(force_feedback_n=self.force, force_target_n=self.target,
                    normal_velocity_mps=self.measured_velocity, normal_command_mps=self.velocity,
                    normal_reference_saturated=self.saturated, force_innovation_n=self.kalman.innovation,
                    innovation_sigma_n=self.kalman.sigma, force_rate_nps=float(self.kalman.x[1]))
