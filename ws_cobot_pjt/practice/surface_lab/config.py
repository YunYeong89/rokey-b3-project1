"""SI units inside the experiment: m, s, kg, N, radians."""
from dataclasses import dataclass


@dataclass
class Settings:
    preset: str = 'robot'
    scene: str = 'mixed'
    controller: str = 'hybrid'
    feed: float = 0.005
    target_force: float = 1.5
    friction: float = 0.15
    noise: float = 0.025
    time_scale: float = 1.0
    physics_hz: int = 500
    sensor_hz: int = 200
    feedback_hz: int = 60
    actuator_hz: int = 100
    control_hz: int = 40
    mass: float = 1.0  # Assumed effective Cartesian mass, not the probe payload mass.
    inertia: float = 0.003
    tip_radius: float = 0.0015
    shaft_length: float = 0.08
    sensor_tau: float = 0.08
    feedback_delay: float = 0.020  # Assumed transport characteristics, not identified hardware values.
    command_delay: float = 0.015
    jitter: float = 0.003
    dropout: float = 0.01  # Missed feedback updates, not TCP packet loss.
    state_timeout: float = 0.15
    command_timeout: float = 0.15
    sensor_bias_x: float = 0.02
    sensor_bias_y: float = -0.03
    inertia_error_mass: float = 0.01
    tare_seconds: float = 0.4
    servo_tau: float = 0.015
    max_linear_speed: float = 0.05
    max_acceleration: float = 0.1
    max_angular_acceleration: float = 2.094
    force_slew: float = 3.0
    position_stiffness: float = 2500.0
    velocity_damping: float = 100.0
    normal_damping: float = 90.0
    hybrid_kp: float = 0.1
    hybrid_ki: float = 0.15
    max_actuator_force: float = 12.0
    max_contact_force: float = 8.0
    max_angular_speed: float = 1.047
    angular_stiffness: float = 2.0
    angular_damping: float = 0.16
    max_torque: float = 0.3
    admittance_mass: float = 8.0
    admittance_damping: float = 800.0
    max_normal_speed: float = 0.005
    max_reference_offset: float = 0.005
    point_spacing: float = 0.0005
    search_speed: float = 0.003
    adaptive: bool = True
    soft_force_margin: float = 0.9
    max_reference_normal: float = 0.002
    max_reference_tangent: float = 0.002
    contact_stiffness_bound: float = 3500.0  # Assumed conservative N/m; must be identified for a real robot.
    max_moment_residual: float = 0.025
    bias_drift_y: float = 0.0  # Optional residual drift [N/s] after the initial tare.
    braking_deceleration: float = 0.08  # Assumed available braking, not a hardware guarantee [m/s^2].

    @classmethod
    def for_preset(cls, name):
        if name == 'ideal':
            return cls(preset='ideal', noise=0, time_scale=4, sensor_hz=100, feedback_hz=100,
                       control_hz=100, mass=.18, inertia=.0004, tip_radius=.006,
                       sensor_tau=.02, feedback_delay=0, command_delay=0, jitter=0, dropout=0,
                       sensor_bias_x=0, sensor_bias_y=0, inertia_error_mass=0, servo_tau=0,
                       velocity_damping=45, normal_damping=30, hybrid_kp=.4, hybrid_ki=.5,
                       admittance_mass=1, admittance_damping=220, max_angular_speed=1.4,
                       angular_stiffness=1, angular_damping=.05, max_torque=.06,
                       max_normal_speed=.01, search_speed=.008)
        return cls()
