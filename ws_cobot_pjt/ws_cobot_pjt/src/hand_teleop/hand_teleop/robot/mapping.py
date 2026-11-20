"""Hand signals in camera axes -> BASE TCP target (mm, rotation)."""
import numpy as np
from scipy.spatial.transform import Rotation


class TeleopMapper:
    """Translation, velocity mapping: the wrist offset from grounding (camera axes, mm) is turned into BASE axes by
    `camera_to_base` (operator and robot face the same way). Past the dead band it sets the TCP speed, up to
    `max_speed_mm_s` at `full_speed_mm`, which is integrated into the target position inside the box around HOME.
    Orientation: the wrist rotation since grounding, in BASE axes, scaled and limited, turns the TCP quaternion held at
    grounding: q_tcp = q_wrist * q_grounded."""

    def __init__(self, config):
        self.camera = np.array(config['camera_to_base'], dtype=float)
        self.scale = np.array(config['offset_scale'], dtype=float)
        self.deadband, self.full, self.speed = config['deadband_mm'], config['full_speed_mm'], config['max_speed_mm_s']
        self.box, self.floor = np.array(config['workspace_mm'], dtype=float), config['min_z_mm']
        self.gain = config['rotation_gain']
        self.rotation_deadband, self.rotation_max = np.radians(
            [config['rotation_deadband_deg'], config['rotation_max_deg']])

    def reset(self, position, rotation):
        """At HOME: the centre of the box and the start of the target."""
        self.home = np.array(position, dtype=float)
        self.place(position, rotation)

    def place(self, position, rotation):
        """The target restarts where the TCP rests (HOME or a stop)."""
        self.position = np.array(position, dtype=float)
        self.rotation = self.reference = Rotation.from_matrix(rotation)

    def anchor(self):
        """At grounding: the neutral wrist now stands for the current target orientation (clutch, no jump)."""
        self.reference = self.rotation

    def velocity(self, offset):
        v = self.camera @ (self.scale * np.asarray(offset))
        size = np.linalg.norm(v)
        if size <= self.deadband:
            return np.zeros(3)
        return v / size * self.speed * min((size - self.deadband) / (self.full - self.deadband), 1.0)

    def step(self, offset, quaternion, dt):
        """Next TCP target: BASE position (mm) and rotation matrix."""
        position = np.clip(self.position + self.velocity(offset) * dt,
                           self.home + self.box[:, 0], self.home + self.box[:, 1])
        position[2] = max(position[2], self.floor)
        self.position = position
        turn = self.camera @ Rotation.from_quat(quaternion).as_rotvec()
        angle = np.linalg.norm(turn)
        scaled = np.clip(self.gain * (angle - self.rotation_deadband), 0.0, self.rotation_max)
        self.rotation = Rotation.from_rotvec(turn / max(angle, 1e-9) * scaled) * self.reference
        return self.position, self.rotation.as_matrix()
