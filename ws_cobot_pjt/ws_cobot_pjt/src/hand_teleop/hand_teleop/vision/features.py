"""Hand features from MediaPipe landmarks: the wrist as the position point, a rotation-independent depth scale, the
hand frame and the finger curl."""
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

WRIST, INDEX_MCP, MIDDLE_MCP, PINKY_MCP = 0, 5, 9, 17
TIPS, MCPS = [8, 12, 16, 20], [5, 9, 13, 17]
PALM_EDGES = np.array([(0, 5), (5, 17), (17, 0), (0, 9)]).T


def unit(vector):
    return vector / np.linalg.norm(vector)


@dataclass
class HandSample:
    image: np.ndarray       # 21 x 2 px in the raw (not mirrored) camera image
    wrist: np.ndarray       # px; turning the hand about the wrist leaves it in place
    pixels_per_m: float     # palm size in the image / its metric size seen from the camera = focal / depth
    rotation: Rotation      # hand frame in camera axes (x right, y down, z away from the lens)
    curl: float             # mean finger tip / MCP distance from the wrist: ~2 open palm, ~1 fist

    @classmethod
    def from_landmarks(cls, image, world):
        """`image` 21 x 2 px, `world` 21 x 3 m (MediaPipe world landmarks, camera axes). The palm edges projected from
        the metric landmarks shorten with a tilt exactly as in the image, so their ratio follows only the depth."""
        a, b = PALM_EDGES
        scale = (np.linalg.norm(image[a] - image[b], axis=1).sum()
                 / np.linalg.norm(world[a, :2] - world[b, :2], axis=1).sum())
        x = unit(world[INDEX_MCP] - world[PINKY_MCP])
        z = unit(np.cross(x, world[MIDDLE_MCP] - world[WRIST]))
        curl = np.linalg.norm(world[TIPS] - world[WRIST], axis=1) / np.linalg.norm(world[MCPS] - world[WRIST], axis=1)
        return cls(image, image[WRIST], float(scale), Rotation.from_matrix(np.column_stack([x, np.cross(z, x), z])),
                   float(curl.mean()))
