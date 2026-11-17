"""defect_scan's 3D path: BASE sphere centres and normals, progress following the measured probe."""
import numpy as np

from ..rotations import align, unit  # noqa: F401  (align: re-exported)


def bounded(value, maximum):
    return value * min(1., maximum / max(np.linalg.norm(value), 1e-12))


class SurfacePath:
    """BASE sphere centres [m] and outward normal priors; progress follows measured centre."""
    def __init__(self, centres, normals):
        self.centres = np.asarray(centres, dtype=float)
        self.normals = np.asarray(normals, dtype=float)
        if (self.centres.ndim != 2 or self.centres.shape[1] != 3 or len(self.centres) < 2
                or self.normals.shape != self.centres.shape
                or not np.isfinite(np.r_[self.centres, self.normals]).all()):
            raise ValueError('Path requires matching finite Nx3 centres and normals')
        self.normals = np.array([unit(n) for n in self.normals])
        delta = np.diff(self.centres, axis=0)
        self.lengths = np.linalg.norm(delta, axis=1)
        if np.any(self.lengths < 1e-6):
            raise ValueError('Consecutive path points must be distinct')
        self.tangents = delta / self.lengths[:, None]
        self.arc = np.r_[0., np.cumsum(self.lengths)]
        self.progress = 0.0
        self.index = 0

    @property
    def length(self):
        return float(self.arc[-1])

    def distance(self, centre, i):
        along = np.clip((centre - self.centres[i]) @ self.tangents[i], 0, self.lengths[i])
        return float(np.linalg.norm(centre - self.centres[i] - along * self.tangents[i]))

    def observe(self, centre):
        # Local forward projection, one segment at a time, avoids jumping to a nearby return leg of a path. The next
        # segment takes over once it is at least as close: advancing 0.3 mm early held the progress on the nodes of
        # a 0.36 mm path (the tool re-aimed in bursts), and waiting for the exact segment end stalled at a raster
        # corner the probe passed 19 um short of.
        while (self.index < len(self.lengths) - 1
               and self.distance(centre, self.index + 1) <= self.distance(centre, self.index)):
            self.index += 1
        i = self.index
        along = float((centre - self.centres[i]) @ self.tangents[i])
        self.progress = max(self.progress, self.arc[i] + np.clip(along, 0, self.lengths[i]))
        return self.sample(self.progress)

    def sample(self, distance):
        i = min(len(self.lengths) - 1, np.searchsorted(self.arc, distance, side='right') - 1)
        i = max(0, i)
        fraction = float(np.clip((distance - self.arc[i]) / self.lengths[i], 0, 1))
        point = self.centres[i] + fraction * (self.centres[i + 1] - self.centres[i])
        normal = unit((1 - fraction) * self.normals[i] + fraction * self.normals[i + 1])
        tangent = self.tangents[i] - normal * (self.tangents[i] @ normal)
        return point, unit(tangent), normal
