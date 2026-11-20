"""Local probe-only clearance on cached clouds; no mesh, arm or transfer planning."""
import numpy as np
from scipy.spatial import cKDTree


class CloudProbeClearance:
    def __init__(self, points, cylinders):
        self.points, self.tree, self.cylinders = points, cKDTree(points), cylinders

    def blocked(self, tips, normals):
        blocked = np.zeros(len(tips), bool)
        for start, end, radius in self.cylinders:
            centres = tips + (start + end) / 2 * normals
            neighbourhoods = self.tree.query_ball_point(centres, np.hypot((end - start) / 2, radius))
            for k, ids in enumerate(neighbourhoods):
                delta = self.points[ids] - tips[k]
                axial = delta @ normals[k]
                radial = np.sum(delta**2, axis=1) - axial**2
                blocked[k] |= np.any((axial >= start) & (axial <= end) & (radial < radius**2))
        return blocked
