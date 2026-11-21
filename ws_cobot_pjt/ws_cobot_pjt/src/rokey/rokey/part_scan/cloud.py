"""Reference point cloud of one real-vehicle part in BASE (mm): area-uniform samples of the part's mesh (holes such
as door windows have none), and its outermost surface seen along a fixed probe axis (HeightMap), as defect_scan
probes the block top; no normal vectors."""
from itertools import product

import numpy as np
from scipy.ndimage import map_coordinates

from ..surface_follow.pointcloud_file import PointCloudFile


class PartCloud:
    """The part's samples in the vehicle frame, taken once and kept in `path` (PLY, vehicle frame, metres; delete it
    after changing the mesh or the spacing), placed at the session `pose`."""

    def __init__(self, vehicle, part, pose, spacing, path):
        if path.exists():
            points = PointCloudFile.load_ply(path) * 1000
        else:
            points, _ = vehicle.sample(spacing, vehicle.parts == part)
            if not len(points):
                raise ValueError(f'No {part} triangles in the real vehicle')
            PointCloudFile.save_ply(path, points / 1000, frame='vehicle',
                                    comments=[f'real vehicle {part} reference, spacing {spacing} mm'])
        self.part, self.path, self.points = part, path, points @ pose[:3, :3].T + pose[:3, 3]

    @staticmethod
    def neighbours(directory, part, pose, bounds, margin):
        """Existing neighbouring part clouds, cropped near the selected part; no mesh construction."""
        pieces = []
        for path in directory.glob('real_*_reference.ply'):
            if path.stem == f'real_{part}_reference':
                continue
            points = PointCloudFile.load_ply(path) * 1000 @ pose[:3, :3].T + pose[:3, 3]
            keep = np.all((points >= bounds[0] - margin) & (points <= bounds[1] + margin), axis=1)
            pieces.append(points[keep])
        return np.concatenate(pieces) if pieces else np.empty((0, 3))


def max_filter(values, radius, outside=-np.inf):
    """Maximum over the (2 radius + 1)^2 neighbourhood of each cell (`outside` beyond the grid), one axis at a time."""
    for axis in (0, 1):
        width = [(0, 0), (0, 0)]
        width[axis] = (radius, radius)
        padded, result = np.pad(values, width, constant_values=outside), np.full_like(values, -np.inf)
        for a in range(2 * radius + 1):
            result = np.maximum(result, np.take(padded, range(a, a + values.shape[axis]), axis))
        values = result
    return values


def ball_bottom(uv, height, origin, step, shape, radius):
    """Height of the bottom of a ball of `radius` lowered along the axis onto the points (uv, height) over each cell
    centre (-inf where no point lies within the radius): max over the points of height + sqrt(r^2 - d^2) - r."""
    index, n = np.floor((uv - origin) / step).astype(int), int(np.ceil(radius / step))
    result = np.full(shape, -np.inf)
    for offset in product(range(-n, n + 1), repeat=2):
        cell = index + offset
        d2 = np.sum((origin + (cell + .5) * step - uv) ** 2, axis=1)
        keep = (d2 <= radius ** 2) & np.all((cell >= 0) & (cell < shape), axis=1)
        np.maximum.at(result, tuple(cell[keep].T), height[keep] + np.sqrt(radius ** 2 - d2[keep]) - radius)
    return result


class HeightMap:
    """Outermost surface of the points along a fixed probe `axis` (BASE unit vector, the side the probe comes from)
    on a `step` grid across it, raised to where a ball of `radius` first touches (maximum over its footprint, which
    also fills the sampling gaps; planning). `ball`: the bottom of the resting ball (the probe TCP) over each cell, the
    reference of the measured TCP (a sphere, not the square footprint, so slopes are not biased high). `tilt`: slope along the axis (deg, 90 at and off the surface edge); `inner`: on
    the surface at least `margin` from an empty cell (part edges, window openings). Horizontal rows run along u:
    BASE y (left-right as seen from the robot), or BASE x when the axis is y (horizontal along a side); vertical rows
    along v = axis x u."""

    def __init__(self, points, axis, step, radius, margin):
        self.axis, self.step = np.asarray(axis, float), step
        u = np.array([0., 1., 0.]) if abs(self.axis[1]) < .9 else np.array([1., 0., 0.])
        self.u = u - self.axis * (u @ self.axis)
        self.u /= np.linalg.norm(self.u)
        self.v = np.cross(self.axis, self.u)
        uv = np.c_[points @ self.u, points @ self.v]
        self.origin = uv.min(0) - step
        index = np.floor((uv - self.origin) / step).astype(int)
        height = np.full(index.max(0) + 2, -np.inf)
        np.maximum.at(height, (index[:, 0], index[:, 1]), points @ self.axis)
        self.height = max_filter(height, int(np.ceil(radius / step)))
        self.ball = ball_bottom(uv, points @ self.axis, self.origin, step, height.shape, radius)
        known = np.isfinite(self.height)
        gu, gv = np.gradient(np.where(known, self.height, np.nan), step)
        self.tilt = np.nan_to_num(np.degrees(np.arctan(np.hypot(gu, gv))), nan=90.)
        # beyond the grid is empty too: the part's outermost rows (the doors' lower edge) keep the margin as well
        self.inner = max_filter(np.where(known, 0., 1.), int(np.ceil(margin / step)), outside=1.) <= 0

    def positions(self, i, j):
        """BASE tip points (mm) at cells (i, j) (index arrays, either one may be a single index; fractional between
        cells): the point across the axis on the surface height of the nearest cell."""
        i, j = np.broadcast_arrays(np.atleast_1d(i), np.atleast_1d(j))
        u = self.origin[0] + (i + .5) * self.step
        v = self.origin[1] + (j + .5) * self.step
        height = self.height[np.rint(i).astype(int), np.rint(j).astype(int)]
        return u[:, None] * self.u + v[:, None] * self.v + height[:, None] * self.axis

    def ball_at(self, points):
        """Ball-bottom height at BASE points (mm) across the axis, bilinear between cell centres (nan off the
        surface)."""
        cells = (np.c_[points @ self.u, points @ self.v] - self.origin) / self.step - .5
        return map_coordinates(np.where(np.isfinite(self.ball), self.ball, np.nan), cells.T, order=1, cval=np.nan)

    def ball_points(self):
        """BASE points (mm) of the ball-bottom surface, one per cell."""
        i, j = np.nonzero(np.isfinite(self.ball))
        return self.positions(i, j) + (self.ball[i, j] - self.height[i, j])[:, None] * self.axis
