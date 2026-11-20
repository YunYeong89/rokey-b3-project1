"""Round raster joins and smooth tool normals without changing the measured cloud."""
import numpy as np
from scipy.ndimage import gaussian_filter1d


class CloudPath:
    def __init__(self, radius, step):
        self.radius, self.step = radius, step

    def sample(self, cells, grid_step):
        points = np.asarray(cells, float) * grid_step
        if len(points) < 2:
            return np.asarray(cells, float)
        delta = np.diff(points, axis=0)
        tangent = delta / np.linalg.norm(delta, axis=1, keepdims=True)
        corners = np.flatnonzero(np.sum(tangent[:-1] * tangent[1:], axis=1) < .99999) + 1
        points = points[np.r_[0, corners, len(points) - 1]]
        pieces = [points[:1]]
        for a, b, c in zip(points[:-2], points[1:-1], points[2:]):
            incoming, outgoing = b - a, c - b
            before, after = np.linalg.norm(incoming), np.linalg.norm(outgoing)
            u, v = incoming / before, outgoing / after
            angle = np.arccos(np.clip(u @ v, -1., 1.))
            if angle > np.pi - 1e-5:
                pieces.append(b[None])
                continue
            cut = min(self.radius * np.tan(angle / 2), .45 * before, .45 * after)
            radius = cut / np.tan(angle / 2)
            across = (v - u * np.cos(angle)) / np.sin(angle)
            entry = b - cut * u
            centre = entry + radius * across
            angles = np.linspace(0., angle, max(2, int(np.ceil(radius * angle / self.step)) + 1))
            pieces.append(centre + radius * (-np.cos(angles[:, None]) * across
                                              + np.sin(angles[:, None]) * u))
        pieces.append(points[-1:])
        curve = np.concatenate(pieces)
        dense = [curve[:1]]
        for a, b in zip(curve[:-1], curve[1:]):
            dense.append(np.linspace(a, b, max(1, int(np.ceil(np.linalg.norm(b - a) / self.step))) + 1)[1:])
        return np.concatenate(dense) / grid_step

    @staticmethod
    def normals(normals, distance, step, tolerance_deg):
        smoothed = gaussian_filter1d(normals, distance / step, axis=0, mode='nearest')
        smoothed /= np.linalg.norm(smoothed, axis=1, keepdims=True)
        angles = np.arccos(np.clip(np.sum(normals * smoothed, axis=1), -1., 1.))
        axis = np.cross(normals, smoothed)
        axis /= np.maximum(np.linalg.norm(axis, axis=1, keepdims=True), 1e-12)
        # Soft limit (tanh): a hard clamp cut the smoothing off on curved stretches and kinked the tool turn there.
        limit = np.radians(tolerance_deg)
        angles = (limit * np.tanh(angles / limit))[:, None]
        smoothed = normals * np.cos(angles) + np.cross(axis, normals) * np.sin(angles)
        return smoothed / np.linalg.norm(smoothed, axis=1, keepdims=True)
