"""The team vehicle assembly on the stand, as the simulation uses it (rokey real_vehicle.py rules, reproduced here so
that this project stays independent): the tilted STL in unknown units -> vehicle frame mm (x front, y left, z up,
origin at the footprint centre on the ground), split into bonnet / bumper / door / body / other."""
import numpy as np

from .mesh import MassProperties, normals, read_stl


def unit(vector):
    vector = np.asarray(vector, float)
    return vector / np.linalg.norm(vector)


def clip(triangles, axis, value):
    """Split the triangles crossing the plane (coordinate `axis` = value) so that none crosses it; winding kept."""
    side = triangles[:, :, axis] > value
    cross = (side.sum(1) == 1) | (side.sum(1) == 2)
    t, s = triangles[cross], side[cross]
    lone = np.where(s.sum(1) == 1, np.argmax(s, 1), np.argmin(s, 1))
    row = np.arange(len(t))
    a, b, c = t[row, lone], t[row, (lone + 1) % 3], t[row, (lone + 2) % 3]
    cut = lambda p, q: p + ((value - p[:, axis]) / (q[:, axis] - p[:, axis]))[:, None] * (q - p)
    ab, ac = cut(a, b), cut(a, c)
    return np.concatenate([triangles[~cross], np.stack([a, ab, ac], 1), np.stack([ab, b, c], 1),
                           np.stack([ab, c, ac], 1)])


def connected(pairs, count):
    """Component label of `count` nodes joined by index `pairs` (minimum-label propagation)."""
    label = np.arange(count)
    while True:
        before = label.copy()
        low = np.minimum(label[pairs[:, 0]], label[pairs[:, 1]])
        np.minimum.at(label, pairs[:, 0], low)
        np.minimum.at(label, pairs[:, 1], low)
        label = label[label]
        if np.array_equal(label, before):
            return np.unique(label, return_inverse=True)[1].ravel()


class Vehicle:
    def __init__(self, spec, root):
        self.spec = spec
        front = unit(spec['front'])
        up = unit(np.cross(front, spec['left']))
        triangles = read_stl((root / spec['mesh']).resolve()) @ np.array([front, np.cross(up, front), up]).T
        low, high = triangles.reshape(-1, 3).min(0), triangles.reshape(-1, 3).max(0)
        scale = spec['length_mm'] / (high[0] - low[0])
        self.triangles = (triangles - [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]]) * scale
        self.size = (high - low) * scale
        x0, x1, z0, z1, _ = spec['door']
        for axis, value in ((0, x0), (0, x1), (2, z0), (2, z1)):
            self.triangles = clip(self.triangles, axis, value)
        self.parts = self.split()
        props = MassProperties(self.triangles, spec['density_g_cm3'] * spec['fill'])
        self.mass = spec['mass_kg'] or props.mass
        self.centre, self.inertia = props.centre, props.inertia * self.mass / props.mass

    def split(self):
        t, spec = self.triangles, self.spec
        _, vertex = np.unique(t.reshape(-1, 3).round(4), axis=0, return_inverse=True)
        vertex = vertex.reshape(-1, 3)
        body = connected(np.r_[vertex[:, :2], vertex[:, 1:]], vertex.max() + 1)[vertex[:, 0]]
        count = body.max() + 1
        low, high = np.full((count, 3), np.inf), np.full((count, 3), -np.inf)
        np.minimum.at(low, body, t.min(1))
        np.maximum.at(high, body, t.max(1))
        outer_low, outer_high = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
        margin, top = spec['shell_mm']
        reaches = np.c_[low[:, :2] - outer_low[:2] <= margin, outer_high - high <= margin].any(1)
        labels = np.where((reaches & (high[:, 2] >= top))[body], 'body', 'other').astype(object)
        x0, x1, z0, z1, side = spec['door']
        centroid = t.mean(1)
        door = ((labels == 'body') & (centroid[:, 0] >= x0) & (centroid[:, 0] <= x1) & (centroid[:, 2] >= z0)
                & (centroid[:, 2] <= z1) & (np.abs(normals(t)[:, 1]) >= side))
        labels[door] = 'door'
        for part, seeds in spec['parts'].items():
            for seed in seeds:
                labels[body == body[np.argmin(np.linalg.norm(centroid - seed, axis=1))]] = part
        return labels
