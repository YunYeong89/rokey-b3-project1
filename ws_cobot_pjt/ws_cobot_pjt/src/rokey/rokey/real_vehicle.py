"""The team's vehicle assembly (vehicles.yaml `real`, a tilted STL in unknown units) in the vehicle frame of
vehicle_model: mm, x toward the front, y toward the left, z up, origin at the footprint centre on the ground; split
into the scan parts bumper, bonnet, door, body (the outer shell) and other (numpy only)."""
from pathlib import Path

import numpy as np

if __package__:
    from .configs import PACKAGE, load
    from .vehicle_model import pose_matrix, write_stl
else:
    from configs import PACKAGE, load
    from vehicle_model import pose_matrix, write_stl

PARTS = ('bumper', 'bonnet', 'body', 'door')


def read_stl(path):
    """Binary STL -> (n, 3, 3) triangles."""
    data = Path(path).read_bytes()
    record = np.frombuffer(data, dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')],
                           count=int.from_bytes(data[80:84], 'little'), offset=84)
    return record['v'].astype(float)


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def clip(triangles, axis, value):
    """Split the triangles crossing the plane (coordinate `axis` = value) so that none crosses it; winding kept."""
    coordinates = triangles[:, :, axis]
    side = coordinates > value
    cross = (coordinates.min(1) < value - 1e-8) & (coordinates.max(1) > value + 1e-8)
    t, s = triangles[cross], side[cross]
    lone = np.where(s.sum(1) == 1, np.argmax(s, 1), np.argmin(s, 1))  # the vertex alone on its side
    row = np.arange(len(t))
    a, b, c = t[row, lone], t[row, (lone + 1) % 3], t[row, (lone + 2) % 3]
    cut = lambda p, q: p + ((value - p[:, axis]) / (q[:, axis] - p[:, axis]))[:, None] * (q - p)
    ab, ac = cut(a, b), cut(a, c)
    ab[:, axis] = ac[:, axis] = value
    pieces = np.concatenate([np.stack([a, ab, ac], 1), np.stack([ab, b, c], 1), np.stack([ab, c, ac], 1)])
    area = np.linalg.norm(np.cross(pieces[:, 1] - pieces[:, 0], pieces[:, 2] - pieces[:, 0]), axis=1)
    return np.concatenate([triangles[~cross], pieces[area > 1e-10]])


def connected(pairs, count):
    """Component label (0..k-1) of `count` nodes joined by index `pairs` (minimum-label propagation)."""
    label = np.arange(count)
    while True:
        before = label.copy()
        low = np.minimum(label[pairs[:, 0]], label[pairs[:, 1]])
        np.minimum.at(label, pairs[:, 0], low)
        np.minimum.at(label, pairs[:, 1], low)
        label = label[label]
        if np.array_equal(label, before):
            return np.unique(label, return_inverse=True)[1].ravel()


class RealVehicle:
    kind = 'real'

    def __init__(self, prepared=True, clip_parts=True):
        self.spec = load('vehicles')['real']
        front = unit(self.spec['front'])
        up = unit(np.cross(front, self.spec['left']))
        triangles = read_stl(PACKAGE / self.spec['mesh']) @ np.array([front, np.cross(up, front), up]).T
        low, high = triangles.reshape(-1, 3).min(0), triangles.reshape(-1, 3).max(0)
        scale = self.spec['length_mm'] / (high[0] - low[0])
        self.triangles = (triangles - [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]]) * scale
        self.size = (high - low) * scale
        if prepared and 'surface_mesh' in self.spec:
            # Prepared mesh: the vehicle frame in mm with the original footprint, at surface_mesh_length_mm.
            with np.load(PACKAGE / self.spec['surface_mesh']) as surface:
                self.triangles = surface['triangles'] * (self.spec['length_mm'] / self.spec['surface_mesh_length_mm'])
        self.surface_triangles = self.triangles
        x0, x1, z0, z1, _ = self.spec['door']  # CAD panels are long triangles: cut them at the door box
        for axis, value in (((0, x0), (0, x1), (2, z0), (2, z1)) if clip_parts else ()):
            self.triangles = clip(self.triangles, axis, value)
        self.damage = []
        self.parts = self.split()
        self._trim_door_interior()

    def _trim_door_interior(self):
        """Keep only the two outer door panels as scan targets, retaining the rest as collision geometry."""
        doors = self.parts == 'door'
        threshold = self.spec['door_outer_y_mm']
        triangles = clip(clip(self.triangles[doors], 1, -threshold), 1, threshold)
        exterior = np.abs(triangles[:, :, 1].mean(1)) >= threshold
        self.triangles = np.concatenate((self.triangles[~doors], triangles))
        self.parts = np.concatenate((self.parts[~doors], np.where(exterior, 'door', 'body')))

    def export_collision(self, directory):
        """One welded surface across visual part boundaries; OBJ retains shared vertex indices."""
        path = Path(directory) / 'real_surface.obj'
        vertices, inverse = np.unique(self.surface_triangles.reshape(-1, 3).round(8), axis=0, return_inverse=True)
        faces = inverse.reshape(-1, 3)
        t = vertices[faces]
        face_normals = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        keep = np.linalg.norm(face_normals, axis=1) > 1e-9
        normals = np.zeros_like(vertices)
        np.add.at(normals, faces[keep].ravel(), np.repeat(face_normals[keep], 3, axis=0))
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        with path.open('w') as stream:
            for vertex in vertices:
                stream.write('v %.10f %.10f %.10f\n' % tuple(vertex))
            for normal in normals:
                stream.write('vn %.10f %.10f %.10f\n' % tuple(normal))
            for face in faces[keep] + 1:
                stream.write('f ' + ' '.join(f'{index}//{index}' for index in face) + '\n')
        return path

    def placement(self):
        """[x, y, yaw] in BASE (mm, deg): on the robot's front axis, the front bumper at nose_x_mm, facing the robot."""
        return [self.spec['nose_x_mm'] + self.size[0] / 2, 0.0, 180.0]

    def pose(self, placement=None, table_z=0.0):
        """Vehicle frame -> BASE (4x4, mm), lift_mm above the table."""
        return pose_matrix(*(placement or self.placement()), table_z + self.spec['lift_mm'])

    def normals(self, triangles=None):
        t = self.triangles if triangles is None else triangles
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)

    def split(self):
        """Part per triangle (see vehicles.yaml `real`)."""
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
                & (centroid[:, 2] <= z1) & (np.abs(self.normals()[:, 1]) >= side))
        labels[door] = 'door'
        for part, seeds in spec['parts'].items():
            for seed in seeds:
                labels[body == body[np.argmin(np.linalg.norm(centroid - seed, axis=1))]] = part
        return labels

    def sample(self, spacing, mask=None, seed=0):
        """Area-uniform points (about one per spacing^2) on the masked triangles, with their triangle normals."""
        t = self.triangles if mask is None else self.triangles[mask]
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        rng = np.random.default_rng(seed)
        index = np.repeat(np.arange(len(t)), rng.poisson(np.linalg.norm(np.cross(b - a, c - a), axis=1) / 2
                                                          / spacing ** 2))
        u, v = rng.random((2, len(index)))
        flip = u + v > 1
        u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
        points = a[index] + u[:, None] * (b - a)[index] + v[:, None] * (c - a)[index]
        return points, self.normals(t)[index]

    def part_colors(self):
        body = np.array(self.spec['color'], dtype=float)
        return {'body': body, 'door': np.minimum(body * 1.25, 1.0), 'bonnet': np.minimum(body * 1.5, 1.0),
                'bumper': (0.16, 0.16, 0.17), 'other': (0.07, 0.07, 0.08)}

    def export(self, directory):
        """One STL (mm) per part, for Gazebo, RViz and the web viewer."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        paths = {}
        for part in dict.fromkeys(self.parts):
            paths[part] = directory / f'{self.kind}_{part}.stl'
            write_stl(paths[part], self.triangles[self.parts == part])
        return paths
