"""Binary STL in/out, OBJ out and mass properties of closed triangle meshes (mm, kg). numpy only."""
from pathlib import Path

import numpy as np

RECORD = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])


def read_stl(path):
    """Binary STL -> (n, 3, 3) triangles."""
    data = Path(path).read_bytes()
    return np.frombuffer(data, RECORD, count=int.from_bytes(data[80:84], 'little'), offset=84)['v'].astype(float)


def write_stl(path, triangles):
    t = np.asarray(triangles, float)
    normal = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    record = np.zeros(len(t), RECORD)
    record['n'] = normal / np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
    record['v'] = t
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(bytes(80) + np.uint32(len(record)).tobytes() + record.tobytes())


def write_obj(path, triangles):
    """Shared-vertex OBJ (MuJoCo reads STL only up to 200000 faces)."""
    vertices, faces = np.unique(np.round(np.asarray(triangles, float).reshape(-1, 3), 4), axis=0, return_inverse=True)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        np.savetxt(f, vertices, fmt='v %.4f %.4f %.4f')
        np.savetxt(f, faces.reshape(-1, 3) + 1, fmt='f %d %d %d')


def normals(triangles):
    n = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


class MassProperties:
    """Of a closed, outward-wound mesh (signed tetrahedra to the origin): `volume` mm^3, `centre` mm, `mass` kg and
    `inertia` kg mm^2 about the centre for a density in g/cm^3."""

    def __init__(self, triangles, density=1.0):
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        v = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
        s = a + b + c
        self.volume = float(v.sum())
        self.centre = (v[:, None] * s).sum(0) / 4 / self.volume
        # second moment of each tetrahedron (origin, a, b, c): V / 20 (a a' + b b' + c c' + s s')
        second = np.einsum('i,ij,ik->jk', v / 20, a, a) + np.einsum('i,ij,ik->jk', v / 20, b, b) \
            + np.einsum('i,ij,ik->jk', v / 20, c, c) + np.einsum('i,ij,ik->jk', v / 20, s, s)
        second -= self.volume * np.outer(self.centre, self.centre)
        self.mass = self.volume * density * 1e-6
        self.inertia = density * 1e-6 * (np.trace(second) * np.eye(3) - second)


def sample(triangles, spacing, seed=0):
    """Area-uniform points (about one per spacing^2) and the index of their triangle."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    rng = np.random.default_rng(seed)
    index = np.repeat(np.arange(len(triangles)),
                      rng.poisson(np.linalg.norm(np.cross(b - a, c - a), axis=1) / 2 / spacing ** 2))
    u, v = rng.random((2, len(index)))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    return a[index] + u[:, None] * (b - a)[index] + v[:, None] * (c - a)[index], index


def box_inertia(mass, size_mm):
    """Solid box inertia (kg m2) about its centre."""
    x, y, z = np.asarray(size_mm, float) / 1000
    return mass / 12 * np.diag([y * y + z * z, x * x + z * z, x * x + y * y])


def box_triangles(low, high):
    """Closed, outward-wound box."""
    (x0, y0, z0), (x1, y1, z1) = low, high
    p = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return p[np.array(faces)]
