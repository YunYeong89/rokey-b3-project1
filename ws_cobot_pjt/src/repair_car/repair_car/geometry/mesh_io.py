"""Binary STL in/out as (n, 3, 3) triangle arrays (numpy only)."""
from pathlib import Path

import numpy as np

RECORD = [('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]


def read_stl(path, scale=1.0):
    data = Path(path).read_bytes()
    record = np.frombuffer(data, dtype=RECORD, count=int.from_bytes(data[80:84], 'little'), offset=84)
    return record['v'].astype(float) * scale


def write_stl(path, triangles, scale=1.0):
    triangles = np.asarray(triangles, dtype=float) * scale
    normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    record = np.zeros(len(triangles), dtype=RECORD)
    record['n'] = normal / np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
    record['v'] = triangles
    Path(path).write_bytes(bytes(80) + np.uint32(len(record)).tobytes() + record.tobytes())
