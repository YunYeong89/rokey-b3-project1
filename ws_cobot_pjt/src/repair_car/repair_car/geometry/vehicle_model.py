"""Vehicle shapes from the repair_car/meshes/vehicles DAE files, shared by the native simulation map and the robot
programs (numpy only).

Vehicle frame (right-handed, mm): x toward the front, y toward the left, z up, origin at the centre of the
footprint on the ground. The INTERFACE.md frame (origin rear-left-bottom, y toward the right) is only used
at the web/DB boundary (`to_interface`).
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

if __package__:
    from ..config import PACKAGE, load
else:
    from config import PACKAGE, load

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def pose_matrix(x, y, yaw_deg, z=0.0):
    """Planar pose (mm, deg) as a 4x4 transform."""
    c, s = np.cos(np.deg2rad(yaw_deg)), np.sin(np.deg2rad(yaw_deg))
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1.0]])


def transform(matrix, points):
    points = np.asarray(points, dtype=float)
    return points @ matrix[:3, :3].T + matrix[:3, 3]


class VehicleModel:
    """Triangles of one vehicle type split into parts, a top-down height map and the queries on it."""

    def __init__(self, kind, damage=(), resolution=1.0):
        self.kind, self.spec = kind, load('vehicles')['types'][kind]
        self.resolution = resolution
        triangles, self.materials = self.read_dae(PACKAGE / self.spec['mesh'], self.spec['mm_per_unit'])
        low, high = triangles.reshape(-1, 3).min(0), triangles.reshape(-1, 3).max(0)
        self.triangles = triangles - [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]]
        self.size = high - low
        self.orient()
        self.parts = self.split()
        self.damage = [self.apply(dict(item)) for item in damage]
        self.height_map()

    @staticmethod
    def read_dae(path, scale):
        """All instanced triangles with their node transforms, mapped from the Y-up DAE (x left, z front)."""
        root = ET.parse(path).getroot()
        geometries = {g.get('id'): g.find('c:mesh', NS) for g in root.iterfind('c:library_geometries/c:geometry', NS)}
        axes = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])  # (z, x, y)_dae
        triangles, materials = [], []
        for node in root.iterfind('c:library_visual_scenes/c:visual_scene/c:node', NS):
            matrix = axes @ np.array(node.find('c:matrix', NS).text.split(), float).reshape(4, 4)
            for instance in node.findall('c:instance_geometry', NS):
                mesh = geometries[instance.get('url')[1:]]
                source = mesh.find('c:vertices/c:input[@semantic="POSITION"]', NS).get('source')[1:]
                points = np.array(mesh.find(f'c:source[@id="{source}"]/c:float_array', NS).text.split(),
                                  float).reshape(-1, 3)
                points = transform(matrix, points) * scale
                for primitive in mesh.findall('c:polylist', NS) + mesh.findall('c:triangles', NS):
                    stride = 1 + max(int(i.get('offset')) for i in primitive.findall('c:input', NS))
                    index = np.array(primitive.find('c:p', NS).text.split(), int)[::stride].reshape(-1, 3)
                    triangles.append(points[index])
                    materials += [instance.find('.//c:instance_material', NS).get('target')[1:]] * len(index)
        return np.concatenate(triangles), np.array(materials)

    def split(self):
        """Part label per triangle: the first region box of vehicles.yaml containing its centroid, else body.
        Wheels and tyres stay `wheel`."""
        centroid = self.triangles.mean(1)
        labels = np.where(np.isin(self.materials, ['tires', 'wheels']), 'wheel', 'body').astype(object)
        for part, box in self.spec['parts'].items():
            low, high = np.array(box).reshape(2, 3)
            inside = np.all((centroid >= low) & (centroid <= high), axis=1) & (labels == 'body')
            labels[inside] = part
        return labels

    def normals(self):
        n = np.cross(self.triangles[:, 1] - self.triangles[:, 0], self.triangles[:, 2] - self.triangles[:, 0])
        return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)

    def orient(self):
        """The pack's winding is mixed; point every normal away from the body centre (scaled to the box)
        so the STL renders and collides from the outside."""
        outward = (self.triangles.mean(1) - [0.0, 0.0, self.size[2] / 2]) / self.size
        flip = np.einsum('ij,ij->i', self.normals(), outward) < 0
        self.triangles[flip] = self.triangles[flip][:, ::-1]

    def apply(self, damage):
        """Simulated damage, returned with its resolved centre. {'type': 'missing', 'part'} removes a part;
        {'type': 'dent', 'part', 'offset_mm': [dx, dy], 'radius', 'depth', optional 'length', 'angle_deg'}
        presses a round dent (with `length`, an elongated crack-like groove) into the top surface at the
        part's centre + offset, refining the triangles under it first."""
        if damage['type'] == 'missing':
            keep = self.parts != damage['part']
            self.triangles, self.parts, self.materials = self.triangles[keep], self.parts[keep], self.materials[keep]
            return damage
        part = self.triangles[self.parts == damage['part']]
        area = np.linalg.norm(np.cross(part[:, 1] - part[:, 0], part[:, 2] - part[:, 0]), axis=1)
        centre = (area @ part.mean(1)[:, :2]) / area.sum() + damage.get('offset_mm', [0.0, 0.0])
        angle = np.deg2rad(damage.get('angle_deg', 0.0))
        half = damage.get('length', 0.0) / 2 * np.array([np.cos(angle), np.sin(angle)])
        a, b, radius, depth = centre - half, centre + half, damage['radius'], damage['depth']

        def distance(points):
            ab = b - a
            t = np.clip(((points - a) @ ab) / max(ab @ ab, 1e-9), 0, 1)
            return np.linalg.norm(points - (a + t[:, None] * ab), axis=1)

        centroid = self.triangles.mean(1)
        reach = np.linalg.norm(self.triangles[:, :, :2] - centroid[:, None, :2], axis=2).max(1)
        near = (distance(centroid[:, :2]) < radius + reach) & (np.abs(self.normals()[:, 2]) > 0.5)
        near &= centroid[:, 2] > self.size[2] / 4  # top surfaces, not the underside
        pieces = [self.subdivide(triangle, max(radius / 4, 1.0)) for triangle in self.triangles[near]]
        counts = [len(piece) for piece in pieces]
        points = np.concatenate(pieces).reshape(-1, 3)
        points[:, 2] -= depth * np.clip(1 - (distance(points[:, :2]) / radius) ** 2, 0, None)
        self.triangles = np.concatenate([self.triangles[~near], points.reshape(-1, 3, 3)])
        self.parts = np.concatenate([self.parts[~near], np.repeat(self.parts[near], counts)])
        self.materials = np.concatenate([self.materials[~near], np.repeat(self.materials[near], counts)])
        return {**damage, 'from_mm': a.round(2).tolist(), 'to_mm': b.round(2).tolist()}

    @staticmethod
    def subdivide(triangle, edge):
        """Split into n^2 similar triangles with edges no longer than `edge`."""
        n = int(np.ceil(max(np.linalg.norm(triangle - np.roll(triangle, 1, 0), axis=1)) / edge))
        a, u, v = triangle[0], (triangle[1] - triangle[0]) / n, (triangle[2] - triangle[0]) / n
        i, j = np.nonzero(np.add.outer(np.arange(n), np.arange(n)) < n)
        point = lambda i, j: a + i[:, None] * u + j[:, None] * v
        up = np.stack([point(i, j), point(i + 1, j), point(i, j + 1)], 1)
        k = i + j < n - 1
        down = np.stack([point(i[k] + 1, j[k]), point(i[k] + 1, j[k] + 1), point(i[k], j[k] + 1)], 1)
        return np.concatenate([up, down])

    def height_map(self):
        """Highest surface z and its part on a `resolution` grid (vertical touches see exactly this)."""
        r = self.resolution
        self.origin = self.triangles[:, :, :2].reshape(-1, 2).min(0) - 2 * r
        shape = np.ceil((self.triangles[:, :, :2].reshape(-1, 2).max(0) + 2 * r - self.origin) / r).astype(int) + 1
        self.z = np.full(shape, np.nan)
        self.label = np.full(shape, '', dtype=object)
        for triangle, part in zip(self.triangles, self.parts):
            xy = (triangle[:, :2] - self.origin) / r
            low, high = np.floor(xy.min(0)).astype(int), np.ceil(xy.max(0)).astype(int)
            gx, gy = np.meshgrid(np.arange(low[0], high[0] + 1), np.arange(low[1], high[1] + 1), indexing='ij')
            p = np.c_[gx.ravel(), gy.ravel()].astype(float)
            d = np.array([xy[1] - xy[0], xy[2] - xy[0]]).T
            if abs(np.linalg.det(d)) < 1e-9:
                continue
            w = np.linalg.solve(d, (p - xy[0]).T).T
            inside = (w >= -1e-6).all(1) & (w.sum(1) <= 1 + 1e-6)
            z = triangle[0, 2] + w[inside] @ (triangle[1:, 2] - triangle[0, 2])
            i, j = gx.ravel()[inside], gy.ravel()[inside]
            higher = ~(self.z[i, j] >= z)
            self.z[i[higher], j[higher]], self.label[i[higher], j[higher]] = z[higher], part

    def cells(self, points):
        index = np.rint((np.asarray(points, float)[..., :2] - self.origin) / self.resolution).astype(int)
        index = np.clip(index, 0, np.array(self.z.shape) - 1)
        return index[..., 0], index[..., 1]

    def surface(self, points):
        """Top surface z (nan off the vehicle) and the part label at vehicle-frame (x, y) points."""
        i, j = self.cells(points)
        return self.z[i, j], self.label[i, j]

    def probe_height(self, points, radius):
        """Lowest tip z of a sphere probe of `radius` coming down at (x, y): the max over its footprint
        of surface z + the sphere's rise at that offset; the ground (z = 0) off the vehicle."""
        points = np.atleast_2d(points)
        k = int(np.ceil(radius / self.resolution))
        offsets = np.array([(a, b) for a in range(-k, k + 1) for b in range(-k, k + 1)]) * self.resolution
        rise = np.sqrt(np.clip(radius ** 2 - (offsets ** 2).sum(1), 0, None)) - radius
        valid = (offsets ** 2).sum(1) <= radius ** 2
        z, _ = self.surface(points[:, None, :] + offsets[valid])
        return np.maximum(np.nanmax(np.where(np.isnan(z), -np.inf, z + rise[valid]), axis=1), 0.0)

    def shadow(self, points, radius):
        """Highest surface within `radius` of (x, y) (ground 0 included): what the gripper above a
        touch at (x, y) has to clear."""
        if getattr(self, 'shadow_radius', None) != radius:
            z, k = np.nan_to_num(self.z), int(np.ceil(radius / self.resolution))
            self.shadow_map, self.shadow_radius = z.copy(), radius
            for a in range(-k, k + 1, 2):
                for b in range(-k, k + 1, 2):
                    if a * a + b * b <= k * k:
                        self.shadow_map = np.maximum(self.shadow_map, np.roll(z, (a, b), (0, 1)))
        i, j = self.cells(points)
        return self.shadow_map[i, j]

    def normal(self, points):
        """Upward surface normal from the height-map gradient."""
        step = np.array([[self.resolution, 0], [0, self.resolution]])
        z = lambda offset: self.surface(np.asarray(points, float) + offset)[0]
        gx = (z(step[0]) - z(-step[0])) / (2 * self.resolution)
        gy = (z(step[1]) - z(-step[1])) / (2 * self.resolution)
        n = np.stack([-gx, -gy, np.ones_like(gx)], -1)
        return n / np.linalg.norm(n, axis=-1, keepdims=True)

    def part_cells(self, part):
        """(x, y, z) of the top-surface cells belonging to a part."""
        i, j = np.nonzero(self.label == part)
        return np.c_[self.origin + np.c_[i, j] * self.resolution, self.z[i, j]]

    def sample(self, spacing):
        """Area-uniform points on all triangles (about one per spacing^2) with part labels."""
        a, b, c = self.triangles[:, 0], self.triangles[:, 1], self.triangles[:, 2]
        area = np.linalg.norm(np.cross(b - a, c - a), axis=1) / 2
        rng = np.random.default_rng(0)
        count = rng.poisson(area / spacing ** 2)
        index = np.repeat(np.arange(len(area)), count)
        u, v = rng.random((2, len(index)))
        flip = u + v > 1
        u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
        points = a[index] + u[:, None] * (b - a)[index] + v[:, None] * (c - a)[index]
        return points, self.parts[index]

    def to_interface(self, points):
        """Vehicle frame -> INTERFACE.md frame (origin rear-left-bottom, x forward, y to the right)."""
        points = np.atleast_2d(np.asarray(points, float))
        return np.c_[points[:, 0] + self.size[0] / 2, self.size[1] / 2 - points[:, 1], points[:, 2]]

    def from_interface(self, points):
        points = np.atleast_2d(np.asarray(points, float))
        return np.c_[points[:, 0] - self.size[0] / 2, self.size[1] / 2 - points[:, 1], points[:, 2]]

    def part_colors(self):
        """Display colours (rgb 0..1): the type colour of vehicles.yaml, a lighter bonnet, dark mirrors and wheels."""
        body = np.array(self.spec['color'], dtype=float)
        return {'body': body, 'bonnet': np.minimum(body * 1.25, 1.0), 'mirror_l': (0.12, 0.12, 0.14),
                'mirror_r': (0.12, 0.12, 0.14), 'wheel': (0.05, 0.05, 0.05)}

    def export_glb(self, path):
        """glTF binary for the web viewer (rokey-b3-project1 viewer3d.js): one named mesh per part, mm, in
        the INTERFACE frame with Y up (origin rear-left-bottom, +X front, +Z to the right)."""
        import json
        import struct
        blob, views, accessors, meshes, materials, nodes = bytearray(), [], [], [], [], []
        colors = self.part_colors()
        for index, part in enumerate(dict.fromkeys(self.parts)):
            triangles = self.triangles[self.parts == part]
            interface = self.to_interface(triangles.reshape(-1, 3))
            points = np.c_[interface[:, 0], interface[:, 2], interface[:, 1]].astype(np.float32)  # (X, up, right)
            normal = np.cross(points[1::3] - points[0::3], points[2::3] - points[0::3])
            normal = np.repeat(normal / np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12), 3, axis=0)
            for values in (points, normal.astype(np.float32)):
                views.append({'buffer': 0, 'byteOffset': len(blob), 'byteLength': values.nbytes, 'target': 34962})
                accessors.append({'bufferView': len(views) - 1, 'componentType': 5126, 'count': len(values),
                                  'type': 'VEC3', 'min': values.min(0).tolist(), 'max': values.max(0).tolist()})
                blob += values.tobytes()
            materials.append({'name': part, 'pbrMetallicRoughness': {
                'baseColorFactor': [*map(float, colors[part]), 1.0], 'metallicFactor': 0.15, 'roughnessFactor': 0.55}})
            meshes.append({'name': part, 'primitives': [{'attributes': {'POSITION': 2 * index, 'NORMAL': 2 * index + 1},
                                                         'material': index}]})
            nodes.append({'name': part, 'mesh': index})
        document = {'asset': {'version': '2.0', 'generator': 'repair_car vehicle_model'}, 'scene': 0,
                    'scenes': [{'nodes': list(range(len(nodes)))}], 'nodes': nodes, 'meshes': meshes,
                    'materials': materials, 'accessors': accessors, 'bufferViews': views,
                    'buffers': [{'byteLength': len(blob)}]}
        text = json.dumps(document).encode()
        text += b' ' * (-len(text) % 4)
        blob += bytes(-len(blob) % 4)
        chunks = struct.pack('<II', len(text), 0x4E4F534A) + text + struct.pack('<II', len(blob), 0x004E4942) + blob
        Path(path).write_bytes(struct.pack('<III', 0x46546C67, 2, 12 + len(chunks)) + chunks)

    def export(self, directory):
        """One STL (mm) per part, for Gazebo, RViz and the web viewer."""
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        paths = {}
        for part in dict.fromkeys(self.parts):
            paths[part] = directory / f'{self.kind}_{part}.stl'
            write_stl(paths[part], self.triangles[self.parts == part])
        return paths
