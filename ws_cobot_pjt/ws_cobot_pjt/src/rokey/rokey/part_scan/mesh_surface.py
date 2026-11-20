"""Triangle geometry in BASE millimetres; no reference point cloud is used."""
from collections import defaultdict

import numpy as np
from scipy.ndimage import distance_transform_edt


class TriangleTree:
    """AABB hierarchy for conservative tool/arm clearance against the actual mesh."""
    def __init__(self, triangles):
        self.triangles = triangles
        self.root = self.build(np.arange(len(triangles)))

    def build(self, ids):
        t = self.triangles[ids]
        low, high = t.min((0, 1)), t.max((0, 1))
        if len(ids) <= 24:
            return low, high, ids, None
        centres = t.mean(1)
        order = ids[np.argsort(centres[:, np.argmax(np.ptp(centres, axis=0))])]
        middle = len(order) // 2
        return low, high, self.build(order[:middle]), self.build(order[middle:])

    @staticmethod
    def distances(point, triangles):
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        ab, ac, ap = b - a, c - a, point - a
        aa, bb, cc = np.sum(ab * ab, 1), np.sum(ab * ac, 1), np.sum(ac * ac, 1)
        d, e = np.sum(ap * ab, 1), np.sum(ap * ac, 1)
        denominator = aa * cc - bb * bb
        safe = np.maximum(denominator, 1e-24)
        u, v = (cc * d - bb * e) / safe, (aa * e - bb * d) / safe
        projected = a + u[:, None] * ab + v[:, None] * ac
        inside = (denominator > 1e-24) & (u >= 0) & (v >= 0) & (u + v <= 1)
        result = np.where(inside, np.sum((point - projected)**2, 1), np.inf)
        for start, end in ((a, b), (b, c), (c, a)):
            delta = end - start
            fraction = np.clip(np.sum((point - start) * delta, 1) /
                               np.maximum(np.sum(delta * delta, 1), 1e-24), 0, 1)
            result = np.minimum(result, np.sum((point - start - fraction[:, None] * delta)**2, 1))
        return result

    def clear(self, point, radius):
        pending, squared = [self.root], radius**2
        while pending:
            low, high, left, right = pending.pop()
            if np.sum(np.maximum(np.maximum(low - point, point - high), 0)**2) > squared:
                continue
            if right is None:
                if np.any(self.distances(point, self.triangles[left]) < squared):
                    return False
            else:
                pending.extend((left, right))
        return True

    def candidates(self, low, high):
        pending, found = [self.root], []
        while pending:
            node_low, node_high, left, right = pending.pop()
            if np.any(node_high < low) or np.any(node_low > high):
                continue
            if right is None:
                found.append(left)
            else:
                pending.extend((left, right))
        return np.concatenate(found) if found else np.empty(0, int)

    @staticmethod
    def cylinder_triangles_clear(triangles, tip, axis, start, end, radius):
        """Clip triangles to the axial slab, then intersect their projection with the cylinder disk."""
        axial = (triangles - tip) @ axis
        triangles = triangles[(axial.max(1) >= start) & (axial.min(1) <= end)]
        u = np.cross(axis, [1., 0., 0.] if abs(axis[0]) < .9 else [0., 1., 0.])
        u /= np.linalg.norm(u)
        frame = np.stack((u, np.cross(axis, u), axis), axis=1)
        for triangle in (triangles - tip) @ frame:
            polygon = list(triangle)
            for level, sign in ((start, 1.), (end, -1.)):
                clipped = []
                for previous, current in zip(polygon[-1:] + polygon[:-1], polygon):
                    a, b = sign * (previous[2] - level), sign * (current[2] - level)
                    if (a >= 0) != (b >= 0):
                        clipped.append(previous + a / (a - b) * (current - previous))
                    if b >= 0:
                        clipped.append(current)
                polygon = clipped
                if not polygon:
                    break
            if not polygon:
                continue
            points = np.asarray(polygon)[:, :2]
            following = np.roll(points, -1, axis=0)
            delta = following - points
            fraction = np.clip(-np.sum(points * delta, 1) / np.maximum(np.sum(delta * delta, 1), 1e-24), 0, 1)
            if np.any(np.sum((points + fraction[:, None] * delta)**2, 1) <= radius**2):
                return False
            cross = points[:, 0] * following[:, 1] - points[:, 1] * following[:, 0]
            if abs(cross.sum()) > 1e-10 and (np.all(cross >= 0) or np.all(cross <= 0)):
                return False
        return True

    def cylinder_clear(self, tip, axis, start, end, radius):
        a, b = tip + start * axis, tip + end * axis
        rim = radius * np.sqrt(np.maximum(0., 1 - axis**2))
        ids = self.candidates(np.minimum(a, b) - rim, np.maximum(a, b) + rim)
        return self.cylinder_triangles_clear(self.triangles[ids], tip, axis, start, end, radius)


class MeshSurface:
    def __init__(self, vehicle, part, pose, crease_deg, side=None):
        self.triangles = vehicle.triangles @ pose[:3, :3].T + pose[:3, 3]
        self.target = vehicle.parts == part
        if part == 'door' and side:
            self.target &= vehicle.triangles[:, :, 1].mean(1) * (1 if side == 'left' else -1) > 0
        self.part, self.pose = part, pose
        cross = np.cross(self.triangles[:, 1] - self.triangles[:, 0],
                         self.triangles[:, 2] - self.triangles[:, 0])
        self.area = np.linalg.norm(cross, axis=1)
        self.normals = cross / np.maximum(self.area[:, None], 1e-12)
        _, indices = np.unique(self.triangles.reshape(-1, 3).round(5), axis=0, return_inverse=True)
        indices = indices.reshape(-1, 3)
        adjacent = defaultdict(list)
        for face in np.flatnonzero(self.target):
            for vertex in indices[face]:
                adjacent[vertex].append(face)
        self.vertex_normals = np.repeat(self.normals[:, None, :], 3, axis=1)
        threshold = np.cos(np.radians(crease_deg))
        for face in np.flatnonzero(self.target):
            for corner, vertex in enumerate(indices[face]):
                neighbours = np.asarray(adjacent[vertex])
                dot = self.normals[neighbours] @ self.normals[face]
                keep = np.abs(dot) >= threshold
                normal = ((self.normals[neighbours[keep]] * np.sign(dot[keep])[:, None]) *
                          self.area[neighbours[keep], None]).sum(0)
                self.vertex_normals[face, corner] = normal / max(np.linalg.norm(normal), 1e-12)
        self.tree = TriangleTree(self.triangles[self.area > 1e-10])

    def roi_coordinates(self, points):
        local = (points - self.pose[:3, 3]) @ self.pose[:3, :3]
        return local[:, (0, 2) if self.part == 'door' else (0, 1)]


class MeshGrid:
    """Exact outermost triangle intersections on a raster; fractional queries use triangles too."""
    def __init__(self, surface, axis, step, margin, bin_mm=5.):
        self.surface, self.axis, self.step, self.bin_mm = surface, np.asarray(axis), step, bin_mm
        u = np.array([0., 1., 0.]) if abs(axis[1]) < .9 else np.array([1., 0., 0.])
        self.u = u - self.axis * (u @ self.axis)
        self.u /= np.linalg.norm(self.u)
        self.v = np.cross(self.axis, self.u)
        self.projected = np.stack((surface.triangles @ self.u, surface.triangles @ self.v), axis=-1)
        target = self.projected[surface.target]
        self.origin = target.min((0, 1)) - step
        shape = np.ceil((target.max((0, 1)) - self.origin) / step).astype(int) + 2
        self.bins = defaultdict(list)
        self.low, self.high = self.projected.min(1), self.projected.max(1)
        region_low, region_high = self.origin, self.origin + shape * step
        relevant = np.flatnonzero(np.all(self.high >= region_low, 1) & np.all(self.low <= region_high, 1)
                                 & (np.abs(surface.normals @ self.axis) > 1e-8))
        for face in relevant:
            low = np.floor((np.maximum(self.low[face], region_low) - self.origin) / bin_mm).astype(int)
            high = np.floor((np.minimum(self.high[face], region_high) - self.origin) / bin_mm).astype(int)
            for i in range(low[0], high[0] + 1):
                for j in range(low[1], high[1] + 1):
                    self.bins[i, j].append(face)
        indices = np.indices(shape).reshape(2, -1).T
        positions, normals, faces = self.sample(indices)
        self.height = np.full(np.prod(shape), -np.inf)
        known = faces >= 0
        self.height[known] = positions[known] @ self.axis
        self.height = self.height.reshape(shape)
        self.normal_grid = normals.reshape(*shape, 3)
        self.faces = faces.reshape(shape)
        inside = (self.faces >= 0)
        padded = np.pad(inside, 1)
        self.edge_distance = distance_transform_edt(padded)[1:-1, 1:-1] * step
        self.inner = inside & (self.edge_distance > margin)

    def sample(self, cells):
        uv = self.origin + (np.asarray(cells) + .5) * self.step
        bins = np.floor((uv - self.origin) / self.bin_mm).astype(int)
        unique, inverse = np.unique(bins, axis=0, return_inverse=True)
        positions, normals, faces = np.zeros((len(uv), 3)), np.zeros((len(uv), 3)), np.full(len(uv), -1)
        for group, key in enumerate(unique):
            ids = np.asarray(self.bins.get(tuple(key), []), int)
            if not len(ids):
                continue
            rows = np.flatnonzero(inverse == group)
            a, b, c = self.projected[ids, 0], self.projected[ids, 1], self.projected[ids, 2]
            ab, ac = b - a, c - a
            den = ab[:, 0] * ac[:, 1] - ab[:, 1] * ac[:, 0]
            for chunk in np.array_split(rows, max(1, int(np.ceil(len(rows) * len(ids) / 200000)))):
                ap = uv[chunk, None] - a
                x = (ap[:, :, 0] * ac[:, 1] - ap[:, :, 1] * ac[:, 0]) / den
                y = (ab[:, 0] * ap[:, :, 1] - ab[:, 1] * ap[:, :, 0]) / den
                weights = np.stack((1 - x - y, x, y), axis=-1)
                hit = np.all(weights >= -1e-8, axis=2)
                heights = np.einsum('nfc,fc->nf', weights, self.surface.triangles[ids] @ self.axis)
                heights[~hit] = -np.inf
                pick = heights.argmax(1)
                face = ids[pick]
                valid = np.isfinite(heights[np.arange(len(chunk)), pick]) & self.surface.target[face]
                # On a side, never see the opposite door through a window.
                if abs(self.axis[2]) < .9:
                    centre = self.surface.triangles.reshape(-1, 3).mean(0)
                    valid &= heights[np.arange(len(chunk)), pick] > centre @ self.axis
                own, face, weight = chunk[valid], face[valid], weights[np.arange(len(chunk))[valid], pick[valid]]
                positions[own] = np.einsum('nc,ncd->nd', weight, self.surface.triangles[face])
                normal = np.einsum('nc,ncd->nd', weight, self.surface.vertex_normals[face])
                normal *= np.where(normal @ self.axis >= 0, 1., -1.)[:, None]
                normals[own] = normal / np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
                faces[own] = face
        return positions, normals, faces

    def positions(self, i, j):
        i, j = np.broadcast_arrays(np.atleast_1d(i), np.atleast_1d(j))
        return self.sample(np.c_[i, j])[0]
