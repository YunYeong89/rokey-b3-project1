"""Point-cloud ball paths with sparse IK and local probe clearance, without arm/transit validation."""
import numpy as np
from scipy.ndimage import binary_dilation, binary_fill_holes, distance_transform_edt, gaussian_filter, map_coordinates
from scipy.spatial import cKDTree

from repair_car.inspection.part_scan.cloud_clearance import CloudProbeClearance


class CloudSurface:
    geometry_source = 'pointcloud'

    def __init__(self, points, part, radius=1., obstacles=None, tool=()):
        self.points, self.part = np.asarray(points, float), part
        self.radius = radius
        self.obstacles = np.empty((0, 3)) if obstacles is None else np.asarray(obstacles, float)
        self.tool = tool


class CloudGrid:
    """Smoothed outer height field and normals derived solely from the displayed cloud."""
    def __init__(self, surface, axis, settings):
        self.axis, self.step = np.asarray(axis, float), settings['grid_mm']
        u = np.array([0., 1., 0.]) if abs(self.axis[1]) < .9 else np.array([1., 0., 0.])
        self.u = u - self.axis * (u @ self.axis)
        self.u /= np.linalg.norm(self.u)
        self.v = np.cross(self.axis, self.u)
        points = surface.points
        if abs(self.axis[2]) < .9:
            centre = (points.min(0) + points.max(0)) / 2
            points = points[(points - centre) @ self.axis > 0]
        if not len(points):
            raise ValueError('No point-cloud samples face the selected approach axis')
        uv = np.c_[points @ self.u, points @ self.v]
        self.radius, self.clearance_margin = surface.radius, settings['clearance_margin_mm']
        self.cloud_uv, self.cloud_height = uv, points @ self.axis
        self.cloud_tree = cKDTree(uv)
        self.origin = uv.min(0) - self.step
        cells = np.floor((uv - self.origin) / self.step).astype(int)
        height = np.full(cells.max(0) + 2, -np.inf)
        np.maximum.at(height, (cells[:, 0], cells[:, 1]), points @ self.axis)
        known = np.isfinite(height)
        distances, nearest = distance_transform_edt(~known, return_indices=True)
        filled = height[tuple(nearest)]
        valid = known | (binary_fill_holes(known) & (distances * self.step <= settings['gap_fill_mm']))
        sigma = settings['smooth_mm'] / self.step
        self.height = gaussian_filter(filled, sigma=sigma)
        gu, gv = np.gradient(self.height, self.step)
        normals = self.axis - gu[..., None] * self.u - gv[..., None] * self.v
        self.normals = normals / np.linalg.norm(normals, axis=2, keepdims=True)
        self.valid = valid
        distance = distance_transform_edt(np.pad(valid, 1))[1:-1, 1:-1] * self.step
        self.inner = valid & (distance > settings['edge_margin_mm'])
        self.occluded = np.zeros_like(valid)
        if len(surface.obstacles):
            obstacles = surface.obstacles
            indices = np.floor((np.c_[obstacles @ self.u, obstacles @ self.v] - self.origin) / self.step).astype(int)
            keep = np.all((indices >= 0) & (indices < valid.shape), axis=1)
            indices, obstacles = indices[keep], obstacles[keep]
            outer = np.full_like(height, -np.inf)
            np.maximum.at(outer, (indices[:, 0], indices[:, 1]), obstacles @ self.axis)
            self.occluded = binary_dilation(outer > filled + settings['occlusion_tolerance_mm'],
                                             iterations=max(1, int(np.ceil(surface.radius / self.step))))
            self.inner &= ~self.occluded
        self.tool_excluded, self.probe = 0, None
        if surface.tool:
            cells = np.argwhere(self.inner)
            centres, normals = self.sample_ball(cells)
            self.probe = check = CloudProbeClearance(np.vstack((surface.points, surface.obstacles)), surface.tool)
            blocked = check.blocked(centres - surface.radius * normals, normals)
            self.inner[tuple(cells[blocked].T)] = False
            self.tool_excluded = int(blocked.sum())

    def support_height(self, uv, normals):
        """TCP height clearing the raw cloud with the actual ball, independently of normal smoothing."""
        centres = np.asarray(uv) + self.radius * np.c_[normals @ self.u, normals @ self.v]
        return self.centre_height(centres, normals) - self.radius * (normals @ self.axis) + self.clearance_margin

    def centre_height(self, centres, normals):
        """Tangent support removes the artificial sphere-cap scallops between cloud samples."""
        neighbours = self.cloud_tree.query_ball_point(centres, self.radius)
        heights = np.empty(len(centres))
        axial = np.maximum(normals @ self.axis, .05)
        slopes = -np.c_[normals @ self.u, normals @ self.v] / axial[:, None]
        for k, ids in enumerate(neighbours):
            if ids:
                # Tangent-plane support is an upper bound on each point's sphere cap and avoids sampling ripples.
                delta = self.cloud_uv[ids] - centres[k]
                heights[k] = np.max(self.cloud_height[ids] - delta @ slopes[k]) + self.radius / axial[k]
            else:
                _, nearest = self.cloud_tree.query(centres[k])
                delta = self.cloud_uv[nearest] - centres[k]
                heights[k] = self.cloud_height[nearest] - delta @ slopes[k] + self.radius / axial[k]
        return heights

    def sample(self, cells):
        cells = np.asarray(cells, float)
        uv = self.origin + (cells + .5) * self.step
        normals = np.stack([map_coordinates(self.normals[:, :, k], cells.T, order=1, mode='nearest')
                            for k in range(3)], axis=1)
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)
        h = self.support_height(uv, normals)
        valid = map_coordinates(self.valid.astype(float), cells.T, order=0, mode='constant', cval=0.) > .5
        positions = uv[:, :1] * self.u + uv[:, 1:] * self.v + h[:, None] * self.axis
        return positions, normals, valid

    def sample_ball(self, cells):
        """Sphere centre clearing the cloud; its path is independent of the gripper orientation."""
        cells = np.asarray(cells, float)
        uv = self.origin + (cells + .5) * self.step
        normals = np.stack([map_coordinates(self.normals[:, :, k], cells.T, order=1, mode='nearest')
                            for k in range(3)], axis=1)
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)
        heights = self.centre_height(uv, normals)
        centres = (uv[:, :1] * self.u + uv[:, 1:] * self.v
                   + heights[:, None] * self.axis)
        return centres, normals

