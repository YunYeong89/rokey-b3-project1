"""Fast point-cloud normal planning: sparse IK, without tool/arm collision or transit validation."""
import numpy as np
from scipy.ndimage import binary_fill_holes, distance_transform_edt, gaussian_filter, map_coordinates
from scipy.spatial import cKDTree

from ..rotations import align
from .mesh_plan import MeshPath
from .planner import PartPlan
from .trajectory import PoseTrajectory


class CloudSurface:
    geometry_source = 'pointcloud'

    def __init__(self, points, part, radius=1.):
        self.points, self.part = np.asarray(points, float), part
        self.radius = radius


class CloudReach:
    """Joint limits and IK only. No scene geometry is accessed here."""
    def __init__(self, chain, home):
        self.chain, self.home = chain, home
        self.frame = chain.forward(home)[0][:3, :3]
        self.failures = dict(ik=0)

    def pose(self, position, rotation, seed):
        q = self.chain.solve(position / 1000, rotation, seed)
        if q is None:
            self.failures['ik'] += 1
        return q


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

    def support_height(self, uv, normals):
        """TCP height clearing the raw cloud with the actual ball, independently of normal smoothing."""
        centres = np.asarray(uv) + self.radius * np.c_[normals @ self.u, normals @ self.v]
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
                heights[k] = self.cloud_height[nearest] + self.radius
        return heights - self.radius * (normals @ self.axis) + self.clearance_margin

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


class CloudNormalPlan:
    def __init__(self, surface, axes, reach, scan, travel_z, log=None):
        self.surface, self.reach, self.scan, self.travel_z = surface, reach, scan, travel_z
        self.paths, self.maps, self.grids, self.rejected, self.diagnostics = [], [], [], [], []
        self.log = log or (lambda message: None)
        settings = scan['mesh_normal']['pointcloud']
        for axis in axes:
            grid = CloudGrid(surface, axis, settings)
            self.grids.append((axis, grid))
            self.log(f'pointcloud axis {axis.tolist()}: {int(grid.inner.sum())} cells, '
                     f'{grid.step} mm grid; sparse IK only')
            for along in (0, 1):
                paths = PartPlan.passes(self, grid.inner, grid.step, along)
                self.diagnostics.append(dict(axis=axis.tolist(), along=along, surface_cells=int(grid.valid.sum()),
                                             interior_cells=int(grid.inner.sum()), raster_paths=len(paths)))
                for cells in paths:
                    cells = np.concatenate([cells[:1], *[
                        np.linspace(a, b, max(1, int(np.ceil(np.linalg.norm(b - a) * grid.step /
                                                          settings['surface_sample_mm']))) + 1)[1:]
                        for a, b in zip(cells[:-1], cells[1:])]])
                    positions, normals, valid = grid.sample(cells)
                    distance = np.linalg.norm(np.diff(positions, axis=0), axis=1)
                    breaks = ~valid[:-1] | ~valid[1:] | (distance > settings['max_surface_step_mm'])
                    cuts = np.r_[0, np.flatnonzero(breaks) + 1, len(positions)]
                    for start, end in zip(cuts[:-1], cuts[1:]):
                        own = np.arange(start, end)[valid[start:end]]
                        self.add(positions[own], normals[own], axis, grid)

    @property
    def planning_checks(self):
        return dict(ik_step_mm=self.scan['mesh_normal']['validation_step_mm'],
                    ik_angle_deg=self.scan['mesh_normal']['validation_angle_deg'],
                    tool_collision=False, arm_collision=False, transit=False, mesh_projection=False,
                    ball_cloud_clearance=True)

    def add(self, positions, normals, axis, grid):
        if len(positions) < 2 or np.linalg.norm(np.diff(positions, axis=0), axis=1).sum() < self.scan['min_path_mm']:
            return
        settings = self.scan['mesh_normal']
        targets = positions + settings['surface_offset_mm'] * normals
        rotations, previous = [], self.reach.frame
        for normal in normals:
            previous = align(previous, normal, np.pi)
            rotations.append(previous)
        projected = targets - np.outer(targets @ axis, axis)
        delta = np.diff(projected, axis=0)
        tangent = delta / np.maximum(np.linalg.norm(delta, axis=1, keepdims=True), 1e-12)
        corners = np.flatnonzero(np.sum(tangent[:-1] * tangent[1:], axis=1) < .9) + 1
        tree, forced = cKDTree(targets), set()
        try:
            # Where the interpolated ball sinks into the cloud, the targets around it become checkpoints; where it
            # still sinks with all of them kept (a step no curve clears), the stroke is split there.
            while True:
                trajectory = PoseTrajectory(targets, np.asarray(rotations), settings, corners, sorted(forced))
                sinking = self.sinking(trajectory, grid, axis)
                if not len(sinking):
                    break
                near = tree.query(sinking)[1]
                added = set(np.clip(np.r_[near - 1, near, near + 1], 0, len(targets) - 1).tolist()) - forced
                if not added:
                    step = near[0]
                    self.rejected.append(f'Cloud ball clearance step at {targets[step].round(1).tolist()}; '
                                         f'split into separate strokes')
                    self.add(positions[:step], normals[:step], axis, grid)
                    self.add(positions[step + 1:], normals[step + 1:], axis, grid)
                    return
                forced |= added
            initial = self.reach.pose(targets[0], rotations[0], self.reach.home)
            if initial is None:
                raise ValueError('Point-cloud path start is unreachable')
            trajectory.validate(self.reach, initial, self.scan['speed_mm_s'],
                                self.scan['measurable']['joint_jump_deg'])
            self.paths.append(MeshPath(targets, normals, axis, trajectory))
        except ValueError as error:
            self.rejected.append(str(error))
            middle = len(positions) // 2
            self.add(positions[:middle], normals[:middle], axis, grid)
            self.add(positions[middle:], normals[middle:], axis, grid)

    def sinking(self, trajectory, grid, axis):
        """Interpolated TCP samples more than clearance_tolerance_mm below the ball's support on the raw cloud."""
        cloud = self.scan['mesh_normal']['pointcloud']
        sinking = []
        for span in trajectory.spans:
            _, sampled, orientations = span.validation_samples(cloud['surface_sample_mm'] / 2)
            required = grid.support_height(np.c_[sampled @ grid.u, sampled @ grid.v], -orientations[:, :, 2])
            sinking.append(sampled[required - sampled @ axis > cloud['clearance_tolerance_mm']])
        return np.concatenate(sinking)

    def summary(self):
        samples = sum(path.trajectory.dense_count for path in self.paths)
        checkpoints = sum(len(span.knots) for path in self.paths for span in path.trajectory.spans)
        duration = sum(path.trajectory.duration for path in self.paths)
        return (f'cloud-normal: {len(self.paths)} paths, {samples} samples -> {checkpoints} checkpoints, '
                f'{duration / 60:.1f} min strokes; sparse IK, collision/transit checks disabled; force measurement only')
