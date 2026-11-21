"""Point-cloud ball paths with sparse IK and local probe clearance, without arm/transit validation."""
import numpy as np
from scipy.ndimage import binary_dilation, binary_fill_holes, distance_transform_edt, gaussian_filter, map_coordinates
from scipy.spatial import cKDTree

from ..rotations import align
from .mesh_plan import MeshPath
from .planner import PartPlan
from .trajectory import PoseTrajectory
from .cloud_path import CloudPath
from .cloud_clearance import CloudProbeClearance


class CloudSurface:
    geometry_source = 'pointcloud'

    def __init__(self, points, part, radius=1., obstacles=None, tool=()):
        self.points, self.part = np.asarray(points, float), part
        self.radius = radius
        self.obstacles = np.empty((0, 3)) if obstacles is None else np.asarray(obstacles, float)
        self.tool = tool


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


class CloudNormalPlan:
    """One path per pass (horizontal, then vertical, per axis) over the cloud's interior cells, as the fixed-axis plan:
    a segment per connected region (`paths`; the probe lifted only between them). The TCP follows the ball's support
    on the raw cloud with the tool along the smoothed normal; between two samples it never dips below the support
    (fine samples more than clearance_tolerance_mm above the chord of their neighbours are kept as samples too), so a
    stroke is never split. A pose out of reach takes the cells within half ik_step_mm of it out of the pass, which is
    then rastered again."""
    FINE = 4  # support sub-samples per surface sample

    def __init__(self, surface, axes, reach, scan, travel_z, log=None):
        self.surface, self.reach, self.scan, self.travel_z = surface, reach, scan, travel_z
        scan['mesh_normal']['ball_radius_mm'] = surface.radius
        self.paths, self.maps, self.grids, self.rejected, self.diagnostics = [], [], [], [], []
        self.log = log or (lambda message: None)
        settings = scan['mesh_normal']['pointcloud']
        for axis in axes:
            grid = CloudGrid(surface, axis, settings)
            self.grids.append((axis, grid))
            self.log(f'pointcloud axis {axis.tolist()}: {int(grid.inner.sum())} cells, '
                     f'{grid.step} mm grid; sparse IK, {grid.tool_excluded} probe-obstructed cells excluded')
            for along in (0, 1):
                mask = grid.inner.copy()
                while True:
                    rasters = PartPlan.passes(self, mask, grid.step, along)
                    try:
                        segments = [self.segment(cells, axis, grid) for cells in rasters]
                        break
                    except ValueError as error:
                        self.rejected.append(str(error))
                        i, j = np.nonzero(mask)
                        distance = np.linalg.norm(grid.origin + (np.c_[i, j] + .5) * grid.step
                                                  - [error.position @ grid.u, error.position @ grid.v], axis=1)
                        out = distance <= settings['ik_step_mm'] / 2
                        out[np.argmin(distance)] = True
                        mask[i[out], j[out]] = False
                segments = [segment for segment in segments if segment is not None]
                self.paths += segments
                self.diagnostics.append(dict(axis=axis.tolist(), along=along, surface_cells=int(grid.valid.sum()),
                                             interior_cells=int(grid.inner.sum()), scanned_cells=int(mask.sum()),
                                             occluded_cells=int(grid.occluded.sum()),
                                             probe_excluded_cells=grid.tool_excluded,
                                             raster_paths=len(rasters), segments=len(segments)))

    @property
    def planning_checks(self):
        return dict(ik_step_mm=self.scan['mesh_normal']['validation_step_mm'],
                    ik_angle_deg=self.scan['mesh_normal']['validation_angle_deg'],
                    tool_collision=bool(self.surface.tool), tool_collision_source='pointcloud_probe_only',
                    arm_collision=False, transit=False, mesh_projection=False,
                    ball_cloud_clearance=True, neighbouring_cloud_occlusion=bool(len(self.surface.obstacles)))

    def segment(self, cells, axis, grid):
        """The MeshPath along raster cells, None when shorter than min_path_mm; a ValueError with the `position`
        (BASE mm) of a pose out of reach."""
        settings = self.scan['mesh_normal']
        step = settings['pointcloud']['surface_sample_mm'] / self.FINE
        curve = CloudPath(settings['pointcloud']['turn_radius_mm'], step)
        cells = curve.sample(cells, grid.step)
        positions, surface_normals = grid.sample_ball(cells)
        # The tool axis follows the normal smoothed like a curved surface; the ball centre path stays on the cloud.
        normals = curve.normals(surface_normals, settings['normal_smooth_mm'], step,
                                settings['normal_smoothing_tolerance_deg'])
        delta = np.diff(cells, axis=0)
        tangent = delta / np.maximum(np.linalg.norm(delta, axis=1, keepdims=True), 1e-12)
        turn = np.sum(tangent[:-1] * tangent[1:], axis=1)
        hard_corners = np.flatnonzero(turn < .9) + 1  # passed exactly; a stop only where the stroke reverses
        reversals = np.flatnonzero(turn < -.5) + 1
        height, index = positions @ axis, np.arange(len(positions))
        keep = np.unique(np.r_[index[::self.FINE], index[-1], hard_corners])
        while True:  # the stroke between kept samples clears every fine one
            above = height - np.interp(index, keep, height[keep]) > settings['pointcloud']['clearance_tolerance_mm']
            if not above.any():
                break
            keep = np.union1d(keep, index[above])
        positions, normals, surface_normals = positions[keep], normals[keep], surface_normals[keep]
        if np.linalg.norm(np.diff(positions, axis=0), axis=1).sum() < self.scan['min_path_mm']:
            return None
        targets = positions + (settings['surface_offset_mm'] - settings['contact_preload_mm']
                               - self.surface.radius) * normals
        if grid.probe is not None:  # the probe body at the smoothed tool axis, every 2 mm
            check = np.r_[np.arange(0, len(positions), self.FINE), len(positions) - 1]
            blocked = grid.probe.blocked(positions[check] - self.surface.radius * normals[check], normals[check])
            if blocked.any():
                error = ValueError('Probe body meets the cloud at the smoothed tool axis')
                error.position = targets[check[np.argmax(blocked)]]
                raise error
        rotations, previous = [], self.reach.frame
        for normal in normals:
            previous = align(previous, normal, np.pi)
            rotations.append(previous)
        corners = np.searchsorted(keep, reversals)
        trajectory = PoseTrajectory(targets, np.asarray(rotations), settings, corners)
        initial = self.reach.pose(targets[0], rotations[0], self.reach.home)
        if initial is None:
            error = ValueError('Point-cloud path start is unreachable')
            error.position = targets[0]
            raise error
        trajectory.validate(self.reach, initial, self.scan['speed_mm_s'], self.scan['measurable']['joint_jump_deg'])
        return MeshPath(targets, surface_normals, axis, trajectory)

    def summary(self):
        samples = sum(path.trajectory.dense_count for path in self.paths)
        checkpoints = sum(len(span.knots) for path in self.paths for span in path.trajectory.spans)
        duration = sum(path.trajectory.duration for path in self.paths)
        passes = sum(1 for d in self.diagnostics if d.get('segments'))
        return (f'cloud-normal: {passes} paths ({len(self.paths)} segments, lifted between), {samples} samples -> '
                f'{checkpoints} checkpoints, '
                f'{duration / 60:.1f} min strokes; cloud probe clearance + sparse IK; pressed along the TCP axis')
