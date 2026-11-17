"""Mesh-normal raster planning, with full-pose continuous IK and mesh clearance."""
from dataclasses import dataclass

import numpy as np

from ..rotations import align
from .mesh_surface import MeshGrid
from .planner import PartPlan, densify, per_cell
from .tool import ToolClearance
from .trajectory import PoseTrajectory


class MeshClearance(ToolClearance):
    def __init__(self, surface, scan, table_z, rack):
        super().__init__(np.empty((0, 3)), scan['tool'], scan['robot_base_mm'], table_z)
        self.tree = surface.tree
        self.rack = np.asarray(rack[:3])
        angle = np.radians(rack[3])
        self.rack_rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                                       [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        self.sample_step = scan['mesh_normal']['clearance_step_mm']
        corners = np.array([[x, y, z] for x in (-14., 14.) for y in (-30., 30.) for z in (-16., -13.)])
        faces = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
                          [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                          [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
        self.rack_triangles = (corners @ self.rack_rotation.T + self.rack)[faces]
        self.tool_checks, self.arm_checks = {}, {}

    def segment_clear(self, start, end, radius):
        length = np.linalg.norm(end - start)
        count = max(1, int(np.ceil(length / self.sample_step)))
        # The expanded sampled balls cover the entire capsule, including between samples.
        expanded = np.hypot(radius, length / count / 2)
        for point in np.linspace(start, end, count + 1):
            local = (point - self.rack) @ self.rack_rotation
            delta = np.maximum(np.maximum(np.array([-14., -30., -16.]) - local,
                                           local - [14., 30., -13.]), 0.)
            if np.linalg.norm(delta) < expanded or not self.tree.clear(point, expanded):
                return False
        return True

    def clear(self, tip, axis):
        key = np.r_[tip, axis].tobytes()
        if key not in self.tool_checks:
            self.tool_checks[key] = self.tool_clear(tip, axis)
        return self.tool_checks[key]

    def tool_clear(self, tip, axis):
        if not super().clear(tip, axis):
            return False
        if (hasattr(self, 'ball_radius') and
                not self.tree.clear(tip + self.ball_radius * axis, self.ball_radius - self.ball_penetration)):
            return False
        return all(self.tree.cylinder_clear(tip, axis, a, b, radius) and
                   self.tree.cylinder_triangles_clear(self.rack_triangles, tip, axis, a, b, radius)
                   for a, b, radius in self.cylinders)

    def capsule_clear(self, a, b, radius):
        key = np.r_[a, b, radius].tobytes()
        if key not in self.arm_checks:
            self.arm_checks[key] = min(a[2], b[2]) - radius > self.table and self.segment_clear(a, b, radius)
        return self.arm_checks[key]


@dataclass
class MeshPath:
    positions: np.ndarray
    normals: np.ndarray
    axis: np.ndarray
    trajectory: PoseTrajectory


class MeshNormalPlan:
    def __init__(self, surface, axes, reach, scan, travel_z, log=None):
        self.surface, self.reach, self.scan = surface, reach, scan
        self.travel_z = travel_z
        self.paths, self.maps, self.grids, self.rejected = [], [], [], []
        self.diagnostics = []
        self.log = log or (lambda message: None)
        settings = scan['mesh_normal']
        margin = scan['edge_margin_mm'].get(surface.part, scan['edge_margin_mm']['default'])
        for axis in axes:
            grid = MeshGrid(surface, axis, scan['step_mm'], margin)
            self.grids.append((axis, grid))
            mask = grid.inner.copy()
            self.log(f'mesh axis {axis.tolist()}: {int(mask.sum())} interior cells; checking tool/IK/arm clearance')
            i, j = np.nonzero(mask)
            if len(i):
                points, normals, _ = grid.sample(np.c_[i, j])
                points += settings['surface_offset_mm'] * normals
                mask[i, j] = per_cell(points, scan['ik_mm'], lambda k:
                    reach.pose(points[k], align(reach.frame, normals[k], np.pi), reach.home) is not None)
            self.log(f'mesh axis {axis.tolist()}: {int(mask.sum())} accessible cells; pose exclusions {reach.failures}')
            for along in (0, 1):
                cells_paths = PartPlan.passes(self, mask, grid.step, along)
                self.log(f'mesh pass {along}: {len(cells_paths)} raster candidates; validating interpolated poses')
                self.diagnostics.append(dict(axis=axis.tolist(), along=along,
                                             surface_cells=int((grid.faces >= 0).sum()),
                                             interior_cells=int(grid.inner.sum()),
                                             accessible_cells=int(mask.sum()), raster_paths=len(cells_paths)))
                self.maps.append((axis, grid, along, mask, cells_paths))
                for cells in cells_paths:
                    # Fractional joins are queried on triangles; no nearest-cell height approximation.
                    cells = densify(cells)
                    points, normals, faces = grid.sample(cells)
                    valid = faces >= 0
                    distance = np.linalg.norm(np.diff(points, axis=0), axis=1)
                    angle = np.sum(normals[:-1] * normals[1:], axis=1)
                    breaks = (~valid[:-1] | ~valid[1:] |
                              (distance > settings['max_surface_step_mm']) |
                              (angle < np.cos(np.radians(settings['crease_deg']))))
                    cuts = np.r_[0, np.flatnonzero(breaks) + 1, len(points)]
                    for start, end in zip(cuts[:-1], cuts[1:]):
                        own = np.arange(start, end)[valid[start:end]]
                        self.add(points[own] + settings['surface_offset_mm'] * normals[own], normals[own], axis, grid)

    def add(self, positions, normals, axis, grid):
        if len(positions) < 2 or np.linalg.norm(np.diff(positions, axis=0), axis=1).sum() < self.scan['min_path_mm']:
            return
        rotations, previous = [], self.reach.frame
        for normal in normals:
            previous = align(previous, normal, np.pi)
            rotations.append(previous)
        rotations = np.asarray(rotations)
        # Raster turns stop smoothly. Surface curvature itself is handled by the spline.
        projected = positions - np.outer(positions @ axis, axis)
        delta = np.diff(projected, axis=0)
        tangent = delta / np.maximum(np.linalg.norm(delta, axis=1, keepdims=True), 1e-12)
        corners = np.flatnonzero(np.sum(tangent[:-1] * tangent[1:], axis=1) < .9) + 1
        try:
            trajectory = PoseTrajectory(positions, rotations, self.scan['mesh_normal'], corners)
            initial = self.reach.pose(positions[0], rotations[0], self.reach.home)
            if initial is None:
                raise ValueError('Mesh path start is unreachable')
            def geometry(points, orientations):
                uv = np.c_[points @ grid.u, points @ grid.v]
                cells = (uv - grid.origin) / grid.step - .5
                reference, local_normals, faces = grid.sample(cells)
                settings = self.scan['mesh_normal']
                error = np.abs((points - reference) @ axis - settings['surface_offset_mm'] * (local_normals @ axis))
                alignment = np.sum(-orientations[:, :, 2] * local_normals, axis=1)
                return bool(np.all((faces >= 0) & (error <= settings['position_interp_error_mm']) &
                                   (alignment >= np.cos(np.radians(settings['normal_interp_error_deg'])))))

            last = trajectory.validate(self.reach, initial, self.scan['speed_mm_s'],
                                       self.scan['measurable']['joint_jump_deg'], geometry)
            self.transit(positions[0], rotations[0], initial)
            self.transit(positions[-1], rotations[-1], last)
            self.paths.append(MeshPath(positions, normals, axis, trajectory))
        except ValueError as error:
            self.rejected.append(str(error))
            # Preserve usable sections without bridging an IK/collision failure.
            middle = len(positions) // 2
            self.add(positions[:middle], normals[:middle], axis, grid)
            self.add(positions[middle:], normals[middle:], axis, grid)

    def transit(self, tip, rotation, seed):
        """Check the entire endpoint withdrawal, turn and vertical travel before accepting a stroke."""
        normal, down = -rotation[:, 2], self.reach.frame
        out = tip + self.scan['standoff_mm'] * normal
        high = np.r_[out[:2], max(self.travel_z, out[2])]
        poses = [(out, rotation), (out, down), (high, down)]
        q, position, orientation = seed, tip, rotation
        for target, following in poses:
            if np.linalg.norm(target - position) < 1e-8 and np.linalg.norm(following - orientation) < 1e-8:
                continue
            trajectory = PoseTrajectory(np.array([position, target]), np.array([orientation, following]),
                                        self.scan['mesh_normal'])
            q = trajectory.validate(self.reach, q, self.scan['mesh_normal']['approach_speed_mm_s'],
                                     self.scan['measurable']['joint_jump_deg'])
            position, orientation = target, following

    def summary(self):
        dense = sum(path.trajectory.dense_count for path in self.paths)
        checkpoints = sum(len(span.knots) for path in self.paths for span in path.trajectory.spans)
        duration = sum(path.trajectory.duration for path in self.paths)
        return (f'mesh-normal: {len(self.paths)} paths, {dense} reference samples -> {checkpoints} checkpoints, '
                f'{duration / 60:.1f} min strokes; force measurement only')
