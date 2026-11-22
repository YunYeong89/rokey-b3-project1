"""Mesh-normal raster planning, with full-pose continuous IK and mesh clearance."""
from dataclasses import dataclass

import numpy as np

from ..rotations import align
from .mesh_surface import MeshGrid
from .cloud_path import CloudPath
from .planner import PartPlan, per_cell
from .tool import ToolClearance
from .trajectory import PoseTrajectory


class MeshClearance(ToolClearance):
    def __init__(self, surface, scan, table_z, rack, box):
        super().__init__(np.empty((0, 3)), scan['tool'], scan['robot_base_mm'], table_z)
        self.tree = surface.tree
        self.rack = np.asarray(rack[:3])
        angle = np.radians(rack[3])
        self.rack_rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                                       [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        self.sample_step = scan['mesh_normal']['clearance_step_mm']
        self.rack_box = np.asarray(box, float)  # [low, high] mm in the rack frame (ScanScene: rack_box_mm)
        corners = np.array([[x, y, z] for x in self.rack_box[:, 0] for y in self.rack_box[:, 1]
                            for z in self.rack_box[:, 2]])
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
            delta = np.maximum(np.maximum(self.rack_box[0] - local, local - self.rack_box[1]), 0.)
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


def in_scan_roi(points, scan, surface):
    boxes = scan.get('roi_local_mm') or scan.get('roi_base_mm')
    if not boxes:
        return np.ones(len(points), bool)
    bounds = np.asarray(boxes).reshape(-1, 2, 2)
    uv = (surface.roi_coordinates(points) if scan.get('roi_local_mm') else points[:, :2])[:, None]
    return np.any(np.all((uv >= bounds[:, 0] - 1e-7) & (uv <= bounds[:, 1] + 1e-7), axis=2), axis=1)


class MeshNormalPlan:
    """Raster strokes on the mesh like the point-cloud plan: rounded raster turns (turn_radius_mm), the ball centre on
    the mesh offset (contact_preload_mm pressed in, surface_offset_mm outward first), the tool axis along the mesh
    normal smoothed like a curved surface (normal_smooth_mm, soft limit normal_smoothing_tolerance_deg). One path per
    connected region of a pass: a pose that fails (IK, tool/arm clearance, interpolation off the mesh, transit) or a
    join over an opening, crease or surface step takes its cells out of the pass, which is rastered again."""
    ACCESS_MM = None  # accessible-cell pose check on this grid (mm); None: real_scan.ik_mm

    def __init__(self, surface, axes, reach, scan, travel_z, log=None):
        self.surface, self.reach, self.scan = surface, reach, scan
        self.travel_z = travel_z
        self.paths, self.maps, self.grids, self.rejected = [], [], [], []
        self.diagnostics = []
        self.log = log or (lambda message: None)
        settings = scan['mesh_normal']
        self.depth = settings['surface_offset_mm'] - settings['contact_preload_mm']
        margin = scan['edge_margin_mm'].get(surface.part, scan['edge_margin_mm']['default'])
        for axis in axes:
            grid = MeshGrid(surface, axis, scan['step_mm'], margin)
            self.grids.append((axis, grid))
            mask = grid.inner.copy()
            self.log(f'mesh axis {axis.tolist()}: {int(mask.sum())} interior cells; checking tool/IK/arm clearance')
            i, j = np.nonzero(mask)
            if len(i):
                points, normals, _ = grid.sample(np.c_[i, j])
                keep = self.scan_area(points)
                mask[i[~keep], j[~keep]] = False
                i, j, points, normals = i[keep], j[keep], points[keep], normals[keep]
                mask[i, j] = per_cell(points, self.ACCESS_MM or scan['ik_mm'], lambda k:
                    reach.pose(*self.probe_pose(points[k], normals[k]), reach.home) is not None)
            self.log(f'mesh axis {axis.tolist()}: {int(mask.sum())} accessible cells; pose exclusions {reach.failures}')
            for along in ((0,) if scan.get('roi_base_mm') else (0, 1)):
                usable = mask.copy()
                while True:
                    cells_paths = self.passes(usable, grid.step, along)
                    try:
                        paths = [path for cells in cells_paths for path in self.pieces(cells, axis, grid)]
                        break
                    except ValueError as error:
                        self.rejected.append(str(error))
                        self.exclude(usable, grid, error.position)
                self.log(f'mesh pass {along}: {len(paths)} path(s) from {len(cells_paths)} raster region(s), '
                         f'{int(mask.sum() - usable.sum())} cells taken out')
                self.paths += paths
                self.maps.append((axis, grid, along, usable, cells_paths))
                self.diagnostics.append(dict(axis=axis.tolist(), along=along,
                                             surface_cells=int((grid.faces >= 0).sum()),
                                             interior_cells=int(grid.inner.sum()), accessible_cells=int(mask.sum()),
                                             scanned_cells=int(usable.sum()), raster_paths=len(cells_paths),
                                             paths=len(paths)))

    def passes(self, valid, step, along):
        return PartPlan.passes(self, valid, step, along)

    def scan_area(self, points):
        return in_scan_roi(points, self.scan, self.surface)

    def probe_pose(self, point, normal):
        return point + self.depth * normal, align(self.reach.frame, normal, np.pi)

    def exclude(self, usable, grid, position):
        """Take the cells within ik_mm / 2 of a failed pose (at least the nearest one) out of the pass."""
        i, j = np.nonzero(usable)
        distance = np.linalg.norm(grid.origin + (np.c_[i, j] + .5) * grid.step
                                  - [position @ grid.u, position @ grid.v], axis=1)
        out = distance <= self.scan['ik_mm'] / 2
        out[np.argmin(distance)] = True
        usable[i[out], j[out]] = False

    def pieces(self, cells, axis, grid):
        """The stroke along one raster (rounded turns); a ValueError at an opening, crease or surface step."""
        settings = self.scan['mesh_normal']
        # Fractional cells are queried on triangles; no nearest-cell height approximation.
        cells = CloudPath(settings['turn_radius_mm'], grid.step / 2).sample(cells, grid.step)
        points, normals, faces = grid.sample(cells)
        valid = faces >= 0
        distance = np.linalg.norm(np.diff(points, axis=0), axis=1)
        angle = np.sum(normals[:-1] * normals[1:], axis=1)
        breaks = (~valid[:-1] | ~valid[1:] | (distance > settings['max_surface_step_mm']) |
                  (angle < np.cos(np.radians(settings['crease_deg']))))
        if breaks.any():
            k = np.flatnonzero(breaks)[0]
            failure = ValueError('Mesh raster crosses an opening, crease or surface step')
            uv = grid.origin + (cells[k:k + 2].mean(0) + .5) * grid.step  # between the two samples; only u, v count
            failure.position = uv[0] * grid.u + uv[1] * grid.v
            raise failure
        stroke = self.stroke(points, normals, axis, grid)
        return [] if stroke is None else [stroke]

    def stroke(self, surface, normals, axis, grid):
        """A checked MeshPath over mesh points with their normals, None when shorter than min_path_mm; a ValueError
        with the `position` (BASE mm) of the first pose that fails."""
        if len(surface) < 2 or np.linalg.norm(np.diff(surface, axis=0), axis=1).sum() < self.scan['min_path_mm']:
            return None
        settings = self.scan['mesh_normal']
        radius, offset = settings['ball_radius_mm'], self.depth + settings['ball_radius_mm']
        tool = CloudPath.normals(normals, settings['normal_smooth_mm'], self.scan['step_mm'] / 2,
                                 settings['normal_smoothing_tolerance_deg'])
        positions = surface + offset * normals - radius * tool  # ball centre on the offset, TCP along the tool axis
        rotations, previous = [], self.reach.frame
        for normal in tool:
            previous = align(previous, normal, np.pi)
            rotations.append(previous)
        rotations = np.asarray(rotations)
        # Rounded turns are driven through; a stop only where the stroke reverses. Surface curvature is the spline's.
        delta = np.diff(surface - np.outer(surface @ axis, axis), axis=0)
        tangent = delta / np.maximum(np.linalg.norm(delta, axis=1, keepdims=True), 1e-12)
        corners = np.flatnonzero(np.sum(tangent[:-1] * tangent[1:], axis=1) < -.5) + 1
        tolerance = np.cos(np.radians(settings['normal_smoothing_tolerance_deg'] + settings['normal_interp_error_deg']))

        def geometry(points, orientations):
            """Interpolated ball centres on the mesh offset (contact found along its normal) and tool axes within the
            smoothing tolerance of it, over triangles."""
            centres = points - radius * orientations[:, :, 2]
            uv = np.c_[centres @ grid.u, centres @ grid.v]
            local = np.zeros_like(centres)
            for _ in range(3):
                contact = uv - offset * np.c_[local @ grid.u, local @ grid.v]
                reference, local, faces = grid.sample((contact - grid.origin) / grid.step - .5)
            error = np.abs((centres - reference) @ axis - offset * (local @ axis))
            bad = ((faces < 0) | (error > settings['offset_check_mm']) |
                   (np.sum(-orientations[:, :, 2] * local, axis=1) < tolerance))
            if bad.any():
                k = np.argmax(bad)
                failure = ValueError('Interpolated mesh path ' + ('crosses an opening' if faces[k] < 0 else
                                     'leaves the mesh offset' if error[k] > settings['offset_check_mm']
                                     else 'turns the tool beyond the smoothing tolerance'))
                failure.position = points[k]
                raise failure
            return True

        return self.validate_stroke(positions, rotations, normals, axis, corners, geometry)

    def validate_stroke(self, positions, rotations, normals, axis, corners, geometry):
        first, last = self.ends(positions, rotations)
        positions, rotations, normals = positions[first:last + 1], rotations[first:last + 1], normals[first:last + 1]
        corners = corners[(corners > first) & (corners < last)] - first
        if len(positions) < 2 or np.linalg.norm(np.diff(positions, axis=0), axis=1).sum() < self.scan['min_path_mm']:
            return None
        trajectory = PoseTrajectory(positions, rotations, self.scan['mesh_normal'], corners)
        initial = self.reach.pose(positions[0], rotations[0], self.reach.home)
        trajectory.validate(self.reach, initial, self.scan['speed_mm_s'],
                            self.scan['measurable']['joint_jump_deg'], geometry)
        return MeshPath(positions, normals, axis, trajectory)

    def ends(self, positions, rotations, *, entry_mask=None):
        """The stroke kept from its first to its last point that can be approached and left (the withdrawal, turn and
        vertical travel of transit), stepping in by ik_mm: a blocked end shortens the stroke, as the default plan
        trims, instead of a re-raster per blocked end cell."""
        step = max(1, int(round(self.scan['ik_mm'] / max(np.median(np.linalg.norm(np.diff(positions, axis=0), axis=1)),
                                                          1e-6))))

        def clear(k):
            seed = self.reach.pose(positions[k], rotations[k], self.reach.home)
            if seed is None:
                return False
            try:
                self.transit(positions[k], rotations[k], seed)
                return True
            except ValueError:
                return False

        entries = range(0, len(positions), step)
        if entry_mask is not None:
            entries = (k for k in entries if entry_mask[k])
        first = next((k for k in entries if clear(k)), None)
        if first is None:
            failure = ValueError('No approachable point on the mesh path')
            failure.position = positions[len(positions) // 2]
            raise failure
        last = next(k for k in [*range(len(positions) - 1, first, -step), first] if clear(k))
        return first, last

    def transit(self, tip, rotation, seed, *, withdrawal_axis=None):
        """Check the entire endpoint withdrawal, turn and vertical travel before accepting a stroke."""
        axis = -rotation[:, 2] if withdrawal_axis is None else withdrawal_axis
        down = self.reach.frame
        out = tip + self.scan['standoff_mm'] * axis
        high = np.r_[out[:2], max(self.travel_z, out[2])]
        margin = self.scan.get('travel_margin_mm', 0.)
        if margin and not self.reach.within_outer_reach(high[None], np.array([0., 0., 1.]), margin)[0]:
            raise ValueError(f'Lifted entry/exit within {margin:g} mm of the outer reach')
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
                f'{duration / 60:.1f} min strokes; pressed along the TCP axis')
