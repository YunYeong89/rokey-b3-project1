"""Mesh height feedforward with a fixed probe orientation and force along one fixed BASE axis: the bonnet from above
(force along BASE -Z), a door turned toward the robot from BASE -x (force along +X, the TCP axis tilted
tool_tilt_deg toward straight down: along +x the wrist leaves the workspace). Planned and checked on the mesh like
mesh-normal (raster of the accessible mesh cells, IK, tool/arm/rack clearance, interpolated heights on the mesh, one
path per connected region of a pass), but the IK check is sparse: the tool never turns, so joints only feed the
timing, and a failed pose's re-raster costs seconds. "Height" is the depth along the fixed axis: the ball's support,
where a ball of the probe radius lowered along the axis first touches the mesh, not the mesh point straight below
(on a slope the ball would sink r (1 - cos) into the rise: past ball_penetration_mm at ~23 deg, which took the
character lines out of every bonnet row)."""
from copy import deepcopy

import numpy as np
from scipy.ndimage import map_coordinates, minimum_filter

from ..rotations import align
from .cloud import ball_bottom
from .mesh_plan import MeshNormalPlan
from .planner import PartPlan, sweep


def press_label(axis):
    """'-Z', '+X': the BASE direction pressed into the surface for the outward `axis`."""
    k = int(np.argmax(np.abs(axis)))
    return f'{"+" if axis[k] < 0 else "-"}{"XYZ"[k]}'


class MeshHeightPlan(MeshNormalPlan):
    controller = 'mesh-height'
    # Accessible cells checked every 2 mm, not ik_mm: a tool-collision strip (the bonnet's rear edge) slipped between
    # the 4 mm checks and was then excluded 2 mm per re-raster, each revalidating the pass (minutes).
    ACCESS_MM = 2.

    def __init__(self, surface, axes, reach, scan, travel_z, log=None):
        # One fixed axis (the side the probe comes from) and one TCP axis: tool_tilt_deg from the press direction
        # toward straight down (0: along the press, the bonnet).
        settings = scan['mesh_normal']
        self.axis = np.asarray(axes[0], float)
        press, down, tilt = -self.axis, np.array([0., 0., -1.]), np.radians(settings['tool_tilt_deg'])
        side = down - press * (down @ press)
        self.tool = np.cos(tilt) * press + (np.sin(tilt) * side / np.linalg.norm(side) if tilt else 0.)
        self.radius = settings['ball_radius_mm']
        self.door_max_z = None
        if surface.part == 'door':
            heights = surface.roi_coordinates(surface.triangles[surface.target].reshape(-1, 3))[:, 1]
            self.door_max_z = (heights.min() + heights.max()) / 2
        super().__init__(surface, axes, reach, scan, travel_z, log)
        if self.door_max_z is not None and len(self.paths) != 2:
            self.rejected.insert(0, '도어 하단에서 가로·세로 검사 경로를 모두 만들 수 없습니다')
            self.paths.clear()

    @staticmethod
    def configure(scan):
        scan = deepcopy(scan)
        settings = scan['mesh_normal']
        settings.update(scan['mesh_height'])
        settings.update(planning_source='mesh', position_interpolator='pchip')
        # Sparse IK along the strokes (0.5 mm made each re-raster revalidate a whole pass for minutes).
        settings['validation_step_mm'] = settings['pointcloud']['ik_step_mm']
        scan['measurable']['joint_jump_deg'] = settings['pointcloud']['joint_jump_deg']
        settings['pd']['enabled'] = False
        # The stream presses with and records contact by these (mesh-normal / cloud-normal: the default survey's).
        scan['default_scan']['survey']['force_n'] = settings['force_n']
        scan['default_scan']['contact_threshold_n'] = settings['record_threshold_n']  # measured points
        scan['default_scan']['real_contact_offset_n'] = settings['contact_threshold_n']  # real robot search (DSR)
        return scan

    def passes(self, valid, step, along):
        """Each pass one way only: pass 0 rows across (bonnet: BASE y) from the far side toward the robot (door: top
        down), pass 1 rows along it from the robot's right to its left; no segment visited out of the line order."""
        paths = PartPlan.passes(self, valid, step, along, join=sweep)
        if self.door_max_z is not None and paths:
            return [max(paths, key=len)]
        return paths

    def scan_area(self, points):
        allowed = super().scan_area(points)
        if self.door_max_z is not None:
            allowed &= self.surface.roi_coordinates(points)[:, 1] <= self.door_max_z
        return allowed

    def transit(self, tip, rotation, seed):
        return super().transit(tip, rotation, seed, withdrawal_axis=self.axis)

    def support(self, grid, points):
        """Ball-bottom height along the axis at the points' place across it (cloud.ball_bottom over the grid's mesh
        heights, once per grid); the mesh point's own height where the ball overhangs the grid's edge."""
        if not hasattr(grid, 'ball'):
            i, j = np.nonzero(np.isfinite(grid.height))
            bottom = ball_bottom(grid.origin + (np.c_[i, j] + .5) * grid.step, grid.height[i, j], grid.origin, grid.step,
                                 grid.height.shape, self.radius)
            grid.ball = np.where(np.isfinite(bottom), bottom, np.nan)
        cells = (np.c_[points @ grid.u, points @ grid.v] - grid.origin) / grid.step - .5
        bottom = map_coordinates(grid.ball, cells.T, order=1, cval=np.nan)
        return np.where(np.isfinite(bottom), bottom, points @ self.axis)

    def tcp(self, surface, grid):
        """TCP over mesh points: the ball centre (support + radius + depth) along the axis, the TCP a radius on along
        the tool axis (tilt 0: the ball bottom, depth out)."""
        across = surface - np.outer(surface @ self.axis, self.axis)
        centre = across + np.outer(self.support(grid, surface) + self.radius + self.depth, self.axis)
        return centre + self.radius * self.tool

    def contact(self, tcp):
        """The ball bottom along the axis under a TCP (the support, depth out)."""
        return tcp - self.radius * self.tool - self.radius * self.axis

    def rotation(self):
        return align(self.reach.frame, -self.tool, np.pi)

    def probe_pose(self, point, normal):
        # Only depth along the axis comes from the mesh; neither the tool nor the force axis follows its normals.
        # The grid being planned is the last one added (MeshNormalPlan).
        return self.tcp(point[None], self.grids[-1][1])[0], self.rotation()

    def ends(self, positions, rotations):
        """Start the bonnet path on a gently sloped patch with room for the probe footprint."""
        if self.surface.part != 'bonnet':
            return super().ends(positions, rotations)
        grid = self.grids[-1][1]
        settings = self.scan['mesh_normal']
        cells = (np.c_[positions @ grid.u, positions @ grid.v] - grid.origin) / grid.step - .5
        edge = map_coordinates(grid.edge_distance, cells.T, order=1)
        radius_cells = int(np.ceil(self.radius / grid.step))
        alignment = minimum_filter(grid.normal_grid @ self.axis, size=2 * radius_cells + 1)
        alignment = map_coordinates(alignment, cells.T, order=1)
        entry_mask = ((edge >= settings['bonnet_entry_margin_mm']) &
                      (alignment >= np.cos(np.radians(settings['bonnet_entry_tilt_deg']))))
        return super().ends(positions, rotations, entry_mask=entry_mask)

    def stroke(self, surface, normals, axis, grid):
        if len(surface) < 2 or np.linalg.norm(np.diff(surface, axis=0), axis=1).sum() < self.scan['min_path_mm']:
            return None
        settings = self.scan['mesh_normal']
        positions = self.tcp(surface, grid)
        rotations = np.repeat(self.rotation()[None], len(surface), axis=0)
        fixed_normals = np.repeat(self.axis[None], len(surface), axis=0)
        flat = surface - np.outer(surface @ self.axis, self.axis)  # the raster across the axis
        tangent = np.diff(flat, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
        corners = np.flatnonzero(np.sum(tangent[:-1] * tangent[1:], axis=1) < -.5) + 1

        def geometry(points, orientations):
            points = self.contact(points)
            cells = (np.c_[points @ grid.u, points @ grid.v] - grid.origin) / grid.step - .5
            _, _, faces = grid.sample(cells)
            error = np.abs(points @ self.axis - self.support(grid, points) - self.depth)
            bad = (faces < 0) | (error > settings['offset_check_mm'])
            bad |= ~self.scan_area(points)
            if bad.any():
                failure = ValueError('Interpolated height path crosses an opening or leaves the mesh height')
                failure.position = points[np.flatnonzero(bad)[0]]
                raise failure
            return True

        return self.validate_stroke(positions, rotations, fixed_normals, self.axis, corners, geometry)

    def summary(self):
        duration = sum(path.trajectory.duration for path in self.paths)
        tilt = self.scan['mesh_normal']['tool_tilt_deg']
        return (f'mesh-height: {len(self.paths)} paths, {duration / 60:.1f} min strokes; fixed tool orientation'
                f'{f" (TCP axis {tilt:g} deg toward down)" if tilt else ""}, mesh depth feedforward, '
                f'BASE {press_label(self.paths[0].axis) if self.paths else ""} force, PD OFF')
