"""Mesh height feedforward with a fixed probe orientation and force along one fixed BASE axis: the bonnet from above
(force along BASE -Z), a door turned toward the robot from BASE -x (force along +X, the TCP axis tilted
tool_tilt_deg toward straight down: along +x the wrist leaves the workspace). Planned and checked on the mesh like
shared mesh raster (accessible mesh cells, IK, tool/arm/rack clearance, interpolated heights on the mesh, one
path per connected region of a pass), but the IK check is sparse: the tool never turns, so joints only feed the
timing, and a failed pose's re-raster costs seconds. "Height" is the depth along the fixed axis: the ball's support,
where a ball of the probe radius lowered along the axis first touches the mesh, not the mesh point straight below
(on a slope the ball would sink r (1 - cos) into the rise: past ball_penetration_mm at ~23 deg, which took the
character lines out of every bonnet row)."""
from copy import deepcopy

import numpy as np
from scipy.ndimage import distance_transform_edt, label, map_coordinates, minimum_filter

from repair_car.robot_control.rotations import align
from repair_car.pointcloud.cloud import ball_bottom
from repair_car.inspection.part_scan.mesh_plan import MeshPlan
from repair_car.inspection.part_scan.planner import PartPlan, sweep


def door_panel(surface, grid, settings):
    """The door's flat panel under the window, as grid cells: under the door's mid height, the normal within
    flat_deg of the panel's (their median), the height within step_mm of a quadratic fit over them (twice: the handle,
    a crease off it), seam_mm inside any other cell (the door/body joint, the window line), no strip narrower than
    2 x narrow_mm, the largest region left, from its first row bottom_width as wide as its median row up, without its
    front_mm toward the vehicle's front. Cached on the grid."""
    if not hasattr(grid, 'panel'):
        i, j = np.nonzero(grid.faces >= 0)
        uv = grid.origin + (np.c_[i, j] + .5) * grid.step
        height = grid.height[i, j]
        points = uv[:, :1] * grid.u + uv[:, 1:] * grid.v + height[:, None] * grid.axis
        door = surface.roi_coordinates(surface.triangles[surface.target].reshape(-1, 3))[:, 1]
        keep = surface.roi_coordinates(points)[:, 1] <= (door.min() + door.max()) / 2
        normals = grid.normal_grid[i, j]
        reference = np.median(normals[keep], 0)
        keep &= normals @ (reference / np.linalg.norm(reference)) >= np.cos(np.radians(settings['flat_deg']))
        terms = np.c_[np.ones(len(uv)), uv, uv ** 2, uv[:, :1] * uv[:, 1:]]
        for _ in range(2):
            fit = np.linalg.lstsq(terms[keep], height[keep], rcond=None)[0]
            keep &= np.abs(terms @ fit - height) <= settings['step_mm']
        mask = np.zeros(grid.faces.shape, bool)
        mask[i[keep], j[keep]] = True
        mask = distance_transform_edt(np.pad(mask, 1))[1:-1, 1:-1] * grid.step > settings['seam_mm']
        # opened: strips narrower than 2 x narrow_mm dropped (the tab under the left panel, 2.6 mm wide), the rest kept
        core = distance_transform_edt(np.pad(mask, 1))[1:-1, 1:-1] * grid.step > settings['narrow_mm']
        mask &= distance_transform_edt(~core) * grid.step <= settings['narrow_mm']
        regions, count = label(mask)
        panel = regions == np.bincount(regions.ravel())[1:].argmax() + 1 if count else mask
        # its bottom rows up to the first bottom_width as wide as its median row dropped (the bulge under the right panel),
        # and its front_mm toward the vehicle's front (vehicle +x: the hinge joint by the bonnet, left and right doors)
        i, j = np.nonzero(panel)
        uv = grid.origin + (np.c_[i, j] + .5) * grid.step
        local = surface.roi_coordinates(uv[:, :1] * grid.u + uv[:, 1:] * grid.v + grid.height[i, j][:, None] * grid.axis)
        rows = np.round(local[:, 1] / grid.step).astype(int)
        widths = np.bincount(rows - rows.min())
        bottom = np.flatnonzero(widths >= settings['bottom_width'] * np.median(widths[widths > 0]))[0]
        drop = (rows - rows.min() < bottom) | (local[:, 0] > local[:, 0].max() - settings['front_mm'])
        panel[i[drop], j[drop]] = False
        grid.panel = panel
    return grid.panel


def panel_cells(grid, points):
    """The grid cell (i, j) of each point across the grid's axis."""
    return np.floor((np.c_[points @ grid.u, points @ grid.v] - grid.origin) / grid.step).astype(int).T


def press_label(axis):
    """'-Z', '+X': the BASE direction pressed into the surface for the outward `axis`."""
    k = int(np.argmax(np.abs(axis)))
    return f'{"+" if axis[k] < 0 else "-"}{"XYZ"[k]}'


class MeshHeightPlan(MeshPlan):
    controller = 'mesh-height'
    # Accessible cells checked every 2 mm, not ik_mm: a tool-collision strip (the bonnet's rear edge) slipped between
    # the 4 mm checks and was then excluded 2 mm per re-raster, each revalidating the pass (minutes).
    ACCESS_MM = 2.

    def __init__(self, surface, axes, reach, scan, travel_z, log=None):
        # One fixed axis (the side the probe comes from) and one TCP axis: tool_tilt_deg from the press direction
        # toward straight down (0: along the press, the bonnet).
        settings = scan['mesh_height']
        self.axis = np.asarray(axes[0], float)
        press, down, tilt = -self.axis, np.array([0., 0., -1.]), np.radians(settings['tool_tilt_deg'])
        self.spin = np.radians(settings.get('tool_spin_deg', 0.))
        side = down - press * (down @ press)
        self.tool = np.cos(tilt) * press + (np.sin(tilt) * side / np.linalg.norm(side) if tilt else 0.)
        self.radius = settings['ball_radius_mm']
        self.door_max_z = None
        if surface.part == 'door':
            heights = surface.roi_coordinates(surface.triangles[surface.target].reshape(-1, 3))[:, 1]
            self.door_max_z = (heights.min() + heights.max()) / 2
        super().__init__(surface, axes, reach, scan, travel_z, log)
        if self.door_max_z is not None and {entry['along'] for entry in self.diagnostics if entry['paths']} != {0, 1}:
            self.rejected.insert(0, 'Cannot build both horizontal and vertical inspection paths on the lower door')
            self.paths.clear()

    @staticmethod
    def configure(scan):
        scan = deepcopy(scan)
        settings = scan['mesh_height']
        settings.update(planning_source='mesh', position_interpolator='pchip')
        # Sparse IK along the strokes (0.5 mm made each re-raster revalidate a whole pass for minutes).
        settings['validation_step_mm'] = settings['pointcloud']['ik_step_mm']
        scan['measurable']['joint_jump_deg'] = settings['pointcloud']['joint_jump_deg']
        # Contact search and recorded samples use the mesh-height force thresholds.
        scan['default_scan']['survey']['force_n'] = settings['force_n']
        scan['default_scan']['contact_threshold_n'] = settings['record_threshold_n']  # measured points
        scan['default_scan']['real_contact_offset_n'] = settings['contact_threshold_n']  # real robot search (DSR)
        return scan

    def passes(self, valid, step, along):
        """Each pass one way only: pass 0 rows across (bonnet: BASE y) from the far side toward the robot (door: top
        down), pass 1 rows along it from the robot's right to its left; no segment visited out of the line order."""
        paths = PartPlan.passes(self, valid, step, along, join=sweep)
        if self.door_max_z is not None and paths and not self.scan.get('rescan_of'):
            return [max(paths, key=len)]
        return paths

    def scan_area(self, points):
        """A door: its flat panel under the window only (door_panel), no handle, crease or door/body joint."""
        allowed = super().scan_area(points)
        if self.door_max_z is not None:
            grid = self.grids[-1][1]
            i, j = panel_cells(grid, points)
            allowed &= door_panel(self.surface, grid, self.scan['mesh_height']['door_panel'])[i, j]
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
        """The TCP axis on `tool`, turned the shortest way from HOME's, then spun tool_spin_deg about itself (J6; minus:
        clockwise seen from the wrist down the tool)."""
        c, s = np.cos(self.spin), np.sin(self.spin)
        return align(self.reach.frame, -self.tool, np.pi) @ np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

    def probe_pose(self, point, normal):
        # Only depth along the axis comes from the mesh; neither the tool nor the force axis follows its normals.
        # The grid being planned is the last one added (MeshPlan).
        return self.tcp(point[None], self.grids[-1][1])[0], self.rotation()

    def ends(self, positions, rotations):
        """Start the bonnet path on a gently sloped patch with room for the probe footprint."""
        if self.surface.part != 'bonnet':
            return super().ends(positions, rotations)
        grid = self.grids[-1][1]
        settings = self.scan['mesh_height']
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
        settings = self.scan['mesh_height']
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
        tilt = self.scan['mesh_height']['tool_tilt_deg']
        return (f'mesh-height: {len(self.paths)} paths, {duration / 60:.1f} min strokes; fixed tool orientation'
                f'{f" (TCP axis {tilt:g} deg toward down)" if tilt else ""}, mesh depth feedforward, '
                f'BASE {press_label(self.paths[0].axis) if self.paths else ""} force, PD {"ON" if self.scan["mesh_height"]["pd"]["enabled"] else "OFF"}')
