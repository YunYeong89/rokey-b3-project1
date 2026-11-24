"""Nominal ball support and nearby stroke baselines for fixed-axis bonnet ROI analysis."""
import numpy as np
from scipy.ndimage import convolve, distance_transform_edt, label, map_coordinates, maximum_filter
from scipy.stats import binned_statistic_2d

from repair_car.pointcloud.cloud import ball_bottom
from repair_car.inspection.part_scan.cloud_plan import CloudGrid, CloudSurface


class CloudRoi:
    def __init__(self, points, radius, settings, cloud_settings):
        self.settings, self.radius = settings, radius
        grid = self.grid = CloudGrid(CloudSurface(points, 'bonnet', radius), np.array([0., 0., 1.]),
                                    dict(cloud_settings, grid_mm=settings['grid_mm'], smooth_mm=settings['smooth_mm']))
        uv = np.c_[points @ grid.u, points @ grid.v]
        bottom = ball_bottom(uv, points[:, 2], grid.origin, grid.step, grid.valid.shape, radius)
        self.support = np.where(np.isfinite(bottom), bottom + radius, np.nan)
        self.edge_distance = distance_transform_edt(np.pad(grid.valid, 1))[1:-1, 1:-1] * grid.step
        self.tilt = np.degrees(np.arccos(np.clip(grid.normals[..., 2], -1., 1.)))
        du, dv = np.gradient(grid.normals, grid.step, axis=(0, 1))
        turn = np.maximum(np.linalg.norm(du, axis=2), np.linalg.norm(dv, axis=2))
        reach = int(np.ceil(radius / grid.step))
        self.normal_change = np.degrees(2 * radius * maximum_filter(turn, size=2 * reach + 1))

    def sample(self, position):
        grid = self.grid
        cells = (np.c_[position @ grid.u, position @ grid.v] - grid.origin) / grid.step - .5
        support, edge, tilt, change = (map_coordinates(field, cells.T, order=1, cval=np.nan)
                                       for field in (self.support, self.edge_distance, self.tilt, self.normal_change))
        usable = (np.isfinite(support) & (edge > self.radius) & (tilt <= self.settings['max_tilt_deg']) &
                  (change <= self.settings['max_normal_change_deg']))
        roi_surface = (usable & (tilt <= self.settings['candidate_tilt_deg']) &
                       (change <= self.settings['confirm_normal_change_deg']))
        return support, usable, roi_surface

    def normal_change_at(self, position):
        """Turn of the nominal surface across the ball footprint (deg) at BASE positions."""
        grid = self.grid
        cells = (np.c_[position @ grid.u, position @ grid.v] - grid.origin) / grid.step - .5
        return map_coordinates(self.normal_change, cells.T, order=1, cval=np.nan)

    @staticmethod
    def headings(velocity):
        axis = np.argmax(np.abs(velocity), axis=1)
        along = velocity[np.arange(len(velocity)), axis]
        return 2 * axis + (along < 0)

    @staticmethod
    def steady(velocity, heading, speed):
        axis = heading // 2
        rows = np.arange(len(velocity))
        along = np.abs(velocity[rows, axis])
        across = np.abs(velocity[rows, 1 - axis])
        return (along >= .35 * speed) & (along > 2. * across)

    def paired_strokes(self, points, depth, force, heading, groups):
        """Retain a sustained downward deviation and force drop in the same stroke."""
        hit = (depth >= self.settings['confirm_depth_mm']) & (force <= -self.settings['confirm_force_drop_n'])
        same_stroke = (heading[1:] == heading[:-1]) & (groups[1:] == groups[:-1])
        joined = hit[:-1] & hit[1:] & same_stroke
        starts = np.flatnonzero(hit & ~np.r_[False, joined])
        ends = np.flatnonzero(hit & ~np.r_[joined, False])
        paired = np.zeros(len(points), bool)
        for first, last in zip(starts, ends):
            travel = np.linalg.norm(points[last, :2] - points[first, :2])
            if last - first >= 2 and travel >= self.settings['confirm_travel_mm']:
                paired[first:last + 1] = True
        return paired

    def cross_pass_labels(self, points, heading, step):
        """Match X and Y passes within one neighbouring cell, requiring two connected cells."""
        cells = np.floor(points[:, :2] / step).astype(int)
        cells -= cells.min(0)
        seen = np.zeros((2, *(cells.max(0) + 1)), bool)
        seen[(heading // 2, *cells.T)] = True
        cross = maximum_filter(seen[0], size=3) & maximum_filter(seen[1], size=3)
        labels, _ = label(cross & seen.any(0), np.ones((3, 3)))
        labels[np.bincount(labels.ravel())[labels] < 2] = 0
        return labels[tuple(cells.T)]

    def residuals(self, points, observations, velocity, usable, speed, step):
        """Subtract nearby cell medians from the same pass and travel direction."""
        inner, outer = self.settings['baseline_ring_mm']
        extent = int(np.ceil(outer / step))
        offsets = np.arange(-extent, extent + 1) * step
        distance = np.hypot(offsets[:, None], offsets)
        ring = ((distance >= inner) & (distance <= outer)).astype(float)
        direction = self.headings(velocity)
        stable = usable & np.all(np.isfinite(observations), axis=1) & self.steady(velocity, direction, speed)
        residual = np.full_like(observations, np.nan)
        for heading in np.unique(direction[stable]):
            at = np.flatnonzero(stable & (direction == heading))
            if len(at) < 5:
                continue
            xy = points[at, :2]
            first = np.floor(xy.min(0) / step).astype(int)
            last = np.floor(xy.max(0) / step).astype(int)
            bins = [np.arange(low, high + 2) * step for low, high in zip(first, last)]
            values = binned_statistic_2d(*xy.T, observations[at].T, statistic='median', bins=bins).statistic
            count = convolve(np.isfinite(values[0]).astype(float), ring, mode='constant')
            cells = ((xy - first * step) / step - .5).T
            enough = map_coordinates(count, cells, order=1, mode='nearest') >= 5
            for column, field in enumerate(values):
                background = convolve(np.nan_to_num(field), ring, mode='constant') / np.maximum(count, 1)
                baseline = map_coordinates(background, cells, order=1, mode='nearest')
                residual[at[enough], column] = observations[at[enough], column] - baseline[enough]
        return residual
