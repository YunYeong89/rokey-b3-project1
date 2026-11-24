"""Offline survey candidates from force OR reference-relative depth, including contact loss."""
import json
from datetime import datetime
from itertools import product

import numpy as np
from scipy.ndimage import label

from repair_car.inspection.part_scan.planner import PartPlan
from repair_car.inspection.part_scan.roi import ScanRoi


class SurveyRoi:
    def __init__(self, scraper, grids, settings):
        self.settings = settings
        records = np.asarray(scraper.records, float).reshape(-1, 9)
        self.residual, self.depth = np.full(len(records), np.nan), np.full(len(records), np.nan)
        self.regions, self.limits = [], []
        force = scraper.scan['force_n']
        for axis, grid in grids:
            own = np.all(np.isclose(records[:, 5:8], axis), axis=1)
            rows = records[own]
            regions = []
            if not len(rows):
                self.regions += self.merge(regions)
                continue
            residual = ScanRoi.force_residual(rows, axis, grid, force)
            # Remove a steady sensor offset, without re-centring measured depths over damaged samples.
            finite = np.isfinite(residual)
            if finite.any():
                residual -= np.median(residual[finite])
            sigma = 1.4826 * np.median(np.abs(residual[finite])) if finite.any() else 0.
            limit = max(settings['force_min_n'], settings['noise_sigma'] * sigma)
            depth = grid.ball_at(rows[:, 1:4]) - rows[:, 1:4] @ axis
            known = depth[np.isfinite(depth)]
            depth_sigma = 1.4826 * np.median(np.abs(known - np.median(known))) if len(known) else 0.
            depth_threshold = max(settings['depth_min_mm'], settings['noise_sigma'] * depth_sigma)
            self.residual[own], self.depth[own] = residual, depth
            self.limits.append(dict(axis=axis.tolist(), force_n=float(limit), depth_mm=float(depth_threshold)))
            flags = {'force': np.abs(residual) > limit,
                     'geometry': np.abs(depth) > depth_threshold,
                     'contact_loss': rows[:, 4] <= scraper.threshold}
            for reason, flag in flags.items():
                kept = np.zeros(len(rows), bool)
                for path in np.unique(rows[:, 8]):
                    at = np.flatnonzero(rows[:, 8] == path)
                    edges = np.flatnonzero(np.diff(np.r_[False, flag[at], False]))
                    for first, last in zip(edges[::2], edges[1::2]):
                        run = at[first:last]
                        travel = np.diff(rows[run, 1:4], axis=0)
                        travel -= np.outer(travel @ axis, axis)
                        if len(run) >= 3 and np.linalg.norm(travel, axis=1).sum() >= settings['min_run_mm']:
                            kept[run] = True
                if kept.any():
                    regions += self.components(rows[kept, 1:4], depth[kept], residual[kept], axis, grid, reason)
            self.regions += self.merge(regions)
        self.limit = max((item['force_n'] for item in self.limits), default=settings['force_min_n'])
        for number, region in enumerate(self.regions, 1):
            region['id'] = f'roi_{number:03d}'

    def region(self, points, axis, evidence, depth, force):
        depth = np.nan if depth is None else depth
        pad = self.settings['pad_mm']
        low, high = points.min(0) - pad, points.max(0) + pad
        return dict(type='survey_candidate', confirmed=False, evidence=evidence, axis=axis.tolist(),
                    bounds_mm=np.r_[low, high].tolist(), centre_mm=((low + high) / 2).tolist(),
                    max_deviation_mm=float(abs(depth)) if np.isfinite(depth) else None,
                    force_residual_n=float(force) if np.isfinite(force) else None)

    def component_labels(self, points, grid):
        cells = np.floor(np.c_[points @ grid.u, points @ grid.v] / self.settings['link_mm']).astype(int)
        cells -= cells.min(0)
        mask = np.zeros(cells.max(0) + 1, bool)
        mask[tuple(cells.T)] = True
        return label(mask, np.ones((3, 3)))[0][tuple(cells.T)]

    def components(self, points, depth, force, axis, grid, reason):
        groups = self.component_labels(points, grid)
        regions = []
        for group in np.unique(groups):
            own = groups == group
            d, f = depth[own], force[own]
            regions.append(self.region(points[own], axis, [reason],
                                       np.nanmax(np.abs(d)) if np.isfinite(d).any() else np.nan,
                                       f[np.nanargmax(np.abs(f))] if np.isfinite(f).any() else np.nan))
        return regions

    @staticmethod
    def merge(regions):
        pending, merged = list(regions), []
        while pending:
            current = pending.pop()
            changed = True
            while changed:
                changed = False
                low, high = np.asarray(current['bounds_mm']).reshape(2, 3)
                for index, other in enumerate(pending):
                    a, b = np.asarray(other['bounds_mm']).reshape(2, 3)
                    if np.all(low <= b) and np.all(a <= high):
                        low, high = np.minimum(low, a), np.maximum(high, b)
                        current['bounds_mm'] = np.r_[low, high].tolist()
                        current['centre_mm'] = ((low + high) / 2).tolist()
                        current['evidence'] = sorted(set(current['evidence'] + other['evidence']))
                        current['max_deviation_mm'] = max(current['max_deviation_mm'] or 0., other['max_deviation_mm'] or 0.)
                        if abs(other['force_residual_n'] or 0.) > abs(current['force_residual_n'] or 0.):
                            current['force_residual_n'] = other['force_residual_n']
                        pending.pop(index)
                        changed = True
                        break
            # Survey candidates are all revisited; confirmation is separate from eligibility.
            current['confirmed'] = 'geometry' in current['evidence'] and bool(
                set(current['evidence']) & {'force', 'contact_loss'})
            merged.append(current)
        return merged

    def save(self, path, part):
        path.write_text(json.dumps(dict(frame='base_link', part=part, created_at=datetime.now().astimezone().isoformat(),
                                        regions=[r for r in self.regions if r['confirmed']],
                                        candidates=[r for r in self.regions if not r['confirmed']],
                                        rescan_regions=self.regions, force_limits=self.limits,
                                        parameters=self.settings), ensure_ascii=False, indent=2,
                                   allow_nan=False) + '\n')
        return path


class RoiPlan(PartPlan):
    """Re-raster only candidates inside the original reachable/clear masks; no defect mesh is used."""
    def __init__(self, parent, regions, scan):
        self.clearance, self.reach, self.travel, self.scan = parent.clearance, parent.reach, parent.travel, scan
        self.grids, self.maps, self.paths, self.sweeps = parent.grids, [], [], []
        self.count = parent.count.copy()
        for axis, grid, along, valid, _ in parent.maps:
            i, j = np.nonzero(valid)
            points = grid.positions(i, j)
            selected = np.zeros(len(points), bool)
            for region in regions:
                if not np.allclose(region['axis'], axis):
                    continue
                low, high = np.asarray(region['bounds_mm']).reshape(2, 3)
                corners = np.asarray(list(product(*zip(low, high))))
                uv, bounds = np.c_[points @ grid.u, points @ grid.v], np.c_[corners @ grid.u, corners @ grid.v]
                selected |= np.all((uv >= bounds.min(0)) & (uv <= bounds.max(0)), axis=1)
            mask = np.zeros_like(valid)
            mask[i[selected], j[selected]] = True
            paths = self.passes(mask, grid.step, along)
            self.maps.append((axis, grid, along, mask, paths))
            segments = [kept for cells in paths
                        if (kept := self.trim(grid.positions(cells[:, 0], cells[:, 1]), axis, every=1)) is not None]
            if segments:
                segments = self.order(segments)
                self.sweeps.append((segments, axis, [self.hop(a[-1], b[0], axis)
                                                    for a, b in zip(segments[:-1], segments[1:])]))
                self.paths += [(segment, axis) for segment in segments]
