"""ROI of a real_scan run (default controller) from two kinds of evidence, both against the known shape, so slopes,
curves and character lines are not defects:
geometry: the contact TCP points against the ball-bottom surface of the reference (HeightMap.ball, the sphere
  envelope), compared by pointcloud_damage (normal deviation, connected regions, dent / crack / deformation);
force: the force along the axis less the one the known shape explains while following it at the measured feed,
  F_cmd + damping x reference slope x feed (+ mass x its rate): climbing a slope raises the reaction by
  damping x feed x tan(slope), going down lowers it. A dent, hole or bump makes the probe leave the reference and
  the force differ; runs beyond the limit along the path, connected across the lines, are force regions.
Overlapping regions are merged; a merged region is confirmed by both kinds of evidence, or by one strong kind.
-> rokey/images/real_<part>_<time>_roi.json in the pointcloud_damage ROI format (frame base_link, regions with
bounds_mm [x0, y0, z0, x1, y1, z1]: the confirmed ones; candidates beside them).
The constants are provisional, from a healthy MuJoCo bonnet run (2 N, 5 mm/s: no confirmed ROI; noise geometry
0.05 mm, force 0.14 N robust sigma; the character lines' front ends hit up to 2.6 mm force runs, 0.76 mm deviation)."""
import json
from dataclasses import asdict
from datetime import datetime

import numpy as np
from scipy.ndimage import label
from scipy.sparse.csgraph import connected_components

from ..pointcloud_damage import PartComparison, PointCloud, Settings
from .report import moving_average

SETTLE_S = 3.0              # the first seconds of each path are left out (the TCP reads ~0.75 mm low after the press)
# geometry: pointcloud_damage on 1 mm cells; normal deviation of the contact TCP from the ball-bottom surface (mm)
DAMAGE = Settings(cell_mm=1.0, tolerance_mm=0.4, full_damage_mm=2.0, match_radius_mm=1.5, max_deviation_mm=10.0,
                  min_region_cells=6, crack_width_mm=3.0, crack_aspect=4.0, align=False)
# force residual (N): smoothed measured force less F_cmd + damping x axial speed following the reference (+ mass x
# its rate)
FORCE_DAMPING_NS_M = 800.0  # virtual admittance damping (MuJoCo); identify on the real DRCF
FORCE_MASS_KG = 0.0         # 8 kg admittance mass: its acceleration term adds more noise than it explains
FORCE_SLOPE_MM = 2.0        # reference slope over this path length (the reference's sampling noise at shorter ones)
SPEED_SAMPLES = 5           # feed speed: centred difference over +-this many samples
FORCE_WINDOW = 9            # moving average of the measured force (samples)
FORCE_SIGMA = 8.0           # residual beyond this many robust sigmas (1.4826 MAD) ...
FORCE_MIN_N = 1.2           # ... and at least this far
FORCE_MIN_RUN_MM = 3.0      # beyond it over at least this far along the path (shorter: edge impacts)
FORCE_LINK_MM = 2.0         # runs this close (cells across the axis) form one region (> pitch 1.5 mm)
# merging and confirmation
MERGE_MM = 2.0              # regions closer than this are one ROI
PAD_MM = 1.5                # ROI bounds grown by this (probe ball radius) on every side
CONFIRM_DEVIATION_MM = 1.0  # geometry alone confirms from this deviation
CONFIRM_FORCE_N = 1.5       # force alone confirms from this residual


class ScanRoi:
    """records: AxisScraper.records; grids: [(axis, HeightMap)] of the plan; force_n: the commanded scan force;
    absolute: keep the TCP offset along the axis (no median removal)."""

    def __init__(self, records, grids, force_n, threshold, absolute=False):
        self.records = records = np.asarray(records, float)
        self.residual = np.full(len(records), np.nan)
        settled = np.zeros(len(records), bool)  # force: every stroke sample (a lost contact is evidence too)
        for path in np.unique(records[:, 8]):
            own = records[:, 8] == path
            settled[own] = records[own, 0] >= records[own, 0].min() + SETTLE_S
        for axis, grid in grids:
            own = np.all(np.isclose(records[:, 5:8], axis), axis=1)
            self.residual[own] = self.force_residual(records[own], axis, grid, force_n)
        self.residual[~settled] = np.nan
        used = settled & (records[:, 4] > threshold)  # geometry: the contact points
        sigma = 1.4826 * np.nanmedian(np.abs(self.residual - np.nanmedian(self.residual)))
        self.limit = max(FORCE_SIGMA * sigma, FORCE_MIN_N)
        regions = []
        for axis, grid in grids:
            own = np.all(np.isclose(records[:, 5:8], axis), axis=1)
            if (own & used).sum() >= DAMAGE.min_region_cells:
                regions += self.geometry(records[own & used, 1:4], axis, grid, absolute)
                regions += self.force(records[own, 1:4], self.residual[own], axis, grid)
        self.regions = self.merge(regions) if regions else []

    @staticmethod
    def force_residual(records, axis, grid, force_n):
        residual = np.zeros(len(records))
        for path in np.unique(records[:, 8]):
            own = np.nonzero(records[:, 8] == path)[0]
            t, p = records[own, 0], records[own, 1:4]
            s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p - np.outer(p @ axis, axis), axis=0), axis=1))]
            h = grid.ball_at(p)
            slope = (np.interp(s + FORCE_SLOPE_MM / 2, s, h) - np.interp(s - FORCE_SLOPE_MM / 2, s, h)) / FORCE_SLOPE_MM
            i = np.arange(len(own))
            low, high = np.maximum(i - SPEED_SAMPLES, 0), np.minimum(i + SPEED_SAMPLES, len(own) - 1)
            dt = t[high] - t[low]
            speed = slope * np.divide(s[high] - s[low], dt, out=np.zeros(len(own)), where=dt > 0) / 1000
            expected = force_n + FORCE_DAMPING_NS_M * speed
            if FORCE_MASS_KG:
                expected += FORCE_MASS_KG * np.gradient(speed, t)
            residual[own] = moving_average(records[own, 4], FORCE_WINDOW) - expected
        return residual

    @staticmethod
    def geometry(points, axis, grid, absolute):
        offset = 0. if absolute else np.nanmedian(points @ axis - grid.ball_at(points))
        comparison = PartComparison('scan', PointCloud(grid.ball_points() / 1000, 'base_link'),
                                    PointCloud((points - offset * axis) / 1000, 'base_link'), DAMAGE, axis=axis)
        return [dict(kind='geometry', type=r['type'], axis=axis, low=np.asarray(r['roi']['bounds_mm'][:3]),
                     high=np.asarray(r['roi']['bounds_mm'][3:]), deviation=r['max_deviation_mm'],
                     area=r['area_mm2'], length=r['length_mm'], width=r['width_mm'])
                for r in comparison.regions]

    def force(self, points, residual, axis, grid):
        """points, residual: the axis's samples in time order (nan: not used)."""
        out = np.abs(residual) > self.limit
        edges = np.flatnonzero(np.diff(np.r_[0, out.astype(int), 0]))
        keep = np.zeros(len(out), bool)
        for start, end in zip(edges[::2], edges[1::2]):
            keep[start:end] = np.linalg.norm(points[end - 1] - points[start]) >= FORCE_MIN_RUN_MM
        if not keep.any():
            return []
        points, residual = points[keep], residual[keep]
        cells = np.floor(np.c_[points @ grid.u, points @ grid.v] / FORCE_LINK_MM).astype(int)
        cells -= cells.min(0)
        mask = np.zeros(cells.max(0) + 1, bool)
        mask[tuple(cells.T)] = True
        labels = label(mask, np.ones((3, 3)))[0][tuple(cells.T)]
        regions = []
        for k in np.unique(labels):
            own = labels == k
            regions.append(dict(kind='force', axis=axis, low=points[own].min(0), high=points[own].max(0),
                                force=float(residual[own][np.argmax(np.abs(residual[own]))]), samples=int(own.sum())))
        return regions

    @staticmethod
    def merge(regions):
        low = np.array([r['low'] for r in regions]) - MERGE_MM / 2
        high = np.array([r['high'] for r in regions]) + MERGE_MM / 2
        axes = np.array([r['axis'] for r in regions])
        overlap = np.all((low[:, None] <= high[None]) & (low[None] <= high[:, None]), axis=2) & (axes @ axes.T > .99)
        _, labels = connected_components(overlap, directed=False)
        merged = []
        for k in np.unique(labels):
            own = [r for r, g in zip(regions, labels) if g == k]
            shape = [r for r in own if r['kind'] == 'geometry']
            force = [r for r in own if r['kind'] == 'force']
            main = max(shape, key=lambda r: r['area']) if shape else None
            deviation = max((r['deviation'] for r in shape), default=0.)
            peak = max((r['force'] for r in force), key=abs, default=0.)
            lo, hi = np.min([r['low'] for r in own], 0) - PAD_MM, np.max([r['high'] for r in own], 0) + PAD_MM
            merged.append({'type': main['type'] if main else 'force_high' if peak > 0 else 'force_low',
                           'confirmed': bool(shape and force) or deviation >= CONFIRM_DEVIATION_MM
                                        or abs(peak) >= CONFIRM_FORCE_N,
                           'evidence': [kind for kind, found in (('geometry', shape), ('force', force)) if found],
                           'bounds_mm': np.r_[lo, hi].round(2).tolist(), 'centre_mm': ((lo + hi) / 2).round(2).tolist(),
                           'axis': own[0]['axis'].tolist(), 'area_mm2': sum(r['area'] for r in shape),
                           'length_mm': main['length'] if main else None, 'width_mm': main['width'] if main else None,
                           'max_deviation_mm': deviation if shape else None,
                           'force_residual_n': peak if force else None,
                           'force_samples': sum(r['samples'] for r in force)})
        merged.sort(key=lambda r: (not r['confirmed'], -(r['max_deviation_mm'] or 0), -abs(r['force_residual_n'] or 0)))
        for number, region in enumerate(merged, 1):
            region['id'] = f'roi_{number:03d}'
        return merged

    def save(self, path, part):
        """The ROI JSON (pointcloud_damage --roi input: regions = the confirmed ones). -> path"""
        path.write_text(json.dumps({
            'frame': 'base_link', 'part': part, 'created_at': datetime.now().astimezone().isoformat(),
            'regions': [r for r in self.regions if r['confirmed']],
            'candidates': [r for r in self.regions if not r['confirmed']],
            'force_limit_n': self.limit,
            'parameters': {'damage': asdict(DAMAGE), **{k: v for k, v in globals().items() if k.startswith(
                ('SETTLE_', 'FORCE_', 'SPEED_', 'MERGE_', 'PAD_', 'CONFIRM_'))}}}, ensure_ascii=False, indent=2) + '\n')
        return path
