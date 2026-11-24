"""Point-cloud report for saved scans. Mesh-filled cells are display-only."""
from datetime import datetime
import os
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Rectangle
from matplotlib.ticker import MaxNLocator
import numpy as np
from scipy.ndimage import distance_transform_edt, gaussian_filter

from repair_car.report.pointcloud_damage import PartComparison, PointCloud


def report_font(size):
    noto = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')
    return FontProperties(fname=noto, size=size) if noto.is_file() else FontProperties(
        family=['Apple SD Gothic Neo' if os.uname().sysname == 'Darwin' else 'DejaVu Sans'],
        size=size, weight='bold')


class SurfaceHeight:
    CELL_MM = 1.

    def __init__(self, reference, measured):
        points = reference if len(reference) else measured
        self.origin = np.floor(points[:, :2].min(0) / self.CELL_MM) * self.CELL_MM - self.CELL_MM
        self.shape = tuple(np.ceil((points[:, :2].max(0) - self.origin) / self.CELL_MM).astype(int) + 2)
        self.extent = (self.origin[0], self.origin[0] + self.shape[0] * self.CELL_MM,
                       self.origin[1], self.origin[1] + self.shape[1] * self.CELL_MM)
        mesh = np.full(self.shape, -np.inf)
        if len(reference):
            np.maximum.at(mesh, tuple(self.cells(reference).T), reference[:, 2])
        known = np.isfinite(mesh)
        if known.any():
            gap, nearest = distance_transform_edt(~known, return_indices=True)
            mesh = np.where(gap <= 1.5, mesh[tuple(nearest)], np.nan)
        else:
            mesh.fill(np.nan)

        observed = np.full(self.shape, np.nan)
        cells = self.cells(measured)
        inside = np.all((cells >= 0) & (cells < self.shape), axis=1)
        if inside.any():
            flat = np.ravel_multi_index(cells[inside].T, self.shape)
            order = np.lexsort((measured[inside, 2], flat))
            flat = flat[order]
            values = measured[inside, 2][order]
            indexes, starts, counts = np.unique(flat, return_index=True, return_counts=True)
            observed.flat[indexes] = (values[starts + counts // 2] + values[starts + (counts - 1) // 2]) / 2

        # Interpolate the measured deviation from the mesh locally, keeping the CAD shape in the gaps.
        self.observed = np.isfinite(observed)
        overlap = self.observed & np.isfinite(mesh)
        values = np.where(overlap, observed - mesh, 0.)
        weights = gaussian_filter(overlap.astype(float), 1., radius=2)
        residual = np.divide(gaussian_filter(values, 1., radius=2), weights,
                             out=np.zeros_like(mesh), where=weights > 0)
        self.height = np.where(self.observed, observed, mesh + residual)

    def cells(self, points):
        return np.floor((points[:, :2] - self.origin) / self.CELL_MM).astype(int)


class DamageReport:
    STYLES = {'dent': ('#ff9f36', 'o'), 'crack': ('#ed3d78', 'D')}

    def __init__(self, reference, measured, *, axis=None, defects=None, web=None):
        self.reference, self.measured = self.points(reference), self.points(measured)
        self.axis = None if axis is None else np.asarray(axis)
        self.web = web
        self.defects = self.geometry_candidates() if defects is None else defects
        self.plane = np.delete(np.arange(3), np.argmax(np.abs(self.axis))) if self.axis is not None else [0, 1]
        self.scored_axes = self.plane if self.axis is not None else [0, 1, 2]
        self.front = self.axis is not None and np.argmax(np.abs(self.axis)) != 2  # a door: seen from the front
        combined = np.vstack((self.reference, self.measured))
        if len(combined):
            low, high = combined.min(0), combined.max(0)
            pad = np.maximum((high - low) * .04, .5)
            self.bounds = (low - pad, high + pad)
            self.zero = np.median((self.reference if len(self.reference) else combined) @ self.axis) if self.front else 0.
            depth = self.depth(combined)
            self.span = (depth.min(), depth.max())

    def depth(self, points):
        """Colour value: BASE Z, or for a part seen from the front its outward offset from the mesh's median along the axis."""
        return points @ self.axis - self.zero if self.front else points[:, 2]

    @staticmethod
    def points(values):
        points = np.asarray(values, float).reshape(-1, 3)
        return points[np.isfinite(points).all(axis=1)]

    def geometry_candidates(self):
        if len(self.reference) < 6 or not len(self.measured):
            return []
        from repair_car.inspection.part_scan.roi import DAMAGE
        comparison = PartComparison('scan', PointCloud(self.reference / 1000, 'base_link'),
                                    PointCloud(self.measured / 1000, 'base_link'), DAMAGE, axis=self.axis)
        defects = []
        points = comparison.surface.points * 1000
        for region in comparison.regions:
            kind = {'dent': 'dent', 'crack_candidate': 'crack'}.get(region['type'])
            if kind:
                bounds = region['roi']['bounds_mm']
                low, high = np.asarray(bounds).reshape(2, 3)
                own = ((comparison.labels == comparison.LABELS[region['type']]) &
                       np.all((points >= low) & (points <= high), axis=1))
                defects.append((dict(type=kind, bounds_mm=bounds, confirmed=False), points[own]))
        return defects

    def summary(self):
        def cells(points):
            return {tuple(cell) for cell in np.floor(points[:, self.scored_axes] / SurfaceHeight.CELL_MM).astype(int)}

        assessed = cells(self.measured)
        damaged = {kind: set() for kind in self.STYLES}
        counts = {kind: [0, 0] for kind in self.STYLES}
        for region, points in self.defects:
            kind = region['type']
            counts[kind][not region['confirmed']] += 1
            if region['confirmed']:
                damaged[kind].update(cells(points) & assessed)
        total = len(assessed)
        return dict(damage_percent=100 * len(set.union(*damaged.values())) / total if total else None,
                    assessed_cells=total, cell_mm=SurfaceHeight.CELL_MM,
                    regions={kind: dict(confirmed=counts[kind][0], candidates=counts[kind][1],
                                        damage_percent=100 * len(damaged[kind]) / total if total else None)
                             for kind in self.STYLES})

    def plotted_cloud(self, panel, cloud, title):
        if len(cloud):
            points = cloud[::max(1, int(np.ceil(len(cloud) / 25000)))]
            panel.scatter(*points.T, c=self.depth(points), cmap='viridis', s=1.5, linewidths=0,
                          vmin=self.span[0], vmax=self.span[1])
        else:
            panel.text2D(.5, .5, 'No point cloud', transform=panel.transAxes, ha='center')
        if hasattr(self, 'bounds'):
            low, high = self.bounds
            panel.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]))
            span = high - low
            panel.set_box_aspect(np.maximum(span, span.max() * .22), zoom=.89)
            counts = [4, 4, 2]
            if self.front:  # the panel's thin depth: one tick
                counts[np.argmax(np.abs(self.axis))] = 1
            for dimension, count in zip((panel.xaxis, panel.yaxis, panel.zaxis), counts):
                dimension.set_major_locator(MaxNLocator(count, min_n_ticks=min(count, 2)))
        panel.set(xlabel='BASE X (mm)', ylabel='BASE Y (mm)', zlabel='BASE Z (mm)', title=title)
        if self.front:  # from the probe's side, a little off the axis
            panel.view_init(elev=18, azim=np.degrees(np.arctan2(self.axis[1], self.axis[0])) + 25)
        else:
            panel.view_init(elev=32, azim=-55)

    @staticmethod
    def plane_labels(panel, plane):
        x, y = ('XYZ'[axis] for axis in plane)
        panel.set(xlabel=f'BASE {x} (mm)', ylabel=f'BASE {y} (mm)', aspect='equal')

    def defects_on(self, panel, plane, *, boxes=False):
        used = set()
        for region, points in self.defects:
            kind = region['type']
            color, marker = self.STYLES[kind]
            label = kind.capitalize() if kind not in used else None
            if len(points):
                points = points[::max(1, int(np.ceil(len(points) / 1500)))]
                panel.scatter(*points[:, plane].T, color=color, marker=marker, s=11, linewidths=.35,
                              edgecolors='#31233c' if boxes else 'white', zorder=5, label=label)
                used.add(kind)
            if boxes:
                a, b = np.asarray(region['bounds_mm']).reshape(2, 3)[:, plane]
                size = np.maximum(b - a, SurfaceHeight.CELL_MM)
                panel.add_patch(Rectangle(a, *size, fill=False, edgecolor=color, linewidth=1.6,
                                          linestyle='-' if region['confirmed'] else '--', zorder=4))
                panel.annotate(kind, (a[0], b[1]), xytext=(0, 3), textcoords='offset points',
                               fontsize=8, color=color)
        if used:
            panel.legend(loc='upper right', fontsize=8)

    def save(self, path):
        fig = plt.figure(figsize=(15, 11), layout='constrained')
        try:
            grid = fig.add_gridspec(3, 2, height_ratios=(1, 1.15, .25))
            self.plotted_cloud(fig.add_subplot(grid[0, 0], projection='3d'), self.reference, 'Reference mesh point cloud')
            self.plotted_cloud(fig.add_subplot(grid[0, 1], projection='3d'), self.measured, 'Measured contact point cloud')
            top, roi = fig.add_subplot(grid[1, 0]), fig.add_subplot(grid[1, 1])
            self.plane_labels(top, self.plane)
            self.plane_labels(roi, self.plane)
            top.set_title('Front view · measured outward offset and mesh interpolation' if self.front else
                          'Top view · measured height and mesh interpolation')
            roi.set_title('Damage ROI · solid confirmed, dashed candidate')
            if len(self.reference) or len(self.measured):
                view = lambda points: np.c_[points[:, self.plane], self.depth(points)]
                height = SurfaceHeight(view(self.reference), view(self.measured))
                low, high = np.nanpercentile(height.height, [2, 98]) if self.front else (None, None)  # spikes clipped
                image = top.imshow(height.height.T, extent=height.extent, origin='lower', cmap='viridis',
                                   alpha=np.where(height.observed.T, 1., .48), interpolation='nearest', vmin=low, vmax=high)
                # an inset colorbar: a layout one (ax=top) pushed the panel's y label off the figure
                fig.colorbar(image, cax=top.inset_axes([1.04, .1, .035, .8]),
                             label='Outward offset from mesh (mm)' if self.front else 'BASE Z (mm)',
                             extend='both' if self.front else 'neither')
                top.text(.01, -.20, 'Full color: measured cells   •   Pale: mesh-filled cells',
                         transform=top.transAxes, fontsize=9)
                self.defects_on(top, self.plane)
                for cloud, color, size in ((self.reference, '#e1e7ee', 2), (self.measured, '#8798aa', 2)):
                    roi.scatter(*cloud[::max(1, len(cloud) // 15000), self.plane].T, c=color, s=size, linewidths=0)
                self.defects_on(roi, self.plane, boxes=True)
            if not self.defects:
                roi.text(.5, .93, 'No dent or crack ROI detected', transform=roi.transAxes, ha='center', fontsize=10)
            if self.front and np.cross(-self.axis, [0., 0., 1.])[self.plane[0]] < 0:  # as seen from the probe's side
                top.invert_xaxis()
                roi.invert_xaxis()

            summary = self.summary()
            panel = fig.add_subplot(grid[2, :])
            panel.axis('off')
            pct = lambda value: f'{value:.2f}%' if value is not None else 'Unassessed'
            panel.text(.02, .70, f'Confirmed damage  {pct(summary["damage_percent"])}', fontsize=21,
                       weight='bold', va='center', color='#1f3349')
            details = '   •   '.join(f'{kind.capitalize()} {pct(info["damage_percent"])}'
                                     f' ({info["confirmed"]} confirmed, {info["candidates"]} candidates)'
                                     for kind, info in summary['regions'].items())
            panel.text(.02, .32, details, fontsize=11)
            panel.text(.02, .04, f'{summary["assessed_cells"]:,} measured cells at {SurfaceHeight.CELL_MM:g} mm. '
                       'Percentages use confirmed damage cells / measured cells.', fontsize=9, color='#48596b')
            if self.web:
                stamp = datetime.fromisoformat(self.web['started_at']).strftime('%Y-%m-%d  %H:%M:%S KST')
                vehicle = self.web.get('name', self.web['model'])
                owner = f'  |  Owner: {self.web["owner"]}' if 'owner' in self.web else ''
                fig.suptitle(f'{stamp}  |  Vehicle: {vehicle}  |  Plate: {self.web["plate"]}{owner}',
                             fontproperties=report_font(17), parse_math=False)
            path.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(path, dpi=140)
        finally:
            plt.close(fig)
        return summary
