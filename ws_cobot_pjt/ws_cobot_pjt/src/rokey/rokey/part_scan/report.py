"""Figures of a real_scan run (no display: Agg PNGs in rokey/images/real_<part>_<time>_*.png) and its samples (CSV).
Before scanning, per probe axis, the measurable cells of each pass and the selected lines joined into the paths.
After it, per probe axis, the measured surface seen along the axis (from above for the bonnet / bumper, facing the
door from its side), coloured by its position along the axis, and its deviation from the reference surface (the
ball-bottom surface of the planning HeightMap: the sphere envelope, so slopes are not outliers) with the outliers and
the ROI (roi.py; solid: confirmed, dashed: candidate) marked; the force along the axis over time, raw and denoised by
a centred moving average (per path, so paths are not blended), and its residual against the known shape."""
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

COLUMNS = 't_s,x_mm,y_mm,z_mm,force_n,axis_x,axis_y,axis_z,path'


def moving_average(values, window):
    """Centred mean over `window` samples, fewer at the ends."""
    kernel = np.ones(min(window,len(values)))
    return np.convolve(values, kernel, 'same') / np.convolve(np.ones_like(values), kernel, 'same')


class ScanReport:
    def __init__(self, directory, name, threshold, settings):
        self.directory, self.name, self.threshold = directory, name, threshold
        self.window, self.sigma, self.minimum = settings['window'], settings['outlier_sigma'], settings['outlier_min_mm']
        self.preserve_absolute = settings.get('preserve_absolute',False)
        directory.mkdir(parents=True, exist_ok=True)

    def plan(self, maps):
        """PartPlan.maps, per axis (its horizontal and vertical pass): the surface (grey), the cells measurable along
        the pass's lines (green), the paths (one colour each, start as a black dot). -> the files written."""
        files = []
        for number, passes in enumerate(zip(maps[::2], maps[1::2]), 1):
            fig, panels = plt.subplots(1, 2, figsize=(16, 7))
            for panel, (axis, grid, along, valid, paths) in zip(panels, passes):
                for mask, colour in ((np.isfinite(grid.height) & ~valid, '0.82'), (valid, (.55, .85, .55))):
                    h, v, labels = self.view(grid.positions(*np.nonzero(mask)), axis)
                    panel.scatter(h, v, s=1, c=[colour], marker='s', linewidths=0)
                for k, cells in enumerate(paths):
                    h, v, _ = self.view(grid.positions(cells[:, 0], cells[:, 1]), axis)
                    panel.plot(h, v, linewidth=.6, color=f'C{k % 10}')
                    panel.plot(h[:1], v[:1], 'ko', markersize=4)
                panel.set(xlabel=labels[0], ylabel=labels[1], aspect='equal',
                          title=f'{self.name}: {"horizontal" if along == 0 else "vertical"} pass, axis '
                                f'{axis.round(0).tolist()}: {valid.sum()} measurable cells, {len(paths)} path(s)')
            files.append(self.close(fig, self.figure('plan', number)))
        return files

    def save(self, records, grids, rois=None):
        """records: AxisScraper.records; grids: [(axis, HeightMap)] of the plan; rois: their ScanRoi. -> the files
        written."""
        records = np.asarray(records, float)
        files = [self.directory / f'{self.name}_samples.csv']
        np.savetxt(files[0], records, delimiter=',', header=COLUMNS, comments='', fmt=['%.4f'] + ['%.6g'] * 8)
        contact = records[records[:, 4] > self.threshold]
        for number, (axis, grid) in enumerate(grids, 1):
            own = contact[np.all(np.isclose(contact[:, 5:8], axis), axis=1)]
            if len(own):
                files += [self.surface(own[:, 1:4], axis, number),
                          self.outliers(own[:, 1:4], axis, grid, number, rois.regions if rois else [])]
        return files + [self.force(records, False), self.force(records, True)] + (
            [self.residual(records, rois)] if rois else [])

    @staticmethod
    def view(points, axis):
        """In-plane coordinates seen along the axis (BASE x, y from above; x, z from a side) and the labels."""
        if abs(axis[2]) > .9:
            return points[:, 0], points[:, 1], ('BASE x (mm)', 'BASE y (mm)')
        return points[:, 0], points[:, 2], ('BASE x (mm)', 'BASE z (mm)')

    def figure(self, suffix, number):
        return self.directory / f'{self.name}_{suffix}{number if number > 1 else ""}.png'

    def surface(self, points, axis, number):
        h, v, labels = self.view(points, axis)
        fig, plot = plt.subplots(figsize=(8, 6.5))
        dots = plot.scatter(h, v, c=points @ axis, s=3, cmap='viridis')
        fig.colorbar(dots, label='position along the probe axis (mm)')
        plot.set(xlabel=labels[0], ylabel=labels[1], aspect='equal',
                 title=f'{self.name}: measured surface, axis {axis.round(0).tolist()}, {len(points)} contact samples')
        return self.close(fig, self.figure('surface', number))

    def deviations(self, points, axis, grid):
        """Points on the reference surface, their deviation along the axis from its ball-bottom height there less the
        median (the TCP / ball offset), the outlier limit (outlier_sigma robust sigmas, 1.4826 MAD, at least
        outlier_min_mm) and the outliers beyond it."""
        reference = grid.ball_at(points)
        known = np.isfinite(reference)
        points, deviation = points[known], (points @ axis - reference)[known]
        if not self.preserve_absolute:
            deviation -= np.median(deviation)
        limit = max(self.sigma * 1.4826 * np.median(np.abs(deviation)), self.minimum)
        return points, deviation, limit, np.abs(deviation) > limit

    def outliers(self, points, axis, grid, number, regions):
        points, deviation, limit, out = self.deviations(points, axis, grid)
        h, v, labels = self.view(points, axis)
        fig, (plot, histogram) = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={'width_ratios': [3, 2]})
        scale = max(3 * limit, np.abs(deviation).max(initial=0))
        dots = plot.scatter(h, v, c=deviation, s=3, cmap='coolwarm', vmin=-scale, vmax=scale)
        plot.scatter(h[out], v[out], s=24, facecolors='none', edgecolors='k', linewidths=.8, label='outlier')
        fig.colorbar(dots, ax=plot, label='deviation from the reference along the axis (mm)')
        plot.set(xlabel=labels[0], ylabel=labels[1], aspect='equal',
                 title=f'{self.name}: deviation, axis {axis.round(0).tolist()}: {out.sum()} outliers '
                       f'({out.mean():.1%}) beyond +-{limit:.2f} mm')
        for region in regions:
            if np.allclose(region['axis'], axis):
                h, v, _ = self.view(np.reshape(region['bounds_mm'], (2, 3)), axis)
                plot.add_patch(plt.Rectangle((h.min(), v.min()), np.ptp(h), np.ptp(v), fill=False, linewidth=1.2,
                                             edgecolor='k' if region['confirmed'] else '0.45',
                                             linestyle='-' if region['confirmed'] else '--'))
                plot.annotate(f'{region["id"]} {region["type"]}', (h.max(), v.max()), fontsize=7)
        plot.legend(loc='upper right')
        histogram.hist(deviation, bins=80, color='0.4')
        for value in (-limit, limit):
            histogram.axvline(value, color='r', linestyle='--')
        histogram.set(xlabel='deviation (mm)', ylabel='samples', yscale='log', title='deviation distribution')
        return self.close(fig, self.figure('outliers', number))

    def force(self, records, denoised):
        time = records[:, 0] - records[0, 0]
        fig, plot = plt.subplots(figsize=(12, 4.5))
        for path in np.unique(records[:, 8]):
            own = records[:, 8] == path
            force = moving_average(records[own, 4], self.window) if denoised else records[own, 4]
            plot.plot(time[own], force, linewidth=.8, label=f'path {int(path)}')
        plot.axhline(self.threshold, color='r', linestyle='--', linewidth=.8, label='contact threshold')
        kind = f'moving average over {self.window} samples' if denoised else 'raw'
        plot.set(xlabel='time (s)', ylabel='force along the probe axis (N)', title=f'{self.name}: force, {kind}')
        plot.legend(loc='upper right', fontsize=8)
        return self.close(fig, self.directory / f'{self.name}_force_{"denoised" if denoised else "raw"}.png')

    def residual(self, records, rois):
        """ScanRoi.residual over time per path (nan: settling, off the reference) and its limit."""
        time = records[:, 0] - records[0, 0]
        fig, plot = plt.subplots(figsize=(12, 4.5))
        for path in np.unique(records[:, 8]):
            own = records[:, 8] == path
            plot.plot(time[own], rois.residual[own], linewidth=.8, label=f'path {int(path)}')
        for value in (-rois.limit, rois.limit):
            plot.axhline(value, color='r', linestyle='--', linewidth=.8)
        plot.set(xlabel='time (s)', ylabel='force residual (N)',
                 title=f'{self.name}: force less the known shape\'s (F_cmd + damping x slope x feed), limit '
                       f'+-{rois.limit:.2f} N, {sum(r["confirmed"] for r in rois.regions)} confirmed ROI')
        plot.legend(loc='upper right', fontsize=8)
        return self.close(fig, self.directory / f'{self.name}_force_residual.png')

    @staticmethod
    def close(fig, path):
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
        return path
