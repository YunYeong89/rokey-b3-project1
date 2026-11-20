"""Reference geometry, measured samples and damage report for a fixed-axis scan."""
import numpy as np

from .damage_report import DamageReport
import matplotlib.pyplot as plt

from ..surface_follow.pointcloud_file import PointCloudFile

COLUMNS = 't_s,x_mm,y_mm,z_mm,force_n,axis_x,axis_y,axis_z,path'


def moving_average(values, window):
    """Centred mean over window samples, fewer at the ends."""
    kernel = np.ones(min(window, len(values)))
    return np.convolve(values, kernel, 'same') / np.convolve(np.ones_like(values), kernel, 'same')


class ScanReport:
    def __init__(self, directory, name, reference, web_report=None):
        self.directory, self.name, self.reference = directory, name, reference
        self.path = directory / name / 'report.png'
        self.web_report = web_report
        directory.mkdir(parents=True, exist_ok=True)

    def plan(self, maps):
        """Separate path preview with the measurable cells and planned sweeps."""
        files = []
        for number, passes in enumerate(zip(maps[::2], maps[1::2]), 1):
            fig, panels = plt.subplots(1, 2, figsize=(14, 6), layout='constrained')
            try:
                for panel, (axis, grid, along, valid, paths) in zip(panels, passes):
                    for mask, color in ((np.isfinite(grid.height) & ~valid, '0.82'), (valid, '#9bd79b')):
                        panel.scatter(*self.view(grid.positions(*np.nonzero(mask)), axis), s=1, c=color,
                                      marker='s', linewidths=0)
                    for cells in paths:
                        h, v = self.view(grid.positions(cells[:, 0], cells[:, 1]), axis)
                        panel.plot(h, v, linewidth=.6)
                        panel.plot(h[:1], v[:1], 'ko', markersize=4)
                    vertical = 'Y' if abs(axis[2]) > .9 else 'Z'
                    panel.set(xlabel='BASE X (mm)', ylabel=f'BASE {vertical} (mm)', aspect='equal',
                              title=f'{"Horizontal" if along == 0 else "Vertical"} pass · {len(paths)} paths')
                suffix = str(number) if number > 1 else ''
                path = self.directory / f'{self.name}_plan{suffix}.png'
                fig.savefig(path, dpi=110)
                files.append(path)
            finally:
                plt.close(fig)
        return files

    @staticmethod
    def view(points, axis):
        return points[:, 0], points[:, 1 if abs(axis[2]) > .9 else 2]

    def samples(self, records):
        path = self.directory / f'{self.name}_samples.csv'
        np.savetxt(path, records, delimiter=',', header=COLUMNS, comments='', fmt=['%.4f'] + ['%.6g'] * 8)
        return path

    def save_reference(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        return PointCloudFile.save_ply(self.path.parent / 'nominal_reference.ply', self.reference / 1000)

    def save(self, measured, axis=None):
        points = np.asarray(measured, float).reshape(-1, 4)[:, :3] * 1000
        return DamageReport(self.reference, points, axis=axis, web=self.web_report).save(self.path)
