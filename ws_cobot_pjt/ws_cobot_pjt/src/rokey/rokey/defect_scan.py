"""Defect scan of the contact-test block top, after `pick_probe` (native Gazebo only). surface_follow's hybrid
controller rubs the whole top in one serpentine raster: rows along BASE x every pitch_mm, from the bottom-left
(-x, -y) corner to the top-right one, the tip-sphere centre margin_mm inside the edge, at force_n and speed_mm_s
(robot.yaml `defect_scan`). The block carries the crack and dent of contact_test.block_defects (native scene).
The block centre is the probe TCP x, y at HOME (where the scene places the block), read from the same native
state as every scan sample once the arm is still. Lag compensation: a measured feature shows up v x tau late along
the feed; tau is estimated per channel (height, force) from the serpentine itself - a row against the mean of its
two neighbours, run the other way - and every sample is moved back by its own velocity x tau.
Graphs only, no point cloud -> rokey/images/defect_scan_<time>.png / .csv:
  maps on a grid_mm grid  relative height (tip vs the scan median), normal force, |innovation| / sigma of the
                          native force servo's Kalman filter, tangential force along the feed; true outlines,
                          blocked edges (dots: the force target dropped to climb) and hops (x: lifted over, unscanned)
  profiles                crack crossings at three widths and the dent row: measured height vs the trace a ball of
                          the tip radius can follow (narrow cracks are bridged), with force and innovation
  localisation            groove centre per row minus the true crack centre line, raw and lag-compensated, per feed
                          direction; the lag fit; the dent centre when its rows were scanned
surface_follow also writes its trace to rokey/surface_runs/.
  ros2 run rokey defect_scan [--pitch 1.0] [--speed 4.0] [--rows N] [--from-row K]
  e.g. the dent only: --from-row 14 --rows 22
"""
import argparse
from .robot_namespace import program_args
import csv
from datetime import datetime
import textwrap

import matplotlib

matplotlib.use('Agg')
from matplotlib.colors import TwoSlopeNorm
import matplotlib.pyplot as plt
import numpy as np

from .block_scan import DefectScan, RasterPath, home_centre
from .defect_block import DefectBlock
from .force_data import IMAGE_DIR
from .program import RobotProgram
from .surface_follow.transport import VirtualTransport


class ScanMap:
    """One entry per native sample of the TRACK phase, block-local mm; positions lag-compensated per channel."""
    columns = ['t_s', 'row', 'x_mm', 'y_mm', 'x_height_mm', 'x_force_mm', 'height_mm', 'Fn_N', 'Ft_N', 'Fx_N', 'Fy_N',
               'Fz_N', 'innovation_N', 'innovation_sigma_N', 'force_feedback_N', 'edge', 'feed_scale',
               'normal_x', 'normal_y', 'normal_z']

    def __init__(self, rows, centre_mm, block, raster, cell):
        seen, track = set(), []
        for row in rows:
            if row['phase'] == 'TRACK' and row['source_sequence'] not in seen:
                seen.add(row['source_sequence'])
                track.append(row)
        hops = [r for r in rows if r['phase'] == 'HOP']
        self.hop_xy = np.array([[r['x_m'] * 1000 - centre_mm[0], r['y_m'] * 1000 - centre_mm[1]] for r in hops]).reshape(-1, 2)
        self.hops = sum(1 for a, b in zip(rows, rows[1:]) if b['phase'] == 'HOP' and a['phase'] != 'HOP')
        get = lambda key: np.array([np.nan if r.get(key) is None else float(r[key]) for r in track])
        self.block, self.raster, self.cell = block, raster, cell
        self.t = get('source_time_s')  # native sample time: the position and force belong to it
        self.x, self.y = get('x_m') * 1000 - centre_mm[0], get('y_m') * 1000 - centre_mm[1]
        z = get('z_m') * 1000  # ProbeTCP = the ball's lowest point (vertical tool)
        self.offset = float(np.median(z) - block.top) if len(z) else 0.  # constant tip-model offset
        self.height = z - block.top - self.offset
        self.row = np.rint((self.y - raster.y[0]) / raster.pitch).astype(int)
        self.fx, self.fy, self.fz, self.fn = get('fx_n'), get('fy_n'), get('fz_n'), get('fn_n')
        self.ft = self.fx * np.where(self.row % 2 == 0, 1.0, -1.0)  # along the feed: friction < 0
        self.innovation, self.sigma = get('force_innovation_n'), get('innovation_sigma_n')
        self.feedback = get('force_feedback_n')
        self.edge, self.governor = get('edge') > 0, get('governor')  # feed scale of the contact governor
        self.normal = np.c_[get('normal_x'), get('normal_y'), get('normal_z')]  # the tool was aimed along it
        self.tilt = np.degrees(np.arccos(np.clip(self.normal[:, 2], -1, 1)))
        self.nu = self.innovation / self.sigma
        step = np.maximum(np.gradient(self.t), 1e-3) if len(self.t) > 2 else np.ones(len(self.t))
        self.vx, self.vy = ((np.gradient(v) if len(v) > 2 else np.zeros(len(v))) / step for v in (self.x, self.y))
        self.edges = [np.arange(raster.x[0] - cell / 2, raster.x[1] + cell, cell),
                      np.arange(raster.y[0] - cell / 2, raster.y[-1] + cell, cell)]
        self.fits = {'height': self.delay(self.height, .02), 'force': self.delay(self.fn, .2)}
        self.tau = {key: (0. if np.isnan(fit[0]) else fit[0]) for key, fit in self.fits.items()}

    def at(self, tau):
        """Positions moved back along their own velocity by the lag tau (s)."""
        return self.x - self.vx * tau, self.y - self.vy * tau

    def profiles(self, values, tau):
        grid, x = np.arange(self.raster.x[0], self.raster.x[1], .02), self.at(tau)[0]
        result = {}
        for k in np.unique(self.row):
            m = (self.row == k) & np.isfinite(values)
            if m.sum() >= 10:
                xs, index = np.unique(x[m], return_index=True)
                result[k] = np.interp(grid, xs, values[m][index], left=np.nan, right=np.nan)
        return result

    def delay(self, values, threshold):
        """Lag (s) of a channel behind the tip position, without ground truth: a feature shows up v x tau late along
        the feed, so a row disagrees with the mean of its two neighbours (run the other way) by 2 v tau. The lag
        minimising that disagreement over the feature regions (+-1 mm); nan without features."""
        base, pairs = self.profiles(values, 0.), []
        for k, p in base.items():
            if k - 1 in base and k + 1 in base:
                level, ref = np.nanmedian(p), (base[k - 1] + base[k + 1]) / 2
                feature = (np.abs(p - level) > threshold) | (np.abs(ref - level) > threshold)
                if feature.sum() >= 5:
                    pairs.append((k, np.convolve(feature, np.ones(101), 'same') > 0))
        if not pairs:
            return np.nan, None
        taus, cost = np.arange(0., .3001, .005), []  # a measurement cannot lead the motion
        for tau in taus:
            p = self.profiles(values, tau)
            cost.append(np.nanmean([np.nanmean(((p[k] - (p[k - 1] + p[k + 1]) / 2)[mask]) ** 2) for k, mask in pairs]))
        cost = np.asarray(cost)
        return float(taus[np.nanargmin(cost)]), (taus, cost / np.nanmin(cost))

    def cells(self, values, reduce, tau=0.):
        nx, ny = len(self.edges[0]) - 1, len(self.edges[1]) - 1
        x, y = self.at(tau)
        ix = np.floor((x - self.edges[0][0]) / self.cell).astype(int)
        iy = np.floor((y - self.edges[1][0]) / self.cell).astype(int)
        keep = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & np.isfinite(values)
        grid = np.full(ny * nx, np.nan)
        if keep.any():
            index, inverse = np.unique(iy[keep] * nx + ix[keep], return_inverse=True)
            order = np.argsort(inverse)
            groups = np.split(values[keep][order], np.cumsum(np.bincount(inverse))[:-1])
            grid[index] = [reduce(group) for group in groups]
        return grid.reshape(ny, nx)

    def outlines(self, axis, color='k'):
        start, end, _, direction = self.block.crack_frame()
        side, u = np.array([-direction[1], direction[0]]), np.linspace(0, 1, 120)
        centre, half = start + u[:, None] * (end - start), self.block.crack_width(u)[:, None] / 2
        polygon = np.r_[centre + side * half, (centre - side * half)[::-1], centre[:1]]
        axis.plot(polygon[:, 0], polygon[:, 1], color=color, lw=.6, ls='--')
        d, angle = self.block.dent, np.linspace(0, 2 * np.pi, 120)
        axis.plot(d['centre_mm'][0] + d['radius_mm'] * np.cos(angle), d['centre_mm'][1] + d['radius_mm'] * np.sin(angle),
                  color=color, lw=.6, ls='--')

    def crack_rows(self, tau):
        """Scanned rows over the crack (away from its tapered tips): y, feed direction, measured groove centre minus
        the true centre line (depth-weighted centroid within 4 mm, evaluation only), measured and ball-trace depth."""
        start, end, length, direction = self.block.crack_frame()
        x, rows = self.at(tau)[0], []
        for k in np.unique(self.row):
            y = self.raster.y[k]
            u = (y - start[1]) / direction[1] / length
            if not .05 < u < .95:
                continue
            centre_x = start[0] + (y - start[1]) * direction[0] / direction[1]
            m = (self.row == k) & (np.abs(x - centre_x) < 4)
            weight = np.clip(-self.height[m] - .01, 0, None)
            if weight.sum() > 0:
                model = np.linspace(-3, 3, 241) + centre_x
                rows.append((k, y, 1 if k % 2 == 0 else -1, (weight * x[m]).sum() / weight.sum() - centre_x,
                             -self.height[m].min(), -self.block.trace(model, np.full_like(model, y), 1.5).min()))
        return np.array(rows).reshape(-1, 6)

    def dent_error(self, tau):
        """Measured dent centre minus the true one (depth-weighted centroid), when all its rows were scanned."""
        d, (x, y) = self.block.dent, self.at(tau)
        if not (self.raster.y[self.row.min()] <= d['centre_mm'][1] - d['radius_mm']
                and self.raster.y[self.row.max()] >= d['centre_mm'][1] + d['radius_mm']):
            return None
        m = np.hypot(x - d['centre_mm'][0], y - d['centre_mm'][1]) < d['radius_mm'] + 3
        weight = np.clip(-self.height[m] - .05, 0, None)
        if weight.sum() <= 0:
            return None
        return (weight * x[m]).sum() / weight.sum() - d['centre_mm'][0], (weight * y[m]).sum() / weight.sum() - d['centre_mm'][1]

    def profile(self, axis, mask, centre_x, y, label, color, tip):
        x = self.at(self.tau['height'])[0][mask]
        order = np.argsort(x)
        axis.plot(x[order] - centre_x, self.height[mask][order], color=color, lw=1, label=f'{label} measured')
        model = np.linspace(-4, 4, 321) + centre_x
        axis.plot(model - centre_x, self.block.trace(model, np.full_like(model, y), tip), color=color, lw=1, ls='--',
                  label=f'{label} ball trace')

    def true_normal(self, x, y, step=.05):
        """Block top normal at x, y from the defect model (evaluation only)."""
        h = self.block.height
        gradient = [(h(x + step, y) - h(x - step, y)) / (2 * step), (h(x, y + step) - h(x, y - step)) / (2 * step)]
        normal = np.c_[-gradient[0], -gradient[1], np.ones(len(np.atleast_1d(x)))]
        return normal / np.linalg.norm(normal, axis=1, keepdims=True)

    def normal_error(self):
        """Angle between the aimed normal and the true one over the dent (deg), where it was scanned."""
        d = self.block.dent
        x, y = self.at(self.tau['force'])
        m = (np.hypot(x - d['centre_mm'][0], y - d['centre_mm'][1]) < d['radius_mm']) & np.isfinite(self.tilt)
        if not m.any():
            return None
        return np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i', self.normal[m], self.true_normal(x[m], y[m])), -1, 1)))

    def save(self, path, title, tip, target):
        fig, axes = plt.subplots(3, 4, figsize=(25, 17.5))
        extent = [self.edges[0][0], self.edges[0][-1], self.edges[1][0], self.edges[1][-1]]
        th, tf = self.tau['height'], self.tau['force']
        height = self.cells(self.height, np.median, th)
        low, high = np.nanmin(height, initial=-.05), np.nanmax(height, initial=.05)
        maps = [(f'Relative height [mm] (ball bottom vs scan median), lag {th * 1000:.0f} ms', height, 'RdBu_r',
                 TwoSlopeNorm(0, min(low, -.05), max(high, .05))),
                (f'Normal force Fn [N], lag {tf * 1000:.0f} ms', self.cells(self.fn, np.mean, tf), 'viridis', None),
                ('Kalman innovation max |v| / sigma', self.cells(np.abs(self.nu), np.max, tf), 'magma', None),
                ('Tangential force along the feed [N] (friction < 0)', self.cells(self.ft, np.mean, tf), 'coolwarm',
                 TwoSlopeNorm(0, min(np.nanmin(self.ft, initial=-.05), -.05), max(np.nanmax(self.ft, initial=.05), .05)))]
        ex, ey = self.at(tf)
        for axis, (name, grid, cmap, norm) in zip(axes[0], maps):
            image = axis.imshow(grid, origin='lower', extent=extent, cmap=cmap, norm=norm, interpolation='nearest')
            fig.colorbar(image, ax=axis, shrink=.85)
            self.outlines(axis, 'w' if cmap == 'magma' else 'k')
            if cmap in ('RdBu_r', 'coolwarm'):
                axis.plot(ex[self.edge], ey[self.edge], '.', color='k', ms=2, label='blocked edge')
                axis.plot(*self.hop_xy.T, 'x', color='0.4', ms=3, label='hop (unscanned)')
                axis.legend(fontsize=7, loc='upper right')
            axis.set(title=name, xlabel='x [mm] (BASE, from the block centre)', ylabel='y [mm]', aspect='equal')
        crossings = self.crack_rows(th)
        picks = crossings[np.unique(np.linspace(0, len(crossings) - 1, 3).round().astype(int))] if len(crossings) else []
        start, _, length, direction = self.block.crack_frame()
        fx = self.at(tf)[0]
        for (k, y, *_), color in zip(picks, ['tab:red', 'tab:orange', 'tab:purple']):
            centre_x = start[0] + (y - start[1]) * direction[0] / direction[1]
            mask = (self.row == k) & (np.abs(self.at(th)[0] - centre_x) < 4)
            label = f'w {self.block.crack_width((y - start[1]) / direction[1] / length):.2f} mm (y {y:.0f})'
            self.profile(axes[1, 0], mask, centre_x, y, label, color, tip)
            order = np.argsort(fx[mask])
            axes[1, 1].plot(fx[mask][order] - centre_x, self.fn[mask][order], color=color, lw=1, label=f'{label} Fn')
            axes[1, 1].plot(fx[mask][order] - centre_x, target + .25 * self.nu[mask][order], color=color, lw=.8, ls=':',
                            label=f'{label} {target:g} + 0.25 x innovation/sigma')
        d = self.block.dent
        row = int(np.rint((d['centre_mm'][1] - self.raster.y[0]) / self.raster.pitch))
        mask = (self.row == row) & (np.abs(self.at(th)[0] - d['centre_mm'][0]) < d['radius_mm'] + 5)
        if mask.any():
            self.profile(axes[1, 2], mask, d['centre_mm'][0], self.raster.y[row], 'dent row', 'tab:blue', tip)
            twin, x = axes[1, 2].twinx(), self.at(tf)[0][mask]
            twin.plot(np.sort(x) - d['centre_mm'][0], self.fn[mask][np.argsort(x)], color='tab:green', lw=.8, label='Fn')
            twin.set_ylabel('Fn [N]', color='tab:green')
        elapsed = self.t - self.t[0] if len(self.t) else self.t
        axes[1, 3].plot(elapsed, self.fn, color='k', lw=.5, label='Fn (80 ms filtered)')
        axes[1, 3].plot(elapsed, self.feedback, color='tab:cyan', lw=.5, label='native Kalman estimate')
        axes[1, 3].axhline(target, color='tab:orange', ls='--', lw=.8)
        raw = self.crack_rows(0.)
        for data, fill, name in ((raw, 'none', 'raw'), (crossings, None, f'lag-compensated ({th * 1000:.0f} ms)')):
            for sign, color in ((1, 'tab:red'), (-1, 'tab:blue')):
                m = data[:, 2] == sign if len(data) else []
                axes[2, 0].plot(data[m, 1] if len(data) else [], data[m, 3] if len(data) else [], 'o', mfc=fill,
                                color=color, label=f'{"+x" if sign > 0 else "-x"} rows, {name}')
        axes[2, 0].axhline(0, color='k', lw=.6)
        if len(crossings):
            axes[2, 1].plot(crossings[:, 1], crossings[:, 4], 'o-', color='tab:green', label='measured depth')
            axes[2, 1].plot(crossings[:, 1], crossings[:, 5], 's--', color='0.4', label='ball-trace depth (model)')
        for key, color in (('height', 'tab:blue'), ('force', 'tab:orange')):
            if self.fits[key][1] is not None:
                axes[2, 2].plot(self.fits[key][1][0] * 1000, self.fits[key][1][1], color=color,
                                label=f'{key}: {self.tau[key] * 1000:.0f} ms')
        metrics, dent = self.metrics(), self.dent_error(th)
        text = ', '.join([f'{k} {v:.3g}' if isinstance(v, float) else f'{k} {v}' for k, v in metrics.items()]
                         + ([f'dent centre error x, y {dent[0]:+.2f}, {dent[1]:+.2f} mm'] if dent else
                            ['dent not fully scanned']))
        fig.text(.01, .003, textwrap.fill(text, 230), family='monospace', fontsize=8, va='bottom')
        image = axes[2, 3].imshow(self.cells(self.tilt, np.mean, tf), origin='lower', extent=extent, cmap='magma',
                                  vmin=0, vmax=max(2., np.nanmax(self.tilt, initial=0)), interpolation='nearest')
        fig.colorbar(image, ax=axes[2, 3], shrink=.85)
        self.outlines(axes[2, 3], 'w')
        axes[2, 3].set(title='Tool normal tilt from the path normal [deg]', xlabel='x [mm]', ylabel='y [mm]',
                       aspect='equal')
        labels = [('Crack crossings: height', 'x from the crack centre line [mm]', 'height [mm]'),
                  ('Crack crossings: force and innovation', 'x from the crack centre line [mm]', 'N'),
                  ('Dent row: height (left) and Fn (right)', 'x from the dent centre [mm]', 'height [mm]'),
                  ('Normal force over the scan', 'time in TRACK [s]', 'N'),
                  ('Groove centre - true crack centre line, per row', 'row y [mm]', 'x error [mm]'),
                  ('Groove depth per row', 'row y [mm]', 'depth [mm]'),
                  ('Lag fit: row vs neighbour-mean mismatch', 'lag [ms]', 'relative mismatch')]
        for axis, (name, xlabel, ylabel) in zip([*axes[1], *axes[2, :3]], labels):
            axis.set(title=name, xlabel=xlabel, ylabel=ylabel)
            axis.grid(alpha=.3)
            if axis.get_legend_handles_labels()[0]:
                axis.legend(fontsize=7, loc='best')
        fig.suptitle(title)
        fig.tight_layout(rect=(0, .03, 1, 1))
        fig.savefig(path, dpi=100)
        plt.close(fig)

    def save_csv(self, path):
        with path.open('w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.columns)
            writer.writerows(np.c_[self.t, self.row, self.x, self.y, self.at(self.tau['height'])[0],
                                   self.at(self.tau['force'])[0], self.height, self.fn, self.ft, self.fx, self.fy,
                                   self.fz, self.innovation, self.sigma, self.feedback, self.edge, self.governor,
                                   self.normal].round(5))

    def metrics(self):
        error, raw, fixed = self.fn - np.nanmean(self.fn), self.crack_rows(0.), self.crack_rows(self.tau['height'])
        rising = lambda flag: int(np.sum(flag[1:] & ~flag[:-1]) + (flag[:1].sum() if len(flag) else 0))
        result = {'rows': int(self.row.max() + 1) if len(self.row) else 0, 'samples': len(self.t),
                  'Fn_mean_N': float(np.nanmean(self.fn)), 'Fn_std_N': float(np.nanstd(error)),
                  'innovation_abs_p99_sigma': float(np.nanpercentile(np.abs(self.nu), 99)) if len(self.nu) else 0.,
                  'edges': rising(self.edge), 'slowed': rising(self.governor < .5), 'hops': self.hops,
                  'lag_height_ms': self.tau['height'] * 1000, 'lag_force_ms': self.tau['force'] * 1000,
                  'tip_offset_mm': self.offset, 'tilt_max_deg': float(np.nanmax(self.tilt, initial=0))}
        error = self.normal_error()
        if error is not None:
            result.update(dent_normal_error_median_deg=float(np.median(error)),
                          dent_normal_error_p90_deg=float(np.percentile(error, 90)))
        if len(raw):
            result.update(crack_rows=len(raw), crack_error_raw_mm=float(np.abs(raw[:, 3]).mean()),
                          crack_error_compensated_mm=float(np.abs(fixed[:, 3]).mean()),
                          crack_depth_ratio=float(np.mean(fixed[:, 4] / fixed[:, 5])))
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--pitch', type=float, help='row spacing, mm (default robot.yaml defect_scan.pitch_mm)')
    parser.add_argument('--speed', type=float, help='feed along the rows, mm/s (default defect_scan.speed_mm_s)')
    parser.add_argument('--rows', type=int, help='only N rows (quick check)')
    parser.add_argument('--from-row', type=int, default=0, help='skip this many rows from the bottom (dent: 15)')
    options = parser.parse_args(program_args())
    program = RobotProgram('defect_scan')
    try:
        if program.mode != 'virtual':
            raise RuntimeError('defect_scan은 native Gazebo 가상 세션에서만 실행할 수 있습니다')
        config = program.config
        scan = dict(config['defect_scan'])
        scan.update({key: value for key, value in (('pitch_mm', options.pitch), ('speed_mm_s', options.speed)) if value})
        probe = config['probe']
        program.robot.verify_profiles(probe['tool_name'], probe['tcp_name'])
        transport = VirtualTransport(program)
        program.home()
        centre, block = home_centre(transport), DefectBlock(config['contact_test'])
        dsr, _ = program.probe.read()
        program.log(f'block centre (native HOME TCP) {centre.round(3).tolist()} mm; DSR reading differs by '
                    f'{(np.asarray(dsr[:2]) - centre).round(3).tolist()} mm')
        raster = RasterPath(centre, block, scan, probe['tip_radius_mm'], options.rows, start=options.from_row)
        run = DefectScan(program, raster, scan, transport,
                         settings=DefectScan.settings_for(config, scan))
        length = run.run.path.length * 1000
        program.log(f'defect_scan: {len(raster.y)} rows x {raster.x[1] - raster.x[0]:.0f} mm every {raster.pitch:g} mm, '
                    f'{length:.0f} mm path, ~{length / scan["speed_mm_s"] / 60:.0f} min at {scan["speed_mm_s"]:g} mm/s')
        failed = 1
        try:
            failed = run.execute()
        finally:
            result = ScanMap(run.run.recording.rows, centre, block, raster, scan['grid_mm'])
            if len(result.t):
                IMAGE_DIR.mkdir(exist_ok=True)
                stem = IMAGE_DIR / f'defect_scan_{datetime.now():%Y%m%d_%H%M%S}'
                result.save_csv(stem.with_suffix('.csv'))
                metrics = result.metrics()
                result.save(stem.with_suffix('.png'),
                            f'Defect scan (hybrid, {program.mode}, {scan["force_n"]:g} N, '
                            f'{scan["speed_mm_s"]:g} mm/s, '
                            f'pitch {raster.pitch:g} mm): {metrics["rows"]}/{len(raster.y)} rows, Fn '
                            f'{metrics["Fn_mean_N"]:.2f} +- {metrics["Fn_std_N"]:.2f} N', probe['tip_radius_mm'],
                            scan['force_n'])
                program.log(f'{stem}.png / .csv: ' + ', '.join(f'{k} {v:.3g}' if isinstance(v, float) else f'{k} {v}'
                                                              for k, v in metrics.items()))
        if failed:
            raise RuntimeError('defect_scan 실패: rokey/surface_runs/의 summary.json을 확인하세요')
    finally:
        program.close()


if __name__ == '__main__':
    main()
