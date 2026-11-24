"""ROI rescan report (plan.json rescan_of): the cracks and dents found drawn on the part's mesh itself (its nominal
reference cloud, lit as painted metal) instead of ROI boxes over the measured cloud. A header with the counts, the
whole part in perspective with the defects and the rescanned areas on its surface, each area enlarged from above
(scan rows, crossings, callouts, scale bar), and a table. Crack ROIs within JOIN_MM of each other are one crack
(081134: crack B came as four pieces), drawn through their crossings in order along it, lightly smoothed; a dent is
its samples' outline, shaded deeper toward the middle; the survey's confirmed dents are added when not rescanned."""
from datetime import datetime
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.patheffects as effects
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle
from mpl_toolkits.axes_grid1.anchored_artists import AnchoredSizeBar
import numpy as np
from scipy.interpolate import LinearNDInterpolator, RegularGridInterpolator
from scipy.ndimage import distance_transform_edt, gaussian_filter
from scipy.spatial import ConvexHull, QhullError, cKDTree

from repair_car.report.damage_report import report_font


class DefectMap:
    COLORS = {'crack': '#d7193a', 'dent': '#f28a1e'}
    GLOW = {'crack': '#ff6b81', 'dent': '#ffc078'}
    INK, MUTED, AREA, BAND = '#15263a', '#5d6f82', '#2f6fd6', '#eef2f6'
    PAINT = np.array([.68, .74, .81])  # the bonnet's paint, blue-grey metal
    LIGHT = LightSource(azdeg=315, altdeg=38)
    SMOOTH_MM, JOIN_MM, LIFT_MM = 2.5, 4., .35  # SMOOTH_MM: the reference cloud's ripples gone, its creases kept
    HALO = [effects.withStroke(linewidth=4., foreground='white')]

    def __init__(self, reference, measured, defects, metadata, survey=()):
        """survey: the survey's confirmed dent regions (roi.json), drawn as the ellipse in their bounds unless one of
        the rescan's own dents covers their centre: the rescan goes over the cracks only (081134 grazed the edge of
        075234's 0.36 mm dent and, in one pass, could not confirm it)."""
        door = metadata.get('part') == 'door'
        axis = np.asarray(metadata['settings']['door_axis'] if door else (0., 0., 1.))
        depth = int(np.argmax(np.abs(axis)))
        plane = [index for index in range(3) if index != depth]
        self.part = f'{metadata["side"].capitalize()} door' if door and metadata.get('side') else (
            'Door' if door else 'Bonnet')

        def project(points):
            points = np.asarray(points, float).reshape(-1, 3)
            return np.c_[points[:, plane], points[:, depth] * np.sign(axis[depth])]

        self.reference = project(reference)
        self.measured = project(measured)
        self.metadata, self.areas = metadata, metadata.get('roi_base_mm') or []
        self.height = (LinearNDInterpolator(self.reference[:, :2], self.reference[:, 2])
                       if len(self.reference) > 3 else None)
        defects = [(region, project(points)) for region, points in defects if len(points)]
        found = [region for region, _ in defects if region['type'] == 'dent']
        angle = np.linspace(0., 2 * np.pi, 33)[:-1]
        for region in survey:
            low, high = np.asarray(region['bounds_mm']).reshape(2, 3)[:, plane]
            centre = np.take(region['centre_mm'], plane)
            if not any(np.all((bounds[0] <= centre) & (centre <= bounds[1]))
                       for bounds in (np.asarray(own['bounds_mm']).reshape(2, 3)[:, plane] for own in found)):
                ellipse = (low + high) / 2 + (high - low) / 2 * np.c_[np.cos(angle), np.sin(angle)]
                defects = [*defects, (dict(region, source='survey'), ellipse)]
        self.defects = []  # (name, type, [regions], xy)
        for kind in ('crack', 'dent'):
            own = [(region, np.asarray(points)[:, :2]) for region, points in defects if region['type'] == kind]
            for number, group in enumerate(self.join(own) if kind == 'crack' else [[item] for item in own], 1):
                self.defects.append((f'{kind[0].upper()}{number}', kind, [region for region, _ in group],
                                     np.vstack([xy for _, xy in group])))

    def join(self, cracks):
        """Crack ROIs whose crossings come within JOIN_MM of each other, in groups."""
        labels = list(range(len(cracks)))
        root = lambda i: i if labels[i] == i else root(labels[i])
        for i in range(len(cracks)):
            for j in range(i + 1, len(cracks)):
                if cKDTree(cracks[i][1]).query(cracks[j][1])[0].min() <= self.JOIN_MM:
                    labels[root(j)] = root(i)
        groups = {}
        for i in range(len(cracks)):
            groups.setdefault(root(i), []).append(cracks[i])
        return list(groups.values())

    @staticmethod
    def trace(xy):
        """A crack's crossings in order along their main direction, those of one row (within 0.75 mm along it)
        averaged, from four on a 3-point running mean (the ends kept): the crossings scatter about 1 mm about the
        crack."""
        centre = xy.mean(0)
        direction = np.linalg.svd(xy - centre, full_matrices=False)[2][0] if len(xy) > 1 else np.r_[1., 0.]
        along = (xy - centre) @ direction
        order = np.argsort(along)
        cuts = np.r_[0, np.flatnonzero(np.diff(along[order]) > .75) + 1, len(xy)]
        line = np.array([xy[order[a:b]].mean(0) for a, b in zip(cuts[:-1], cuts[1:])])
        if len(line) >= 4:
            line[1:-1] = (line[:-2] + line[1:-1] + line[2:]) / 3
        return line

    @staticmethod
    def outline(xy):
        """A dent's samples' convex outline (a 1 mm circle round fewer)."""
        try:
            return xy[ConvexHull(xy).vertices]
        except (QhullError, ValueError):
            angle = np.linspace(0., 2 * np.pi, 17)[:-1]
            return xy.mean(0) + np.c_[np.cos(angle), np.sin(angle)]

    def grid(self, low, high, cell):
        """The mesh height on `cell` mm over [low, high], smoothed over SMOOTH_MM (the sampled points' own facets
        otherwise light up as grain) -> x, y, z (off the mesh: its nearest height, so no wall at the rim), off-mesh
        mask."""
        x, y = np.arange(low[0], high[0] + cell, cell), np.arange(low[1], high[1] + cell, cell)
        X, Y = np.meshgrid(x, y)
        Z = self.height(X, Y)
        off = np.isnan(Z)
        sigma = self.SMOOTH_MM / cell
        weight = gaussian_filter((~off).astype(float), sigma)
        Z = gaussian_filter(np.where(off, 0., Z), sigma) / np.maximum(weight, 1e-9)
        Z = Z[tuple(distance_transform_edt(off, return_distances=False, return_indices=True))]
        return X, Y, Z, off

    def lit(self, Z, cell, exaggeration):
        return self.LIGHT.shade_rgb(np.broadcast_to(self.PAINT, Z.shape + (3,)), Z, fraction=1.,
                                    blend_mode='soft', vert_exag=exaggeration, dx=cell, dy=cell)

    def on_surface(self, xy):
        """Heights on the smoothed overview surface (the raw cloud's facets zigzag at 3x), LIFT_MM above it."""
        return self.overview(xy[:, ::-1]) + self.LIFT_MM

    def summary(self):
        cracks = [(xy, regions) for _, kind, regions, xy in self.defects if kind == 'crack']
        dents = [regions for _, kind, regions, _ in self.defects if kind == 'dent']
        length = sum(np.linalg.norm(np.diff(self.trace(xy), axis=0), axis=1).sum() for xy, _ in cracks)
        depth = max((max(r['max_deviation_mm'] for r in regions) for regions in dents), default=0.)
        return [(f'{len(cracks)}', 'Cracks', f'{length:.1f} mm in total'),
                (f'{len(dents)}', 'Dents', f'{depth:.2f} mm deepest' if dents else 'none'),
                (f'{len(self.areas)}', 'Rescan areas', f'{self.metadata["speed_mm_s"]:g} mm/s · '
                                                       f'{self.metadata.get("target_normal_force_n", 0):g} N')]

    def header(self, fig, band):
        band.axis('off')
        band.add_patch(Rectangle((0, 0), 1, 1, transform=band.transAxes, color=self.BAND, zorder=0))
        web = self.metadata.get('web_report') or {}
        stamp = (web.get('started_at', '')[:16].replace('T', ' ') or
                 datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y-%m-%d %H:%M KST'))
        band.text(.012, .77, f'{self.part} predicted mesh', fontsize=22, weight='bold', color=self.INK, va='center')
        stage = 'ROI rescan complete' if self.metadata.get('rescan_of') else 'Survey complete'
        band.text(.012, .44, f'{stage}  ·  {stamp}', fontsize=11, color=self.MUTED, va='center')
        if web:
            owner = f'  ·  {web["owner"]}' if web.get('owner') else ''
            band.text(.012, .17, f'{web.get("name", web["model"])}  ·  {web["plate"]}{owner}',
                      fontproperties=report_font(11), color=self.MUTED, va='center', parse_math=False)
        for k, (value, label, detail) in enumerate(self.summary()):
            left = .52 + k * .162
            band.add_patch(FancyBboxPatch((left, .12), .148, .76, boxstyle='round,pad=0,rounding_size=.08',
                                          transform=band.transAxes, facecolor='white', edgecolor='#d5dde6', lw=1))
            accent = (self.COLORS['crack'], self.COLORS['dent'], self.AREA)[k]
            band.add_patch(Rectangle((left + .004, .26), .004, .48, transform=band.transAxes, color=accent))
            band.text(left + .018, .56, value, fontsize=24, weight='bold', color=accent, va='center')
            band.text(left + .055, .64, label, fontsize=11, weight='bold', color=self.INK, va='center')
            band.text(left + .055, .36, detail, fontsize=9.5, color=self.MUTED, va='center')

    def perspective(self, ax):
        """The whole part in perspective, its height 3x, the defects and the areas laid on its surface."""
        low, high = self.reference[:, :2].min(0) - 1., self.reference[:, :2].max(0) + 1.
        X, Y, Z, off = self.grid(low, high, .6)
        self.overview = RegularGridInterpolator((Y[:, 0], X[0]), Z, bounds_error=False, fill_value=None)
        colors = np.dstack([self.lit(Z, .6, 2.), (~off).astype(float)])
        ax.plot_surface(X, Y, Z, facecolors=colors, rstride=1, cstride=1, linewidth=0, antialiased=False, shade=False)
        span = high - low
        ax.set_box_aspect((span[0], span[1], max(np.ptp(Z[~off]) * 3, 1.)), zoom=1.02)
        ax.view_init(elev=40, azim=-128)
        ax.set_axis_off()
        for number, box in enumerate(self.areas, 1):
            a, b = np.asarray(box[:2]), np.asarray(box[2:])
            edge = np.vstack([np.linspace(c, d, 40) for c, d in ((a, [b[0], a[1]]), ([b[0], a[1]], b),
                                                                (b, [a[0], b[1]]), ([a[0], b[1]], a))])
            ax.plot(*edge.T, self.on_surface(edge), color=self.AREA, lw=1.3, ls=(0, (4, 3)), zorder=8)
            ax.text(a[0], b[1], self.on_surface(np.array([[a[0], b[1]]]))[0] + 1.5, f'A{number}', color=self.AREA,
                    fontsize=10, weight='bold', zorder=9)
        for name, kind, _, xy in self.defects:
            if kind == 'crack':
                line = self.trace(xy)
                z = self.on_surface(line)
                ax.plot(*line.T, z, color=self.GLOW[kind], lw=7, alpha=.35, solid_capstyle='round', zorder=10)
                ax.plot(*line.T, z, color=self.COLORS[kind], lw=2.4, solid_capstyle='round', zorder=11)
                top = line[np.argmax(line[:, 1])]
            else:
                ring = self.outline(xy)
                ring = np.vstack([ring, ring[:1]])
                z = self.on_surface(ring)
                ax.plot(*ring.T, z, color=self.GLOW[kind], lw=8, alpha=.45, zorder=10)  # mplot3d hides fills under
                ax.plot(*ring.T, z, color=self.COLORS[kind], lw=2.4, zorder=11)         # the surface: an outline
                centre = ring[:-1].mean(0)
                ax.scatter(*centre, self.on_surface(centre[None])[0], s=40, c=self.COLORS[kind], edgecolors='white',
                           linewidths=1., depthshade=False, zorder=12)
                top = ring[np.argmax(ring[:, 1])]
            ax.text(*top, self.on_surface(top[None])[0] + 2.5, name, color=self.COLORS[kind], fontsize=11,
                    weight='bold', zorder=12, path_effects=self.HALO)
        ax.set_title(f'{self.part}  ·  perspective, depth scale ×3', fontsize=12, color=self.INK, loc='left', pad=2)

    def zoom(self, ax, number, box):
        """One rescanned area from above: lit mesh, scan rows, defects with their crossings, callouts, scale bar."""
        low, high = np.asarray(box[:2]), np.asarray(box[2:])
        X, Y, Z, off = self.grid(low, high, .1)
        image = self.lit(Z, .1, 1.)
        image[off] = 1.
        ax.imshow(image, extent=(X[0, 0], X[0, -1], Y[0, 0], Y[-1, 0]), origin='lower', interpolation='bilinear')
        inside = np.all((self.measured[:, :2] >= low) & (self.measured[:, :2] <= high), axis=1)
        scanned = self.measured[inside][::max(1, int(inside.sum() // 20000))]
        ax.scatter(*scanned[:, :2].T, s=.5, c=self.AREA, alpha=.22, linewidths=0, zorder=2)
        for name, kind, regions, xy in self.defects:
            color = self.COLORS[kind]
            if kind == 'crack':
                line = self.trace(xy)
                ax.plot(*line.T, color=self.GLOW[kind], lw=9, alpha=.35, solid_capstyle='round', zorder=4)
                ax.plot(*line.T, color=color, lw=2.6, solid_capstyle='round', zorder=5)
                ax.scatter(*xy.T, s=14, c='white', edgecolors=color, linewidths=1., zorder=6)
                anchor = line[len(line) // 2]
                detail = f'{np.linalg.norm(np.diff(line, axis=0), axis=1).sum():.1f} mm'
            else:
                ring = self.outline(xy)
                centre = ring.mean(0)
                for scale in np.linspace(1., .2, 6):
                    ax.add_patch(Polygon(centre + (ring - centre) * scale, closed=True, facecolor=color, alpha=.13,
                                         lw=0, zorder=4))
                ax.add_patch(Polygon(ring, closed=True, fill=False, edgecolor=color, lw=1.8, zorder=5))
                anchor = centre
                detail = f'{max(r["max_deviation_mm"] for r in regions):.2f} mm deep'
            if np.all((anchor >= low) & (anchor <= high)):
                ax.annotate(f'{name}  {detail}', anchor, xytext=(18, 16), textcoords='offset points', fontsize=9.5,
                            weight='bold', color=color, zorder=7,
                            bbox=dict(boxstyle='round,pad=.3', facecolor='white', edgecolor=color, lw=1.),
                            arrowprops=dict(arrowstyle='-', color=color, lw=1.))
        ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), aspect='equal')
        ax.tick_params(labelsize=8, colors=self.MUTED, length=2)
        for spine in ax.spines.values():
            spine.set(color=self.AREA, lw=1.6)
        ax.add_artist(AnchoredSizeBar(ax.transData, 5, '5 mm', 'lower right', pad=.4, frameon=False,
                                      color=self.INK, size_vertical=.12, fontproperties=dict(size=8)))
        size = high - low
        ax.set_title(f'Area A{number}  ·  {size[0]:.0f} x {size[1]:.0f} mm', fontsize=11, color=self.AREA,
                     loc='left', weight='bold', pad=3)

    def table(self, ax):
        ax.axis('off')
        if not self.defects:
            ax.text(0., .8, 'No confirmed crack or dent.', fontsize=11, color=self.MUTED)
            return
        rows, colors = [], []
        for name, kind, regions, xy in self.defects:
            low, high = xy.min(0), xy.max(0)
            size = (f'{np.linalg.norm(np.diff(self.trace(xy), axis=0), axis=1).sum():.1f} mm long' if kind == 'crack'
                    else f'{high[0] - low[0]:.1f} x {high[1] - low[1]:.1f} mm')
            rows.append([name, kind.capitalize(), size, f'{max(r["max_deviation_mm"] for r in regions):.3f}',
                         f'{min(r["force_residual_n"] for r in regions):+.2f}',
                         f'{sum(r["force_samples"] for r in regions)} / {len(regions)}',
                         'Confirmed' if any(r['confirmed'] for r in regions) else 'Candidate',
                         ', '.join(f'{v:.1f}' for v in xy.mean(0)),
                         'Survey' if any(r.get('source') == 'survey' for r in regions) else 'Rescan'])
            colors.append(self.COLORS[kind])
        table = ax.table(cellText=rows, colLabels=['ID', 'Type', 'Size', 'Depth (mm)', 'Force (N)', 'Samples / ROIs',
                                                   'Status', 'Centre BASE (mm)', 'Source'],
                         loc='upper left', cellLoc='left', colLoc='left', bbox=(0, 0, 1, 1))
        table.auto_set_font_size(False)
        for (row, column), cell in table.get_celld().items():
            cell.set(edgecolor='#dfe5ec', linewidth=.6)
            cell.get_text().set(fontsize=9.5, color=self.INK)
            if row == 0:
                cell.set_facecolor(self.INK)
                cell.get_text().set(color='white', weight='bold')
            else:
                cell.set_facecolor('#f6f8fb' if row % 2 else 'white')
                if column in (0, 1):
                    cell.get_text().set(color=colors[row - 1], weight='bold')

    def save(self, path):
        areas = self.areas[:3]
        fig = plt.figure(figsize=(17, 9.6 + .35 * len(self.defects)), facecolor='white')
        try:
            outer = fig.add_gridspec(3, 1, height_ratios=(.15, 1, .07 + .05 * max(1, len(self.defects))),
                                     hspace=.08, left=.03, right=.97, top=.98, bottom=.03)
            self.header(fig, fig.add_subplot(outer[0]))
            main = outer[1].subgridspec(max(1, len(areas)), 2, width_ratios=(1.35, 1), wspace=.06, hspace=.28)
            if self.height is not None:
                self.perspective(fig.add_subplot(main[:, 0], projection='3d'))
                for number, box in enumerate(areas, 1):
                    self.zoom(fig.add_subplot(main[number - 1, 1]), number, box)
            self.table(fig.add_subplot(outer[2]))
            path.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(path, dpi=130, facecolor='white')
        finally:
            plt.close(fig)
        return path
