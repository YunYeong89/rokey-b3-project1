"""ROI rescan report (plan.json rescan_of): the cracks and dents found drawn on the part's mesh itself (its nominal
reference cloud, lit as a surface) instead of ROI boxes over the measured cloud. The whole part with the rescanned
areas, then each area enlarged. Crack ROIs within JOIN_MM of each other are one crack (081134: crack B came as four
pieces), drawn as a line through their crossings in order along it, lightly smoothed; a dent is its samples' outline."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.patheffects as effects
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from matplotlib.patches import Polygon, Rectangle
import numpy as np
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from scipy.spatial import ConvexHull, QhullError, cKDTree


class DefectMap:
    COLORS = {'crack': '#d1203a', 'dent': '#f08a24'}
    LIGHT = LightSource(azdeg=315, altdeg=40)
    SMOOTH_MM = 1.5
    PAINT = np.array([.80, .84, .88])  # the bonnet's paint, light steel grey
    HALO = [effects.withStroke(linewidth=4.5, foreground='white')]
    JOIN_MM = 4.

    def __init__(self, reference, measured, defects, metadata):
        self.reference = np.asarray(reference, float).reshape(-1, 3)
        self.measured = np.asarray(measured, float).reshape(-1, 3)
        self.metadata, self.areas = metadata, metadata.get('roi_base_mm') or []
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

    def surface(self, panel, low, high, cell):
        """The mesh's reference cloud in [low, high] (BASE x, y) on `cell` mm, smoothed over SMOOTH_MM (the sampled
        points' own facets otherwise light up as grain), one paint colour lit from the top left (height as colour made
        blotches of a few hundredths of a mm in an area); white off the mesh."""
        near = np.all((self.reference[:, :2] >= low - 2.) & (self.reference[:, :2] <= high + 2.), axis=1)
        x, y = np.arange(low[0], high[0] + cell, cell), np.arange(low[1], high[1] + cell, cell)
        height = griddata(self.reference[near, :2], self.reference[near, 2], tuple(np.meshgrid(x, y)), method='linear')
        off = np.isnan(height)
        sigma = self.SMOOTH_MM / cell
        weight = gaussian_filter((~off).astype(float), sigma)
        height = gaussian_filter(np.where(off, 0., height), sigma) / np.maximum(weight, 1e-9)
        height[off] = height[~off].min()
        light = self.LIGHT.hillshade(height, vert_exag=3. if cell >= .5 else 1., dx=cell, dy=cell)  # zoom: no blotches
        image = self.PAINT * (.62 + .38 * light[..., None])
        image[off] = 1.
        panel.imshow(image, extent=(x[0], x[-1], y[0], y[-1]), origin='lower', interpolation='bilinear')
        panel.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), aspect='equal',
                  xlabel='BASE X (mm)', ylabel='BASE Y (mm)')

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

    def draw(self, panel, scale=1.):
        for name, kind, _, xy in self.defects:
            color = self.COLORS[kind]
            if kind == 'crack':
                line = self.trace(xy)
                panel.plot(*line.T, color=color, lw=2.6 * scale, solid_capstyle='round', solid_joinstyle='round',
                           marker='o' if len(line) < 3 else None, ms=4 * scale, path_effects=self.HALO, zorder=5)
                anchor = line[-1]
            else:
                outline = self.outline(xy)
                panel.add_patch(Polygon(outline, closed=True, facecolor=color, alpha=.4, zorder=4))
                panel.add_patch(Polygon(outline, closed=True, fill=False, edgecolor=color, lw=2. * scale, zorder=5))
                anchor = outline[np.argmax(outline[:, 1])]
            panel.annotate(name, anchor, xytext=(4, 4), textcoords='offset points', fontsize=9 * scale, weight='bold',
                           color=color, path_effects=self.HALO, zorder=6)

    def rows(self):
        lines = []
        for name, kind, regions, xy in self.defects:
            low, high = xy.min(0), xy.max(0)
            size = (f'length {np.linalg.norm(np.diff(self.trace(xy), axis=0), axis=1).sum():.1f} mm' if kind == 'crack'
                    else f'{high[0] - low[0]:.1f} x {high[1] - low[1]:.1f} mm')
            lines.append(f'{name}  {kind:<5}  {size:<18}  depth {max(r["max_deviation_mm"] for r in regions):.3f} mm  '
                         f'force {min(r["force_residual_n"] for r in regions):+.2f} N  '
                         f'{sum(r["force_samples"] for r in regions)} samples in {len(regions)} ROI  '
                         f'{"confirmed" if any(r["confirmed"] for r in regions) else "candidate"}  '
                         f'centre BASE {np.round(xy.mean(0), 1).tolist()}')
        return lines or ['No crack or dent met the rescan criteria.']

    def save(self, path):
        panels = 1 + len(self.areas)
        fig = plt.figure(figsize=(6.2 * panels, 8.4), layout='constrained')
        try:
            grid = fig.add_gridspec(2, panels, height_ratios=(1, .22))
            whole = fig.add_subplot(grid[0, 0])
            if len(self.reference):
                low, high = self.reference[:, :2].min(0) - 3., self.reference[:, :2].max(0) + 3.
                self.surface(whole, low, high, .5)
            for number, box in enumerate(self.areas, 1):
                low, high = np.asarray(box[:2]), np.asarray(box[2:])
                whole.add_patch(Rectangle(low, *(high - low), fill=False, edgecolor='#2a6fdb', lw=1.4, ls='--',
                                          zorder=3))
                whole.annotate(f'A{number}', (low[0], high[1]), xytext=(0, 3), textcoords='offset points',
                               fontsize=9, color='#2a6fdb', weight='bold')
                panel = fig.add_subplot(grid[0, number])
                if len(self.reference):
                    self.surface(panel, low, high, .1)
                inside = np.all((self.measured[:, :2] >= low) & (self.measured[:, :2] <= high), axis=1)
                scanned = self.measured[inside][::max(1, int(inside.sum() // 20000))]
                panel.scatter(*scanned[:, :2].T, s=.6, c='#2a6fdb', alpha=.3, linewidths=0, zorder=2)
                self.draw(panel, 1.2)
                panel.set_title(f'A{number} · rescanned at {self.metadata["speed_mm_s"]:g} mm/s', fontsize=11)
            self.draw(whole)
            whole.set_title('Bonnet mesh · cracks and dents found', fontsize=11)
            text = fig.add_subplot(grid[1, :])
            text.axis('off')
            kinds = [kind for _, kind, _, _ in self.defects]
            text.text(0., .95, f'{kinds.count("crack")} crack(s), {kinds.count("dent")} dent(s) · rescan criteria '
                               f'(survey {self.metadata.get("rescan_of", "-")})', fontsize=14, weight='bold',
                      color='#1f3349', va='top')
            text.text(0., .62, '\n'.join(self.rows()), fontsize=9.5, family='monospace', va='top', color='#2b3a4a')
            path.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(path, dpi=140)
        finally:
            plt.close(fig)
        return path
