"""Simulated damage on the contact-test block top (robot.yaml contact_test.block_defects): a straight tapered
crack (closing to zero width over tip_mm at both ends, 45 deg chamfered edges) and a smooth dent, in block-local mm
(origin at the block centre, axes of BASE). Used by the native scene for the block's collision mesh."""
import numpy as np


class DefectBlock:
    def __init__(self, test):
        self.size, self.top = test['block_size_mm'], test['surface_z_mm']
        self.crack, self.dent = test['block_defects']['crack'], test['block_defects']['dent']

    def crack_frame(self):
        start, end = np.array(self.crack['start_mm'], float), np.array(self.crack['end_mm'], float)
        return start, end, np.linalg.norm(end - start), (end - start) / np.linalg.norm(end - start)

    def crack_width(self, u):
        low, high = self.crack['width_mm']
        u, length = np.clip(u, 0, 1), self.crack_frame()[2]
        return (low + (high - low) * u) * np.sqrt(np.clip(np.minimum(u, 1 - u) * length / self.crack['tip_mm'], 0, 1))

    def height(self, x, y):
        """Surface height relative to the block top (mm, <= 0) at block-local x, y."""
        x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
        start, _, length, axis = self.crack_frame()
        u = ((x - start[0]) * axis[0] + (y - start[1]) * axis[1]) / length
        across = np.abs((y - start[1]) * axis[0] - (x - start[0]) * axis[1])
        gap = np.where((u >= 0) & (u <= 1), self.crack_width(u) / 2 - across, 0.)  # inward from the crack edge
        # 45 deg chamfer for the first chamfer_mm, then the vertical wall: a sharp mesh edge pushed the ball back
        # with a near-horizontal contact normal in Gazebo.
        crack = np.where(gap <= 0, 0., np.where(gap < self.crack['chamfer_mm'], -gap, -self.crack['depth_mm']))
        d = self.dent
        r = np.hypot(x - d['centre_mm'][0], y - d['centre_mm'][1])
        dent = np.where(r < d['radius_mm'], -d['depth_mm'] * .5 * (1 + np.cos(np.pi * r / d['radius_mm'])), 0.)
        return np.minimum(crack, dent)

    def trace(self, x, y, radius, step=.05):
        """Height of a resting ball's lowest point (mm): a crack narrower than the ball is bridged, its dip only
        r - sqrt(r^2 - w^2/4) (0.02 mm at w = 0.5 mm, 0.38 mm at 2 mm for r = 1.5 mm)."""
        offsets = np.arange(-radius, radius + step / 2, step)
        dx, dy = (v.ravel() for v in np.meshgrid(offsets, offsets))
        inside = dx ** 2 + dy ** 2 <= radius ** 2
        dx, dy = dx[inside], dy[inside]
        cap = np.sqrt(np.maximum(radius ** 2 - dx ** 2 - dy ** 2, 0)) - radius
        x, y = np.atleast_1d(np.asarray(x, float)), np.atleast_1d(np.asarray(y, float))
        return (self.height(x[:, None] + dx, y[:, None] + dy) + cap).max(axis=1)

    def axes(self):
        """Mesh grid lines (mm), aligned to multiples of their step: 2 mm on the flat top, 0.1 x 0.25 mm over the crack
        (chamfer and walls), 0.25 mm over the dent."""
        half, start, end = self.size / 2, *self.crack_frame()[:2]
        pad, d = max(self.crack['width_mm']) / 2 + .5, self.dent

        def lines(low, high, step):
            return np.arange(np.ceil(max(low, -half) / step) * step, min(high, half) + step / 2, step)
        coarse = np.linspace(-half, half, int(round(self.size / 2)) + 1)
        xs = [coarse, lines(min(start[0], end[0]) - pad, max(start[0], end[0]) + pad, .1),
              lines(d['centre_mm'][0] - d['radius_mm'], d['centre_mm'][0] + d['radius_mm'], .25)]
        ys = [coarse, lines(min(start[1], end[1]) - pad, max(start[1], end[1]) + pad, .25),
              lines(d['centre_mm'][1] - d['radius_mm'], d['centre_mm'][1] + d['radius_mm'], .25)]
        return (np.unique(np.round(np.concatenate(v), 4)) for v in (xs, ys))

    def triangles(self, bottom):
        """Closed block mesh (mm; x, y block-local, z in BASE): the damaged top, four sides down to `bottom`, base."""
        xs, ys = self.axes()
        X, Y = np.meshgrid(xs, ys, indexing='ij')
        top = np.stack([X, Y, self.top + self.height(X, Y)], -1)
        a, b, c, d = top[:-1, :-1], top[1:, :-1], top[1:, 1:], top[:-1, 1:]
        faces = [np.stack(f, -2).reshape(-1, 3, 3) for f in ((a, b, c), (a, c, d))]
        loop = np.r_[np.c_[xs, np.full(len(xs), ys[0])], np.c_[np.full(len(ys) - 1, xs[-1]), ys[1:]],
                     np.c_[xs[-2::-1], np.full(len(xs) - 1, ys[-1])], np.c_[np.full(len(ys) - 2, xs[0]), ys[-2:0:-1]]]
        p, q = loop, np.roll(loop, -1, 0)
        up = lambda e: np.c_[e, self.top + self.height(e[:, 0], e[:, 1])]
        down = lambda e: np.c_[e, np.full(len(e), bottom)]
        faces += [np.stack([down(p), down(q), up(q)], 1), np.stack([down(p), up(q), up(p)], 1)]
        corners = np.array([[xs[0], ys[0]], [xs[-1], ys[0]], [xs[-1], ys[-1]], [xs[0], ys[-1]]])
        base = down(corners)
        faces.append(np.array([[base[0], base[2], base[1]], [base[0], base[3], base[2]]]))
        return np.concatenate(faces)
