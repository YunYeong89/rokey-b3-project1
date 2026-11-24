"""User-drawn bonnet inspection areas in vehicle X/Y, in millimetres."""
import json
from pathlib import Path

import numpy as np


class BonnetPath:
    def __init__(self, regions):
        if not isinstance(regions, list) or not 1 <= len(regions) <= 8:
            raise ValueError('Draw 1 to 8 inspection areas.')
        self.regions = [np.asarray(region, float) for region in regions]
        for polygon in self.regions:
            if (polygon.ndim != 2 or polygon.shape[1] != 2 or not 3 <= len(polygon) <= 400
                    or not np.isfinite(polygon).all()):
                raise ValueError('Each area needs 3 to 400 valid X/Y coordinates.')
            x, y = polygon.T
            if abs(x @ np.roll(y, -1) - y @ np.roll(x, -1)) < 8.:
                raise ValueError('The inspection area is too narrow. Draw a wider outline.')

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text())['regions'])

    def contains(self, points, margin=0.):
        """Union of the drawn polygons, using even–odd fill as in the editor."""
        x, y = np.asarray(points, float).T
        selected = np.zeros(len(x), bool)
        for polygon in self.regions:
            inside = np.zeros(len(x), bool)
            for (ax, ay), (bx, by) in zip(polygon, np.roll(polygon, -1, axis=0)):
                crossing = (ay > y) != (by > y)
                if crossing.any():
                    inside[crossing] ^= x[crossing] < ax + (bx - ax) * (y[crossing] - ay) / (by - ay)
            if margin and inside.any():
                indexes = np.flatnonzero(inside)
                coords = np.c_[x[indexes], y[indexes]]
                for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
                    delta = b - a
                    foot = a + np.clip(((coords - a) @ delta) / max(delta @ delta, 1e-12), 0., 1.)[:, None] * delta
                    inside[indexes] &= np.sum((coords - foot) ** 2, axis=1) >= margin ** 2
            selected |= inside
        return selected

    def coordinates(self):
        return [region.tolist() for region in self.regions]

    def enclosing_trapezoid(self):
        """Minimum area with front/rear edges parallel to vehicle Y, over all the drawn points."""
        from scipy.optimize import linprog

        points = np.concatenate(self.regions)
        x, y = points.T
        ends = np.array([x.min(), x.max()])
        u = (x - ends.mean()) / np.ptp(ends)
        terms = np.c_[u, np.ones(len(u))]
        lines = []
        for sign in (1., -1.):
            # Area depends on side-to-side width at the midpoint. Minimize each outward line there.
            solution = linprog([0., 1.], A_ub=-terms, b_ub=-sign * y, bounds=(None, None), method='highs')
            if not solution.success:
                raise ValueError('Cannot compute a trapezoid that contains the saved outline.')
            slope, intercept = solution.x
            intercept = max(intercept, np.max(sign * y - slope * u))  # include points on the support line
            lines.append(sign * (intercept + slope * np.array([-.5, .5])))
        left, right = lines
        return np.array([[ends[0], left[0]], [ends[1], left[1]], [ends[1], right[1]], [ends[0], right[0]]])
