"""Detect narrow depressions that repeat across adjacent raster strokes.

A crossing: a narrow depth drop below both 1 mm shoulders of the stroke, with force evidence there: the normal reaction
dropping, or the reaction across the press axis changing (the groove's flanks push the ball sideways). On the real
bonnet (2026-11-20, CAD crack centrelines) both cracks gave 0.26-1.26 N across the axis on 5 mm and 2 mm balls while
the normal reaction rose or fell by run, so the normal drop alone missed them. Crossings on adjacent rows (one missed
row allowed) form a crack; a lone crossing with strong force evidence is a candidate too."""
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


class CrackRoi:
    STEP_MM = .1

    def __init__(self, settings):
        self.settings = settings

    def peaks(self, points, depth, force, tangential, heading, groups, usable, contact_threshold):
        """Compare each crossing with healthy reference-supported shoulders on both sides. `tangential`: the reaction
        across the press axis (N, 3). -> crossings, [depth drop mm, force evidence N]"""
        usable = usable & np.isfinite(depth) & np.isfinite(force)
        length = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
        joined = ((heading[1:] == heading[:-1]) & (groups[1:] == groups[:-1]) &
                  usable[1:] & usable[:-1] & (length <= .35))
        cuts = np.r_[0, np.flatnonzero(~joined) + 1, len(points)]
        shoulder = self.settings['shoulder_mm']
        offset = round(shoulder / self.STEP_MM)
        peaks, contrasts = [], []
        for lo, hi in zip(cuts[:-1], cuts[1:]):
            if hi - lo < 3:
                continue
            axis, reverse = divmod(heading[lo], 2)
            distance = (points[lo:hi, axis] - points[lo, axis]) * (1 - 2 * reverse)
            if np.any(np.diff(distance) < 0):
                continue
            if distance[-1] < 2 * shoulder + 2 * self.STEP_MM:
                continue
            s = np.arange(0., distance[-1], self.STEP_MM)
            z, f, *t = (gaussian_filter1d(np.interp(s, distance, values[lo:hi]), 1.)
                        for values in (depth, force, *tangential.T))
            t = np.array(t)
            drop = z[offset:-offset] - np.maximum(z[:-2 * offset], z[2 * offset:])
            shoulders = np.minimum(f[:-2 * offset], f[2 * offset:])
            loss = shoulders - f[offset:-offset]
            lateral = np.linalg.norm(t[:, offset:-offset] - (t[:, :-2 * offset] + t[:, 2 * offset:]) / 2, axis=0)
            hits, _ = find_peaks(drop, height=self.settings['depth_mm'], distance=offset,
                                 width=(1., self.settings['width_mm'] / self.STEP_MM))
            hits = hits[((loss[hits] >= self.settings['force_drop_n']) | (lateral[hits] >= self.settings['lateral_n']))
                        & (shoulders[hits] > contact_threshold)]
            if len(hits):
                peaks.extend(lo + np.searchsorted(distance, s[hits + offset]))
                contrasts.extend(np.c_[drop[hits], np.maximum(loss[hits], lateral[hits])])
        return np.asarray(peaks, int), np.asarray(contrasts).reshape(-1, 2)

    def traces(self, points, heading, evidence):
        """Join nearby crossings on different raster rows into elongated regions; a crossing in none with force
        `evidence` (N) of at least single_n stands alone (unconfirmed)."""
        step = self.settings['row_gap_mm']
        joined = np.zeros(len(points), bool)
        for axis in (0, 1):
            at = np.flatnonzero(heading // 2 == axis)
            xy = points[at, :2]
            i, j = cKDTree(xy).query_pairs(1.5 * step, output_type='ndarray').T
            delta = np.abs(xy[i] - xy[j])
            across = delta[:, 1 - axis]
            keep = (across >= .5) & (across <= step) & (delta[:, axis] <= across)
            graph = coo_matrix((np.ones(keep.sum()), (i[keep], j[keep])), shape=(len(at), len(at)))
            count, labels = connected_components(graph, directed=False)
            for group in range(count):
                own = np.flatnonzero(labels == group)
                if len(own) < 2:
                    continue
                centred = xy[own] - xy[own].mean(0)
                _, _, vectors = np.linalg.svd(centred, full_matrices=False)
                length, width = np.ptp(centred @ vectors.T, axis=0)
                if width <= self.settings['width_mm']:
                    opposite = np.unique(heading[at[own]]).size == 2
                    confirmed = len(own) >= 3 and np.ptp(xy[own, 1 - axis]) >= self.settings['length_mm'] and opposite
                    joined[at[own]] = True
                    yield at[own], float(length), float(width), confirmed
        for k in np.flatnonzero(~joined & (evidence >= self.settings['single_n'])):
            yield np.array([k]), 0., 0., False
