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

    def peaks(self, points, depth, force, tangential, heading, groups, usable, contact_threshold, *, channels=None):
        """Compare each crossing with healthy reference-supported shoulders on both sides. `tangential`: the reaction
        across the press axis (N, 3). -> crossings, [depth drop mm, force evidence N]"""
        usable = usable & np.isfinite(depth) & np.isfinite(force)
        length = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
        joined = ((heading[1:] == heading[:-1]) & (groups[1:] == groups[:-1]) &
                  usable[1:] & usable[:-1] & (length <= .35))
        cuts = np.r_[0, np.flatnonzero(~joined) + 1, len(points)]
        shoulder = self.settings['shoulder_mm']
        offset = round(shoulder / self.STEP_MM)
        channels = np.empty((len(points), 0)) if channels is None else channels
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
            if self.settings.get('force_led'):
                # The crossing found by its force evidence, the depth only checked: pressed stiffly the ball is held
                # on the path (081134, 3 N/mm: 3 and 5 crossings with any depth drop on cracks A and B, but 7 and 10
                # with the reaction across the axis changing 0.5 N or more).
                evidence = np.maximum(loss / self.settings['force_drop_n'], lateral / self.settings['lateral_n'])
                hits, _ = find_peaks(evidence, height=1., distance=offset,
                                     width=(1., self.settings['width_mm'] / self.STEP_MM))
                hits = hits[(drop[hits] >= self.settings['depth_mm']) & (shoulders[hits] > contact_threshold)]
            else:
                hits, _ = find_peaks(drop, height=self.settings['depth_mm'], distance=offset,
                                     width=(1., self.settings['width_mm'] / self.STEP_MM))
                hits = hits[((loss[hits] >= self.settings['force_drop_n']) |
                             (lateral[hits] >= self.settings['lateral_n'])) & (shoulders[hits] > contact_threshold)]
            if len(hits):
                peaks.extend(lo + np.searchsorted(distance, s[hits + offset]))
                extra = [gaussian_filter1d(np.interp(s, distance, channel[lo:hi]), 1.) for channel in channels.T]
                centres = hits + offset
                delta = [values[centres] - (values[hits] + values[hits + 2 * offset]) / 2 for values in extra]
                contrasts.extend(np.column_stack([drop[hits], np.maximum(loss[hits], lateral[hits]), *delta]))
        return np.asarray(peaks, int), np.asarray(contrasts).reshape(-1, 2 + channels.shape[1])

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
            keep = (across >= .5) & (across <= step) & (delta[:, axis] <= self.settings.get('trace_slope', 1.) * across)
            graph = coo_matrix((np.ones(keep.sum()), (i[keep], j[keep])), shape=(len(at), len(at)))
            count, labels = connected_components(graph, directed=False)
            for group in range(count):
                own = np.flatnonzero(labels == group)
                if len(own) < 2:
                    continue
                centred = xy[own] - xy[own].mean(0)
                _, _, vectors = np.linalg.svd(centred, full_matrices=False)
                length, width = np.ptp(centred @ vectors.T, axis=0)
                if width <= self.settings.get('trace_width_mm', self.settings['width_mm']):  # spread across the trace
                    opposite = np.unique(heading[at[own]]).size == 2
                    confirmed = len(own) >= 3 and np.ptp(xy[own, 1 - axis]) >= self.settings['length_mm'] and opposite
                    joined[at[own]] = True
                    yield at[own], float(length), float(width), confirmed
        for k in np.flatnonzero(~joined & (evidence >= self.settings['single_n'])):
            yield np.array([k]), 0., 0., False


class DoorCrackRoi(CrackRoi):
    """Find repeated force crossings on the plane transverse to a door's press axis."""

    def __init__(self, settings):
        super().__init__({**settings, 'force_led': True, 'single_n': np.inf})

    def detect(self, points, target, axis, force, wrench, velocity, paths, groups, stroke, speed, threshold):
        plane = np.flatnonzero(np.abs(axis) < .9)
        uv = points[:, plane]
        smooth = np.empty_like(uv)
        cuts = np.r_[0, np.flatnonzero(np.diff(groups)) + 1, len(groups)]
        for first, last in zip(cuts[:-1], cuts[1:]):
            smooth[first:last] = gaussian_filter1d(velocity[first:last, plane], 5., axis=0)
        travel = np.argmax(np.abs(smooth), axis=1)
        major = np.take_along_axis(smooth, travel[:, None], axis=1)[:, 0]
        minor = np.take_along_axis(smooth, (1 - travel)[:, None], axis=1)[:, 0]
        heading = 2 * travel + (major < 0.)
        usable = stroke & (np.abs(major) > max(.5, .36 * speed)) & (np.abs(minor) < .33 * np.abs(major))
        for path in np.unique(paths[usable]):
            own = paths == path
            primary = np.bincount(travel[own & usable]).argmax()
            usable[own] &= travel[own] == primary
        depth = (target - points) @ axis
        peaks, values = self.peaks(uv, depth, force, wrench[:, plane], heading, groups, usable, threshold,
                                   channels=np.c_[target @ axis, wrench])
        quiet = np.abs(values[:, 2]) <= self.settings['mesh_contrast_mm']
        peaks, values = peaks[quiet], values[quiet]
        for own, length, width, confirmed in self.traces(uv[peaks], heading[peaks], values[:, 1]):
            at = peaks[own]
            across = 1 - travel[at[0]]
            lanes = np.unique(np.floor(uv[at, across] / .8)).size
            if lanes >= self.settings['min_rows'] and length >= self.settings['length_mm']:
                yield at, values[own], length, width, confirmed, lanes
