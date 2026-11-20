"""Rescan areas of a mesh-height survey: the ROIs of the chosen types (confirmed and candidates), each joined with its
near neighbours only, as BASE rectangles for real_scan --roi-base. 072408 (4 mm/s, 2.4 N) left 8 crack candidates
1-4 mm apart along each crack and the two cracks 4.6 mm apart: joined with the margins first, all of them made one
44.6 x 26.5 mm area."""
import numpy as np


class RescanAreas:
    def __init__(self, settings):
        self.settings = settings

    def boxes(self, regions):
        """[[x0, y0, x1, y1], ...] BASE mm: ROI boxes at most merge_mm apart (the gap between the boxes themselves)
        joined until none are, then each margin_mm wider; areas only touching through their margins stay apart."""
        boxes = [np.take(g['bounds_mm'], [0, 1, 3, 4]) for g in regions if g['type'] in self.settings['types']]
        i = 0
        while i < len(boxes):
            for j in range(i + 1, len(boxes)):
                a, b = boxes[i], boxes[j]
                if np.linalg.norm(np.maximum(np.maximum(a[:2], b[:2]) - np.minimum(a[2:], b[2:]), 0.)) <= \
                        self.settings['merge_mm']:
                    boxes[i] = np.r_[np.minimum(a[:2], b[:2]), np.maximum(a[2:], b[2:])]
                    del boxes[j]
                    i = -1  # the grown box may now reach earlier ones
                    break
            i += 1
        margin = self.settings['margin_mm']
        return [(box + np.r_[-margin, -margin, margin, margin]).round(1).tolist() for box in boxes]
