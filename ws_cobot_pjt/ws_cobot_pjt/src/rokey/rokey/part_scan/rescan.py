"""Rescan areas of a mesh-height survey: the ROIs of the chosen types (confirmed and candidates) joined with their
neighbours into BASE rectangles for real_scan --roi-base at the rescan speed. 072408 (4 mm/s, 2.4 N) left 8 crack
candidates 1-4 mm apart along both cracks: one area once joined, the two cracks 4.1 mm apart."""
import numpy as np


class RescanAreas:
    def __init__(self, settings):
        self.settings = settings

    def boxes(self, regions):
        """[[x0, y0, x1, y1], ...] BASE mm: each ROI's box margin_mm wider, those overlapping or within merge_mm on
        both axes joined until none are (no area scanned twice)."""
        margin = self.settings['margin_mm']
        boxes = [np.take(g['bounds_mm'], [0, 1, 3, 4]) + np.r_[-margin, -margin, margin, margin]
                 for g in regions if g['type'] in self.settings['types']]
        i = 0
        while i < len(boxes):
            for j in range(i + 1, len(boxes)):
                a, b = boxes[i], boxes[j]
                if np.all(np.maximum(a[:2], b[:2]) - np.minimum(a[2:], b[2:]) <= self.settings['merge_mm']):
                    boxes[i] = np.r_[np.minimum(a[:2], b[:2]), np.maximum(a[2:], b[2:])]
                    del boxes[j]
                    i = -1  # the grown box may now reach earlier ones
                    break
            i += 1
        return [box.round(1).tolist() for box in boxes]
