"""Join nearby mesh-height survey ROIs into rectangles in the panel's BASE plane."""
import numpy as np


class RescanAreas:
    def __init__(self, settings):
        self.settings = settings

    def boxes(self, regions, plane=(0, 1)):
        """BASE mm: ROI boxes at most merge_mm apart (the gap between the boxes themselves)
        joined until none are, then each margin_mm wider; areas only touching through their margins stay apart."""
        u, v = plane
        boxes = [np.take(g['bounds_mm'], [u, v, u + 3, v + 3]) for g in regions if g['type'] in self.settings['types']]
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
