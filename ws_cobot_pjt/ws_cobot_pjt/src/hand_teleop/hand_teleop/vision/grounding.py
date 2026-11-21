"""Neutral hand capture (grounding) and the hand relative to it."""
import numpy as np
from scipy.spatial.transform import Rotation

from .features import HandSample
from .filter import OneEuroFilter


class Grounding:
    """After a request (id >= 0) an open palm held still for `seconds` becomes the neutral hand: wrist position and
    depth, hand frame and finger curl, averaged. From then on `update` reports the hand relative to it in camera axes
    (x right, y down, z away from the lens of the raw image): offset in mm (metric through `focal_px`), rotation since
    grounding as a quaternion (x y z w) and openness (0 fist ... 1 the grounded palm)."""

    def __init__(self, seconds=5.0, still_mm=40.0, fist_curl=1.1, focal_px=500.0):
        self.seconds, self.still_mm, self.fist_curl, self.focal_px = seconds, still_mm, fist_curl, focal_px
        self.filters = {'offset': OneEuroFilter(1.0, 0.02), 'rotation': OneEuroFilter(1.0, 1.0),
                        'openness': OneEuroFilter(1.0, 1.0)}
        self.start(-1)

    def start(self, request):
        self.id, self.done, self.samples, self.neutral, self.progress = request, -1, [], None, 0.0
        self.output = {'tracked': False, 'ground_id': -1, 'progress': 0.0}

    def update(self, hand, now):
        if self.neutral is None and self.id >= 0:
            self.collect(hand, now)
        if hand is None or self.neutral is None:
            for f in self.filters.values():
                f.reset()
            self.output = {'tracked': False, 'ground_id': self.done, 'progress': self.progress}
            return self.output
        n, f = self.neutral, self.filters
        offset = 1000 * np.array([*(hand.wrist - n.wrist) / n.pixels_per_m,
                                  self.focal_px * (1 / hand.pixels_per_m - 1 / n.pixels_per_m)])
        rotation = (hand.rotation * n.rotation.inv()).as_rotvec()
        openness = (hand.curl - self.fist_curl) / (n.curl - self.fist_curl)
        self.output = {'tracked': True, 'ground_id': self.done, 'progress': 1.0,
                       'offset': f['offset'](offset, now).tolist(),
                       'rotation': Rotation.from_rotvec(f['rotation'](rotation, now)).as_quat().tolist(),
                       'openness': float(np.clip(f['openness'](openness, now), 0.0, 1.0))}
        return self.output

    def collect(self, hand, now):
        """Restart while the hand is missing or wanders more than `still_mm` from the first sample."""
        if hand is None or (self.samples and np.linalg.norm(hand.wrist - self.samples[0][1].wrist) * 1000
                            / hand.pixels_per_m > self.still_mm):
            self.samples = []
        if hand is not None:
            self.samples.append((now, hand))
        self.progress = min((now - self.samples[0][0]) / self.seconds, 1.0) if self.samples else 0.0
        if self.progress == 1.0:
            hands = [h for _, h in self.samples]
            self.neutral = HandSample(hands[-1].image, np.mean([h.wrist for h in hands], axis=0),
                                      float(np.mean([h.pixels_per_m for h in hands])),
                                      Rotation.concatenate([h.rotation for h in hands]).mean(),
                                      float(np.mean([h.curl for h in hands])))
            self.done = self.id
