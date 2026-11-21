"""Feed and force-target shaping in contact on a model trajectory (hybrid). SI units."""
from collections import deque

import numpy as np


class ContactGovernor:
    """Grooves, steps and slopes in one pass, without modes. The feed is held where
      - the native normal servo can follow the surface: |slope| x feed <= follow_speed, the slope of the tip depth
        over the last slope_window of progress, its peak held over slope_hold (a groove is crossed slowly to its far
        edge, so the ball settles on the edges: full depth),
      - the force falls short of the target (support lost under the ball),
      - the feed meets resistance (a far edge);
    and the force target drops by that resistance so the native servo lifts the ball over the edge. At 4 mm/s a
    1.2 mm groove gave a far-edge push of 2.3 N in 0.25 s, before any debounced edge handling, and UNLOAD / RECOVER
    re-entered the same groove; a 9 deg dent slope needs 0.63 mm/s of normal travel, ~2 N of force error."""

    def __init__(self, settings):
        self.s = settings
        self.history = deque()
        self.slope = self.hold = self.last = self.scale = 0.

    def observe(self, progress, depth):
        s, history = self.s, self.history
        if history and progress <= history[-1][0] + 2e-5:
            return
        history.append((progress, depth))
        while len(history) > 2 and progress - history[1][0] >= s.slope_window:
            history.popleft()
        start, level = history[0]
        if progress - start >= s.slope_window / 2:
            self.slope = (depth - level) / (progress - start)
        self.hold = max(abs(self.slope), self.hold * np.exp(-(progress - self.last) / s.slope_hold))
        self.last = progress

    def feed(self, force, goal, resistance):
        """Feed scale in [crawl_speed / speed, 1]."""
        s = self.s
        low = s.crawl_speed / s.speed
        slope = s.follow_speed / max(self.hold * s.speed, 1e-9)
        support = (force / max(goal, 1e-3) - .6) / .3
        resist = 1 - (resistance - s.edge_force / 4) / s.edge_force
        self.scale = float(np.clip(min(slope, support, resist), low, 1))
        return self.scale

    def target(self, resistance):
        s = self.s
        return max(s.edge_min_force, s.force - max(0., resistance - s.edge_force / 4))
