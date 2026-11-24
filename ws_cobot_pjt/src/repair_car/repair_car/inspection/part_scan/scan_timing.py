"""Separate nominal stroke time from tracking pauses and endpoint settling."""
import time


class StrokeTiming:
    def __init__(self, number, duration, clock=time.monotonic):
        self.number, self.duration, self.clock = number, duration, clock
        self.started = self.previous = clock()
        self.stopped = None
        self.elapsed = self.pause = self.position_pause = self.angle_pause = self.settle = 0.
        self.completed = False
        self.resumes = 0  # interrupted and resumed where it stopped

    def observe(self, now, position_ok, angle_ok, at_end, elapsed):
        dt, self.previous = now - self.previous, now
        self.elapsed = elapsed
        if at_end:
            self.settle += dt
        elif not (position_ok and angle_ok):
            self.pause += dt
            self.position_pause += dt * (not position_ok)
            self.angle_pause += dt * (not angle_ok)

    def finish(self):
        self.stopped = self.clock()

    def export(self):
        return dict(path=self.number, completed=self.completed, planned_s=self.duration,
                    advanced_s=self.elapsed, wall_s=self.stopped - self.started,
                    resumes=self.resumes, tracking_pause_s=self.pause, position_pause_s=self.position_pause,
                    orientation_pause_s=self.angle_pause, endpoint_wait_s=self.settle)
