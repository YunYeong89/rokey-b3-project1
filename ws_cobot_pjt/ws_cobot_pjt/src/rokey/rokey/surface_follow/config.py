"""SI parameters of defect_scan's hybrid surface follower (the tool on the model path normal); independent of ROS."""
from dataclasses import dataclass
from math import isfinite


@dataclass
class Settings:
    method: str = 'hybrid'
    hz: float = 40.0
    force: float = 1.5
    speed: float = .005
    radius: float = .0015
    touch: float = .3
    force_limit: float = 8.0
    search_speed: float = .003
    search_depth: float = .008
    reference_limit: float = .002
    acceleration: float = .1
    angular_speed: float = 1.047
    angular_acceleration: float = 2.094
    kp: float = .1
    ki: float = .15
    soft_margin: float = .9
    stiffness_bound: float = 3500.0
    braking: float = .08
    latency_budget: float = .10  # Conservative experiment assumption, not a measured network bound.
    spacing: float = .0005
    clearance: float = .010
    moment_residual: float = .025  # N m, the Kalman-filtered TOOL x / y moment residual, 0.2 s -> abort
    moment_noise: float = .0004    # (N m)^2 sample variance (0.02 N m)
    moment_process: float = .0005  # (N m)^2/s level change: a lasting moment is followed with ~0.14 s
    # Blocked edge: resistance along the feed above max(edge_force, edge_ratio x Fn)
    # or the path lagging the trajectory clock by edge_lag, for 0.15 s -> the force target drops toward
    # edge_min_force so the native servo lifts the ball over; still stalled after hop_after -> HOP: lift hop_height,
    # pass hop_length along the path in the air, search down again (the skipped part is recorded).
    edge_force: float = .4
    edge_ratio: float = .3
    edge_lag: float = .0009
    edge_min_force: float = .3
    hop_after: float = 2.
    hop_height: float = .001
    hop_length: float = .002
    # Contact governor (see governor.py): feed held to follow_speed / |slope| (slope of
    # the tip depth over slope_window of progress, peak held over slope_hold), to the force support and the feed
    # resistance, never below crawl_speed; the force target drops by the resistance.
    follow_speed: float = .0003  # under the ~0.44 mm/s the native force servo can descend at 1.5 N error
    slope_window: float = .0003
    slope_hold: float = .001
    crawl_speed: float = .0004   # 0.6 mm/s x the 0.82 slope at a 1.9 mm groove centre lifted the ball off
    pointcloud_format: str = 'plt'
    finish_tolerance: float = .0004

    def validate(self):
        if self.method != 'hybrid':
            raise ValueError('method must be hybrid')
        if self.pointcloud_format not in ('plt', 'ply'):
            raise ValueError('pointcloud_format must be plt or ply')
        if not (all(isfinite(v) for k, v in vars(self).items() if k not in ('method', 'pointcloud_format'))
                and .3 <= self.force <= 5 and 0 < self.speed <= .02 and .0001 <= self.radius <= .01
                and .1 <= self.touch < self.force
                and 20 <= self.hz <= 60 and self.latency_budget >= .04
                and 0 < self.finish_tolerance <= .001
                and self.stiffness_bound > 0 and self.braking > 0 and self.soft_margin > .2):
            raise ValueError('Invalid surface controller force / speed / probe / dynamics parameters')

