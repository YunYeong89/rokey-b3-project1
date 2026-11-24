"""Plan the real bonnet's free-space route without trimming its contact strokes."""
from copy import copy

import numpy as np

from repair_car.inspection.part_scan.tool import Reach
from repair_car.inspection.part_scan.trajectory import PoseTrajectory


class BonnetTransfer:
    HEIGHT_STEP_MM = 5.

    def __init__(self, reach, scan, travel_z):
        self.reach, self.scan = reach, scan
        self.home = reach.chain.forward(reach.home)[0][:3, 3] * 1000
        self.park = np.r_[self.home[:2], travel_z]
        clearance = copy(reach.clearance)
        clearance.tool_free, clearance.tool_checks = False, {}
        self.travel_reach = Reach(reach.chain, reach.home, clearance, reach.radii)

    def plan(self, tip, rotation):
        """Return waypoints from the lifted contact endpoint back to the shared parking pose."""
        settings = self.scan['bonnet_transfer']
        low = tip[2] + self.scan['gap_mm'] + settings['clearance_mm']
        high = max(self.park[2], tip[2] + self.scan['standoff_mm'], low)
        axis = np.array([0., 0., 1.])
        for height in np.r_[np.arange(high, low, -self.HEIGHT_STEP_MM), low]:
            out, lane = np.r_[tip[:2], height], np.r_[self.park[:2], height]
            if not self.reach.within_outer_reach(np.array([out, lane, self.park]), axis, settings['margin_mm']).all():
                continue
            waypoints = np.array([out, lane] if abs(height - self.park[2]) < 1e-8 else [out, lane, self.park])
            seed = self.reach.pose(tip, rotation, self.reach.home)
            try:
                seed = self.validate(tip, out, rotation, seed, self.reach)
                for start, end in zip(waypoints, waypoints[1:]):
                    seed = self.validate(start, end, rotation, seed, self.travel_reach)
                seed = self.validate(self.park, self.home, rotation, seed, self.travel_reach)
            except ValueError:
                continue
            if np.max(np.abs(seed - self.reach.home)) <= np.radians(self.scan['measurable']['joint_jump_deg']):
                return waypoints
        raise RuntimeError(f'Could not find an entry/exit route that keeps the bonnet scan paths: '
                           f'TCP {tip.round(2).tolist()} mm')

    def validate(self, start, end, rotation, seed, reach):
        if np.linalg.norm(end - start) < 1e-8:
            return seed
        motion = self.scan['mesh_height']
        leg = PoseTrajectory([start, end], [rotation, rotation], motion)
        return leg.validate(reach, seed, motion['approach_speed_mm_s'], self.scan['measurable']['joint_jump_deg'])
