"""Two mesh-height raster passes constrained to the hand-drawn bonnet area."""
from functools import partial

import numpy as np

from .height_plan import MeshHeightPlan
from .planner import PartPlan, sweep


class BonnetPathPlan(MeshHeightPlan):
    BOUNDARY_MARGIN_MM = .8

    def __init__(self, surface, axes, reach, scan, travel_z, log=None):
        super().__init__(surface, axes, reach, scan, travel_z, log)
        if {item['along'] for item in self.diagnostics if item['paths']} != {0, 1}:
            self.rejected.insert(0, '그린 보닛 범위 안에서 가로·세로 경로를 모두 만들 수 없습니다')
            self.paths.clear()

    def scan_area(self, points):
        return self.surface.bonnet_path.contains(self.surface.roi_coordinates(points), self.BOUNDARY_MARGIN_MM)

    def passes(self, valid, step, along):
        return PartPlan.passes(self, valid, step, along, join=partial(sweep, strict=True))

    def validate_stroke(self, positions, rotations, normals, axis, corners, geometry):
        path = super().validate_stroke(positions, rotations, normals, axis, corners, geometry)
        if path is not None:
            for span in path.trajectory.spans:
                _, points, _ = span.validation_samples(.25)
                outside = ~self.surface.bonnet_path.contains(self.surface.roi_coordinates(points))
                if outside.any():
                    error = ValueError('보닛 보간 경로가 그린 범위를 벗어납니다')
                    error.position = points[np.argmax(outside)]
                    raise error
        return path
