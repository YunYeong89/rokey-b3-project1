"""Two mesh-height raster passes constrained to the hand-drawn bonnet area."""
from functools import partial

import numpy as np

from repair_car.inspection.part_scan.height_plan import MeshHeightPlan
from repair_car.inspection.part_scan.planner import PartPlan, sweep


class BonnetPathPlan(MeshHeightPlan):
    BOUNDARY_MARGIN_MM = .8

    def __init__(self, surface, axes, reach, scan, travel_z, log=None):
        super().__init__(surface, axes, reach, scan, travel_z, log)
        if not scan.get('rescan_of') and {item['along'] for item in self.diagnostics if item['paths']} != {0, 1}:
            self.rejected.insert(0, 'Cannot build both horizontal and vertical paths inside the drawn bonnet area')
            self.paths.clear()

    def scan_area(self, points):
        return (super().scan_area(points) &
                self.surface.bonnet_path.contains(self.surface.roi_coordinates(points), self.BOUNDARY_MARGIN_MM))

    def passes(self, valid, step, along):
        return PartPlan.passes(self, valid, step, along, join=partial(sweep, strict=True))

    def validate_stroke(self, positions, rotations, normals, axis, corners, geometry):
        path = super().validate_stroke(positions, rotations, normals, axis, corners, geometry)
        if path is not None:
            for span in path.trajectory.spans:
                _, points, _ = span.validation_samples(.25)
                outside = ~self.surface.bonnet_path.contains(self.surface.roi_coordinates(points))
                if outside.any():
                    error = ValueError('The interpolated bonnet path leaves the drawn area')
                    error.position = points[np.argmax(outside)]
                    raise error
        return path
