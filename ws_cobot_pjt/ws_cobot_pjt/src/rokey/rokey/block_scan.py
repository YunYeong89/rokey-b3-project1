"""The defect_scan raster and hybrid execution, shared with adaptive ROI passes."""
from dataclasses import dataclass

import numpy as np

from .configs import load
from .surface_follow.config import Settings
from .surface_follow.geometry import SurfacePath


@dataclass(frozen=True)
class BlockSurface:
    size: float
    top: float

    @classmethod
    def from_config(cls, config):
        test = config['contact_test']
        return cls(test['block_size_mm'], test['surface_z_mm'])


class Raster:
    """Local-mm serpentine: alternating BASE +/-X rows connected by a +Y pitch step."""
    def __init__(self, bounds_mm, pitch_mm, rows=None):
        self.bounds = np.asarray(bounds_mm, float)
        x0, y0, x1, y1 = self.bounds
        if pitch_mm <= 0 or x1 <= x0 or y1 <= y0:
            raise ValueError('Raster requires positive pitch and area')
        self.pitch, self.x = pitch_mm, np.array([x0, x1])
        count = int(np.floor((y1 - y0) / self.pitch + 1e-9)) + 1
        self.y = y0 + self.pitch * np.arange(count if rows is None else min(rows, count))
        if not len(self.y):
            raise ValueError('rows must be positive')
        self.ys = self.y
        self.points = np.asarray([[x, y] for row, y in enumerate(self.y)
                                  for x in (self.x if row % 2 == 0 else self.x[::-1])])
        self.length = float(np.linalg.norm(np.diff(self.points, axis=0), axis=1).sum())
        self.area = float((x1 - x0) * (y1 - y0))

    def surface(self, centre_mm, top_mm, radius_mm):
        centres = np.c_[self.points + centre_mm, np.full(len(self.points), top_mm + radius_mm)] / 1000
        return SurfacePath(centres, np.tile([0., 0., 1.], (len(centres), 1)))


class RasterPath(Raster):
    """defect_scan's BASE sphere centres; bounds optionally restrict the same scan to an ROI."""
    def __init__(self, centre_mm, block, scan, tip, rows=None, bounds_mm=None, start=0):
        if bounds_mm is None:  # start: skip that many rows from the bottom (e.g. straight to the dent)
            half = block.size / 2 - scan['margin_mm']
            bounds_mm = [-half, -half + start * scan['pitch_mm'], half, half]
        super().__init__(bounds_mm, scan['pitch_mm'], rows)
        path = self.surface(centre_mm, block.top, tip)
        self.centres, self.normals = path.centres, path.normals


def home_centre(transport, samples=20):
    """Freeze the native HOME TCP XY once; do not mix it with the DSR nominal pose."""
    transport.wait_ready()
    transport.wait_still()
    seen, points = set(), []
    while len(points) < samples:
        transport.spin(.02)
        state = transport.state()
        if state.sequence not in seen:
            seen.add(state.sequence)
            points.append(state.position[:2] * 1000)
    return np.mean(points, axis=0)


class DefectScan:
    @staticmethod
    def settings_for(config, scan, **overrides):
        probe = config['probe']
        values = dict(load('surface_following'), method='hybrid',
                      radius=probe['tip_radius_mm'] / 1000, touch=config['scan']['touch_threshold'],
                      force=scan['force_n'], speed=scan['speed_mm_s'] / 1000)
        settings = Settings(**dict(values, **overrides))
        settings.validate()
        return settings

    def __init__(self, program, raster, scan, transport, settings=None, recording_factory=None):
        from .surface_follow.runtime import SurfaceRun
        self.settings = settings if settings is not None else self.settings_for(program.config, scan)
        path = SurfacePath(raster.centres, raster.normals)
        recording = recording_factory(self.settings, path) if recording_factory else None
        self.run = SurfaceRun(self.settings, path, program, transport, recording=recording)

    def execute(self):
        return self.run.execute(3 * self.run.path.length / self.settings.speed + 120)
