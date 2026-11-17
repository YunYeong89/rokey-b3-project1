"""Content-addressed cache of validated mesh strokes; JSON only, no executable serialization."""
from hashlib import sha256
import json
from pathlib import Path

import numpy as np

from .mesh_plan import MeshNormalPlan, MeshPath
from .trajectory import PoseTrajectory


class MeshPlanCache:
    def __init__(self, directory, surface, axes, reach, scan, travel_z,
                 plan_class=MeshNormalPlan, geometry_arrays=None, scene=None):
        self.surface, self.reach, self.scan, self.travel_z = surface, reach, scan, travel_z
        self.plan_class = plan_class
        digest = sha256()
        arrays = (surface.triangles, surface.target) if geometry_arrays is None else geometry_arrays
        for values in arrays:
            digest.update(np.ascontiguousarray(values).tobytes())
        chain = reach.chain
        if scene is None:
            clearance = reach.clearance
            scene = dict(table=clearance.table, rack=clearance.rack.tolist(),
                         rack_rotation=clearance.rack_rotation.tolist(), ball_radius=clearance.ball_radius)
        recipe = dict(part=surface.part, axes=np.asarray(axes).tolist(),
                      scan={key: value for key, value in scan.items() if key != 'speed_mm_s'},
                      urdf=chain.description, tcp=chain.tcp.tolist(), home=reach.home.tolist(),
                      scene=scene, plan_class=plan_class.__name__, travel_z=travel_z)
        digest.update(json.dumps(recipe, sort_keys=True).encode())
        sources = ('mesh_cache.py', 'mesh_plan.py', 'mesh_surface.py', 'cloud_plan.py',
                   'trajectory.py', 'tool.py', 'kinematics.py', 'planner.py')
        for name in sources:
            digest.update(Path(__file__).with_name(name).read_bytes())
        digest.update(Path(__file__).parents[1].joinpath('rotations.py').read_bytes())
        self.key = digest.hexdigest()
        self.path = directory / f'{self.key}.json'

    def load(self):
        if not self.path.exists():
            return None
        data = json.loads(self.path.read_text())
        if data['key'] != self.key:
            raise ValueError('Mesh cache signature mismatch')
        plan = self.plan_class.__new__(self.plan_class)
        plan.surface, plan.reach, plan.scan, plan.travel_z = self.surface, self.reach, self.scan, self.travel_z
        plan.maps, plan.grids = [], []
        plan.rejected, plan.diagnostics = data['rejected'], data['diagnostics']
        plan.paths = [MeshPath(np.asarray(path['positions']), np.asarray(path['normals']), np.asarray(path['axis']),
                               PoseTrajectory.from_cache(path['trajectory'], self.scan['mesh_normal'], self.scan['speed_mm_s']))
                      for path in data['paths']]
        return plan

    def save(self, plan):
        if not plan.paths:
            return
        data = dict(key=self.key, diagnostics=plan.diagnostics, rejected=plan.rejected,
                    paths=[dict(positions=path.positions.tolist(), normals=path.normals.tolist(), axis=path.axis.tolist(),
                                trajectory=path.trajectory.cache_data()) for path in plan.paths])
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, separators=(',', ':')))
        temporary.replace(self.path)
