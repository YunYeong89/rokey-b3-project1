"""Content-addressed cache of validated mesh strokes; JSON only, no executable serialization."""
from hashlib import sha256
import json
from pathlib import Path

import numpy as np

from repair_car.inspection.part_scan.mesh_plan import MeshPlan, MeshPath
from repair_car.inspection.part_scan.trajectory import PoseTrajectory


class MeshPlanCache:
    def __init__(self, directory, surface, axes, reach, scan, travel_z,
                 plan_class=MeshPlan, geometry_arrays=None, scene=None):
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
                         rack_rotation=clearance.rack_rotation.tolist(), rack_box=clearance.rack_box.tolist(),
                         ball_radius=clearance.ball_radius)
        # Runtime-only settings (stroke speed, measured-cloud sampling, PD, resume, forces and contact thresholds) leave
        # the plan as it is. Mesh-height planning values are compared below.
        planning_scan = {key: value for key, value in scan.items()
                         if key not in ('speed_mm_s', 'sample_mm', 'point_decimation', 'default_scan',
                                        'rescan_of')}
        planning_scan['mesh_height'] = {key: value for key, value in scan['mesh_height'].items()
                                       if key not in ('pd', 'resume', 'point_decimation', 'position_stiffness_scale',
                                                      'position_feedback_limit_mm', 'force_n', 'contact_threshold_n',
                                                      'stiffness', 'press_settle_s', 'record_threshold_n',
                                                      'ground_s', 'extra_depth_mm', 'extra_depth_speed_mm_s',
                                                      'force_trim', 'cloud_threshold_n', 'lift_push',
                                                      'telemetry_average', 'rescan', 'press_limit_n',
                                                      'press_confirm_n')}
        recipe = dict(part=surface.part, axes=np.asarray(axes).tolist(), scan=planning_scan,
                      urdf=chain.description, tcp=chain.tcp.tolist(), home=reach.home.tolist(),
                      scene=scene, plan_class=plan_class.__name__, travel_z=travel_z)
        digest.update(json.dumps(recipe, sort_keys=True).encode())
        sources = ('mesh_cache.py', 'mesh_plan.py', 'height_plan.py', 'mesh_surface.py', 'cloud_plan.py', 'cloud_path.py', 'cloud_clearance.py',
                   'trajectory.py', 'path_timing.py', 'tool.py', 'kinematics.py', 'planner.py')
        if 'bonnet_path_mm' in scan:
            sources += ('bonnet_path.py', 'bonnet_plan.py')
        if 'bonnet_transfer' in scan:
            sources += ('bonnet_transfer.py',)
        for name in sources:
            digest.update(Path(__file__).with_name(name).read_bytes())
        digest.update(Path(__file__).parents[2].joinpath('robot_control', 'rotations.py').read_bytes())
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
                               PoseTrajectory.from_cache(path['trajectory'], self.scan['mesh_height'], self.scan['speed_mm_s']),
                               tuple(np.asarray(route) for route in path['transfers']) if path.get('transfers') else None)
                      for path in data['paths']]
        return plan

    def save(self, plan):
        if not plan.paths:
            return
        data = dict(key=self.key, diagnostics=plan.diagnostics, rejected=plan.rejected,
                    paths=[dict(positions=path.positions.tolist(), normals=path.normals.tolist(), axis=path.axis.tolist(),
                                trajectory=path.trajectory.cache_data(),
                                transfers=[route.tolist() for route in path.transfers] if path.transfers else None)
                           for path in plan.paths])
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, separators=(',', ':')))
        temporary.replace(self.path)
