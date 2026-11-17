"""Normal-following scan: point-cloud planning by default; force is telemetry only."""
from copy import deepcopy
import numpy as np
from time import perf_counter

from .configs import PACKAGE
from .part_scan.mesh_report import MeshReport
from .part_scan.mesh_cache import MeshPlanCache
from .part_scan.cloud import PartCloud
from .part_scan.cloud_plan import CloudSurface, CloudReach, CloudNormalPlan
from .part_scan.run import CloudChannel
from .part_scan.tool import robot_chain
from .surface_follow.pointcloud_file import PointCloudFile


def run_mesh_scan(program, options, scan, vehicle, pose, probe_plan, truth, stamp):
    started = perf_counter()
    settings = scan['mesh_normal']
    source = 'pointcloud' if options.controller == 'cloud-normal' else settings['planning_source']
    if source == 'pointcloud':
        scan = deepcopy(scan)
        settings = scan['mesh_normal']
        cloud_settings = settings['pointcloud']
        for key in ('checkpoint_max_mm', 'normal_step_deg', 'normal_interp_error_deg', 'position_interp_error_mm',
                    'position_interpolator'):
            settings[key] = cloud_settings[key]
        settings['validation_step_mm'] = cloud_settings['ik_step_mm']
        settings['validation_angle_deg'] = cloud_settings['ik_angle_deg']
        scan['measurable']['joint_jump_deg'] = cloud_settings['joint_jump_deg']
    if scan['speed_mm_s'] > settings['stream_speed_mm_s']:
        raise ValueError('--speed exceeds real_scan.mesh_normal.stream_speed_mm_s')
    program.log(f'{options.part}: {source} TCP position/orientation control; force measurement only')
    channel = CloudChannel()
    reference_path = PACKAGE / 'pointcloud' / f'real_{options.part}_reference.ply'
    if vehicle is None and (source == 'mesh' or not reference_path.exists()):
        from .real_vehicle import RealVehicle
        vehicle = RealVehicle()
    cloud = PartCloud(vehicle, options.part, pose, scan['spacing_mm'],
                       reference_path)
    points = cloud.points
    if options.side:
        centre = (points.min(0) + points.max(0)) / 2
        points = points[((points - centre) @ pose[:3, 1] > 0) == (options.side == 'left')]
    channel.write('reference', points / 1000)
    channel.write('measured', [])
    program.log(f'Full reference cloud: {len(points)} points -> RViz; planning source={source}')
    axes = np.asarray(scan['axes'][options.part], float)
    if options.side:
        left = pose[:3, 1]
        axes = axes[(axes @ left > 0) == (options.side == 'left')]
    chain = robot_chain(program.robot, probe_plan['tcp_mm_deg'], scan['ik'])
    home = np.radians(program.config['robot']['home_joints_deg'])
    if source == 'pointcloud':
        surface = CloudSurface(points, options.part, program.config['probe']['tip_radius_mm'])
        reach = CloudReach(chain, home)
        plan_class = CloudNormalPlan
        scene = dict(table=truth['table_z_mm'], rack=probe_plan['tip_pose_base_mm_deg'],
                     ball_radius=program.config['probe']['tip_radius_mm'])
        cache = MeshPlanCache(PACKAGE / '.cache' / 'cloud_plans', surface, axes, reach, scan,
                              program.safe_z(), plan_class=plan_class, geometry_arrays=(points,), scene=scene)
    elif source == 'mesh':
        from .part_scan.mesh_surface import MeshSurface
        from .part_scan.mesh_plan import MeshClearance, MeshNormalPlan
        from .part_scan.tool import Reach
        surface = MeshSurface(vehicle, options.part, pose, settings['crease_deg'])
        clearance = MeshClearance(surface, scan, truth['table_z_mm'] + scan['table_margin_mm'],
                                  probe_plan['tip_pose_base_mm_deg'])
        clearance.ball_radius = program.config['probe']['tip_radius_mm']
        clearance.ball_penetration = settings['ball_penetration_mm']
        reach = Reach(chain, home, clearance, scan['ik']['arm_radii_mm'])
        plan_class = MeshNormalPlan
        cache = MeshPlanCache(PACKAGE / '.cache' / 'mesh_plans', surface, axes, reach, scan, program.safe_z())
    else:
        raise ValueError(f'Unknown planning source: {source}')
    plan = None if options.replan else cache.load()
    cached = plan is not None
    if plan is None:
        plan = plan_class(surface, axes, reach, scan, program.safe_z(), program.log)
        cache.save(plan)
    plan.cache_hit, plan.planning_seconds = cached, perf_counter() - started
    plan.reference_points = len(points)
    program.log(f'{source} plan {"cache hit" if cached else "checked and cached"}: {plan.planning_seconds:.2f} s')
    program.log(plan.summary())
    name = f'real_{options.part}' + (f'_{options.side}' if options.side else '') + f'_{source}_{stamp}'
    report = MeshReport(PACKAGE / 'images', name)
    program.log(f'{source} plan and TCP axes -> {report.plan(plan)}')
    if not plan.paths:
        raise RuntimeError(f'실행 가능한 {source} 경로가 없습니다. 단계별 통계={plan.diagnostics}, '
                           f'자세 제외 원인={reach.failures}, 경로 제외 원인={plan.rejected[:3]}. '
                           f'상세 결과: {report.directory / "plan.json"}')
    if options.plan_only:
        return
    motion = program.config['motion']
    program.log(f'검출 {scan["speed_mm_s"]} mm/s (custom); 일반 이동 {motion["linear_speed"]} mm/s, '
                f'접근 {motion["approach_speed"]} mm/s, 근접 진입 {settings["approach_speed_mm_s"]} mm/s (DSR)')
    program.log(f'검출 위치 복원 계수 {settings["position_stiffness_scale"]}배, '
                f'추가 위치 보정 상한 {settings["position_feedback_limit_mm"]} mm; 힘 피드백 없음')
    from .part_scan.mesh_run import MeshNormalScan
    runner = MeshNormalScan(program, plan, channel, report)
    try:
        runner.run()
    finally:
        if runner.points:
            path = PointCloudFile.save(PACKAGE / 'pointcloud' / f'{name}_measured.ply', runner.points)
            program.log(f'Measured geometric samples (no force/contact filtering) -> {path}')
