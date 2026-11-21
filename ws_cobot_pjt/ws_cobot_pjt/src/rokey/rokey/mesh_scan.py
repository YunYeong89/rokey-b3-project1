"""Plan mesh-based scans and run them through DSR or the simulation Cartesian stream."""
from time import perf_counter

import numpy as np

from .configs import PACKAGE, output_dir
from .part_scan.bonnet_plan import BonnetPathPlan
from .part_scan.cloud import PartCloud
from .part_scan.height_plan import MeshHeightPlan, door_panel, panel_cells, press_label
from .part_scan.mesh_cache import MeshPlanCache
from .part_scan.mesh_plan import MeshClearance, MeshNormalPlan
from .part_scan.mesh_report import MeshReport
from .part_scan.mesh_surface import MeshGrid, MeshSurface
from .part_scan.run import CloudChannel
from .part_scan.tool import Reach, robot_chain
from .real_vehicle import RealVehicle
from .surface_follow.pointcloud_file import PointCloudFile
from .scan_scene import ScanScene


def run_mesh_scan(program, options, scan, vehicle, pose, probe_plan, truth, stamp):
    started = perf_counter()
    part = 'box' if options.box else options.part
    height_mode = options.controller == 'mesh-height'
    bonnet_path = getattr(options, 'bonnet_path', None)
    if height_mode:
        scan = MeshHeightPlan.configure(scan)
    settings = scan['mesh_normal']
    if not height_mode:
        scan['default_scan']['survey']['force_n'] = settings['force_n']
        scan['default_scan']['contact_threshold_n'] = settings['record_threshold_n']
        scan['default_scan']['real_contact_offset_n'] = settings['contact_threshold_n']
    if part == 'bonnet':  # as the default plan: the controller refuses lifted entries/exits near the outer reach
        scan['travel_margin_mm'] = scan['default_scan']['bonnet_travel_margin_mm']
    if options.roi_base:
        scan['roi_base_mm'] = options.roi_base
    if options.roi_local:
        scan['roi_local_mm'] = options.roi_local
    if bonnet_path is not None:
        scan['bonnet_path_mm'] = bonnet_path.coordinates()
    if options.roi_base or options.roi_local or bonnet_path is not None:
        scan['min_path_mm'] = 3 * scan['measurable']['min_segment_mm']
    # The ball centre is the motion reference (local path timing, pivot about the ball), the tip pressed
    # contact_preload_mm into the surface, cruising at cruise_speed_mm_s unless --speed.
    settings['ball_radius_mm'] = program.config['probe']['tip_radius_mm']
    if options.speed is None and not options.box:
        scan['speed_mm_s'] = settings['cruise_speed_mm_s']
    if scan['speed_mm_s'] > settings['stream_speed_mm_s']:
        raise ValueError('--speed exceeds real_scan.mesh_normal.stream_speed_mm_s')
    if height_mode:
        # One fixed axis: the bonnet from above, a door from BASE -x (the vehicle turned so that it faces the robot),
        # the TCP axis tilted door_tool_tilt_deg toward straight down there; the force along the axis either way.
        door_virtual_force, door_force = settings.pop('door_virtual_force_n'), settings.pop('door_force_n')
        bonnet_force = settings.pop('bonnet_force_n', settings['force_n'])
        height_axis = np.asarray(settings['door_axis'], float) if part == 'door' else np.array([0., 0., 1.])
        settings['tool_tilt_deg'], settings['tool_spin_deg'] = 0., 0.
        if part == 'door':
            if program.mode == 'virtual':
                settings['tool_tilt_deg'] = settings['door_virtual_tool_tilt_deg']
                scan['default_scan']['survey']['force_n'] = door_virtual_force
            else:
                settings['tool_tilt_deg'] = settings['door_tool_tilt_deg']
                settings['tool_spin_deg'] = settings['door_tool_spin_deg']
                scan['default_scan']['survey']['force_n'] = door_force
            facing = float(pose[:3, 1] @ height_axis)  # the vehicle's left along the door axis: +1 left, -1 right
            side = 'left' if facing > .9 else 'right' if facing < -.9 else None
            if side is None or options.side not in (None, side):
                raise RuntimeError(f'mesh-height 도어는 로봇을 향한 도어 하나만 측정합니다 (BASE {press_label(height_axis)} '
                                   f'누름): 차량 yaw를 왼쪽 도어 90°, 오른쪽 도어 -90°로 놓으세요 '
                                   f'(real_scene.yaml vehicle.pose_base_mm_deg, 시뮬레이션 --car demo_left/demo_right). '
                                   f'현재 차량 왼쪽 방향 {pose[:3, 1].round(2).tolist()}'
                                   + (f', 요청 --side {options.side}' if options.side else ''))
            options.side = side
        elif part == 'bonnet' and program.mode == 'real' and not scan.get('rescan_of'):
            settings['force_n'] = scan['default_scan']['survey']['force_n'] = bonnet_force
        tilt = settings['tool_tilt_deg']
    program.log(f'{part}{f"_{options.side}" if options.side else ""}: mesh height path, fixed TCP'
                f'{f" (tilted {tilt:g} deg)" if tilt else ""}, BASE {press_label(height_axis)} force, PD OFF'
                if height_mode else
                f'{part}: mesh TCP position/orientation control, pressed along the TCP axis')
    channel = CloudChannel()
    if options.box:
        # Both inputs come from the healthy coupon: no defect cloud, height or normal enters planning.
        cloud = vehicle.reference(pose, scan['step_mm'])
        regions = vehicle.planning_regions(scan, options.region, options.roi)
        surface_vehicle = vehicle.planning_surface(regions=regions)
        selection = '사용자 ROI' if options.roi else options.region or 'crack + dent'
        program.log(f'시편 검출 영역 {selection}: {len(regions)}개 ROI, '
                    '계획 높이는 정상 평면')
    else:
        reference_path = PACKAGE / 'pointcloud' / f'real_{part}_reference.ply'
        if vehicle is None:
            vehicle = RealVehicle()
        cloud = PartCloud(vehicle, part, pose, scan['spacing_mm'], reference_path)
        surface_vehicle = vehicle
    points = cloud.points
    if options.side:
        centre = (points.min(0) + points.max(0)) / 2
        points = points[((points - centre) @ pose[:3, 1] > 0) == (options.side == 'left')]
    nominal = points
    if bonnet_path is not None:
        nominal = points[bonnet_path.contains((points - pose[:3, 3]) @ pose[:3, :2])]
        program.log(f'보닛 직접 지정 {len(bonnet_path.regions)}개 영역: 가로·세로 계획, 도구 형상 충돌 검사 비활성')
    publish_cloud = program.mode != 'real' or not options.plan_only
    if publish_cloud:
        channel.begin(nominal / 1000, truth.get('inspection_scene'))
    program.log(f'Full reference cloud: {len(points)} points; planning source=mesh')
    axes = np.array([[0., 0., 1.]]) if options.box else ScanScene.axes(scan['axes'][part], pose)
    if options.side:
        left = pose[:3, 1]
        axes = axes[(axes @ left > 0) == (options.side == 'left')]
    if height_mode:
        axes = height_axis[None]
    chain = robot_chain(program.robot, probe_plan['tcp_mm_deg'], scan['ik'])
    home = np.radians(program.config['robot']['home_joints_deg'])
    surface = MeshSurface(surface_vehicle, part, pose, settings['crease_deg'], side=options.side)
    if options.box:
        surface.planning_reference = 'healthy_nominal_plane'
        surface.roi_local_mm = options.roi
        surface.regions_local_mm = regions
    if options.roi_local:
        surface.roi_local_mm = options.roi_local
    if bonnet_path is not None:
        surface.bonnet_path = bonnet_path
    clearance = MeshClearance(surface, scan, truth['table_z_mm'] + scan['table_margin_mm'],
                              probe_plan['tip_pose_base_mm_deg'], probe_plan['rack_box_mm'])
    clearance.ball_radius = program.config['probe']['tip_radius_mm']
    clearance.ball_penetration = settings['ball_penetration_mm']
    # a real door: the tool's shape against the vehicle and rack unchecked (door_tool_clearance false); the ball's
    # support, the table, the robot base, IK and the arm links still are
    clearance.tool_free = bonnet_path is not None or (
        height_mode and part == 'door' and program.mode == 'real' and not settings['door_tool_clearance'])
    reach = Reach(chain, home, clearance, scan['ik']['arm_radii_mm'])
    plan_class = BonnetPathPlan if bonnet_path is not None else MeshHeightPlan if height_mode else MeshNormalPlan
    cache = MeshPlanCache(PACKAGE / '.cache' / 'mesh_plans', surface, axes, reach, scan, program.safe_z(),
                          plan_class=plan_class)
    plan = None if options.replan else cache.load()
    cached = plan is not None
    if plan is None:
        plan = plan_class(surface, axes, reach, scan, program.safe_z(), program.log)
        cache.save(plan)
    plan.cache_hit, plan.planning_seconds = cached, perf_counter() - started
    plan.reference_points = len(points)
    program.log(f'mesh plan {"cache hit" if cached else "checked and cached"}: {plan.planning_seconds:.2f} s')
    program.log(plan.summary())
    if height_mode and part == 'door':  # RViz and the report: the door's planning area only (its flat panel under the window)
        grid = plan.grids[0][1] if plan.grids else MeshGrid(surface, axes[0], scan['step_mm'], scan['edge_margin_mm'].get(
            part, scan['edge_margin_mm']['default']))
        i, j = panel_cells(grid, points)
        inside = (i >= 0) & (j >= 0) & (i < grid.faces.shape[0]) & (j < grid.faces.shape[1])
        # the panel cells' outer face only: the door skin's back face is ~3 mm behind it in the same cell
        inside[inside] = (door_panel(surface, grid, settings['door_panel'])[i[inside], j[inside]] &
                          (points[inside] @ grid.axis >= grid.height[i[inside], j[inside]] - 1.))
        nominal = points[inside]
        if publish_cloud:
            channel.write('reference', nominal / 1000)
        program.log(f'도어 계획 영역(창문 아래 평면 패널) {int(inside.sum())}/{len(points)}점')
    label = 'mesh_height' if height_mode else 'mesh'
    if bonnet_path is not None:
        label += '_drawn'
    elif options.roi_base or options.roi_local:
        label += '_roi'
    if options.inspection_region:
        label += f'_r{options.inspection_region}'
    prefix = 'performance' if options.box else f'real_{part}'
    name = prefix + (f'_{options.side}' if options.side else '') + f'_{label}_{stamp}'
    report = MeshReport(output_dir('images'), name)
    roi_reference = points if program.mode == 'real' and part == 'bonnet' and height_mode else None
    directory = report.plan(plan, roi_reference=roi_reference, roi_cloud_settings=program.config['bonnet_roi_cloud'],
                            nominal=nominal, web_report=options.web_report, scene=truth.get('inspection_scene'))
    program.log(f'mesh plan and TCP axes -> {directory}')
    program.topics.reports(directory / 'report.png', directory / 'plan.json')
    if not plan.paths:
        program.log(f'경로 계획 진단: 단계별 통계={plan.diagnostics}, '
                    f'자세 제외 원인={reach.failures}, 경로 제외 원인={plan.rejected[:3]}')
        raise RuntimeError('선택 영역에 접근 가능한 스캔 경로가 없습니다.')
    if options.plan_only:
        return report
    motion = program.config['motion']
    if program.mode == 'real':
        from .part_scan.height_dsr import MeshHeightDsr
        program.log(f'실제 로봇 mesh-height: DSR movesx {scan["speed_mm_s"]} mm/s, '
                    f'접촉 판정 +{scan["default_scan"]["real_contact_offset_n"]} N, '
                    f'측정점 기록 {scan["default_scan"]["contact_threshold_n"]} N 이상, '
                    f'BASE {press_label(height_axis)} 누름 {scan["default_scan"]["survey"]["force_n"]} N, '
                    f'누름 축 강성 {settings["stiffness"][2] / 1000:g} N/mm, 접근 후 정지 {settings["press_settle_s"]:g} s')
        runner = MeshHeightDsr(program, plan, channel, report)
    else:
        program.log(f'검출 {scan["speed_mm_s"]} mm/s (custom); 일반 이동 {motion["linear_speed"]} mm/s, '
                    f'접근 {motion["approach_speed"]} mm/s, 근접 진입 {settings["approach_speed_mm_s"]} mm/s (DSR)')
        if settings['ball_radius_mm'] and not height_mode:
            program.log(f'구 중심 속도 기준; 그리퍼 구 중심 회전, 고정 눌림 {settings["contact_preload_mm"]} mm; '
                        '가속도 제한 구간만 국소 감속')
        force_axis = f'BASE {press_label(height_axis)}' if height_mode and part == 'door' else 'TCP 축'
        program.log(f'검출 위치 복원 계수 {settings["position_stiffness_scale"]}배, '
                    f'추가 위치 보정 상한 {settings["position_feedback_limit_mm"]} mm; '
                    f'{force_axis} 힘제어 {scan["default_scan"]["survey"]["force_n"]} N, '
                    f'측정점 기록 {scan["default_scan"]["contact_threshold_n"]} N 이상')
        pd = settings['pd']
        program.log(f'검출 PD {"ON" if pd["enabled"] else "OFF"}: '
                    f'위치 Kp={pd["position_kp"]}, Kd={pd["position_kd_s"]} s; '
                    f'자세 Kp={pd["orientation_kp"]}, Kd={pd["orientation_kd_s"]} s; I 없음')
        from .part_scan.mesh_run import MeshNormalScan
        runner = MeshNormalScan(program, plan, channel, report, height_mode=height_mode)
    try:
        runner.run()
    finally:
        program.log(f'Combined subplot report -> {report.directory / "report.png"}')
        if report.rois is not None:
            roi_file = report.directory / 'roi.json'
            program.topics.regions(report.rois.regions, report.rois.limit)
            program.topics.record(roi_file=str(roi_file))
            program.topics.reports(roi_file)
        if runner.points:
            path = PointCloudFile.save(output_dir('pointcloud') / f'{name}_measured.ply', runner.points)
            program.log(f'Measured contact points (normal force above the default contact threshold) -> {path}')
            program.topics.record(measured_cloud=str(path), points=len(runner.points))
    return report
