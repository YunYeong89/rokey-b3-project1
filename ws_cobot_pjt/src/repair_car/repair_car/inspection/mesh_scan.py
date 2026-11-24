"""Plan mesh-based scans and run them through DSR or the simulation Cartesian stream."""
from datetime import datetime
from time import perf_counter

import numpy as np

from repair_car.config import PACKAGE
from repair_car.inspection.part_scan.archive import InspectionArchive
from repair_car.inspection.part_scan.bonnet_plan import BonnetPathPlan
from repair_car.pointcloud.cloud import PartCloud
from repair_car.inspection.part_scan.height_plan import MeshHeightPlan, door_panel, panel_cells, press_label
from repair_car.inspection.part_scan.mesh_cache import MeshPlanCache
from repair_car.inspection.part_scan.mesh_plan import MeshClearance
from repair_car.report.mesh_report import MeshReport
from repair_car.inspection.part_scan.mesh_surface import MeshGrid, MeshSurface
from repair_car.inspection.part_scan.run import CloudChannel
from repair_car.inspection.part_scan.tool import Reach, robot_chain
from repair_car.geometry.real_vehicle import RealVehicle
from repair_car.pointcloud.pointcloud_file import PointCloudFile
from repair_car.inspection.scan_scene import ScanScene


def run_mesh_scan(program, options, scan, vehicle, pose, probe_plan, truth, stamp):
    started = perf_counter()
    part = 'box' if options.box else options.part
    bonnet_path = getattr(options, 'bonnet_path', None)
    scan = MeshHeightPlan.configure(scan)
    if program.mode == 'real' and part == 'bonnet':
        scan['bonnet_transfer'] = dict(margin_mm=scan['default_scan']['bonnet_real_travel_margin_mm'],
                                       clearance_mm=program.config['scan']['clearance_mm'])
    settings = scan['mesh_height']
    if program.mode == 'virtual':
        settings.update(settings.get('virtual_tracking', {}))
    if options.roi_base:
        scan['roi_base_mm'] = options.roi_base
        if part == 'door':
            scan['roi_base_axes'] = (1, 2)
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
        raise ValueError('--speed exceeds real_scan.mesh_height.stream_speed_mm_s')
    door_force = settings.pop('door_force_n')
    bonnet_force = settings.pop('bonnet_force_n', settings['force_n'])
    height_axis = np.asarray(settings['door_axis'], float) if part == 'door' else np.array([0., 0., 1.])
    settings['tool_tilt_deg'], settings['tool_spin_deg'] = 0., 0.
    if part == 'door':
        settings['tool_tilt_deg'] = settings['door_tool_tilt_deg']
        settings['tool_spin_deg'] = settings['door_tool_spin_deg']
        settings['force_n'] = scan['default_scan']['survey']['force_n'] = door_force
        facing = float(pose[:3, 1] @ height_axis)  # the vehicle's left along the door axis: +1 left, -1 right
        side = 'left' if facing > .9 else 'right' if facing < -.9 else None
        if side is None or options.side not in (None, side):
            raise RuntimeError(f'mesh-height measures only the one door facing the robot (BASE {press_label(height_axis)} '
                               f'press): set the vehicle yaw to 90° for the left door, -90° for the right door '
                               f'(real_scene.yaml vehicle.pose_base_mm_deg, simulation --car demo_left/demo_right). '
                               f'Current vehicle left direction {pose[:3, 1].round(2).tolist()}'
                               + (f', requested --side {options.side}' if options.side else ''))
        options.side = side
    elif part == 'bonnet' and not scan.get('rescan_of'):
        settings['force_n'] = scan['default_scan']['survey']['force_n'] = bonnet_force
    tilt = settings['tool_tilt_deg']
    program.log(f'{part}{f"_{options.side}" if options.side else ""}: mesh height path, fixed TCP'
                f'{f" (tilted {tilt:g} deg)" if tilt else ""}, BASE {press_label(height_axis)} force, '
                f'PD {"ON" if settings["pd"]["enabled"] else "OFF"}')
    channel = CloudChannel()
    if options.box:
        # Both inputs come from the healthy coupon: no defect cloud, height or normal enters planning.
        cloud = vehicle.reference(pose, scan['step_mm'])
        regions = vehicle.planning_regions(scan, options.region, options.roi)
        surface_vehicle = vehicle.planning_surface(regions=regions)
        selection = 'user ROI' if options.roi else options.region or 'crack + dent'
        program.log(f'Coupon detection area {selection}: {len(regions)} ROIs, '
                    'planned height is the nominal plane')
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
        program.log(f'{len(bonnet_path.regions)} user-drawn bonnet areas: horizontal and vertical plans, tool-shape collision check disabled')
    publish_cloud = not options.plan_only or (program.mode != 'real' and not ScanScene.multi_panel())
    if publish_cloud:
        channel.begin(nominal / 1000, truth.get('inspection_scene'))
    program.log(f'Full reference cloud: {len(points)} points; planning source=mesh')
    axes = np.array([[0., 0., 1.]]) if options.box else ScanScene.axes(scan['axes'][part], pose)
    if options.side:
        left = pose[:3, 1]
        axes = axes[(axes @ left > 0) == (options.side == 'left')]
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
        part == 'door' and not settings['door_tool_clearance'])
    reach = Reach(chain, home, clearance, scan['ik']['arm_radii_mm'])
    plan_class = BonnetPathPlan if bonnet_path is not None else MeshHeightPlan
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
    if part == 'door':  # RViz and the report: the door's planning area only (its flat panel under the window)
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
        program.log(f'Door planning area (flat panel below the window) {int(inside.sum())}/{len(points)} points')
    label = 'mesh_height'
    if bonnet_path is not None:
        label += '_drawn'
    elif options.roi_base or options.roi_local:
        label += '_roi'
    if options.inspection_region:
        label += f'_r{options.inspection_region}'
    prefix = 'performance' if options.box else f'real_{part}'
    name = prefix + (f'_{options.side}' if options.side else '') + f'_{label}_{stamp}'
    archive = InspectionArchive.directory(options.web_report, part, options.side)
    if options.web_report:
        name += datetime.now().strftime('_%f')
    report = MeshReport(archive, name, flat=bool(options.web_report))
    files = report.files
    roi_reference = points if part == 'bonnet' else None
    report.plan(plan, roi_reference=roi_reference, roi_cloud_settings=program.config['bonnet_roi_cloud'],
                nominal=nominal, web_report=options.web_report, scene=truth.get('inspection_scene'), side=options.side)
    program.log(f'mesh plan and TCP axes -> {files.path("plan.json") if files.prefix else files.directory}')
    program.topics.reports(files.path('report.png'), files.path('plan.json'))
    if not plan.paths:
        program.log(f'Path planning diagnostics: per-stage stats={plan.diagnostics}, '
                    f'pose rejection causes={reach.failures}, path rejection causes={plan.rejected[:3]}')
        raise RuntimeError('No reachable scan path in the selected area.')
    if options.plan_only:
        return report
    motion = program.config['motion']
    if program.mode == 'real':
        from repair_car.inspection.part_scan.height_dsr import MeshHeightDsr
        program.log(f'Real robot mesh-height: DSR movesx {scan["speed_mm_s"]} mm/s, '
                    f'contact threshold +{scan["default_scan"]["real_contact_offset_n"]} N, '
                    f'points recorded at {scan["default_scan"]["contact_threshold_n"]} N or more, '
                    f'BASE {press_label(height_axis)} press {scan["default_scan"]["survey"]["force_n"]} N, '
                    f'press axis stiffness {settings["stiffness"][2] / 1000:g} N/mm, settle after approach {settings["press_settle_s"]:g} s')
        runner = MeshHeightDsr(program, plan, channel, report)
    else:
        program.log(f'Detection {scan["speed_mm_s"]} mm/s (custom); ordinary moves {motion["linear_speed"]} mm/s, '
                    f'approach {motion["approach_speed"]} mm/s, close approach {settings["approach_speed_mm_s"]} mm/s (DSR)')
        force_axis = f'BASE {press_label(height_axis)}' if part == 'door' else 'TCP axis'
        program.log(f'Detection position restoring factor x{settings["position_stiffness_scale"]}, '
                    f'extra position correction limit {settings["position_feedback_limit_mm"]} mm; '
                    f'{force_axis} force control {scan["default_scan"]["survey"]["force_n"]} N, '
                    f'points recorded at {scan["default_scan"]["contact_threshold_n"]} N or more')
        pd = settings['pd']
        program.log(f'Detection PD {"ON" if pd["enabled"] else "OFF"}: '
                    f'position Kp={pd["position_kp"]}, Kd={pd["position_kd_s"]} s; '
                    f'orientation Kp={pd["orientation_kp"]}, Kd={pd["orientation_kd_s"]} s; no I')
        from repair_car.inspection.part_scan.mesh_run import MeshHeightScan
        runner = MeshHeightScan(program, plan, channel, report)
    try:
        runner.run()
    finally:
        program.log(f'Combined subplot report -> {files.path("report.png")}')
        if report.rois is not None:
            roi_file = files.path('roi.json')
            program.topics.regions(report.rois.regions, report.rois.limit)
            program.topics.record(roi_file=str(roi_file))
            program.topics.reports(roi_file)
        if runner.points:
            path = PointCloudFile.save(files.path('measured.ply'), runner.points)
            program.log(f'Measured contact points (normal force above the default contact threshold) -> {path}')
            program.topics.record(measured_cloud=str(path), points=len(runner.points))
            program.topics.reports(path)
    return report
