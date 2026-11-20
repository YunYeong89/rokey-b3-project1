"""Measured point cloud of one part of the real vehicle in the native Gazebo session, by fixed-axis force control
and compliance (the DSR services), as contact_scrape scrapes the block.

  host:      ./run_virtual.sh --car real --probe ball
             ./run_virtual.sh --car performance --probe ball
  container: ros2 run rokey pick_probe
             ros2 run rokey real_scan --part {bumper,bonnet,body,door} [--side {left,right}] [--plan-only]
                                      [--pitch MM] [--speed MM_S]
             ros2 run rokey real_scan --box [--plan-only] [--pitch MM] [--speed MM_S]
             ros2 run rokey real_scan --box --controller mesh-height [--roi X0 Y0 X1 Y1]

Real robot (./run_real.sh): the vehicle and probe rack placement and the ProbeTCP come from the measured
config/real_scene.yaml (scan_scene.py) instead of the session; --part with the default controller or mesh-height
(the mesh plan run with DSR movesx strokes, part_scan/height_dsr.py). On the real robot,
mesh-normal is available for planning only.

Every run also publishes rokey_interfaces topics (part_scan/scan_topics.py): /scan/samples (ContactSampleArray),
/scan/progress (ScanPart feedback), /scan/roi (RoiArray, default controller) and /scan/result (ScanPart result);
web_control's ScanPart action /scan_part starts real_scan and relays them.

--controller mesh-normal follows the vehicle mesh normals with the probe axis in simulation.
The Cartesian stream runs during detection, with ordinary transfers using DSR moves.
--controller mesh-height keeps a fixed TCP orientation and follows the mesh height along the press axis. On the whole
bonnet it then rescans the survey's crack ROIs, joined into areas, at real_scan.mesh_height.rescan.speed_mm_s.

The default controller first surveys at -0.5 N, then revisits force/depth/contact-loss candidates
at -0.5 N, 2 mm/s and 0.5 mm pitch. Planned strokes use DSR motion and force/compliance control;
force and depth are measurement/ROI data only. Survey and detail data are saved separately.
--box uses a healthy nominal plane for the Gazebo coupon and the same default two-stage workflow.
With --box --controller mesh-height, both the reference cloud and the planning mesh are healthy planes;
planning is restricted to the configured crack/dent regions (--region selects one kind, --roi overrides them).
Defect geometry remains in simulation contact and offline evaluation only.

1. The part's reference point cloud (vehicles.yaml real: the assembly at the session's pose, lift_mm above the
   table; door = the vehicle's left door, then its right one, or only the --side one; window openings have no mesh
   and so no points) -> RViz Cloud reference, replaced after planning by its measurable points only (step 3: the
   outer surface the probe can touch). Sampled once per part and reused: rokey/pointcloud/
   real_<part>_reference.ply (vehicle frame; delete it after changing the mesh or real_scan.spacing_mm).
2. Fixed probe axes per part, no normal vectors (robot.yaml real_scan.axes: bonnet and bumper from above, the doors
   from either side). Along each axis the outermost surface on a grid; a point is accessible when the probe touching
   it along the axis keeps the rest of the tool (shaft, probe body, RG2, wrist: real_scan.tool) clear of the vehicle,
   the probe rack, the robot base and the table, and the pose is reachable from HOME with the arm links clear.
3. Measurable segments (real_scan.measurable): the accessible cells off the edges, not steeper than tilt_deg and,
   on the doors, on the outer skin only; every 0.5 mm line of a pass, horizontal (left-right from above, horizontal
   from a side) and vertical, followed as a straight move (IK continuity), what it gets through kept. Lines every
   pitch selected, their segments joined in contact into one segment per connected measurable region (straight over
   measurable cells or round them); both segment ends approachable from the travel height above the vehicle. One
   path per pass, so 2 paths per axis: a pass cut into regions (a door's character line) visits its segments in
   turn, the probe lifted only between them (straight across at the standoff where clear, else over the travel
   height).
4. Per segment: task compliance + desired force along the axis, movel strokes through the row ends; the force is only
   measured. The probe is lifted only between segments (to the travel height, across, down to the next start), HOME
   after the last. The contact points -> RViz Cloud measured while scanning, and
   rokey/pointcloud/real_<part>_<time>_measured.ply.
5. Report: rokey/images/real_<part>_<time>/report.png shows the nominal and measured point clouds, a top-view
   height map with mesh-filled gaps, measured defect ROIs, and the assessed damage percentage.
   The samples CSV and ROI JSON retain the measurements and ROI evidence."""
import argparse
from .robot_namespace import program_args
import signal
from copy import deepcopy
from datetime import datetime
from types import SimpleNamespace

import numpy as np

from .configs import PACKAGE, output_dir
from .part_scan import BOX_CONTROLLERS, MESH_CONTROLLERS, SCAN_CONTROLLERS
from .part_scan.cloud import PartCloud
from .part_scan.damage_report import web_report_metadata
from .part_scan.planner import PartPlan
from .part_scan.report import ScanReport
from .part_scan.run import CloudChannel
from .part_scan.scan_topics import ScanTopics
from .part_scan.tool import Reach, ToolClearance, robot_chain, scene_points
from .program import RobotProgram
from .real_vehicle import PARTS, RealVehicle
from .rotations import rotation_zyz
from .scan_scene import ScanScene
from .surface_follow.pointcloud_file import PointCloudFile
from .vehicle_model import pose_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--part', choices=PARTS)
    target.add_argument('--box', action='store_true', help='Performance coupon: plan on its healthy nominal top')
    parser.add_argument('--plan-only', action='store_true', help='reference cloud and planned paths only, no contact')
    parser.add_argument('--pitch', type=float, help='row spacing, mm (default robot.yaml real_scan.pitch_mm)')
    parser.add_argument('--speed', type=float, help='stroke speed, mm/s (default real_scan.speed_mm_s)')
    parser.add_argument('--gaussian-sigma-ms', type=float,
                        help='default-controller report only: offline Gaussian sigma in milliseconds')
    parser.add_argument('--roi', nargs=4, type=float, metavar=('X0','Y0','X1','Y1'),
                        help='--box only: coupon-local rectangle in mm, for a shorter performance scan')
    parser.add_argument('--region', choices=['all', 'crack', 'dent'],
                        help='--box --controller mesh-height: known defect regions only (default: all)')
    parser.add_argument('--roi-base', nargs=4, type=float, action='append', metavar=('X0', 'Y0', 'X1', 'Y1'),
                        help='Bonnet mesh-height only: repeatable BASE-frame rectangles in mm, scanned along Y')
    parser.add_argument('--roi-local', nargs=4, type=float, metavar=('U0', 'V0', 'U1', 'V1'),
                        help='Mesh-height rectangle in vehicle mm (bonnet XY, door XZ)')
    parser.add_argument('--inspection-position', action='store_true',
                        help='Use the inspection placement for the selected panel (taught or demo)')
    parser.add_argument('--inspection-region', type=int, default=0,
                        help='Region number in the web inspection report')
    parser.add_argument('--probe-tip-mm', nargs=2, type=float, metavar=('RADIUS', 'TCP_Z'),
                        help='Real --roi-base only: mounted ball radius and flange-to-tip Z in mm for this scan')
    parser.add_argument('--side', choices=['left', 'right'],
                        help='door only: the vehicle\'s left or right door alone (default: left, then right)')
    parser.add_argument('--controller', choices=SCAN_CONTROLLERS, default='default')
    parser.add_argument('--replan', action='store_true', help='mesh controllers: rebuild the plan cache')
    options = parser.parse_args(program_args())
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    if options.gaussian_sigma_ms is not None and (options.gaussian_sigma_ms < 0 or options.controller != 'default'):
        parser.error('--gaussian-sigma-ms requires controller default and a nonnegative value')
    if options.replan and options.controller not in MESH_CONTROLLERS:
        parser.error('--replan requires --controller mesh-normal or mesh-height')
    if options.box and options.controller not in BOX_CONTROLLERS:
        parser.error('--box supports --controller default or mesh-height')
    if options.region and (not options.box or options.controller != 'mesh-height' or options.roi):
        parser.error('--region requires --box --controller mesh-height without --roi')
    if options.roi and (not options.box or options.roi[0]>=options.roi[2] or options.roi[1]>=options.roi[3]):
        parser.error('--roi requires --box and X0 < X1, Y0 < Y1')
    if options.roi_base and (options.part != 'bonnet' or options.controller != 'mesh-height' or
                             not all(np.isfinite(box).all() and box[0] < box[2] and box[1] < box[3]
                                     for box in options.roi_base)):
        parser.error('--roi-base requires --part bonnet --controller mesh-height and X0 < X1, Y0 < Y1')
    if options.roi_local is not None:
        u0, v0, u1, v1 = options.roi_local
        if (options.part not in ('bonnet', 'door') or options.controller != 'mesh-height' or options.roi_base or
                not np.isfinite(options.roi_local).all() or u0 >= u1 or v0 >= v1):
            parser.error('--roi-local requires mesh-height bonnet or door and increasing vehicle-frame bounds')
    if options.inspection_position and (options.part not in ('bonnet', 'door') or
                                        options.part == 'door' and not options.side):
        parser.error('--inspection-position requires bonnet or an explicit door side')
    if options.inspection_region < 0 or (options.inspection_region and options.roi_local is None):
        parser.error('--inspection-region requires a positive number and --roi-local')
    if options.probe_tip_mm and (not options.roi_base or not np.isfinite(options.probe_tip_mm).all() or
                                 min(options.probe_tip_mm) <= 0.):
        parser.error('--probe-tip-mm requires --roi-base and positive RADIUS and TCP_Z')
    if options.pitch is not None and options.pitch <= 0 or options.speed is not None and options.speed <= 0:
        parser.error('--pitch and --speed must be positive')
    if options.side and options.part != 'door':
        parser.error('--side applies to --part door only')
    options.web_report = web_report_metadata()
    name = 'box' if options.box else f'{options.part}_{options.side}' if options.side else options.part
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    prefix = 'performance' if options.box else f'real_{name}'
    if options.inspection_region:
        prefix += f'_r{options.inspection_region}'
    program = RobotProgram('real_scan')
    topics = program.topics = ScanTopics(program.node, f'{prefix}_{stamp}', name)
    try:
        scan_part(program, options, parser, name, prefix, stamp)
        topics.finish(True, '완료')
    except BaseException as error:
        topics.finish(False, '중지' if isinstance(error, KeyboardInterrupt) else str(error) or type(error).__name__)
        raise
    finally:
        program.close()


def scan_part(program, options, parser, name, prefix, stamp):
    panel = 'bonnet' if options.part == 'bonnet' else f'{options.side}_door'
    truth, plan = ScanScene.load(program.mode, 'performance' if options.box else 'real',
                                 inspection_part=panel if options.inspection_position else None)
    config, probe = program.config, program.config['probe']
    if options.probe_tip_mm:
        if program.mode != 'real':
            parser.error('--probe-tip-mm requires the real robot')
        probe['tip_radius_mm'], plan['tcp_mm_deg'][2] = options.probe_tip_mm
    scan = deepcopy(config['real_scan'])
    if options.box:
        scan.update(config['performance_scan'])
        scan['report']['preserve_absolute'] = True
    scan.update({key: value for key, value in (('pitch_mm', options.pitch), ('speed_mm_s', options.speed)) if value})
    if options.gaussian_sigma_ms is not None:
        scan['report']['gaussian_sigma_s'] = options.gaussian_sigma_ms / 1000
    if options.controller == 'default':
        from .part_scan.staged_scan import DefaultScan
        scan = DefaultScan.survey_settings(scan, real=program.mode == 'real')
    if program.mode == 'real':
        if options.controller == 'mesh-normal' and not options.plan_only:
            raise RuntimeError(f'실제 로봇에서 {options.controller}는 --plan-only만 가능합니다 (검출 스트림은 시뮬레이션 전용)')
        if program.robot.check_motion() != 0:
            raise RuntimeError('활성 TCP를 읽기 전에 로봇을 정지하세요.')
        # Detection uses real_scene.yaml's TCP (plan['tcp_mm_deg']), not the pendant's: the controller still moves
        # and reports its active TCP, so the probe converts commands and readings by the difference.
        defined, active = plan['tcp_mm_deg'], program.robot.get_active_tcp_profile()
        turn = np.degrees(np.arccos(np.clip((np.trace(rotation_zyz(defined[3:]).T @ rotation_zyz(active[3:])) - 1) / 2,
                                            -1, 1)))
        if turn > .5:
            raise RuntimeError(f'real_scene.yaml TCP {defined}와 컨트롤러 활성 TCP {np.round(active, 2).tolist()}의 '
                               f'방향이 {turn:.1f}° 다릅니다: 위치 보정만으로 맞출 수 없습니다')
        program.probe.offset = rotation_zyz(active[3:]).T @ np.subtract(defined[:3], active[:3])
        program.log(f'검출 TCP: real_scene.yaml {defined} (mm, deg); 컨트롤러 활성 TCP {np.round(active, 2).tolist()} '
                    f'-> 명령·측정 위치를 공구 좌표 {np.round(program.probe.offset, 2).tolist()} mm 보정')
        program.robot.verify_profiles(plan['tool_name'])  # real_scene.yaml tool_weight_name, else ProbeToolWeight
        from .force_offset import ForceOffset
        offset = ForceOffset.load(program)
        if offset is None:
            program.log('힘 오프셋 없음: ros2 run rokey force_offset으로 측정하면 측정값과 누름 힘이 보정됩니다')
        else:
            program.probe.bias = offset
            program.log(f'힘 오프셋 적용 (session/force_offset.json): F {np.round(offset[:3], 3).tolist()} N')
    else:
        program.robot.verify_profiles(probe['tool_name'], probe['tcp_name'])
    if options.box:
        from .performance_sample import PerformanceSample, PerformanceReport
        vehicle = PerformanceSample()
        if vehicle.source_hash != truth['source_hash']:
            raise RuntimeError('시편 mesh가 변경되었습니다. performance 세션을 재시작하세요.')
    else:
        vehicle = None if options.controller in MESH_CONTROLLERS else RealVehicle()
    pose = pose_matrix(*truth['pose_base_mm_deg'], truth['base_z_mm'])
    program.top_mm = pose[2, 3] + (vehicle.size[2] if vehicle is not None else truth['size_mm'][2])
    clouds = output_dir('pointcloud')
    images = output_dir('images')
    clouds.mkdir(exist_ok=True)
    if options.controller in MESH_CONTROLLERS:
        from .mesh_scan import run_mesh_scan
        report = run_mesh_scan(program, options, deepcopy(scan), vehicle, pose, plan, truth, stamp)
        if (options.controller == 'mesh-height' and options.part == 'bonnet' and not options.plan_only and
                not options.roi_base and not options.roi_local and report.rois is not None):
            rescan(program, options, scan, vehicle, pose, plan, truth, report.rois.regions)
        return
    threshold = scan['default_scan']['contact_threshold_n']
    cloud = (vehicle.reference(pose,scan['step_mm']) if options.box else
             PartCloud(vehicle, options.part, pose, scan['spacing_mm'], PACKAGE / 'pointcloud' / f'real_{options.part}_reference.ply'))
    axes, left = (np.array([[0.,0.,1.]]) if options.box else ScanScene.axes(scan['axes'][options.part], pose)), pose[:3, 1]
    points = cloud.points
    if options.part == 'door':  # each door from its own side: the left one, then the right one, or --side
        axes = np.concatenate([axes[(axes @ left > 0) == (side == 'left')]
                               for side in ([options.side] if options.side else ['left', 'right'])])
        if options.side:
            outward = (points - (points.min(0) + points.max(0)) / 2) @ left
            points = points[(outward > 0) == (options.side == 'left')]
    ik = scan['ik']
    chain = robot_chain(program.robot, plan['tcp_mm_deg'], ik)
    home = np.deg2rad(config['robot']['home_joints_deg'])
    clearance = ToolClearance(scene_points(vehicle.nominal() if options.box else vehicle, pose,
                                          scan['obstacle_mm'], plan['tip_pose_base_mm_deg'], plan['rack_box_mm']),
                              scan['tool'], scan['robot_base_mm'], truth['table_z_mm'] + scan['table_margin_mm'])
    reach = Reach(chain, home, clearance, ik['arm_radii_mm'])
    planning_cloud = cloud
    if options.part == 'bonnet':
        axis = axes[0]
        keep = reach.within_outer_reach(points + scan['standoff_mm'] * axis, axis,
                                        scan['default_scan']['bonnet_standoff_margin_mm'])
        if not np.any(keep):
            raise RuntimeError('보닛에 탐침이 접근 가능한 포인트가 없습니다.')
        program.log(f'{name}: 리프트 높이에서 닿는 참조점 {keep.sum()}/{len(points)}개')
        points = points[keep]
        planning_cloud = SimpleNamespace(part=cloud.part, points=points)
        scan['travel_margin_mm'] = scan['default_scan']['bonnet_travel_margin_mm']
    if options.roi:
        local = (points-pose[:3,3]) @ pose[:3,:3]
        low,high = np.array(options.roi).reshape(2,2)
        if np.any(low < -vehicle.size[:2]/2) or np.any(high > vehicle.size[:2]/2):
            parser.error('--roi lies outside the performance coupon')
        keep = np.all((local[:,:2]>=low)&(local[:,:2]<=high),axis=1)
        if keep.sum()<9:
            parser.error('--roi is too small for the planning grid')
        planning_cloud = SimpleNamespace(part='box',points=points[keep])
    channel = CloudChannel()
    channel.write('reference', points / 1000)
    channel.write('measured', [])
    program.log(f'{name}: reference cloud {len(points)} points ({cloud.path}) -> RViz Cloud reference')
    home_position = chain.forward(home)[0][:3, 3] * 1000
    if program.mode == 'real':
        controller_home = program.robot.fkin(config['robot']['home_joints_deg'])
        controller_home = np.asarray(controller_home[:3]) + rotation_zyz(controller_home[3:]) @ program.probe.offset
        if np.linalg.norm(home_position - controller_home) > 2.0:
            raise RuntimeError(f'실제 로봇의 TCP가 계획에 사용한 {plan["tcp_mm_deg"]} (mm, deg)과 일치하지 않습니다.')
    program.home()
    actual, _ = program.probe.read()
    if np.linalg.norm(home_position - actual[:3]) > 2.0:
        raise RuntimeError('URDF/ProbeTCP HOME differs from the controller HOME by more than 2 mm')
    program.log(f'{name}: 스캔 경로와 접근·이탈 자세를 계산 중입니다')
    part_plan = PartPlan(planning_cloud, axes, clearance, reach, scan, probe['tip_radius_mm'], program.safe_z())
    program.log(f'{name}: {part_plan.summary(scan["speed_mm_s"])}')
    touchable = points[part_plan.on_measurable(points)]
    channel.write('reference', touchable / 1000)
    program.log(f'{name}: RViz Cloud reference -> the {len(touchable)} measurable points (outer surface the probe '
                f'can touch)')
    report = ScanReport(images, f'{prefix}_{stamp}', cloud.points, options.web_report)
    report.save_reference()
    evaluation = (PerformanceReport(vehicle,pose,cloud,images,f'{prefix}_{stamp}',
                                    probe['tip_radius_mm'],threshold,config['performance_scan'],
                                    sigma_s=scan['report']['gaussian_sigma_s'], plot_force=False)
                  if options.box else None)
    for path in report.plan(part_plan.maps):
        program.log(f'{name}: measurable segments and paths -> {path}')
        program.topics.reports(path)
    if evaluation:
        program.log(f'{name}: {evaluation.save_plan(part_plan,options.roi)}')
    if options.plan_only or not part_plan.paths:
        report.save([])
        program.topics.reports(report.path)
        if not part_plan.paths and not options.plan_only:
            raise RuntimeError(f'{name}: 접근과 이탈이 가능한 스캔 경로가 없습니다.')
        return
    scanner = DefaultScan(program, scan, part_plan, channel, report, evaluation, name, clouds, stamp)
    try:
        scanner.run()
    finally:
        scanner.save_report()


def rescan(program, options, scan, vehicle, pose, plan, truth, regions):
    """After a whole-bonnet mesh-height survey: its ROIs of real_scan.mesh_height.rescan.types joined with their
    neighbours into BASE rectangles (part_scan/rescan.py), scanned again as --roi-base at the rescan speed, every
    other mesh-height setting as the survey's; its own report (real_bonnet_mesh_height_roi_<time>)."""
    from .mesh_scan import run_mesh_scan
    from .part_scan.rescan import RescanAreas
    settings = scan['mesh_height']['rescan']
    boxes = RescanAreas(settings).boxes(regions)
    found = sum(g['type'] in settings['types'] for g in regions)
    if not boxes:
        program.log(f'정밀 재측정 없음: {"/".join(settings["types"])} ROI가 없습니다')
        return
    program.log(f'정밀 재측정: {"/".join(settings["types"])} ROI {found}개 -> 영역 {len(boxes)}개 BASE [x0, y0, x1, y1] mm '
                f'{boxes}, {settings["speed_mm_s"]:g} mm/s')
    again = SimpleNamespace(**{**vars(options), 'roi_base': boxes, 'speed': settings['speed_mm_s'], 'replan': False})
    run_mesh_scan(program, again, dict(deepcopy(scan), speed_mm_s=settings['speed_mm_s']), vehicle, pose, plan, truth,
                  datetime.now().strftime('%Y%m%d_%H%M%S'))


if __name__ == '__main__':
    main()
