"""Measured point cloud of one part of the real vehicle in the native Gazebo session, by fixed-axis force control
and compliance (the DSR services), as contact_scrape scrapes the block.

  host:      ./run_virtual.sh --car real --probe ball
             ./run_virtual.sh --car performance --probe ball
  container: ros2 run repair_car pick_probe
             ros2 run repair_car real_scan --part {bumper,bonnet,body,door} [--side {left,right}] [--plan-only]
                                      [--pitch MM] [--speed MM_S]
             ros2 run repair_car real_scan --box [--plan-only] [--pitch MM] [--speed MM_S]
             ros2 run repair_car real_scan --box --controller mesh-height [--roi X0 Y0 X1 Y1]

Real robot (./run_real.sh): config/real_scene.yaml supplies the cell and ProbeTCP. With --car demo_left or
demo_right, --part door uses the session's selected door and placement. Real mesh-height runs DSR movesx
strokes (part_scan/height_dsr.py). Mesh-normal is available for planning only.

Every run also publishes custom_interfaces topics (part_scan/scan_topics.py): /scan/samples (ContactSampleArray),
/scan/progress (ScanPart feedback), /scan/roi (RoiArray, default controller) and /scan/result (ScanPart result);
web_control's ScanPart action /scan_part starts real_scan and relays them.

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
   outer surface the probe can touch). Sampled once per part and reused: repair_car/pointcloud/
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
   after the last. Contact points appear in RViz Cloud measured during the scan. The measured PLY is saved in repair_car/data.
5. Report: repair_car/data stores the nominal and measured point clouds, a top-view
   height map with mesh-filled gaps, measured defect ROIs, and the assessed damage percentage.
   The samples CSV and ROI JSON retain the measurements and ROI evidence."""
import argparse
from repair_car.robot_control.robot_bridge import program_args
import json
import signal
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from repair_car.config import PACKAGE, CONFIG_DIR, CONFIG_ROOT
from repair_car.inspection.part_scan import BOX_CONTROLLERS, MESH_CONTROLLERS, SCAN_CONTROLLERS
from repair_car.inspection.part_scan.archive import InspectionArchive
from repair_car.inspection.part_scan.bonnet_path import BonnetPath
from repair_car.pointcloud.cloud import PartCloud
from repair_car.inspection.part_scan.planner import PartPlan
from repair_car.report.report import ScanReport
from repair_car.inspection.part_scan.run import CloudChannel
from repair_car.inspection.part_scan.scan_topics import ScanTopics
from repair_car.inspection.part_scan.tool import Reach, ToolClearance, robot_chain, scene_points
from repair_car.probe.pick_probe import ProbePickup
from repair_car.robot_control.program import RobotProgram
from repair_car.geometry.real_vehicle import PARTS, RealVehicle
from repair_car.robot_control.rotations import rotation_zyz
from repair_car.inspection.scan_scene import ScanScene
from repair_car.pointcloud.pointcloud_file import PointCloudFile
from repair_car.geometry.vehicle_model import pose_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    saved_bonnet = CONFIG_DIR / 'bonnet_path.json'
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
    parser.add_argument('--bonnet-path', nargs='?', type=Path, const=saved_bonnet, metavar='JSON',
                        help='Require the bonnet editor selection, or read a specific selection JSON file')
    parser.add_argument('--full-bonnet', action='store_true',
                        help='Scan the full bonnet with mesh-height, bypassing the saved editor selection')
    parser.add_argument('--result-manifest', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--rescan-drawn-bonnet', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--inspection-position', action='store_true',
                        help='Use the panel inspection placement (automatic for real bonnet scans)')
    parser.add_argument('--inspection-region', type=int, default=0,
                        help='Region number in the web inspection report')
    parser.add_argument('--probe-tip-mm', nargs=2, type=float, metavar=('RADIUS', 'TCP_Z'),
                        help='Real --roi-base only: mounted ball radius and flange-to-tip Z in mm for this scan')
    parser.add_argument('--side', choices=['left', 'right'],
                        help='door side; defaults to the selected demo door in a real demo session')
    parser.add_argument('--controller', choices=SCAN_CONTROLLERS, default='default')
    parser.add_argument('--replan', action='store_true', help='mesh controllers: rebuild the plan cache')
    options = parser.parse_args(program_args())
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    if options.gaussian_sigma_ms is not None and (options.gaussian_sigma_ms < 0 or options.controller != 'default'):
        parser.error('--gaussian-sigma-ms requires controller default and a nonnegative value')
    if options.replan and options.controller not in MESH_CONTROLLERS:
        parser.error('--replan requires --controller mesh-height')
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
    full_bonnet = options.part == 'bonnet' and options.controller == 'mesh-height' and not (
        options.roi_base or options.roi_local)
    if options.full_bonnet and (not full_bonnet or options.bonnet_path is not None):
        parser.error('--full-bonnet requires mesh-height bonnet without another region selection')
    drawn_bonnet = full_bonnet and not options.full_bonnet
    if options.bonnet_path is not None and not drawn_bonnet:
        parser.error('--bonnet-path requires --part bonnet --controller mesh-height without other ROI options')
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
    options.web_report = InspectionArchive.metadata()
    program = RobotProgram('real_scan')
    topics = None
    try:
        selected_file, options.bonnet_path = options.bonnet_path, None
        shared_bonnet = CONFIG_ROOT / saved_bonnet.name
        saved = saved_bonnet if saved_bonnet.is_file() else shared_bonnet
        if selected_file == saved_bonnet and program.mode == 'real':
            selected_file = saved
        if drawn_bonnet and selected_file is None and program.mode == 'real':
            selected_file = saved if saved.is_file() else None
        if selected_file is not None:
            if not selected_file.is_file():
                raise ValueError('Save the inspection area in the bonnet editor first.')
            options.bonnet_path = BonnetPath.load(selected_file)
            program.log(f'Using the saved bonnet inspection area: {selected_file}')
        door_scene = None
        if ScanScene.multi_panel() and options.part == 'door' and not options.side:
            parser.error('The demo_final door inspection needs --side left or --side right.')
        if program.mode == 'real' and options.part == 'door' and not ScanScene.multi_panel():
            options.side, door_scene = ScanScene.real_door(options.side, reposition=options.inspection_position)
        name = 'box' if options.box else f'{options.part}_{options.side}' if options.side else options.part
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        prefix = 'performance' if options.box else f'real_{name}'
        if options.inspection_region:
            prefix += f'_r{options.inspection_region}'
        topics = program.topics = ScanTopics(program.node, f'{prefix}_{stamp}', name)
        scan_part(program, options, parser, name, prefix, stamp, door_scene=door_scene)
        if options.result_manifest is not None:
            options.result_manifest.write_text(json.dumps(topics.outcome.get('reports', [])))
        topics.finish(True, 'done')
    except BaseException as error:
        if topics is not None:
            topics.finish(False, 'stopped' if isinstance(error, KeyboardInterrupt) else str(error) or type(error).__name__)
        raise
    finally:
        program.close()


def scan_part(program, options, parser, name, prefix, stamp, *, door_scene=None):
    program.scan_blend = options.part != 'bonnet'
    panel = 'bonnet' if options.part == 'bonnet' else f'{options.side}_door'
    fixture = ScanScene.multi_panel() and options.part in ('bonnet', 'door')
    if fixture and program.mode == 'real' and not options.plan_only and not options.inspection_position:
        raise RuntimeError('Place the vehicle with that side facing the robot, then confirm with --inspection-position.')
    inspection_part = panel if options.inspection_position or (program.mode == 'real' and panel == 'bonnet') else None
    kind = 'performance' if options.box else 'real'
    truth, plan = ScanScene.load(program.mode, kind,
                                 inspection_part=panel if fixture else inspection_part, door_scene=door_scene)
    if fixture and program.mode == 'virtual' and options.plan_only:
        pose, base_z = ScanScene.inspection_pose(panel)
        truth = dict(truth, pose_base_mm_deg=pose, base_z_mm=base_z)
    if program.mode == 'real':
        truth['inspection_scene'] = dict(panel=panel if options.part == 'door' and options.side else options.part,
                                         run_id=program.topics.run_id,
                                         pose_base_mm_deg=truth['pose_base_mm_deg'], base_z_mm=truth['base_z_mm'])
    config, probe = program.config, program.config['probe']
    if options.probe_tip_mm:
        if program.mode != 'real':
            parser.error('--probe-tip-mm requires the real robot')
        probe['tip_radius_mm'], plan['tcp_mm_deg'][2] = options.probe_tip_mm
    scan = deepcopy(config['real_scan'])
    if options.part == 'bonnet':
        margin_key = ('bonnet_real_travel_margin_mm' if program.mode == 'real' and options.controller != 'mesh-height'
                      else 'bonnet_travel_margin_mm')
        scan['travel_margin_mm'] = scan['default_scan'][margin_key]
    if options.box:
        scan.update(config['performance_scan'])
        scan['report']['preserve_absolute'] = True
    scan.update({key: value for key, value in (('pitch_mm', options.pitch), ('speed_mm_s', options.speed)) if value})
    if options.gaussian_sigma_ms is not None:
        scan['report']['gaussian_sigma_s'] = options.gaussian_sigma_ms / 1000
    if options.controller == 'default':
        from repair_car.inspection.part_scan.staged_scan import DefaultScan
        scan = DefaultScan.survey_settings(scan, real=not options.box)
    if program.mode == 'real':
        if program.robot.check_motion() != 0:
            raise RuntimeError('Stop the robot before reading the active TCP.')
        # Detection uses real_scene.yaml's TCP (plan['tcp_mm_deg']), not the pendant's: the controller still moves
        # and reports its active TCP, so the probe converts commands and readings by the difference.
        defined, active = plan['tcp_mm_deg'], program.robot.get_active_tcp_profile()
        turn = np.degrees(np.arccos(np.clip((np.trace(rotation_zyz(defined[3:]).T @ rotation_zyz(active[3:])) - 1) / 2,
                                            -1, 1)))
        if turn > .5:
            raise RuntimeError(f'real_scene.yaml TCP {defined} and the controller active TCP {np.round(active, 2).tolist()} '
                               f'differ in orientation by {turn:.1f}°: a position correction alone cannot match them')
        program.probe.offset = rotation_zyz(active[3:]).T @ np.subtract(defined[:3], active[:3])
        program.log(f'Detection TCP: real_scene.yaml {defined} (mm, deg); controller active TCP {np.round(active, 2).tolist()} '
                    f'-> commanded and measured positions corrected by the tool offset {np.round(program.probe.offset, 2).tolist()} mm')
        program.robot.verify_profiles(plan['tool_name'])  # real_scene.yaml tool_weight_name, else ProbeToolWeight
        from repair_car.calibration.force_offset import ForceOffset
        offset = ForceOffset.load(program)
        if offset is None:
            program.log('No force offset: measuring it with ros2 run repair_car force_offset corrects the readings and the press force')
        else:
            program.probe.bias = offset
            program.log(f'Force offset applied (config/force_offset.json): F {np.round(offset[:3], 3).tolist()} N')
    elif not options.plan_only:
        ProbePickup(program.node, program.robot, plan, config['motion']).require_attached()
        if fixture:
            ScanScene.prepare_virtual(program, panel)
            truth, plan = ScanScene.load(program.mode, kind)
    if options.box:
        from repair_car.experiments.performance_sample import PerformanceSample, PerformanceReport
        vehicle = PerformanceSample()
        if vehicle.source_hash != truth['source_hash']:
            raise RuntimeError('The coupon mesh changed. Restart the performance session.')
    else:
        vehicle = None if options.controller in MESH_CONTROLLERS else RealVehicle()
    pose = pose_matrix(*truth['pose_base_mm_deg'], truth['base_z_mm'])
    program.top_mm = pose[2, 3] + (vehicle.size[2] if vehicle is not None else truth['size_mm'][2])
    if options.controller in MESH_CONTROLLERS:
        from repair_car.inspection.mesh_scan import run_mesh_scan
        report = run_mesh_scan(program, options, deepcopy(scan), vehicle, pose, plan, truth, stamp)
        if (options.controller == 'mesh-height' and name in ('bonnet', 'door_left', 'door_right', 'door') and
                not options.plan_only and not options.roi_base
                and (not options.bonnet_path or options.rescan_drawn_bonnet)):
            final = rescan(program, options, scan, vehicle, pose, plan, truth, report)
            mesh = final.figures.save_mesh()
            program.topics.reports(mesh)
            program.log(f'Predicted mesh image -> {mesh}')
        return
    archive = InspectionArchive.directory(options.web_report, name)
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
            raise RuntimeError('The bonnet has no point the probe can reach.')
        program.log(f'{name}: {keep.sum()}/{len(points)} reference points reachable at the lift height')
        points = points[keep]
        planning_cloud = SimpleNamespace(part=cloud.part, points=points)
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
    publish_cloud = not options.plan_only or (program.mode != 'real' and not fixture)
    if publish_cloud:
        channel.begin(points / 1000, truth.get('inspection_scene'))
    program.log(f'{name}: reference cloud {len(points)} points ({cloud.path})')
    home_position = chain.forward(home)[0][:3, 3] * 1000
    if program.mode == 'real':
        controller_home = program.robot.fkin(config['robot']['home_joints_deg'])
        controller_home = np.asarray(controller_home[:3]) + rotation_zyz(controller_home[3:]) @ program.probe.offset
        if np.linalg.norm(home_position - controller_home) > 2.0:
            raise RuntimeError(f'The real robot TCP does not match the {plan["tcp_mm_deg"]} (mm, deg) used by the plan.')
    program.log(f'{name}: computing scan paths and approach/retract poses')
    part_plan = PartPlan(planning_cloud, axes, clearance, reach, scan, probe['tip_radius_mm'], program.safe_z())
    program.log(f'{name}: {part_plan.summary(scan["speed_mm_s"])}')
    touchable = points[part_plan.on_measurable(points)]
    if publish_cloud:
        channel.write('reference', touchable / 1000)
        program.log(f'{name}: RViz Cloud reference -> {len(touchable)} measurable points')
    report = ScanReport(archive, f'{prefix}_{stamp}', cloud.points, options.web_report)
    report.save_reference()
    evaluation = (PerformanceReport(vehicle,pose,cloud,archive,f'{prefix}_{stamp}',
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
            raise RuntimeError(f'{name}: no scan path with a feasible approach and retract.')
        return
    program.home(scan_travel=True)
    actual, _ = program.probe.read()
    if np.linalg.norm(home_position - actual[:3]) > 2.0:
        raise RuntimeError('URDF/ProbeTCP HOME differs from the controller HOME by more than 2 mm')
    scanner = DefaultScan(program, scan, part_plan, channel, report, evaluation, name, report.path.parent, stamp)
    try:
        scanner.run()
    finally:
        scanner.save_report()


def rescan(program, options, scan, vehicle, pose, plan, truth, survey):
    """Join nearby candidate areas in the panel plane and repeat measurement with the mesh-height planner."""
    from repair_car.inspection.mesh_scan import run_mesh_scan
    from repair_car.inspection.part_scan.rescan import RescanAreas
    door = options.part == 'door'
    settings = scan['mesh_height']['rescan']
    regions = survey.rois.regions if survey.rois else []
    boxes = RescanAreas(settings).boxes(regions, plane=(1, 2) if door else (0, 1))
    if not boxes:
        program.log(f'No detail rescan: no {"/".join(settings["types"])} ROI')
        return survey
    force = scan['mesh_height']['door_force_n'] if door else settings['force_n']
    program.log(f'Detail rescan: {len(boxes)} {"/".join(settings["types"])} ROI areas '
                f'BASE {"YZ" if door else "XY"} mm {boxes}, {settings["speed_mm_s"]:g} mm/s, {force:g} N')
    again = SimpleNamespace(**{**vars(options), 'roi_base': boxes, 'speed': settings['speed_mm_s'], 'replan': False})
    scan = dict(deepcopy(scan), speed_mm_s=settings['speed_mm_s'], rescan_of=survey.files.run)
    scan['mesh_height']['force_n'] = force
    return run_mesh_scan(program, again, scan, vehicle, pose, plan, truth, datetime.now().strftime('%Y%m%d_%H%M%S'))

if __name__ == '__main__':
    main()
