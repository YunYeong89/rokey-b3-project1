"""Measured point cloud of one part of the real vehicle in the native Gazebo session, by fixed-axis force control
and compliance (the DSR services), as contact_scrape scrapes the block.

  host:      ./run_virtual.sh --car real --probe ball
             ./run_virtual.sh --car performance --probe ball
  container: ros2 run rokey pick_probe
             ros2 run rokey real_scan --part {bumper,bonnet,body,door} [--side {left,right}] [--plan-only]
                                      [--pitch MM] [--speed MM_S]
             ros2 run rokey real_scan --box [--plan-only] [--pitch MM] [--speed MM_S]

--controller cloud-normal plans on the displayed reference cloud with sparse IK only; mesh-normal
uses real_scan.mesh_normal.planning_source (pointcloud by default). The rokey_interfaces Cartesian
stream is used only during detection. Ordinary transfers keep DSR moves and their own speeds.
Wrenches are recorded only; --plan-only sends no motion commands.

--box uses a healthy nominal plane for the Gazebo coupon, with 1.5 N downward force, 10 mm/s
feed, and [3000,3000,100,200,200,200] compliance. It saves point clouds, control/depth graphs,
damage regions and source-mesh evaluation in images/performance_<time>/performance.json.

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
   pitch selected, their segments joined in contact into one path per connected measurable region (straight over
   measurable cells or round them); both path ends approachable from the travel height above the vehicle. So 2 paths
   per axis, more only where a region is cut off (on a door the character line may split the vertical pass).
4. Per path: task compliance + desired force along the axis, movel strokes through the row ends; the force is only
   measured. The probe is lifted only between paths (to the travel height, across, down to the next start), HOME
   after the last. The contact points -> RViz Cloud measured while scanning, and
   rokey/pointcloud/real_<part>_<time>_measured.ply.
5. Report (part_scan/report.py, real_scan.report): rokey/images/real_<part>_<time>_plan.png (measurable cells and
   the paths per pass, also with --plan-only), _surface.png (measured surface
   seen along the axis, from above for the bonnet), _outliers.png (deviation from the ball-bottom reference, outliers
   and ROI marked), _force_raw.png and _force_denoised.png (moving average), _force_residual.png (against the known
   shape), _samples.csv, and _roi.json (part_scan/roi.py: pointcloud_damage geometry + force residual regions;
   confirmed ones as pointcloud_damage ROI regions)."""
import argparse
from copy import deepcopy
from datetime import datetime

import numpy as np

from .configs import PACKAGE, read_session
from .part_scan.cloud import PartCloud
from .part_scan.planner import PartPlan
from .part_scan.report import ScanReport
from .part_scan.roi import ScanRoi
from .part_scan.run import AxisScraper, CloudChannel
from .part_scan.tool import Reach, ToolClearance, robot_chain, scene_points
from .program import RobotProgram
from .real_vehicle import PARTS, RealVehicle
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
    parser.add_argument('--roi', nargs=4, type=float, metavar=('X0','Y0','X1','Y1'),
                        help='--box only: coupon-local rectangle in mm, for a shorter performance scan')
    parser.add_argument('--side', choices=['left', 'right'],
                        help='door only: the vehicle\'s left or right door alone (default: left, then right)')
    parser.add_argument('--controller', choices=['default', 'pid', 'mesh-normal', 'cloud-normal'], default='default',
                        help='cloud-normal: fast cloud planning and custom detection stream; force measurement only')
    parser.add_argument('--replan', action='store_true', help='normal-following modes: rebuild the plan cache')
    options = parser.parse_args()
    if options.replan and options.controller not in ('mesh-normal', 'cloud-normal'):
        parser.error('--replan requires --controller mesh-normal or cloud-normal')
    if options.box and options.controller != 'default':
        parser.error('--box uses the Gazebo default force/compliance controller')
    if options.roi and (not options.box or options.roi[0]>=options.roi[2] or options.roi[1]>=options.roi[3]):
        parser.error('--roi requires --box and X0 < X1, Y0 < Y1')
    if options.pitch is not None and options.pitch <= 0 or options.speed is not None and options.speed <= 0:
        parser.error('--pitch and --speed must be positive')
    if options.side and options.part != 'door':
        parser.error('--side applies to --part door only')
    name = 'box' if options.box else f'{options.part}_{options.side}' if options.side else options.part
    if options.controller == 'pid' and not options.plan_only:
        geometry = read_session('collision_geometry.json') or {}
        if not geometry.get('validated'):
            raise RuntimeError('검증된 MuJoCo 차량 충돌 형상이 필요합니다. '
                               '새 충돌 형상을 적용한 MuJoCo 세션을 시작하세요.')
    program = RobotProgram('real_scan')
    try:
        truth, plan = read_session('scene_truth.json'), read_session('probe_plan.json')
        kind = 'performance' if options.box else 'real'
        if program.mode != 'virtual' or not truth or truth['type'] != kind or not plan:
            raise RuntimeError(f'run_virtual.sh --car {kind} 가상 세션이 필요합니다')
        config, probe = program.config, program.config['probe']
        scan = deepcopy(config['real_scan'])
        if options.box:
            scan.update(config['performance_scan'])
            scan['report']['preserve_absolute'] = True
        scan.update({key: value for key, value in (('pitch_mm', options.pitch), ('speed_mm_s', options.speed)) if value})
        program.robot.verify_profiles(probe['tool_name'], probe['tcp_name'])
        if options.box:
            from .performance_sample import PerformanceSample, PerformanceReport
            vehicle = PerformanceSample()
            if vehicle.source_hash != truth['source_hash']:
                raise RuntimeError('시편 mesh가 변경되었습니다. performance 세션을 재시작하세요.')
        else:
            vehicle = None if options.controller in ('mesh-normal', 'cloud-normal') else RealVehicle()
        pose = pose_matrix(*truth['pose_base_mm_deg'], truth['base_z_mm'])
        program.top_mm = pose[2, 3] + (vehicle.size[2] if vehicle is not None else truth['size_mm'][2])
        stamp, clouds = datetime.now().strftime('%Y%m%d_%H%M%S'), PACKAGE / 'pointcloud'
        clouds.mkdir(exist_ok=True)
        prefix = 'performance' if options.box else f'real_{name}'
        if options.controller in ('mesh-normal', 'cloud-normal'):
            from .mesh_scan import run_mesh_scan
            run_mesh_scan(program, options, scan, vehicle, pose, plan, truth, stamp)
            return
        cloud = (vehicle.reference(pose,scan['step_mm'],clouds/f'performance_{stamp}_reference.ply') if options.box else
                 PartCloud(vehicle, options.part, pose, scan['spacing_mm'], clouds / f'real_{options.part}_reference.ply'))
        axes, left = (np.array([[0.,0.,1.]]) if options.box else np.asarray(scan['axes'][options.part], float)), pose[:3, 1]
        points = cloud.points
        if options.part == 'door':  # each door from its own side: the left one, then the right one, or --side
            axes = np.concatenate([axes[(axes @ left > 0) == (side == 'left')]
                                   for side in ([options.side] if options.side else ['left', 'right'])])
            if options.side:
                outward = (points - (points.min(0) + points.max(0)) / 2) @ left
                points = points[(outward > 0) == (options.side == 'left')]
        channel = CloudChannel()
        channel.write('reference', points / 1000)
        channel.write('measured', [])
        program.log(f'{name}: reference cloud {len(points)} points ({cloud.path}) -> RViz Cloud reference')
        program.home()
        ik = scan['ik']
        chain = robot_chain(program.robot, plan['tcp_mm_deg'], ik)
        home = np.deg2rad(config['robot']['home_joints_deg'])
        actual, _ = program.robot.get_current_posx()
        if np.linalg.norm(chain.forward(home)[0][:3, 3] * 1000 - actual[:3]) > 2.0:
            raise RuntimeError('URDF/ProbeTCP HOME differs from the controller HOME by more than 2 mm')
        clearance = ToolClearance(scene_points(vehicle.nominal() if options.box else vehicle, pose,
                                              scan['obstacle_mm'], plan['tip_pose_base_mm_deg']),
                                  scan['tool'], scan['robot_base_mm'], truth['table_z_mm'] + scan['table_margin_mm'])
        planning_cloud = cloud
        if options.roi:
            from types import SimpleNamespace
            local = (points-pose[:3,3]) @ pose[:3,:3]
            low,high = np.array(options.roi).reshape(2,2)
            if np.any(low < -vehicle.size[:2]/2) or np.any(high > vehicle.size[:2]/2):
                parser.error('--roi lies outside the performance coupon')
            keep = np.all((local[:,:2]>=low)&(local[:,:2]<=high),axis=1)
            if keep.sum()<9:
                parser.error('--roi is too small for the planning grid')
            planning_cloud = SimpleNamespace(part='box',points=points[keep])
        part_plan = PartPlan(planning_cloud, axes, clearance, Reach(chain, home, clearance, ik['arm_radii_mm']), scan,
                             probe['tip_radius_mm'], program.safe_z())
        program.log(f'{name}: {part_plan.summary(scan["speed_mm_s"])}')
        touchable = points[part_plan.on_measurable(points)]
        channel.write('reference', touchable / 1000)
        program.log(f'{name}: RViz Cloud reference -> the {len(touchable)} measurable points (outer surface the probe '
                    f'can touch)')
        report = ScanReport(PACKAGE / 'images', f'{prefix}_{stamp}', config['scan']['touch_threshold'], scan['report'])
        evaluation = (PerformanceReport(vehicle,pose,cloud,PACKAGE/'images',f'{prefix}_{stamp}',
                                        probe['tip_radius_mm'],config['scan']['touch_threshold'],config['performance_scan'])
                      if options.box else None)
        for path in report.plan(part_plan.maps):
            program.log(f'{name}: measurable segments and paths -> {path}')
        if evaluation:
            program.log(f'{name}: {evaluation.save_plan(part_plan,options.roi)}')
        if options.plan_only or not part_plan.paths:
            return
        scraper = AxisScraper(program, scan, channel)
        if options.controller == 'pid':
            from .pid_controller import PIDScan
            scan['speed_mm_s'] = options.speed or 10.
            scraper = PIDScan(program, scan, channel)
        completed = False
        try:
            for number, (positions, axis) in enumerate(part_plan.paths, 1):
                side = ' (left side)' if axis @ left > .5 else ' (right side)' if axis @ left < -.5 else ''
                program.log(f'{name} path {number}/{len(part_plan.paths)}: axis {axis.round(0).tolist()}{side}, '
                            f'{np.linalg.norm(np.diff(positions, axis=0), axis=1).sum():.0f} mm')
                scraper.run(positions, axis)
            program.home()
            completed = True
        finally:
            program.probe.close()
            channel.write('measured', scraper.points)
            if scraper.points:
                path = PointCloudFile.save(clouds / f'{prefix}_{stamp}_measured.ply', scraper.points)
                program.log(f'{name}: measured cloud {len(scraper.points)} points -> {path}')
            if options.controller == 'default' and scraper.records:
                rois = ScanRoi(scraper.records, part_plan.grids, scan['force_n'], config['scan']['touch_threshold'],
                               scan['report'].get('preserve_absolute', False))
                for path in report.save(scraper.records, part_plan.grids, rois):
                    program.log(f'{name}: {path}')
                program.log(f'{name}: ROI {sum(r["confirmed"] for r in rois.regions)} confirmed, '
                            f'{sum(not r["confirmed"] for r in rois.regions)} candidates -> '
                            f'{rois.save(report.directory / f"{report.name}_roi.json", name)}')
            if evaluation:
                program.log(f'{name}: {evaluation.save(scraper,completed,part_plan)}')
    finally:
        program.close()


if __name__ == '__main__':
    main()
