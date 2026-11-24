"""Where real_scan's object is: the virtual session's scene (data/scene_truth.json, probe_plan.json) or, on the real
robot, the measured cell in config/real_scene.yaml. Both give the scene_truth / probe_plan fields real_scan uses."""
import time

import numpy as np

if __package__:
    from repair_car.config import PACKAGE, load, read_session
    from repair_car.geometry.mesh_io import read_stl
    from repair_car.geometry.real_vehicle import RealVehicle
else:
    from config import PACKAGE, load, read_session
    from geometry.mesh_io import read_stl
    from geometry.real_vehicle import RealVehicle

# The simulated two-rod rack: its plate below the docked tip (mm, rack frame). The real robot's holder box comes from
# its mesh (real_scene.yaml probe.holder_mesh) about its measured base-plate centre.
RACK_PLATE_MM = [[-14., -30., -16.], [14., 30., -13.]]
DOOR_DEMO_PANELS = {'demo_left': 'left_door', 'demo_right': 'right_door'}
REAL_DOOR_SCENE = 'real_door_scene.json'
INSPECTION_SCENE = 'inspection_scene.json'
FIXTURE_MODE = 'inspection_fixture.json'
PANELS = ('bonnet', 'left_door', 'right_door')
AT_HOME_REFUSAL = 'Keep the virtual robot at HOME'   # native virtual_force: the Gazebo arm not yet within 1 deg of HOME
SETTLE_AT_HOME_S = 10.


class ScanScene:
    @staticmethod
    def multi_panel():
        return (read_session(FIXTURE_MODE) or {}).get('type') == 'demo_final'

    @staticmethod
    def at_home(program, call):
        """call() of a stand service (fixture/select, lift/command): the controller reports the HOME move done before
        the Gazebo arm that the session checks has settled there, so that refusal is retried while it settles."""
        deadline = time.monotonic() + SETTLE_AT_HOME_S
        while True:
            try:
                return call()
            except RuntimeError as error:
                if AT_HOME_REFUSAL not in str(error) or time.monotonic() >= deadline:
                    raise
            program.robot.check_cancelled()
            time.sleep(.3)

    @staticmethod
    def prepare_virtual(program, panel):
        if not ScanScene.multi_panel() or panel == ScanScene.virtual_panel():
            return
        from onrobot_rg_msgs.srv import SetCommand
        program.home()
        if program.robot.check_motion() != 0:
            raise RuntimeError('Stop the robot before changing the virtual vehicle placement.')
        robot = program.robot
        previous = robot.timeout
        robot.timeout = max(previous, 8.)
        try:
            ScanScene.at_home(program, lambda: robot.request(program.node.get_namespace() + '/fixture/select',
                                                            SetCommand, command=panel))
        finally:
            robot.timeout = previous
        deadline = time.monotonic() + 3.
        while ScanScene.virtual_panel() != panel:
            if time.monotonic() >= deadline:
                raise RuntimeError('Could not confirm the inspection placement of the virtual vehicle.')
            robot.executor.spin_once(timeout_sec=.05)

    @staticmethod
    def virtual_panel():
        scene = read_session('scene_truth.json') or {}
        if scene.get('model', scene.get('type')) != 'real':
            return None
        yaw = np.radians(scene['pose_base_mm_deg'][2])
        panel = ('bonnet' if np.cos(yaw) < -.9 else ('left_door' if np.sin(yaw) > 0 else 'right_door')
                 if abs(np.sin(yaw)) > .9 else None)
        if panel and ScanScene.multi_panel():
            # demo_final turns and raises the lift stand: the panel only at its exact demo placement and height
            pose, base_z = ScanScene.inspection_pose(panel, vehicle_height_mm=scene['size_mm'][2])
            here = np.r_[scene['pose_base_mm_deg'][:2], scene['base_z_mm']]
            if np.abs(here - np.r_[pose[:2], base_z]).max() > .5:
                return None
        return panel

    @staticmethod
    def active_panel():
        scene = read_session(INSPECTION_SCENE)
        if scene is not None:
            return scene['panel']
        door = read_session(REAL_DOOR_SCENE)
        return DOOR_DEMO_PANELS.get(door['type']) if door else None

    @staticmethod
    def inspection_pose(part, cell=None, vehicle_height_mm=None, *, placement=None):
        """Use a taught placement when available, otherwise the door's demo scene placement."""
        cell = cell or load('real_scene')
        vehicle = cell['vehicle']
        pose = cell.get('inspection_poses', {}).get(part)
        demo_name = next((name for name, panel in DOOR_DEMO_PANELS.items() if panel == part), None)
        if pose is None and demo_name:
            demo = load('vehicles')['demo']
            height = RealVehicle().size[2] if vehicle_height_mm is None else vehicle_height_mm
            base_z = vehicle['table_z_mm'] + demo['height_mm'] - height
            pose = [*demo['centre_mm'], demo['yaw_deg'][demo_name], base_z]
        elif pose is None:
            pose = [*vehicle['pose_base_mm_deg'], vehicle['base_z_mm']]
        if len(pose) != 4 or not np.isfinite(pose).all():
            raise ValueError(f'real_scene.yaml inspection_poses.{part}: enter the [X, Y, yaw, Z] placement')
        xy_yaw = pose[:3] if placement is None else placement
        side = {'left_door': 1., 'right_door': -1.}.get(part)
        if not np.isfinite(xy_yaw).all() or side is not None and side * np.sin(np.radians(xy_yaw[2])) <= .9:
            source = '--car-pose' if placement is not None else f'real_scene.yaml inspection_poses.{part}'
            raise ValueError(f'{source}: enter the measured placement with that door facing the robot')
        return xy_yaw, pose[3]

    @staticmethod
    def real_door(side, *, reposition=False):
        active = read_session(INSPECTION_SCENE)
        declared = read_session(REAL_DOOR_SCENE)
        active_panel = active['panel'] if active else None
        declared_panel = DOOR_DEMO_PANELS[declared['type']] if declared else None
        panel = f'{side}_door' if side else active_panel or declared_panel
        facing = active_panel or declared_panel
        if not reposition and facing and facing != panel:
            raise RuntimeError(f'The current real placement is {facing}. Requested door: {side}')
        if panel in DOOR_DEMO_PANELS.values():
            scene = active if panel == active_panel else declared if panel == declared_panel else None
            return panel.removesuffix('_door'), scene
        return side, None

    @staticmethod
    def load(mode, kind, holder=True, inspection_part=None, door_scene=None):
        """kind: real (the team vehicle) or performance (the coupon, simulation only). -> (truth, plan); holder: the
        real probe holder's pose required (the scans' rack obstacle), not for programs that never plan near it."""
        if mode == 'virtual':
            truth, plan = read_session('scene_truth.json'), read_session('probe_plan.json')
            if not truth or truth.get('model', truth['type']) != kind or not plan:  # demo* scenes: model real
                raise RuntimeError(f'A run_virtual.sh --car {kind} virtual session is required')
            # A placed holder (probe_tool.py holder_pose: the real holder, as in --car demo) is the rack obstacle.
            return truth, {**plan, 'tip_pose_base_mm_deg': plan.get('rack_pose_base_mm_deg', plan['tip_pose_base_mm_deg']),
                           'rack_box_mm': plan.get('rack_box_mm', RACK_PLATE_MM)}
        if kind != 'real':
            raise RuntimeError('The real robot only supports --part scans (--box is for the virtual coupon only)')
        cell = load('real_scene')
        vehicle, probe = cell['vehicle'], cell['probe']
        missing = [f'{group}.{key}' for group in ('vehicle', 'probe') for key, value in cell[group].items()
                   if value is None and key != 'tool_weight_name' and (holder or key != 'holder_pose_base_mm_deg')]
        if missing:
            raise RuntimeError(f'Enter the measured values in config/real_scene.yaml: {", ".join(missing)}')
        mesh = read_stl(PACKAGE / probe['holder_mesh']).reshape(-1, 3)
        size = RealVehicle().size
        if door_scene is not None:
            pose, base_z = door_scene['pose_base_mm_deg'], door_scene['base_z_mm']
        elif inspection_part:
            pose, base_z = ScanScene.inspection_pose(inspection_part, cell, size[2])
        else:
            pose, base_z = vehicle['pose_base_mm_deg'], vehicle['base_z_mm']
        truth = dict(type='real', pose_base_mm_deg=pose, base_z_mm=base_z,
                     table_z_mm=vehicle['table_z_mm'], size_mm=size.tolist())
        # The measured tool weight profile; ProbeToolWeight (robot.yaml) only while none is measured.
        tool = probe['tool_weight_name'] or load('robot')['probe']['tool_name']
        return truth, dict(tcp_mm_deg=probe['tcp_mm_deg'],
                           tip_pose_base_mm_deg=probe['holder_pose_base_mm_deg'],
                           rack_box_mm=[mesh.min(0).tolist(), mesh.max(0).tolist()], tool_name=tool)

    @staticmethod
    def axes(axes, pose):
        """real_scan.axes (BASE) are given for the vehicle facing the robot (yaw 180 deg, `real`): turned about z with
        the vehicle's yaw, so that a door axis stays on its door (demo_left / demo_right)."""
        yaw = np.arctan2(pose[1, 0], pose[0, 0]) - np.pi
        turn = np.array([[np.cos(yaw), -np.sin(yaw), 0.], [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
        return np.round(np.asarray(axes, float) @ turn.T, 12)
