"""Where real_scan's object is: the virtual session's scene (session/scene_truth.json, probe_plan.json) or, on the real
robot, the measured cell in config/real_scene.yaml. Both give the scene_truth / probe_plan fields real_scan uses."""
import numpy as np

from .configs import PACKAGE, load, read_session

# The simulated two-rod rack: its plate below the docked tip (mm, rack frame). The real robot's holder box comes from
# its mesh (real_scene.yaml probe.holder_mesh) about its measured base-plate centre.
RACK_PLATE_MM = [[-14., -30., -16.], [14., 30., -13.]]


class ScanScene:
    @staticmethod
    def inspection_pose(part, cell=None, vehicle_height_mm=None):
        """Use a taught placement when available, otherwise the door's demo scene placement."""
        from .real_vehicle import RealVehicle

        cell = cell or load('real_scene')
        vehicle = cell['vehicle']
        pose = cell.get('inspection_poses', {}).get(part)
        demo_name = {'left_door': 'demo_left', 'right_door': 'demo_right'}.get(part)
        if pose is None and demo_name:
            demo = load('vehicles')['demo']
            height = RealVehicle().size[2] if vehicle_height_mm is None else vehicle_height_mm
            base_z = vehicle['table_z_mm'] + demo['height_mm'] - height
            pose = [*demo['centre_mm'], demo['yaw_deg'][demo_name], base_z]
        elif pose is None:
            pose = [*vehicle['pose_base_mm_deg'], vehicle['base_z_mm']]
        if len(pose) != 4 or not np.isfinite(pose).all():
            raise ValueError(f'real_scene.yaml inspection_poses.{part}: [X, Y, yaw, Z] 배치를 입력하세요')
        side = {'left_door': 1., 'right_door': -1.}.get(part)
        if side is not None and side * np.sin(np.radians(pose[2])) <= .9:
            raise ValueError(f'real_scene.yaml inspection_poses.{part}: 해당 도어가 로봇을 향하는 실측 배치를 입력하세요')
        return pose[:3], pose[3]

    @staticmethod
    def load(mode, kind, holder=True, inspection_part=None):
        """kind: real (the team vehicle) or performance (the coupon, simulation only). -> (truth, plan); holder: the
        real probe holder's pose required (the scans' rack obstacle), not for programs that never plan near it."""
        if mode == 'virtual':
            truth, plan = read_session('scene_truth.json'), read_session('probe_plan.json')
            if not truth or truth.get('model', truth['type']) != kind or not plan:  # demo* scenes: model real
                raise RuntimeError(f'run_virtual.sh --car {kind} 가상 세션이 필요합니다')
            # A placed holder (probe_tool.py holder_pose: the real holder, as in --car demo) is the rack obstacle.
            return truth, {**plan, 'tip_pose_base_mm_deg': plan.get('rack_pose_base_mm_deg', plan['tip_pose_base_mm_deg']),
                           'rack_box_mm': plan.get('rack_box_mm', RACK_PLATE_MM)}
        if kind != 'real':
            raise RuntimeError('실제 로봇에서는 --part 스캔만 지원합니다 (--box는 가상 시편 전용)')
        cell = load('real_scene')
        vehicle, probe = cell['vehicle'], cell['probe']
        missing = [f'{group}.{key}' for group in ('vehicle', 'probe') for key, value in cell[group].items()
                   if value is None and key != 'tool_weight_name' and (holder or key != 'holder_pose_base_mm_deg')]
        if missing:
            raise RuntimeError(f'config/real_scene.yaml에 실측값을 넣으세요: {", ".join(missing)}')
        from .real_vehicle import RealVehicle, read_stl
        mesh = read_stl(PACKAGE / probe['holder_mesh']).reshape(-1, 3)
        size = RealVehicle().size
        pose, base_z = (ScanScene.inspection_pose(inspection_part, cell, size[2]) if inspection_part else
                        (vehicle['pose_base_mm_deg'], vehicle['base_z_mm']))
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
