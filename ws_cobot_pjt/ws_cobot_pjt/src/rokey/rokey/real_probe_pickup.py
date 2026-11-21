"""Position-controlled pickup and release at the taught real-robot rack."""
import time

import numpy as np
from dsr_msgs2.srv import SetCtrlBoxDigitalOutput

if __package__:
    from .contact_probe import ContactProbe
    from .rotations import rotation_zyz
else:
    from contact_probe import ContactProbe
    from rotations import rotation_zyz


class RealProbePickup:
    GRIPPER_WAIT_S = 1.
    NEAR_GRASP_MM = 6.
    # The taught grasp pose measured 24.29 N before lifting and 2.25 N after raising 6 mm.
    NEAR_GRASP_LIMIT_N = 28.
    TRANSFER_SPEED = 55.
    TRANSFER_RADIUS_MM = 5.

    def __init__(self, robot, settings, home_joints):
        self.robot, self.settings, self.home_joints = robot, settings, home_joints
        self.log = robot.node.get_logger()
        self.probe = ContactProbe(robot, force=0.)

    def gripper(self, *, opening):
        off, on = (1, 2) if opening else (2, 1)
        service = 'io/set_ctrl_box_digital_output'
        # Match DSR_ROBOT2's OFF=0 and ON=1, switching the opposite output off first.
        self.robot.request(service, SetCtrlBoxDigitalOutput, index=off, value=0)
        self.robot.request(service, SetCtrlBoxDigitalOutput, index=on, value=1)
        time.sleep(self.GRIPPER_WAIT_S)

    def move(self, pose, speed, *, at_fixture=False):
        rotation = rotation_zyz(pose[3:])
        # Near the taught contact point use the measured absolute load; elsewhere use the tared load.
        bias, limit = ((self.probe.bias[:3], self.NEAR_GRASP_LIMIT_N) if at_fixture
                       else (0., self.probe.max_force))
        if self.probe.move(pose[:3], rotation, speed,
                           until=lambda _, wrench: np.linalg.norm(wrench[:3] + bias) > limit,
                           acc=50., angular_speed=40., angular_acc=120.):
            raise RuntimeError(f'탐침 거치대 이동 중 반력 한계 초과 ({limit:g} N)')
        reached, _ = self.robot.get_current_posx()
        if (np.linalg.norm(np.subtract(reached[:3], pose[:3])) > .5
                or self.probe.turn_deg(reached[3:], rotation) > .5):
            raise RuntimeError(f'탐침 거치대 목표 미도달: {pose[:3].tolist()} mm')

    def run(self, *, release=False):
        action = '놓기' if release else '파지'
        if self.robot.get_robot_mode() != 1 or self.robot.get_control_mode() != 3 or self.robot.check_motion() != 0:
            raise RuntimeError(f'탐침 {action}는 자동 모드의 위치 제어 정지 상태에서 실행하세요.')
        if np.max(np.abs(np.subtract(self.robot.get_current_posj(), self.home_joints))) > 1.:
            raise RuntimeError(f'탐침 {action}를 시작하려면 먼저 HOME으로 이동하세요.')
        self.robot.verify_profiles(self.settings['tool_weight_name'], self.settings['pickup_tcp_name'])

        home, solution = self.robot.get_current_posx()
        home = np.asarray(home)
        approach = np.r_[self.settings['pickup_approach_base_mm'], home[3:]]
        grasp = np.asarray(self.settings['pickup_grasp_base_mm_deg'])
        target = grasp.copy()
        if release:
            target[2] += self.settings['release_z_offset_mm']
        above_target = np.r_[target[:2], approach[2], target[3:]]
        near_target = target + [0., 0., self.NEAR_GRASP_MM, 0., 0., 0.]
        lift = grasp + [0., 0., self.settings['pickup_lift_mm'], 0., 0., 0.]
        above_home = np.r_[home[:2], lift[2], home[3:]]
        for point in (approach, above_target, near_target, target, lift, above_home, home):
            joints = self.robot.ikin(point, solution)
            if joints is None or not np.isfinite(joints).all():
                raise RuntimeError(f'탐침 {action} 경유점의 역기구학 해가 없습니다: {point[:3].tolist()} mm')

        self.probe.tare(samples=3)
        if not release:
            self.gripper(opening=True)
        self.log.info(f'HOME → 탐침 거치대 접근점 → {action} 위치 (Z {target[2]:.2f} mm)')
        self.move(approach, self.TRANSFER_SPEED)
        self.move(above_target, 16.)
        self.move(near_target, 20.)
        self.move(target, 6., at_fixture=not release)
        self.gripper(opening=release)

        self.log.info(f'{action} 후 수직 상승하여 HOME 복귀')
        self.move(near_target, 6., at_fixture=not release)
        if lift[2] > above_target[2] + 2. * self.TRANSFER_RADIUS_MM:
            self.move(above_target, 20.)
            route = [lift, above_home, home]
        else:
            self.move(lift, 20.)
            route = [above_home, home]
        poses = [(point[:3], rotation_zyz(point[3:])) for point in route]
        if self.probe.blend(poses, self.TRANSFER_SPEED, self.TRANSFER_RADIUS_MM,
                            until=lambda _, wrench: np.linalg.norm(wrench[:3]) > self.probe.max_force,
                            acc=50., angular_speed=40., angular_acc=120.):
            raise RuntimeError(f'탐침 HOME 복귀 중 반력 한계 초과 ({self.probe.max_force:g} N)')
        self.log.info(f'탐침 {action} 동작 완료. HOME 도착.')
