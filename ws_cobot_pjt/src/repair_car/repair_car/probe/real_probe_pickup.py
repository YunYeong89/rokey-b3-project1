"""Position-controlled pickup and release at the taught real-robot rack."""
import time

import numpy as np
from dsr_msgs2.srv import SetCtrlBoxDigitalOutput

if __package__:
    from repair_car.config import load
    from repair_car.robot_control.contact_probe import ContactProbe
    from repair_car.robot_control.home_motion import HomeMotion
    from repair_car.geometry.real_vehicle import RealVehicle
    from repair_car.robot_control.rg2_state import RG2State
    from repair_car.robot_control.rotations import rotation_zyz
else:
    from config import load
    from repair_car.robot_control.contact_probe import ContactProbe
    from repair_car.robot_control.home_motion import HomeMotion
    from geometry.real_vehicle import RealVehicle
    from robot_control.rg2_state import RG2State
    from robot_control.rotations import rotation_zyz


class RealProbePickup:
    NEAR_GRASP_MM = 6.
    TRANSFER_SPEED = 55.
    TRANSFER_RADIUS_MM = 5.

    def __init__(self, robot, settings, home_joints, *, gripper=None, verify_position=True):
        self.robot, self.settings, self.home_joints = robot, settings, home_joints
        self.verify_position = verify_position
        self.tool_name = settings['tool_weight_name'] or load('robot')['probe']['tool_name']
        self.log = robot.node.get_logger()
        self.probe = ContactProbe(robot, force=0.)
        self.feedback = gripper if gripper is not None else RG2State(robot.node, settings['held_width_mm'])

    def spin(self):
        self.robot.check_cancelled()
        self.robot.executor.spin_once(timeout_sec=.05)

    def gripper(self, *, opening):
        off, on = (1, 2) if opening else (2, 1)
        service = 'io/set_ctrl_box_digital_output'
        # Match DSR_ROBOT2's OFF=0 and ON=1, switching the opposite output off first.
        self.robot.request(service, SetCtrlBoxDigitalOutput, index=off, value=0)
        self.robot.request(service, SetCtrlBoxDigitalOutput, index=on, value=1)
        check = self.feedback.opened if opening else self.feedback.held
        action = 'gripper open' if opening else 'probe grasp'
        self.feedback.wait(check, (True,), self.spin, f'{action} check failed', after=time.monotonic())

    def monitor(self, *, holding):
        check = self.feedback.held if holding else self.feedback.opened
        action = 'probe grasp' if holding else 'gripper open'

        def until(_, wrench):
            if check(settled=False) is not True:
                self.robot.stop()
                raise RuntimeError(f'Lost {action} state while moving at the rack ({self.feedback.description()})')
            return False

        return until

    def move(self, pose, speed, *, until):
        until(None, None)
        rotation = rotation_zyz(pose[3:])
        self.probe.move(pose[:3], rotation, speed, until=until, acc=50., angular_speed=40., angular_acc=120.)
        if self.verify_position:
            reached, _ = self.robot.get_current_posx()
            if (np.linalg.norm(np.subtract(reached[:3], pose[:3])) > .5
                    or self.probe.turn_deg(reached[3:], rotation) > .5):
                raise RuntimeError(f'Probe rack target not reached: {pose[:3].tolist()} mm')

    def ensure_home(self, action):
        if self.robot.get_robot_mode() != 1 or self.robot.get_control_mode() != 3 or self.robot.check_motion() != 0:
            raise RuntimeError(f'Run probe {action} in auto mode, stopped under position control.')
        if np.max(np.abs(np.subtract(self.robot.get_current_posj(), self.home_joints))) <= HomeMotion.JOINT_TOLERANCE_DEG:
            return
        self.log.info(f'Probe {action} preparation: returning HOME, then continuing.')
        config = load('robot')
        vehicle = load('real_scene')['vehicle']
        zone = config['vehicle_zone']
        travel_z = max(zone['table_z_mm'] + zone['max_height_mm'],
                       vehicle['base_z_mm'] + RealVehicle().size[2]) + config['scan']['clearance_mm']
        HomeMotion(self.robot, self.probe, config, travel_z).run(self.home_joints)

    def ensure(self, *, allow_pickup=False, report=None):
        expected = (True, False) if allow_pickup else (True,)
        try:
            held = self.feedback.wait(self.feedback.held, expected, self.spin, 'Probe grasp check before repair failed',
                                      after=time.monotonic())
        except RuntimeError:
            if not allow_pickup or self.feedback.width is None:
                raise   # no RG2 feedback: the gripper cannot be driven blind
            held = False   # an unproven grip (part open, another width): not the probe, picked from the rack
        if not held:
            if report:
                report('Starting the automatic grasp at the probe rack')
            self.run()
        else:
            self.ensure_home('check')
        self.robot.verify_profiles(self.tool_name)
        if report:
            report('RG2 probe grasp confirmed')

    def run(self, *, release=False):
        action = 'release' if release else 'grasp'
        self.ensure_home(action)
        self.robot.verify_profiles(self.tool_name, self.settings['pickup_tcp_name'])
        if release:   # a grasp opens the jaws below, whatever they hold
            self.feedback.wait(self.feedback.held, (True,), self.spin, f'Grasp state mismatch before probe {action}')

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
        if self.verify_position:
            for point in (approach, above_target, near_target, target, lift, above_home, home):
                joints = self.robot.ikin(point, solution)
                if joints is None or not np.isfinite(joints).all():
                    raise RuntimeError(f'No inverse kinematics solution for the probe {action} waypoint: {point[:3].tolist()} mm')

        if release:
            self.probe.tare(samples=3)
        else:
            self.gripper(opening=True)
        until = self.monitor(holding=release)
        self.log.info(f'HOME → probe rack approach point → {action} position (Z {target[2]:.2f} mm)')
        self.move(approach, self.TRANSFER_SPEED, until=until)
        self.move(above_target, 16., until=until)
        self.move(near_target, 20., until=until)
        self.move(target, 6., until=until)
        self.gripper(opening=release)

        until = self.monitor(holding=not release)
        self.log.info(f'After {action}, rising vertically and returning HOME')
        self.move(near_target, 6., until=until)
        if lift[2] > above_target[2] + 2. * self.TRANSFER_RADIUS_MM:
            self.move(above_target, 20., until=until)
            route = [lift, above_home, home]
        else:
            self.move(lift, 20., until=until)
            route = [above_home, home]
        poses = [(point[:3], rotation_zyz(point[3:])) for point in route]
        until(None, None)
        self.probe.blend(poses, self.TRANSFER_SPEED, self.TRANSFER_RADIUS_MM, until=until,
                         acc=50., angular_speed=40., angular_acc=120., verify_endpoint=self.verify_position)
        check = self.feedback.opened if release else self.feedback.held
        self.feedback.wait(check, (True,), self.spin, f'Probe {action} state check after arriving HOME failed',
                           after=time.monotonic())
        self.log.info(f'Probe {action} done. Arrived HOME.')
