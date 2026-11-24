"""Pick up the probe from HOME and return to HOME.

The real robot uses the taught points in real_scene.yaml, and the virtual robot uses its session plan.
  ros2 run repair_car pick_probe [--plan VIRTUAL_PLAN]
"""
import argparse

if __package__:
    from repair_car.robot_control.robot_bridge import program_args
else:
    from robot_control.robot_bridge import program_args
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from dsr_msgs2 import srv
from onrobot_rg_msgs.srv import SetCommand
from std_msgs.msg import String
from std_srvs.srv import Trigger

if __package__:
    from repair_car.config import SESSION, load
    from repair_car.probe.real_probe_pickup import RealProbePickup
    from repair_car.robot_control.robot_bridge import RobotServices
    from repair_car.probe.virtual_probe_profiles import VirtualProbeProfiles
else:
    from config import SESSION, load
    from repair_car.probe.real_probe_pickup import RealProbePickup
    from robot_control.robot_bridge import RobotServices
    from repair_car.probe.virtual_probe_profiles import VirtualProbeProfiles

SESSION_PLAN = SESSION / 'probe_plan.json'


class ProbePickup:
    """HOME -> above the docked tip with the gripper aligned to it -> straight down -> grasp -> lift -> HOME."""

    GEOMETRY_REJECTED = ('Grasp alignment rejected', 'fingertip is not seated', 'fingertip extends outside')
    ATTACH_SETTLE_S = .25
    MAX_RESEAT_CORRECTION_MM = 1.

    def __init__(self, node, robot, plan, motion):
        self.node, self.robot, self.plan, self.motion = node, robot, plan, motion
        self.profiles = VirtualProbeProfiles(robot, plan)
        self.status, self.status_at = {}, 0.
        self.sub = node.create_subscription(String, 'virtual_control/status', self.receive, 10)

    def receive(self, message):
        self.status = json.loads(message.data)
        self.status_at = time.monotonic()

    def wait_ready(self, *, after=0., attached=None):
        deadline = time.monotonic()+30
        while time.monotonic()<deadline:
            self.robot.executor.spin_once(timeout_sec=.1)
            self.robot.check_cancelled()
            if (self.status_at > after and self.status.get('ready') and self.status.get('settled') and
                    (attached is None or self.status.get('probe_attached') is attached)):
                return
        if attached is not None:
            raise TimeoutError('Virtual probe grasp or docked state check timed out')
        raise TimeoutError('Virtual physics/force feedback did not become ready')

    def gripper(self, angle):
        self.robot.request('/onrobot/sendCommand', SetCommand, command=str(angle))
        after = time.monotonic()
        deadline = after + 5
        while time.monotonic() < deadline:
            self.robot.executor.spin_once(timeout_sec=.05)
            self.robot.check_cancelled()
            current = self.status.get('gripper_angle_rad')
            if self.status_at > after and current is not None and abs(current-angle) < .03:
                return
        raise TimeoutError(f'RG2 did not reach opening: target={angle:.3f}, actual={current}')

    def attach(self, grasp):
        service = self.node.get_namespace() + '/probe/attach'
        previous = self.robot.timeout
        self.robot.timeout = 12.
        try:
            for attempt in range(2):
                self.wait_ready(after=time.monotonic() + self.ATTACH_SETTLE_S)
                try:
                    self.robot.request(service, Trigger)
                    return
                except RuntimeError as error:
                    # Geometry rejections occur before the Gazebo constraints change.
                    if attempt or not any(reason in str(error) for reason in self.GEOMETRY_REJECTED):
                        raise
                    self.node.get_logger().warning(f'Rechecking the virtual grasp alignment: {error}')
                    self.reseat(grasp)
        finally:
            self.robot.timeout = previous

    def reseat(self, grasp):
        reached, _ = self.robot.get_current_posx()
        correction = [target - actual for target, actual in zip(grasp[:3], reached[:3])]
        distance = math.dist(grasp[:3], reached[:3])
        if distance > 2.:
            raise RuntimeError(f'The virtual probe grasp is {distance:.2f} mm off the target.')
        scale = self.MAX_RESEAT_CORRECTION_MM / max(distance, self.MAX_RESEAT_CORRECTION_MM)
        target = [*(value + offset * scale for value, offset in zip(grasp[:3], correction)), *grasp[3:]]
        lift = RealProbePickup.NEAR_GRASP_MM
        acc = self.motion['approach_acc']
        self.gripper(self.plan['open_angle_rad'])
        self.move([*reached[:2], reached[2] + lift, *grasp[3:]], 6., acc)
        self.move([*target[:2], target[2] + lift, *target[3:]], 6., acc)
        self.move(target, 6., acc)
        self.gripper(self.plan['close_angle_rad'])

    def move(self, pose, speed, acc):
        current, _ = self.robot.get_current_posx()
        distance = math.dist(current[:3], pose[:3])
        timeout = distance / speed + 5.
        self.node.get_logger().info(f'movel {distance:.1f} mm at {speed:.0f} mm/s')
        self.robot.amovel([float(x) for x in pose], [speed, self.motion['angular_speed']],
                          [acc, self.motion['angular_acc']])
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            time.sleep(.05)
            if self.robot.check_motion()==0:return
        raise TimeoutError('Probe pickup Cartesian motion timed out')

    def attachment_state(self):
        if self.robot.get_robot_system() != 1:
            raise RuntimeError('The virtual probe state can only be checked on the virtual robot.')
        self.wait_ready()
        attached = self.status.get('probe_attached')
        if type(attached) is not bool:
            raise RuntimeError('Cannot check the virtual probe grasp state.')
        return attached

    def wait_automatic(self):
        deadline = time.monotonic() + 30
        while self.robot.get_robot_mode() != 1:
            self.robot.check_cancelled()
            if time.monotonic() > deadline:
                raise TimeoutError('The virtual robot did not switch to auto mode.')
            self.robot.executor.spin_once(timeout_sec=.1)

    def require_attached(self):
        if not self.attachment_state():
            raise RuntimeError('Confirm the probe grasp in Gazebo before inspecting.')
        self.wait_automatic()
        self.profiles.select(attached=True)

    def run(self, *, require_empty=False):
        attached = self.attachment_state()
        self.wait_automatic()
        if attached:
            if require_empty:
                raise RuntimeError('Place the probe in the rack to start the automatic repair.')
            self.profiles.select(attached=True)
            self.node.get_logger().info('Virtual probe grasp and TCP/Tool settings confirmed.')
            return
        self.profiles.select(attached=False)
        m = self.motion
        home = (self.plan['home_joints_deg'], m['joint_speed'], m['joint_acc'])
        self.node.get_logger().info('1/7 HOME')
        self.robot.movej(*home)
        self.node.get_logger().info('2/7 OPEN RG2')
        self.gripper(self.plan['open_angle_rad'])
        grasp = self.plan['grasp_pose_mm_deg']
        measured = self.plan.get('pickup_approach_mm_deg')
        lift = self.plan.get('pickup_lift_mm', m['approach_height_mm'])
        z = measured[2] if measured else grasp[2] + m['approach_height_mm']
        self.node.get_logger().info(f'3/7 ABOVE THE TIP, gripper aligned (grip axis {self.plan["grip_axis"]})')
        if measured:
            self.move(measured, RealProbePickup.TRANSFER_SPEED, m['linear_acc'])
        self.move([*grasp[:2], z, *grasp[3:]], 16. if measured else m['linear_speed'], m['linear_acc'])
        self.node.get_logger().info('4/7 DESCEND ALONG -Z')
        if measured:
            self.move([*grasp[:2], grasp[2] + RealProbePickup.NEAR_GRASP_MM, *grasp[3:]], 20., m['approach_acc'])
        self.move(grasp, 6. if measured else m['approach_speed'], m['approach_acc'])
        self.node.get_logger().info('5/7 CLOSE RG2')
        self.gripper(self.plan['close_angle_rad'])
        self.node.get_logger().info('5/7 VERIFY GAZEBO GRASP AND RELEASE RACK')
        self.attach(grasp)
        self.plan.update(json.loads(SESSION_PLAN.read_text()))
        self.node.get_logger().info('6/7 SELECT PROBE TCP/TOOL')
        self.profiles.select(attached=True, refresh=True)
        self.wait_ready(after=time.monotonic(), attached=True)
        self.node.get_logger().info('7/7 LIFT ALONG +Z AND RETURN HOME')
        current, _ = self.robot.get_current_posx()
        self.move([*current[:2], current[2] + lift, *current[3:]], 20. if measured else m['approach_speed'], m['approach_acc'])
        self.robot.movej(*home)
        self.wait_ready(after=time.monotonic(), attached=True)
        self.node.get_logger().info('PROBE PICKED; READY FROM HOME')

    def dock(self):
        attached = self.attachment_state()
        self.wait_automatic()
        self.profiles.select(attached=attached)
        if not attached:
            return
        m = self.motion
        home = (self.plan['home_joints_deg'], m['joint_speed'], m['joint_acc'])
        target = self.plan['dock_probe_tcp_mm_deg']
        gap = self.plan.get('release_z_offset_mm', RealProbePickup.NEAR_GRASP_MM)
        approach = self.plan.get('pickup_approach_mm_deg')
        lift = approach[2] - self.plan['grasp_pose_mm_deg'][2] if approach else m['approach_height_mm']
        self.robot.movej(*home)
        self.move([*target[:2], target[2] + lift, *target[3:]], RealProbePickup.TRANSFER_SPEED, m['linear_acc'])
        self.move([*target[:2], target[2] + gap, *target[3:]], 20., m['approach_acc'])
        self.move(target, 6., m['approach_acc'])
        self.wait_ready(after=time.monotonic())
        previous = self.robot.timeout
        self.robot.timeout = 12.
        try:
            self.robot.request(self.node.get_namespace() + '/probe/dock', Trigger)
        finally:
            self.robot.timeout = previous
        self.gripper(self.plan['open_angle_rad'])
        self.profiles.select(attached=False)
        self.wait_ready(after=time.monotonic(), attached=False)
        current, _ = self.robot.get_current_posx()
        self.move([*current[:2], current[2] + lift, *current[3:]], 20., m['approach_acc'])
        self.robot.movej(*home)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--plan', type=Path, help='probe placement plan file of the virtual robot')
    args = parser.parse_args(program_args())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('pick_probe', namespace='dsr01')
    config = load('robot')
    robot = RobotServices(node, config['robot']['service_timeout'])
    try:
        if robot.get_robot_system() == 0:
            if args.plan is not None:
                parser.error('--plan is only available on the virtual robot')
            RealProbePickup(robot, load('real_scene')['probe'], config['robot']['home_joints_deg']).run()
        else:
            path = args.plan or SESSION_PLAN
            if not path.exists():
                parser.error(f'{path} missing: start a virtual session with ./run_virtual.sh --car sedan')
            plan = json.loads(path.read_text())
            if not plan.get('simulation_only'):
                parser.error('The virtual robot needs a probe placement plan')
            ProbePickup(node, robot, plan, config['motion']).run()
    except (RuntimeError, TimeoutError) as error:
        node.get_logger().error(str(error))
        raise
    finally:
        try:
            if robot.motion_active:
                robot.stop()
        finally:
            robot.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
