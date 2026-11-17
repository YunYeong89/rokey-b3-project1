"""Virtual-only probe pickup from the rack of the session plan: home -> above the tip -> down -> grasp ->
probe TCP/tool -> lift -> home. repair_manager runs the same pickup when the probe is not held.

Run inside the container while ./run_virtual.sh --car <sedan|suv|sports|random> is up:
  ros2 run rokey pick_probe
"""
import argparse
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
    from .configs import SESSION, load
    from .robot_services import RobotServices
else:
    from configs import SESSION, load
    from robot_services import RobotServices

SESSION_PLAN = SESSION / 'probe_plan.json'


class ProbePickup:
    """HOME -> above the docked tip with the gripper aligned to it -> straight down -> grasp -> lift -> HOME."""

    def __init__(self, node, robot, plan, motion):
        self.node, self.robot, self.plan, self.motion = node, robot, plan, motion
        self.status = {}
        self.sub = node.create_subscription(String, '/dsr01/virtual_control/status', self.receive, 10)

    def receive(self, message):
        self.status = json.loads(message.data)

    def wait_ready(self):
        deadline = time.monotonic()+30
        while time.monotonic()<deadline:
            self.robot.executor.spin_once(timeout_sec=.1)
            if self.status.get('ready') and self.status.get('settled'):
                return
        raise TimeoutError('Virtual physics/force feedback did not become ready')

    def gripper(self, angle):
        self.robot.request('/onrobot/sendCommand', SetCommand, command=str(angle))
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            self.robot.executor.spin_once(timeout_sec=.05)
            current = self.status.get('gripper_angle_rad')
            if current is not None and abs(current-angle) < .03:
                return
        raise TimeoutError(f'RG2 did not reach opening: target={angle:.3f}, actual={current}')

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

    def run(self):
        self.wait_ready()
        if self.robot.request('system/get_robot_system',srv.GetRobotSystem).robot_system != 1:
            raise RuntimeError('Probe pickup is restricted to the virtual robot')
        initial = self.plan['initial_profiles']
        deadline = time.monotonic() + 30
        while (self.robot.get_robot_mode() != 1 or self.robot.get_tcp() != initial['tcp']
               or self.robot.get_tool() != initial['tool']):
            if time.monotonic() > deadline:
                raise TimeoutError('Waiting for virtual TCP/tool profiles and automatic mode')
            self.robot.executor.spin_once(timeout_sec=.1)
        m = self.motion
        home = (self.plan['home_joints_deg'], m['joint_speed'], m['joint_acc'])
        self.node.get_logger().info('1/7 HOME')
        self.robot.movej(*home)
        self.node.get_logger().info('2/7 OPEN RG2')
        self.gripper(self.plan['open_angle_rad'])
        grasp = self.plan['grasp_pose_mm_deg']
        self.node.get_logger().info(f'3/7 ABOVE THE TIP, gripper aligned (grip axis {self.plan["grip_axis"]})')
        self.move([*grasp[:2], grasp[2] + m['approach_height_mm'], *grasp[3:]], m['linear_speed'], m['linear_acc'])
        self.node.get_logger().info('4/7 DESCEND ALONG -Z')
        self.move(grasp, m['approach_speed'], m['approach_acc'])
        self.node.get_logger().info('5/7 CLOSE RG2 AND VERIFY GRASP')
        self.gripper(self.plan['close_angle_rad'])
        self.wait_ready()
        previous=self.robot.timeout;self.robot.timeout=12.
        try:self.robot.request('/dsr01/probe/attach',Trigger)
        finally:self.robot.timeout=previous
        self.node.get_logger().info('6/7 SELECT PROBE TCP/TOOL')
        mode=self.robot.get_robot_mode()
        self.robot.request('system/set_robot_mode',srv.SetRobotMode,robot_mode=0)
        try:
            self.robot.set_tool(self.plan['tool_name'])
            self.robot.set_tcp(self.plan['tcp_name'])
        finally:self.robot.request('system/set_robot_mode',srv.SetRobotMode,robot_mode=mode)
        self.node.get_logger().info('7/7 LIFT ALONG +Z AND RETURN HOME')
        current, _ = self.robot.get_current_posx()
        self.move([*current[:2], current[2] + m['approach_height_mm'], *current[3:]], m['approach_speed'], m['approach_acc'])
        self.robot.movej(*home)
        self.node.get_logger().info('PROBE PICKED; READY FROM HOME')


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--plan',type=Path,default=SESSION_PLAN)
    args=parser.parse_args()
    if not args.plan.exists():parser.error(f'{args.plan} 없음: ./run_virtual.sh --car sedan (suv, sports, random) 으로 세션을 시작하세요')
    plan=json.loads(args.plan.read_text())
    if not plan.get('simulation_only'):parser.error('Expected a simulation-only probe plan')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('pick_probe',namespace='dsr01');robot=RobotServices(node,5.)
    try:
        ProbePickup(node,robot,plan,load('robot')['motion']).run()
    finally:
        try:
            if robot.motion_active:robot.stop()
        finally:robot.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
