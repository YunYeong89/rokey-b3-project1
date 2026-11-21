"""Pick up the probe from HOME and return to HOME.

The real robot uses the taught points in real_scene.yaml, and the virtual robot uses its session plan.
  ros2 run rokey pick_probe [--plan VIRTUAL_PLAN]
"""
import argparse

if __package__:
    from .robot_namespace import program_args
else:
    from robot_namespace import program_args
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
    from .real_probe_pickup import RealProbePickup
    from .robot_services import RobotServices
else:
    from configs import SESSION, load
    from real_probe_pickup import RealProbePickup
    from robot_services import RobotServices

SESSION_PLAN = SESSION / 'probe_plan.json'


class ProbePickup:
    """HOME -> above the docked tip with the gripper aligned to it -> straight down -> grasp -> lift -> HOME."""

    GEOMETRY_REJECTED = ('Grasp alignment rejected', 'fingertip is not seated', 'fingertip extends outside')
    ATTACH_SETTLE_S = .25

    def __init__(self, node, robot, plan, motion):
        self.node, self.robot, self.plan, self.motion = node, robot, plan, motion
        self.status, self.status_at = {}, 0.
        self.sub = node.create_subscription(String, 'virtual_control/status', self.receive, 10)

    def receive(self, message):
        self.status = json.loads(message.data)
        self.status_at = time.monotonic()

    def wait_ready(self, *, after=0.):
        deadline = time.monotonic()+30
        while time.monotonic()<deadline:
            self.robot.executor.spin_once(timeout_sec=.1)
            if self.status_at > after and self.status.get('ready') and self.status.get('settled'):
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

    def attach(self):
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
                    self.node.get_logger().warning(f'가상 파지 정렬 재확인: {error}')
        finally:
            self.robot.timeout = previous

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
        if self.status.get('probe_attached'):
            if (self.robot.get_tcp(), self.robot.get_tool()) != (self.plan['tcp_name'], self.plan['tool_name']):
                raise RuntimeError('가상 탐침이 이미 고정됐지만 탐침 TCP·Tool이 적용되지 않았습니다.')
            self.node.get_logger().info('가상 탐침 파지 및 TCP·Tool 적용 상태를 확인했습니다.')
            return
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
        self.node.get_logger().info('5/7 CLOSE RG2')
        self.gripper(self.plan['close_angle_rad'])
        self.node.get_logger().info('5/7 VERIFY GAZEBO GRASP AND RELEASE RACK')
        self.attach()
        self.plan.update(json.loads(SESSION_PLAN.read_text()))
        self.node.get_logger().info('6/7 SELECT PROBE TCP/TOOL')
        mode=self.robot.get_robot_mode()
        self.robot.request('system/set_robot_mode',srv.SetRobotMode,robot_mode=0)
        try:
            self.robot.request('tcp/config_delete_tcp', srv.ConfigDeleteTcp, name=self.plan['tcp_name'])
            self.robot.request('tcp/config_create_tcp', srv.ConfigCreateTcp,
                               name=self.plan['tcp_name'], pos=self.plan['tcp_mm_deg'])
            self.robot.request('tool/config_delete_tool', srv.ConfigDeleteTool, name=self.plan['tool_name'])
            self.robot.request('tool/config_create_tool', srv.ConfigCreateTool,
                               name=self.plan['tool_name'], **self.plan['tool_profile'])
            self.robot.set_tool(self.plan['tool_name'])
            self.robot.set_tcp(self.plan['tcp_name'])
        finally:self.robot.request('system/set_robot_mode',srv.SetRobotMode,robot_mode=mode)
        self.node.get_logger().info('7/7 LIFT ALONG +Z AND RETURN HOME')
        current, _ = self.robot.get_current_posx()
        self.move([*current[:2], current[2] + m['approach_height_mm'], *current[3:]], m['approach_speed'], m['approach_acc'])
        self.robot.movej(*home)
        self.node.get_logger().info('PROBE PICKED; READY FROM HOME')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--plan', type=Path, help='가상 로봇의 탐침 배치 계획 파일')
    args = parser.parse_args(program_args())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('pick_probe', namespace='dsr01')
    config = load('robot')
    robot = RobotServices(node, config['robot']['service_timeout'])
    try:
        if robot.get_robot_system() == 0:
            if args.plan is not None:
                parser.error('--plan은 가상 로봇에서만 사용할 수 있습니다')
            RealProbePickup(robot, load('real_scene')['probe'], config['robot']['home_joints_deg']).run()
        else:
            path = args.plan or SESSION_PLAN
            if not path.exists():
                parser.error(f'{path} 없음: ./run_virtual.sh --car sedan으로 가상 세션을 시작하세요')
            plan = json.loads(path.read_text())
            if not plan.get('simulation_only'):
                parser.error('가상 로봇용 탐침 배치 계획이 필요합니다')
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
