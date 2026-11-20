"""Connection check of the robot, real or simulated: HOME, then the RG2 closed and opened (OnRobot /onrobot/sendCommand
'c' / 'o'), with the TCP pose, the external force and the finger angle logged at each step. The gripper part is refused
while the probe TCP is selected: opening the gripper would drop the probe.
  ros2 run rokey robot_demo [--cycles N] [--no-gripper]        (web: 연결 확인 데모)
"""
import argparse
from .robot_namespace import program_args
import time

from onrobot_rg_msgs.srv import SetCommand
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .program import RobotProgram

FINGER = 'rg2_finger_joint'  # /dsr01/gripper_joint_states, rad: open -0.56, closed 0.79
MIN_S, STILL_S, TIMEOUT_S = 1.5, .5, 5.0  # per gripper command: at least MIN_S, done once still for STILL_S


class RobotDemo:
    def __init__(self, program):
        self.program, self.robot = program, program.robot
        self.angle = None
        program.node.create_subscription(JointState, 'gripper_joint_states', self.joints, qos_profile_sensor_data)

    def joints(self, message):
        if FINGER in message.name:
            self.angle = message.position[message.name.index(FINGER)]

    @staticmethod
    def rad(value):
        return '미수신' if value is None else f'{value:+.3f}'

    def report(self, label):
        pose, _ = self.robot.get_current_posx()
        force = self.robot.get_tool_force()
        self.program.log(f'{label}: TCP [{", ".join(f"{v:.1f}" for v in pose)}] mm/deg, '
                         f'외력 [{", ".join(f"{v:.2f}" for v in force[:3])}] N, 그리퍼 {self.rad(self.angle)} rad')

    def gripper(self, command, label):
        before = self.angle
        self.robot.request('/onrobot/sendCommand', SetCommand, command=command)
        sent = changed = time.monotonic()
        last = self.angle
        while time.monotonic() - sent < TIMEOUT_S:
            self.robot.executor.spin_once(timeout_sec=.05)
            self.robot.check_cancelled()
            now = time.monotonic()
            if self.angle is not None and (last is None or abs(self.angle - last) > 1e-3):
                last, changed = self.angle, now
            if now - sent > MIN_S and now - changed > STILL_S:
                break
        note = (' (그리퍼 상태 미수신: 동작을 눈으로 확인하세요)' if self.angle is None else
                ' (변화 없음: 이미 그 상태였거나 그리퍼가 응답하지 않았습니다)'
                if before is not None and abs(self.angle - before) < .05 else '')
        self.program.log(f'{label}: {FINGER} {self.rad(before)} -> {self.rad(self.angle)} rad{note}')

    def run(self, cycles, gripper):
        p, r = self.program, self.robot
        tcp = r.get_tcp()
        p.log(f'{p.mode} 로봇 연결: Tool={r.get_tool()}, TCP={tcp}')
        if gripper and tcp == p.config['probe']['tcp_name']:
            raise RuntimeError(f'탐침 TCP({tcp})가 선택되어 있습니다. 탐침을 잡은 채 그리퍼를 열면 떨어지므로, '
                               '탐침을 내려놓거나 HOME만 확인하세요 (--no-gripper)')
        r.executor.spin_once(timeout_sec=.2)  # the first gripper state
        self.report('시작')
        p.log('1. HOME 이동')
        p.home()
        self.report('HOME')
        for cycle in range(1, cycles + 1) if gripper else []:
            self.gripper('c', f'2.{cycle} 그리퍼 닫기')
            self.gripper('o', f'3.{cycle} 그리퍼 열기')
        self.report('완료')
        p.log('연결 확인 완료')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--cycles', type=int, default=1, help='close/open repetitions (default: 1)')
    parser.add_argument('--no-gripper', action='store_true', help='HOME only')
    options = parser.parse_args(program_args())
    program = RobotProgram('robot_demo')
    try:
        RobotDemo(program).run(max(1, options.cycles), not options.no_gripper)
    finally:
        program.close()


if __name__ == '__main__':
    main()
