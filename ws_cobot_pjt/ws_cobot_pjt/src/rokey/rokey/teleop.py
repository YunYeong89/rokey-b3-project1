"""Teleoperation from the terminal. An input() prompt asks the control mode:
  user   the TCP moved from the keyboard in BASE: w/s +-x, a/d +-y, r/f +-z, a velocity jog (DSR jog_multi, its own
         acceleration) while the key repeats, move_stop once it stops repeating or another key comes; o/c open/close
         the RG2 gradually while held (not fully); 1-5 the speed 2/5/15/30/50 mm/s; p prints the TCP and joints (for
         pose.json); m back to the prompt;
  other  a pose: a name of config/pose.json (home, ...) or six joint angles (deg), reached the safe way: force and
         compliance released when left on, straight up to the travel height above the scene's vehicle (real_scene.yaml
         on the real robot, the session's scene in simulation) when in the work area, across above the target TCP,
         movej (RobotProgram.home).
q quits, at a prompt and in user mode.
--web: user mode only, the keys from the web's teleop popup (/robot/teleop, WebKeys) instead of the keyboard: a held
button repeats its key, its release stops the jog at once (move_stop); the web's stop ends the program (SIGINT).
A jog is only ever ended by move_stop: jog_multi again while jogging (speed 0, another axis) hung the controller's
service on the emulator.
  ros2 run rokey teleop [--web]"""
import argparse
from collections import deque
import json
import os
import re
import select
import sys
import termios
import time
import tty

import numpy as np
from dsr_msgs2 import srv
from onrobot_rg_msgs.srv import SetCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .configs import PACKAGE, read_session
from .program import RobotProgram
from .robot_namespace import program_args, robot_topic
from .scan_scene import ScanScene


class PoseBook:
    """config/pose.json: {name: {"joints_deg": [6 joint angles], "description": ...}}, read again at each use."""
    PATH = PACKAGE / 'config' / 'pose.json'

    def __init__(self):
        self.poses = json.loads(self.PATH.read_text())

    def joints(self, text):
        """A pose name or six joint angles (deg, spaces or commas) -> joints, else None."""
        if text in self.poses:
            return self.poses[text]['joints_deg']
        values = text.replace(',', ' ').split()
        if len(values) == 6 and all(re.fullmatch(r'[-+]?\d+(\.\d*)?', value) for value in values):
            return [float(value) for value in values]
        return None


class Keyboard:
    """Single keys without Enter (cbreak) in user mode, unbuffered so that select sees every key repeat; the terminal
    restored on exit."""
    FIRST_S = .6  # held after a first press until the terminal's key repeat starts (~0.25-0.5 s)
    HELD_S = .15  # then a repeat this recent: still held (key repeat ~30 Hz)

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def read(self, timeout):
        ready, _, _ = select.select([self.fd], [], [], timeout)
        return os.read(self.fd, 1).decode(errors='ignore') if ready else None


class WebKeys:
    """Keys of the web teleop popup (/robot/teleop String JSON {"key": k}, robot_teleop.js): a held button repeats its
    key every 80 ms, its release sends "" (Teleop.jog stops at once)."""
    FIRST_S = HELD_S = .3  # a repeat this recent: still held (80 ms repeats from the press on, network jitter)

    def __init__(self, program):
        self.robot, self.queue = program.robot, deque()
        program.node.create_subscription(String, robot_topic(program.node, '/robot/teleop'), self.receive, 10)

    def receive(self, message):
        self.queue.append(str(json.loads(message.data).get('key', '')))

    def read(self, timeout):
        end = time.monotonic() + timeout
        while not self.queue and time.monotonic() < end:
            self.robot.executor.spin_once(timeout_sec=max(end - time.monotonic(), 0.))
        return self.queue.popleft() if self.queue else None


class Gripper:
    """The RG2 opened or closed a little per key repeat while o/c are held, from its measured opening: on the real
    robot the width command of /onrobot/sendCommand (0.1 mm units) from the finger joint (gripper_joint_states), in
    simulation the finger angle between the probe plan's open and closed angles (virtual_control/status)."""
    STEP_MM = 1.  # per key repeat (~30 Hz)
    WIDTH_MM = (0., 110.)  # RG2
    RESYNC_S = .3  # a pause this long: the next press starts from the measured opening (an object may stop the jaws)

    def __init__(self, program):
        self.robot, self.real = program.robot, program.mode == 'real'
        self.measured = self.target = None
        self.last = 0.
        if self.real:
            program.node.create_subscription(JointState, 'gripper_joint_states', self.joint_state, 10)
        else:
            plan = read_session('probe_plan.json')
            self.angles = plan['open_angle_rad'], plan['close_angle_rad']
            program.node.create_subscription(String, 'virtual_control/status', self.status, 10)

    @staticmethod
    def width_mm(angle):
        """RG2 opening at the finger joint angle (onrobot_rg_control jointValueToWidth)."""
        return 2000. * (np.cos(angle + .76794) * .055 - .0144 + .108505 * np.cos(1.41371))

    def joint_state(self, message):
        self.measured = self.width_mm(message.position[0])

    def status(self, message):
        self.measured = json.loads(message.data).get('gripper_angle_rad')

    def step(self, opening):
        if self.measured is None:
            print('\r그리퍼 상태를 아직 받지 못했습니다', end='', flush=True)
            return
        now = time.monotonic()
        if self.target is None or now - self.last > self.RESYNC_S:
            self.target = self.measured
        self.last = now
        if self.real:
            self.target = float(np.clip(self.target + (self.STEP_MM if opening else -self.STEP_MM), *self.WIDTH_MM))
            command = str(int(round(self.target * 10)))
        else:
            opened, closed = self.angles
            self.target = float(np.clip(self.target + (opened - closed if opening else closed - opened) / 50,
                                        min(self.angles), max(self.angles)))
            command = str(self.target)
        self.robot.request('/onrobot/sendCommand', SetCommand, command=command)
        print(f'\r그리퍼 {"열기" if opening else "닫기"}: 목표 {self.target:.3g}{" mm" if self.real else " rad"}     ',
              end='', flush=True)


class Teleop:
    KEYS = {'w': (1, 0, 0), 's': (-1, 0, 0), 'a': (0, 1, 0), 'd': (0, -1, 0), 'r': (0, 0, 1), 'f': (0, 0, -1)}
    SPEEDS = {'1': 2., '2': 5., '3': 15., '4': 30., '5': 50.}  # mm/s of a jog
    JOG_FULL_MM_S = 250. * 1.73  # jog_multi speed 100 % (dsr_msgs2 JogMulti)
    FLOOR_MM = 5.  # a jog down stops this high above the table
    HELP = ('user 모드: w/s +x/-x, a/d +y/-y, r/f +z/-z (BASE, 누르는 동안 이동), o/c 그리퍼 열기/닫기 (누르는 동안), '
            '1/2/3/4/5 속도 2/5/15/30/50 mm/s, p 현재 자세 출력, m 모드 선택으로, q 종료')

    def __init__(self, program):
        self.program, self.robot, self.probe = program, program.robot, program.probe
        self.speed, self.gripper = 5., None

    def release(self):
        """Refuse while another program moves the robot; force and compliance left on released (RobotProgram.home and
        program.close: probe.close)."""
        if self.robot.check_motion() != 0:
            raise RuntimeError('로봇이 움직이는 중입니다: 실행 중인 프로그램을 먼저 멈추세요')
        if self.robot.get_control_mode() == 4:
            self.probe.compliant = self.probe.forcing = True
            self.probe.close()
            self.program.log('힘제어·컴플라이언스 해제')

    def top_mm(self):
        """Top of the scene's vehicle (BASE mm), 0 without one."""
        p = self.program
        truth = ScanScene.load('real', 'real', holder=False)[0] if p.mode == 'real' else read_session('scene_truth.json')
        return truth['base_z_mm'] + truth['size_mm'][2] if truth else 0.

    def go(self, joints, label):
        p = self.program
        self.release()
        p.top_mm = self.top_mm()
        pose, _ = self.probe.read()
        p.log(f'{label} 이동: TCP {pose[:3].round(1).tolist()} mm -> 관절 {list(joints)} deg, 이동 높이 {p.safe_z():.1f} mm')
        p.home(joints)
        p.log(f'{label} 도착')

    def jog(self, keys, key):
        """Velocity jog along the key's BASE direction while the key repeats; move_stop once it stops repeating, on a web
        release ("") or another key (-> that key, handled next)."""
        direction, floor = np.array(self.KEYS[key], float), self.program.config['vehicle_zone']['table_z_mm'] + self.FLOOR_MM
        if direction[2] < 0 and self.probe.read()[0][2] < floor:
            print(f'\r테이블 위 {self.FLOOR_MM:g} mm 아래로는 내려가지 않습니다     ', end='', flush=True)
            return None
        self.robot.jog_multi([*direction, 0., 0., 0.], 100. * self.speed / self.JOG_FULL_MM_S)
        started = seen = time.monotonic()
        other = None
        try:
            while time.monotonic() - seen <= (keys.FIRST_S if seen == started else keys.HELD_S):
                pressed = keys.read(.02)
                if pressed == key:
                    seen = time.monotonic()
                elif pressed is not None:
                    other = pressed or None
                    break
                pose, _ = self.probe.read()
                print(f'\rTCP {pose[:3].round(1).tolist()} mm, {self.speed:g} mm/s     ', end='', flush=True)
                if direction[2] < 0 and pose[2] < floor:
                    break
        finally:
            self.robot.stop()
        return other

    def user(self, keys):
        """Control from `keys` (Keyboard, WebKeys) until m (-> False) or q (-> True)."""
        self.release()
        self.gripper = self.gripper or Gripper(self.program)
        key = None
        while True:
            self.robot.executor.spin_once(timeout_sec=0.)
            key = key or keys.read(.05)
            if key in ('q', 'm'):
                print()
                return key == 'q'
            if key in self.SPEEDS:
                self.speed = self.SPEEDS[key]
                print(f'\r속도 {self.speed:g} mm/s     ', end='', flush=True)
            elif key in ('o', 'c'):
                self.gripper.step(key == 'o')
            elif key == 'p':
                pose, _ = self.probe.read()
                joints = self.robot.get_current_posj()
                print(f'\nTCP {pose.round(2).tolist()} (mm, deg), 관절 {np.round(joints, 2).tolist()} deg')
            elif key in self.KEYS:
                key = self.jog(keys, key)
                continue
            key = None

    def run(self):
        while True:
            mode = input('\n제어 모드 (user: 키보드 직접 제어, 그 외: 지정 자세로 이동, q: 종료) > ').strip().lower()
            if mode == 'q':
                return
            try:
                if mode == 'user':
                    print(self.HELP)
                    with Keyboard() as keys:
                        if self.user(keys):
                            return
                    continue
                book = PoseBook()
                text = input(f'자세 이름 {list(book.poses)} 또는 관절 6개 (deg), q: 종료 > ').strip()
                if text == 'q':
                    return
                joints = book.joints(text)
                if joints is None:
                    print(f'알 수 없는 자세입니다: {text!r} ({PoseBook.PATH})')
                    continue
                self.go(joints, text)
            except (RuntimeError, TimeoutError) as error:
                print(f'\n실패: {error}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--web', action='store_true', help='user mode from the web teleop popup (/robot/teleop)')
    options = parser.parse_args(program_args())
    program = RobotProgram('teleop')
    try:
        if options.web:
            program.log('웹 teleop: /robot/teleop 버튼 입력 대기 (누르는 동안만 이동)')
            Teleop(program).user(WebKeys(program))  # until the web's stop (SIGINT)
        else:
            Teleop(program).run()
    finally:
        program.close()


if __name__ == '__main__':
    main()
