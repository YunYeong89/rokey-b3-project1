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
  ros2 run repair_car teleop [--web]"""
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

from ..config import PACKAGE, read_session
from .program import RobotProgram
from .robot_bridge import program_args, robot_topic
from repair_car.inspection.scan_scene import ScanScene


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
            print('\rGripper state not received yet', end='', flush=True)
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
        print(f'\rGripper {"open" if opening else "close"}: target {self.target:.3g}{" mm" if self.real else " rad"}     ',
              end='', flush=True)


class Teleop:
    KEYS = {'w': (1, 0, 0), 's': (-1, 0, 0), 'a': (0, 1, 0), 'd': (0, -1, 0), 'r': (0, 0, 1), 'f': (0, 0, -1)}
    SPEEDS = {'1': 2., '2': 5., '3': 15., '4': 30., '5': 50.}  # mm/s of a jog
    JOG_FULL_MM_S = 250. * 1.73  # jog_multi speed 100 % (dsr_msgs2 JogMulti)
    FLOOR_MM = 5.  # a jog down stops this high above the table
    HELP = ('user mode: w/s +x/-x, a/d +y/-y, r/f +z/-z (BASE, moves while held), o/c gripper open/close (while held), '
            '1/2/3/4/5 speed 2/5/15/30/50 mm/s, p print the current pose, m back to mode selection, q quit')

    def __init__(self, program):
        self.program, self.robot, self.probe = program, program.robot, program.probe
        self.speed, self.gripper = 5., None

    def release(self):
        """Refuse while another program moves the robot; force and compliance left on released (RobotProgram.home and
        program.close: probe.close)."""
        if self.robot.check_motion() != 0:
            raise RuntimeError('The robot is moving: stop the running program first')
        if self.robot.get_control_mode() == 4:
            self.probe.compliant = self.probe.forcing = True
            self.probe.close()
            self.program.log('Force control and compliance released')

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
        p.log(f'{label} move: TCP {pose[:3].round(1).tolist()} mm -> joints {list(joints)} deg, travel height {p.safe_z():.1f} mm')
        p.home(joints)
        p.log(f'{label} reached')

    def jog(self, keys, key):
        """Velocity jog along the key's BASE direction while the key repeats; move_stop once it stops repeating, on a web
        release ("") or another key (-> that key, handled next)."""
        direction, floor = np.array(self.KEYS[key], float), self.program.config['vehicle_zone']['table_z_mm'] + self.FLOOR_MM
        if direction[2] < 0 and self.probe.read()[0][2] < floor:
            print(f'\rNot going below {self.FLOOR_MM:g} mm above the table     ', end='', flush=True)
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
                print(f'\rSpeed {self.speed:g} mm/s     ', end='', flush=True)
            elif key in ('o', 'c'):
                self.gripper.step(key == 'o')
            elif key == 'p':
                pose, _ = self.probe.read()
                joints = self.robot.get_current_posj()
                print(f'\nTCP {pose.round(2).tolist()} (mm, deg), joints {np.round(joints, 2).tolist()} deg')
            elif key in self.KEYS:
                key = self.jog(keys, key)
                continue
            key = None

    def run(self):
        while True:
            mode = input('\nControl mode (user: direct keyboard control, anything else: move to a pose, q: quit) > ').strip().lower()
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
                text = input(f'Pose name {list(book.poses)} or 6 joint angles (deg), q: quit > ').strip()
                if text == 'q':
                    return
                joints = book.joints(text)
                if joints is None:
                    print(f'Unknown pose: {text!r} ({PoseBook.PATH})')
                    continue
                self.go(joints, text)
            except (RuntimeError, TimeoutError) as error:
                print(f'\nFailed: {error}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--web', action='store_true', help='user mode from the web teleop popup (/robot/teleop)')
    options = parser.parse_args(program_args())
    program = RobotProgram('teleop')
    try:
        if options.web:
            program.log('Web teleop: waiting for /robot/teleop button input (moves only while held)')
            Teleop(program).user(WebKeys(program))  # until the web's stop (SIGINT)
        else:
            Teleop(program).run()
    finally:
        program.close()


if __name__ == '__main__':
    main()
