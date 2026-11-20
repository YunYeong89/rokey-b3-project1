"""Teleoperation states: HOMING -> GROUNDING (camera side, 5 s) -> TELEOP, HOLD while the hand is missing (ground
again after `reground_after_s`). Keys from the camera window: h HOME at once then PAUSED, s stop in place then
PAUSED, c (while PAUSED) ground in place then TELEOP, q quit."""
import time

from rokey.rotations import rotation_zyz

from .gripper import RG2Gripper
from .mapping import TeleopMapper
from .servo import ServoStream


class TeleopSession:
    def __init__(self, node, robot, link, config):
        self.node, self.robot, self.link, self.config = node, robot, link, config
        self.mapper = TeleopMapper(config['mapping'])
        self.servo = ServoStream(node, config['servol'])
        self.gripper = RG2Gripper(node, robot.get_robot_system() == 1, config['gripper'])
        self.period = 1.0 / config['rate_hz']
        self.ground_id, self.state, self.asked, self.lost = 0, None, 0.0, None

    def status(self, state):
        if state != self.state:
            self.state = state
            self.link.send(status=state)
            self.node.get_logger().info(state)

    def spin(self, seconds):
        self.robot.executor.spin_once(timeout_sec=seconds)

    def settle(self):
        """Wait until the TCP rests; its pose becomes the servol start. BASE position (mm) and rotation matrix."""
        while self.robot.check_motion() != 0:
            time.sleep(0.05)
        pose, _ = self.robot.get_current_posx()
        self.servo.reset(pose[3:])
        return pose[:3], rotation_zyz(pose[3:])

    def home(self):
        """Stop the stream and go HOME at once; HOME becomes the centre of the workspace box."""
        self.status('HOMING')
        self.robot.stop()
        self.robot.set_ref_coord(self.robot.DR_BASE)
        self.robot.movej(self.robot.HOME, self.config['home']['joint_speed'], self.config['home']['joint_acc'])
        self.mapper.reset(*self.settle())

    def pause(self):
        """Stop in place; the target stays where the TCP came to rest."""
        self.robot.stop()
        self.mapper.place(*self.settle())
        self.status('PAUSED')

    def ground(self):
        """Ask the camera side for a new neutral hand at the current TCP pose; `grounded` waits for it."""
        self.ground_id += 1
        self.asked = 0.0
        self.status('GROUNDING')

    def grounded(self):
        """TELEOP once the camera side reports this grounding; the request is repeated every second (reconnects)."""
        if self.link.latest()[0].get('ground_id') == self.ground_id:
            self.mapper.anchor()
            self.lost = None
            self.status('TELEOP')
        elif time.monotonic() - self.asked > 1.0:
            self.link.send(ground=self.ground_id, status=self.state)
            self.asked = time.monotonic()

    def follow(self):
        message, age = self.link.latest()
        if message['tracked'] and message['ground_id'] == self.ground_id and age < self.config['stale_s']:
            self.lost = None
            self.status('TELEOP')
            self.servo.send(*self.mapper.step(message['offset'], message['rotation'], self.period))
            self.gripper.open_to(message['openness'])
        else:
            self.lost = self.lost or time.monotonic()
            self.status('HOLD')
            if time.monotonic() - self.lost > self.config['reground_after_s']:
                self.ground()

    def run(self):
        while not self.link.connected:
            self.node.get_logger().info(f'카메라 연결 대기: {self.link.address}', throttle_duration_sec=5.0)
            self.spin(0.2)
        self.home()
        self.ground()
        tick = time.monotonic()
        while True:
            key = self.link.key()
            if key == 'quit':
                return
            if key == 'home':
                self.home()
                self.status('PAUSED')
            elif key == 'stop' and self.state != 'PAUSED':
                self.pause()
            elif key == 'continue' and self.state == 'PAUSED':
                self.ground()
            elif self.state == 'GROUNDING':
                self.grounded()
            elif self.state != 'PAUSED':
                self.follow()
            tick = max(tick + self.period, time.monotonic())  # no catch-up burst after HOMING
            self.spin(max(tick - time.monotonic(), 0.0))
