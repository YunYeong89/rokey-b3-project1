"""Return to the saved preparation pose, then slide BASE +X with TOOL +Z force control.

Units: mm, deg, s, N. Uses the controller's currently selected Tool/TCP.
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

if __package__:
    from repair_car.config import load
    from repair_car.robot_control.contact_probe import ContactProbe
    from repair_car.report.force_slide_recording import ForceSlideRecorder
    from repair_car.experiments.force_slide_pose import ReadyPose
    from repair_car.robot_control.robot_bridge import RobotServices
    from repair_car.robot_control.rotations import rotation_zyz
else:
    from config import load
    from repair_car.robot_control.contact_probe import ContactProbe
    from repair_car.report.force_slide_recording import ForceSlideRecorder
    from repair_car.experiments.force_slide_pose import ReadyPose
    from robot_control.robot_bridge import RobotServices
    from robot_control.rotations import rotation_zyz


class ForceSlide:
    distance_mm = 50.0
    speed_mm_s = 5.0
    settle_s = 5.0
    relative_motion = 1  # DR_MV_MOD_REL, separate from the force-control mode

    def __init__(self, robot, force_n):
        self.robot = robot
        self.probe = ContactProbe(robot, force_n)
        self.recorder = ForceSlideRecorder(robot, force_n)
        self.ready = ReadyPose()
        self.force_reference_set = False

    def log(self, message):
        self.robot.node.get_logger().info(message)

    @staticmethod
    def pose_error(actual, target):
        distance = float(np.linalg.norm(np.subtract(actual[:3], target[:3])))
        rotation = rotation_zyz(target[3:]).T @ rotation_zyz(actual[3:])
        angle = float(np.degrees(np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))))
        return distance, angle

    def wait_motion(self, target=None, timeout=60.0):
        """Require three idle readings and, for positioning, the measured target pose."""
        deadline, idle = time.monotonic() + timeout, 0
        while time.monotonic() < deadline:
            self.robot.check_cancelled()
            status = self.robot.check_motion()
            idle = idle + 1 if status == 0 else 0
            if idle >= 3:
                if target is None:
                    return
                actual, _ = self.robot.get_current_posx(ref=self.robot.DR_BASE)
                distance, angle = self.pose_error(actual, target)
                if distance <= 1.0 and angle <= 1.0:
                    self.log(f'Target reached: actual pose={actual}, position error={distance:.3f} mm, '
                             f'orientation error={angle:.3f} deg')
                    return
            time.sleep(0.1)
        self.robot.stop()
        raise TimeoutError(f'Move/target check timed out: target={target}, '
                           f'last alarm={self.robot.last_alarm or "none"}')

    def home(self):
        target = self.robot.fkin(self.robot.HOME, ref=self.robot.DR_BASE)
        self.robot.movej(self.robot.HOME, vel=30.0, acc=30.0)
        self.wait_motion(target)

    def move(self, pose, speed, *, mod=0, timeout=60.0):
        """Wait for BASE linear motion while leaving the force frame at TOOL."""
        self.robot.amovel(pose, [speed, 20.0], [20.0, 40.0],
                          ref=self.robot.DR_BASE, mod=mod)
        self.wait_motion(pose if mod == self.robot.DR_MV_MOD_ABS else None, timeout)

    def log_force(self):
        wrench = self.robot.get_tool_force(ref=self.robot.DR_TOOL)
        pose, _ = self.robot.get_current_posx(ref=self.robot.DR_BASE)
        self.log(f'TOOL Fz={wrench[2]:+.3f} N, '
                 f'BASE TCP XYZ={[round(value, 3) for value in pose[:3]]} mm')
        return pose, wrench

    def wait_force(self):
        deadline = time.monotonic() + self.settle_s
        initial, _ = self.log_force()
        actual = initial
        while time.monotonic() < deadline:
            time.sleep(min(1.0, max(0.0, deadline - time.monotonic())))
            actual, _ = self.log_force()
        travel = float(np.subtract(actual[:3], initial[:3]) @ rotation_zyz(initial[3:])[:, 2])
        self.log(f'Actual displacement along TOOL +Z during the 5 s wait: {travel:+.3f} mm '
                 '(force control is not judged active from sensor values before contact alone)')

    def run(self):
        current, _ = self.robot.get_current_posx(ref=self.robot.DR_BASE)
        start_pose = self.ready.load()
        self.log(f'1. Current TCP pose (BASE, mm/deg): {current}')
        self.log(f'Fixed ready pose: {start_pose} ({self.ready.path})')
        self.log(f'Current Tool={self.robot.get_tool()}, TCP={self.robot.get_tcp()}')
        self.recorder.start()

        self.log('2. Move HOME, then to the fixed ready pose')
        self.recorder.phase = 'home_start'
        self.home()
        self.recorder.phase = 'approach'
        self.move(start_pose, speed=10.0)

        try:
            direction = '+Z' if self.probe.force > 0 else '-Z'
            self.log(f'3. TOOL compliance and TCP {direction} force control: {self.probe.force:+.3f} N')
            self.recorder.phase = 'force_enable'
            # Set the controller frame before compliance, never during it.
            self.robot.set_ref_coord(self.robot.DR_TOOL)
            self.force_reference_set = True
            self.probe.enable(frame=self.robot.DR_TOOL)
            time.sleep(0.1)
            # Send the signed target explicitly, without the contact probe's bias adjustment.
            self.probe.forcing = True
            self.robot.set_desired_force(
                [0.0, 0.0, self.probe.force, 0.0, 0.0, 0.0], [0, 0, 1, 0, 0, 0],
                time=0.5, mod=self.robot.DR_FC_MOD_ABS)
            self.log('Force control command succeeded: ref=TOOL, dir=[0, 0, 1, 0, 0, 0], mod=ABS')

            self.log(f'4. Waiting {self.settle_s} s while holding force control')
            self.recorder.phase = 'force_settle'
            self.wait_force()

            self.log(f'5. Moving BASE +X {self.distance_mm} mm while holding force control '
                     f'({self.speed_mm_s} mm/s)')
            self.recorder.phase = 'slide'
            self.move([self.distance_mm, 0.0, 0.0, 0.0, 0.0, 0.0],
                      speed=self.speed_mm_s, mod=self.relative_motion,
                      timeout=self.distance_mm / self.speed_mm_s + 10.0)
        finally:
            # Release force and compliance before any HOME motion.
            self.recorder.phase = 'release'
            self.close()

        self.log('6. Force/compliance control released, returning HOME')
        self.recorder.phase = 'home_return'
        self.home()
        self.log('Task complete')

    def close(self):
        self.probe.close()  # Also stops an interrupted approach or HOME motion.
        if self.force_reference_set:
            self.robot.set_ref_coord(self.robot.DR_BASE)
            self.force_reference_set = False


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force', type=float, default=1.5,
                        help='TCP Z target force [N], positive=+Z / negative=-Z (default: +1.5)')
    parser.add_argument('--capture-ready', action='store_true',
                        help='save the current BASE TCP pose as the new ready pose and exit without moving')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    if options.force == 0:
        parser.error('--force must be a non-zero target force [N]')

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('force_slide', namespace='dsr01')
    robot = RobotServices(node, timeout=load('robot')['robot']['service_timeout'])
    task = ForceSlide(robot, options.force)
    try:
        if options.capture_ready:
            pose, _ = robot.get_current_posx(ref=robot.DR_BASE)
            task.ready.save(pose)
            node.get_logger().info(f'Ready pose saved: {pose} ({task.ready.path}); robot not moved')
        else:
            task.run()
    except KeyboardInterrupt:
        node.get_logger().info('User stop: motion stopped, force/compliance control released')
    finally:
        try:
            task.close()
        finally:
            try:
                task.recorder.stop()
                path = task.recorder.save()
                if path is not None:
                    node.get_logger().info(f'Graph saved: {path} (raw data: {path.with_suffix(".csv")})')
            finally:
                robot.close()
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
