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
    from .configs import load
    from .contact_probe import ContactProbe
    from .force_slide_recording import ForceSlideRecorder
    from .force_slide_pose import ReadyPose
    from .robot_services import RobotServices
    from .rotations import rotation_zyz
else:
    from configs import load
    from contact_probe import ContactProbe
    from force_slide_recording import ForceSlideRecorder
    from force_slide_pose import ReadyPose
    from robot_services import RobotServices
    from rotations import rotation_zyz


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
                    self.log(f'목표 도달: 실제 pose={actual}, 위치 오차={distance:.3f} mm, '
                             f'자세 오차={angle:.3f} deg')
                    return
            time.sleep(0.1)
        self.robot.stop()
        raise TimeoutError(f'이동/목표 도달 확인 시간 초과: 목표={target}, '
                           f'마지막 알람={self.robot.last_alarm or "없음"}')

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
        self.log(f'5초 대기 중 TOOL +Z 방향 실제 변위: {travel:+.3f} mm '
                 '(접촉 전 센서값만으로 힘제어 활성 여부를 판정하지 않습니다)')

    def run(self):
        current, _ = self.robot.get_current_posx(ref=self.robot.DR_BASE)
        start_pose = self.ready.load()
        self.log(f'1. 현재 TCP pose (BASE, mm/deg): {current}')
        self.log(f'고정 준비 pose: {start_pose} ({self.ready.path})')
        self.log(f'현재 Tool={self.robot.get_tool()}, TCP={self.robot.get_tcp()}')
        self.recorder.start()

        self.log('2. HOME 이동 후 고정 준비 pose로 이동')
        self.recorder.phase = 'home_start'
        self.home()
        self.recorder.phase = 'approach'
        self.move(start_pose, speed=10.0)

        try:
            direction = '+Z' if self.probe.force > 0 else '-Z'
            self.log(f'3. TOOL 순응제어 및 TCP {direction} 힘제어: {self.probe.force:+.3f} N')
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
            self.log('힘제어 명령 응답 성공: ref=TOOL, dir=[0, 0, 1, 0, 0, 0], mod=ABS')

            self.log(f'4. 힘제어를 유지하며 {self.settle_s}초 대기')
            self.recorder.phase = 'force_settle'
            self.wait_force()

            self.log(f'5. 힘제어를 유지하며 BASE +X {self.distance_mm} mm 이동 '
                     f'({self.speed_mm_s} mm/s)')
            self.recorder.phase = 'slide'
            self.move([self.distance_mm, 0.0, 0.0, 0.0, 0.0, 0.0],
                      speed=self.speed_mm_s, mod=self.relative_motion,
                      timeout=self.distance_mm / self.speed_mm_s + 10.0)
        finally:
            # Release force and compliance before any HOME motion.
            self.recorder.phase = 'release'
            self.close()

        self.log('6. 힘제어/순응제어 해제 완료, HOME 복귀')
        self.recorder.phase = 'home_return'
        self.home()
        self.log('작업 완료')

    def close(self):
        self.probe.close()  # Also stops an interrupted approach or HOME motion.
        if self.force_reference_set:
            self.robot.set_ref_coord(self.robot.DR_BASE)
            self.force_reference_set = False


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force', type=float, default=1.5,
                        help='TCP Z 목표 힘 [N], 양수=+Z / 음수=-Z (기본값: +1.5)')
    parser.add_argument('--capture-ready', action='store_true',
                        help='현재 BASE TCP pose를 새 준비 위치로 저장하고 이동 없이 종료')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    if options.force == 0:
        parser.error('--force는 0이 아닌 목표 힘 [N]으로 지정하세요')

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('force_slide', namespace='dsr01')
    robot = RobotServices(node, timeout=load('robot')['robot']['service_timeout'])
    task = ForceSlide(robot, options.force)
    try:
        if options.capture_ready:
            pose, _ = robot.get_current_posx(ref=robot.DR_BASE)
            task.ready.save(pose)
            node.get_logger().info(f'준비 pose 저장: {pose} ({task.ready.path}); 로봇 이동 없음')
        else:
            task.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단: 이동 정지 및 힘제어/순응제어 해제')
    finally:
        try:
            task.close()
        finally:
            try:
                task.recorder.stop()
                path = task.recorder.save()
                if path is not None:
                    node.get_logger().info(f'그래프 저장: {path} (원본 데이터: {path.with_suffix(".csv")})')
            finally:
                robot.close()
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
