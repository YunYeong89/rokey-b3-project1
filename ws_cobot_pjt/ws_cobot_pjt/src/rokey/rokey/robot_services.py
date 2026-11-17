"""Bounded ROS service calls of the robot programs, without SDK global spins."""
import threading
import time

from dsr_msgs2 import srv
from dsr_msgs2.msg import RobotError
from rclpy.executors import SingleThreadedExecutor

if __package__:
    from .configs import load
else:
    from configs import load


class RobotServices:
    DR_BASE, DR_TOOL, DR_MV_MOD_ABS, DR_FC_MOD_ABS = 0, 1, 0, 0
    HOME = load('robot')['robot']['home_joints_deg']  # joint deg; the one HOME in robot.yaml

    def __init__(self, node, timeout=3.0):
        self.node, self.timeout = node, timeout
        self.prefix = node.get_namespace().rstrip('/') + '/dsr_controller2/'
        self.clients = {}
        self.lock = threading.Lock()
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(node)
        self.reference = self.DR_BASE
        self.motion_active = False
        self.rejection_count = 0
        self.rejection_message = ''
        self.last_alarm = ''
        self.cancelled = False  # set by a stop command; later requests fail until cleared
        self.alarm_subscription = node.create_subscription(
            RobotError, node.get_namespace().rstrip('/') + '/error', self._alarm, 10)

    def _alarm(self, message):
        text = ' '.join(part for part in (message.msg1, message.msg2, message.msg3) if part)
        self.last_alarm = f'level={message.level} group={message.group} code={message.code} {text}'.rstrip()
        if 'rejected event' in text:
            self.rejection_count += 1
            self.rejection_message = f'code={message.code}: {text}'
            self.node.get_logger().error(f'컨트롤러 명령 거절: {self.rejection_message}')
        else:
            self.node.get_logger().warn(f'컨트롤러 알람: {self.last_alarm}')

    def _check_rejection(self, previous, service_path):
        if self.rejection_count != previous:
            raise RuntimeError(f'컨트롤러가 명령을 거절했습니다 ({service_path}): {self.rejection_message}')

    SAFE = ('motion/move_stop', 'force/release_force', 'force/release_compliance_ctrl', 'motion/check_motion')

    # Read-only queries are asked again after a response timeout: on the Rosetta-emulated controller one
    # check_motion went unanswered for 3 s while the motion itself carried on, and the whole scan aborted.
    QUERIES = ('motion/check_motion', 'aux_control/get_current_posx', 'aux_control/get_tool_force')

    def request(self, service_path, service_type, **values):
        attempts = 3 if service_path in self.QUERIES else 1
        for attempt in range(1, attempts + 1):
            try:
                return self.call(service_path, service_type, **values)
            except TimeoutError as error:
                if attempt == attempts or '응답 시간 초과' not in str(error):
                    raise
                self.node.get_logger().warn(f'{error}; 다시 조회합니다 ({attempt + 1}/{attempts})')

    def call(self, service_path, service_type, **values):
        if self.cancelled and service_path not in self.SAFE:
            raise RuntimeError('작업 정지 요청으로 중단했습니다')
        with self.lock:
            rejection_before = self.rejection_count
            path = service_path if service_path.startswith('/') else self.prefix + service_path
            if service_path not in self.clients:
                self.clients[service_path] = self.node.create_client(service_type, path)
            client = self.clients[service_path]
            deadline = time.monotonic() + self.timeout
            while not client.wait_for_service(timeout_sec=0.05):
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'서비스 발견 시간 초과: {path}')
            future = client.call_async(service_type.Request(**values))
            try:
                # Finite waits let Python handle Ctrl+C while a response is missing.
                while not future.done():
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f'서비스 응답 시간 초과: {path}')
                    self.executor.spin_once(timeout_sec=0.05)
                    self._check_rejection(rejection_before, service_path)
                result = future.result()
                if result is None or not result.success:
                    message = getattr(result, 'message', '')
                    raise RuntimeError(f'서비스 명령 실패: {path}, 요청={values} {message}')
                return result
            finally:
                if not future.done():
                    future.cancel()

    def get_robot_system(self):
        """0: real, 1: virtual."""
        return self.request('system/get_robot_system', srv.GetRobotSystem).robot_system

    def get_robot_mode(self):
        return self.request('system/get_robot_mode', srv.GetRobotMode).robot_mode

    def get_tool(self):
        return self.request('tool/get_current_tool', srv.GetCurrentTool).info

    def get_tcp(self):
        return self.request('tcp/get_current_tcp', srv.GetCurrentTcp).info

    def verify_profiles(self, tool, tcp):
        active_tool, active_tcp = self.get_tool(), self.get_tcp()
        if (active_tool, active_tcp) != (tool, tcp):
            raise RuntimeError(
                f'Tool/TCP 불일치: 현재 Tool={active_tool}, TCP={active_tcp}; '
                f'필요 Tool={tool}, TCP={tcp}. 수동 모드에서 선택한 뒤 자동 모드로 실행하세요.')
        self.node.get_logger().info(f'기존 설정 사용: Tool={active_tool}, TCP={active_tcp} (변경 없음)')

    def set_tool(self, name):
        self.request('tool/set_current_tool', srv.SetCurrentTool, name=name)
        return 0

    def set_tcp(self, name):
        self.request('tcp/set_current_tcp', srv.SetCurrentTcp, name=name)
        return 0

    def get_current_posx(self, ref=0):
        result = self.request('aux_control/get_current_posx', srv.GetCurrentPosx, ref=ref)
        values = result.task_pos_info[0].data
        return list(values[:6]), int(round(values[6]))

    def fkin(self, joints_deg, ref=0):
        """BASE task pose (mm, deg) of the active TCP at joint angles `joints_deg`."""
        return list(self.request('motion/fkin', srv.Fkin, pos=[float(v) for v in joints_deg], ref=ref).conv_posx)

    def ikin(self, pose, solution, ref=0):
        """Joint angles (deg) of the active TCP at a BASE task pose (mm, deg) in one solution space, None if out of
        reach."""
        result = self.request('motion/ikin', srv.Ikin, pos=[float(v) for v in pose], sol_space=int(solution), ref=ref)
        return list(result.conv_posj) if result.success else None

    def get_tool_force(self, ref=0):
        return list(self.request('aux_control/get_tool_force', srv.GetToolForce, ref=ref).tool_force)

    def use_force_frame(self, coord):
        # Force services carry their own ref. Do not change the controller's
        # global reference while compliance control is active (alarm 1903).
        self.reference = coord
        return 0

    def set_ref_coord(self, coord):
        """Set the controller reference while force/compliance control is OFF."""
        self.request('motion/set_ref_coord', srv.SetRefCoord, coord=coord)
        self.reference = coord
        return 0

    def task_compliance_ctrl(self, stx):
        self.request('force/task_compliance_ctrl', srv.TaskComplianceCtrl,
                     stx=stx, ref=self.reference, time=0.0)
        return 0

    def set_desired_force(self, fd, direction, time=0.0, mod=0):
        self.request('force/set_desired_force', srv.SetDesiredForce,
                     fd=fd, dir=direction, ref=self.reference, time=time, mod=mod)
        return 0

    def release_force(self, time=0.0):
        self.request('force/release_force', srv.ReleaseForce, time=time)
        return 0

    def release_compliance_ctrl(self):
        self.request('force/release_compliance_ctrl', srv.ReleaseComplianceCtrl)
        return 0

    def amovel(self, pose, vel, acc, ref=0, mod=0):
        self.motion_active = True
        self.request('motion/move_line', srv.MoveLine, pos=pose, vel=vel, acc=acc,
                     ref=ref, mode=mod, sync_type=1)
        return 0

    def movej(self, pose, vel, acc):
        # Start asynchronously, then wait through interruptible, bounded state queries.
        self.motion_active = True
        self.request('motion/move_joint', srv.MoveJoint, pos=[float(value) for value in pose],
                     vel=float(vel), acc=float(acc), sync_type=1)
        deadline = time.monotonic() + 30.0
        while True:
            time.sleep(0.05)
            if self.check_motion() == 0:
                return 0
            self.check_cancelled()
            if time.monotonic() >= deadline:
                raise TimeoutError('초기 관절 이동 시간 초과')

    def check_cancelled(self):
        """Stop a running motion as soon as a stop command has set `cancelled`."""
        if self.cancelled:
            self.stop()
            raise RuntimeError('작업 정지 요청으로 중단했습니다')

    def check_motion(self):
        status = self.request('motion/check_motion', srv.CheckMotion).status
        if status == 0:
            self.motion_active = False
        return status

    def stop(self):
        self.request('motion/move_stop', srv.MoveStop, stop_mode=1)
        self.motion_active = False
        return 0

    def close(self):
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=1.0)
