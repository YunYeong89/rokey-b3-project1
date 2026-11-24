"""ROS bridge of the robot programs: namespaced topics, program arguments and bounded DSR service calls."""
import threading
import time

import numpy as np
from dsr_msgs2 import srv
from dsr_msgs2.msg import RobotError
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray

if __package__:
    from ..config import fleet_id, load
    from .rotations import angles_zyz, rotation_zyz
else:
    from config import fleet_id, load
    from robot_control.rotations import angles_zyz, rotation_zyz


def robot_topic(node, absolute):
    namespace = node.get_namespace()
    return namespace + absolute if fleet_id(namespace) else absolute


def program_args():
    from rclpy.utilities import remove_ros_args
    return remove_ros_args()[1:]


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
        self.sent = self.queued = None  # send(): the request in flight and the one waiting for it
        self.motion_active = False
        self.rejection_count = 0
        self.motion_fault_count = 0
        self.rejection_message = ''
        self.last_alarm = ''
        self.cancelled = False  # set by a stop command; later requests fail until cleared
        self.alarm_subscription = node.create_subscription(
            RobotError, node.get_namespace().rstrip('/') + '/error', self._alarm, 10)

    def _alarm(self, message):
        text = ' '.join(part for part in (message.msg1, message.msg2, message.msg3) if part)
        self.last_alarm = f'level={message.level} group={message.group} code={message.code} {text}'.rstrip()
        rejected = 'rejected event' in text
        if message.level >= 2 or rejected:
            self.motion_fault_count += 1
        if rejected:
            self.rejection_count += 1
            self.rejection_message = f'code={message.code}: {text}'
            self.node.get_logger().error(f'Controller rejected the command: {self.rejection_message}')
        else:
            self.node.get_logger().warn(f'Controller alarm: {self.last_alarm}')

    def _check_rejection(self, previous, service_path):
        if self.rejection_count != previous:
            raise RuntimeError(f'The controller rejected the command ({service_path}): {self.rejection_message}')

    SAFE = ('motion/move_stop', 'force/release_force', 'force/release_compliance_ctrl', 'motion/check_motion')

    # Read-only queries are asked again after a response timeout: on the Rosetta-emulated controller one
    # check_motion went unanswered for 3 s while the motion itself carried on, and the whole scan aborted.
    QUERIES = ('motion/check_motion', 'aux_control/get_current_posx', 'aux_control/get_tool_force',
               'system/get_robot_state')

    def request(self, service_path, service_type, **values):
        attempts = 3 if service_path in self.QUERIES else 1
        for attempt in range(1, attempts + 1):
            try:
                return self.call(service_path, service_type, **values)
            except TimeoutError as error:
                if attempt == attempts or 'response timed out' not in str(error):
                    raise
                self.node.get_logger().warn(f'{error}; querying again ({attempt + 1}/{attempts})')

    def call(self, service_path, service_type, **values):
        if self.cancelled and service_path not in self.SAFE:
            raise RuntimeError('Stopped by a stop request')
        with self.lock:
            rejection_before = self.rejection_count
            path = robot_topic(self.node, service_path) if service_path.startswith('/onrobot/') else (
                service_path if service_path.startswith('/') else self.prefix + service_path)
            if service_path not in self.clients:
                self.clients[service_path] = self.node.create_client(service_type, path)
            client = self.clients[service_path]
            deadline = time.monotonic() + self.timeout
            while not client.wait_for_service(timeout_sec=0.05):
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'Service discovery timed out: {path}')
            future = client.call_async(service_type.Request(**values))
            try:
                # Finite waits let Python handle Ctrl+C while a response is missing.
                while not future.done():
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f'Service response timed out: {path}')
                    self.executor.spin_once(timeout_sec=0.05)
                    self._check_rejection(rejection_before, service_path)
                result = future.result()
                if result is None or not result.success:
                    message = getattr(result, 'message', '')
                    raise RuntimeError(f'Service command failed: {path}, request={values} {message}')
                return result
            finally:
                if not future.done():
                    future.cancel()

    def send(self, service_path, service_type, **values):
        """Request without waiting: the response is taken on a later call's spin and not checked. For commands
        sent during a motion (set_desired_force took 0.08-0.18 s, the samples stopped meanwhile). One in flight: a
        newer request waits in its place and goes on flush() (072408: the queued commands held get_tool_force past
        its 3 x 3 s timeout and ended the scan)."""
        if self.cancelled:
            raise RuntimeError('Stopped by a stop request')
        self.queued = service_path, service_type, values
        self.flush()

    def flush(self):
        """Send the waiting request once the one in flight is answered."""
        if self.queued is None or self.sent is not None and not self.sent.done():
            return
        service_path, service_type, values = self.queued
        self.queued = None
        with self.lock:
            if service_path not in self.clients:
                self.clients[service_path] = self.node.create_client(service_type, self.prefix + service_path)
            self.sent = self.clients[service_path].call_async(service_type.Request(**values))

    def get_robot_system(self):
        """0: real, 1: virtual."""
        return self.request('system/get_robot_system', srv.GetRobotSystem).robot_system

    def get_robot_mode(self):
        return self.request('system/get_robot_mode', srv.GetRobotMode).robot_mode

    def get_robot_state(self):
        return self.request('system/get_robot_state', srv.GetRobotState).robot_state

    def get_control_mode(self):
        """3: position control, 4: torque control (compliance / force control)."""
        return self.request('aux_control/get_control_mode', srv.GetControlMode).control_mode

    def get_tool(self):
        return self.request('tool/get_current_tool', srv.GetCurrentTool).info

    def get_tcp(self):
        return self.request('tcp/get_current_tcp', srv.GetCurrentTcp).info

    def verify_profiles(self, tool, tcp=None):
        active_tool = self.get_tool()
        active, expected = f'Tool={active_tool}', f'Tool={tool}'
        active_tcp = None
        if tcp is not None:
            active_tcp = self.get_tcp()
            active += f', TCP={active_tcp}'
            expected += f', TCP={tcp}'
        if (active_tool, active_tcp) != (tool, tcp):
            raise RuntimeError(
                f'Profile mismatch: active {active}; required {expected}. Select it in manual mode, then run in auto mode.')
        self.node.get_logger().info(f'Using the existing setting: {active}')

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

    def get_current_posj(self):
        return list(self.request('aux_control/get_current_posj', srv.GetCurrentPosj).pos)

    def get_active_tcp_profile(self):
        """Flange-to-active-TCP transform [mm, deg ZYZ] from the controller's two BASE poses, while stationary."""
        flange = self.request('aux_control/get_current_tool_flange_posx', srv.GetCurrentToolFlangePosx,
                              ref=self.DR_BASE).pos
        tcp, _ = self.get_current_posx(self.DR_BASE)
        if not np.isfinite([flange, tcp]).all() or not np.any(flange):
            raise RuntimeError('Cannot read the flange or TCP pose.')
        base_to_flange = rotation_zyz(flange[3:]).T
        offset = base_to_flange @ np.subtract(tcp[:3], flange[:3])
        if np.linalg.norm(offset) < .001:
            raise RuntimeError('The active TCP has no probe offset.')
        angles = angles_zyz(base_to_flange @ rotation_zyz(tcp[3:]), [0., 0., 0.])
        return [*offset.tolist(), *angles]

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

    def set_desired_force(self, fd, direction, time=0.0, mod=0, wait=True):
        if wait:
            self.queued = None  # a waiting unanswered command would override this one later
        (self.request if wait else self.send)('force/set_desired_force', srv.SetDesiredForce,
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

    def amovesx(self, poses, vel, acc, ref=0, mod=0, opt=1):
        """Asynchronous spline from the TCP through up to 100 task poses (mm, deg); opt 1: constant speed."""
        self.motion_active = True
        self.request('motion/move_spline_task', srv.MoveSplineTask,
                     pos=[Float64MultiArray(data=[float(value) for value in pose]) for pose in poses],
                     pos_cnt=len(poses), vel=vel, acc=acc, time=0.0, ref=ref, mode=mod, opt=opt, sync_type=1)
        return 0

    def amoveb(self, poses, radius, vel, acc):
        """Send one BASE moveb of line segments. The controller ignores the final radius."""
        segments = [Float64MultiArray(data=[*pose, *[0.] * 6, 0., float(radius)]) for pose in poses]
        self.motion_active = True
        self.request('motion/move_blending', srv.MoveBlending, segment=segments, pos_cnt=len(segments),
                     vel=vel, acc=acc, ref=self.DR_BASE, mode=self.DR_MV_MOD_ABS, sync_type=1)

    def jog_multi(self, axis, speed, ref=0):
        """Velocity jog along the unit task-space `axis` [x y z rx ry rz] at `speed` % of 250 x 1.73 mm/s; end it with
        stop() only (jog_multi again while jogging hung the controller's service on the emulator)."""
        self.motion_active = True
        self.request('motion/jog_multi', srv.JogMulti, jog_axis=[float(value) for value in axis],
                     move_reference=ref, speed=float(speed))

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
                raise TimeoutError('Initial joint move timed out')

    def check_cancelled(self):
        """Stop a running motion as soon as a stop command has set `cancelled`."""
        if self.cancelled:
            self.stop()
            raise RuntimeError('Stopped by a stop request')

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
