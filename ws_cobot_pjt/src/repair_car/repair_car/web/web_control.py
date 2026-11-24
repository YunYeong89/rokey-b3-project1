"""Web robot control on the virtual session or the real robot (the controller's robot system tells them apart): one
robot program at a time for /robot/command and /repair/command, the live force graph (draw_graph) beside it, and a
robot stop that works with or without a program started here. State on /robot/status (String JSON) every 0.5 s.
  /robot/command start (individual tasks), inspect (ordered panel scans), inspection_continue, stop, graph
  /scan_part     custom_interfaces ScanPart action: real_scan (part box: the performance coupon) as a program here;
                 its /scan/progress is the feedback (also on /robot/status as "scan"), /scan/result the result, and
                 cancelling stops it like the robot stop."""
import fcntl
import json
import sys
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from dsr_msgs2.srv import GetRobotSystem, MoveStop, ReleaseComplianceCtrl, ReleaseForce
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from custom_interfaces.action import ScanPart

from repair_car.config import fleet_id, load, read_session
from repair_car.inspection.part_scan import BOX_CONTROLLERS
from repair_car.geometry.real_vehicle import RealVehicle
from repair_car.web.repair_db import RepairDB
from repair_car.robot_control.rg2_state import RG2State
from repair_car.inspection.scan_scene import ScanScene
from repair_car.web.web_commands import WebCommands
from repair_car.web.web_inspection import InspectionProcess, InspectionStep
from repair_car.inspection.part_scan.archive import InspectionArchive
from repair_car.robot_control.robot_bridge import robot_topic
from repair_car.web.web_process import ManagedProcess

CONTROLLER = 'dsr_controller2/'
MODEL_SCENES = ('sedan', 'suv', 'sports')
ROBOT_NAMES = {'virtual': '가상 로봇', 'real': '실제 로봇', None: '로봇'}
VEHICLE_ROBOTS = {'real': 'robot_1', 'virtual': 'robot_2'}
STOP_GRACE_S = 15.0  # SIGINT to SIGKILL
QUERY_TIMEOUT_S = 2.0  # a controller query without an answer: asked again
SCAN_STAGES = ('planning', 'moving', 'scanning', 'reporting')  # ScanPart.Feedback stage


class VehicleRobotMismatch(ValueError):
    pass


class RobotWatch:
    """Controller readiness plus the virtual attachment or the real RG2 grip."""

    def __init__(self, node):
        self.system = node.create_client(GetRobotSystem, CONTROLLER + 'system/get_robot_system')
        node.create_subscription(String, 'virtual_control/status', self.virtual_status, 10)
        node.create_subscription(JointState, 'joint_states', self.joints, qos_profile_sensor_data)
        settings = load('real_scene')['probe']
        self.gripper = RG2State(node, settings.get('held_width_mm', RG2State.HELD_WIDTH_MM))
        self.mode, self.feedback, self.feedback_time, self.joint_time = None, {}, 0., 0.
        self.calls = {}

    def virtual_status(self, message):
        self.feedback, self.feedback_time = json.loads(message.data), time.monotonic()
        self.mode = 'virtual'

    def joints(self, _message):
        self.joint_time = time.monotonic()

    def ask(self, name, client, request, answer):
        """One query in flight per name, its answer taken on a later tick; dropped after QUERY_TIMEOUT_S (a controller
        restarted by the robot session never answers what the one before it received)."""
        call, sent = self.calls.get(name, (None, 0.))
        if call is not None and call.done():
            if call.exception() is None:
                answer(call.result())
            call = None
        elif call is not None and time.monotonic() - sent > QUERY_TIMEOUT_S:
            client.remove_pending_request(call)
            call = None
        if call is None and client.service_is_ready():
            call, sent = client.call_async(request), time.monotonic()
        self.calls[name] = (call, sent)

    def tick(self):
        if self.mode is None:
            self.ask('system', self.system, GetRobotSystem.Request(),
                     lambda result: setattr(self, 'mode', 'virtual' if result.robot_system == 1 else 'real'))

    def ready(self):
        if self.mode == 'virtual':
            return time.monotonic() - self.feedback_time < 2 and self.feedback.get('ready', False)
        return self.mode == 'real' and time.monotonic() - self.joint_time < 2

    def probe_attached(self):
        if self.mode == 'virtual':
            return self.feedback.get('probe_attached') if self.ready() else None
        return self.gripper.held() if self.mode == 'real' else None


class WebControl(Node):
    def __init__(self):
        super().__init__('web_control', namespace='dsr01')
        self.robot_namespace = self.get_namespace()
        robot_id = fleet_id(self.robot_namespace)
        self.ros_args = ['--ros-args', '-r', f'__ns:={self.robot_namespace}'] if robot_id else []
        lock_name = f'repair_car-web-control-{robot_id}' if robot_id else 'repair_car-web-control'
        self.lock = open(f'/tmp/{lock_name}.lock', 'a+')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.commands, self.robot = WebCommands(), RobotWatch(self)
        self.db = RepairDB(**load('robot')['db'])
        self.task = self.graph = self.job = None
        self.motion, self.stop_calls, self.graph_log = False, [], ''
        self.state = dict(state='idle', task=None, request_id=None, message='실행 대기')
        self.publisher = self.create_publisher(String, robot_topic(self, '/robot/status'), 10)
        self.create_subscription(String, robot_topic(self, '/robot/command'), self.command, 10)
        self.create_subscription(String, robot_topic(self, '/repair/command'), self.repair, 10)
        self.motion_stop = self.create_client(MoveStop, CONTROLLER + 'motion/move_stop')
        self.force_release = self.create_client(ReleaseForce, CONTROLLER + 'force/release_force')
        self.compliance_release = self.create_client(ReleaseComplianceCtrl,
                                                     CONTROLLER + 'force/release_compliance_ctrl')
        self.surface_stop = self.create_client(Trigger, 'surface_follow/end')
        self.scan, self.scan_goal, self.scan_done, self.scan_result = (0., None), None, None, None
        self.create_subscription(ScanPart.Feedback, robot_topic(self, '/scan/progress'), self.scan_progress, 10)
        self.create_subscription(ScanPart.Result, robot_topic(self, '/scan/result'), self.scan_finished, 10)
        self.scan_action = ActionServer(self, ScanPart, robot_topic(self, '/scan_part'), self.scan_execute,
                                        goal_callback=self.scan_accept, cancel_callback=self.scan_cancel)
        self.create_timer(.5, self.tick)
        self.orphan_timer = self.create_timer(2., self.close_orphaned_jobs)
        self.get_logger().info('Web control ready (virtual session or real robot), waiting for commands')

    def close_orphaned_jobs(self):
        """Once, at the DB's first answer after this start: a repair of this robot's vehicles left 'running' had its
        process end with an earlier session (web_control runs repair_manager as its child), so it is closed as
        stopped; otherwise it blocks every new repair (app.py: one running job at a time). A repair_manager started
        from a terminal in this namespace keeps its job."""
        robot = fleet_id(self.robot_namespace) or 'dsr01'
        try:
            jobs = self.db.running_jobs(robot)
        except RuntimeError:
            return   # the web server is not up yet: try again
        self.orphan_timer.cancel()
        if any(name == 'repair_manager' and namespace == self.robot_namespace
               for name, namespace in self.get_node_names_and_namespaces()):
            return
        for job in jobs:
            if job['id'] == self.job:
                continue
            try:
                self.db.finish(job['id'], 'stopped', '이전 세션 종료로 중단된 작업 정리 (수리 프로세스 없음)')
                self.get_logger().warn(f'job {job["id"]}: left running by an earlier session, closed as stopped')
            except RuntimeError as error:
                self.get_logger().error(f'job {job["id"]}: {error}')

    @staticmethod
    def scene(key='type'):
        """The virtual scene: its type (sedan, real, demo_left, ...) or, key model, the vehicle real_scan expects."""
        truth = read_session('scene_truth.json') or {}
        return truth.get(key, truth.get('type'))

    def external_tasks(self):
        if self.task is not None and not (isinstance(self.task, InspectionProcess) and self.task.phase == 'waiting_pose'):
            return []
        names = self.commands.nodes()
        return sorted({name for name, namespace in self.get_node_names_and_namespaces()
                       if namespace == self.robot_namespace and name in names})

    def publish(self, **event):
        external, mode = self.external_tasks(), self.robot.mode
        inspection = self.task.status() if isinstance(self.task, InspectionProcess) else None
        self.publisher.publish(String(data=json.dumps({**self.state, 'mode': mode, 'namespace': self.robot_namespace, 'ready': self.robot.ready(),
            'busy': self.task is not None or bool(external), 'managed': self.task is not None,
            'external_tasks': external, 'probe_attached': self.robot.probe_attached(),
            'gripper_width_mm': self.robot.gripper.width if mode == 'real' else None,
            'scene': self.scene() if mode == 'virtual' else None,
            'graph': self.graph is not None, 'graph_log': self.graph_log, 'inspection': inspection,
            'scan': self.scan_status() if inspection is None or inspection['phase'] == 'scanning' else None,
            'catalog': self.commands.catalog(), **event})))

    def check_slot(self):
        if self.task is not None:
            raise ValueError('이미 실행 중인 기능이 있습니다')
        if self.external_tasks():
            raise ValueError('터미널에서 실행 중인 로봇 프로그램을 먼저 종료하세요')

    def launch(self, task, arguments, request_id=None, job=None, motion=True, web_vehicle=None):
        self.check_slot()
        env = InspectionArchive.environment(web_vehicle)
        self.task = ManagedProcess([*arguments, *self.ros_args], env=env)
        self.started(task, request_id, job, motion)

    def started(self, task, request_id, job, motion):
        self.job, self.motion, self.stop_calls = job, motion, []
        self.state = dict(state='running', task=task, request_id=request_id, job_id=job, plan_only=not motion,
                          message='실행 중' if motion else '경로 계획 중', log='',
                          started_at=self.task.started_at)
        self.publish(event='accepted')

    def check_ready(self):
        if not self.robot.ready():
            raise ValueError(f'{ROBOT_NAMES[self.robot.mode]} 피드백·컨트롤러 준비를 기다리세요')

    def check_scan_ready(self):
        self.check_ready()
        held = self.robot.probe_attached()
        if held is None:
            raise ValueError('그리퍼 상태를 아직 확인할 수 없습니다')
        if not held:
            raise ValueError('탐색 전에 탐침을 수동으로 파지하세요')

    def check_probe_docked(self):
        self.check_ready()
        held = self.robot.probe_attached()
        if held is None:
            raise ValueError('그리퍼 상태를 먼저 확인하세요.')
        if held:
            raise ValueError('작업을 시작하려면 탐침을 거치대에 놓으세요.')

    def launch_steps(self, steps, request_id=None, plan_only=False, web_vehicle=None, pose_check=None, *, pick_bonnet=False):
        self.check_slot()
        pickup = None
        if not plan_only:
            virtual_probe_held = self.robot.mode == 'virtual' and self.robot.probe_attached() is True
            if pick_bonnet and not virtual_probe_held:
                self.check_probe_docked()
                pickup = ([sys.executable, '-u', '-m', 'repair_car.probe.pick_probe', *self.ros_args], self.check_probe_docked)
            else:
                self.check_scan_ready()
        env = InspectionArchive.environment(web_vehicle)
        steps = [InspectionStep(step.part, step.region, [*step.arguments, *self.ros_args]) for step in steps]
        self.scan = (0., None)
        self.task = InspectionProcess(steps, pose_check or (lambda previous, part: False),
                                      None if plan_only else self.check_scan_ready, env=env, pickup=pickup)
        self.started('real_scan', request_id, None, not plan_only)

    def check_vehicle_robot(self, vehicle):
        target = vehicle.get('robot_namespace')
        if not target:
            return
        namespace = fleet_id(self.robot_namespace) or VEHICLE_ROBOTS.get(self.robot.mode)
        if target != namespace or VEHICLE_ROBOTS.get(self.robot.mode) != target:
            raise VehicleRobotMismatch(f'이 차량은 /{target} 로봇에 연결되어 있습니다.')

    def inspect(self, data):
        plan_only = data.get('plan_only', False)
        steps = self.commands.inspection(data['parts'], plan_only, self.robot.mode)
        if type(data.get('vehicle_id')) is not int or data['vehicle_id'] < 1:
            raise ValueError('등록된 차량을 선택하세요')
        vehicle = self.db.vehicle(data['vehicle_id'])
        self.check_vehicle_robot(vehicle)
        if self.robot.mode == 'real':
            cell = load('real_scene')
            parts = dict.fromkeys(step.part for step in steps)
            height = RealVehicle().size[2] if any(part != 'bonnet' for part in parts) else None
            for part in parts:
                ScanScene.inspection_pose(part, cell, height)
        elif not ScanScene.multi_panel():
            panel = ScanScene.virtual_panel()
            if any(step.part != panel for step in steps):
                raise ValueError('가상 검사는 현재 배치된 면을 선택하세요. 배치는 --car demo_front, demo_left, demo_right로 바꿀 수 있습니다.')

        def pose_check(previous, part):
            if self.robot.mode != 'real' or plan_only:
                return False
            if ScanScene.multi_panel():
                return previous != part
            if previous is None and part == 'bonnet':
                return True
            return part != previous

        self.launch_steps(steps, data.get('request_id'), plan_only,
                          InspectionArchive.vehicle(vehicle), pose_check,
                          pick_bonnet=any(step.part == 'bonnet' for step in steps))

    def check(self, task):
        """Preconditions of a task that moves the robot."""
        spec, mode = self.commands.commands[task], self.robot.mode
        self.check_ready()
        if spec.get('probe') and not self.robot.probe_attached():
            raise ValueError('먼저 탐침 파지를 실행하세요')
        if task == 'bonnet_replace' and self.robot.probe_attached() is None:
            raise ValueError('그리퍼 파지 상태를 확인하지 못했습니다. 피드백과 탐침 위치를 확인하세요.')
        wanted = spec.get('scene')
        if mode == 'virtual' and wanted == 'real' and self.scene('model') != wanted:
            raise ValueError(f'--car {wanted} 장면에서 실행하세요')

    def command(self, message):
        data = {}
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('JSON 객체가 필요합니다')
            if data.get('cmd') == 'stop':
                return self.stop()
            if data.get('cmd') == 'graph':
                return self.toggle_graph(data.get('on') is True, data.get('compensated') is True)
            if self.robot.mode is None:
                raise ValueError('로봇 컨트롤러 응답을 기다리세요')
            if data.get('cmd') == 'inspect':
                return self.inspect(data)
            if data.get('cmd') == 'inspection_continue':
                if not isinstance(self.task, InspectionProcess) or data.get('request_id') != self.state['request_id']:
                    raise ValueError('배치 확인을 기다리는 작업이 없습니다')
                if self.external_tasks():
                    raise ValueError('터미널에서 실행 중인 로봇 프로그램을 먼저 종료하세요')
                self.task.confirm()
                return self.publish()
            if data.get('cmd') != 'start':
                raise ValueError('지원하지 않는 로봇 명령입니다')
            task, parameters = data['task'], data.get('parameters', {})
            arguments = self.commands.build(task, parameters, self.robot.mode)
            plan_only = parameters.get('plan_only') is True
            if not plan_only:
                self.check(task)
            web_vehicle = None
            if task == 'real_scan' and 'vehicle_id' in data:
                if type(data['vehicle_id']) is not int or data['vehicle_id'] < 1:
                    raise ValueError('등록된 차량을 선택하세요')
                vehicle = self.db.vehicle(data['vehicle_id'])
                self.check_vehicle_robot(vehicle)
                web_vehicle = InspectionArchive.vehicle(vehicle)
            if task == 'real_scan' and not plan_only:
                part = f'{parameters["side"]}_door' if parameters.get('side') else 'bonnet'
                self.launch_steps([InspectionStep(part, 0, arguments)], data.get('request_id'), web_vehicle=web_vehicle)
            else:
                self.launch(task, arguments, data.get('request_id'), motion=not plan_only, web_vehicle=web_vehicle)
        except (ValueError, KeyError, TypeError, OSError, RuntimeError) as error:
            self.publish(event='rejected', rejected_request=data.get('request_id') if isinstance(data, dict) else None,
                         error=str(error))

    def repair(self, message):
        data = {}
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('JSON 객체가 필요합니다')
            job_id = data['job_id']
            if type(job_id) is not int or job_id < 1:
                raise ValueError('잘못된 수리 작업 번호')
            if data.get('cmd') == 'stop':
                if self.job == job_id:
                    self.stop()
                return
            if data.get('cmd') != 'start':
                raise ValueError('알 수 없는 수리 명령')
            job = self.db.job(job_id)
            self.check_vehicle_robot(job)
            if not self.robot.ready():
                raise ValueError(f'{ROBOT_NAMES[self.robot.mode]}이 준비되지 않았습니다')
            if self.robot.mode == 'virtual' and self.scene() not in MODEL_SCENES and self.scene('model') != 'real':
                raise ValueError('수리 공정은 sedan/suv/sports 또는 실차(real, demo) 장면에서 실행하세요')
            if self.task is not None:
                if self.job == job_id:
                    return
                raise ValueError('다른 로봇 기능이 실행 중입니다')
            if job['status'] != 'running':
                raise ValueError('진행 중인 DB 작업만 실행할 수 있습니다')
            # the probe state is not required here: repair_manager picks the probe up first when it is not held
            args = [sys.executable, '-u', '-m', 'repair_car.web.repair_manager', '--job-id', str(job_id)]
            if self.robot.mode == 'real':
                if job.get('mode') == 'manual':
                    if job['requested'] != ['bonnet']:
                        raise ValueError('실기 수동 교체는 보닛 한 부품을 지원합니다.')
            self.launch('repair', args, job=job_id)
        except VehicleRobotMismatch as error:
            self.publish(event='rejected', rejected_job=job_id, error=str(error))
        except (ValueError, KeyError, TypeError, OSError, RuntimeError) as error:
            job_id = data.get('job_id') if isinstance(data, dict) else None
            if (isinstance(data, dict) and data.get('cmd') == 'start'
                    and type(job_id) is int and job_id > 0 and job_id != self.job):
                try:
                    self.db.finish(job_id, 'failed', str(error))
                except RuntimeError as db_error:
                    self.get_logger().error(str(db_error))
            self.publish(event='rejected', rejected_job=job_id, error=str(error))

    def stop(self):
        """Robot stop: move_stop at once (and the native surface stream in simulation), also with no program started
        here (one from a terminal keeps running but its motion stops); a program started here is interrupted too."""
        if self.task is not None and self.task.stop_due:
            return
        calls = [(self.motion_stop, MoveStop.Request(stop_mode=1))]
        if self.robot.mode == 'virtual':
            calls.append((self.surface_stop, Trigger.Request()))
        self.stop_calls = [client.call_async(request) for client, request in calls if client.service_is_ready()]
        if self.task is None:
            self.publish(event='robot_stopped', stop_sent=len(self.stop_calls))
            return
        self.task.interrupt(STOP_GRACE_S)
        self.state.update(state='stopping', message='중지 요청 처리 중')
        self.publish()

    def release(self):
        """Force and compliance off, after a program that was killed before it could release them itself."""
        for client, request in ((self.force_release, ReleaseForce.Request()),
                                (self.compliance_release, ReleaseComplianceCtrl.Request())):
            if client.service_is_ready():
                client.call_async(request)

    def toggle_graph(self, on, compensated):
        """Live force monitor: draw_graph streams /robot/force_graph to the web chart, nothing saved."""
        if on and self.graph is None:
            self.graph_log = ''
            self.graph = ManagedProcess([sys.executable, '-u', '-m', 'repair_car.report.draw_graph', '--rate', '20', '--no-png', '--no-save',
                                         *(['--compensated'] if compensated else []), *self.ros_args])
        elif not on and self.graph is not None:
            self.graph.interrupt(10.)
        self.publish()

    def finish_task(self):
        code = self.task.poll()
        self.state['log'] = self.task.log()
        if self.task.killed:
            self.state['message'] = '작업 종료 지연으로 프로세스를 종료했습니다'
        if code is None:
            return
        if self.state['task'] == 'teleop' and self.motion_stop.service_is_ready():
            self.motion_stop.call_async(MoveStop.Request(stop_mode=1))  # a jog outlives a killed teleop
        stopped = bool(self.task.stop_due)
        if stopped and any(not f.done() for f in self.stop_calls) and time.monotonic() < self.task.stop_due:
            return
        if self.task.killed:
            self.release()
        inspection = isinstance(self.task, InspectionProcess)
        error = self.task.error if inspection else ''
        message = f'작업 종료 (코드 {code})'
        if code == 0:
            message = '경로 계획 완료 · 로봇 이동 없음' if not self.motion else (
                '부품 탐색 완료' if inspection else '작업 완료')
        self.state.update(state='stopped' if stopped else ('succeeded' if code == 0 else 'failed'),
                          returncode=code, message='작업 중지' if stopped else error or message)
        if stopped:
            self.state['stop_confirmed'] = not self.motion or (bool(self.stop_calls) and all(
                f.done() and f.exception() is None and f.result().success for f in self.stop_calls))
            if not self.state['stop_confirmed']:
                self.state.update(state='failed', message='프로세스 종료 · 로봇 정지 응답 확인 필요')
        if self.scan_done is not None:
            self.scan_done.set_result(dict(self.state))
            self.scan_done = None
        if self.job is not None:
            try:
                self.db.finish(self.job, 'stopped' if self.state['state'] == 'stopped' else 'failed',
                               self.state['message'])
            except RuntimeError as error:
                self.state['message'] += f'; {error}'
        self.task.close()
        self.task = self.job = None

    def scan_progress(self, message):
        self.scan = (time.monotonic(), message)
        if self.scan_goal is not None:
            self.scan_goal.publish_feedback(message)

    def scan_finished(self, message):
        if self.scan_goal is not None:
            self.scan_result = message

    def scan_status(self):
        """The latest real_scan progress (started here or from a terminal), while it is fresh."""
        received, m = self.scan
        if m is None or time.monotonic() - received > 3.:
            return None
        return dict(stage=SCAN_STAGES[m.stage], path=m.path, paths=m.paths, segment=m.segment,
                    segments=m.segments, progress=m.progress, points=m.points, force_n=m.force_n,
                    remaining_s=m.remaining_s)

    def scan_request(self, goal):
        """A ScanPart goal as a web command: (task, real_scan arguments)."""
        task = 'box_scan' if goal.part == 'box' else 'real_scan'
        parameters = dict(plan_only=goal.plan_only, controller=goal.controller or 'default')
        parameters.update({key: value for key, value in (('pitch', goal.pitch_mm), ('speed', goal.speed_mm_s)) if value})
        if task == 'real_scan':
            parameters.update(part=goal.part, side=goal.side, replan=goal.replan)
        else:
            if self.robot.mode != 'virtual' or parameters['controller'] not in BOX_CONTROLLERS:
                raise ValueError('시편 액션은 가상 로봇의 시편 제어기에서만 지원합니다')
            parameters['part'] = 'bonnet'
        arguments = self.commands.build('real_scan', parameters, self.robot.mode)
        if task == 'box_scan':
            i = arguments.index('--part')
            arguments[i:i + 2] = ['--box']
        if any(goal.roi_mm):
            arguments += ['--roi', *map(str, goal.roi_mm)]
        return task, arguments

    def scan_accept(self, _goal):
        return GoalResponse.REJECT if self.task is not None or self.robot.mode is None else GoalResponse.ACCEPT

    def scan_cancel(self, _goal):
        self.stop()
        return CancelResponse.ACCEPT

    async def scan_execute(self, goal):
        """Run real_scan as this node's program until it exits; the result is its /scan/result, else the exit."""
        request = goal.request
        try:
            task, arguments = self.scan_request(request)
            if not request.plan_only and task == 'real_scan':
                self.check(task)
                part = f'{request.side}_door' if request.side else 'bonnet'
                self.launch_steps([InspectionStep(part, 0, arguments)])
            else:
                if task == 'box_scan' and not request.plan_only:
                    self.check_ready()
                    if not self.robot.probe_attached() or self.scene('model') != 'performance':
                        raise ValueError('시편 장면과 탐침 파지를 확인하세요')
                self.launch(task, arguments, motion=not request.plan_only)
        except (ValueError, KeyError, TypeError, OSError) as error:
            self.publish(event='rejected', error=str(error))
            goal.abort()
            return ScanPart.Result(message=str(error))
        self.scan_goal, self.scan_result, self.scan_done = goal, None, Future()
        state = await self.scan_done
        result = self.scan_result or ScanPart.Result(success=True, message=state['message'])
        result.success = result.success and state['state'] == 'succeeded'
        self.scan_goal = None
        if goal.is_cancel_requested:
            goal.canceled()
        elif result.success:
            goal.succeed()
        else:
            goal.abort()
        return result

    def tick(self):
        self.robot.tick()
        if self.graph is not None and self.graph.poll() is not None:
            self.graph_log = self.graph.log(800) if self.graph.process.returncode else ''
            self.graph.close()
            self.graph = None
        if self.task is not None:
            self.finish_task()
        self.publish()

    def close(self):
        if self.graph is not None:
            self.graph.interrupt(5.)
        if self.task is not None:
            self.stop()
            deadline = time.monotonic() + STOP_GRACE_S + 2
            while self.task is not None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.1)
        self.lock.close()
        self.destroy_node()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    control = WebControl()
    try:
        rclpy.spin(control)
    except KeyboardInterrupt:
        pass
    finally:
        control.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
