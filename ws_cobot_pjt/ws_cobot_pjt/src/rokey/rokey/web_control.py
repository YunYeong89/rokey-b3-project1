"""One virtual-session task at a time, shared by /robot/command and /repair/command."""
import fcntl
import json
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from dsr_msgs2.srv import GetRobotSystem, MoveStop
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .configs import load, read_session
from .repair_db import RepairDB
from .web_commands import WebCommands


class WebControl(Node):
    def __init__(self):
        super().__init__('web_control', namespace='dsr01')
        self.lock = open('/tmp/rokey-web-control.lock', 'a+')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.commands = WebCommands()
        self.db = RepairDB(**load('robot')['db'])
        self.process = self.output = None
        self.job = None
        self.state = dict(state='idle', task=None, request_id=None, message='실행 대기')
        self.feedback, self.received, self.virtual = {}, 0., False
        self.stop_due = 0.
        self.stop_calls = []
        self.publisher = self.create_publisher(String, '/robot/status', 10)
        self.create_subscription(String, '/robot/command', self.command, 10)
        self.create_subscription(String, '/repair/command', self.repair, 10)
        self.create_subscription(String, 'virtual_control/status', self.receive, 10)
        prefix = '/dsr01/dsr_controller2/'
        self.system = self.create_client(GetRobotSystem, prefix + 'system/get_robot_system')
        self.motion_stop = self.create_client(MoveStop, prefix + 'motion/move_stop')
        self.surface_stop = self.create_client(Trigger, '/dsr01/surface_follow/end')
        self.system_call = None
        self.create_timer(.5, self.tick)
        self.get_logger().info('Web control ready; virtual session only, waiting for commands')

    def receive(self, message):
        self.feedback, self.received = json.loads(message.data), time.monotonic()

    def ready(self):
        return (self.virtual and time.monotonic() - self.received < 2
                and self.feedback.get('ready', False))

    def external_tasks(self):
        if self.process is not None:
            return []
        names = set(self.commands.commands) | {'repair_manager', 'web_home'}
        return sorted({name for name, namespace in self.get_node_names_and_namespaces()
                       if namespace == '/dsr01' and name in names})

    def publish(self, **event):
        external = self.external_tasks()
        self.publisher.publish(String(data=json.dumps({**self.state, 'ready': self.ready(),
            'busy': self.process is not None or bool(external), 'managed': self.process is not None,
            'external_tasks': external, 'probe_attached': self.feedback.get('probe_attached', False),
            'scene': (read_session('scene_truth.json') or {}).get('type'),
            'catalog': self.commands.catalog(), **event})))

    def launch(self, task, arguments, request_id=None, job=None, motion=True):
        if self.process is not None:
            raise ValueError('이미 실행 중인 기능이 있습니다')
        if self.external_tasks():
            raise ValueError('터미널에서 실행 중인 로봇 프로그램을 먼저 종료하세요')
        self.output = tempfile.TemporaryFile(mode='w+b')
        self.process = subprocess.Popen(arguments, stdout=self.output, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.job, self.stop_due, self.stop_calls = job, 0., []
        self.motion = motion
        self.state = dict(state='running', task=task, request_id=request_id,
                          job_id=job, message='실행 중', log='')
        self.publish(event='accepted')

    def command(self, message):
        data = {}
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('JSON 객체가 필요합니다')
            if data.get('cmd') == 'stop':
                self.stop()
                return
            if data.get('cmd') != 'start':
                raise ValueError('cmd는 start 또는 stop이어야 합니다')
            task, parameters = data['task'], data.get('parameters', {})
            arguments = self.commands.build(task, parameters)
            plan_only = parameters.get('plan_only') is True
            if not plan_only:
                if not self.ready():
                    raise ValueError('가상 로봇 피드백·컨트롤러 준비를 기다리세요')
                spec = self.commands.commands[task]
                if spec.get('probe') and not self.feedback.get('probe_attached'):
                    raise ValueError('먼저 탐침 파지를 실행하세요')
                if task == 'pick_probe' and self.feedback.get('probe_attached'):
                    raise ValueError('탐침이 이미 파지되어 있습니다')
                scene = (read_session('scene_truth.json') or {}).get('type')
                if spec.get('real_scene') and scene != 'real':
                    raise ValueError('--car real 장면에서 실행하세요')
                if spec.get('model') and scene not in ('sedan', 'suv', 'sports'):
                    raise ValueError('sedan/suv/sports 장면에서 실행하세요')
                if task == 'scan_surface' and not read_session('vehicle.json'):
                    raise ValueError('먼저 차량 식별을 실행하세요')
            self.launch(task, arguments, data.get('request_id'), motion=not plan_only)
        except (ValueError, KeyError, TypeError, OSError) as error:
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
            if not self.ready():
                raise ValueError('가상 로봇이 준비되지 않았습니다')
            if (read_session('scene_truth.json') or {}).get('type') not in ('sedan', 'suv', 'sports'):
                raise ValueError('수리 공정은 sedan/suv/sports 장면에서 실행하세요')
            if self.process is not None:
                if self.job == job_id:
                    return
                raise ValueError('다른 로봇 기능이 실행 중입니다')
            job = self.db.job(job_id)
            if job['status'] != 'running':
                raise ValueError('진행 중인 DB 작업만 실행할 수 있습니다')
            self.launch('repair', [sys.executable, '-u', '-m', 'rokey.repair_manager',
                                  '--job-id', str(job_id)], job=job_id)
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
        if self.process is None or self.stop_due:
            return
        self.state.update(state='stopping', message='중지 요청 처리 중')
        os.killpg(self.process.pid, signal.SIGINT)
        self.stop_due = time.monotonic() + 15
        if self.motion and self.virtual:
            for client, request in ((self.motion_stop, MoveStop.Request(stop_mode=1)),
                                    (self.surface_stop, Trigger.Request())):
                if client.service_is_ready():
                    self.stop_calls.append(client.call_async(request))
        self.publish()

    def tick(self):
        if self.system_call is not None and self.system_call.done():
            try:
                self.virtual = self.system_call.result().robot_system == 1
            except Exception:
                self.virtual = False
            self.system_call = None
        if self.system_call is None and self.system.service_is_ready():
            self.system_call = self.system.call_async(GetRobotSystem.Request())
        if self.process is not None:
            code = self.process.poll()
            size = os.fstat(self.output.fileno()).st_size
            self.state['log'] = os.pread(self.output.fileno(), 6000, max(0, size - 6000)).decode(errors='replace')
            if code is None and self.stop_due and time.monotonic() > self.stop_due:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.state['message'] = '작업 종료 지연으로 프로세스를 종료했습니다'
            if code is not None:
                stopped = bool(self.stop_due)
                if stopped and any(not f.done() for f in self.stop_calls) and time.monotonic() < self.stop_due:
                    self.publish()
                    return
                self.state.update(state='stopped' if stopped else ('succeeded' if code == 0 else 'failed'),
                                  returncode=code, message='작업 중지' if stopped else f'작업 종료 (코드 {code})')
                if stopped:
                    self.state['stop_confirmed'] = not self.motion or (len(self.stop_calls) == 2 and all(
                        f.done() and f.exception() is None and f.result().success for f in self.stop_calls))
                    if not self.state['stop_confirmed']:
                        self.state.update(state='failed', message='프로세스 종료 · 로봇 정지 응답 확인 필요')
                if self.job is not None:
                    try:
                        self.db.finish(self.job, 'stopped' if self.state['state'] == 'stopped' else 'failed',
                                       self.state['message'])
                    except RuntimeError as error:
                        self.state['message'] += f'; {error}'
                self.output.close()
                self.process = self.output = self.job = None
        self.publish()

    def close(self):
        if self.process is not None:
            self.stop()
            deadline = time.monotonic() + 17
            while self.process is not None and time.monotonic() < deadline:
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
