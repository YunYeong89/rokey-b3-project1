"""Foreground supervisor for independent Gazebo worlds on one namespaced ROS graph."""
import argparse
from copy import deepcopy
import fcntl
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time

from service_launch import ROS_SETUP


ROOT = Path(__file__).resolve().parents[4]
COMPOSE = Path(__file__).resolve().parents[1] / 'Docker/compose.yaml'
CACHE = Path(os.environ.get('COBOT_FLEET_CACHE', ROOT / 'ws_cobot_pjt/src/repair_car/native/.cache'))
STATE = ROOT / 'ws_cobot_pjt/src/repair_car/data'
CONTAINER = os.environ.get('COBOT_CONTAINER', 'ws_cobot_pjt')


class RobotFleet:
    def __init__(self, args):
        self.args = args
        self.linux = platform.system() == 'Linux'
        self.virtual = [f'robot_{index}' for index in range(2, args.virtual_count + 2)]
        self.robots = ([('robot_1', 'real')] if args.real_host else []) + [
            (name, 'virtual') for name in self.virtual]
        self.emulators = [f'emulator_{name}' for name in self.virtual]
        self.processes = {}
        self.manifest = STATE / 'service_robots.json'
        self.binding = '127.0.0.1'
        self.services = ['cobot', *self.emulators]
        self.emulator_cpus = []

    def compose_override(self):
        import yaml

        template = yaml.safe_load(COMPOSE.read_text())['services']['emulator']
        template = {key: value for key, value in template.items()
                    if key not in ('profiles', 'ports', 'container_name', 'cpuset')}
        cobot = {'cpuset': '', 'environment': {'ROS_DOMAIN_ID': str(self.args.ros_domain),
                                              'COBOT_SERVICE_MODE': '1'}}
        if 'COBOT_WEB_PORT' in os.environ:
            cobot['environment']['REPAIR_DB_URL'] = f'http://host.docker.internal:{os.environ["COBOT_WEB_PORT"]}'
        if self.linux:
            cobot['extra_hosts'] = ['host.docker.internal:host-gateway']
        emulators = {name: deepcopy(template) for name in self.emulators}
        for name, cpus in zip(self.emulators, self.emulator_cpus):
            emulators[name]['cpuset'] = cpus
        return {'services': {'cobot': cobot, **emulators}}

    def assign_emulator_cpus(self):
        if self.linux or len(self.emulators) < 2:
            return
        count = int(subprocess.check_output(['docker', 'info', '--format', '{{.NCPU}}'], text=True))
        # DRCF startup is sensitive to contention under Rosetta. Use disjoint cores when available.
        width = min(4, count // len(self.emulators))
        if width >= 2:
            self.emulator_cpus = [f'{index * width}-{(index + 1) * width - 1}'
                                  for index in range(len(self.emulators))]

    def docker(self, *args, **kwargs):
        return subprocess.run(['docker', 'exec', CONTAINER, *args], **kwargs)

    def prepare(self):
        self.compose('up', '-d', '--wait', 'cobot')
        for emulator in self.emulators:
            self.compose('up', '-d', '--wait', '--wait-timeout', '90', emulator)
        packages = ('m0609_rg2_bringup', 'dsr_controller2', 'dsr_hardware2', 'onrobot_rg_control',
                    'custom_interfaces', 'repair_car')
        check = ' && '.join(f'test -d /workspace/install/{name}/share/{name}' for name in packages)
        if self.args.build or self.docker('bash', '-c', check).returncode:
            print('Building the ROS 2 workspace in Docker', flush=True)
            self.docker('bash', '-lc', 'source /opt/ros/jazzy/setup.bash && cd /workspace && '
                        'colcon build --symlink-install --packages-up-to ' + ' '.join(packages) +
                        ' --cmake-args -DBUILD_TESTING=OFF', check=True)
        if self.linux:
            addresses = self.docker('getent', 'ahostsv4', 'host.docker.internal', text=True,
                                    capture_output=True, check=True)
            self.binding = addresses.stdout.split()[0]

    def compose(self, *args, **kwargs):
        kwargs.setdefault('check', True)
        return subprocess.run(['docker', 'compose', '--progress', 'plain', '-f', str(COMPOSE),
                               '-f', str(self.override), *args], **kwargs)

    def start(self, name, args, **kwargs):
        self.processes[name] = subprocess.Popen(args, start_new_session=True, **kwargs)

    def launch(self):
        STATE.mkdir(parents=True, exist_ok=True)
        robots = []
        for name, mode in self.robots:
            directory = CACHE / 'service' / name
            env = {**os.environ, 'COBOT_SERVICE_ROBOT': name,
                   'COBOT_SERVICE_DOMAIN': str(self.args.ros_domain),
                   'COBOT_RELAY_BIND': self.binding, 'COBOT_NATIVE_CACHE': str(directory),
                   'COBOT_SESSION_DIR': str(STATE / name)}
            args = [sys.executable, str(ROOT / 'ws_cobot_pjt/src/repair_car/native/session.py'), '--mode', mode]
            if not self.args.render or self.args.no_gui:
                args += ['--no-gui']
            if mode == 'virtual':
                args += ['--no-rviz']
                if self.args.web or self.args.no_gazebo_gui:
                    args += ['--no-gazebo-gui']
                args += ['--car', self.args.car, '--probe', self.args.probe]
                if self.args.car_pose:
                    args += ['--car-pose', *map(str, self.args.car_pose)]
            else:
                args += ['--host', self.args.real_host, '--car', self.args.real_car]
                if self.args.real_car_pose:
                    args += ['--car-pose', *map(str, self.args.real_car_pose)]
                if self.args.rt_host:
                    args += ['--rt-host', self.args.rt_host]
            self.start(name, args, env=env)
            ns = '/' + name
            robots.append(dict(namespace=ns, mode=mode,
                               joints=ns + ('/sim_joint_states' if mode == 'virtual' else '/joint_states'),
                               description=ns + '/robot_description',
                               scene=ns + '/sim_scene' if mode == 'virtual' else None,
                               wrench=ns + '/virtual_control/wrench' if mode == 'virtual' else None,
                               status=ns + '/robot/status', command=ns + '/robot/command',
                               scan_action=ns + '/scan_part', scan_progress=ns + '/scan/progress',
                               scan_samples=ns + '/scan/samples', force_graph=ns + '/robot/force_graph',
                               session_dir=name, scene_dir=str(directory / 'scene')))
        with (CACHE / 'service/rosbridge.log').open('w') as log:
            self.start('rosbridge', ['docker', 'exec', CONTAINER, 'bash', '-lc', ROS_SETUP +
                       'exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090'],
                       stdout=log, stderr=subprocess.STDOUT)
        self.manifest.write_text(json.dumps(dict(ros_domain_id=self.args.ros_domain,
                                                rosbridge='ws://127.0.0.1:9090', robots=robots), indent=2) + '\n')
        print(f'ROS graph: {", ".join(robot["namespace"] for robot in robots)} | {self.manifest}', flush=True)
        if self.args.web:
            self.start('web', [str(ROOT / 'ws_cobot_pjt/src/repair_car/launch/run_web.sh'), 'start'])
        # A robot session that ends leaves the others running (the real robot and the simulation are independent);
        # a shared process (rosbridge, web) or the last robot ending stops the fleet.
        robots, ended = {name for name, _ in self.robots}, set()
        while True:
            for name, process in self.processes.items():
                if name in ended or process.poll() is None:
                    continue
                if name not in robots or robots <= ended | {name}:
                    raise RuntimeError(f'{name} exited ({process.returncode}). Logs: {CACHE / "service"}')
                ended.add(name)
                print(f'/{name} exited ({process.returncode}); the other robots keep running. Logs: {CACHE / "service"}',
                      flush=True)
            time.sleep(.5)

    def stop(self):
        self.manifest.unlink(missing_ok=True)
        for process in self.processes.values():
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
        deadline = time.monotonic() + 25
        for process in self.processes.values():
            try:
                process.wait(timeout=max(0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        self.compose('stop', *self.services, check=False)
        if self.emulators:
            self.compose('rm', '-f', *self.emulators, check=False)

    def run(self):
        (CACHE / 'service').mkdir(parents=True, exist_ok=True)
        with (CACHE / 'native-session.lock').open('a+') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError('Close the existing run_virtual.sh, run_real.sh or run_service.sh session') from error
            self.override = CACHE / 'service/compose.json'
            if not self.args.check:
                self.assign_emulator_cpus()
            self.override.write_text(json.dumps(self.compose_override(), indent=2))
            if self.args.check:
                self.compose('config', '-q')
                print('Compose configuration valid:', ', '.join('/' + name for name, _ in self.robots))
                return
            if self.virtual:
                try:
                    from gz.msgs10 import model_pb2  # noqa: F401
                    from gz.transport13 import _transport  # noqa: F401
                    if not shutil.which('gz'):
                        raise ImportError('gz command unavailable')
                except ImportError as error:
                    if not self.linux:
                        raise
                    raise RuntimeError('Ubuntu host needs Gazebo Harmonic and its Python bindings. '
                                       'Install gz-harmonic python3-gz-transport13 python3-gz-msgs10 '
                                       'python3-numpy python3-scipy python3-yaml.') from error
            try:
                self.prepare()
                self.launch()
            finally:
                self.stop()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog='Ubuntu 24.04: install Docker Engine with Compose and Gazebo Harmonic '
               '(gz-harmonic, python3-gz-transport13, python3-gz-msgs10, '
               'python3-numpy, python3-scipy, python3-yaml).')
    parser.add_argument('--virtual-count', type=int, default=2, help='Gazebo robots starting at /robot_2')
    parser.add_argument('--real-host', help='Physical robot controller IP for /robot_1')
    parser.add_argument('--rt-host', help='Physical robot RT interface IP, by default --real-host')
    fixtures = ('real', 'demo', 'demo_front', 'demo_left', 'demo_right', 'demo_final')
    parser.add_argument('--car', choices=[*fixtures, 'sedan', 'suv', 'sports', 'performance'], default='real')
    parser.add_argument('--real-car', choices=fixtures, default='real')
    parser.add_argument('--real-car-pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--car-pose', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--probe', choices=['standard', 'ball'], default='ball')
    parser.add_argument('--ros-domain', type=int, default=71)
    parser.add_argument('--build', action='store_true', help='Rebuild the ROS workspace before starting robots')
    parser.add_argument('--render', action='store_true', help='Open the real RViz and virtual Gazebo windows')
    parser.add_argument('--no-gui', action='store_true', help='Run without native display windows')
    parser.add_argument('--no-gazebo-gui', action='store_true', help='Run Gazebo server only')
    parser.add_argument('--web', action='store_true',
                        help='Start the web robot panel and run Gazebo without its GUI')
    parser.add_argument('--check', action='store_true', help='Check Docker Compose configuration without starting robots')
    args = parser.parse_args()
    if args.virtual_count < 0 or not (args.virtual_count or args.real_host):
        parser.error('Select at least one robot with --virtual-count or --real-host')
    if not 0 <= args.ros_domain <= 101:
        parser.error('--ros-domain must be between 0 and 101')
    if args.rt_host and not args.real_host:
        parser.error('--rt-host requires --real-host')

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        RobotFleet(args).run()
    except KeyboardInterrupt:
        pass
    except (ImportError, OSError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'run_service.sh: {error}\n')


if __name__ == '__main__':
    main()
