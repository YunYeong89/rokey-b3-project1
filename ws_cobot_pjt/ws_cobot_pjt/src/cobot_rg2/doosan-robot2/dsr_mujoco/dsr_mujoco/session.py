"""Native MuJoCo session using the unchanged Docker ROS controller and TCP bridge."""
import argparse
import fcntl
import json
import os
import secrets
import shlex
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

from displays import RVizDisplay, StateReceiver
from model_export import CACHE, ROOT
from .model import MuJoCoRobotModel as RobotModel
from robot_network import RobotNetwork
from calibration import DEFAULT_CALIBRATION

# Native processes of a session, recognisable by their files: Gazebo server / window, RViz, robot_state_publisher.
LEFTOVERS = [str(CACHE / 'world.sdf'), str(ROOT / 'native/gazebo.config'), str(CACHE / 'native.rviz'),
             str(CACHE / 'robot_state_publisher.yaml')]


def quiet(command, timeout=15):
    """Best-effort cleanup step: an unresponsive Docker must not hang the shutdown (a hung shutdown got killed by
    hand and left Gazebo / RViz / robot_state_publisher running)."""
    try:
        subprocess.run(command, check=False, timeout=timeout, capture_output=True)
    except subprocess.TimeoutExpired:
        print(f'Cleanup step timed out after {timeout} s: {" ".join(command[:5])}', flush=True)

# Mounted into the container as /workspace/session: with --car, the probe plan and the scene truth (for
# checking choose_car/scan_surface only). Both are removed when the session ends.
SESSION = ROOT / 'ws_cobot_pjt/session'
SESSION_PLAN = SESSION / 'probe_plan.json'
ROS = 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
# Container path of the bringup sources: the --acc launch file and controller bridge run from them (no rebuild).
BRINGUP = '/workspace/src/cobot_rg2/rg2/m0609_rg2_bringup'


class NativeSession:
    def __init__(self, args):
        self.args = args
        self.processes, self.logs = [], []
        self.remote_pidfiles = []
        self.receiver = self.display = None
        self.displays, self.viewers = [], []
        self.relay_path = None
        self.calibration_path = None
        self.session_files = []
        self.robot_network = None
        self.force_backend = None
        self.session_lock = None
        self.token = secrets.token_hex(16)
        self.env = {**os.environ, 'GZ_PARTITION': f'cobot_{self.token}',
                    'GZ_IP': '127.0.0.1', 'ROS_DOMAIN_ID': '129'}
        for key in ('GZ_PARTITION', 'GZ_IP', 'ROS_DOMAIN_ID'):
            os.environ[key] = self.env[key]
        # Prevent inherited Linux rendering backends from reaching native GUIs.
        for key in ['DISPLAY', 'MUJOCO_GL', 'PYOPENGL_PLATFORM', 'LIBGL_ALWAYS_SOFTWARE']:
            self.env.pop(key, None)

    def start(self, command, env=None):
        log = (CACHE / f'{self.token}_{len(self.logs)}.log').open('w')
        self.logs.append(log)
        process = subprocess.Popen(command, env={**self.env, **(env or {})}, stdout=log, stderr=log,
                                   start_new_session=True)
        self.processes.append(process)
        return process

    def start_remote(self, command, label):
        pidfile = f'/tmp/cobot-{self.token}-{label}.pid'
        self.remote_pidfiles.append(pidfile)
        script = f'echo $$ > {pidfile}; {command}'
        return self.start(['docker', 'exec', 'ws_cobot_pjt',
                           'setsid', '--wait', 'bash', '-lc', script])

    def run(self):
        CACHE.mkdir(exist_ok=True)
        lock = (CACHE / 'native-session.lock').open('a+')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            raise RuntimeError('다른 native 로봇 세션이 실행 중입니다. 기존 세션을 종료한 뒤 다시 실행하세요.')
        self.sweep()
        self.session_lock = lock
        lock.seek(0)
        lock.truncate()
        lock.write(str(os.getpid()))
        lock.flush()
        acc = self.args.acc is not None  # accelerated: the native emulated controller, no emulator container
        profile = ['--profile', 'virtual'] if self.args.mode == 'virtual' and not acc else []
        compose = ['docker', 'compose', '-f', str(ROOT / 'compose.yaml'), *profile]
        subprocess.run([*compose, 'up', '-d', '--wait'], check=True)
        if self.args.mode == 'virtual' and not acc and subprocess.run(
                ['docker', 'exec', 'ws_cobot_pjt', 'getent', 'hosts', 'emulator'], capture_output=True).returncode:
            # A restarted emulator can come up detached from the compose network; the controller
            # then never connects and no dsr_controller2 service exists. Recreate it attached.
            print('Emulator not reachable from ws_cobot_pjt; recreating it on the compose network', flush=True)
            subprocess.run([*compose, 'up', '-d', '--wait', '--force-recreate', 'emulator'], check=True)
        network_env = {}
        if self.args.mode == 'real' and not self.args.observe_only:
            self.robot_network = RobotNetwork(self.args.host, self.token)
            network_env = self.robot_network.start()
        if self.args.check_connection:
            result = self.robot_network.check_connection(
                network_env, self.args.rt_host or self.args.host)
            CACHE.mkdir(exist_ok=True)
            (CACHE / 'real_connection.json').write_text(json.dumps(result, indent=2))
            return
        calibration_file = self.args.calibration_file.resolve() if self.args.mode == 'virtual' else None
        model = RobotModel(calibration_file=calibration_file)
        if self.args.mode == 'virtual':
            self.calibration_path = None if acc else f'/tmp/cobot-calibration-{self.token}.json'
            from .backend import MuJoCoForceBackend as GazeboForceBackend
            model.gazebo_world(self.args.car, self.args.car_pose, self.args.probe)
            profiles=json.loads(calibration_file.read_text())
            scene=model.scene_map
            if scene is not None:
                probe_tool=scene.probe_tool
                plan=probe_tool.plan
                plan['initial_profiles'] = profiles['active'].copy()
                profiles['tcps'][plan['tcp_name']]={'pos':plan['tcp_mm_deg']}
                original=profiles['tools'][profiles['active']['tool']]
                weight=original['weight']+probe_tool.mass
                cog=[(original['weight']*value+probe_tool.mass*extra)/weight
                     for value,extra in zip(original['cog'],probe_tool.com_flange*1000)]
                profiles['tools'][plan['tool_name']]={'weight':weight,'cog':cog,'inertia':None}
                self.write_session(SESSION_PLAN, plan)
                self.write_session(SESSION/'scene_truth.json', scene.vehicle.truth)
                self.write_session(SESSION/'collision_geometry.json', model.collision_info)
            profile_path=CACHE/'session_calibration.json'
            profile_path.write_text(json.dumps(profiles,indent=2))
            if acc:
                from .accelerated import AcceleratedBackend
                self.display = self.force_backend = AcceleratedBackend(model, profiles, self.args.acc)
            else:
                subprocess.run(['docker','cp',str(profile_path),f'ws_cobot_pjt:{self.calibration_path}'],check=True)
                self.display = self.force_backend = GazeboForceBackend(model)
            self.displays.append(self.display)
            if not (self.args.no_gui or self.args.no_gazebo_gui):
                self.display.engine.view()
        if not self.args.no_gui and self.args.rviz:
            rviz = RVizDisplay(model, self.start)
            self.displays.append(rviz)
            self.viewers.append(rviz.viewer)
        if acc:
            self.start_remote(ROS + f'exec ros2 launch {BRINGUP}/launch/accelerated.launch.py', 'controller')
        elif not self.args.observe_only:
            host = 'emulator' if self.args.mode == 'virtual' else self.args.host
            environment = ' '.join(f'{key}={shlex.quote(value)}'
                                   for key, value in network_env.items())
            command = ('source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
                       f'exec env {environment} ros2 launch m0609_rg2_bringup bringup.launch.py '
                       f'mode:={self.args.mode} host:={shlex.quote(host)} port:=12345 '
                       f'rt_host:={shlex.quote(self.args.rt_host or host)} '
                       f'virtual_force:={str(self.args.mode == "virtual").lower()} '
                       'manage_emulator:=false gui:=false')
            if self.calibration_path:
                command += f' virtual_profile_file:={shlex.quote(self.calibration_path)}'
            self.start_remote(command, 'controller')
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(30)
        if self.force_backend:
            from virtual_force import ForceStateReceiver
            self.receiver = ForceStateReceiver(listener, self.token, self.force_backend)
            bridge = (f'env PYTHONPATH={BRINGUP}:$PYTHONPATH python3 -m cobot_virtual.controller' if acc
                      else 'ros2 run m0609_rg2_bringup virtual_force_bridge.py')
            command = ROS + f'exec {bridge} host.docker.internal {listener.getsockname()[1]} {self.token}'
        else:
            self.receiver = StateReceiver(listener, self.token)
            self.relay_path = f'/tmp/cobot-display-{self.token}.py'
            subprocess.run(['docker', 'cp', str(ROOT / 'native/ros_state_relay.py'),
                            f'ws_cobot_pjt:{self.relay_path}'], check=True)
            command = ('source /opt/ros/jazzy/setup.bash && exec python3 '
                       f'{self.relay_path} host.docker.internal {listener.getsockname()[1]} {self.token}')
        self.start_remote(command, 'relay')
        if self.args.web:
            sys.path.insert(0, str(self.args.web_dir))
            from robot_session import SimulationWeb
            SimulationWeb(self.args.web_dir, self.args.web_port).start(self, model)
        print(f'Native {self.args.mode} session. Logs: {CACHE}/{self.token}_*.log', flush=True)
        if not self.args.observe_only:
            print('Robot programs run in the container: docker exec -it ws_cobot_pjt bash -> ros2 run rokey <program>',
                  flush=True)
        start = time.monotonic()
        excursion, scene_due = {}, 0.0
        while not self.receiver.closed.is_set():
            if any(viewer.poll() == 0 for viewer in self.viewers):
                break
            if any(p.poll() is not None for p in self.processes):
                raise RuntimeError(f'A session process exited; inspect {CACHE}/{self.token}_*.log')
            positions = self.receiver.snapshot()
            for display in self.displays:
                display.update(positions.copy())
                if display is self.force_backend:
                    feedback = self.force_backend.snapshot()
                    self.receiver.send({'kind': 'feedback', **feedback})
                    positions = feedback['positions']
            if model.scene_map is not None:
                model.scene_map.probe_tool.update(positions)
            if model.scene_map is not None and time.monotonic() >= scene_due:
                # Same scene as RViz, for container-side viewers (/dsr01/sim_scene, e.g. --web).
                self.receiver.send({'kind': 'scene', 'markers': model.scene_map.marker_dicts()})
                scene_due = time.monotonic() + 0.2
            for name, value in positions.items():
                excursion[name] = max(excursion.get(name, 0), abs(value))
            if self.args.duration and time.monotonic() - start >= self.args.duration:
                break
            time.sleep(1 / 60)
        positions = self.receiver.snapshot()
        result = {'mode': self.args.mode, 'ros_positions': positions,
                  'max_absolute_positions_rad': excursion}
        if self.args.mode == 'virtual':
            result['mujoco_positions'] = self.display.feedback
            result['virtual_force'] = self.force_backend.snapshot()
            if model.scene_map is not None:
                result['scene'] = model.scene_map.vehicle.truth
        (CACHE / 'last_mujoco_session.json').write_text(json.dumps(result, indent=2))
        if self.args.duration and not positions and not acc:  # --acc: no ROS joint stream by design
            raise RuntimeError('No ROS joint states received; inspect session logs')
        if self.args.duration and self.args.mode == 'virtual' and not self.display.feedback:
            raise RuntimeError('No MuJoCo joint feedback received; inspect session logs')

    def close(self):
        if self.session_lock is None:
            return
        try:
            self.close_processes()
        finally:
            try:
                if self.robot_network:
                    self.robot_network.close()
                quiet(['docker', 'compose', '-f', str(ROOT / 'compose.yaml'), '--profile', 'virtual', 'stop'],
                      timeout=60)  # the emulator ignores TERM: compose waits its 10 s grace period, then kills
            finally:
                self.session_lock.close()
                self.session_lock = None

    @staticmethod
    def sweep():
        """Leftovers of a crashed or force-killed earlier session; safe here, this session holds the lock."""
        stale = [pid for pattern in LEFTOVERS
                 for pid in subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout.split()]
        if stale:
            print(f'Stopping {len(stale)} leftover process(es) of an earlier session: {" ".join(stale)}', flush=True)
            subprocess.run(['kill', '-TERM', *stale], check=False, capture_output=True)
            time.sleep(2)
            subprocess.run(['kill', '-KILL', *stale], check=False, capture_output=True)

    def close_processes(self):
        for pidfile in self.remote_pidfiles:
            quiet(['docker', 'exec', 'ws_cobot_pjt', 'bash', '-c',
                   f'if [ -f {pidfile} ]; then kill -INT -- -$(cat {pidfile}) 2>/dev/null || true; fi'])
        if self.receiver:
            self.receiver.close()
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for display in reversed(self.displays):
            display.close()
        for log in self.logs:
            log.close()
        for pidfile in self.remote_pidfiles:
            quiet(['docker', 'exec', 'ws_cobot_pjt', 'bash', '-c',
                   f'if [ -f {pidfile} ]; then kill -KILL -- -$(cat {pidfile}) 2>/dev/null || true; rm -f {pidfile}; fi'])
        if self.relay_path:
            quiet(['docker', 'exec', 'ws_cobot_pjt', 'rm', '-f', self.relay_path])
        if self.calibration_path:
            quiet(['docker', 'exec', 'ws_cobot_pjt', 'rm', '-f', self.calibration_path])
        for path in self.session_files:
            path.unlink(missing_ok=True)

    def write_session(self, path, data):
        path.parent.mkdir(exist_ok=True)
        path.write_text(json.dumps(data, indent=2))
        self.session_files.append(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['virtual', 'real'], default='virtual')
    parser.add_argument('--host')
    parser.add_argument('--rt-host', help='Controller RT interface IP; defaults to --host')
    parser.add_argument('--observe-only', action='store_true')
    parser.add_argument('--check-connection', action='store_true',
                        help='Check real TCP and RT telemetry without requesting control or motion')
    parser.add_argument('--no-gui', action='store_true')
    parser.add_argument('--rviz', action='store_true', help='Also start RViz (optional extra GUI load)')
    parser.add_argument('--no-simulator-gui', '--no-gazebo-gui', dest='no_gazebo_gui', action='store_true',
                        help='Disable the MuJoCo window; optional --rviz stays enabled')
    parser.add_argument('--web', action='store_true', help='Start the web robot controls, URDF viewer and rosbridge')
    parser.add_argument('--web-port', type=int, default=5050)
    parser.add_argument('--web-dir', type=Path, default=ROOT/'rokey-b3-project1/web')
    from scene import probe_types, vehicle_types
    parser.add_argument('--car', choices=[*vehicle_types(), 'random', 'real'],
                        help='Virtual only: put this vehicle (scaled, damaged as in native/scene.json) into the '
                             'vehicle zone of rokey robot.yaml, with the probe rack; real = the team vehicle '
                             'assembly (vehicles.yaml real) in front of the robot, its front toward the robot')
    parser.add_argument('--car-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
                        help='Vehicle pose in BASE (mm, deg) instead of a random pose inside the zone')
    parser.add_argument('--probe', choices=probe_types(), default='standard',
                        help='Probe on the rack (native/scene.json probes): standard tip, or ball = the same probe '
                             'with a low-friction metal ball tip')
    parser.add_argument('--acc', type=float, nargs='?', const=4.0, metavar='FACTOR',
                        help='Accelerated session without the Doosan emulator: a native controller replica in '
                             'simulation time, FACTOR times real time (default 4; 0 = as fast as possible)')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--calibration-file', type=Path, default=DEFAULT_CALIBRATION,
                        help='Real TCP/tool calibration JSON, applied in virtual mode')
    args = parser.parse_args()
    if args.mode != 'virtual':
        parser.error('MuJoCo supports virtual mode only')
    if args.acc is not None and (args.acc < 0 or args.observe_only):
        parser.error('--acc takes a factor >= 0 and drives the robot (not with --observe-only)')
    if args.web and args.mode != 'virtual':
        parser.error('--web currently requires --mode virtual')
    if args.mode != 'virtual' and args.car:
        parser.error('--car requires --mode virtual')
    if args.car_pose and not args.car:
        parser.error('--car-pose requires --car')
    if args.probe != 'standard' and not args.car:
        parser.error('--probe requires --car (the probe rack is part of the car scene)')
    if args.mode == 'real' and not args.observe_only and not args.host:
        parser.error('Real mode requires --host ROBOT_IP, or --observe-only for display only')
    if args.check_connection and (args.mode != 'real' or args.observe_only):
        parser.error('--check-connection requires real mode and --host')
    session = NativeSession(args)
    def interrupt(_signal, _frame):
        raise KeyboardInterrupt
    # SIGINT too: a session started from a background job inherits it ignored and would never see Ctrl+C.
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    try:
        session.run()
    except KeyboardInterrupt:
        pass
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        session.close()


if __name__ == '__main__':
    main()
