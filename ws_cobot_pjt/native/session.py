"""Run native displays and virtual force control; real control stays in Docker."""
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
from model_export import CACHE, ROOT, RobotModel
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
SESSION = Path(os.environ.get('COBOT_SESSION_DIR', ROOT / 'ws_cobot_pjt/session'))
SESSION_PLAN = SESSION / 'probe_plan.json'


class NativeSession:
    def __init__(self, args):
        self.args = args
        self.robot_id = os.environ.get('COBOT_SERVICE_ROBOT')
        self.bind_host = os.environ.get('COBOT_RELAY_BIND', '127.0.0.1')
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
        env = ['-e', f'ROS_DOMAIN_ID={os.environ["COBOT_SERVICE_DOMAIN"]}'] if self.robot_id else []
        return self.start(['docker', 'exec', *env, 'ws_cobot_pjt',
                           'setsid', '--wait', 'bash', '-lc', script])

    def run(self):
        if not self.robot_id:
            CACHE.mkdir(parents=True, exist_ok=True)
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
            profile = ['--profile', 'virtual'] if self.args.mode == 'virtual' else []
            compose = ['docker', 'compose', '-f', str(ROOT / 'compose.yaml'), *profile]
            subprocess.run([*compose, 'up', '-d', '--wait'], check=True)
            if self.args.mode == 'virtual' and subprocess.run(
                    ['docker', 'exec', 'ws_cobot_pjt', 'getent', 'hosts', 'emulator'], capture_output=True).returncode:
                # A restarted emulator can come up detached from the compose network; the controller
                # then never connects and no dsr_controller2 service exists. Recreate it attached.
                print('Emulator not reachable from ws_cobot_pjt; recreating it on the compose network', flush=True)
                subprocess.run([*compose, 'up', '-d', '--wait', '--force-recreate', 'emulator'], check=True)
        network_env = {}
        if self.args.mode == 'real' and not self.args.observe_only:
            self.robot_network = RobotNetwork(self.args.host, self.token, self.bind_host)
            network_env = self.robot_network.start()
        if self.args.check_connection:
            result = self.robot_network.check_connection(
                network_env, self.args.rt_host or self.args.host)
            CACHE.mkdir(exist_ok=True)
            (CACHE / 'real_connection.json').write_text(json.dumps(result, indent=2))
            return
        calibration_file = self.args.calibration_file.resolve() if self.args.mode == 'virtual' else None
        model = RobotModel(calibration_file=calibration_file)
        real_scene = None
        if self.args.mode == 'virtual':
            self.calibration_path = f'/tmp/cobot-calibration-{self.token}.json'
            from virtual_force import GazeboForceBackend
            world = model.gazebo_world(self.args.car, self.args.car_pose, self.args.probe)
            profiles=json.loads(calibration_file.read_text())
            scene=model.scene_map
            if scene is not None:
                probe_tool=scene.probe_tool
                plan=probe_tool.plan
                plan['initial_profiles'] = profiles['active'].copy()
                profiles['tcps'][plan['tcp_name']]={'pos':plan['tcp_mm_deg']}
                original=profiles['tools'][profiles['active']['tool']]
                plan['base_tool_profile'] = original
                probe_tool.session_plan_path = SESSION_PLAN
                weight=original['weight']+probe_tool.mass
                cog=[(original['weight']*value+probe_tool.mass*extra)/weight
                     for value,extra in zip(original['cog'],probe_tool.com_flange*1000)]
                profiles['tools'][plan['tool_name']]={'weight':weight,'cog':cog,'inertia':None}
                plan['tool_profile'] = dict(weight=weight, cog=cog, inertia=[0.] * 6)
                self.write_session(SESSION_PLAN, plan)
                self.write_session(SESSION/'scene_truth.json', scene.vehicle.truth)
            profile_path=CACHE/'session_calibration.json'
            profile_path.write_text(json.dumps(profiles,indent=2))
            subprocess.run(['docker','cp',str(profile_path),f'ws_cobot_pjt:{self.calibration_path}'],check=True)
            self.display = self.force_backend = GazeboForceBackend(model)
            self.displays.append(self.display)
            self.start(['gz', 'sim', '-s', *([] if self.display.probe else ['-r']), str(world)])
            if self.display.probe:
                self.display.probe.initialize()
            if not (self.args.no_gui or self.args.no_gazebo_gui):
                self.viewers.append(self.start(['gz', 'sim', '-g', '-v3',
                            '--gui-config', str(ROOT / 'native/gazebo.config'),
                            '--render-engine-gui', 'ogre2',
                            '--render-engine-gui-api-backend', 'metal']))
        elif self.args.car == 'real':
            from scene import RealDisplayScene
            model.scene_map = real_scene = RealDisplayScene(CACHE / 'scene', self.args.car_pose)
        if not self.args.no_gui:
            rviz = RVizDisplay(model, self.start)
            self.displays.append(rviz)
            self.viewers.append(rviz.viewer)
        if real_scene is not None:
            self.session_files.append(real_scene.write_reference(SESSION))
        if not self.args.observe_only and not self.robot_id:
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
        listener.bind((self.bind_host, 0))
        listener.listen(1)
        listener.settimeout(30)
        if self.force_backend:
            from virtual_force import ForceStateReceiver
            self.receiver = ForceStateReceiver(listener, self.token, self.force_backend)
            command = ('source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
                       'exec ros2 run m0609_rg2_bringup virtual_force_bridge.py '
                       f'host.docker.internal {listener.getsockname()[1]} {self.token}')
        else:
            self.receiver = StateReceiver(listener, self.token)
            self.relay_path = f'/tmp/cobot-display-{self.token}.py'
            subprocess.run(['docker', 'cp', str(ROOT / 'native/ros_state_relay.py'),
                            f'ws_cobot_pjt:{self.relay_path}'], check=True)
            command = ('source /opt/ros/jazzy/setup.bash && exec python3 '
                       f'{self.relay_path} host.docker.internal {listener.getsockname()[1]} {self.token}')
        if self.robot_id:
            from service_launch import launch_command
            if not self.force_backend:
                command += f' --ros-args -r __ns:=/{self.robot_id}'
                self.start_remote(command, 'relay')
            self.start_remote(launch_command(self, network_env, listener.getsockname()[1]), 'robot')
        else:
            self.start_remote(command, 'relay')
        if self.args.web:
            sys.path.insert(0, str(self.args.web_dir))
            from robot_session import SimulationWeb
            joints = '/dsr01/sim_joint_states' if self.args.mode == 'virtual' else '/dsr01/joint_states'
            SimulationWeb(self.args.web_dir, self.args.web_port, joints).start(self, model)
        print(f'Native {self.args.mode} session. Logs: {CACHE}/{self.token}_*.log', flush=True)
        if self.robot_id:
            print(f'/{self.robot_id}: {self.args.mode} session started', flush=True)
        elif not self.args.observe_only:
            print('Robot programs run in the container: docker exec -it ws_cobot_pjt bash -> ros2 run rokey <program>',
                  flush=True)
        start = time.monotonic()
        excursion, scene_due = {}, 0.0
        while not self.receiver.closed.is_set():
            if any(viewer.poll() == 0 for viewer in self.viewers):
                break
            if any(p.poll() is not None for p in self.processes):
                raise RuntimeError(f'A session process exited; inspect {CACHE}/{self.token}_*.log')
            positions = self.receiver.snapshot(smooth=self.args.mode == 'virtual')
            for display in self.displays:
                display.update(positions.copy())
                if display is self.force_backend:
                    feedback = self.force_backend.snapshot()
                    self.receiver.send({'kind': 'feedback', **feedback})
                    positions = feedback['positions']
            if self.force_backend and model.scene_map is not None:
                model.scene_map.probe_tool.update(positions)
                if time.monotonic() >= scene_due:
                    # Same scene as RViz, for container-side viewers (/dsr01/sim_scene, e.g. --web).
                    self.receiver.send({'kind': 'scene', 'markers': model.scene_map.marker_dicts()})
                    scene_due = time.monotonic() + 0.2
            for name, value in positions.items():
                excursion[name] = max(excursion.get(name, 0), abs(value))
            if self.args.duration and time.monotonic() - start >= self.args.duration:
                break
            time.sleep(1 / 60)
        if self.robot_id and self.receiver.closed.is_set():
            raise RuntimeError(f'/{self.robot_id}: ROS relay disconnected; inspect {CACHE}')
        positions = self.receiver.snapshot()
        result = {'mode': self.args.mode, 'ros_positions': positions,
                  'max_absolute_positions_rad': excursion}
        if self.args.mode == 'virtual':
            result['gazebo_positions'] = self.display.feedback
            result['virtual_force'] = self.force_backend.snapshot()
        if model.scene_map is not None:
            result['scene'] = model.scene_map.vehicle.truth
        (CACHE / f'last_{self.args.mode}_session.json').write_text(json.dumps(result, indent=2))
        if self.args.duration and not positions:
            raise RuntimeError('No ROS joint states received; inspect session logs')
        if self.args.duration and self.args.mode == 'virtual' and not self.display.feedback:
            raise RuntimeError('No Gazebo joint feedback received; inspect session logs')

    def close(self):
        if self.robot_id:
            try:
                self.close_processes()
            finally:
                if self.robot_network:
                    self.robot_network.close()
            return
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
        path.parent.mkdir(parents=True, exist_ok=True)
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
    parser.add_argument('--no-gazebo-gui', action='store_true',
                        help='Gazebo server only, no Gazebo window (saves ~30%% CPU); RViz stays')
    parser.add_argument('--web', action='store_true',
                        help='Start the web robot controls, URDF viewer and rosbridge (virtual or real)')
    parser.add_argument('--web-port', type=int, default=5050)
    parser.add_argument('--web-dir', type=Path, default=next(
        (d for d in (ROOT/'rokey-b3-project1/web', ROOT.parent/'web') if d.is_dir()), ROOT/'rokey-b3-project1/web'))
    from scene import DEMO, probe_types, vehicle_types
    parser.add_argument('--car', choices=[*vehicle_types(), 'random', 'real', 'performance', *DEMO],
                        help='Real mode displays the holder-mounted vehicle; virtual mode puts this vehicle into the '
                             'vehicle zone of rokey robot.yaml, with the probe rack; real = the team vehicle '
                             'assembly; performance = the fracture_sample.stl coupon with a probe rack; demo '
                             '(= demo_front), demo_front, demo_left, demo_right = the team vehicle on its holder 650 mm '
                             'ahead, the front, left door or right door toward the robot')
    parser.add_argument('--car-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
                        help='Vehicle pose in BASE (mm, deg) instead of a random pose inside the zone')
    parser.add_argument('--probe', choices=probe_types(),
                        help='Probe on the rack: standard, ball (v3 5 mm), or ball-2mm (v3 2 mm). '
                             'The demo scenes default to ball-2mm; other scenes default to standard.')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--calibration-file', type=Path, default=DEFAULT_CALIBRATION,
                        help='Real TCP/tool calibration JSON, applied in virtual mode')
    args = parser.parse_args()
    args.probe = args.probe or ('ball-2mm' if args.car in DEMO else 'standard')
    if args.mode == 'real' and args.car not in (None, 'real'):
        parser.error('Real mode supports --car real')
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
