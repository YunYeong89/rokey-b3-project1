"""Run Linux ROS control in Docker and native Gazebo/RViz joint-state displays."""
import argparse
import os
import secrets
import signal
import socket
import subprocess
import time

from displays import GazeboDisplay, RVizDisplay, StateReceiver
from model_export import CACHE, ROOT, RobotModel


class NativeSession:
    def __init__(self, args):
        self.args = args
        self.processes, self.logs, self.pidfiles, self.displays, self.viewers = [], [], [], [], []
        self.receiver = None
        self.controller_output = None
        self.token = secrets.token_hex(16)
        self.compose = ['docker', 'compose', '-f', str(ROOT/'compose.yaml')]
        self.env = {**os.environ, 'GZ_PARTITION': f'cobot_{self.token}',
                    'GZ_IP': '127.0.0.1', 'ROS_DOMAIN_ID': '129'}
        for key in ['GZ_PARTITION', 'GZ_IP', 'ROS_DOMAIN_ID']:
            os.environ[key] = self.env[key]
        for key in ['DISPLAY', 'MUJOCO_GL', 'PYOPENGL_PLATFORM', 'LIBGL_ALWAYS_SOFTWARE']:
            self.env.pop(key, None)

    def start(self, command):
        log = (CACHE/f'{self.token}_{len(self.logs)}.log').open('w')
        self.logs.append(log)
        process = subprocess.Popen(command, env=self.env, stdout=log, stderr=log,
                                   start_new_session=True)
        self.processes.append(process)
        return process

    def start_remote(self, command, label):
        pidfile = f'/tmp/cobot-{self.token}-{label}.pid'
        self.pidfiles.append(pidfile)
        return self.start([*self.compose, 'exec', '-T', 'cobot', 'setsid', '--wait',
                           'bash', '-lc', f'echo $$ > {pidfile}; {command}'])

    def run(self):
        subprocess.run([*self.compose, '--profile', 'virtual', 'up', '-d', '--wait'], check=True)
        model = RobotModel()
        world = model.gazebo_world()
        self.displays.append(GazeboDisplay(model))
        self.start(['gz', 'sim', '-s', '-r', str(world)])
        if not self.args.no_gui:
            self.viewers.append(self.start([
                'gz', 'sim', '-g', '-v3', '--gui-config', str(ROOT/'native/gazebo.config'),
                '--render-engine-gui', 'ogre2', '--render-engine-gui-api-backend', 'metal']))
            rviz = RVizDisplay(model, self.start)
            self.displays.append(rviz)
            self.viewers.append(rviz.viewer)
        setup = 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
        self.start_remote(setup + 'exec ros2 launch m0609_rg2_bringup bringup.launch.py '
                          'mode:=virtual host:=emulator port:=12345 gui:=false', 'controller')
        self.controller_output = open(self.logs[-1].name)
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(30)
        self.receiver = StateReceiver(listener, self.token)
        self.start_remote(setup + 'exec python3 /opt/rokey-native/ros_state_relay.py '
                          f'host.docker.internal {listener.getsockname()[1]} {self.token}', 'relay')
        print(f'Native virtual session. Logs: {CACHE}/{self.token}_*.log', flush=True)
        started = time.monotonic()
        while not self.receiver.closed.is_set():
            output = self.controller_output.read()
            if output:
                print(output, end='', flush=True)
            if any(viewer.poll() == 0 for viewer in self.viewers):
                break
            if any(process.poll() is not None for process in self.processes):
                raise RuntimeError(f'Session process exited; inspect {CACHE}/{self.token}_*.log')
            positions = self.receiver.snapshot(smooth=True)
            for display in self.displays:
                display.update(positions.copy())
            if self.args.duration and time.monotonic()-started >= self.args.duration:
                break
            time.sleep(1/60)
        if not self.receiver.snapshot():
            raise RuntimeError('No ROS joint states received; inspect session logs')

    def close(self):
        for pidfile in self.pidfiles:
            subprocess.run([*self.compose, 'exec', '-T', 'cobot', 'bash', '-c',
                            f'if [ -f {pidfile} ]; then kill -INT -- -$(cat {pidfile}) 2>/dev/null || true; fi'],
                           check=False)
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
        if self.controller_output:
            self.controller_output.close()
        subprocess.run([*self.compose, '--profile', 'virtual', 'stop'], check=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-gui', action='store_true')
    parser.add_argument('--duration', type=float)
    args = parser.parse_args()
    session = NativeSession(args)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    try:
        session.run()
    except KeyboardInterrupt:
        pass
    finally:
        for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]:
            signal.signal(sig, signal.SIG_IGN)
        session.close()


if __name__ == '__main__':
    main()
