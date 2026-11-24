"""Read real controller profiles without starting ROS control or requesting authority."""
import argparse
import json
from pathlib import Path
import subprocess
import uuid
from datetime import datetime

from robot_network import RobotNetwork

ROOT = Path(__file__).resolve().parents[4]
COMPOSE = Path(__file__).resolve().parents[1] / 'Docker/compose.yaml'


class RobotProfileInspection:
    def __init__(self, host):
        self.network = RobotNetwork(host, uuid.uuid4().hex)

    def run(self, output):
        environment = self.network.start()
        source = Path(__file__).with_name('robot_profile_probe.cpp')
        probe = self.network.probe
        subprocess.run(['docker', 'cp', str(source), f'ws_cobot_pjt:{probe}.cpp'], check=True)
        common = '/workspace/src/cobot_rg2/doosan-robot2/dsr_common2'
        subprocess.run(['docker', 'exec', 'ws_cobot_pjt', 'g++', '-O2', '-std=c++17',
                        '-DDRCF_VERSION=2', f'-I{common}/include', f'{probe}.cpp',
                        f'{common}/lib/jazzy/arm64/libDRFL.a', '-lPocoNet',
                        '-lPocoFoundation', '-lpthread', '-o', probe], check=True)
        command = ['docker', 'exec', '-e', f'LD_PRELOAD={self.network.library}']
        for key, value in environment.items():
            command.extend(['-e', f'{key}={value}'])
        result = subprocess.run([*command, 'ws_cobot_pjt', 'timeout', '15',
                                 probe, self.network.host], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        cache = ROOT / 'ws_cobot_pjt/src/repair_car/native/.cache'
        cache.mkdir(exist_ok=True)
        (cache / 'real_profiles.log').write_text(result.stdout)
        print(result.stdout, end='', flush=True)
        result.check_returncode()
        line = next(line for line in result.stdout.splitlines()
                    if line.startswith('COBOT_PROFILE_RESULT='))
        data = json.loads(line.split('=', 1)[1])
        (cache / 'real_profiles.json').write_text(json.dumps(data, indent=2))
        self.save_calibration(data, output)
        return data

    def save_calibration(self, data, output, captured_at=None):
        config = data['safety_configuration']
        if not config:
            raise RuntimeError('Controller did not return TCP/tool configuration')
        calibration = {
            'schema_version': 1,
            'source': {'kind': 'real_controller', 'host': self.network.host,
                       'controller_version': data['controller_version'],
                       'captured_at': captured_at or datetime.now().astimezone().isoformat()},
            'units': {'tcp_position': 'mm', 'tcp_orientation': 'deg (Euler ZYZ)',
                      'tool_weight': 'kg', 'tool_cog': 'mm'},
            'active': {'tcp': data['tcp'], 'tool': data['tool']},
            'tcps': {entry['name']: {'pos': entry['pos_mm_deg']} for entry in config['tcps']},
            'tools': {entry['name']: {'weight': entry['weight_kg'], 'cog': entry['cog_mm'],
                                     'inertia': entry['inertia']} for entry in config['tools']},
        }
        if (calibration['active']['tcp'] not in calibration['tcps']
                or calibration['active']['tool'] not in calibration['tools']):
            raise RuntimeError('Active TCP/tool is missing from controller profiles')
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(calibration, indent=2) + '\n', encoding='utf-8')
        temporary.replace(output)
        print(f'Calibration saved: {output}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.100')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'ws_cobot_pjt/src/repair_car/config/calibration.json')
    args = parser.parse_args()
    state = subprocess.check_output(['docker', 'inspect', '-f', '{{.State.Running}}',
                                     'ws_cobot_pjt'], text=True).strip()
    started = state != 'true'
    if started:
        subprocess.run(['docker', 'compose', '-f', str(COMPOSE),
                        'up', '-d', 'cobot'], check=True)
    inspection = RobotProfileInspection(args.host)
    try:
        inspection.run(args.output)
    finally:
        inspection.network.close()
        if started:
            # Preserve a real session started by the user during the inspection.
            active = subprocess.run(['docker', 'exec', 'ws_cobot_pjt', 'pgrep', '-f',
                                     '/opt/ros/jazzy/bin/ros2 launch'], capture_output=True)
            if active.returncode == 1:
                subprocess.run(['docker', 'compose', '-f', str(COMPOSE),
                                'stop', 'cobot'], check=False)


if __name__ == '__main__':
    main()
