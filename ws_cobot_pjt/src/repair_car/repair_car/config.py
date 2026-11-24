"""YAML and saved robot settings in config; runtime files in data."""
from datetime import datetime
import fcntl
import json
import re
import sys
from pathlib import Path

import yaml



def namespace_args(argv=None):
    ros_args = False
    namespace = '/dsr01'
    for arg in sys.argv[1:] if argv is None else argv:
        if arg == '--ros-args':
            ros_args = True
        elif arg == '--':
            ros_args = False
        elif ros_args and arg.startswith('__ns:='):
            namespace = '/' + arg.partition(':=')[2].strip('/')
    return namespace


def fleet_id(namespace):
    name = namespace.strip('/')
    return name if re.fullmatch(r'robot_[1-9][0-9]*', name) else None


PACKAGE = Path(__file__).resolve().parents[1]
CALIBRATION_DIR = PACKAGE / 'calibration'
ROBOT_ID = fleet_id(namespace_args())
CONFIG_ROOT = PACKAGE / 'config'
CONFIG_DIR = CONFIG_ROOT / ROBOT_ID if ROBOT_ID else CONFIG_ROOT
SHARED_SESSION = PACKAGE / 'data'
SESSION = SHARED_SESSION / ROBOT_ID if ROBOT_ID else SHARED_SESSION
SAVED_SETTINGS = frozenset(('bonnet_path.json', 'offsets.json', 'poses.json'))


class SettingsFile:
    """JSON sections with robot-specific overrides and a shared fallback."""

    def __init__(self, name, directory=None, shared=None):
        self.path = Path(directory if directory is not None else CONFIG_DIR) / name
        self.shared = Path(shared if shared is not None else CONFIG_ROOT) / name

    def read(self, section=None):
        for path in dict.fromkeys((self.path, self.shared)):
            if path.exists():
                data = json.loads(path.read_text())
                if section is None:
                    return data
                if section in data:
                    return data[section]
        return None

    def write(self, data, section=None):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.path.with_name('.' + self.path.name + '.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if section is not None:
                saved = json.loads(self.path.read_text()) if self.path.exists() else {}
                saved[section] = data
                data = saved
            temporary = self.path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')
            temporary.replace(self.path)
        return self.path


def read_offset(section):
    return SettingsFile('offsets.json').read(section)


def write_offset(section, data):
    return SettingsFile('offsets.json').write(
        {**data, 'created': datetime.now().isoformat(timespec='seconds')}, section)


def session_path(name):
    return (CONFIG_DIR if name in SAVED_SETTINGS else SESSION) / name


def output_dir(name):
    path = SESSION / name if ROBOT_ID else PACKAGE / name
    path.mkdir(parents=True, exist_ok=True)
    return path


def load(name):
    return yaml.safe_load((CONFIG_ROOT / f'{name}.yaml').read_text())


def read_session(name):
    path = session_path(name)
    if name in SAVED_SETTINGS and not path.exists():
        path = CONFIG_ROOT / name
    try:
        return json.loads(path.read_text())
    except FileNotFoundError:
        return None


def write_session(name, data):
    path = session_path(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({**data, 'created': datetime.now().isoformat(timespec='seconds')}, indent=2))
    return path


class Config(dict):
    """robot.yaml with the active cell's probe profiles and saved contact calibration."""

    def __init__(self, mode):
        super().__init__(load('robot'))
        plan = read_session('probe_plan.json') if mode == 'virtual' else None
        if plan:
            self['probe'].update(tcp_name=plan['tcp_name'], tool_name=plan['tool_name'], tip_radius_mm=plan['probe_radius_mm'])
        if mode == 'real':
            cell = load('real_scene')['probe']
            probe = self['probe']
            probe.update(tool_name=cell.get('tool_weight_name') or probe['tool_name'],
                         tcp_name=cell.get('pickup_tcp_name') or probe['tcp_name'])
        result = read_offset('calibration')
        if result and result['mode'] == mode:
            self['scan']['touch_threshold'] = result['touch_threshold_n']
            self['inspection']['height_sigma_mm'] = result['height_sigma_mm']
