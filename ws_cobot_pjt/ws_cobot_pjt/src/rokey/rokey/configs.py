"""Configuration of the robot programs: rokey/config/*.yaml plus results shared through /workspace/session."""
from datetime import datetime
import json
from pathlib import Path

import yaml

if __package__:
    from .robot_namespace import fleet_id, namespace_args
else:
    from robot_namespace import fleet_id, namespace_args

PACKAGE = Path(__file__).resolve().parents[1]
CALIBRATION_DIR = PACKAGE / 'calibration'
ROBOT_ID = fleet_id(namespace_args())
SESSION = PACKAGE.parents[1] / 'session'
if ROBOT_ID:
    SESSION /= ROBOT_ID


def output_dir(name):
    path = SESSION / name if ROBOT_ID else PACKAGE / name
    path.mkdir(parents=True, exist_ok=True)
    return path


def load(name):
    return yaml.safe_load((PACKAGE / 'config' / f'{name}.yaml').read_text())


def read_session(name):
    path = SESSION / name
    return json.loads(path.read_text()) if path.exists() else None


def write_session(name, data):
    SESSION.mkdir(parents=True, exist_ok=True)
    path = SESSION / name
    path.write_text(json.dumps({**data, 'created': datetime.now().isoformat(timespec='seconds')}, indent=2))
    return path


class Config(dict):
    """robot.yaml with the virtual session plan and the latest calibration result of this mode applied."""

    def __init__(self, mode):
        super().__init__(load('robot'))
        plan = read_session('probe_plan.json') if mode == 'virtual' else None
        if plan:
            self['probe'].update(tcp_name=plan['tcp_name'], tool_name=plan['tool_name'], tip_radius_mm=plan['probe_radius_mm'])
        result = read_session('calibration_result.json')
        if result and result['mode'] == mode:
            self['scan']['touch_threshold'] = result['touch_threshold_n']
            self['inspection']['height_sigma_mm'] = result['height_sigma_mm']
