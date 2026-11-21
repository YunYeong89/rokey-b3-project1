"""device.yaml and project paths."""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'out'


def load(path=None):
    return yaml.safe_load(Path(path or ROOT / 'config/device.yaml').read_text())
