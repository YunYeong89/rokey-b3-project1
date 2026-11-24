"""Persistent BASE TCP preparation pose, independent of where the next run starts."""
import json
from pathlib import Path


class ReadyPose:
    path = Path(__file__).resolve().parents[2] / 'config' / 'force_slide_ready.json'

    def load(self):
        return json.loads(self.path.read_text())['pose_mm_deg']

    def save(self, pose):
        self.path.write_text(json.dumps({'reference': 'BASE', 'pose_mm_deg': list(pose)}, indent=2) + '\n')
