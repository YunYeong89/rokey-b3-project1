"""Inspection identity and filenames shared by the web launcher and scan reports."""
from datetime import datetime
import json
import os
from pathlib import Path
import re
from zoneinfo import ZoneInfo

if __package__ == 'part_scan':
    from config import output_dir
else:
    from repair_car.config import output_dir


class InspectionArchive:
    ENV = 'REPAIR_CAR_WEB_SCAN_VEHICLE'
    PLATE = re.compile(r'(?:^|\s)(\d{2,3}\s*[가-힣]\s*\d{4})$')

    @classmethod
    def vehicle(cls, vehicle):
        display = vehicle['name'].strip()
        plate = cls.PLATE.search(display)
        return dict(name=(display[:plate.start()].strip() if plate else display) or vehicle['model'],
                    plate=plate[1] if plate else 'unregistered', model=vehicle['model'],
                    owner=vehicle.get('owner_name') or 'unassigned',
                    started_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat(timespec='seconds'))

    @classmethod
    def environment(cls, vehicle):
        return {**os.environ, cls.ENV: json.dumps(vehicle)} if vehicle else None

    @classmethod
    def metadata(cls):
        value = os.environ.get(cls.ENV)
        if not value:
            return None
        vehicle = json.loads(value)
        if 'name' in vehicle:
            return vehicle
        details = cls.vehicle(dict(name=vehicle['plate'], model=vehicle['model']))
        details['started_at'] = vehicle.get('started_at', details['started_at'])
        return details

    @staticmethod
    def directory(vehicle, part, side=None):
        root = output_dir('data')
        if not vehicle:
            return root
        identity = '_'.join(re.sub(r'[\\/:*?"<>|\x00-\x1f]+', '_', vehicle[key]).strip(' .') or 'unassigned'
                            for key in ('name', 'plate', 'owner'))
        stamp = datetime.fromisoformat(vehicle['started_at']).strftime('%Y-%m-%d-%H-%M')
        return root / identity / stamp / (f'{part}_{side}' if side else part)


class ScanFiles:
    """Use a filename prefix for a web inspection, or the existing per-run layout."""

    def __init__(self, directory, prefix=''):
        self.directory, self.prefix = directory, prefix

    @classmethod
    def from_plan(cls, path):
        path = Path(path)
        prefix = path.stem.removesuffix('_plan') if path.name.endswith('_plan.json') else ''
        return cls(path.parent, prefix)

    @property
    def run(self):
        return self.prefix or self.directory.name

    def path(self, filename):
        return self.directory / (f'{self.prefix}_{filename}' if self.prefix else filename)
