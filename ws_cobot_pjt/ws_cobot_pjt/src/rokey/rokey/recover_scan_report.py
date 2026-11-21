"""Restore a saved scan report from its point clouds or telemetry, without robot motion."""
import argparse
import json
from pathlib import Path

import numpy as np

from .part_scan.damage_report import DamageReport
from .part_scan.mesh_figures import MeshFigures
from .performance_sample import PerformanceSample
from .surface_follow.pointcloud_file import PointCloudFile


class SavedScanReport:
    def __init__(self, directory, plan):
        self.directory, self.plan = directory, plan

    def reference(self):
        path = self.directory / 'nominal_reference.ply'
        if path.exists():
            return PointCloudFile.load_ply(path)[:, :3] * 1000
        model = PerformanceSample(self.plan.get('source_mesh'))
        if model.source_hash != self.plan['source_hash']:
            raise ValueError('Saved scan and source mesh differ')
        return model.reference(np.asarray(self.plan['pose_base_mm']), 1.).points

    def save(self):
        stages = self.directory / 'stages.json'
        manifest = json.loads(stages.read_text()) if stages.exists() else {}
        points, axes = [], []
        for stage in ('survey', 'detail'):
            directory = self.directory / stage
            if not (directory / 'measured.ply').exists():
                continue
            plan = json.loads((directory / 'plan.json').read_text())
            axes.extend(path['axis'] for path in plan['paths'])
            cloud = PointCloudFile.load_ply(directory / 'measured.ply')
            if cloud.size:
                points.extend(cloud[:, :3] * 1000)
        axis = axes[0] if axes and np.allclose(axes, axes[:1]) else None
        path = self.directory / 'report.png'
        DamageReport(self.reference(), points, axis=axis, web=manifest.get('web_report')).save(path)
        return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--reference-ply', type=Path,
                        help='Healthy bonnet reference cloud in BASE (metres), for older mesh-height reports')
    options = parser.parse_args()
    directory = options.directory
    plan_file = directory / 'plan.json'
    plan = json.loads(plan_file.read_text()) if plan_file.exists() else {}
    controller = plan.get('controller', '').removesuffix('-stream')
    if options.reference_ply and (controller != 'mesh-height' or plan.get('part') != 'bonnet'):
        parser.error('--reference-ply requires a bonnet mesh-height report')
    if controller in ('cloud-normal', 'mesh-normal', 'mesh-height'):
        print(MeshFigures.restore(directory, reference=options.reference_ply))
    else:
        print(SavedScanReport(directory, plan).save())


if __name__ == '__main__':
    main()
