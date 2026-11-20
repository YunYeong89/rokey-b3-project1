"""Save or restore a mesh scan's point-cloud and damage report."""
import csv
import json
import shutil
from pathlib import Path

import numpy as np

from ..surface_follow.pointcloud_file import PointCloudFile
from .damage_report import DamageReport
from .defect_map import DefectMap
from .mesh_roi import MeshRoi


class MeshFigures:
    def __init__(self, directory, metadata):
        self.directory, self.metadata = directory, metadata
        self.rois, self.damage = None, None

    def load_cloud(self, filename):
        return PointCloudFile.load_ply(self.directory / filename)[:, :3] * 1000 if filename else []

    def save(self, records=None):
        roi_file = self.metadata.get('roi_reference_cloud')
        nominal_file = self.metadata.get('nominal_reference_cloud') or roi_file
        nominal = self.load_cloud(nominal_file)
        measured, axis, defects = [], None, None
        if records:
            reference = (nominal if roi_file == nominal_file else self.load_cloud(roi_file)) if roi_file else None
            self.rois = rois = MeshRoi(records, self.metadata, reference=reference)
            part = self.metadata.get('part', self.directory.name.split('_')[1]
                                     if self.directory.name.startswith('real_') else 'scan')
            rois.save(self.directory / 'roi.json', part)
            rois.save(self.directory.parent / f'{self.directory.name}_roi.json', part)
            rows = rois.records
            radii = {i: path.get('ball_radius_mm', self.metadata.get('settings', {}).get('ball_radius_mm', 0.))
                     for i, path in enumerate(self.metadata.get('paths', []), 1)}
            contact = (rows[:, 4] > rois.threshold) & np.fromiter((row[2] == 'stroke' for row in records), bool)
            if contact.any():
                normals = rows[contact, 5:8]
                radius = np.array([radii.get(int(path), 0.) for path in rows[contact, 8]])
                measured = rois.position[contact] - radius[:, None] * normals
                if np.allclose(normals, normals[:1]) and np.linalg.norm(normals[0]) > 0:
                    axis = normals[0]
            if roi_file:
                defects = rois.damage_samples
        path = self.directory / 'report.png'
        self.damage = DamageReport(nominal, measured, axis=axis, defects=defects,
                                   web=self.metadata.get('web_report')).save(path)
        if self.metadata.get('rescan_of') and defects is not None:  # an ROI rescan: its defects drawn on the mesh too,
            drawing = DefectMap(nominal, measured, defects, self.metadata).save(self.directory / 'defects.png')
            shutil.copy(drawing, self.directory.parent / f'{self.directory.name}_defects.png')  # beside its _roi.json
        return path

    @classmethod
    def restore(cls, directory, reference=None):
        directory = Path(directory)
        metadata = json.loads((directory / 'plan.json').read_text())
        if reference is not None:
            metadata['roi_reference_cloud'] = str(reference.resolve())
        with (directory / 'samples.csv').open() as source:
            reader = csv.reader(source)
            next(reader)
            records = [[value if index == 2 else float(value) for index, value in enumerate(row)]
                       for row in reader]
        return cls(directory, metadata).save(records)
