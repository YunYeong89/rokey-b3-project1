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
        if axis is None and self.metadata.get('part') == 'door' and self.metadata.get('controller') == 'mesh-height':
            axis = np.asarray(self.metadata['settings']['door_axis'], float)  # a door plan: seen along its fixed axis
        path = self.directory / 'report.png'
        report = DamageReport(nominal, measured, axis=axis, defects=defects, web=self.metadata.get('web_report'))
        if not roi_file and self.rois and self.rois.damage_samples:
            detected_types = {region['type'] for region, _ in self.rois.damage_samples}
            report.defects = [(region, points) for region, points in report.defects
                              if region['type'] not in detected_types] + self.rois.damage_samples
        self.damage = report.save(path)
        if self.metadata.get('rescan_of') and defects is not None:  # an ROI rescan: its defects drawn on the mesh too,
            survey = self.directory.parent / self.metadata['rescan_of'] / 'roi.json'  # with the survey's dents
            dents = [region for region in json.loads(survey.read_text())['regions'] if region['type'] == 'dent'] \
                if survey.exists() else []
            drawing = DefectMap(nominal, measured, defects, self.metadata, dents).save(self.directory / 'defects.png')
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
        figures = cls(directory, metadata)
        path = figures.save(records)
        result_path = directory / 'result.json'
        if result_path.is_file():
            result = json.loads(result_path.read_text())
            result['damage'] = figures.damage
            result_path.write_text(json.dumps(result, indent=2) + '\n')
        return path
