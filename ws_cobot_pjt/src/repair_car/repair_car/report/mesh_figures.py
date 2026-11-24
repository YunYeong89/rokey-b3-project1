"""Save or restore a mesh scan's point-cloud and damage report."""
import csv
import json
import shutil
from pathlib import Path

import numpy as np

from repair_car.pointcloud.pointcloud_file import PointCloudFile
from repair_car.inspection.part_scan.archive import ScanFiles
from repair_car.report.damage_report import DamageReport
from repair_car.report.defect_map import DefectMap
from repair_car.inspection.part_scan.mesh_roi import MeshRoi


class MeshFigures:
    def __init__(self, files, metadata):
        self.files, self.metadata = files, metadata
        self.rois, self.damage = None, None

    def load_cloud(self, filename):
        return PointCloudFile.load_ply(self.files.directory / filename)[:, :3] * 1000 if filename else []

    def save(self, records=None):
        roi_file = self.metadata.get('roi_reference_cloud')
        nominal_file = self.metadata.get('nominal_reference_cloud') or roi_file
        nominal = self.load_cloud(nominal_file)
        measured, axis, defects = [], None, None
        if records:
            reference = (nominal if roi_file == nominal_file else self.load_cloud(roi_file)) if roi_file else None
            self.rois = rois = MeshRoi(records, self.metadata, reference=reference)
            part = self.metadata.get('part', self.files.run.split('_')[1]
                                     if self.files.run.startswith('real_') else 'scan')
            rois.save(self.files.path('roi.json'), part)
            if not self.files.prefix:
                rois.save(self.files.directory.parent / f'{self.files.run}_roi.json', part)
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
        path = self.files.path('report.png')
        report = DamageReport(nominal, measured, axis=axis, defects=defects, web=self.metadata.get('web_report'))
        if not roi_file and self.rois and self.rois.damage_samples:
            detected_types = {region['type'] for region, _ in self.rois.damage_samples}
            report.defects = [(region, points) for region, points in report.defects
                              if region['type'] not in detected_types] + self.rois.damage_samples
        self.report, self.damage = report, report.save(path)
        return path

    def save_mesh(self):
        survey_run = self.metadata.get('rescan_of')
        directory = self.files.directory
        dents = []
        if survey_run:
            survey = (ScanFiles(directory, survey_run) if self.files.prefix else
                      ScanFiles(directory.parent / survey_run)).path('roi.json')
            if survey.exists():
                dents = [region for region in json.loads(survey.read_text())['regions']
                         if region['type'] == 'dent' and region['confirmed']]
        defects = [(region, points) for region, points in self.report.defects if region['confirmed']]
        drawing = DefectMap(self.report.reference, self.report.measured, defects, self.metadata, dents).save(
            self.files.path('predicted_mesh.png' if self.files.prefix or not survey_run else 'defects.png'))
        if survey_run and not self.files.prefix:
            shutil.copy(drawing, directory.parent / f'{self.files.run}_defects.png')
        return drawing

    @classmethod
    def restore(cls, plan_file, reference=None):
        plan_file = Path(plan_file)
        if plan_file.is_dir():
            plan_file /= 'plan.json'
        files = ScanFiles.from_plan(plan_file)
        metadata = json.loads(plan_file.read_text())
        if reference is not None:
            metadata['roi_reference_cloud'] = str(reference.resolve())
        with files.path('samples.csv').open() as source:
            reader = csv.reader(source)
            next(reader)
            records = [[value if index == 2 else float(value) for index, value in enumerate(row)]
                       for row in reader]
        figures = cls(files, metadata)
        path = figures.save(records)
        result_path = files.path('result.json')
        if result_path.is_file():
            result = json.loads(result_path.read_text())
            result['damage'] = figures.damage
            if result.get('completed') and (metadata.get('rescan_of') or
                                            files.path('predicted_mesh.png').is_file()):
                figures.save_mesh()
            result_path.write_text(json.dumps(result, indent=2) + '\n')
        return path
