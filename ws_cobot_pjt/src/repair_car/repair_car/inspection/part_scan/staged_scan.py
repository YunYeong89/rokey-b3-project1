"""Default-controller survey and ROI detail scans using the existing DSR force/motion services."""
from copy import deepcopy
import json

import numpy as np

from repair_car.pointcloud.pointcloud_file import PointCloudFile
from repair_car.report.report import ScanReport
from repair_car.inspection.part_scan.run import AxisScraper
from repair_car.inspection.part_scan.staged_roi import RoiPlan, SurveyRoi


class StageScraper(AxisScraper):
    """Existing force-controlled strokes; depth is telemetry only and never changes motion."""
    def __init__(self, program, scan, channel, plan):
        super().__init__(program, scan, channel)
        self.threshold = scan['default_scan']['contact_threshold_n']
        self.plan, self.control = plan, []
        if program.mode == 'virtual' and self.transport is None:
            from repair_car.inspection.surface_follow.transport import VirtualTransport
            self.transport = VirtualTransport(program)
            self.transport.wait_ready()
            self.sequence = -1

    def measure(self, axis):
        record = super().measure(axis)
        def until(pose, wrench):
            previous = len(self.records)
            record(pose, wrench)
            if len(self.records) != previous:
                row = self.records[-1]
                position = np.asarray(row[1:4])
                depth = float(self.grid.ball_at(position.reshape(1, 3))[0] - position @ axis)
                self.control.append([row[0], self.path, depth, self.scan['force_n']])
            return False
        return until

    def run(self, segments, axis, straight):
        self.grid = next(grid for own, grid in self.plan.grids if np.allclose(own, axis))
        super().run(segments, axis, straight)


class DefaultScan:
    def __init__(self, program, scan, plan, channel, report, evaluation, part, clouds, stamp):
        self.program, self.scan, self.plan, self.channel = program, scan, plan, channel
        self.report, self.evaluation, self.part = report, evaluation, part
        self.clouds, self.stamp = clouds, stamp
        self.root = report.directory / report.name
        self.root.mkdir(exist_ok=True)
        self.settings = scan['default_scan']
        self.stages, self.points = [], []
        self.damage = None

    @staticmethod
    def survey_settings(scan, *, real=False):
        result = deepcopy(scan)
        result.update(scan['default_scan']['survey'])
        if real:
            result['speed_mm_s'] = min(result['speed_mm_s'], scan['default_scan']['real_survey_max_speed_mm_s'])
            result['force_n'] = result['approach_force_n'] = scan['default_scan']['real_force_n']
        result['approach_force_n'] = min(result['approach_force_n'], result['force_n'])
        return result

    def execute(self, plan, scan, name, report, evaluation=None):
        scraper = StageScraper(self.program, scan, self.channel, plan)
        stage = dict(name=name, completed=False, force_n=scan['force_n'], press_along_axis_n=-scan['force_n'],
                     speed_mm_s=scan['speed_mm_s'], pitch_mm=scan['pitch_mm'], paths=len(plan.paths))
        self.stages.append(stage)
        self.program.topics.plan([[np.linalg.norm(np.diff(p, axis=0), axis=1).sum() for p in segments]
                                  for segments, _, _ in plan.sweeps], scan['speed_mm_s'])
        try:
            for number, (segments, axis, straight) in enumerate(plan.sweeps, 1):
                self.program.log(f'default {name} {number}/{len(plan.sweeps)}: '
                                 f'press {-scan["force_n"]:g} N, {scan["speed_mm_s"]:g} mm/s')
                scraper.run(segments, axis, straight)
            stage['completed'] = True
        finally:
            try:
                self.program.probe.close()
            finally:
                self.points.extend(scraper.points)
                rois = self.save_stage(scraper, plan, stage, name, report, evaluation)
        return scraper, rois

    def save_stage(self, scraper, plan, stage, name, report, evaluation):
        rois = SurveyRoi(scraper, plan.grids, self.settings['roi'])
        stage.update(roi_count=len(rois.regions), samples=len(scraper.records), stored_points=len(scraper.points),
                     point_candidates=scraper.sampler.candidates, point_decimation=scraper.sampler.decimation,
                     missed_telemetry_samples=scraper.missed_samples)
        path = self.root / name
        path.mkdir(exist_ok=True)
        PointCloudFile.save(path / 'measured.ply', scraper.points)
        np.savetxt(path / 'samples.csv', np.asarray(scraper.records).reshape(-1, 9), delimiter=',',
                   header='t_s,x_mm,y_mm,z_mm,force_n,axis_x,axis_y,axis_z,path', comments='')
        rois.save(path / 'roi.json', self.part)
        topics = self.program.topics
        if name == 'survey':
            roi_file = rois.save(report.directory / f'{report.name}_roi.json', self.part)
            topics.regions(rois.regions, rois.limit)
            topics.record(roi_file=str(roi_file))
        (path / 'plan.json').write_text(json.dumps(
            dict(profile=stage, reference='original accessible surface',
                 paths=[dict(axis=axis.tolist(), points_mm=points.tolist()) for points, axis in plan.paths]),
            ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        np.savetxt(path / 'control.csv', np.asarray(scraper.control).reshape(-1, 4), delimiter=',',
                   header='t_s,path,depth_mm,target_force_n', comments='')
        self.save_manifest()
        topics.reports(path / 'samples.csv')
        if scraper.records:
            if name == 'survey':
                topics.reports(report.samples(scraper.records))
            else:
                report.save(scraper.points, self.axis(plan))
                topics.reports(report.path)
            if evaluation and scraper.points:
                self.program.log(str(evaluation.save(scraper, stage['completed'], plan)))
        return rois

    def save_manifest(self):
        manifest = dict(controller='default', settings=self.settings, stages=self.stages, damage=self.damage)
        if self.report.web_report:
            manifest['web_report'] = self.report.web_report
        (self.root / 'stages.json').write_text(json.dumps(
            manifest, ensure_ascii=False, indent=2, allow_nan=False) + '\n')

    @staticmethod
    def axis(plan):
        return plan.grids[0][0] if len(plan.grids) == 1 else None

    def save_report(self):
        self.damage = self.report.save(self.points, self.axis(self.plan))
        self.save_manifest()
        self.program.topics.reports(self.report.path)

    def run(self):
        real = self.program.mode == 'real'
        survey = self.survey_settings(self.scan, real=real)
        _, rois = self.execute(self.plan, survey, 'survey', self.report, self.evaluation)
        detail = deepcopy(survey)
        detail.update(self.settings['detail'])
        if real:
            detail['force_n'] = self.settings['real_force_n']
        detail['approach_force_n'] = min(detail['approach_force_n'], detail['force_n'])
        detail['min_path_mm'] = detail['measurable']['min_segment_mm'] = 2.
        plan = RoiPlan(self.plan, rois.regions, detail)
        if plan.paths:
            report = ScanReport(self.root, 'detail', self.report.reference, self.report.web_report)
            self.program.topics.reports(*report.plan(plan.maps))
            self.execute(plan, detail, 'detail', report)
        else:
            self.stages.append(dict(name='detail', completed=True, paths=0, reason='no_reachable_roi_paths'))
            self.save_manifest()
            self.program.log('default: no ROI path to rescan')
        self.channel.write('measured', self.points)
        cloud = PointCloudFile.save(self.clouds / f'real_{self.part}_{self.stamp}_measured.ply', self.points)
        self.program.topics.record(measured_cloud=str(cloud), points=len(self.points))
        self.program.home(scan_travel=True)
