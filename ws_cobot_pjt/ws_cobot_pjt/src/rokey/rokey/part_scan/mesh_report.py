"""Mesh plan previews and position-stream telemetry; force is recorded without thresholds."""
import csv
import json

import numpy as np

from ..surface_follow.pointcloud_file import PointCloudFile
from .mesh_figures import MeshFigures


class MeshReport:
    def __init__(self, directory, name):
        self.directory = directory / name
        self.directory.mkdir(parents=True, exist_ok=True)
        self.rois = None

    def plan(self, plan, roi_reference=None, roi_cloud_settings=None, *, nominal=None, web_report=None):
        source = getattr(plan.surface, 'geometry_source', 'mesh')
        checks = plan.planning_checks if source == 'pointcloud' else dict(
            tool_collision=True, arm_collision=True, transit=True, mesh_projection=True)
        self.metadata = dict(controller=getattr(plan, 'controller',
                                               'cloud-normal' if source == 'pointcloud' else 'mesh-normal'),
                             planning_source=source, planning_checks=checks,
                             planning_reference=getattr(plan.surface, 'planning_reference', source),
                             roi_local_mm=getattr(plan.surface, 'roi_local_mm', None),
                             regions_local_mm=getattr(plan.surface, 'regions_local_mm', None),
                             force_feedback_used=True, frame='base_link', units='mm',
                             speed_mm_s=plan.scan['speed_mm_s'],
                             cache_hit=plan.cache_hit, planning_seconds=plan.planning_seconds,
                             full_reference_points=plan.reference_points,
                             settings=plan.scan['mesh_normal'],
                             report_settings=plan.scan['report'],
                             part=plan.surface.part,
                             roi_settings=plan.scan['default_scan']['roi'],
                             contact_threshold_n=plan.scan['default_scan']['contact_threshold_n'],
                             target_normal_force_n=plan.scan['default_scan']['survey']['force_n'],
                             rejected=plan.rejected,
                             diagnostics=plan.diagnostics, pose_failures=plan.reach.failures,
                             paths=[path.trajectory.export() for path in plan.paths])
        if roi_reference is not None:
            reference = PointCloudFile.save_ply(self.directory / 'roi_reference.ply', roi_reference / 1000)
            self.metadata['roi_reference_cloud'] = reference.name
            self.metadata['roi_cloud_settings'] = roi_cloud_settings
        if nominal is not None:
            self.metadata['nominal_reference_cloud'] = self.metadata.get('roi_reference_cloud') if nominal is roi_reference else (
                PointCloudFile.save_ply(self.directory / 'nominal_reference.ply', nominal / 1000).name)
        if web_report:
            self.metadata['web_report'] = web_report
        if plan.scan.get('roi_base_mm'):
            self.metadata['roi_base_mm'] = plan.scan['roi_base_mm']
        if plan.scan.get('rescan_of'):  # the survey this ROI rescan follows (real_scan.rescan): strict, drawn on the mesh
            self.metadata['rescan_of'] = plan.scan['rescan_of']
        (self.directory / 'plan.json').write_text(json.dumps(self.metadata, indent=2))
        MeshFigures(self.directory, self.metadata).save()
        return self.directory

    def route(self, route):
        (self.directory / 'route.json').write_text(json.dumps([
            dict(phase=phase, path=number, **trajectory.export())
            for trajectory, phase, number in route], indent=2))

    def save(self, records, completed, timings=None, *, approaches=None, failure=None):
        columns = ('t_s,path,phase,x_mm,y_mm,z_mm,target_x_mm,target_y_mm,target_z_mm,'
                   'normal_x,normal_y,normal_z,fx_n,fy_n,fz_n,mx_nm,my_nm,mz_nm,'
                   'normal_force_n,tcp_error_mm,orientation_error_deg,sensor_age_s,'
                   'vx_mm_s,vy_mm_s,vz_mm_s,wx_deg_s,wy_deg_s,wz_deg_s,'
                   'q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg').split(',')
        columns += ['pd_enabled', 'pd_saturated']
        for group in ('error', 'p', 'd', 'offset'):
            columns += [f'pd_{group}_{axis}_{unit}' for unit, axes in (('mm', ('x', 'y', 'z')),
                                                                      ('deg', ('rx', 'ry', 'rz')))
                        for axis in axes]
        columns += [f'pd_velocity_error_{axis}_{unit}'
                    for unit, axes in (('mm_s', ('x', 'y', 'z')), ('deg_s', ('rx', 'ry', 'rz'))) for axis in axes]
        columns += [f'{group}_{axis}_{unit}' for group, unit in
                    (('ball', 'mm'), ('target_ball', 'mm'), ('ball_velocity', 'mm_s')) for axis in ('x', 'y', 'z')]
        with (self.directory / 'samples.csv').open('w', newline='') as output:
            writer = csv.writer(output)
            writer.writerow(columns)
            writer.writerows(records)
        metrics = dict(completed=completed, force_feedback_used=self.metadata.get('force_feedback_used', False),
                       samples=len(records), figure='report.png')
        if approaches is not None:
            metrics['approaches'] = approaches
        if failure is not None:
            metrics['failure'] = failure
        if timings is not None:
            metrics['timing'] = dict(
                planned_strokes_s=sum(path['duration_s'] for path in self.metadata['paths']),
                executed_strokes=timings)
        if records:
            values = np.array([[row[0], *row[18:22]] for row in records], float)
            metrics.update(max_tcp_error_mm=float(values[:, 2].max()),
                           max_orientation_error_deg=float(values[:, 3].max()),
                           max_sensor_age_s=float(values[:, 4].max()))
            velocity = np.array([row[22:28] for row in records], float)
            metrics.update(max_linear_speed_mm_s=float(np.linalg.norm(velocity[:, :3], axis=1).max()),
                           max_angular_speed_deg_s=float(np.linalg.norm(velocity[:, 3:], axis=1).max()))
            ball = np.array([row[66:75] for row in records], float)
            metrics['ball'] = dict(
                motion_reference=self.metadata.get('settings', {}).get('ball_radius_mm', 0.) > 0,
                rms_error_mm=float(np.sqrt(np.mean(np.sum((ball[:, :3] - ball[:, 3:6])**2, axis=1)))),
                speed_p50_p95_mm_s=np.percentile(np.linalg.norm(ball[:, 6:], axis=1), [50, 95]).tolist())
            pd = np.array([row[34:66] for row in records], float)
            if pd.shape[1] and np.any(pd[:, 0] > 0):
                active = pd[:, 0] > 0
                metrics['pd'] = dict(
                    integral_enabled=False, saturated_share=float(np.mean(pd[active, 1] > 0)),
                    rms_position_error_mm=float(np.sqrt(np.mean(np.sum(pd[active, 2:5]**2, axis=1)))),
                    rms_orientation_error_deg=float(np.sqrt(np.mean(np.sum(pd[active, 5:8]**2, axis=1)))),
                    paths={})
                for path in sorted(set(row[1] for row in records)):
                    own = np.array([row[1] == path for row in records]) & active
                    if not own.any():
                        continue
                    selected = np.flatnonzero(own)
                    end = values[selected[-1], 0]
                    tail = selected[values[selected, 0] >= end - .5]
                    tail_metrics = dict(
                        endpoint_mean_position_error_mm=np.mean(pd[tail, 2:5], axis=0).tolist(),
                        endpoint_mean_orientation_error_deg=np.mean(pd[tail, 5:8], axis=0).tolist(),
                        endpoint_saturated_share=float(np.mean(pd[tail, 1] > 0)))
                    finished = next((item['completed'] for item in timings or [] if item['path'] == path), completed)
                    if not finished:
                        tail_metrics = {key.replace('endpoint_', 'last_window_'): value
                                        for key, value in tail_metrics.items()}
                    metrics['pd']['paths'][str(path)] = dict(completed=finished, **tail_metrics)
        figures = MeshFigures(self.directory, self.metadata)
        try:
            return figures.save(records)
        finally:
            self.rois = figures.rois
            metrics['damage'] = figures.damage
            (self.directory / 'result.json').write_text(json.dumps(metrics, indent=2))
