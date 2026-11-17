"""Mesh plan previews and position-stream telemetry; force is recorded without thresholds."""
import csv
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class MeshReport:
    def __init__(self, directory, name):
        self.directory = directory / name
        self.directory.mkdir(parents=True, exist_ok=True)

    def plan(self, plan):
        source = getattr(plan.surface, 'geometry_source', 'mesh')
        checks = plan.planning_checks if source == 'pointcloud' else dict(
            tool_collision=True, arm_collision=True, transit=True, mesh_projection=True)
        self.metadata = dict(controller='cloud-normal' if source == 'pointcloud' else 'mesh-normal',
                             planning_source=source, planning_checks=checks,
                             force_feedback_used=False, frame='base_link', units='mm',
                             cache_hit=plan.cache_hit, planning_seconds=plan.planning_seconds,
                             full_reference_points=plan.reference_points,
                             settings=plan.scan['mesh_normal'],
                             rejected=plan.rejected,
                             diagnostics=plan.diagnostics, pose_failures=plan.reach.failures,
                             paths=[path.trajectory.export() for path in plan.paths])
        (self.directory / 'plan.json').write_text(json.dumps(self.metadata, indent=2))
        fig = plt.figure(figsize=(12, 9))
        panel = fig.add_subplot(111, projection='3d')
        if source == 'pointcloud':
            points = plan.surface.points
            stride = max(1, len(points) // 20000)
            panel.scatter(*points[::stride].T, s=.3, color='.7', alpha=.3)
        else:
            triangles = plan.surface.triangles[plan.surface.target]
            stride = max(1, len(triangles) // 2500)
            panel.add_collection3d(Poly3DCollection(triangles[::stride], facecolor='.75', alpha=.25, linewidth=0))
            points = triangles.reshape(-1, 3)
        for index, path in enumerate(plan.paths):
            for span in path.trajectory.spans:
                s = np.linspace(0., span.length, min(1000, max(2, int(span.length * 2))))
                path_points = span.position(s)
                panel.plot(*path_points.T, linewidth=.8, color=f'C{index % 10}')
                knots = span.checkpoints
                panel.scatter(*knots.T, s=5, color=f'C{index % 10}')
                every = max(1, len(knots) // 12)
                direction = span.rotation(span.knots[::every]).as_matrix()[:, :, 2]
                panel.quiver(*knots[::every].T, *direction.T, length=5., color='tab:red', linewidth=.6)
        low, high = points.min(0), points.max(0)
        panel.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), zlim=(low[2], high[2]),
                  xlabel='BASE x (mm)', ylabel='BASE y (mm)', zlabel='BASE z (mm)',
                  title=f'{source} paths / adaptive checkpoints / TCP +Z (red); force measurement only')
        panel.set_box_aspect(np.maximum(high - low, 1.))
        fig.tight_layout()
        fig.savefig(self.directory / 'plan.png', dpi=130)
        plt.close(fig)
        return self.directory

    def route(self, route):
        (self.directory / 'route.json').write_text(json.dumps([
            dict(phase=phase, path=number, **trajectory.export())
            for trajectory, phase, number in route], indent=2))

    def save(self, records, completed):
        columns = ('t_s,path,phase,x_mm,y_mm,z_mm,target_x_mm,target_y_mm,target_z_mm,'
                   'normal_x,normal_y,normal_z,fx_n,fy_n,fz_n,mx_nm,my_nm,mz_nm,'
                   'normal_force_n,tcp_error_mm,orientation_error_deg,sensor_age_s,'
                   'vx_mm_s,vy_mm_s,vz_mm_s,wx_deg_s,wy_deg_s,wz_deg_s,'
                   'q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg').split(',')
        with (self.directory / 'samples.csv').open('w', newline='') as output:
            writer = csv.writer(output)
            writer.writerow(columns)
            writer.writerows(records)
        metrics = dict(completed=completed, force_feedback_used=False, samples=len(records))
        if records:
            values = np.array([[row[0], *row[18:22]] for row in records], float)
            metrics.update(max_tcp_error_mm=float(values[:, 2].max()),
                           max_orientation_error_deg=float(values[:, 3].max()),
                           max_sensor_age_s=float(values[:, 4].max()))
            velocity = np.array([row[22:28] for row in records], float)
            metrics.update(max_linear_speed_mm_s=float(np.linalg.norm(velocity[:, :3], axis=1).max()),
                           max_angular_speed_deg_s=float(np.linalg.norm(velocity[:, 3:], axis=1).max()))
            fig, panels = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
            t = values[:, 0] - values[0, 0]
            for panel, data, label in zip(panels, values[:, 1:4].T,
                                           ('Measured normal force (N)', 'TCP error (mm)', 'Orientation error (deg)')):
                panel.plot(t, data, linewidth=.6)
                panel.set_ylabel(label)
            panels[-1].set_xlabel('Time (s)')
            fig.tight_layout()
            fig.savefig(self.directory / 'telemetry.png', dpi=120)
            plt.close(fig)
        (self.directory / 'result.json').write_text(json.dumps(metrics, indent=2))
