"""Measured BASE contact points and controller traces, separate from existing inspection outputs."""
from array import array
import csv
from dataclasses import asdict
from datetime import datetime
import json

import numpy as np
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from ..configs import PACKAGE
from .pointcloud_file import PointCloudFile
from .samples import PointSpacing, measurement


class Recording:
    def __init__(self, node, settings, path):
        self.node, self.s, self.path = node, settings, path
        self.rows, self.points, self.events = [], [], []
        self.point_spacing = PointSpacing(settings.spacing)
        # Published cloud kept packed (float32 x, y, z): rebuilding it from the point list every 0.2 s took 8 ms at
        # 11k points (a full defect scan) and 14 ms at 20k, inside the 25 ms control loop.
        self.xyz = array('f')
        self.last_phase = ''
        self.name = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + settings.method
        self.publisher = node.create_publisher(PointCloud2, 'surface_follow/points',
                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status = node.create_publisher(String, 'surface_follow/status', 10)
        self.next_publish = 0.

    def record(self, elapsed, controller, state, dt):
        c = controller
        if c.phase != self.last_phase:
            self.events.append(dict(time_s=elapsed, phase=c.phase))
            self.last_phase = c.phase
        row, point = measurement(self.s, self.path, elapsed, c, state, dt)
        if row['valid'] and self.point_spacing.accept(point):
            self.points.append(point)
            self.xyz.extend(point[:3])
        self.rows.append(row)
        if elapsed >= self.next_publish:
            self.next_publish = elapsed + .2
            self.status.publish(String(data=json.dumps(dict(method=self.s.method, phase=c.phase,
                force_n=c.force, progress_m=self.path.progress, length_m=self.path.length, points=len(self.points)))))
            header = Header(stamp=self.node.get_clock().now().to_msg(), frame_id='base_link')
            self.publisher.publish(self.cloud(header))

    def cloud(self, header):
        count = len(self.xyz) // 3
        return PointCloud2(header=header, height=1, width=count, is_bigendian=False, point_step=12,
                           row_step=12 * count, is_dense=True, data=array('B', self.xyz.tobytes()),
                           fields=[PointField(name=axis, offset=4 * index, datatype=PointField.FLOAT32, count=1)
                                   for index, axis in enumerate('xyz')])

    def save(self, outcome, reason, stop_confirmed, native_state=None):
        directory = PACKAGE / 'surface_runs' / self.name
        directory.mkdir(parents=True, exist_ok=True)
        if self.rows:
            with (directory / 'signals.csv').open('w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=list(self.rows[0]))
                writer.writeheader()
                writer.writerows(self.rows)
        clouds = PACKAGE / 'pointcloud'
        clouds.mkdir(exist_ok=True)
        cloud = PointCloudFile.save(clouds / f'{self.name}.{self.s.pointcloud_format}', self.points)
        summary = dict(outcome=outcome, reason=reason, stop_motion_confirmed=stop_confirmed,
            settings=asdict(self.s), points=len(self.points), travelled_m=self.path.progress, events=self.events,
            measured_peak_n=max((r['raw_magnitude_n'] for r in self.rows), default=0.), pointcloud=str(cloud),
            limitation='Native Gazebo only. Stop confirmation concerns motion; a position hold may retain contact load.')
        track = [r for r in self.rows if r['phase'] == 'TRACK']
        if track:
            error = np.array([r['fn_n'] - self.s.force for r in track])
            summary.update(force_mean_n=float(self.s.force + error.mean()), force_std_n=float(error.std()),
                           force_rmse_n=float(np.sqrt(np.mean(error ** 2))),
                           tracking_duration_s=track[-1]['time_s'] - track[0]['time_s'])
        summary['native_peak_n'] = max(max((r['raw_peak_n'] or 0. for r in self.rows), default=0.),
                                      (native_state or {}).get('control', {}).get('raw_peak_n', 0.))
        summary['native_before_stop'] = native_state
        (directory / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        return directory
