"""Point clouds the container's robot programs write to repair_car/data/clouds (<name>.npy: float32 BASE metres,
repair_car part_scan CloudChannel) as PointCloud2 for the native RViz: the container's ROS topics do not reach it."""
from array import array
import os
from pathlib import Path
import time

import numpy as np

CLOUDS = Path(os.environ.get('COBOT_SESSION_DIR',
                             Path(__file__).resolve().parents[4] / 'ws_cobot_pjt/src/repair_car/data')) / 'clouds'
COLOURS = {'reference': '90; 150; 255', 'measured': '255; 110; 30'}


class CloudFiles:
    """Publishes a cloud file whenever it changes (checked once a second; files older than the session ignored)."""

    def __init__(self, node):
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import PointCloud2
        self.node, self.since, self.seen = node, time.time(), {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publishers = {name: node.create_publisher(PointCloud2, f'/cobot_native/cloud/{name}', latched)
                           for name in COLOURS}
        self.timer = node.create_timer(1.0, self.poll)

    def poll(self):
        for name, publisher in self.publishers.items():
            path = CLOUDS / f'{name}.npy'
            stamp = path.stat().st_mtime if path.exists() else 0.
            if stamp > max(self.since, self.seen.get(name, 0.)):
                self.seen[name] = stamp
                publisher.publish(self.message(np.load(path).astype(np.float32).reshape(-1, 3)))

    def message(self, points):
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import Header
        return PointCloud2(header=Header(stamp=self.node.get_clock().now().to_msg(), frame_id='base_link'), height=1,
                           width=len(points), is_bigendian=False, point_step=12, row_step=12 * len(points),
                           is_dense=True, data=array('B', points.tobytes()),
                           fields=[PointField(name=axis, offset=4 * index, datatype=PointField.FLOAT32, count=1)
                                   for index, axis in enumerate('xyz')])

    @staticmethod
    def displays():
        return [{'Class': 'rviz_default_plugins/PointCloud2', 'Name': f'Cloud {name}', 'Enabled': True, 'Value': True,
                 'Topic': {'Value': f'/cobot_native/cloud/{name}', 'Depth': 1, 'Reliability Policy': 'Reliable',
                           'Durability Policy': 'Transient Local'},
                 'Color Transformer': 'FlatColor', 'Color': colour, 'Style': 'Points', 'Size (Pixels)': 3,
                 'Alpha': 1.0}
                for name, colour in COLOURS.items()]
