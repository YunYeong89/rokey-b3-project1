"""Vehicle point clouds for RViz (frame base_link, metres):
  /repair/pointcloud    full reference cloud of the identified vehicle at its pose, coloured by part and
                        re-coloured by the inspection (ok green, suspect yellow, dent red, missing grey,
                        replaced blue)
  /repair/scan_points   the measured points, coloured by state
Inputs: /repair/vehicle (choose_car; session/vehicle.json at start) and /repair/status point messages
(INTERFACE.md frame) plus {"type": "replaced", "part"} from replace_parts.
  ros2 run rokey pointcloud [--spacing 2]
"""
import argparse
import json

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

if __package__:
    from .configs import load, read_session
    from .vehicle_model import VehicleModel, pose_matrix, transform
else:
    from configs import load, read_session
    from vehicle_model import VehicleModel, pose_matrix, transform

PARTS = {'body': (150, 155, 165), 'bonnet': (90, 140, 200), 'mirror_l': (60, 170, 170), 'mirror_r': (60, 170, 170),
         'wheel': (40, 40, 40)}
STATES = {'ok': (40, 200, 60), 'suspect': (250, 210, 30), 'dent': (230, 40, 30), 'missing': (110, 110, 110),
          'replaced': (40, 90, 250)}
FIELDS = [PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1) for index, name in
          enumerate('xyz')] + [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]


def cloud(header, points_mm, colors):
    data = np.zeros(len(points_mm), dtype=point_cloud2.dtype_from_fields(FIELDS))
    for index, axis in enumerate('xyz'):
        data[axis] = np.asarray(points_mm)[:, index] / 1000.0
    colors = colors.astype(np.uint32)
    data['rgb'] = colors[:, 0] << 16 | colors[:, 1] << 8 | colors[:, 2]
    return point_cloud2.create_cloud(header, FIELDS, data)


class VehicleCloud:
    """Reference samples of one vehicle in BASE with per-point part labels and colours. A measured point
    re-colours the reference points within half the scan spacing, so a raster colours the whole part."""

    def __init__(self, vehicle, spacing):
        inspection = load('robot')['inspection']
        self.radius = max(inspection['line_spacing_mm'], inspection['point_spacing_mm']) / 2
        self.model = VehicleModel(vehicle['type'])
        self.pose = pose_matrix(*vehicle['pose_base_mm_deg'], vehicle['table_z_mm'])
        local, self.parts = self.model.sample(spacing)
        self.points = transform(self.pose, local)
        self.colors = np.array([PARTS[part] for part in self.parts], dtype=np.uint8)
        self.measured, self.measured_colors = [], []

    def mark(self, message):
        """A /repair/status point: (x, y) in the INTERFACE frame, dz from the reference surface."""
        local = self.model.from_interface([message['x'], message['y'], 0.0])[0]
        reference, _ = self.model.surface(local[:2])
        state = 'missing' if message.get('missing') else message['state']
        point = transform(self.pose, [local[0], local[1], np.nan_to_num(reference) + message['dz']])
        anchor = transform(self.pose, [local[0], local[1], np.nan_to_num(reference)])
        near = np.linalg.norm(self.points - anchor, axis=1) < self.radius
        self.colors[near] = STATES[state]
        self.measured.append(point)
        self.measured_colors.append(STATES[state])

    def replaced(self, part):
        self.colors[self.parts == part] = STATES['replaced']


class PointCloudNode:
    def __init__(self, spacing):
        self.node = rclpy.create_node('repair_pointcloud')
        self.spacing, self.vehicle, self.dirty = spacing, None, False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference = self.node.create_publisher(PointCloud2, '/repair/pointcloud', latched)
        self.scan = self.node.create_publisher(PointCloud2, '/repair/scan_points', latched)
        self.node.create_subscription(String, '/repair/vehicle', self.on_vehicle, latched)
        self.node.create_subscription(String, '/repair/status', self.on_status, 100)
        self.node.create_timer(0.5, self.publish)
        saved = read_session('vehicle.json')
        if saved:
            self.load(saved)

    def load(self, vehicle):
        self.vehicle = VehicleCloud(vehicle, self.spacing)
        self.dirty = True
        self.node.get_logger().info(f'{vehicle["type"]} at {np.round(vehicle["pose_base_mm_deg"], 1).tolist()}: '
                                    f'{len(self.vehicle.points)} reference points')

    def on_vehicle(self, message):
        self.load(json.loads(message.data))

    def on_status(self, message):
        data = json.loads(message.data)
        if self.vehicle is None:
            return
        if data.get('type') == 'point':
            self.vehicle.mark(data)
        elif data.get('type') == 'replaced':
            self.vehicle.replaced(data['part'])
        self.dirty = True

    def publish(self):
        if not self.dirty:
            return
        header = Header(stamp=self.node.get_clock().now().to_msg(), frame_id='base_link')
        self.reference.publish(cloud(header, self.vehicle.points, self.vehicle.colors))
        if self.vehicle.measured:
            self.scan.publish(cloud(header, np.array(self.vehicle.measured),
                                    np.array(self.vehicle.measured_colors, dtype=np.uint8)))
        self.dirty = False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--spacing', type=float, default=2.0, help='reference point spacing, mm (default: 2)')
    options, _ = parser.parse_known_args()
    rclpy.init()
    node = PointCloudNode(options.spacing)
    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        pass
    finally:
        node.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
