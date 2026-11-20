"""Native display adapters; the robot controller remains in Docker."""
import json
from pathlib import Path
import subprocess
import sys
import socket
import threading
import time

from joint_interpolator import JointInterpolator


class StateReceiver:
    def __init__(self, listener, token):
        self.listener, self.token = listener, token
        self.positions = {}
        self.interpolator = JointInterpolator()
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.stream = None
        threading.Thread(target=self.receive, daemon=True).start()

    def receive(self):
        try:
            self.stream, _ = self.listener.accept()
            # Small 100 Hz state lines: without it Nagle held them for the container's delayed ACKs (arrivals in
            # bursts, gaps up to 140 ms). The container end already sets it.
            self.stream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.stream.settimeout(30)
            with self.stream.makefile('r') as file:
                if file.readline().strip() != self.token:
                    raise ConnectionError('Invalid display session token')
                for line in file:
                    with self.lock:
                        positions = json.loads(line)
                        self.positions.update(positions)
                        self.interpolator.receive(positions, time.monotonic())
        except OSError:
            pass
        finally:
            self.closed.set()

    def snapshot(self, smooth=False):
        with self.lock:
            if smooth:
                return self.interpolator.snapshot(time.monotonic())
            return dict(self.positions)

    def close(self):
        self.listener.close()
        if self.stream is not None:
            self.stream.close()


class GazeboDisplay:
    def __init__(self, model):
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.model_pb2 import Model
        from gz.transport13._transport import AdvertiseMessageOptions, Node, SubscribeOptions
        self.Double, self.Model = Double, Model
        self.node = Node()
        self.publishers = {name: self.node.advertise(
            f'/cobot/{name}/cmd_pos', 'gz.msgs.Double', AdvertiseMessageOptions())
            for name in model.joints}
        self.feedback = {}
        self.feedback_velocity = {}
        self.node.subscribe_raw('/cobot/gazebo_joint_states', self.receive,
                                'gz.msgs.Model', SubscribeOptions())
        self.model = model

    def receive(self, data, _info):
        message = self.Model.FromString(data)
        self.feedback = {j.name: j.axis1.position for j in message.joint}
        self.feedback_velocity = {j.name: j.axis1.velocity for j in message.joint}

    def update(self, positions):
        for name, (source, multiplier, offset) in self.model.mimics.items():
            positions[name] = positions.get(source, 0) * multiplier + offset
        for name, value in positions.items():
            if name in self.publishers:
                payload = self.Double(data=value).SerializeToString()
                self.publishers[name].publish_raw(payload, 'gz.msgs.Double')

    def close(self):
        pass


class RVizDisplay:
    def __init__(self, model, start_process):
        import rclpy
        import yaml
        from sensor_msgs.msg import JointState
        self.rclpy, self.JointState = rclpy, JointState
        self.model = model
        rclpy.init()
        self.node = rclpy.create_node('cobot_native_joint_source', namespace='/cobot_native')
        self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)
        scene_map = getattr(model, 'scene_map', None)
        self.scene_map = scene_map
        if scene_map is not None:
            from visualization_msgs.msg import MarkerArray
            from rclpy.qos import DurabilityPolicy, QoSProfile
            self.map_publisher = self.node.create_publisher(
                MarkerArray, '/cobot_native/map',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.map_markers = scene_map.markers()
            self.map_timer = self.node.create_timer(0.1, self.publish_map)
            self.publish_map()
            from clouds import CloudFiles
            self.clouds = CloudFiles(self.node)
        parameters = model.urdf.parent / 'robot_state_publisher.yaml'
        parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'robot_description': model.urdf.read_text(), 'use_sim_time': False}}}))
        executable = Path(sys.prefix) / 'lib/robot_state_publisher/robot_state_publisher'
        start_process([str(executable),
                       '--ros-args', '--params-file', str(parameters),
                       '-r', '__ns:=/cobot_native'])
        from rcl_interfaces.srv import SetParameters
        self.description_client = self.node.create_client(
            SetParameters, '/cobot_native/robot_state_publisher/set_parameters')
        self.description_version = model.urdf.stat().st_mtime_ns
        root = Path(__file__).resolve().parent.parent
        source = (root / 'ws_cobot_pjt/src/cobot_rg2/rg2/m0609_rg2_bringup/rviz/default.rviz')
        config = yaml.safe_load(source.read_text())
        for display in config['Visualization Manager']['Displays']:
            if display.get('Class') == 'rviz_default_plugins/RobotModel':
                display['Description Topic']['Value'] = '/cobot_native/robot_description'
        config['Visualization Manager']['Global Options']['Fixed Frame'] = 'world'
        if scene_map is not None:
            config['Visualization Manager']['Displays'].append({
                'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Car scene',
                'Enabled': True, 'Value': True,
                'Topic': {'Value': '/cobot_native/map', 'Depth': 1,
                                 'Reliability Policy': 'Reliable', 'Durability Policy': 'Transient Local'},
            })
            config['Visualization Manager']['Displays'] += CloudFiles.displays()
        target = model.urdf.parent / 'native.rviz'
        target.write_text(yaml.safe_dump(config))
        self.viewer = start_process(['rviz2', '-d', str(target)])

    def publish_map(self):
        self.map_markers = self.scene_map.markers()
        for marker in self.map_markers.markers:
            marker.header.stamp = self.node.get_clock().now().to_msg()
        self.map_publisher.publish(self.map_markers)

    def update(self, positions):
        version = self.model.urdf.stat().st_mtime_ns
        if version != self.description_version and self.description_client.service_is_ready():
            from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
            from rcl_interfaces.srv import SetParameters
            self.description_client.call_async(SetParameters.Request(parameters=[Parameter(
                name='robot_description', value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=self.model.urdf.read_text()))]))
            self.description_version = version
        message = self.JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = list(positions)
        message.position = list(positions.values())
        self.publisher.publish(message)
        self.rclpy.spin_once(self.node, timeout_sec=0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
