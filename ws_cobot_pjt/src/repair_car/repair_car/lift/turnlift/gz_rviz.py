"""RViz view of the Gazebo scissor lift, Gazebo being the physics engine only. LiftUrdf writes the lift's kinematic tree
(gz_lift.py without the loop-closure stubs: Gazebo keeps the loops closed, so its tree joint angles place every link)
as URDF per payload, the robot_state_publisher parameters and an RViz config (from the robot bringup's default.rviz);
JointRelay republishes the Gazebo joint states as sensor_msgs/JointState for robot_state_publisher. Native
(cobot-native), ROS namespace /lift_native: out/rviz/scissor_lift_<payload>.{urdf, yaml}, out/rviz/lift.rviz."""
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .assembly import TREE
from .config import ROOT
from .gz_lift import COLOURS, NAME, save
from .sdf import text

NAMESPACE = '/lift_native'
RVIZ_SOURCE = ROOT.parents[2] / 'cobot_rg2/rg2/m0609_rg2_bringup/rviz/default.rviz'


class LiftUrdf:
    def __init__(self, lift):
        self.lift, self.a = lift, lift.a

    @staticmethod
    def link(robot, name, geometry, rgba, xyz=(0, 0, 0)):
        visual = ET.SubElement(ET.SubElement(robot, 'link', name=name), 'visual')
        ET.SubElement(visual, 'origin', xyz=text(xyz), rpy='0 0 0')
        visual.append(geometry)
        ET.SubElement(ET.SubElement(visual, 'material', name=name), 'color', rgba=text(rgba))

    @staticmethod
    def joint(robot, name, kind, parent, child, xyz, limit=None):
        node = ET.SubElement(robot, 'joint', name=name, type=kind)
        ET.SubElement(node, 'parent', link=parent)
        ET.SubElement(node, 'child', link=child)
        ET.SubElement(node, 'origin', xyz=text(xyz), rpy='0 0 0')
        if kind != 'fixed':
            ET.SubElement(node, 'axis', xyz='0 -1 0')
        if limit:
            ET.SubElement(node, 'limit', **{k: f'{v:.6g}' for k, v in zip(('lower', 'upper', 'effort', 'velocity'), limit)})

    @staticmethod
    def mesh(uri):
        geometry = ET.Element('geometry')
        ET.SubElement(geometry, 'mesh', filename=uri, scale='0.001 0.001 0.001')
        return geometry

    def urdf(self, payload, uri):
        """`uri(name)`: mesh URI of a link or a mesh payload. Lengths m."""
        a, lift = self.a, self.lift
        robot = ET.Element('robot', name=NAME)
        ET.SubElement(robot, 'link', name='world')
        for name in a.links:
            self.link(robot, name, self.mesh(uri(name)), COLOURS[name])
        self.joint(robot, 'fix', 'fixed', 'world', 'base', [0, 0, 0])
        for name, _, parent, child, _ in TREE:
            servo = name == 'servo'
            self.joint(robot, name, 'revolute' if servo else 'continuous', parent, child,
                       (a.origins[child] - a.origins[parent]) / 1000,
                       (0, a.lock, lift.torque, lift.speed) if servo else None)
        shape = lift.payload(payload)[3]
        if isinstance(shape, str):
            self.link(robot, 'payload', self.mesh(uri(shape)), COLOURS['payload'])
        else:
            geometry = ET.Element('geometry')
            ET.SubElement(geometry, 'box', size=text(np.asarray(shape) / 1000))
            self.link(robot, 'payload', geometry, COLOURS['payload'], [0, 0, shape[2] / 2000])
        self.joint(robot, 'payload_mount', 'fixed', 'top', 'payload', a.holder / 1000)
        return robot

    def rviz(self):
        config = yaml.safe_load(RVIZ_SOURCE.read_text())
        manager = config['Visualization Manager']
        for display in manager['Displays']:
            if display['Class'] == 'rviz_default_plugins/RobotModel':
                display['Name'], display['Description Topic']['Value'] = 'Lift', f'{NAMESPACE}/robot_description'
            if display['Class'] == 'rviz_default_plugins/Grid':
                display['Cell Size'], display['Plane Cell Count'] = .05, 12
        manager['Global Options']['Fixed Frame'] = 'world'
        view = manager['Views']['Current']
        view['Distance'], view['Focal Point'] = .55, dict(X=0, Y=0, Z=.09)
        view['Pitch'], view['Yaw'] = .35, -2.2
        return config

    def write(self, directory):
        folder = Path(directory) / 'rviz'
        meshes = Path(directory) / 'models' / NAME / 'meshes'
        uri = lambda n: (meshes / f'{n}.stl').resolve().as_uri()
        files = []
        for payload in self.lift.spec['payloads']:
            path = folder / f'{NAME}_{payload}.urdf'
            save(path, self.urdf(payload, uri))
            path.with_suffix('.yaml').write_text(yaml.safe_dump(
                {'/**': {'ros__parameters': {'robot_description': path.read_text()}}}))
            files.append(str(path))
        (folder / 'lift.rviz').write_text(yaml.safe_dump(self.rviz()))
        return dict(urdf=files, config=str(folder / 'lift.rviz'))


class JointRelay:
    """Gazebo /scissor_lift/joint_state (tree joints) -> ROS /lift_native/joint_states at up to `rate` Hz."""

    def __init__(self, rate=50.):
        import rclpy
        from sensor_msgs.msg import JointState
        from gz.msgs10.model_pb2 import Model
        from gz.transport13 import Node
        self.rclpy, self.JointState, self.period, self.last = rclpy, JointState, 1 / rate, 0.
        rclpy.init()
        self.node = rclpy.create_node('joint_relay', namespace=NAMESPACE)
        self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)
        self.names = {name for name, *_ in TREE}
        self.gz = Node()
        self.gz.subscribe(Model, f'/{NAME}/joint_state', self.on_state)

    def on_state(self, msg):
        if time.monotonic() - self.last < self.period:
            return
        self.last = time.monotonic()
        joints = [j for j in msg.joint if j.name in self.names]
        state = self.JointState(name=[j.name for j in joints], position=[j.axis1.position for j in joints])
        state.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(state)

    def run(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.rclpy.spin(self.node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            self.node.destroy_node()
            self.rclpy.try_shutdown()
