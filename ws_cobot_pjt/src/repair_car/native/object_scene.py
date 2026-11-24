"""--object scene (virtual only): the table plane and movable YCB objects of scene.json `objects` (native/objects, taken
from the ros2_mujoco object catalog). Gazebo draws each textured OBJ and collides its CoACD convex parts; RViz and the
web show the STL copies at the poses Gazebo reports (/world/cobot_world/pose/info)."""
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from kinematics import KinematicChain
from probe_mount import ProbeMount
from scene import SCENE, TableScene, material, mesh_marker, pose_matrix, rpy, vector, write_stl

ROOT = Path(__file__).resolve().parents[4]


def object_names():
    return list(json.loads(SCENE.read_text(encoding='utf-8'))['objects']['items'])


def obj_triangles(path):
    """Triangles (n x 3 x 3) of a plain OBJ (v and f lines)."""
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith('v '):
            vertices.append(line.split()[1:4])
        elif line.startswith('f '):
            faces.append([int(index.split('/')[0]) - 1 for index in line.split()[1:4]])
    return np.array(vertices, dtype=float)[faces]


class SceneObject:
    """One catalog object (metres, origin at its bounding-box centre, resting pose kept) standing on the table."""

    def __init__(self, name, spec, catalog, files, table_z_mm, drop_mm, cache):
        info = files['files'][f'{name}.obj']
        size = np.array(info['size_mm']) / 1000
        self.model = f'object_{name}'
        self.mesh = catalog / f'{name}.obj'
        self.texture = catalog / info['texture'] if 'texture' in info else None
        # The CoACD parts carry no normals: Gazebo's DART drops such OBJ submeshes and then crashes on the empty
        # collision mesh. Binary STL copies have facet normals.
        self.parts = [cache / f'{self.model}_part{index}.stl'
                      for index in range(len(files['collision']['objects'][name]['parts']))]
        for path, part in zip(self.parts, files['collision']['objects'][name]['parts']):
            write_stl(path, obj_triangles(catalog / part))
        self.stl = cache / f'{self.model}.stl'  # the web finds session meshes by file name under native/.cache/scene
        shutil.copyfile(catalog / 'STL' / f'{name}.stl', self.stl)
        x, y, yaw = spec['pose_base_mm_deg']
        self.pose = pose_matrix(x, y, yaw, table_z_mm + size[2] * 500 + drop_mm)
        self.pose[:3, 3] /= 1000
        self.mass, self.color = spec['mass_kg'], spec['color']
        self.inertia = self.mass / 12 * ((size ** 2).sum() - size ** 2)  # solid box of the object's extents

    def add_to_world(self, world, world_base):
        model = ET.SubElement(world, 'model', name=self.model)
        placed = world_base @ self.pose
        ET.SubElement(model, 'pose').text = vector([*placed[:3, 3], *rpy(placed[:3, :3])])
        link = ET.SubElement(model, 'link', name='body')
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = str(self.mass)
        inertia = ET.SubElement(inertial, 'inertia')
        for key, value in zip(['ixx', 'iyy', 'izz'], self.inertia):
            ET.SubElement(inertia, key).text = str(value)
        visual = ET.SubElement(link, 'visual', name='visual')
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh'), 'uri').text = self.mesh.resolve().as_uri()
        if self.texture is None:
            material(visual, self.color)
        else:  # the OBJ names no material file: its texture goes in as the PBR albedo map
            node = ET.SubElement(visual, 'material')
            for tag in ['ambient', 'diffuse']:
                ET.SubElement(node, tag).text = '1 1 1 1'
            metal = ET.SubElement(ET.SubElement(node, 'pbr'), 'metal')
            ET.SubElement(metal, 'albedo_map').text = str(self.texture.resolve())
            ET.SubElement(metal, 'metalness').text = '0'
            ET.SubElement(metal, 'roughness').text = '0.8'
        for index, part in enumerate(self.parts):
            collision = ET.SubElement(link, 'collision', name=f'part_{index}')
            ET.SubElement(ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh'), 'uri').text = \
                part.resolve().as_uri()

    def marker(self, frame):
        marker = mesh_marker(frame, self.model, self.stl, self.pose, self.color)
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0  # the catalog is in metres
        return marker


class ObjectScene(TableScene):
    """The --car scene's interface for session.py, displays.py and the web, without a vehicle or a probe rack."""
    probe_tool = vehicle = None

    def __init__(self, robot_model, cache, names):
        self.config = json.loads(SCENE.read_text(encoding='utf-8'))
        spec = self.config['objects']
        self.frame, cache = self.config['frame'], Path(cache)
        cache.mkdir(parents=True, exist_ok=True)
        transforms, _ = KinematicChain(robot_model.robot).transforms({})
        self.world_base = transforms[self.frame]
        catalog = ROOT / spec['catalog']
        files = json.loads((catalog / 'source.json').read_text())
        table = self.config['plane']['z_m'] * 1000
        self.objects = {name: SceneObject(name, spec['items'][name], catalog, files, table, spec['drop_mm'], cache)
                        for name in names or spec['items']}
        self.follow()
        print(f'Scene: objects {list(self.objects)}', flush=True)

    def follow(self):
        """Move the markers with the object poses Gazebo reports (models object_<name>, world frame)."""
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node, SubscribeOptions
        base = np.linalg.inv(self.world_base)
        models = {item.model: item for item in self.objects.values()}

        def receive(data, _info):
            for pose in Pose_V.FromString(data).pose:
                if pose.name in models:
                    models[pose.name].pose = base @ ProbeMount.matrix(pose)
        self.node = Node()
        self.node.subscribe_raw('/world/cobot_world/pose/info', receive, 'gz.msgs.Pose_V', SubscribeOptions())

    def add_to_world(self, world, robot_model):
        self.add_plane(world)
        for item in self.objects.values():
            item.add_to_world(world, self.world_base)

    def markers(self):
        """Republished 10 / s to RViz and 5 / s to the container, at the latest object poses."""
        from visualization_msgs.msg import MarkerArray
        return MarkerArray(markers=[self.plane_marker(), *(item.marker(self.frame) for item in self.objects.values())])
