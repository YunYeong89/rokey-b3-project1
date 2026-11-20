"""Convert an in-memory robot/scene copy to MJCF; source URDF and meshes stay untouched."""
from copy import deepcopy
import hashlib
import json
from itertools import combinations, product
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import mujoco
import manifold3d
import numpy as np
import trimesh

from model_export import CACHE, RobotModel


def values(value):
    return ' '.join(map(str, np.asarray(value).ravel()))


def pose_attributes(matrix):
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.asarray(matrix[:3, :3]).ravel())
    return {'pos': values(matrix[:3, 3]), 'quat': values(quat)}


class VehicleCollision:
    """Partition original surface triangles into local convex solids; never join disconnected CAD objects.

    Each original triangle belongs to a leaf hull. Subdivision removes hull faces lying outside the source
    solid farther than tolerance. Open CAD sheets get at most 0.04 mm collision thickness.
    """
    tolerance = .0003
    version = 'surface-partition-v4'

    def __init__(self, meshes, cache):
        self.meshes = sorted(map(Path, meshes))
        digest = hashlib.sha256(f'{self.version}:{self.tolerance}'.encode())
        for path in self.meshes:
            digest.update(path.read_bytes())
        self.key = digest.hexdigest()
        self.directory = Path(cache) / 'collision' / self.key[:20]
        self.directory.mkdir(parents=True, exist_ok=True)

    def build(self):
        manifest = self.directory / 'manifest.json'
        archive = Path(__file__).with_name('assets') / f'vehicle_{self.key[:20]}.zip'
        if not manifest.exists() and archive.exists():
            with ZipFile(archive) as bundle:
                metadata = json.loads(bundle.read('manifest.json'))
                if metadata['source_hash'] != self.key:
                    raise ValueError('Bundled vehicle collision geometry does not match the source mesh')
                bundle.extractall(self.directory)
        if not manifest.exists():
            mesh = trimesh.util.concatenate([trimesh.load(path, force='mesh') for path in self.meshes])
            mesh.apply_scale(.001)
            mesh.merge_vertices(digits_vertex=7)
            mesh.update_faces(mesh.nondegenerate_faces(height=1e-10))
            mesh.remove_unreferenced_vertices()
            components = sorted(mesh.split(only_watertight=False, repair=False), key=lambda part: -len(part.faces))
            names = []
            for index, part in enumerate(components):
                entry = self.directory / f'component_{index}.json'
                if entry.exists():
                    names.extend(json.loads(entry.read_text()))
                    continue
                generated = []
                for number, hull in enumerate(self.partition(part)):
                    name = f'component_{index}_{number}.obj'
                    self.regularize(hull).export(self.directory / name, digits=12)
                    generated.append(name)
                entry.write_text(json.dumps(generated))
                names.extend(generated)
                print(f'Vehicle collision: component {index + 1}/{len(components)}, {len(generated)} hulls', flush=True)
            manifest.write_text(json.dumps({'source_hash': self.key, 'method': self.version,
                'tolerance_m': self.tolerance, 'sheet_thickness_m': .00004,
                'components': len(components), 'files': names, 'prepared': True}, indent=2))
        self.info = json.loads(manifest.read_text())
        if not self.info.get('prepared'):
            for name in self.info['files']:
                path = self.directory / name
                hull = trimesh.load(path, force='mesh', process=False)
                regular = self.regularize(hull)
                if regular is not hull:
                    regular.export(path, digits=12)
            self.info['prepared'] = True
            self.info['numerical_padding_max_m'] = float(np.sqrt(3) * .00001)
            manifest.write_text(json.dumps(self.info, indent=2))
        digest = hashlib.sha256(self.key.encode())
        for name in self.info['files']:
            digest.update((self.directory / name).read_bytes())
        self.info['collision_digest'] = digest.hexdigest()
        validation = self.directory / 'validation.json'
        checked = json.loads(validation.read_text()) if validation.exists() else {}
        self.info['validated'] = checked.get('passed', False) and checked.get('collision_digest') == self.info['collision_digest']
        return [self.directory / name for name in self.info['files']]

    @staticmethod
    def regularize(hull):
        """Avoid float32 collapse of CAD slivers in MuJoCo, adding at most 10 micrometres per minor axis."""
        points = hull.vertices
        _, _, axes = np.linalg.svd(points - points.mean(0), full_matrices=False)
        widths = np.ptp((points - points.mean(0)) @ axes.T, axis=0)
        thin = axes[widths < .00002]
        if not len(thin):
            return hull
        offsets = np.array(list(product([-1, 1], repeat=len(thin)))) @ thin * .00001
        return trimesh.convex.convex_hull(np.concatenate([points + offset for offset in offsets]))

    def partition(self, part):
        pending = [np.arange(len(part.faces))]
        solid = manifold3d.Manifold(manifold3d.Mesh(np.asarray(part.vertices, np.float32),
                                                   np.asarray(part.faces, np.uint32))) if part.is_watertight else None
        if solid is not None and solid.status() != manifold3d.Error.NoError:
            solid = None
        centres = part.triangles_center
        while pending:
            faces = pending.pop()
            points = part.vertices[np.unique(part.faces[faces])]
            _, singular, axes = np.linalg.svd(points - points.mean(0), full_matrices=False)
            if singular[-2] / np.sqrt(len(points)) < .00001:
                points = np.concatenate([points + a * axes[-1] * .00002 + b * axes[-2] * .00002
                                         for a in [-1, 1] for b in [-1, 1]])
            elif len(points) < 4 or singular[-1] < 1e-9:
                normal = axes[-1]
                points = np.r_[points + normal * .00002, points - normal * .00002]
            hull = trimesh.convex.convex_hull(points)
            if len(faces) == 1:
                # Source triangle padded by at most 0.02 mm per minor axis; nearest-point arithmetic on
                # CAD sliver triangles is ill-conditioned, whereas this construction has a known bound.
                yield hull
                continue
            # Midpoints also detect holes lying along hull-face edges, rather than at face centroids.
            samples = np.r_[hull.triangles_center, hull.vertices[hull.edges_unique].mean(1)]
            outside = ~self.near_surface(part, samples)
            if solid is not None and outside.any():
                direction = np.array([1., .371, .529])
                inside = []
                for point in samples[outside]:
                    hits = solid.ray_cast(point, point + direction)
                    distances = np.array([hit.distance for hit in hits])
                    count = int(np.count_nonzero(np.r_[True, np.diff(distances) > 1e-8])) if len(hits) else 0
                    inside.append(count % 2 == 1)
                outside[outside] &= ~np.asarray(inside)
            if not outside.any():
                yield hull
                continue
            axis = int(np.argmax(np.ptp(centres[faces], axis=0)))
            order = faces[np.argsort(centres[faces, axis])]
            half = len(order) // 2
            pending.extend([order[:half], order[half:]])

    def near_surface(self, part, samples):
        """Exact point/triangle checks inside tolerance AABBs; no repeated full-mesh nearest-vertex tree."""
        rows, faces = [], []
        for index, point in enumerate(samples):
            candidates = list(part.triangles_tree.intersection(np.r_[point - self.tolerance, point + self.tolerance]))
            rows.extend([index] * len(candidates))
            faces.extend(candidates)
        near = np.zeros(len(samples), dtype=bool)
        if rows:
            rows = np.asarray(rows)
            closest = trimesh.triangles.closest_point(part.triangles[faces], samples[rows])
            distance = np.linalg.norm(closest - samples[rows], axis=1)
            np.logical_or.at(near, rows, distance <= self.tolerance)
        return near


class MuJoCoRobotModel(RobotModel):
    def gazebo_world(self, car=None, car_pose=None, probe='standard'):
        # The session interface uses this name; no Gazebo executable or transport is used here.
        if car is not None:
            from scene import CarScene
            self.scene_map = CarScene(self, CACHE / 'scene', car, car_pose, probe)
        self.mujoco_path = ModelBuilder(self).build()
        return self.mujoco_path


class ModelBuilder:
    def __init__(self, robot):
        self.robot = robot
        self.cache = CACHE / 'mujoco'
        self.cache.mkdir(exist_ok=True)

    def mesh_path(self, filename):
        source = Path(unquote(urlparse(filename).path)) if filename.startswith('file:') else Path(filename)
        target = self.cache / (hashlib.sha256(str(source).encode()).hexdigest()[:16] + '.obj')
        if not target.exists() or target.stat().st_mtime < source.stat().st_mtime:
            mesh = trimesh.load(source, force='scene')
            mesh.to_mesh().export(target)
        return str(target.resolve())

    def build(self):
        robot = deepcopy(self.robot.robot)
        for mesh in robot.iter('mesh'):
            mesh.set('filename', self.mesh_path(mesh.get('filename')))
        for joint in robot.findall('joint'):
            mimic = joint.find('mimic')
            if mimic is not None:
                joint.remove(mimic)
        extension = ET.SubElement(robot, 'mujoco')
        ET.SubElement(extension, 'compiler', fusestatic='false', discardvisual='false', balanceinertia='true')
        path = self.cache / 'robot.urdf'
        ET.ElementTree(robot).write(path, encoding='unicode')
        imported = mujoco.MjModel.from_xml_path(str(path))
        path = self.cache / 'robot.xml'
        mujoco.mj_saveLastXML(str(path), imported)
        self.root = ET.parse(path).getroot()
        self.world = self.root.find('worldbody')
        self.assets = self.root.find('asset')
        if self.assets is None:
            self.assets = ET.SubElement(self.root, 'asset')
        option = ET.SubElement(self.root, 'option', timestep='0.002', integrator='implicitfast', gravity='0 0 -9.81')
        ET.SubElement(option, 'flag', sleep='enable')
        # Keep controlled trees awake while allowing MuJoCo to cache static vehicle geom transforms.
        parents = {child: node for node in self.root.iter() for child in node}
        for body in self.root.findall('.//body'):
            if body.find('joint') is None and body.find('freejoint') is None:
                continue
            ancestor = parents.get(body)
            while ancestor is not None and ancestor.tag != 'worldbody':
                if ancestor.find('joint') is not None or ancestor.find('freejoint') is not None:
                    break
                ancestor = parents.get(ancestor)
            if ancestor is not None and ancestor.tag == 'worldbody':
                body.set('sleep', 'never')
        ET.SubElement(self.world, 'light', pos='0 0 3', dir='0 0 -1', diffuse='.8 .8 .8')
        self.equality = ET.SubElement(self.root, 'equality')
        for name, (source, multiplier, offset) in self.robot.mimics.items():
            ET.SubElement(self.equality, 'joint', joint1=name, joint2=source,
                          polycoef=values([offset, multiplier, 0, 0, 0]))
        actuator = ET.SubElement(self.root, 'actuator')
        gains = [(1500, 35, 163), (2000, 45, 163), (1500, 30, 96),
                 (1000, 6.7, 30), (750, 4.5, 30), (500, 2.2, 30)]
        for joint in self.root.findall('.//joint'):
            name = joint.get('name')
            if name not in self.robot.joints:
                continue
            arm = name.startswith('joint_')
            kp, kd, limit = gains[int(name[-1]) - 1] if arm else (20, .1, 10)
            joint.set('damping', str(kd))
            ET.SubElement(actuator, 'position', name=name, joint=name, kp=str(kp),
                          forcelimited='true', forcerange=values([-limit, limit]))
        tool = self.root.find(".//body[@name='tool0']")
        ET.SubElement(tool, 'site', name='flange_sensor', size='.001')
        sensors = ET.SubElement(self.root, 'sensor')
        ET.SubElement(sensors, 'force', name='flange_force', site='flange_sensor')
        ET.SubElement(sensors, 'torque', name='flange_torque', site='flange_sensor')
        contact = ET.SubElement(self.root, 'contact')
        # Gazebo's existing robot has self_collide disabled. Do not introduce finger hull self-contact.
        for first, second in combinations([b.get('name') for b in self.root.findall('.//body')], 2):
            ET.SubElement(contact, 'exclude', body1=first, body2=second)
        scene = self.robot.scene_map
        if scene is not None:
            self.add_scene(scene)
        path = self.cache / 'world.xml'
        ET.ElementTree(self.root).write(path, encoding='unicode')
        # Compile now, so invalid conversions fail before the controller starts.
        self.robot.mujoco_model = mujoco.MjModel.from_xml_path(str(path))
        return path

    def mesh_geom(self, body, name, path, scale, color, collision=True):
        ET.SubElement(self.assets, 'mesh', name=name, file=self.mesh_path(str(path)), scale=values(scale))
        return ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name, rgba=values(color),
                             contype='1' if collision else '0', conaffinity='1' if collision else '0')

    def add_scene(self, scene):
        base = scene.world_base
        floor = np.eye(4); floor[2, 3] = scene.config['plane']['z_m']
        table = ET.SubElement(self.world, 'body', name='scene_table', **pose_attributes(base @ floor))
        ET.SubElement(table, 'geom', type='plane', size='2 2 .1', rgba='.45 .48 .52 1', friction='.3 .005 .0001')
        vehicle = ET.SubElement(self.world, 'body', name='scene_vehicle', **pose_attributes(base @ scene.vehicle.pose))
        for part, mesh in scene.vehicle.meshes.items():
            self.mesh_geom(vehicle, 'vehicle_' + part, mesh, [.001] * 3, [*scene.vehicle.colors[part], 1], False)
        collision = VehicleCollision(scene.vehicle.collision_meshes, self.cache)
        for index, path in enumerate(collision.build()):
            name = f'vehicle_collision_{index}'
            ET.SubElement(self.assets, 'mesh', name=name, file=str(path.resolve()))
            ET.SubElement(vehicle, 'geom', name=name, type='mesh', mesh=name, rgba='0 0 0 0', group='3')
        self.robot.collision_info = {key: value for key, value in collision.info.items() if key != 'files'}
        for name, item in scene.objects.items():
            body = ET.SubElement(self.world, 'body', name=name, **pose_attributes(base @ item.pose))
            if hasattr(item, 'mesh'):
                geom = self.mesh_geom(body, name + '_mesh', item.mesh, [.001] * 3, [*item.color, 1], False)
                if name == 'contact_block':
                    from configs import load
                    from defect_block import DefectBlock
                    block = DefectBlock(load('robot')['contact_test'])
                    count = int(round(block.size / .2)) + 1
                    xy = np.linspace(-block.size / 2, block.size / 2, count)
                    x, y = np.meshgrid(xy, xy)
                    height = (block.top + block.height(x, y)) / block.top
                    ET.SubElement(self.assets, 'hfield', name='block_surface', nrow=str(count), ncol=str(count),
                                  size=values([block.size / 2000, block.size / 2000, block.top / 1000, .001]),
                                  elevation=values(height))
                    geom = ET.SubElement(body, 'geom', type='hfield', hfield='block_surface', rgba=values([*item.color, 1]))
                else:
                    geom.set('contype', '1'); geom.set('conaffinity', '1')
            elif hasattr(item, 'radius'):
                geom = ET.SubElement(body, 'geom', type='sphere', size=str(item.radius), rgba=values([*item.color, 1]))
            else:
                geom = ET.SubElement(body, 'geom', type='box', size=values([item.side / 2] * 3), rgba=values([*item.color, 1]))
            geom.set('friction', values([item.mu, .005, .0001]))
        probe = scene.probe_tool
        rack = ET.SubElement(self.world, 'body', name='probe_rack', **pose_attributes(base @ probe.park_body))
        self.mesh_geom(rack, 'rack_mesh', probe.holder, [.001] * 3, [.55, .55, .6, 1])
        body = ET.SubElement(self.world, 'body', name='docked_probe', sleep='never', **pose_attributes(base @ probe.park_body))
        ET.SubElement(body, 'freejoint', name='probe_free')
        ET.SubElement(body, 'inertial', mass=str(probe.mass), pos=values(probe.com), diaginertia='.00001 .00001 .00001')
        # The sphere makes tip contact exact. Body and gripper are constrained rather than friction-grasped.
        # priority 1: its own friction holds in every contact (MuJoCo otherwise takes the larger one, the vehicle's
        # default 1.0, with which a fixed-axis press jams on slopes from 45 deg).
        self.mesh_geom(body, 'probe_mesh', probe.mesh, [.001] * 3, [.15, .35, .7, 1], False)
        shaft = self.mesh_geom(body, 'probe_body_collision', probe.contact_mesh(), [.001] * 3, [0, 0, 0, 0])
        shaft.set('group', '3')
        shaft.set('priority', '1')
        shaft.set('friction', values([probe.config.get('contact_friction', probe.ball['friction'] if probe.ball else .3),
                                      .005, .0001]))
        ET.SubElement(body, 'geom', name='probe_tip_sphere', type='sphere', pos=values(probe.tip_center),
                      size=str(probe.tip_radius), friction=values([probe.ball['friction'] if probe.ball else .3, .005, .0001]),
                      priority='1', rgba='.7 .7 .75 1')
        # Stiff welds: their stiffness scales with the probe's tiny inertia, and the default (solref .02, solimp .95)
        # let a 1.5 N tip friction tilt the probe 28 deg and a 1.5 N press sink it 0.8 mm; this keeps both under
        # 0.1 deg / 0.01 mm. Time constant 0.004 s = two 2 ms steps, the stable minimum.
        stiff = dict(solref='0.004 1', solimp='0.99 0.999 0.001')
        ET.SubElement(self.equality, 'weld', name='probe_dock', body1='probe_rack', body2='docked_probe', **stiff)
        transforms, _ = probe.chain.transforms({})
        relative = np.linalg.inv(transforms['tool0']) @ transforms['link_6'] @ probe.flange_body
        pose = pose_attributes(relative)
        ET.SubElement(self.equality, 'weld', name='probe_grip', body1='tool0', body2='docked_probe', active='false',
                      relpose=pose['pos'] + ' ' + pose['quat'], **stiff)
        # Suppress redundant probe/gripper contacts while retaining contact with scene geometry.
        contact = self.root.find('contact')
        ET.SubElement(contact, 'exclude', body1='docked_probe', body2='probe_rack')
        for name in ['tool0', *[b.get('name') for b in self.root.findall('.//body') if b.get('name', '').startswith('rg2_')]]:
            ET.SubElement(contact, 'exclude', body1='docked_probe', body2=name)
