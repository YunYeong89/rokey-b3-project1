"""--car scene shared by native Gazebo, RViz and the web viewer (BASE frame; TableScene is shared with the --object
scene of object_scene.py): the table plane, the probe on its
two-rod rack (scene.json `probes`: the standard tip or the low-friction metal ball tip), one vehicle of repair_car/config/vehicles.yaml with the simulated damage of native/scene.json, and
the contact-test objects of robot.yaml `contact_test`: the square block below the home TCP (repair_car
contact_overshoot, contact_scrape; its top carries the crack and dent of defect_scan) and the hemisphere
(contact_scrape).
The vehicle stands inside the robot.yaml `vehicle_zone` (random pose within its tolerance, front/back either
way) unless a pose is given; `real` (the team's vehicle assembly, vehicles.yaml `real`) stands on the robot's front
axis, its front toward the robot, with the probe rack only (no contact-test block or hemisphere). `demo`, `demo_front`,
`demo_left`, `demo_right` (vehicles.yaml `demo`): the same vehicle fixed on the vehicle holder, both turned so that the
front, the left door or the right door faces the robot."""
import json
from pathlib import Path
import subprocess
import sys
import time
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np

from kinematics import KinematicChain

SCENE = Path(__file__).with_name('scene.json')
sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'ws_cobot_pjt/src/repair_car'))
from repair_car.config import load  # noqa: E402  (repair_car package configuration)
from repair_car.geometry.mesh_io import read_stl, write_stl  # noqa: E402
from repair_car.geometry.vehicle_model import VehicleModel, pose_matrix  # noqa: E402
from repair_car.geometry.real_vehicle import RealVehicle  # noqa: E402
from repair_car.inspection.scan_scene import DOOR_DEMO_PANELS, INSPECTION_SCENE, PANELS, ScanScene  # noqa: E402
from repair_car.experiments.performance_sample import PerformanceSample  # noqa: E402
from repair_car.experiments.defect_block import DefectBlock  # noqa: E402


DEMO = ('demo', 'demo_front', 'demo_left', 'demo_right', 'demo_final')
INSPECTION_CARS = ('real', *DEMO)
PACKAGE = Path(__file__).resolve().parents[4] / 'ws_cobot_pjt/src/repair_car'


def vehicle_types():
    return sorted(load('vehicles')['types'])


def probe_types():
    return list(json.loads(SCENE.read_text(encoding='utf-8'))['probes'])


def default_probe(kind):
    return 'ball' if kind in INSPECTION_CARS else 'standard'


def vector(values):
    return ' '.join(str(float(value)) for value in values)


def rpy(matrix):
    pitch = np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0]))
    return [np.arctan2(matrix[2, 1], matrix[2, 2]), pitch,
            np.arctan2(matrix[1, 0], matrix[0, 0])]


def friction(element, mu):
    """Surface friction of a collision element (Gazebo uses the smaller coefficient of the two contacts)."""
    ode = ET.SubElement(ET.SubElement(ET.SubElement(element, 'surface'), 'friction'), 'ode')
    ET.SubElement(ode, 'mu').text = ET.SubElement(ode, 'mu2').text = str(mu)


def material(element, color):
    node = ET.SubElement(element, 'material')
    for tag in ['ambient', 'diffuse']:
        ET.SubElement(node, tag).text = vector([*color, 1])


class Vehicle:
    def __init__(self, kind, pose, damage, cache, *, visual_only=False, table_z=None):
        types, zone = load('vehicles')['types'], load('robot')['vehicle_zone']
        table_z = zone['table_z_mm'] if table_z is None else table_z
        rng = np.random.default_rng()
        kind = str(rng.choice(sorted(types))) if kind == 'random' else kind
        if kind in DEMO or (kind == 'real' and not visual_only):
            spec = load('vehicles')['demo']
            self.model = RealVehicle()
            if visual_only:
                pose = pose or [*spec['centre_mm'], spec['yaw_deg'][kind]]
                base_z = table_z + spec['height_mm'] - self.model.size[2]
            else:
                cell = load('real_scene')
                table_z = cell['vehicle']['table_z_mm']
                panel = DOOR_DEMO_PANELS.get(kind, 'bonnet')
                pose, base_z = ScanScene.inspection_pose(panel, cell, self.model.size[2], placement=pose)
            self.holder = (LiftStand(self.model, pose, table_z, base_z, cache) if kind == 'demo_final' and not visual_only
                           else Holder(spec, pose, table_z, base_z, self.model.size[2], cache))
            if isinstance(self.holder, LiftStand):
                pose, base_z = self.holder.vehicle()   # the lift starts lowered under the bonnet placement
            self.pose = pose_matrix(*pose, base_z)
        elif kind in ('real', 'performance'):
            self.model = RealVehicle() if kind == 'real' else PerformanceSample()
            pose = pose or self.model.placement()
            self.pose = self.model.pose(pose, table_z)
        else:
            if pose is None:
                offset = rng.uniform(-1, 1, 2) * zone['position_tolerance_mm']
                yaw = zone['yaw_deg'] + rng.uniform(-1, 1) * zone['yaw_tolerance_deg'] + rng.choice([0.0, 180.0])
                pose = [*(np.array(zone['center_mm']) + offset), yaw]
            self.model = VehicleModel(kind, damage)
            self.pose = pose_matrix(*pose, table_z)
        self.pose[:3, 3] /= 1000
        self.meshes = self.model.export(cache / 'vehicle')
        if not visual_only:
            self.collision_meshes = ([self.model.export_collision(cache / 'vehicle')]
                                     if type(self.model) is RealVehicle else list(self.meshes.values()))
            # PerformanceSample derives from RealVehicle but has no welded assembly surface
        self.colors = self.model.part_colors()
        # model: the vehicle real_scan expects (real for the demo scenes too)
        self.truth = {'type': kind, 'model': 'real' if kind in DEMO else kind,
                      'pose_base_mm_deg': [round(float(v), 2) for v in pose],
                      'table_z_mm': table_z, 'base_z_mm': round(float(self.pose[2, 3] * 1000), 2),
                      'size_mm': self.model.size.round(1).tolist(), 'damage': self.model.damage}
        if kind == 'performance':
            self.truth.update(source_mesh=self.model.spec['mesh'], source_hash=self.model.source_hash,
                              source_units='mm', source_up='+Y', scene_up='+Z', nominal_top_mm=float(self.model.size[2]))

    def add_to_world(self, world, static_link):
        link = static_link(world, 'scene_vehicle', self.pose)
        for part, mesh in self.meshes.items():
            element = ET.SubElement(link, 'visual', name=f'{part}_visual')
            geometry = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
            ET.SubElement(geometry, 'uri').text = mesh.resolve().as_uri()
            ET.SubElement(geometry, 'scale').text = '0.001 0.001 0.001'
            material(element, self.colors[part])
        for index, mesh in enumerate(self.collision_meshes):
            element = ET.SubElement(link, 'collision', name=f'surface_{index}_collision')
            geometry = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
            ET.SubElement(geometry, 'uri').text = mesh.resolve().as_uri()
            ET.SubElement(geometry, 'scale').text = '0.001 0.001 0.001'
            if self.truth['type'] == 'performance':
                friction(element, self.model.spec['friction'])

    def markers(self, frame):
        from visualization_msgs.msg import Marker
        from repair_car_pose import quaternion
        result = []
        for index, (part, mesh) in enumerate(self.meshes.items()):
            marker = Marker()
            marker.header.frame_id = frame
            marker.ns, marker.id, marker.type, marker.action = 'vehicle', index, Marker.MESH_RESOURCE, Marker.ADD
            marker.mesh_resource = mesh.resolve().as_uri()
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, self.pose[:3, 3])
            q = quaternion(self.pose[:3, :3])
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q
            marker.scale.x = marker.scale.y = marker.scale.z = 0.001
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *map(float, self.colors[part]), 1.0
            result.append(marker)
        return result


class RealDisplayScene:
    """The holder-mounted vehicle and its reference points for the connected robot's viewers."""

    def __init__(self, cache, placement=None, *, kind='real', session=None):
        cell = load('real_scene')
        vehicle, probe = cell['vehicle'], cell['probe']
        self.probe_holder = ProbeHolder(probe) if probe['holder_pose_base_mm_deg'] else None
        cache.mkdir(parents=True, exist_ok=True)
        panel = DOOR_DEMO_PANELS.get(kind)
        self.inspection = session / INSPECTION_SCENE if kind == 'demo_final' and session is not None else None
        if kind == 'demo_final':
            pose, base_z = ScanScene.inspection_pose('bonnet', cell, placement=placement)
        elif panel:
            pose, base_z = ScanScene.inspection_pose(panel, cell, placement=placement)
        else:
            pose, base_z = placement or vehicle['pose_base_mm_deg'], vehicle['base_z_mm']
        self.vehicle = Vehicle('demo_front' if kind in ('real', 'demo_final') else kind, pose, (), cache,
                               visual_only=True, table_z=vehicle['table_z_mm'])
        if base_z is not None:
            self.vehicle.pose[2, 3] = base_z / 1000
            self.vehicle.truth.update(pose_base_mm_deg=pose, base_z_mm=base_z)

    def markers(self):
        from visualization_msgs.msg import MarkerArray
        if self.inspection is not None and self.inspection.is_file():
            placement = json.loads(self.inspection.read_text())
            pose = placement['pose_base_mm_deg']
            self.vehicle.pose = pose_matrix(*pose, placement['base_z_mm'])
            self.vehicle.pose[:3, 3] /= 1000
            self.vehicle.holder.pose = pose_matrix(*pose, self.vehicle.truth['table_z_mm'])
            self.vehicle.holder.pose[:3, 3] /= 1000
        return MarkerArray(markers=[*self.vehicle.holder.markers('base_link'), *self.vehicle.markers('base_link'),
                                    *(self.probe_holder.markers('base_link') if self.probe_holder else [])])

    def write_reference(self, directory):
        """Sample the full vehicle for display; the scanner generates its own part reference."""
        points, _ = self.vehicle.model.sample(2.)
        pose = self.vehicle.pose
        points = points / 1000 @ pose[:3, :3].T + pose[:3, 3]
        path = directory / 'clouds' / 'reference.npy'
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp.npy')
        np.save(temporary, points.astype(np.float32))
        temporary.replace(path)
        return path


class Holder:
    """The vehicle holder of the demo scenes (vehicles.yaml `demo`): base plate on the table at the vehicle's x, y,
    turned with it (holder x along the vehicle length). The real posts go into the vehicle underside; here they are
    shortened (their top vertices lowered) so that the vehicle resting on them tops out at height_mm."""

    def __init__(self, spec, placement, table_z, base_z, vehicle_height, cache):
        triangles = read_stl(PACKAGE / spec['holder_mesh'])
        top = triangles[..., 2] > triangles[..., 2].max() - 1e-6
        self.top_mm = float(base_z - table_z + spec['seat_mm'])
        triangles[..., 2][top] = self.top_mm
        self.mesh = Path(cache) / 'demo_holder.stl'
        write_stl(self.mesh, triangles)
        self.pose = pose_matrix(*placement, table_z)
        self.pose[:3, 3] /= 1000
        self.color = spec['color']
        self.info = {'mesh': spec['holder_mesh'], 'centre_mm': [float(v) for v in placement[:2]],
                     'yaw_deg': float(placement[2]), 'table_z_mm': table_z, 'post_top_z_mm': table_z + self.top_mm,
                     'top_z_mm': float(base_z + vehicle_height)}

    def add_to_world(self, world, static_link):
        link = static_link(world, 'scene_holder', self.pose)
        for tag in ['visual', 'collision']:
            element = ET.SubElement(link, tag, name=f'holder_{tag}')
            geometry = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
            ET.SubElement(geometry, 'uri').text = self.mesh.resolve().as_uri()
            ET.SubElement(geometry, 'scale').text = '0.001 0.001 0.001'
            if tag == 'visual':
                material(element, self.color)

    def markers(self, frame):
        return [mesh_marker(frame, 'holder', self.mesh, self.pose, self.color)]


class LiftStand:
    """demo_final (virtual only): the lift project's turntable stand (repair_car/lift/turnlift/virtual_stand.py) in place
    of the fixed holder, at the same vehicle placements; its disc turns and its lift raises the vehicle (LiftCommand)."""
    COLOURS = dict(base=(.3, .45, .8), crank=(.85, .85, .85), pushrod=(.95, .95, .95), slider_bottom=(.5, .5, .5),
                   slider_top=(.5, .5, .5), arm_a=(.2, .2, .2), arm_b=(.95, .5, .1), top=(.6, .3, .7))

    def __init__(self, model, placement, table_z, base_z, cache):
        from repair_car.lift.turnlift.virtual_stand import VirtualStand
        self.stand, self.table_z = VirtualStand(), table_z
        self.seat = self.stand.seat(model.triangles)
        self.stand.place_vehicle(*placement, base_z, table_z, self.seat)
        self.stand.servo = float(self.stand.down)   # lowered: the repair raises it (LiftCommand) before scanning
        self.meshes = self.stand.meshes(Path(cache))
        self.colors = {**self.COLOURS, 'disc': self.stand.disc_colour}
        self.poses = self.frames()
        self.info = {'stand': 'lift turntable (repair_car/lift)', 'table_z_mm': table_z, **self.stand.status()}

    def frames(self):
        """{model name: BASE 4x4 m} of the disc and the lift links."""
        result = {}
        for name, pose in self.stand.frames(self.table_z).items():
            pose = pose.copy()
            pose[:3, 3] /= 1000
            result[f'scene_stand_{name}'] = pose
        return result

    def vehicle(self):
        """([x, y, yaw], base_z) of the vehicle resting on the stand now (BASE mm, deg)."""
        return self.stand.vehicle_pose(self.table_z, self.seat)

    def add_to_world(self, world, static_link):
        for name, pose in self.poses.items():
            link = static_link(world, name, pose)
            part = name.removeprefix('scene_stand_')
            for tag in ['visual', 'collision']:
                element = ET.SubElement(link, tag, name=f'{part}_{tag}')
                geometry = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
                ET.SubElement(geometry, 'uri').text = self.meshes[part].resolve().as_uri()
                ET.SubElement(geometry, 'scale').text = '0.001 0.001 0.001'
                if tag == 'visual':
                    material(element, self.colors[part])

    def markers(self, frame):
        return [mesh_marker(frame, name, self.meshes[name.removeprefix('scene_stand_')], pose,
                            self.colors[name.removeprefix('scene_stand_')]) for name, pose in self.poses.items()]


def mesh_marker(frame, namespace, mesh, pose, color):
    """A mm STL mesh marker at `pose` (BASE, metres)."""
    from visualization_msgs.msg import Marker
    from repair_car_pose import quaternion
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns, marker.id, marker.type, marker.action = namespace, 0, Marker.MESH_RESOURCE, Marker.ADD
    marker.mesh_resource = mesh.resolve().as_uri()
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, pose[:3, 3])
    q = quaternion(pose[:3, :3])
    marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q
    marker.scale.x = marker.scale.y = marker.scale.z = 0.001
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = *map(float, color), 1.0
    return marker


class ProbeHolder:
    """The real probe holder (real_scene.yaml probe.holder_mesh, Z-up about its base-plate bottom centre, at
    holder_pose_base_mm_deg [x, y, z, yaw]) for the connected robot's viewers: real_scan plans around its extents."""
    color = (0.95, 0.6, 0.15)

    def __init__(self, probe):
        x, y, z, yaw = probe['holder_pose_base_mm_deg']
        self.mesh = PACKAGE / probe['holder_mesh']
        self.pose = pose_matrix(x, y, yaw, z)
        self.pose[:3, 3] /= 1000

    def markers(self, frame):
        return [mesh_marker(frame, 'probe_holder', self.mesh, self.pose, self.color)]


class ContactBlock:
    """Square block on the table, centred on the home probe TCP x, y, its top at contact_test.surface_z_mm, with
    the crack and dent of contact_test.block_defects (repair_car defect_scan) cut into the top: a closed mesh, flat
    top on a 2 mm grid (coplanar facets), refined over the damage."""
    color = (0.72, 0.62, 0.45)

    def __init__(self, robot_model, cache):
        robot = load('robot')
        test, table = robot['contact_test'], robot['vehicle_zone']['table_z_mm']
        chain = KinematicChain(robot_model.robot, tcp_link='probe_tip')
        transforms, _ = chain.transforms(dict(zip(chain.names, np.deg2rad(robot['robot']['home_joints_deg']))))
        home = (np.linalg.inv(transforms['base_link']) @ transforms['probe_tip'])[:3, 3]
        self.pose = np.eye(4)
        self.pose[:2, 3] = home[:2]
        self.mesh = cache / 'contact_block.stl'
        write_stl(self.mesh, DefectBlock(test).triangles(table))
        self.mu = test['friction']
        self.info = {'centre_mm': (home[:2] * 1000).round(2).tolist(), 'top_z_mm': test['surface_z_mm'],
                     'size_mm': test['block_size_mm'], 'friction': self.mu, 'defects': test['block_defects']}

    def add_to_world(self, world, static_link):
        link = static_link(world, 'scene_contact_block', self.pose)
        for tag in ['visual', 'collision']:
            element = ET.SubElement(link, tag, name=f'block_{tag}')
            geometry = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
            ET.SubElement(geometry, 'uri').text = self.mesh.resolve().as_uri()
            ET.SubElement(geometry, 'scale').text = '0.001 0.001 0.001'
            material(element, self.color) if tag == 'visual' else friction(element, self.mu)

    def markers(self, frame):
        from visualization_msgs.msg import Marker
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns, marker.id, marker.type, marker.action = 'contact_block', 0, Marker.MESH_RESOURCE, Marker.ADD
        marker.mesh_resource = self.mesh.resolve().as_uri()
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, self.pose[:3, 3])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.001
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = *self.color, 1.0
        return [marker]


class Hemisphere:
    """Sphere centred on the table (its upper half shows): smooth contact, no mesh facets in the force."""
    color = (0.45, 0.6, 0.72)

    def __init__(self):
        robot = load('robot')
        test, table = robot['contact_test'], robot['vehicle_zone']['table_z_mm']
        self.radius = test['hemisphere_radius_mm'] / 1000
        self.pose = np.eye(4)
        self.pose[:3, 3] = [*np.array(test['hemisphere_center_mm']) / 1000, table / 1000]
        self.mu = test['friction']
        self.info = {'centre_mm': test['hemisphere_center_mm'], 'radius_mm': test['hemisphere_radius_mm'],
                     'friction': self.mu}

    def add_to_world(self, world, static_link):
        link = static_link(world, 'scene_hemisphere', self.pose)
        for tag in ['visual', 'collision']:
            element = ET.SubElement(link, tag, name=f'hemisphere_{tag}')
            ET.SubElement(ET.SubElement(ET.SubElement(element, 'geometry'), 'sphere'), 'radius').text = str(self.radius)
            material(element, self.color) if tag == 'visual' else friction(element, self.mu)

    def markers(self, frame):
        from visualization_msgs.msg import Marker
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns, marker.id, marker.type, marker.action = 'hemisphere', 0, Marker.SPHERE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, self.pose[:3, 3])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 2 * self.radius
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = *self.color, 1.0
        return [marker]


class TableScene:
    """What the --car and --object scenes share: static links on the BASE frame, the table plane of scene.json and the
    marker data for the web. Subclasses set `config`, `frame`, `world_base` and `markers()`."""

    def static_link(self, world, name, pose):
        """Static model on the BASE frame with one link at `pose` (BASE, 4x4 m)."""
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = vector([*self.world_base[:3, 3], *rpy(self.world_base[:3, :3])])
        link = ET.SubElement(model, 'link', name='body')
        ET.SubElement(link, 'pose').text = vector([*pose[:3, 3], *rpy(pose[:3, :3])])
        return link

    def add_plane(self, world):
        plane = self.config['plane']
        table = np.eye(4)
        table[2, 3] = plane['z_m']
        link = self.static_link(world, 'scene_plane', table)
        for tag in ['visual', 'collision']:
            geometry = ET.SubElement(ET.SubElement(link, tag, name=f'plane_{tag}'), 'geometry')
            shape = ET.SubElement(geometry, 'plane')
            ET.SubElement(shape, 'normal').text = '0 0 1'
            ET.SubElement(shape, 'size').text = vector(plane['size_m'])

    def plane_marker(self):
        from visualization_msgs.msg import Marker
        plane = Marker()
        plane.header.frame_id = self.frame
        plane.ns, plane.id, plane.type, plane.action = 'scene', 0, Marker.CUBE, Marker.ADD
        plane.pose.orientation.w = 1.0
        plane.pose.position.z = self.config['plane']['z_m'] - 0.001
        plane.scale.x, plane.scale.y = self.config['plane']['size_m']
        plane.scale.z = 0.002
        plane.color.r, plane.color.g, plane.color.b, plane.color.a = 0.45, 0.48, 0.52, 1.0
        return plane

    def marker_dicts(self):
        """The RViz markers as plain data for /dsr01/sim_scene; meshes are referenced as scene/<name>.stl."""
        return [{'ns': m.ns, 'id': m.id, 'type': m.type,
                 'pose': [m.pose.position.x, m.pose.position.y, m.pose.position.z, m.pose.orientation.x,
                          m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w],
                 'scale': [m.scale.x, m.scale.y, m.scale.z], 'color': [m.color.r, m.color.g, m.color.b, m.color.a],
                 'mesh': f'scene/{Path(unquote(urlparse(m.mesh_resource).path)).stem}.stl' if m.mesh_resource else ''}
                for m in self.markers().markers]


class CarScene(TableScene):
    def __init__(self, robot_model, cache, kind, pose=None, probe='standard'):
        from probe_tool import ProbeTool
        self.config = json.loads(SCENE.read_text(encoding='utf-8'))
        self.frame, self.cache = self.config['frame'], Path(cache)
        self.cache.mkdir(parents=True, exist_ok=True)
        probe_config = self.config['probes'][probe]
        if kind == 'performance':
            probe_config = {**probe_config,'contact_friction':load('vehicles')['performance']['friction'],
                            'sphere_contact_only':True}
        if kind in INSPECTION_CARS:
            self.config['plane']['z_m'] = load('real_scene')['vehicle']['table_z_mm'] / 1000
        self.probe_tool = ProbeTool(robot_model, self.cache, probe_config, measured_cell=kind in INSPECTION_CARS)
        self.probe_tool.plan['probe'] = probe
        ET.ElementTree(robot_model.robot).write(robot_model.urdf, encoding='unicode')
        transforms, _ = KinematicChain(robot_model.robot).transforms({})
        self.world_base = transforms[self.frame]
        self.vehicle = Vehicle(kind, pose, self.config['damage'], self.cache)
        self.truth_file = None
        if kind == 'demo_final':
            # model name -> the link pose the SDF stores (set_pose_vector moves models)
            self.initial_fixture = {'scene_vehicle': self.vehicle.pose.copy(),
                                    **{name: pose.copy() for name, pose in self.vehicle.holder.poses.items()}}
        # The contact-test objects only with the pack vehicles: the real vehicle stands in front of the robot.
        kind = self.vehicle.truth['type']
        self.objects = ({'holder': self.vehicle.holder} if kind in INSPECTION_CARS else {} if kind == 'performance'
                        else {'contact_block': ContactBlock(robot_model, self.cache), 'hemisphere': Hemisphere()})
        self.static = None  # scene markers built once; only the probe moves
        info = {name: item.info for name, item in self.objects.items()}
        (self.cache / 'scene_info.json').write_text(json.dumps(
            {'source': str(SCENE), 'vehicle': self.vehicle.truth, **info}, indent=2))
        print(f'Scene: probe {probe}, {self.vehicle.truth}, {info}', flush=True)

    def select_fixture(self, panel):
        """Turn and raise the lift stand to the real demo_final placement of `panel` (/fixture/select)."""
        if panel not in PANELS:
            raise ValueError('The inspection placement cannot be changed in this virtual session.')
        stand = self.lift_stand()
        pose, base_z = ScanScene.inspection_pose(panel, vehicle_height_mm=self.vehicle.model.size[2])
        self.move_stand(lambda: stand.stand.place_vehicle(*pose, base_z, stand.table_z, stand.seat) or True)

    def lift_command(self, line):
        """One LiftCommand line (heading / turn / lift / status) on the lift stand -> the firmware-style reply with the
        vehicle frame height (vehicle_z_mm, BASE)."""
        stand = self.lift_stand()
        reply = stand.stand.status()
        if line.split()[:1] != ['status']:
            def change():
                nonlocal reply
                reply = stand.stand.command(line)
                return reply['ok']
            self.move_stand(change)
        return {**reply, 'vehicle_z_mm': stand.vehicle()[1]} if reply['ok'] else reply

    def lift_stand(self):
        if self.vehicle.truth['type'] != 'demo_final' or self.truth_file is None:
            raise ValueError('The inspection placement cannot be changed in this virtual session.')
        return self.vehicle.holder

    def move_stand(self, change):
        """Apply `change` to the stand state (False from it: refused, nothing moves), then move the vehicle and the
        stand in Gazebo and record the vehicle placement; the state is restored when Gazebo refuses."""
        stand = self.vehicle.holder
        before = stand.stand.centre.copy(), stand.stand.heading, stand.stand.servo
        try:
            if not change():
                return
            target = stand.stand.centre.copy(), stand.stand.heading, stand.stand.servo
            turn = (target[1] - before[1] + 180.) % 360. - 180.
            steps = 12 if abs(turn) > .01 or abs(target[2] - before[2]) > .01 or np.linalg.norm(target[0] - before[0]) > .01 else 1
            for fraction in np.linspace(0., 1., steps + 1)[1:]:
                blend = fraction * fraction * (3. - 2. * fraction)
                stand.stand.centre = before[0] + blend * (target[0] - before[0])
                stand.stand.heading = (before[1] + blend * turn) % 360.
                stand.stand.servo = before[2] + blend * (target[2] - before[2])
                pose, base_z = self.apply_stand_pose()
                if steps > 1:
                    time.sleep(.04)
        except BaseException:
            stand.stand.centre, stand.stand.heading, stand.stand.servo = before
            raise
        self.vehicle.truth.update(pose_base_mm_deg=pose, base_z_mm=base_z)
        temporary = self.truth_file.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.vehicle.truth))
        temporary.replace(self.truth_file)
        self.static = None

    def apply_stand_pose(self):
        from repair_car_pose import quaternion
        stand = self.vehicle.holder
        pose, base_z = stand.vehicle()
        vehicle = pose_matrix(*pose, base_z)
        vehicle[:3, 3] /= 1000
        targets = {'scene_vehicle': vehicle, **stand.frames()}
        entries = []
        for name, target in targets.items():
            # The SDF stores the initial fixture pose on the model's child link.
            world_model = self.world_base @ target @ np.linalg.inv(self.initial_fixture[name])
            x, y, z = world_model[:3, 3]
            qx, qy, qz, qw = quaternion(world_model[:3, :3])
            entries.append(f'pose {{name: "{name}" position {{x: {x} y: {y} z: {z}}} '
                           f'orientation {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}')
        response = subprocess.run(['gz', 'service', '-s', '/world/cobot_world/set_pose_vector',
                                   '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
                                   '--timeout', '3000', '--req', ' '.join(entries)], check=True,
                                  capture_output=True, text=True, timeout=4.)
        if 'data: true' not in response.stdout:
            raise RuntimeError('Gazebo did not approve the vehicle placement change.')
        self.vehicle.pose, stand.poses = targets.pop('scene_vehicle'), targets
        self.static = None
        return pose, base_z

    def add_to_world(self, world, robot_model):
        self.add_plane(world)
        for item in [self.vehicle, *self.objects.values()]:
            item.add_to_world(world, self.static_link)
        self.probe_tool.add_to_world(world, robot_model)

    def markers(self):
        """Republished 10 / s to RViz and 5 / s to the container: the static scene (plane, vehicle meshes, block,
        hemisphere) is built once, only the probe markers each time."""
        from visualization_msgs.msg import MarkerArray
        if self.static is None:
            self.static = [self.plane_marker(), *(marker for item in [self.vehicle, *self.objects.values()]
                                                  for marker in item.markers(self.frame))]
        return MarkerArray(markers=[*self.static, *self.probe_tool.markers()])
