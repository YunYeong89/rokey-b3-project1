"""Repository probe mesh and its holder from the STL designs in the repair_car package: the two-rod holder (sharing the
probe frame at `pose_base_mm_deg`) or a placed holder (`holder_pose` real_scene: real_scene.yaml's measured probe
holder, the probe seated in its slot `holder_slot_mm`),
fitted to the RG2 finger collision meshes. STL designs are in mm, in the tip frame: +z toward the top
face, the probe point at -z. The RG2 closes on the tip faces normal to `grip_axis`; the robot takes its
grasp orientation from the docked tip pose given in BASE (`pose_base_mm_deg`). The optional `ball` settings
provide the mesh tip's exact contact sphere, friction and colour."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'ws_cobot_pjt/src/repair_car'))
from repair_car.geometry.mesh_io import read_stl, write_stl  # noqa: E402
from kinematics import KinematicChain, origin, rotation


def home_joints():
    """HOME joint angles (deg): the one value in repair_car robot.yaml (scene.py puts the package on sys.path)."""
    from repair_car.config import load
    return [float(value) for value in load('robot')['robot']['home_joints_deg']]

ROOT = Path(__file__).resolve().parents[4]


def text(values):
    return ' '.join(str(float(value)) for value in values)


class ProbeTool:
    # tip frame -> link_6: the tip's grip axis onto the finger closing axis (link_6 x), tip +z opposite link_6 z
    GRIP = {'x': np.diag([1.0, -1.0, -1.0]), 'y': np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])}
    SEAT_TOLERANCE = .002  # m: fingertip gap to the groove floor off the planned clearance (grasp_error)
    POCKET_EDGE_TOLERANCE = .001  # m: allow simulated tracking error at the nominal STL pocket boundary

    def __init__(self, model, cache, config, *, measured_cell=False):
        self.config, self.cache = config, Path(cache)
        self.robot = model.robot
        self.joint = 'rg2_finger_joint'
        self.clearance = 0.0002
        self.mesh, self.holder = ROOT / config['mesh'], ROOT / config['holder']
        self.pickup = None
        if config.get('holder_pose') == 'real_scene':
            from repair_car.config import load
            self.pickup = load('real_scene')['probe']
            self.holder = ROOT / 'ws_cobot_pjt/src/repair_car' / self.pickup['holder_mesh']
        self.measured_cell = measured_cell
        triangles = read_stl(self.mesh, 0.001)
        low, high = triangles.reshape(-1, 3).min(0), triangles.reshape(-1, 3).max(0)
        self.ball = config.get('ball')
        if self.ball:  # the probe point is the bottom of the ball, below the body mesh
            low[2] = min(low[2], self.ball['centre_mm'][2] / 1000 - config['tip_radius_mm'] / 1000)
        self.mass = float(config['mass_kg'])
        self.tip_radius = config['tip_radius_mm'] / 1000
        self.grip_axis = config['grip_axis']
        self.tcp_name = 'ProbeTCP'
        self.attached = False
        self.session_plan_path = None
        self.current = None
        chain = KinematicChain(self.robot)
        home = dict(zip(chain.names, np.deg2rad(home_joints())))
        fingers = []
        for side in ['left', 'right']:
            link = f'rg2_{side}_inner_finger'
            collision = self.robot.find(f"link[@name='{link}']/collision")
            vertices = read_stl(collision.find('geometry/mesh').get('filename').removeprefix('file://'))
            vertices = vertices.reshape(-1, 3)
            fingers.append((link, origin(collision.find('origin')), np.c_[vertices, np.ones(len(vertices))].T))
        self.fingers = fingers
        # Closing width: the tip across the grip axis where the pads land, not its outline (the grip faces are
        # recessed: 30 mm outside, 14 mm across the channels the fingertips go into).
        self.finger_angle = 0.
        pads, _ = self.pads(chain, home, fingers)
        low_pad, high_pad = np.min([pad[0] for pad in pads], 0), np.max([pad[1] for pad in pads], 0)
        self.width = self.grip_width(triangles, high[2] - config['grip_from_top_mm'] / 1000, high_pad[2] - low_pad[2],
                                     (low_pad[1], high_pad[1]))
        axis = 'xy'.index(self.grip_axis)
        self.groove_depth = (high[axis] - low[axis] - self.width) / 2
        # Finger angle whose pad gap equals the tip width plus clearance (gap shrinks as RG2 closes).
        opened, closed = -0.55, 0.78
        for _ in range(40):
            self.finger_angle = (opened + closed) / 2
            pads, transforms = self.pads(chain, home, fingers)
            if pads[1][0][0] - pads[0][1][0] > self.width + 2 * self.clearance:
                opened = self.finger_angle
            else:
                closed = self.finger_angle
        self.close_angle = self.finger_angle
        self.world_base = transforms['base_link']
        self.initial_tcp = np.linalg.inv(transforms['link_6']) @ transforms[model.tcp_link]
        pad_low, pad_high = min(pad[0][2] for pad in pads), max(pad[1][2] for pad in pads)
        self.grip_z = float((pad_low + pad_high) / 2)
        self.pocket_height = float(pad_high - pad_low + 2 * self.clearance)
        # Tip frame in link_6: probe point (-z) along TOOL +Z, grip band (grip_from_top_mm below the top
        # face) centred on the finger pads, grip axis on the finger closing axis.
        depth = self.grip_z + high[2] - config['grip_from_top_mm'] / 1000
        self.tip_z = self.fit_shaft_length(triangles, depth, low[2])
        self.flange_body = np.eye(4)
        self.flange_body[:3, :3] = self.GRIP[self.grip_axis]
        self.flange_body[2, 3] = depth
        tip_bottom = depth - self.tip_z
        self.tip_center = [0., 0., tip_bottom + self.tip_radius]
        if self.ball:
            self.tip_center[:2] = (np.asarray(self.ball['centre_mm'][:2]) / 1000).tolist()
            self.ball = {**self.ball, 'centre_mm': [value * 1000 for value in self.tip_center]}
        self.body_tcp = np.eye(4)
        self.body_tcp[:3, :3] = self.GRIP[self.grip_axis].T
        self.body_tcp[:3, 3] = [*self.tip_center[:2], tip_bottom]
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        volume = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
        self.com = (volume[:, None] * (a + b + c) / 4).sum(0) / volume.sum()  # tip frame
        self.com_flange = (self.flange_body @ np.r_[self.com, 1])[:3]
        x, y, z, yaw = self.dock = self.docked_pose(config, triangles)
        self.park_body = np.eye(4)
        self.park_body[:3, :3] = rotation(np.array([0.0, 0.0, np.deg2rad(yaw)]))
        self.park_body[:3, 3] = np.array([x, y, z]) / 1000
        self.park_flange = self.park_body @ np.linalg.inv(self.flange_body)
        self.holder_body = self.park_body
        if self.holder_pose is not None:
            self.holder_body = np.eye(4)
            self.holder_body[:3, :3] = rotation(np.array([0.0, 0.0, np.deg2rad(self.holder_pose[3])]))
            self.holder_body[:3, 3] = np.asarray(self.holder_pose[:3]) / 1000
        # Only a TCP frame is present on the robot before pickup, not an attached probe.
        ET.SubElement(self.robot, 'link', name='probe_tip')
        joint = ET.SubElement(self.robot, 'joint', name='probe_tip_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='link_6'); ET.SubElement(joint, 'child', link='probe_tip')
        ET.SubElement(joint, 'origin', xyz=f'0 0 {self.tip_z}', rpy='0 0 0')
        # The robot model's TCP stays as without a car scene; probe_tip only places the scene.
        self.chain = KinematicChain(self.robot, tcp_link='tool0')
        self.write_plan()

    def docked_pose(self, config, triangles):
        """The docked probe frame [x, y, z, yaw] (BASE mm, deg), upright. Two-rod holder: pose_base_mm_deg, the holder
        in the same frame. Placed holder (holder_pose real_scene): its slot holder_slot_mm (holder frame) with the
        probe's head (its full section's underside) on the highest holder point under the head, turned as
        pose_base_mm_deg's yaw; holder_body its own frame."""
        if self.pickup is None:
            self.holder_pose = None
            return list(config['pose_base_mm_deg'])
        hx, hy, hz, hyaw = self.holder_pose = self.pickup['holder_pose_base_mm_deg']
        holder = read_stl(self.holder).reshape(-1, 3)
        self.rack_box = [holder.min(0).tolist(), holder.max(0).tolist()]
        body = triangles.reshape(-1, 3) * 1000
        half = np.abs(body[:, :2]).max()  # the head's half width (the full section)
        head = body[np.abs(body[:, :2]).max(1) >= half - .5, 2].min()
        sx, sy = config['holder_slot_mm']
        under = holder[(np.abs(holder[:, 0] - sx) < half - .1) & (np.abs(holder[:, 1] - sy) < half - .1), 2]
        c, s = np.cos(np.radians(hyaw)), np.sin(np.radians(hyaw))
        self.seat_mm = float(under.max())
        yaw = config['pose_base_mm_deg'][3]
        if self.measured_cell:
            from repair_car.robot_control.rotations import rotation_zyz
            taught = rotation_zyz(self.pickup['pickup_grasp_base_mm_deg'][3:])
            body = taught @ self.initial_tcp[:3, :3].T @ self.flange_body[:3, :3]
            yaw = np.degrees(np.arctan2(body[1, 0], body[0, 0]))
        return [float(v) for v in (hx + c * sx - s * sy, hy + s * sx + c * sy, hz + self.seat_mm - head, yaw)]

    def fit_shaft_length(self, triangles, depth, original_bottom):
        """Shorten the shaft between the grip body and the tip assembly, both of which keep their shape."""
        from repair_car.config import load

        length_mm = self.config.get('tcp_length_mm')
        if length_mm is None:
            length_mm = load('real_scene')['probe']['tcp_mm_deg'][2]
        length = length_mm / 1000
        shift = depth - length - original_bottom
        z = triangles[:, :, 2]
        # The body ends at -14 mm. Below -40 mm the tip assembly translates as a unit.
        z += shift * np.clip((-.014 - z) / .026, 0., 1.)
        self.mesh = self.cache / self.mesh.name
        write_stl(self.mesh, triangles, scale=1000.)
        return length

    def set_mount(self, flange_body):
        """Use the physical fixed grasp for TCP kinematics, gravity compensation and scene markers."""
        from repair_car_pose import zyz
        from scipy.spatial.transform import Rotation
        self.flange_body = flange_body
        tcp = flange_body @ self.body_tcp
        self.com_flange = (flange_body @ np.r_[self.com, 1])[:3]
        origin = self.robot.find("joint[@name='probe_tip_joint']/origin")
        origin.set('xyz', text(tcp[:3, 3]))
        origin.set('rpy', text(Rotation.from_matrix(tcp[:3, :3]).as_euler('xyz')))
        self.chain = KinematicChain(self.robot, tcp_link='tool0')
        self.plan.update(tcp_mm_deg=[*(tcp[:2, 3] * 1000), self.tip_z * 1000, *zyz(tcp[:3, :3])],
                         mount_source='Gazebo fixed grasp', flange_body=flange_body.tolist())
        base = self.plan['base_tool_profile']
        weight = base['weight'] + self.mass
        self.plan['tool_profile'] = dict(weight=weight, cog=(
            (base['weight'] * np.asarray(base['cog']) + self.mass * self.com_flange * 1000) / weight).tolist(),
            inertia=[0.] * 6)
        ET.ElementTree(self.robot).write(self.cache.parent / 'robot.urdf', encoding='unicode')
        for path in (self.cache / 'probe_plan.json', self.session_plan_path):
            if path is not None:
                temporary = path.with_suffix('.tmp')
                temporary.write_text(json.dumps(self.plan, indent=2) + '\n')
                temporary.replace(path)

    def grip_width(self, triangles, centre_z, height, lateral, step=0.001):
        """Extent (m) of the tip mesh along the grip axis over the pad footprint: rays along the grip axis on a
        `step` grid, z within centre_z +- height / 2 and the other horizontal axis over the pads' link_6 y range
        `lateral` (tip frame, Moller-Trumbore). The pads close until they meet the widest of it."""
        g = 'xy'.index(self.grip_axis)
        other = 1 - g
        span = np.sort(self.GRIP[self.grip_axis][1, other] * np.asarray(lateral))
        s, z = np.meshgrid(np.arange(span[0], span[1] + 1e-9, step),
                           np.arange(centre_z - height / 2, centre_z + height / 2 + 1e-9, step))
        origins = np.zeros((s.size, 3))
        origins[:, other], origins[:, 2], origins[:, g] = s.ravel(), z.ravel(), -1.
        direction = np.eye(3)[g]
        a, e1, e2 = triangles[:, 0], triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
        p = np.cross(direction, e2)
        det = np.einsum('ij,ij->i', e1, p)
        keep = np.abs(det) > 1e-12
        a, e1, e2, p, det = a[keep], e1[keep], e2[keep], p[keep], det[keep]
        t = origins[:, None] - a[None]
        q = np.cross(t, e1[None])
        u, v = np.einsum('rtk,tk->rt', t, p) / det, (q @ direction) / det
        distance = np.einsum('rtk,tk->rt', q, e2) / det
        hit = (u >= 0) & (v >= 0) & (u + v <= 1) & (distance > 0)
        along = origins[:, g][:, None] + distance
        return float(along[hit].max() - along[hit].min())

    def pads(self, chain, home, fingers):
        """Fingertip pad boxes (last 15 mm of each inner finger) in link_6 at the current finger angle."""
        transforms, _ = chain.transforms({**home, self.joint: self.finger_angle})
        return [(points.min(axis=0), points.max(axis=0))
                for points in self.finger_vertices(transforms, fingers)], transforms

    @staticmethod
    def finger_vertices(transforms, fingers):
        result = []
        for link, offset, points in fingers:
            local = (np.linalg.inv(transforms['link_6']) @ transforms[link] @ offset @ points).T[:, :3]
            local = local[local[:, 2] >= local[:, 2].max() - 0.015]
            result.append(local)
        return result

    def grasp_error(self, positions):
        """Check both actual fingertip meshes against the flat recesses before fixing the grasp."""
        pocket = self.config['grip_pocket']
        axis = 'xy'.index(self.grip_axis)
        lateral = 1 - axis
        transforms, _ = self.chain.transforms(positions)
        matrix = np.linalg.inv(self.world_base @ self.park_body) @ transforms['link_6']
        for side, local in zip(('left', 'right'), self.finger_vertices(transforms, self.fingers)):
            points = (matrix @ np.c_[local, np.ones(len(local))].T).T[:, :3]
            distance = np.sign(points[:, axis].mean()) * points[:, axis]
            gap = distance.min() - self.width / 2
            entered = points[distance <= self.width / 2 + self.groove_depth]
            # Seated: the gap to the groove floor within SEAT_TOLERANCE of the planned clearance. pick_probe takes
            # the RG2 within 0.03 rad of the close angle (+-0.8 mm here) and the arm settles within ~0.5 mm; the
            # former -0.1 .. 0.7 mm window rejected good grasps at random. A finger outside the groove is still
            # off by its 5 mm depth.
            if not len(entered) or abs(gap - self.clearance) > self.SEAT_TOLERANCE:
                return f'{side} fingertip is not seated at the groove floor (gap {gap*1000:.2f} mm)'
            z0, z1 = np.asarray(pocket['z_mm']) / 1000
            lateral_error = np.abs(entered[:, lateral]).max() - pocket['span_mm'] / 2000
            vertical_error = max(z0 - entered[:, 2].min(), entered[:, 2].max() - z1)
            if max(lateral_error, vertical_error) > self.POCKET_EDGE_TOLERANCE:
                return (f'{side} fingertip extends outside the probe groove '
                        f'(lateral {max(lateral_error, 0.)*1000:.2f} mm, '
                        f'vertical {max(vertical_error, 0.)*1000:.2f} mm)')
        return ''

    def write_plan(self):
        from repair_car_pose import zyz
        grasp = self.park_flange @ self.initial_tcp  # pickup TCP at the grasp; approach and lift move along BASE z
        dock_tcp = self.park_body @ self.body_tcp
        self.plan = {
            'simulation_only': True, 'home_joints_deg': home_joints(),
            'tcp_name': self.tcp_name, 'tcp_mm_deg': [0., 0., self.tip_z*1000, 0., 0., 0.],
            'tool_name': 'ProbeToolWeight',
            'grasp_pose_mm_deg': [*(grasp[:3, 3]*1000), *zyz(grasp[:3, :3])],
            'dock_probe_tcp_mm_deg': [*(dock_tcp[:3, 3] * 1000), *zyz(dock_tcp[:3, :3])],
            'tip_pose_base_mm_deg': self.dock, 'grip_axis': self.grip_axis,
            'open_angle_rad': -0.558505, 'close_angle_rad': self.close_angle,
            'handle_width_mm': self.width*1000, 'pocket_height_mm': self.pocket_height*1000,
            'grip_from_top_mm': self.config['grip_from_top_mm'],
            'groove_depth_mm': self.groove_depth*1000,
            'finger_insertion_mm': (self.groove_depth-self.clearance)*1000,
            'grip_pocket': self.config['grip_pocket'],
            'finger_clearance_mm': self.clearance*1000, 'mass_kg': self.mass,
            'probe_radius_mm': self.tip_radius*1000, 'tip_mesh': self.mesh.name, 'holder_mesh': self.holder.name,
            'tip_ball': self.ball,
            'note': 'STL tip on its holder; fixed grasp constraint after pose/closure verification.',
        }
        if self.holder_pose is not None:  # the scanners' rack obstacle: the placed holder's extents about its frame
            self.plan.update(rack_pose_base_mm_deg=list(self.holder_pose), rack_box_mm=self.rack_box,
                             holder_seat_mm=self.seat_mm)
            if self.measured_cell:
                pickup = self.pickup
                # Transfer the taught BASE displacement to the simulator's geometry-aligned grasp TCP.
                offset = np.subtract(pickup['pickup_approach_base_mm'], pickup['pickup_grasp_base_mm_deg'][:3])
                self.plan.update(pickup_approach_mm_deg=[*(grasp[:3, 3] * 1000 + offset), *zyz(grasp[:3, :3])],
                                 pickup_lift_mm=pickup['pickup_lift_mm'], release_z_offset_mm=pickup['release_z_offset_mm'])
        path = self.cache/'probe_plan.json'
        path.write_text(json.dumps(self.plan, indent=2))

    @staticmethod
    def mesh_link(model, name, mesh, color):
        link = ET.SubElement(model, 'link', name=name)
        for tag in ['visual', 'collision']:
            element = ET.SubElement(link, tag, name=f'{name}_{tag}')
            geometry = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
            ET.SubElement(geometry, 'uri').text = mesh.resolve().as_uri()
            ET.SubElement(geometry, 'scale').text = '0.001 0.001 0.001'
            if tag == 'visual':
                material = ET.SubElement(element, 'material')
                for key in ['ambient', 'diffuse']: ET.SubElement(material, key).text = color
        return link

    @staticmethod
    def coupling(model, parent, child, prefix):
        plugin = ET.SubElement(model, 'plugin', filename='gz-sim-detachable-joint-system',
                               name='gz::sim::systems::DetachableJoint')
        for key, value in {'parent_link':parent, 'child_model':child, 'child_link':'body',
                           'detach_topic':prefix+'/detach', 'attach_topic':prefix+'/attach',
                           'output_topic':prefix+'/state'}.items(): ET.SubElement(plugin, key).text=value

    def contact_mesh(self):
        """Retain shaft and grip collisions; the exact sphere replaces the faceted ball."""
        triangles = read_stl(self.mesh)
        centre = np.asarray(self.tip_center) * 1000
        radius = self.tip_radius * 1000
        on_ball = np.abs(np.linalg.norm(triangles - centre, axis=2) - radius) < .0001
        path = self.cache / 'probe_contact_body.stl'
        write_stl(path, triangles[~on_ball.all(1)])
        return path

    def add_to_world(self, world, robot_model):
        from scene import rpy
        placed = self.world_base @ self.park_body
        pose = text([*placed[:3, 3], *rpy(placed[:3, :3])])
        rack = ET.SubElement(world, 'model', name='probe_rack')
        ET.SubElement(rack, 'static').text = 'true'
        stand = self.world_base @ self.holder_body
        ET.SubElement(rack, 'pose').text = text([*stand[:3, 3], *rpy(stand[:3, :3])])
        self.mesh_link(rack, 'stand', self.holder, '0.55 0.55 0.6 1')
        self.coupling(rack, 'stand', 'docked_probe', '/cobot/probe/dock')
        probe = ET.SubElement(world, 'model', name='docked_probe')
        ET.SubElement(probe, 'pose').text = pose
        link = self.mesh_link(probe, 'body', self.mesh, '0.15 0.35 0.7 1')
        link.find('collision/geometry/mesh/uri').text = self.contact_mesh().resolve().as_uri()
        if 'contact_friction' in self.config or self.ball:
            from scene import friction
            mu = self.config.get('contact_friction', self.ball['friction'] if self.ball else .3)
            friction(link.find('collision'), mu)
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'pose').text = text([*self.com, 0, 0, 0])
        ET.SubElement(inertial, 'mass').text = str(self.mass)
        inertia = ET.SubElement(inertial, 'inertia')
        for key in ['ixx', 'iyy', 'izz']: ET.SubElement(inertia, key).text = '0.00001'
        for key in ['ixy', 'ixz', 'iyz']: ET.SubElement(inertia, key).text = '0'
        # The exact sphere is the only contact geometry at the ball end.
        tip = ET.SubElement(link, 'collision', name='tip_sphere')
        ET.SubElement(tip, 'pose').text = text([*self.tip_center, 0, 0, 0])
        ET.SubElement(ET.SubElement(ET.SubElement(tip, 'geometry'), 'sphere'), 'radius').text = str(self.tip_radius)
        if self.ball:  # Gazebo takes the smaller friction of the two surfaces in contact
            ode = ET.SubElement(ET.SubElement(ET.SubElement(tip, 'surface'), 'friction'), 'ode')
            ET.SubElement(ode, 'mu').text = ET.SubElement(ode, 'mu2').text = str(self.ball['friction'])
            visual = ET.SubElement(link, 'visual', name='tip_ball')
            ET.SubElement(visual, 'pose').text = text([*self.tip_center, 0, 0, 0])
            ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'sphere'), 'radius').text = str(self.tip_radius)
            material = ET.SubElement(visual, 'material')
            for key in ['ambient', 'diffuse', 'specular']:
                ET.SubElement(material, key).text = text([*self.ball['color'], 1])
        self.coupling(robot_model, 'tool0', 'docked_probe', '/cobot/probe/grip')
        from collision_groups import fixed_probe_assembly
        fixed_probe_assembly(robot_model, rack, probe)

    def update(self, positions):
        if self.attached:
            transforms,_=self.chain.transforms(positions)
            self.current=np.linalg.inv(self.world_base)@transforms['link_6']@self.flange_body

    def markers(self):
        from visualization_msgs.msg import Marker
        from repair_car_pose import quaternion
        tip = self.current if self.attached and self.current is not None else self.park_body
        result = []
        for index, (namespace, mesh, matrix, color) in enumerate([
                ('probe_tool', self.mesh, tip, (.15, .35, .7)), ('probe_rack', self.holder, self.holder_body, (.55, .55, .6))]):
            marker = Marker(); marker.header.frame_id = 'base_link'; marker.ns = namespace; marker.id = index
            marker.type, marker.mesh_resource = Marker.MESH_RESOURCE, mesh.resolve().as_uri()
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, matrix[:3, 3])
            q = quaternion(matrix[:3, :3])
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q
            marker.scale.x = marker.scale.y = marker.scale.z = 0.001
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.
            result.append(marker)
        if self.ball:
            centre = tip[:3, :3] @ self.tip_center + tip[:3, 3]
            marker = Marker(); marker.header.frame_id = 'base_link'; marker.ns = 'probe_ball'; marker.id = 2
            marker.type = Marker.SPHERE
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, centre)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 2 * self.tip_radius
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *self.ball['color'], 1.
            result.append(marker)
        return result
