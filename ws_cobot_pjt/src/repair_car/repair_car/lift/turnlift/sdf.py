"""Gazebo (gz sim) model of the stand for a later simulation, standalone from the robot session: chassis on four
wheel joints (TT motor stall torque / no-load speed as effort / velocity limits, skid steer by the DiffDrive plugin),
the lift as a prismatic joint whose force limit is what the MG996R can push at the worst crank angle (the scissor's
varying transmission folded into one conservative limit), and the vehicle mesh on the adapter. Model frame = car
frame (x front, z up, on the table under the turning centre)."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .device import G
from .mesh import write_stl
from .motors import GearMotor, Servo


def text(values):
    return ' '.join(f'{float(v):.6g}' for v in np.ravel(values))


class SdfExport:
    def __init__(self, device):
        self.d, c = device, device.c
        self.servo, self.motor = Servo(c['motors']['servo']), GearMotor(c['motors']['wheel'])
        down, up = c['lift']['servo_down_up_deg']
        lift = device.lift
        self.down, self.stroke = down, (lift.top(up) - lift.top(down)) / 1000
        ratio = max(lift.ratio(a) for a in lift.angles(141)) / 1000                       # m per rad
        self.force = self.servo.usable * self.servo.stall * self.servo.efficiency / ratio
        self.speed = self.servo.speed * min(lift.ratio(a) for a in lift.angles(141)) / 1000

    @staticmethod
    def inertial(link, mass, centre, inertia):
        """`centre` m."""
        node = ET.SubElement(link, 'inertial')
        ET.SubElement(node, 'pose').text = text(np.r_[centre, 0, 0, 0])
        ET.SubElement(node, 'mass').text = f'{mass:.6g}'
        tensor = ET.SubElement(node, 'inertia')
        for (i, j), name in zip([(0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2)], ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']):
            ET.SubElement(tensor, name).text = f'{inertia[i, j]:.6g}'

    @staticmethod
    def shape(link, name, pose, geometry, friction=None, collide=True):
        for tag in ('visual', 'collision') if collide else ('visual',):
            node = ET.SubElement(link, tag, name=f'{name}_{tag}')
            ET.SubElement(node, 'pose').text = text(pose)
            node.append(geometry())
            if tag == 'collision' and friction:
                ode = ET.SubElement(ET.SubElement(ET.SubElement(node, 'surface'), 'friction'), 'ode')
                ET.SubElement(ode, 'mu').text, ET.SubElement(ode, 'mu2').text = str(friction[0]), str(friction[1])
                ET.SubElement(ode, 'fdir1').text = '1 0 0'

    @staticmethod
    def box(size):
        def make():
            geometry = ET.Element('geometry')
            ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = text(np.asarray(size) / 1000)
            return geometry
        return make

    def model(self, mesh_uri):
        d, c = self.d, self.d.c
        car, lc = c['car'], d.lift.lc
        model = ET.Element('model', name='turn_lift')
        chassis = ET.SubElement(model, 'link', name='chassis')
        self.inertial(chassis, *d.fixed_body())
        boxes = {name: (low, high) for name, low, high in d.boxes(self.down)}
        for name in ('car', 'lift_base'):
            low, high = boxes[name]
            self.shape(chassis, name, np.r_[(low + high) / 2000, 0, 0, 0], self.box(high - low))
        r, width = car['wheel_radius_mm'] / 1000, car['wheel_width_mm'] / 1000
        for side, sy in (('left', 1), ('right', -1)):
            for end, sx in (('front', 1), ('rear', -1)):
                name = f'wheel_{end}_{side}'
                wheel = ET.SubElement(model, 'link', name=name)
                ET.SubElement(wheel, 'pose').text = text([sx * car['wheelbase_mm'] / 2000, sy * car['track_mm'] / 2000, r,
                                                          -np.pi / 2, 0, 0])
                self.inertial(wheel, .03, [0, 0, 0], np.diag([.03 * (3 * r * r + width ** 2) / 12] * 2 + [.03 * r * r / 2]))

                def cylinder():
                    geometry = ET.Element('geometry')
                    node = ET.SubElement(geometry, 'cylinder')
                    ET.SubElement(node, 'radius').text, ET.SubElement(node, 'length').text = f'{r:.6g}', f'{width:.6g}'
                    return geometry
                f = car['friction']
                self.shape(wheel, name, [0] * 6, cylinder, (f['longitudinal'], f['lateral']))
                joint = ET.SubElement(model, 'joint', name=f'{name}_joint', type='revolute')
                ET.SubElement(joint, 'parent').text, ET.SubElement(joint, 'child').text = 'chassis', name
                axis = ET.SubElement(joint, 'axis')
                ET.SubElement(axis, 'xyz').text = '0 0 1'
                limit = ET.SubElement(axis, 'limit')
                ET.SubElement(limit, 'effort').text = f'{self.motor.stall:.6g}'      # 100 % duty stall, N m
                ET.SubElement(limit, 'velocity').text = f'{self.motor.no_load:.6g}'
        top = ET.SubElement(model, 'link', name='lift_top')
        z0 = d.plate_z(self.down)
        ET.SubElement(top, 'pose').text = text([*np.asarray(c['lift']['mount_mm']) / 1000, z0 / 1000, 0, 0, 0])
        self.inertial(top, *d.moving_body())
        yaw = np.radians(c['placement']['vehicle_yaw_on_car_deg'])
        a = c['adapter']['size_mm']
        self.shape(top, 'adapter', [0, 0, -a[2] / 2000, 0, 0, 0], self.box(a))
        self.shape(top, 'lift_top', [0, 0, -(a[2] + lc.TOP_T / 2) / 1000, 0, 0, 0], self.box([lc.BL, lc.BW, lc.TOP_T]))

        def mesh():
            geometry = ET.Element('geometry')
            node = ET.SubElement(geometry, 'mesh')
            ET.SubElement(node, 'uri').text, ET.SubElement(node, 'scale').text = mesh_uri, '0.001 0.001 0.001'
            return geometry
        self.shape(top, 'vehicle', [0, 0, 0, 0, 0, yaw], mesh)
        joint = ET.SubElement(model, 'joint', name='lift', type='prismatic')
        ET.SubElement(joint, 'parent').text, ET.SubElement(joint, 'child').text = 'chassis', 'lift_top'
        axis = ET.SubElement(joint, 'axis')
        ET.SubElement(axis, 'xyz').text = '0 0 1'
        limit = ET.SubElement(axis, 'limit')
        for key, value in (('lower', 0), ('upper', self.stroke), ('effort', self.force), ('velocity', self.speed)):
            ET.SubElement(limit, key).text = f'{value:.6g}'
        ET.SubElement(ET.SubElement(axis, 'dynamics'), 'friction').text = '0.5'
        plugins = [('gz-sim-diff-drive-system', 'gz::sim::systems::DiffDrive',
                    [('left_joint', 'wheel_front_left_joint'), ('left_joint', 'wheel_rear_left_joint'),
                     ('right_joint', 'wheel_front_right_joint'), ('right_joint', 'wheel_rear_right_joint'),
                     ('wheel_separation', f'{car["track_mm"] / 1000:.6g}'), ('wheel_radius', f'{r:.6g}'),
                     ('topic', '/turn_lift/cmd_vel'), ('odom_topic', '/turn_lift/odometry')]),
                   ('gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController',
                    [('joint_name', 'lift'), ('topic', '/turn_lift/lift'), ('p_gain', '4000'), ('d_gain', '50'),
                     ('i_gain', '0'), ('cmd_max', f'{self.force:.6g}'), ('cmd_min', f'{-self.force:.6g}')]),
                   ('gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher', [])]
        for filename, name, values in plugins:
            plugin = ET.SubElement(model, 'plugin', filename=filename, name=name)
            for key, value in values:
                ET.SubElement(plugin, key).text = value
        return model

    def write(self, directory):
        """out/models/turn_lift/{model.sdf, model.config, meshes/vehicle.stl} and out/worlds/turn_lift.sdf (the stand
        at its BASE placement, heading 0, lift down)."""
        directory = Path(directory)
        folder = directory / 'models/turn_lift'
        write_stl(folder / 'meshes/vehicle.stl', self.d.vehicle.triangles)
        sdf = ET.Element('sdf', version='1.9')
        sdf.append(self.model('meshes/vehicle.stl'))
        ET.indent(sdf)
        (folder / 'model.sdf').write_text(ET.tostring(sdf, encoding='unicode'))
        (folder / 'model.config').write_text(
            '<?xml version="1.0"?>\n<model><name>turn_lift</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>'
            '<description>Turning scissor-lift stand with the team vehicle (lift/turnlift)</description></model>\n')
        world = ET.Element('sdf', version='1.9')
        w = ET.SubElement(world, 'world', name='turn_lift')
        ET.SubElement(ET.SubElement(w, 'physics', name='2ms', type='dart'), 'max_step_size').text = '0.002'
        for filename, name in (('gz-sim-physics-system', 'gz::sim::systems::Physics'),
                               ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
                               ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
                               ('gz-sim-apply-link-wrench-system', 'gz::sim::systems::ApplyLinkWrench')):   # probe force
            ET.SubElement(w, 'plugin', filename=filename, name=name)
        ET.SubElement(w, 'gravity').text = f'0 0 {-G}'
        ground = ET.SubElement(w, 'model', name='table')
        ET.SubElement(ground, 'static').text = 'true'
        link = ET.SubElement(ground, 'link', name='top')
        for tag in ('visual', 'collision'):
            plane = ET.SubElement(ET.SubElement(ET.SubElement(link, tag, name=tag), 'geometry'), 'plane')
            ET.SubElement(plane, 'normal').text, ET.SubElement(plane, 'size').text = '0 0 1', '2 2'
        include = ET.SubElement(w, 'include')
        ET.SubElement(include, 'uri').text = (folder.resolve()).as_uri()
        p = self.d.c['placement']
        ET.SubElement(include, 'pose').text = text([p['centre_mm'][0] / 1000, p['centre_mm'][1] / 1000, 0, 0, 0,
                                                    np.radians(p['heading0_yaw_deg'])])
        ET.indent(world)
        (directory / 'worlds').mkdir(parents=True, exist_ok=True)
        (directory / 'worlds/turn_lift.sdf').write_text(ET.tostring(world, encoding='unicode'))
        return folder
