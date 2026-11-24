"""Gazebo (gz sim 8, DART) model of the 90 x 60 scissor lift alone, from its CAD assembly (assembly.py): every link with
its mesh and mass, the pin joints as a kinematic tree, and the three joints that close the scissor's loops (base fixed
pin, bottom slot, top slot) as stub links welded to their link by DetachableJoint systems on the model itself (DART
builds joint trees only; its weld constraint is how a loop closes). No friction, no contacts between the parts (their
meshes would collide through the pin holes): the slot ends are the prismatic stub limits. The MG996R is a position
servo: a JointPositionController in velocity mode moves the crank toward its target at the firmware speed, and DART's
servo actuator delivers that only within the joint effort limit, the continuous allowance (motors.Servo: usable x
torque at the firmware speed), so a load beyond it slows or stalls the crank. The gearbox-reflected rotor inertia sits
on the crank. Payloads ride rigidly on the holder. LiftDrive sends a crank target over gz-transport and samples the
crank angle, speed and the top plate rise."""
from functools import cached_property
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from .assembly import CLOSURES, TREE, LiftAssembly
from .config import ROOT
from .device import G
from .mesh import box_inertia, read_stl, write_stl
from .motors import Servo
from .sdf import SdfExport, text

NAME = 'scissor_lift'
STUB = (1e-3, 1e-9)   # loop-closure stub link: kg, kg m2
COLOURS = dict(base=(.3, .45, .8, 1), crank=(.85, .85, .85, 1), pushrod=(.95, .95, .95, 1), slider_bottom=(.5, .5, .5, 1),
               slider_top=(.5, .5, .5, 1), arm_a=(.2, .2, .2, 1), arm_b=(.95, .5, .1, 1), top=(.6, .3, .7, 1),
               payload=(.75, .2, .2, 1))


def mesh(uri):
    def make():
        geometry = ET.Element('geometry')
        node = ET.SubElement(geometry, 'mesh')
        ET.SubElement(node, 'uri').text, ET.SubElement(node, 'scale').text = uri, '0.001 0.001 0.001'
        return geometry
    return make


def visual(link, name, pose, geometry, rgba):
    node = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(node, 'pose').text = text(pose)
    node.append(geometry())
    material = ET.SubElement(node, 'material')
    for key in ('ambient', 'diffuse'):
        ET.SubElement(material, key).text = text(rgba)


def plugin(parent, filename, name, values=()):
    node = ET.SubElement(parent, 'plugin', filename=filename, name=name)
    for key, value in values:
        ET.SubElement(node, key).text = str(value)


def save(path, element):
    ET.indent(element)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(ET.tostring(element, encoding='unicode'))


class LiftSdf:
    def __init__(self, device):
        c = device.c
        self.d, self.spec = device, c['linkage']
        self.a = LiftAssembly(self.spec, c['lift'], ROOT)
        servo = Servo(c['motors']['servo'])
        self.torque, self.speed = servo.usable * servo.torque(servo.speed), servo.speed   # N m allowance, rad/s

    @cached_property
    def meshes(self):
        """Mesh payloads (mm) in the payload frame at the holder top centre: the mesh (vehicle frame: x front, z up)
        scaled to length_mm along x, turned by yaw_deg and lowered onto the holder until it touches."""
        meshes = {}
        for name, spec in self.spec['payloads'].items():
            if 'mesh' in spec:
                path = ROOT / spec['mesh']
                t = np.load(path)['triangles'] if path.suffix == '.npz' else read_stl(path)
                t = t * spec['length_mm'] / np.ptp(t[..., 0])
                c, s = np.cos(np.radians(spec['yaw_deg'])), np.sin(np.radians(spec['yaw_deg']))
                t = t @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]]).T
                meshes[name] = t + [0, 0, self.a.rest(t)]
        return meshes

    def payload(self, name):
        """mass kg, centre m, inertia kg m2 and shape (box size mm, or the mesh name) in the payload frame on the
        holder top; a mesh payload's inertia is that of a solid box of its extent."""
        spec = self.spec['payloads'][name]
        if 'box_mm' in spec:
            size = np.asarray(spec['box_mm'])
            return spec['mass_kg'], [0, 0, size[2] / 2000], box_inertia(spec['mass_kg'], size), size
        points = self.meshes[name].reshape(-1, 3)
        low, high = points.min(0), points.max(0)
        return spec['mass_kg'], (low + high) / 2000, box_inertia(spec['mass_kg'], high - low), name

    @staticmethod
    def joint(model, name, kind, parent, child, limit=None):
        """Revolute about -Y (+ raises the lift at the crank), prismatic along X; `limit` (lower, upper[, effort])."""
        node = ET.SubElement(model, 'joint', name=name, type=kind)
        ET.SubElement(node, 'parent').text, ET.SubElement(node, 'child').text = parent, child
        if kind == 'fixed':
            return
        axis = ET.SubElement(node, 'axis')
        ET.SubElement(axis, 'xyz').text = '0 -1 0' if kind == 'revolute' else '1 0 0'
        if limit is not None:
            bounds = ET.SubElement(axis, 'limit')
            for key, value in zip(('lower', 'upper', 'effort'), limit):
                ET.SubElement(bounds, key).text = f'{value:.6g}'
        dynamics = ET.SubElement(axis, 'dynamics')
        for key in ('damping', 'friction'):
            ET.SubElement(dynamics, key).text = '0'

    def model(self, uri, payload=None):
        """`uri(name)`: mesh URI of a link or a mesh payload."""
        a = self.a
        model = ET.Element('model', name=NAME)
        for name, link in a.links.items():
            node = ET.SubElement(model, 'link', name=name)
            ET.SubElement(node, 'pose').text = text([*a.origins[name] / 1000, 0, 0, 0])
            rotor = self.spec['servo_rotor_kgm2'] * np.diag([.5, 1, .5]) if name == 'crank' else 0   # a disc on the axis
            SdfExport.inertial(node, link['mass'], link['centre'] / 1000, link['inertia'] * 1e-6 + rotor)
            visual(node, name, [0] * 6, mesh(uri(name)), COLOURS[name])
        self.joint(model, 'fix', 'fixed', 'world', 'base')
        for name, kind, parent, child, _ in TREE:
            self.joint(model, name, kind, parent, child, (0, a.lock, self.torque) if name == 'servo' else None)
        for name, kind, parent, child, axis in CLOSURES:
            stub = ET.SubElement(model, 'link', name=f'{name}_stub')
            ET.SubElement(stub, 'pose').text = text([*a.point(axis) / 1000, 0, 0, 0])
            SdfExport.inertial(stub, STUB[0], [0, 0, 0], np.eye(3) * STUB[1])
            self.joint(model, name, kind, parent, f'{name}_stub',
                       (a.slot - a.axes[axis][0]) / 1000 if kind == 'prismatic' else None)
            plugin(model, 'gz-sim-detachable-joint-system', 'gz::sim::systems::DetachableJoint',
                   [('parent_link', f'{name}_stub'), ('child_model', '__model__'), ('child_link', child)]
                   + [(f'{k}_topic', f'/{NAME}/{name}/{k}') for k in ('detach', 'attach', 'output')])
        if payload:
            mass, centre, inertia, shape = self.payload(payload)
            node = ET.SubElement(model, 'link', name='payload')
            ET.SubElement(node, 'pose').text = text([*(a.origins['top'] + a.holder) / 1000, 0, 0, 0])
            SdfExport.inertial(node, mass, centre, inertia)
            box = not isinstance(shape, str)
            visual(node, 'payload', [0, 0, shape[2] / 2000 if box else 0, 0, 0, 0],
                   SdfExport.box(shape) if box else mesh(uri(shape)), COLOURS['payload'])
            self.joint(model, 'payload_mount', 'fixed', 'top', 'payload')
        plugin(model, 'gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController',
               [('joint_name', 'servo'), ('topic', f'/{NAME}/servo'), ('use_velocity_commands', 'true'),
                ('cmd_max', f'{self.speed:.6g}'), ('cmd_min', f'{-self.speed:.6g}'), ('initial_position', 0)])
        plugin(model, 'gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher',
               [('topic', f'/{NAME}/joint_state')])
        return model

    def world(self, payload, uri):
        sdf = ET.Element('sdf', version='1.9')
        world = ET.SubElement(sdf, 'world', name=f'{NAME}_{payload}')
        physics = ET.SubElement(world, 'physics', name='1ms', type='dart')
        ET.SubElement(physics, 'max_step_size').text, ET.SubElement(physics, 'real_time_factor').text = '0.001', '1'
        for filename, name in (('gz-sim-physics-system', 'gz::sim::systems::Physics'),
                               ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
                               ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster')):
            plugin(world, filename, name)
        ET.SubElement(world, 'gravity').text = f'0 0 {-G}'
        light = ET.SubElement(world, 'light', type='directional', name='sun')
        for key, value in (('pose', '0 0 2 0 0 0'), ('diffuse', '.9 .9 .9 1'), ('direction', '-.4 .3 -1')):
            ET.SubElement(light, key).text = value
        table = ET.SubElement(world, 'model', name='table')
        ET.SubElement(table, 'static').text = 'true'
        link = ET.SubElement(table, 'link', name='top')
        plane = ET.SubElement(ET.SubElement(ET.SubElement(link, 'visual', name='visual'), 'geometry'), 'plane')
        ET.SubElement(plane, 'normal').text, ET.SubElement(plane, 'size').text = '0 0 1', '1 1'
        world.append(self.model(uri, payload))
        return sdf

    def write(self, directory):
        """out/models/scissor_lift/{model.sdf, model.config, meshes/} (no payload) and
        out/worlds/scissor_lift_<payload>.sdf (the model inline with the payload, absolute mesh URIs)."""
        directory = Path(directory)
        folder = directory / 'models' / NAME
        for name, link in self.a.links.items():
            write_stl(folder / f'meshes/{name}.stl', link['triangles'])
        for name, triangles in self.meshes.items():
            write_stl(folder / f'meshes/{name}.stl', triangles)
        sdf = ET.Element('sdf', version='1.9')
        sdf.append(self.model(lambda n: f'meshes/{n}.stl'))
        save(folder / 'model.sdf', sdf)
        (folder / 'model.config').write_text(
            f'<?xml version="1.0"?>\n<model><name>{NAME}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>'
            '<description>90 x 60 MG996R scissor lift, multibody from hardware/lift_assembly.stl (ROS package repair_car, turnlift)'
            '</description></model>\n')
        worlds = {p: directory / f'worlds/{NAME}_{p}.sdf' for p in self.spec['payloads']}
        for payload, path in worlds.items():
            save(path, self.world(payload, lambda n: (folder / f'meshes/{n}.stl').resolve().as_uri()))
        return dict(model=str(folder), worlds=[str(p) for p in worlds.values()], **self.summary())

    def summary(self):
        a = self.a
        kinematics = dict(crank_mm=a.radius, pushrod_mm=a.rod, arm_mm=a.arm, toggle_lock_deg=np.degrees(a.lock),
                          slot_stop_deg=np.degrees(a.stop), rise_mm=a.rise(a.slider_x(a.stop)),
                          holder_top_down_mm=a.origins['top'][2] + a.holder[2])
        payloads = []
        for name in self.spec['payloads']:
            mass = a.lifted() + self.payload(name)[0]
            torque, at = a.torque(mass)
            payloads.append(dict(payload=name, lifted_kg=mass, static_torque_nm=torque, at_deg=at,
                                 allowance_nm=self.torque, margin=self.torque / torque))
        return dict(kinematics=kinematics, links_kg={k: v['mass'] for k, v in a.links.items()}, payloads=payloads)


class LiftDrive:
    """Sends a crank target (the controller moves toward it at the firmware speed, within the torque allowance) and
    samples the crank angle, its speed and the top plate rise until the crank has stood still for `settle` s."""

    def __init__(self, lift):
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.model_pb2 import Model
        from gz.transport13 import Node
        self.a, self.speed, self.Double = lift.a, lift.speed, Double
        self.node, self.joints = Node(), {}
        self.publisher = self.node.advertise(f'/{NAME}/servo', Double)
        self.node.subscribe(Model, f'/{NAME}/joint_state', self.on_state)

    def on_state(self, msg):
        self.joints = {j.name: (j.axis1.position, j.axis1.velocity) for j in msg.joint}

    def sample(self):
        angle, speed = self.joints['servo']
        slider = self.a.axes['slider_bottom'][0] + self.joints['slot_bottom'][0] * 1000
        return dict(crank_deg=float(np.degrees(angle)), speed_deg_s=float(np.degrees(speed)),
                    rise_mm=float(self.a.rise(slider)))

    def run(self, target_deg, period=.05, settle=1.):
        print(f'Waiting for /{NAME}/joint_state (gz sim -r must be running)')
        while 'servo' not in self.joints:
            time.sleep(period)
        message = self.Double()
        message.data = float(np.radians(target_deg))
        rows, begin, still = [], time.monotonic(), 0.
        while still < settle:
            self.publisher.publish(message)
            time.sleep(period)
            rows.append(dict(t_s=time.monotonic() - begin, **self.sample()))
            still = still + period if len(rows) > 10 and abs(rows[-1]['speed_deg_s']) < .5 else 0.
            if len(rows) % 5 == 1:
                print('  ' + ', '.join(f'{k} {v:.3f}' for k, v in rows[-1].items()))
        moving = [abs(r['speed_deg_s']) for r in rows if abs(r['speed_deg_s']) >= .5]
        return dict(target_deg=target_deg, reached_deg=rows[-1]['crank_deg'], rise_mm=rows[-1]['rise_mm'],
                    mean_speed_deg_s=float(np.mean(moving)) if moving else 0., servo_speed_deg_s=np.degrees(self.speed),
                    slot_stop_deg=np.degrees(self.a.stop), rows=rows)
