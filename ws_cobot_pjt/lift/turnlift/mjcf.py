"""MuJoCo (MJCF) model of the stand with the vehicle, the same bodies as sdf.py: the chassis on a free joint, four wheel
hinges driven by TT motor actuators with their torque-speed line (torque = stall x duty - stall / no_load x speed: the
TB6612 drive / short-brake average; mujoco_sim.py drops the speed term to coast), gearbox backdrive as joint friction,
the lift as a slide joint under a force-limited position servo (mujoco_sim.py sets the limit from the MG996R torque and
the scissor ratio at the current height) and the vehicle mesh on the lift top. World = robot BASE in m: table plane at
z = 0, the robot base column at the origin, a disc at each scan station. `stand()` adds the stand to any MJCF world,
e.g. later the robot scene of dsr_mujoco."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .device import G
from .mesh import write_obj
from .motors import GearMotor, Servo

LIFT_KP, LIFT_KV = 20000., 100.   # servo loop on the top plate: ~1 mm per 20 N (about 1 deg servo deadband), damped


def text(values):
    return ' '.join(f'{float(v):.6g}' for v in np.ravel(values))


def yaw_quat(deg):
    return text([np.cos(np.radians(deg) / 2), 0, 0, np.sin(np.radians(deg) / 2)])


class MjcfExport:
    def __init__(self, device):
        self.d, c = device, device.c
        self.motor, self.servo = GearMotor(c['motors']['wheel']), Servo(c['motors']['servo'])
        lift = device.lift
        down, up = c['lift']['servo_down_up_deg']
        self.down, self.stroke = down, (lift.top(up) - lift.top(down)) / 1000
        self.force = self.servo.stall * self.servo.efficiency / (max(lift.ratio(a) for a in lift.angles(141)) / 1000)

    @staticmethod
    def inertial(body, mass, centre, inertia):
        ET.SubElement(body, 'inertial', pos=text(centre), mass=f'{mass:.6g}',
                      fullinertia=text(inertia[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]))

    def stand(self, world, assets, actuators, mesh_file, centre_mm=None, heading=0.):
        """Body `turn_lift` (free) with the wheels, `lift_top` and the vehicle, at the BASE turning centre `centre_mm`
        (placement by default) and firmware `heading`; actuators `wheel_{front|rear}_{left|right}` (ctrl = duty
        -1..1, + rolls forward) and `lift` (ctrl = top plate rise m)."""
        d, c = self.d, self.d.c
        car, p = c['car'], c['placement']
        centre = np.asarray(p['centre_mm'] if centre_mm is None else centre_mm) / 1000
        body = ET.SubElement(world, 'body', name='turn_lift', pos=text([*centre, 0]),
                             quat=yaw_quat(p['heading0_yaw_deg'] - heading))
        ET.SubElement(body, 'freejoint', name='turn_lift')
        self.inertial(body, *d.fixed_body())
        boxes = {name: (low, high) for name, low, high in d.boxes(self.down)}
        for name in ('car', 'lift_base'):
            low, high = boxes[name]
            ET.SubElement(body, 'geom', name=name, type='box', pos=text((low + high) / 2000),
                          size=text((high - low) / 2000), rgba='.25 .25 .28 1')
        r, width, stall = car['wheel_radius_mm'] / 1000, car['wheel_width_mm'] / 1000, self.motor.stall
        for end, sx in (('front', 1), ('rear', -1)):
            for side, sy in (('left', 1), ('right', -1)):
                name = f'wheel_{end}_{side}'
                wheel = ET.SubElement(body, 'body', name=name, pos=text([sx * car['wheelbase_mm'] / 2000,
                                                                          sy * car['track_mm'] / 2000, r]))
                ET.SubElement(wheel, 'joint', name=name, type='hinge', axis='0 1 0', armature='1e-4',
                              frictionloss=f"{car['backdrive_nm']:.6g}")
                # isotropic friction: the lateral value, the one that resists the in-place turn
                ET.SubElement(wheel, 'geom', name=name, type='cylinder', size=text([r, width / 2]), zaxis='0 1 0',
                              mass='.03', friction=text([car['friction']['lateral'], .005, .0001]), rgba='.1 .1 .1 1')
                ET.SubElement(actuators, 'general', name=name, joint=name, ctrlrange='-1 1', gainprm=text([stall]),
                              biastype='affine', biasprm=text([0, 0, -stall / self.motor.no_load]))
        lc, a = d.lift.lc, c['adapter']['size_mm']
        top = ET.SubElement(body, 'body', name='lift_top',
                            pos=text([*np.asarray(c['lift']['mount_mm']) / 1000, d.plate_z(self.down) / 1000]))
        ET.SubElement(top, 'joint', name='lift', type='slide', axis='0 0 1', range=text([0, self.stroke]),
                      frictionloss='.5')
        self.inertial(top, *d.moving_body())
        ET.SubElement(top, 'geom', name='adapter', type='box', pos=text([0, 0, -a[2] / 2000]),
                      size=text(np.asarray(a) / 2000), rgba='.85 .85 .8 1')
        ET.SubElement(top, 'geom', name='lift_plate', type='box', pos=text([0, 0, -(a[2] + lc.TOP_T / 2) / 1000]),
                      size=text(np.array([lc.BL, lc.BW, lc.TOP_T]) / 2000), rgba='.7 .7 .75 1')
        ET.SubElement(assets, 'mesh', name='vehicle', file=str(mesh_file), scale='.001 .001 .001')
        ET.SubElement(top, 'geom', name='vehicle', type='mesh', mesh='vehicle', rgba='.75 .2 .2 1',
                      quat=yaw_quat(p['vehicle_yaw_on_car_deg']))
        ET.SubElement(actuators, 'position', name='lift', joint='lift', kp=f'{LIFT_KP:g}', kv=f'{LIFT_KV:g}',
                      ctrlrange=text([0, self.stroke]), forcerange=text([-self.force, self.force]))
        return body

    def write(self, folder):
        """folder/{turn_lift.xml, vehicle.obj}: the stand at its start (placement, heading 0, lift down)."""
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        write_obj(folder / 'vehicle.obj', self.d.vehicle.triangles)
        root = ET.Element('mujoco', model='turn_lift')
        ET.SubElement(root, 'option', timestep='.001', integrator='implicitfast', cone='elliptic', impratio='10',
                      gravity=text([0, 0, -G]))
        assets, world, actuators = (ET.SubElement(root, tag) for tag in ('asset', 'worldbody', 'actuator'))
        ET.SubElement(world, 'light', pos='.5 0 2', dir='0 0 -1', diffuse='.8 .8 .8')
        ET.SubElement(world, 'geom', name='table', type='plane', size='1.5 1.5 .1', rgba='.45 .48 .52 1',
                      friction='.1 .005 .0001')
        radius, height = np.asarray(self.d.c['robot']['base_column_mm']) / 1000
        ET.SubElement(world, 'geom', name='robot_base', type='cylinder', pos=text([0, 0, height / 2]),
                      size=text([radius, height / 2]), rgba='.6 .6 .65 .6')
        for name, spec in self.d.c['scan']['directions'].items():
            ET.SubElement(world, 'site', name=f'station_{name}', type='cylinder', size='.01 .0005',
                          pos=text([*np.asarray(spec['centre_mm']) / 1000, 0]), rgba='.2 .6 .9 1')
        self.stand(world, assets, actuators, 'vehicle.obj')
        ET.indent(root)
        path = folder / 'turn_lift.xml'
        path.write_text(ET.tostring(root, encoding='unicode'))
        return path
