"""Apply saved controller profiles to the native simulation model."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from kinematics import KinematicChain, origin, rotation

DEFAULT_CALIBRATION = Path(__file__).resolve().parents[1] / 'config/calibration.json'


class SimulationCalibration:
    def __init__(self, path=DEFAULT_CALIBRATION):
        self.path = Path(path).resolve()
        self.data = json.loads(self.path.read_text(encoding='utf-8'))
        if self.data['schema_version'] != 1:
            raise ValueError('Unsupported calibration schema')
        self.tcp = self.data['tcps'][self.data['active']['tcp']]['pos']
        self.tool = self.data['tools'][self.data['active']['tool']]
        self.tcp_link = 'calibrated_tcp'

    def apply(self, robot):
        self.apply_payload(robot)
        a, b, c = np.deg2rad(self.tcp[3:])
        matrix = (rotation(np.array([0, 0, a])) @ rotation(np.array([0, b, 0]))
                  @ rotation(np.array([0, 0, c])))
        # URDF uses fixed-axis RPY; controller profiles use Euler ZYZ.
        pitch = np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0]))
        if abs(np.cos(pitch)) > 1e-8:
            roll = np.arctan2(matrix[2, 1], matrix[2, 2])
            yaw = np.arctan2(matrix[1, 0], matrix[0, 0])
        else:
            roll, yaw = 0., np.arctan2(-matrix[0, 1], matrix[1, 1])
        ET.SubElement(robot, 'link', name=self.tcp_link)
        joint = ET.SubElement(robot, 'joint', name='calibrated_tcp_joint', type='fixed')
        # DRCF TCP profiles use the flange (link_6). This URDF's tool0 has an
        # additional fixed rotation for the RG2 model and is not that frame.
        ET.SubElement(joint, 'parent', link='link_6')
        ET.SubElement(joint, 'child', link=self.tcp_link)
        ET.SubElement(joint, 'origin',
                      xyz=' '.join(str(float(value) / 1000) for value in self.tcp[:3]),
                      rpy=f'{roll} {pitch} {yaw}')
        print(f'Virtual calibration: {self.path}; TCP={self.data["active"]["tcp"]}, '
              f'Tool={self.data["active"]["tool"]}', flush=True)

    def apply_payload(self, robot):
        # Match total RG2 mass/COG at zero gripper joint positions.
        # Preserve CAD inertia shapes, scaled by mass; zero controller inertia is unspecified.
        chain = KinematicChain(robot)
        transforms, _ = chain.transforms({})
        flange = transforms['link_6']
        total_mass = sum(mass for _, mass, _ in chain.payload)
        weight = float(self.tool['weight'])
        if total_mass <= 0 or weight <= 0:
            raise ValueError('Native Gazebo payload requires a positive CAD and calibrated mass')
        current_cog = sum(
            mass * (np.linalg.inv(flange) @ transforms[name] @ np.r_[cog, 1])[:3]
            for name, mass, cog in chain.payload) / total_mass
        delta = np.asarray(self.tool['cog'], dtype=float) / 1000 - current_cog
        scale = weight / total_mass
        for name, mass, _ in chain.payload:
            inertial = robot.find(f"link[@name='{name}']/inertial")
            inertial.find('mass').set('value', str(mass * scale))
            for key, value in list(inertial.find('inertia').attrib.items()):
                inertial.find('inertia').set(key, str(float(value) * scale))
            element = inertial.find('origin')
            position = origin(element)[:3, 3] + transforms[name][:3, :3].T @ flange[:3, :3] @ delta
            if element is None:
                element = ET.SubElement(inertial, 'origin')
            element.set('xyz', ' '.join(str(value) for value in position))
