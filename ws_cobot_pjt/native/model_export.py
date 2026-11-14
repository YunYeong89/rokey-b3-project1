"""Export the existing ROS model with Mac-local mesh paths and Gazebo plugins."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
CACHE = ROOT / 'native/.cache'


class RobotModel:
    def __init__(self):
        CACHE.mkdir(exist_ok=True)
        command = ('source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && '
                   'xacro /workspace/install/m0609_rg2_bringup/share/m0609_rg2_bringup/'
                   'xacro/my_m0609_rg2.urdf.xacro mode:=virtual model:=m0609 update_rate:=100')
        xml = subprocess.check_output(
            ['docker', 'compose', '-f', str(ROOT/'compose.yaml'), 'exec', '-T', 'cobot', 'bash', '-lc', command], text=True)
        self.robot = ET.fromstring(xml)
        packages = {ET.parse(p).getroot().findtext('name'): p.parent
                    for p in (ROOT / 'ws_cobot_pjt/src').rglob('package.xml')}
        for element in self.robot.iter():
            for key, value in list(element.attrib.items()):
                if value.startswith('package://'):
                    package, relative = value[10:].split('/', 1)
                    local = packages[package] / relative
                    if not local.exists():
                        raise FileNotFoundError(local)
                    element.set(key, local.resolve().as_uri())
        for element in list(self.robot):
            if element.tag in ('ros2_control', 'transmission', 'gazebo'):
                self.robot.remove(element)
        self.joints = [j.get('name') for j in self.robot.findall('joint')
                       if j.get('type') in ('revolute', 'continuous', 'prismatic')]
        self.mimics = {j.get('name'): (m.get('joint'), float(m.get('multiplier', 1)),
                                     float(m.get('offset', 0)))
                       for j in self.robot.findall('joint')
                       if (m := j.find('mimic')) is not None}
        self.urdf = CACHE / 'robot.urdf'
        ET.ElementTree(self.robot).write(self.urdf, encoding='unicode')

    def gazebo_world(self):
        # Mimic joints are driven explicitly from their source joint in the relay.
        for joint in self.robot.findall('joint'):
            mimic = joint.find('mimic')
            if mimic is not None:
                joint.remove(mimic)
        gazebo_urdf = CACHE / 'gazebo.urdf'
        ET.ElementTree(self.robot).write(gazebo_urdf, encoding='unicode')
        sdf_text = subprocess.check_output(['gz', 'sdf', '-p', str(gazebo_urdf)], text=True)
        model = ET.fromstring(sdf_text).find('model')
        model.set('name', 'cobot')
        for name in self.joints:
            plugin = ET.SubElement(model, 'plugin', {
                'filename': 'gz-sim-joint-position-controller-system',
                'name': 'gz::sim::systems::JointPositionController'})
            arm = name in [f'joint_{index}' for index in range(1, 7)]
            gains = {'joint_1': (1500, 35), 'joint_2': (2000, 45),
                     'joint_3': (1500, 30), 'joint_4': (200, 3),
                     'joint_5': (150, 2), 'joint_6': (100, 1)}
            p_gain, d_gain = gains.get(name, (20, 0.1))
            limit = 163 if name in ('joint_1', 'joint_2') else 96 if name == 'joint_3' else 30 if arm else 10
            for key, value in {'joint_name': name, 'topic': f'/cobot/{name}/cmd_pos',
                               'use_velocity_commands': 'false', 'p_gain': str(p_gain),
                               'd_gain': str(d_gain), 'i_gain': '0',
                               'cmd_max': str(limit), 'cmd_min': str(-limit)}.items():
                ET.SubElement(plugin, key).text = value
        state = ET.SubElement(model, 'plugin', {
            'filename': 'gz-sim-joint-state-publisher-system',
            'name': 'gz::sim::systems::JointStatePublisher'})
        ET.SubElement(state, 'topic').text = '/cobot/gazebo_joint_states'
        ET.SubElement(state, 'update_rate').text = '200'
        sdf = ET.Element('sdf', version='1.9')
        world = ET.SubElement(sdf, 'world', name='cobot_world')
        physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
        ET.SubElement(physics, 'max_step_size').text = '0.002'
        ET.SubElement(physics, 'real_time_factor').text = '1'
        for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                               ('scene-broadcaster', 'SceneBroadcaster')]:
            ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system',
                          name=f'gz::sim::systems::{name}')
        # Position playback only: no gravity compensation or contact-force control.
        ET.SubElement(world, 'gravity').text = '0 0 0'
        light = ET.SubElement(world, 'light', type='directional', name='sun')
        ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
        ET.SubElement(light, 'diffuse').text = '0.9 0.9 0.9 1'
        ET.SubElement(light, 'direction').text = '-0.5 0.2 -1'
        world.append(model)
        path = CACHE / 'world.sdf'
        ET.ElementTree(sdf).write(path, encoding='unicode')
        return path
