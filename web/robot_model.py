"""Export the running robot's URDF and mesh files into this web project."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET


class RobotModelAssets:
    def __init__(self, web_directory):
        self.output = Path(web_directory)/'static/robot'

    def export(self, urdf, source_root, tcp_frame='tool0'):
        source_root = Path(source_root)
        packages = {ET.parse(p).getroot().findtext('name'): p.parent.resolve()
                    for p in source_root.rglob('package.xml')}
        robot = ET.parse(urdf).getroot()
        for element in list(robot):
            if element.tag in ('ros2_control', 'gazebo', 'transmission'):
                robot.remove(element)
        self.output.mkdir(parents=True, exist_ok=True)
        for mesh in robot.iter('mesh'):
            value = mesh.get('filename')
            if value.startswith('package://'):
                package, relative = value[10:].split('/', 1)
                source = packages[package]/relative
            else:
                source = Path(unquote(urlparse(value).path)).resolve()
                package = next(name for name, directory in packages.items()
                               if source.is_relative_to(directory))
                relative = str(source.relative_to(packages[package]))
            destination = self.output/'packages'/package/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
            mesh.set('filename', f'packages/{package}/{relative}')
        for package, directory in packages.items():
            license_file = next((p for p in [directory/'LICENSE', directory.parent/'LICENSE'] if p.is_file()), None)
            if license_file and package in ('dsr_description2', 'onrobot_rg_description'):
                shutil.copy2(license_file, self.output/f'LICENSE.{package}.txt')
        ET.indent(robot, space='  ')
        xml = ET.tostring(robot, encoding='utf-8', xml_declaration=True)
        (self.output/'m0609_rg2.urdf').write_bytes(xml)
        links = [link.get('name') for link in robot.findall('link')]
        metadata = {'name': 'M0609 + OnRobot RG2', 'model_url': '/static/robot/m0609_rg2.urdf',
                    'fixed_frame': 'base_link', 'tcp_frame': tcp_frame if tcp_frame in links else 'tool0',
                    'version': hashlib.sha256(xml).hexdigest()[:12]}
        (self.output/'model.json').write_text(json.dumps(metadata, indent=2)+'\n')
        return metadata


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--tcp-frame', default='tool0', help='link shown as the TCP (tool0 if the URDF lacks it)')
    args = parser.parse_args()
    print(RobotModelAssets(Path(__file__).parent).export(args.urdf, args.source_root, args.tcp_frame))
