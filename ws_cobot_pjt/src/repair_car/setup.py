from setuptools import find_packages, setup
from glob import glob
from pathlib import Path

package_name = 'repair_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={
        'repair_car.lift': [
            str(path.relative_to('repair_car/lift'))
            for name in ('config', 'firmware', 'hardware')
            for path in Path('repair_car/lift', name).rglob('*')
            if path.is_file() and '__pycache__' not in path.parts
        ],
        'repair_car.web': [
            'schema.sql', 'requirements.txt',
            *(str(path.relative_to('repair_car/web'))
              for folder in ('templates', 'static')
              for path in Path('repair_car/web', folder).rglob('*') if path.is_file()),
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
        ('share/' + package_name + '/Docker',
         [str(path) for path in Path('Docker').iterdir() if path.is_file() and path.name != '.env']),
        ('share/' + package_name + '/Docker/emulator',
         [str(path) for path in Path('Docker/emulator').iterdir() if path.is_file()]),
        ('share/' + package_name + '/part_replace/drl', glob('repair_car/part_replace/drl/*.drl')),
    ] + [
        ('share/' + package_name + '/' + str(folder), [str(path) for path in folder.iterdir() if path.is_file()])
        for folder in [Path('meshes'), *Path('meshes').rglob('*')] if folder.is_dir()
    ] + [
        ('share/' + package_name + '/' + str(folder.relative_to('repair_car')),
         [str(path) for path in folder.iterdir() if path.is_file()])
        for name in ('config', 'firmware', 'hardware')
        for folder in [Path('repair_car/lift', name), *Path('repair_car/lift', name).rglob('*')]
        if folder.is_dir() and '__pycache__' not in folder.parts
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BL',
    maintainer_email='mail@mail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lift_bridge = repair_car.lift.node:main',
            'lift_command = repair_car.lift.command:main',
            'pick_probe = repair_car.probe.pick_probe:main',
            'release_probe = repair_car.probe.release_probe:main',
            'draw_graph = repair_car.report.draw_graph:main',
            'contact_overshoot = repair_car.experiments.contact_overshoot:main',
            'contact_scrape = repair_car.experiments.contact_scrape:main',
            'force_slide = repair_car.experiments.force_slide:main',
            'real_scan = repair_car.inspection.real_scan:main',
            'force_offset = repair_car.calibration.force_offset:main',
            'motion_offset = repair_car.calibration.motion_offset:main',
            'teleop = repair_car.robot_control.teleop:main',
            'motion_offset_check = repair_car.calibration.motion_offset_check:main',
            'calibration = repair_car.calibration.calibration:main',
            'choose_car = repair_car.inspection.choose_car:main',
            'scan_surface = repair_car.inspection.scan_surface:main',
            'pointcloud = repair_car.pointcloud.publisher:main',
            'pointcloud_damage = repair_car.report.pointcloud_damage:main',
            'check_db = repair_car.web.check_db:main',
            'replace_parts = repair_car.part_replace.replace_parts:main',
            'bonnet_replace = repair_car.part_replace.bonnet_replace:main',
            'door_replace = repair_car.part_replace.door_replace:main',
            'repair_manager = repair_car.web.repair_manager:main',
            'web_control = repair_car.web.web_control:main',
        ],
    },
)
