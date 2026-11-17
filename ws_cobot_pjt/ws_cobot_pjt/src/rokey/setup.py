from setuptools import find_packages, setup
from glob import glob

package_name = 'rokey'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['calibration.json']),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
        ('share/' + package_name + '/meshes/probes', glob('meshes/probes/*')),
        ('share/' + package_name + '/meshes/vehicles/rgsdev', glob('meshes/vehicles/rgsdev/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BL',
    maintainer_email='mail@mail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_probe = rokey.pick_probe:main',
            'pid_surface = rokey.pid_controller:main',
            'draw_graph = rokey.draw_graph:main',
            'contact_overshoot = rokey.contact_overshoot:main',
            'contact_scrape = rokey.contact_scrape:main',
            'force_slide = rokey.force_slide:main',
            'defect_scan = rokey.defect_scan:main',
            'real_scan = rokey.real_scan:main',
            'force_collect = rokey.force_learning:collect_main',
            'force_learning_train = rokey.force_learning:train_main',
            'force_learning_calibrate = rokey.force_learning:calibrate_main',
            'calibration = rokey.calibration:main',
            'choose_car = rokey.choose_car:main',
            'scan_surface = rokey.scan_surface:main',
            'pointcloud = rokey.pointcloud:main',
            'pointcloud_damage = rokey.pointcloud_damage:main',
            'check_db = rokey.check_db:main',
            'replace_parts = rokey.replace_parts:main',
            'repair_manager = rokey.repair_manager:main',
            'web_control = rokey.web_control:main',
        ],
    },
)
