from glob import glob

from setuptools import find_packages, setup

package_name = 'hand_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BL',
    maintainer_email='mail@mail.com',
    description='Webcam hand-skeleton teleoperation of the M0609 TCP and the RG2 opening',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop = hand_teleop.robot.main:main',
        ],
    },
)
