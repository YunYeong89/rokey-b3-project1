from glob import glob
from setuptools import find_packages, setup
setup(name='dsr_mujoco', version='0.1.0', packages=find_packages(),
      package_data={'dsr_mujoco': ['assets/*.zip']},
      data_files=[('share/ament_index/resource_index/packages', ['resource/dsr_mujoco']),
                  ('share/dsr_mujoco', ['package.xml']),
                  ('share/dsr_mujoco/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['dsr_mujoco_native = dsr_mujoco.bootstrap:main']})
