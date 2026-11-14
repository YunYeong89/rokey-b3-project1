from setuptools import find_packages, setup

setup(
    name='rokey', version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rokey']),
        ('share/rokey', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ROKEY B3', maintainer_email='rokey@example.com',
    description='M0609 simple joint and Cartesian motion example', license='MIT',
    entry_points={'console_scripts': ['simple_move = rokey.move:main']},
)
