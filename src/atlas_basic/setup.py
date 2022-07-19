from setuptools import setup
from glob import glob

package_name    = 'atlas_basic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'],),
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atlas',
    maintainer_email='lorenzo@caltech.edu',
    description='Basic nodes to run Atlas: wheel control, encoder counting, command, etc.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_control  = atlas_basic.wheel_control.wheel_control:main',
            'odometry       = atlas_basic.odometry.odometry:main',
            'teleop         = atlas_basic.odometry.teleop:main',
        ],
    },
)
