import os
from glob import glob
from setuptools import setup

package_name = 'g1_leg_odometry'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoangdc',
    maintainer_email='hoangdc@todo.todo',
    description='Leg odometry cho Unitree G1 (thay the /dog_odom o che do low-level)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leg_odometry_node = g1_leg_odometry.leg_odometry_node:main',
            'leg_odom_check_node = g1_leg_odometry.leg_odom_check_node:main',
        ],
    },
)
