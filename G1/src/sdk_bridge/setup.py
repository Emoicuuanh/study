import os
from glob import glob
from setuptools import setup

package_name = 'sdk_bridge'

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
    description='ROS 2 SDK Bridge package for Unitree G1 humanoid robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_sdk_node = sdk_bridge.cmd_vel_to_sdk_node:main',
            'keyboard_teleop_node = sdk_bridge.keyboard_teleop_node:main',
        ],
    },

)
