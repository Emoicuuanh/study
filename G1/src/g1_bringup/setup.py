from setuptools import setup
from glob import glob
import os

package_name = 'g1_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Ament resource index registration
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name,
            ['package.xml']),
        # Launch files installation
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files installation
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoangdc',
    maintainer_email='hoang.dinhcong@meiko.vn',
    description='ROS 2 G1 Bringup Package containing LowState to JointState bridge and TF launch files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lowstate_jointstate_bridge = g1_bringup.lowstate_jointstate_bridge:main',
            'timestamp_corrector = g1_bringup.timestamp_corrector:main',
        ],
    },
)
