from setuptools import setup
from glob import glob
import os

package_name = 'control_hand'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name,
            ['package.xml']),

        # ===== LAUNCH FILES =====
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoangdc',
    maintainer_email='hoang.dinhcong@meiko.vn',
    description='Control Inspire Hand via ROS2 Node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_node = control_hand.hand_node:main',
            'init_driver = control_hand.init_driver:main',
            'diagnose_hands = control_hand.diagnose_hands:main',
        ],
    },
)
