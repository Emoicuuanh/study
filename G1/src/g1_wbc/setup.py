import os
from glob import glob
from setuptools import setup

package_name = 'g1_wbc'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoangdc',
    maintainer_email='hoangdc@todo.todo',
    description='WBC dong luc hoc giu thang bang cho Unitree G1 (che do bong)',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'wbc_shadow_node = g1_wbc.wbc_shadow_node:main',
        'lowstate_recorder_node = g1_wbc.lowstate_recorder_node:main',
    ]},
)
