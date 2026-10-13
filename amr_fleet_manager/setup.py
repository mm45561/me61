from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'amr_fleet_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohammed',
    maintainer_email='mohammed@todo.todo',
    description='Lightweight fleet manager for GTP AMRs.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'fleet_manager = amr_fleet_manager.fleet_manager_node:main',
            'mock_amr = amr_fleet_manager.mock_amr_node:main',
        ],
    },
)
