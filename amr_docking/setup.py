from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_docking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed abdo',
    maintainer_email='61me38@gmail.com',
    description='Precision docking on floor AprilTags with the Nav2 docking server',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tag_dock_pose = amr_docking.tag_dock_pose:main',
        ],
    },
)