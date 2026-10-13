"""Start the fleet manager and simulated AMRs parked in the ready queue (no Gazebo)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# One entry per simulated robot: (namespace, initial pose [x, y, yaw]).
# The poses are the queue slots of config/site.yaml.
ROBOTS = [
    ('amr_1', [-2.0, 1.5, 0.0]),
    ('amr_2', [-2.0, 0.5, 0.0]),
    ('amr_3', [-2.0, -0.5, 0.0]),
]


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('amr_fleet_manager'), 'config')

    fleet_manager = Node(
        package='amr_fleet_manager',
        executable='fleet_manager',
        namespace='fleet',
        name='fleet_manager',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'fleet_manager.yaml'),
            {   # later entries override the YAML file
                'site_config_file': os.path.join(config_dir, 'site.yaml'),
                'robot_ids': [name for name, _ in ROBOTS],
            },
        ],
    )

    mock_robots = [
        Node(
            package='amr_fleet_manager',
            executable='mock_amr',
            namespace=name,
            name='mock_amr',
            output='screen',
            parameters=[{'robot_id': name, 'initial_pose': pose}],
        )
        for name, pose in ROBOTS
    ]

    return LaunchDescription([fleet_manager, *mock_robots])
