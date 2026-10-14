"""Start everything: Gazebo warehouse + AMR, then RViz, then Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('amr_gazebo')
    nav_share = get_package_share_directory('amr_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    # 1. Render Gazebo (and RViz) on the NVIDIA card, like the prefix we typed by hand
    nvidia = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
    ]

    # 2. Gazebo with the warehouse, the robot and the bridge
    simulation = IncludeLaunchDescription(
        os.path.join(gazebo_share, 'launch', 'sim.launch.py'))

    # 3. RViz with the Nav2 layout, 5 s later so the robot already exists
    rviz = TimerAction(period=5.0, actions=[
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])

    # 4. Nav2 with our map and parameters, 10 s later so /clock, /odom and /scan exist
    navigation = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py'),
            launch_arguments={
                'use_sim_time': 'True',
                'map': os.path.join(gazebo_share, 'maps', 'warehouse.yaml'),
                'params_file': os.path.join(nav_share, 'config', 'nav2_params.yaml'),
            }.items()),
    ])

    return LaunchDescription([*nvidia, simulation, rviz, navigation])