"""Warehouse world + the AMR + the ROS <-> Gazebo bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory('amr_gazebo')
    xacro_file = os.path.join(
        get_package_share_directory('amr_description'), 'urdf', 'amr.urdf.xacro')
    world = os.path.join(gazebo_share, 'worlds', 'warehouse.sdf')
    bridge_config = os.path.join(gazebo_share, 'config', 'bridge.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 0. Tell Gazebo where our own models are (model://apriltag_0 ... in the world)
    models_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models'))

    # 1. Gazebo with the warehouse; -r = start playing immediately
    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
        launch_arguments={'gz_args': f'-r {world}'}.items())

    # 2. Publish the robot model and its TF tree (with simulation time)
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # 3. Put the robot into the world, slightly above the floor
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'amr',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05'])

    # 4. Bridge the topics listed in config/bridge.yaml
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}])

    return LaunchDescription([models_path, gazebo, robot_state_publisher, spawn, bridge])
