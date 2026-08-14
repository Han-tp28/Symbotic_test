"""Bring up the complete differential-drive robot backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('symbotic_description')
    control_share = get_package_share_directory('symbotic_control')

    simulation_launch = os.path.join(
        description_share,
        'launch',
        'simulation.launch.py',
    )
    default_config = os.path.join(
        description_share,
        'config',
        'robot_params.yaml',
    )
    default_world = os.path.join(
        description_share,
        'worlds',
        'empty_world.sdf',
    )
    control_config = os.path.join(control_share, 'config', 'teleop.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'world': LaunchConfiguration('world'),
            'world_name': LaunchConfiguration('world_name'),
            'model_name': LaunchConfiguration('model_name'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    command_arbiter = Node(
        package='symbotic_control',
        executable='command_arbiter',
        name='command_arbiter',
        output='screen',
        parameters=[control_config],
    )

    go_to_goal_server = Node(
        package='symbotic_control',
        executable='go_to_goal_server',
        name='go_to_goal_server',
        output='screen',
        parameters=[control_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Robot physical-parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world SDF file.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='symbotic_world',
            description='World name declared inside the SDF file.',
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='symbotic_diff_drive',
            description='Robot entity name in Gazebo.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run only the Gazebo server without the GUI.',
        ),
        simulation,
        command_arbiter,
        go_to_goal_server,
    ])
