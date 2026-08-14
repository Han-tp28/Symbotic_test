"""Launch the differential-drive robot in Gazebo Harmonic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _launch_setup(context):
    package_share = get_package_share_directory('symbotic_description')
    control_share = get_package_share_directory('symbotic_control')
    config_file = LaunchConfiguration('config_file').perform(context)
    with open(config_file, encoding='utf-8') as stream:
        robot_config = yaml.safe_load(stream)
    controller_config = robot_config['robot']['controller']
    world_file = LaunchConfiguration('world').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    model_name = LaunchConfiguration('model_name').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('1', 'true', 'yes')

    xacro_file = os.path.join(package_share, 'urdf', 'differential_drive.urdf.xacro')
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' config_file:=',
            config_file,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    gazebo_args = ['-r']
    if headless:
        gazebo_args.append('-s')
    gazebo_args.append(world_file)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': ' '.join(gazebo_args)}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_symbotic_robot',
        output='screen',
        parameters=[{
            'world': world_name,
            'name': model_name,
            'topic': '/robot_description',
            'allow_renaming': False,
            'x': 0.0,
            'y': 0.0,
            'z': 0.01,
            'Y': 0.0,
        }],
    )

    joint_state_topic = f'/world/{world_name}/model/{model_name}/joint_state'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'{joint_state_topic}@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[(joint_state_topic, '/joint_states')],
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./tf.publisher.durability': 'volatile',
            'qos_overrides./tf.publisher.reliability': 'reliable',
        }],
    )

    velocity_limiter = Node(
        package='symbotic_control',
        executable='velocity_limiter',
        name='velocity_limiter',
        output='screen',
        parameters=[
            os.path.join(control_share, 'config', 'teleop.yaml'),
            controller_config,
        ],
    )

    return [gazebo, robot_state_publisher, spawn_robot, velocity_limiter, bridge]


def generate_launch_description():
    package_share = get_package_share_directory('symbotic_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(package_share, 'config', 'robot_params.yaml'),
            description='Robot physical-parameter YAML file.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(package_share, 'worlds', 'empty_world.sdf'),
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
        OpaqueFunction(function=_launch_setup),
    ])
