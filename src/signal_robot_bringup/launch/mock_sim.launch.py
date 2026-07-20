"""Bring up the whole mock pipeline in one command.

This is the vertical slice: a fake perception model, the real controller, the
ROS/Gazebo bridge, the warehouse world, and the robot. It exists to prove the
pieces fit together before the real gesture-recognition model is available.

    ros2 launch signal_robot_bringup mock_sim.launch.py

Once the real model lands, drop `use_mock:=false` and point it at the actual
publisher — nothing else in this file has to change.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory('signal_robot_bringup')
    warehouse_share = get_package_share_directory('aws_robomaker_small_warehouse_world')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(
        warehouse_share, 'worlds', 'no_roof_small_warehouse',
        'no_roof_small_warehouse.world',
    )
    bridge_config = os.path.join(bringup_share, 'config', 'bridge.yaml')

    use_mock = LaunchConfiguration('use_mock')

    # Gazebo resolves model:// URIs by looking for a directory of that name
    # directly under one of these paths — so these point at the parent of the
    # model directories, not at the packages themselves. Appending rather than
    # overwriting matters: ROS 2's setup.bash has already put its own share
    # directory in here.
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join(filter(None, [
            os.path.join(bringup_share, 'models'),
            os.path.join(warehouse_share, 'models'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # -r starts the sim running instead of paused; -v 4 makes Gazebo say
            # *why* it could not find a resource, which is most of the debugging.
            'gz_args': ['-r -v 4 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # The robot is spawned into the world rather than written into it, so the
    # vendored AWS world file stays untouched and upgradable.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_signal_robot',
        output='screen',
        arguments=[
            '-world', 'default',          # the AWS world calls itself "default"
            '-file', os.path.join(bringup_share, 'models', 'signal_robot', 'model.sdf'),
            '-name', 'signal_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.15',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    # The real logic of this repository.
    controller = Node(
        package='signal_robot_control',
        executable='command_to_twist',
        name='command_to_twist',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Stands in for the separate perception repository. Turn it off with
    # use_mock:=false when the real thing is connected.
    mock_publisher = Node(
        package='signal_robot_control',
        executable='mock_signal_publisher',
        name='mock_signal_publisher',
        output='screen',
        condition=IfCondition(use_mock),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock', default_value='true',
            description='Publish fake signal commands, standing in for the perception repo.'),
        resource_path,
        gazebo,
        spawn_robot,
        bridge,
        controller,
        mock_publisher,
    ])
