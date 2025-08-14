#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the "use_compression" launch argument (default is "true").
    siyi_camera_package = get_package_share_directory('siyi_camera')

    use_compression_arg = DeclareLaunchArgument(
        'use_compression',
        default_value='true',
        description='Set "true" for compressed, "false" for raw transmission'
    )

    # Define the path to the YAML parameter file.
    config_file = os.path.join(siyi_camera_package, 'config', 'params.yaml')

    # Create the ImagePublisher node with YAML parameters,
    # overriding "use_compression" with the launch argument.
    siyi_camera_node = Node(
        package='siyi_camera',
        executable='ImagePublisher',
        name='stream_publish',
        output='screen',
        parameters=[
            config_file,
            { 'use_compression': LaunchConfiguration('use_compression') }
        ]
    )

    gimbal_control_node = Node(
        package='gimbal_control',
        executable='gimbal_controller',
        name='gimbal_control',
        output='screen',
        parameters=[
            {'system_id': 1},
            {'monitoring_topic': '/fmu/out/monitoring'},
            {'trajectory_topic': '/jfi/in/target'},
            {'gimbal_controller_set_topic': '/a8_mini/set_gimbal_attitude'},
            {'gimbal_controller_get_topic': '/a8_mini/get_gimbal_attitude'}
            ]
    )

    return LaunchDescription([
        use_compression_arg,
        siyi_camera_node,
        gimbal_control_node
    ])
