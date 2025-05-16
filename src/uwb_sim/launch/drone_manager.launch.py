import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    do_record = context.launch_configurations.get('record', 'false').lower() in ('true', '1')

    # MicroXRCEAgent udp4 -p 8888
    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'],
        output='screen',
    )
    
    # Drone manager
    drone_manager_node = Node(
        package='drone_manager',
        executable='drone_manager',
        name='drone_manager',
        parameters=[{'system_id': 1}],
        output='screen'
    )

    # launch nlink_parser_ros2
    linktrack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nlink_parser_ros2'),
                'launch',
                'linktrack.launch.py'
            )
        )
    )
    
    # ros2 bag record
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/drone1/manager/out/ranging',
            '/drone1/fmu/out/monitoring',
        ],
        output='screen',
    )
    
    nodes_to_start = [
        xrce_agent_process,
        drone_manager_node,
        linktrack_launch,
    ]

    if do_record:
        nodes_to_start.append(bag_record)

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'record',
            default_value='false',
        ),
        OpaqueFunction(function=launch_setup)
    ])
