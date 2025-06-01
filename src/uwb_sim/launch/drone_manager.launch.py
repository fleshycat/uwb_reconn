import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    do_record = context.launch_configurations.get('record', 'false').lower() in ('true', '1')
    
    # System ID for the drone
    system_id = context.launch_configurations.get('system_id', '1')

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
        parameters=[{'system_id': int(system_id)}],
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
        ),
        launch_arguments={'system_id': system_id}.items()
    )
    
    nodes_to_start = [
        xrce_agent_process,
        drone_manager_node,
        linktrack_launch,
    ]

    if do_record:
        ranging_topic = f'/drone{system_id}/manager/out/ranging'
        monitoring_topic = f'/drone{system_id}/manager/out/monitoring'
        bag_record = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                ranging_topic,
                monitoring_topic,
            ],
            output='screen',
        )
        nodes_to_start.append(bag_record)

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'record',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'system_id',
            default_value='1',
            description='Drone System ID'
        ),
        OpaqueFunction(function=launch_setup)
    ])
