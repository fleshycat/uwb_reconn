import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    # MicroXRCEAgent udp4 -p 8888
    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'],
        #output='screen',
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
    
    nodes_to_start = [
        xrce_agent_process,
        drone_manager_node,
        linktrack_launch,
    ]

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
