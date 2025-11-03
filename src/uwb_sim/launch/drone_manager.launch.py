import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import FindExecutable
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):
    # Drone manager launch configuration parameters
    system_id     = context.launch_configurations.get('system_id', '1')
    system_id_list= context.launch_configurations.get('system_id_list', '[1,2,3,4]')

    # J-Fi launch configuration parameters
    port_name     = context.launch_configurations.get('port_name', '/dev/ttyUSB0')
    baud_rate     = context.launch_configurations.get('baud_rate', '115200')
    component_id  = context.launch_configurations.get('component_id', '1')

    # MicroXRCEAgent udp4 -p 8888
    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'],
        output='screen',
    )
    
    # Drone manager
    drone_manager_node = Node(
        package='drone_manager',
        executable='drone_manager',
        name=f'drone_manager{system_id}',
        parameters=[
            {'system_id':      int(system_id)},
            {'system_id_list': eval(system_id_list)},   # "[1,2,3,4]" → [1,2,3,4]
        ],
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

    # J-Fi SerialCommNode
    serial_comm = Node(
        package='jfi_comm',
        executable='serial_comm_node',
        name=f'serial_comm_node{system_id}',
        output='screen',
        parameters=[
            {'port_name':       port_name},
            {'baud_rate':       int(baud_rate)},
            {'system_id':       int(system_id)},
            {'component_id':    int(component_id)},
            {'system_id_list':  eval(system_id_list)},
        ]
    )
    
    # ROS Domain Bridge Node
    pkg_share = get_package_share_directory('uwb_sim')

    domain_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='bridge_node',
        output='screen',
        # additional_env={'ROS_DOMAIN_ID': f'{system_id}'},
        arguments=[
            os.path.join(pkg_share, 'config', f'drone{system_id}_bridge_config.yaml')
        ]
    )

    nodes_to_start = [
        # xrce_agent_process,
        drone_manager_node,
        linktrack_launch,
        serial_comm,
        domain_bridge_node,
    ]

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('system_id',      default_value='1',              description='Drone System ID'),
        DeclareLaunchArgument('system_id_list', default_value='[1,2,3,4]',      description='All drone system IDs'),
        DeclareLaunchArgument('port_name',      default_value='/dev/ttyUSB0',   description='Serial port'),
        DeclareLaunchArgument('baud_rate',      default_value='115200',         description='Serial baudrate'),
        DeclareLaunchArgument('component_id',   default_value='1',              description='MAVLink component ID'),
        OpaqueFunction(function=launch_setup)
    ])
