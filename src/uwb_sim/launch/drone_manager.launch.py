import os
import random
import yaml
from jinja2 import Environment, FileSystemLoader
from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from decimal import Decimal
import xml.etree.ElementTree as ET
import numpy as np
from jinja2 import Template
import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
import xml.etree.ElementTree as ET

from launch.event_handlers import OnProcessStart, OnProcessExit

def launch_setup(context, *args, **kwargs):

    # Configuration
    current_package_path = get_package_share_directory('uwb_sim')
    uwb_plugin_package_path = get_package_share_directory('gazebo_uwb_sensor_plugins')
    px4_src_path = LaunchConfiguration('px4_src_path').perform(context)
    gazebo_classic_path = f'{px4_src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic'
    world_type = LaunchConfiguration('world_type').perform(context)
    px4_sim_env = SetEnvironmentVariable('PX4_SIM_MODEL', f'gazebo-classic_iris')
    px4_lat = SetEnvironmentVariable('PX4_HOME_LAT', f'36.6299')
    px4_lon = SetEnvironmentVariable('PX4_HOME_LON', f'127.4588')
    num_drone = int(LaunchConfiguration('num_drone').perform(context))

    # Environments
    resource_path_env = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', f'/usr/share/gazebo-11')
    model_path_env = SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                                            f'{current_package_path}:{current_package_path}/models:{gazebo_classic_path}/models')
    plugin_path_env = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                                            f'{uwb_plugin_package_path}:{px4_src_path}/build/px4_sitl_default/build_gazebo-classic/')

    # MicroXRCEAgent udp4 -p 8888
    xrce_agent_process = ExecuteProcess(
        cmd=[FindExecutable(name='MicroXRCEAgent'), 'udp4', '-p', '8888'],
        #output='screen',
    )

    # gazebo world
    env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'worlds')))
    jinja_world = env.get_template(f'{world_type}.world.jinja')
    tag_position = np.random.uniform(0, 15, 2)
    tag_position = np.append(tag_position, 2)# Random tag position
    simulation_world = jinja_world.render(tag_id = 1, tag_pose = tag_position) 
    # simulation_world = jinja_world.render(tag_id = 1, tag_pose = [0, 0]) 
    world_file_path = os.path.join('/tmp', 'output.world')
    with open(world_file_path, 'w') as f:
        f.write(simulation_world)

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path, 'verbose':'false', 'gui':'true' }.items()
    )

    # Run trilateration Node
    tag_pos_node = Node(
        package='uwb_localization',
        executable='sqrrange_leastsqr_localization',
        name='tag_pos',
        output='screen',
        parameters=[{'system_id_list': [i+1 for i in range(num_drone)]}],
    )
    
    drone_process_list = []
    for i in range(num_drone):
        jinja_cmd = [
            f'{gazebo_classic_path}/scripts/jinja_gen.py',
            f'{current_package_path}/models/iris_uwb/iris_uwb_with_anchor.sdf.jinja',
            f'{current_package_path}',
            '--mavlink_tcp_port', f'{4560+i}',
            '--mavlink_udp_port', f'{14560+i}',
            '--mavlink_id', f'{1+i}',
            '--gst_udp_port' , f'{5600+i}' ,
            '--video_uri', f'{5600+i}',
            '--mavlink_cam_udp_port', f'{14530+i}',
            '--output-file', f'/tmp/model_{i}.sdf'
        ]
        jinja_process = ExecuteProcess(
            cmd=jinja_cmd,
            #output='screen',
        )
        drone_process_list.append(jinja_process)

        # node to spawn robot model in gazebo
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', f'/tmp/model_{i}.sdf', '-entity', f'robot_{i}', '-x', f'{-15 + 10*i }', '-y', f'{-15}' ],
            # arguments=['-file', f'/tmp/model_{i}.sdf', '-entity', f'robot_{i}', '-x', f'{ 3 * (-1)**i }', '-y', f'{3 * (-1 if i%3==0 else 1)}' ],
            #output='screen',
            )
        drone_process_list.append(spawn_entity_node)
                
        # PX4
        # build_path/bin/px4 -i $N -d "$build_path/etc" >out.log 2>err.log &
        cmd = [
            'env', 
            'PX4_SIM_MODEL=gazebo-classic_iris',
            f'{px4_src_path}/build/px4_sitl_default/bin/px4',
            '-i', f'{i}',
            '-d',
            f'{px4_src_path}/build/px4_sitl_default/etc',
            '-w', f'{px4_src_path}/build/px4_sitl_default/ROMFS/instance{i}',
            '>out.log', '2>err.log',
        ]
        
        px4_process = ExecuteProcess(
            cmd=cmd,
            output='screen',
        )
        drone_process_list.append(px4_process)
        
        drone_manager_node = Node(
            package='drone_manager',
            executable='drone_manager',
            name=f'drone_manager{ i + 1 }',
            parameters=[{'system_id': i + 1}],
            output='screen'
        )
        drone_process_list.append(drone_manager_node)
        
        # start_mission_node = Node(
        #     package='mission',
        #     executable='start_mission_hover',
        #     parameters=[{'system_id': i + 1}],
        #     output='screen',
        # )
        # drone_process_list.append(start_mission_node)
    
    uwb_reconn = Node(
        package='mission',
        executable='uwb_reconnaissance',
        name='uwb_reconn',
        parameters=[{'system_id_list': [i+1 for i in range(num_drone)],
                     'formation_square_length': 4.0}],
        output='screen'
    )
    
    # rviz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("uwb_sim"), "confs", "default.rviz"]
    )
    
    # rviz node for visualizing robot model
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )
    
    nodes_to_start = [
        px4_lat,
        px4_lon,
        resource_path_env,
        px4_sim_env,
        model_path_env,
        plugin_path_env,
        xrce_agent_process,
        gazebo_node,
        *drone_process_list,
        # tag_pos_node,
    ]

    return nodes_to_start

def generate_launch_description():
    current_package_path = get_package_share_directory('uwb_sim')
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'px4_src_path',
            default_value='/home/user/PX4Swarm',
            description='px4 source code path'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_type',
            default_value='iris',
            description='Type of the robot to spawn'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'world_type',
            default_value='empty',
            description='Type of the world to spawn'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'num_drone',
            default_value='1',
            description='Number of drone to spawn'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
