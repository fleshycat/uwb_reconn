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

    drone_process_list = []
    i=3
    
    drone_manager_node = Node(
        package='drone_manager',
        executable='drone_manager',
        name=f'drone_manager{ i + 1 }',
        parameters=[{'system_id': i + 1}],
        #output='screen'
    )
    drone_process_list.append(drone_manager_node)
    
    start_mission_node = Node(
        package='mission',
        executable='start_mission_uwb_straight',
        parameters=[{'system_id': i + 1}],
        output='screen',
    )
    drone_process_list.append(start_mission_node)

    nodes_to_start = [
        px4_lat,
        px4_lon,
        resource_path_env,
        px4_sim_env,
        model_path_env,
        plugin_path_env,
        *drone_process_list,
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
